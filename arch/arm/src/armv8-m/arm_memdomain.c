/****************************************************************************
 * arch/arm/src/armv8-m/arm_memdomain.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * [ORT] Per-container MPU memory domain support (PROTOTYPE, PMSAv8)
 *
 * ★ 本文件只保留**与 MPU 模型相关**的部分：
 *     域编码、region 分配与编程、切换钩子。
 *
 *   与 MPU 无关的内核侧机制（监督者槽位、故障事件队列）已抽到
 *   arch/arm/src/common/arm_ortcommon.c —— 三个架构共用一份。
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_ORT_MEMDOMAIN

#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <nuttx/debug.h>
#include <syslog.h>

#include <sys/prctl.h>

#include <nuttx/sched.h>
#include <nuttx/arch.h>
#include <nuttx/signal.h>

#include "mpu.h"
#include "arm_memdomain.h"
#include "arm_ortcommon.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* 域块属性：仅数据（XN），特权/用户均可 RW，普通可缓存内存 */

#define ORT_BLOCK_FLAGS1  (MPU_RBAR_XN | MPU_RBAR_AP_RWRW)
#define ORT_BLOCK_FLAGS2  (MPU_RLAR_WRITE_BACK | MPU_RLAR_PXN)

/* 「不可访问」属性：用户态无权限。
 *
 * ★ 这是 PMSAv8 方案的关键之一：未绑定的容器不是靠"池 region 挡住"，
 *   而是靠**没有任何 region 覆盖** —— 非特权访问未覆盖地址一律 fault。
 *   这个 region 只是为了把 own region 占住并指向一个无害的小块。
 */

#define ORT_DENY_FLAGS1   (MPU_RBAR_XN | MPU_RBAR_AP_RWNO)
#define ORT_DENY_FLAGS2   (MPU_RLAR_WRITE_BACK | MPU_RLAR_PXN)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int g_own_region = -1;    /* 域 region 编号（惰性分配） */

/* region 分配失败的表示（零初始化 = 正常，方向安全）。详见 lazyinit。 */

static bool g_memdomain_broken;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void ort_memdomain_lazyinit(void)
{
  unsigned int own = mpu_allocregion();

  /* ★ 预算耗尽的判据 —— 与 armv7-m 版本同一条债（手册 §三·补五十四）；
   *   那边有更完整的推导。要点：mpu_allocregion() 在 region 用尽时
   *   返回**越界编号**而不是报错，越界编号写进 MPU_RNR 被硬件忽略 ——
   *   region 静默不生效。这里把它变成显式的 broken：
   *   响亮报错 + 绑域 -ENOSPC（fail-closed）。 */

  if (own >= CONFIG_ARM_MPU_NREGIONS)
    {
      _err("[ORT] memdomain(armv8-m) init FAILED: MPU region exhausted "
           "(own=%u, NREGIONS=%d) — ORT domains disabled, "
           "container bind will fail with -ENOSPC\n",
           own, CONFIG_ARM_MPU_NREGIONS);
      g_memdomain_broken = true;
      return;
    }

  g_own_region = (int)own;

  mpu_modify_region((unsigned int)g_own_region,
                    ORT_DOMAIN_POOL_BASE, ORT_DOMAIN_DENY_SIZE,
                    ORT_DENY_FLAGS1, ORT_DENY_FLAGS2);

  syslog(LOG_INFO, "[ORT] memdomain(armv8-m) init: own_region=%d\n",
         g_own_region);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ort_caps
 *
 * Description:
 *   申报本平台**实际强制执行**的 ORT 能力位。
 *
 *   与 armv7-m 同：ARMv8-M 这边也是 MPC + CONFIG_BUILD_PROTECTED，
 *   用户栈是内核堆里的一块内存，终止路径释放它时页表不动 ——
 *   所以容器注册的故障处理器可以被安全地调用。
 *   详细论证见 arch/arm/src/armv7-m/arm_memdomain.c 里的 ort_caps()。
 *
 *   ⚠️ 验证状态：mps3-an547 上的 handler / ignore / handler-ret
 *      只在 H31 之前跑过（手册 §"验证结果（mps3-an547）"），
 *      **尚未**按 §三·补十三 的判据复验。机制与 armv7-m 相同、
 *      决定因素是 PROTECTED 而非 M7/M55，所以这里先按同值申报 ——
 *      但这条是**推理**不是实测，复验时一并确认。
 *
 ****************************************************************************/

uint32_t ort_caps(void)
{
  return ORT_CAP_FAULT_HANDLER;
}

/* 容量 = 池块数（手册 §三·补五十四）—— 与 armv7-m 同义。 */

int ort_domain_capacity(void)
{
  return ORT_DOMAIN_COUNT;
}

int ort_container_domain(FAR struct task_group_s *group)
{
  if (group == NULL || !ORT_DOMAIN_VALID(group->tg_ort_domain))
    {
      return -1;
    }

  return ORT_DOMAIN_DECODE(group->tg_ort_domain);
}

int ort_container_bind(pid_t pid, int domain)
{
  FAR struct tcb_s *tcb;
  FAR struct task_group_s *group;
  irqstate_t flags;

  /* 只有监督者能绑域 —— 域号就是内存块号，容器能自选就能选到别人的块 */

  if (nxsched_self()->pid != ort_supervisor_pid())
    {
      return -EPERM;
    }

  /* 域机制初始化失败 → fail-closed（同 armv7-m，见那里的推导） */

  if (g_memdomain_broken)
    {
      return -ENOSPC;
    }

  flags = enter_critical_section();

  tcb = nxsched_get_tcb(pid);
  if (tcb == NULL || tcb->group == NULL)
    {
      leave_critical_section(flags);
      return -ESRCH;
    }

  group = tcb->group;

  /* 预算之外的域号 → -EINVAL，不改动既有绑定 ——
   * 与 armv7-m / armv7-a 语义统一（手册 §三·补五十四；
   * 旧的"越界→静默解除绑定+OK"已取消，理由见 armv7-m 版本）。 */

  if (domain < 0 || domain >= ort_domain_budget())
    {
      leave_critical_section(flags);
      return -EINVAL;
    }

  group->tg_ort_domain = ORT_DOMAIN_ENCODE(domain);

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: ort_memdomain_switch
 *
 * Description:
 *   上下文切换钩子：为 incoming 容器编程域 region。
 *
 * ★ PMSAv8 版本的全部要点：
 *   只维护**一个** region，指向 incoming 容器的块。
 *   其余地址不被任何 region 覆盖 —— 非特权访问一律 fault。
 *
 *   没有"池 region"，因为 PMSAv8 不允许 region 重叠。
 *
 ****************************************************************************/

void ort_memdomain_switch(FAR struct tcb_s *to)
{
  FAR struct task_group_s *group;
  uintptr_t base;
  int domain;
  int bound;

  /* 初始化失败（region 预算耗尽）→ 不编程、不重试（同 armv7-m）。 */

  if (g_memdomain_broken)
    {
      return;
    }

  if (g_own_region < 0)
    {
      ort_memdomain_lazyinit();
    }

  group = to != NULL ? to->group : NULL;
  bound = group != NULL ? group->tg_ort_domain : ORT_DOMAIN_UNBOUND;

  if (!ORT_DOMAIN_VALID(bound))
    {
      mpu_modify_region((unsigned int)g_own_region,
                        ORT_DOMAIN_POOL_BASE, ORT_DOMAIN_DENY_SIZE,
                        ORT_DENY_FLAGS1, ORT_DENY_FLAGS2);
      return;
    }

  domain = ORT_DOMAIN_DECODE(bound);
  base   = ORT_DOMAIN_POOL_BASE + (uintptr_t)domain * ORT_DOMAIN_BLOCK_SIZE;

  mpu_modify_region((unsigned int)g_own_region,
                    base, ORT_DOMAIN_BLOCK_SIZE,
                    ORT_BLOCK_FLAGS1, ORT_BLOCK_FLAGS2);
}

#endif /* CONFIG_ORT_MEMDOMAIN */
