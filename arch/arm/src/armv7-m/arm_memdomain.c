/****************************************************************************
 * arch/arm/src/armv7-m/arm_memdomain.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * [ORT] Per-container MPU memory domain support (PROTOTYPE)
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
 * Private Data
 ****************************************************************************/

/* 域管理用的两个 MPU region 编号（惰性分配，-1 表示未初始化） */

static int g_pool_region = -1;   /* 整个池：no-access（低优先级） */
static int g_own_region  = -1;   /* 本容器块：user RW（高优先级） */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* 池区属性：Strongly-ordered | Cacheable | Shareable | 无访问 */

#define ORT_FLAGS_DENY  (MPU_RASR_TEX_SO | MPU_RASR_C | MPU_RASR_S | \
                         MPU_RASR_AP_NONO)

/* 自有块属性：Strongly-ordered | Cacheable | Shareable | 特权/用户均可 RW */

#define ORT_FLAGS_ALLOW (MPU_RASR_TEX_SO | MPU_RASR_C | MPU_RASR_S | \
                         MPU_RASR_AP_RWRW)

/****************************************************************************
 * Name: ort_memdomain_lazyinit
 *
 * Description:
 *   首次切换时惰性初始化：向 MPU 申请两个 region 编号。
 *   用 mpu_configure_region() 而不是硬编码编号，避免与板级已用 region 冲突。
 *
 * ★ ARMv7-M（PMSAv7）的隔离手法：
 *     region_pool (低编号, 整个池, no-access)
 *     region_own  (高编号, 本容器的块, user RW)   ← 高编号优先，覆盖上面那个
 *
 *   PMSAv8 上不能这么做（region 不允许重叠），见 armv8-m 版本。
 *
 ****************************************************************************/

static void ort_memdomain_lazyinit(void)
{
  g_pool_region = (int)mpu_configure_region(ORT_DOMAIN_POOL_BASE,
                                            ORT_DOMAIN_POOL_SIZE,
                                            ORT_FLAGS_DENY);

  g_own_region  = (int)mpu_configure_region(ORT_DOMAIN_POOL_BASE,
                                            ORT_DOMAIN_BLOCK_SIZE,
                                            ORT_FLAGS_ALLOW);

  /* 高编号 region 优先，必须保证 own > pool */

  DEBUGASSERT(g_own_region > g_pool_region);

  syslog(LOG_INFO, "[ORT] memdomain init: pool_region=%d own_region=%d\n",
         g_pool_region, g_own_region);
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
 *   ★ ARMv7-M（MPU / CONFIG_BUILD_PROTECTED）提供
 *     ORT_CAP_FAULT_HANDLER：容器注册的 SIGSEGV 处理器**会被调用**。
 *
 *   为什么这边能做而 MMU 那边不能 —— 差别在**用户栈属于谁**：
 *
 *     - 这边用户栈是内核堆里的一块内存（up_create_stack → kmm_malloc），
 *       终止路径释放它时只是还给堆，页表不动 —— 脚下不会塌；
 *     - MMU 那边（BUILD_KERNEL）用户栈是地址环境里的一批页，
 *       终止时被解除映射 —— 正在其上取指就会跳飞。
 *
 *   实测（mps2-an500，orttest）：
 *     handler      → 处理器跑完并 _exit(42)，无升级
 *     handler-ret  → 处理器返回、再踩同一条指令 → faults=2 → SIGKILL
 *   三条终止路径各自走到了不同分支，详见手册 §三·补十三。
 *
 *   ⚠️ 注意"提供"的确切含义：处理器是**通知**不是**恢复** ——
 *      返回后容器仍会被 SIGKILL 终止。
 *
 ****************************************************************************/

uint32_t ort_caps(void)
{
  return ORT_CAP_FAULT_HANDLER;
}

/****************************************************************************
 * Name: ort_container_domain
 *
 * Description:
 *   查询容器的域。未绑定返回 -1。
 *   容器用它来等待「准入」—— 创建与绑域之间存在窗口，容器在绑好之前
 *   不应碰任何受控内存。
 *
 ****************************************************************************/

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

  /* ★ 只有监督者能绑域。
   *
   * 为什么不许容器自己申报：
   *   域号就是池里的块号。容器若能自己选块，就能选到**别的容器的块** ——
   *   那正是隔离要防的事。所以「绑哪个域」必须是监督者的决定。
   */

  if (nxsched_self()->pid != ort_supervisor_pid())
    {
      return -EPERM;
    }

  /* 取目标任务的 group。用 enter_critical_section 保护 ——
   * 目标任务可能正在退出，group 指针随时可能变。
   */

  flags = enter_critical_section();

  tcb = nxsched_get_tcb(pid);
  if (tcb == NULL || tcb->group == NULL)
    {
      leave_critical_section(flags);
      return -ESRCH;
    }

  group = tcb->group;

  /* 越界 → 视为解除绑定（拒绝访问，而不是给出错误映射） */

  if (domain < 0 || domain >= ORT_DOMAIN_COUNT)
    {
      group->tg_ort_domain = ORT_DOMAIN_UNBOUND;
    }
  else
    {
      group->tg_ort_domain = ORT_DOMAIN_ENCODE(domain);
    }

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: ort_memdomain_switch
 *
 * Description:
 *   上下文切换钩子：为 incoming 容器编程 MPU region。
 *
 ****************************************************************************/

void ort_memdomain_switch(FAR struct tcb_s *to)
{
  FAR struct task_group_s *group;
  uintptr_t base;
  int domain;
  int bound;

  if (g_own_region < 0)
    {
      ort_memdomain_lazyinit();
    }

  /* 域是**容器（group）级**的：同一容器的所有线程共享同一个域。 */

  group = to != NULL ? to->group : NULL;
  bound = group != NULL ? group->tg_ort_domain : ORT_DOMAIN_UNBOUND;

  /* ★ 默认拒绝：未绑定 / 越界 → 不给任何域块。
   *
   *   必须用 ORT_DOMAIN_VALID() 而不是「domain >= 0」——
   *   tg_ort_domain 是 BSS 清零的，未绑定的容器天然是 0，
   *   一旦按「域 0」解释就会 fail-open（实测踩过，见 H28）。
   */

  if (!ORT_DOMAIN_VALID(bound))
    {
      mpu_modify_region((unsigned int)g_own_region,
                        ORT_DOMAIN_POOL_BASE, 32, ORT_FLAGS_DENY);
      return;
    }

  domain = ORT_DOMAIN_DECODE(bound);
  base   = ORT_DOMAIN_POOL_BASE + (uintptr_t)domain * ORT_DOMAIN_BLOCK_SIZE;

  /* ★ 关键动作：把 incoming 容器的域块编成 user-RW。
   *   池 region 保持 no-access，二者重叠时高编号（own）胜出。
   */

  mpu_modify_region((unsigned int)g_own_region,
                    base, ORT_DOMAIN_BLOCK_SIZE, ORT_FLAGS_ALLOW);
}

#endif /* CONFIG_ORT_MEMDOMAIN */
