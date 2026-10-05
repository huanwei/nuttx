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

/* region 分配失败的表示（零初始化 = 正常，方向安全）。详见 lazyinit。 */

static bool g_memdomain_broken;

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
  unsigned int pool;
  unsigned int own;

  pool = mpu_configure_region(ORT_DOMAIN_POOL_BASE, ORT_DOMAIN_POOL_SIZE,
                              ORT_FLAGS_DENY);
  own  = mpu_configure_region(ORT_DOMAIN_POOL_BASE, ORT_DOMAIN_BLOCK_SIZE,
                              ORT_FLAGS_ALLOW);

  /* ★ 预算耗尽的判据（手册 §三·补五十四）。
   *
   *   mpu_allocregion() 在 region 用尽时**不报错** —— 它返回一个越界
   *   编号（其自带的 DEBUGASSERT 只在 debug 构建拦），而下面的
   *   "own > pool" 断言会被两个**垃圾编号**同时满足（8 > 7 之类），
   *   于是 debug 构建也拦不住。越界编号写进 MPU_RNR 被硬件忽略 ——
   *   region 静默不生效，隔离看起来"配好了"。
   *
   *   这就是 S1/H28 那一族的形状：**"配置失败"没有表示**。这里的
   *   表示 = g_memdomain_broken（零初始化 false = 正常，方向安全）：
   *     - 响亮报错（配置/构建问题，要在日志上看得见）；
   *     - 绑域从此一律 -ENOSPC（没有域机制就**不放容器进来** ——
   *       fail-closed，池保持板级 MPU 的原始 deny，不会裸奔）。
   */

  if (pool >= CONFIG_ARM_MPU_NREGIONS || own >= CONFIG_ARM_MPU_NREGIONS ||
      own <= pool)
    {
      _err("[ORT] memdomain init FAILED: MPU region exhausted/insane "
           "(pool=%u own=%u, NREGIONS=%d) — ORT domains disabled, "
           "container bind will fail with -ENOSPC\n",
           pool, own, CONFIG_ARM_MPU_NREGIONS);
      g_memdomain_broken = true;
      return;
    }

  g_pool_region = (int)pool;
  g_own_region  = (int)own;

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
 * Name: ort_domain_capacity
 *
 * Description:
 *   本架构**功能上**能给的域数 = 池块数（手册 §三·补五十四）。
 *   域号就是块号，池里有多少块就有多少域 —— 不要报"编码上限"。
 *
 ****************************************************************************/

int ort_domain_capacity(void)
{
  return ORT_DOMAIN_COUNT;
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

  /* ★ 域机制初始化失败（MPU region 预算耗尽，见 lazyinit）→ fail-closed：
   *   没有域机制就**不放容器进来**。池保持板级 MPU 的原始 deny，
   *   不会因"机制坏了但流程继续"而裸奔。 */

  if (g_memdomain_broken)
    {
      return -ENOSPC;
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

  /* ★ 预算之外的域号 → -EINVAL，**不改动**既有绑定（手册 §三·补五十四）。
   *
   *   此前这里的行为是"越界 → 静默解除绑定 + 返回 OK" —— 与 A 侧
   *   （-EINVAL）语义不同，而且 OK 回执是**假的**：监督者以为绑上了，
   *   容器其实没绑。现已统一：两个 SKU 都拒绝、报错、不动既有绑定。
   *
   *   为什么不保持"解除绑定"这条便路：全树核对过，没有任何调用方
   *   需要它；而"非法请求改变状态"本身就是坏契约 —— 撤销绑定要有
   *   显式的动作，不该藏在越界里。
   *
   *   注意判的是**预算**（min(容量, 配额)）而不是池块数：
   *   容量管物理，配额管部署，绑定只看两者取小。 */

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
 *   上下文切换钩子：为 incoming 容器编程 MPU region。
 *
 ****************************************************************************/

void ort_memdomain_switch(FAR struct tcb_s *to)
{
  FAR struct task_group_s *group;
  uintptr_t base;
  int domain;
  int bound;

  /* 初始化失败（region 预算耗尽）→ 不编程、不重试。
   * lazyinit 已响亮报错，绑域已 fail-closed；这里静默返回是**正确的
   * 静默** —— 池保持板级 MPU 的原始 deny，行为可推断。 */

  if (g_memdomain_broken)
    {
      return;
    }

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
