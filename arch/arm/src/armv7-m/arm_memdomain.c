/****************************************************************************
 * arch/arm/src/armv7-m/arm_memdomain.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * [ORT] Per-task MPU memory domain support (PROTOTYPE)
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

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* 域管理用的两个 MPU region 编号（惰性分配，-1 表示未初始化） */

static int g_pool_region = -1;   /* 整个池：no-access（低优先级） */
static int g_own_region  = -1;   /* 本任务块：user RW（高优先级） */

/* ── 内核 → 监督者的故障通道 ────────────────────────────────────────────
 *
 * 为什么需要独立通道：
 *   故障任务自己注册的 SIGSEGV 处理器是**容器可控**的 —— 容器把它删掉
 *   （NuttX 的 SIG_IGN 语义就是删除动作），监督者就再也收不到故障通知了。
 *   终止权已经不由容器决定（见 arm_memfault.c 的 SIGKILL 升级），
 *   但「知道出事了」不能也依赖容器 —— 那是降级状态机的输入。
 *
 * 通道设计（原型）：
 *   监督者启动时 prctl(PR_SET_ORT_SUPERVISOR) 注册；
 *   内核在用户态 memfault 时写一条单槽记录 + nxsig_queue() 发 ORT_SIGFAULT。
 *   信号只作唤醒用（sival_int = 故障容器 pid），详情由监督者
 *   prctl(PR_GET_ORT_FAULT) 取回。
 */

static pid_t g_supervisor = -1;
static struct ort_faultrec_s g_lastfault;

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

void ort_memdomain_bind(FAR struct tcb_s *tcb, int domain)
{
  if (tcb == NULL)
    {
      return;
    }

  /* 越界 → 视为解除绑定（拒绝访问，而不是给出错误映射） */

  if (domain < 0 || domain >= ORT_DOMAIN_COUNT)
    {
      tcb->xcp.domain_id = ORT_DOMAIN_UNBOUND;
    }
  else
    {
      tcb->xcp.domain_id = ORT_DOMAIN_ENCODE(domain);
    }
}

/****************************************************************************
 * Name: ort_supervisor_set
 *
 * Description:
 *   注册 ORT 监督者。只接受首次注册。
 *
 *   ⚠️ 原型期任何任务都能注册 —— 正式实现应只允许 ORT 启动器指定的
 *      那个任务（凭据 / 启动参数），否则恶意的「先注册者」可以顶掉
 *      真正的监督者。
 *
 ****************************************************************************/

int ort_supervisor_set(pid_t pid)
{
  if (g_supervisor >= 0)
    {
      return -EBUSY;
    }

  g_supervisor = pid;
  syslog(LOG_INFO, "[ORT] supervisor registered: pid=%d sig=%d\n",
         pid, ORT_SIGFAULT);
  return OK;
}

/****************************************************************************
 * Name: ort_fault_notify
 *
 * Description:
 *   记录一次容器故障并通知监督者。
 *   从 MemManage 异常处理上下文调用（不可阻塞）。
 *
 * Input Parameters:
 *   victim - 故障容器的 pid
 *   pc     - 触发故障的指令地址
 *   addr   - 被非法访问的地址
 *   faults - 该容器累计故障次数
 *
 ****************************************************************************/

void ort_fault_notify(pid_t victim, uintptr_t pc, uintptr_t addr,
                      uint32_t faults)
{
  union sigval value;
  int ret;

  g_lastfault.seq++;
  g_lastfault.victim = victim;
  g_lastfault.pc     = pc;
  g_lastfault.addr   = addr;
  g_lastfault.faults = faults;

  /* 监督者没注册，或故障的就是监督者自己 —— 无人可通知 */

  if (g_supervisor < 0 || g_supervisor == victim)
    {
      return;
    }

  value.sival_int = (int)victim;

  ret = nxsig_queue(g_supervisor, ORT_SIGFAULT, value);
  if (ret < 0)
    {
      /* 通知失败不能影响隔离动作本身 —— 终止流程必须继续走完。
       * 但要在日志里留下痕迹，否则监督者会「静默失明」。
       */

      if (ret == -ESRCH)
        {
          /* 监督者已经不存在了 → 注销。
           *
           * 为什么要报警而不是静默注销：监督者没了意味着**降级能力没了**，
           * 此后所有容器故障都不会有人处理。这在产品里是必须上抛的事件，
           * 不是可以顺带忽略的小事。
           */

          _alert("ORT: supervisor %d is gone -> deregistering "
                 "(降级能力已失效)\n", g_supervisor);
          g_supervisor = -1;
        }
      else
        {
          /* 通知失败不能影响隔离动作本身 —— 终止流程必须继续走完。
           * 但要在日志里留下痕迹，否则监督者会「静默失明」。
           */

          _alert("ORT: fault notify to supervisor %d failed: %d\n",
                 g_supervisor, ret);
        }
    }
}

/****************************************************************************
 * Name: ort_fault_record
 *
 * Description:
 *   取回最近一条故障记录。监督者用。
 *
 ****************************************************************************/

void ort_fault_record(FAR struct ort_faultrec_s *rec)
{
  *rec = g_lastfault;
}

void ort_memdomain_switch(FAR struct tcb_s *to)
{
  uintptr_t base;
  int domain;
  int bound;

  if (g_own_region < 0)
    {
      ort_memdomain_lazyinit();
    }

  /* 原型阶段的绑定方式：任务自己 prctl(PR_SET_ORT_DOMAIN)。
   *
   * ⚠️ 正式实现应由 ORT 监督者把域绑到 ContainerGroup（task_group_s），
   *    而不是让容器自己申报 —— 容器能自己申报就能自己越权。
   */

  bound = to != NULL ? to->xcp.domain_id : ORT_DOMAIN_UNBOUND;

  /* ★ 默认拒绝：未绑定 / 越界 → 不给任何域块。
   *
   *   这里必须用 ORT_DOMAIN_VALID() 而不是「domain >= 0」——
   *   xcp.domain_id 是 BSS 清零的，未绑定的任务天然是 0，
   *   一旦按「域 0」解释就会 fail-open（实测踩过，见 H28）。
   */

  if (!ORT_DOMAIN_VALID(bound))
    {
      /* 未绑定：把 own region 缩到最小并禁止访问 */

      mpu_modify_region((unsigned int)g_own_region,
                        ORT_DOMAIN_POOL_BASE, 32, ORT_FLAGS_DENY);
      return;
    }

  domain = ORT_DOMAIN_DECODE(bound);
  base   = ORT_DOMAIN_POOL_BASE + (uintptr_t)domain * ORT_DOMAIN_BLOCK_SIZE;

  /* ★ 关键动作：把 incoming 任务的域块编成 user-RW。
   *   池 region 保持 no-access，二者重叠时高编号（own）胜出。
   */

  mpu_modify_region((unsigned int)g_own_region,
                    base, ORT_DOMAIN_BLOCK_SIZE, ORT_FLAGS_ALLOW);
}


#endif /* CONFIG_ORT_MEMDOMAIN */
