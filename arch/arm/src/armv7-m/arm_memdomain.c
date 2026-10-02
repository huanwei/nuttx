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
static bool  g_supervisor_pinned;

/* ── 故障事件队列 ──────────────────────────────────────────────────────
 *
 * 为什么必须是队列而不是单槽：
 *   单槽 + "记住最近一次 victim"的匹配方式在**并发故障**下会认错容器。
 *   两个容器几乎同时失效时，监督者可能拿到 A 的受害 pid 却配 B 的详情，
 *   或者干脆丢掉一条 —— 而它据此决定重启谁、要不要进安全态。
 *
 * 环形缓冲 + 单调序号：
 *   seq   —— 事件序号（从 1 开始），监督者据此判断是否漏收
 *   lost  —— 本条之前被丢弃的条数（0 = 无丢失）
 *
 *   lost 直接放在记录里而不是单独查询：监督者读到的每一条都自带
 *   "我之前丢过多少"，不需要额外的状态查询接口，也不会漏判。
 *
 * 只有监督者能读（见 ort_fault_read）：故障记录是**监督者的私有视图**，
 * 容器读不到 —— 否则容器可以消费掉监督者的事件，或者窥探别的容器的故障。
 *
 * 写入方是异常处理上下文（不可阻塞），所以这里只用最朴素的赋值，
 * 不分配、不等待。生产者在 IRQ 上下文、消费者在任务上下文，
 * 竞争窗口靠"读游标只在消费者侧推进"来约束。
 */

#define ORT_FAULTQ_SIZE  16

static struct ort_faultrec_s g_faultq[ORT_FAULTQ_SIZE];
static uint32_t g_faultq_total;     /* 产生的事件总数（= 最后一条的 seq） */
static uint32_t g_faultq_read;      /* 已被监督者取走的条数 */
static uint32_t g_faultq_dropped;   /* 累计丢弃条数 */

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
   *
   *   这不是理论风险：早期原型就是自助式绑定（prctl 直接写自己的域），
   *   等价于把隔离的门钥匙交给被隔离的人。
   */

  if (nxsched_self()->pid != g_supervisor)
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
 * Name: ort_supervisor_set
 *
 * Description:
 *   注册 ORT 监督者。**首次注册即钉住**。
 *
 *   钉住之后只有同一个任务能重复注册（幂等），其它一律 -EBUSY，
 *   无论原监督者是否还活着。
 *
 *   ⚠️ 仍缺的一环：**谁有资格做"首次注册"**。
 *      当前是"先到先得"，在 ORT 的架构里够用（监督者在创建任何容器
 *      之前就注册了，此时系统里只有它），但这是**依赖启动顺序**而非
 *      强制约束。正式实现应把资格绑定到 ORT 启动器指定的那个任务
 *      （构建配置 + 凭据），见 patches/nuttx/README.md 的欠账。
 *
 ****************************************************************************/

int ort_supervisor_set(pid_t pid)
{
  if (g_supervisor_pinned)
    {
      /* ★ 已钉住：只接受**同一个任务**的重复注册（幂等），
       *   其它任务一律拒绝 —— **无论原监督者是否还活着**。
       *
       * 为什么不能"原监督者没了就允许接管"：
       *   那等于把"谁能当监督者"变成运行期竞争。任何任务只要等到
       *   监督者退出（或干脆把它耗死）就能补位，从而接管：
       *     - 故障通知的收件人（瞎掉真监督者）
       *     - 容器的域分配权（PR_SET_ORT_DOMAIN）
       *   安全动作绝不能依赖被管理者的善意 —— 这是公理 S1。
       *
       * 监督者退出是**灾难性事件**，不是"换个任务继续"的场景：
       *   它意味着降级能力没了。此时系统应当由独立的看门狗判定，
       *   而不是让内核把槽位空出来等人抢。
       */

      if (pid == g_supervisor)
        {
          return OK;
        }

      _alert("ORT: supervisor slot is pinned to pid=%d, "
             "rejecting pid=%d\n", g_supervisor, pid);
      return -EBUSY;
    }

  g_supervisor        = pid;
  g_supervisor_pinned = true;

  /* ★ 新一任监督者上任 → 清空事件队列。
   *
   * 为什么：事件是属于**某一任监督者**的。上一任缺席期间积压的事件，
   * 已经没有正确的处理者了 —— 那些容器早就被内核终止（终止权不依赖
   * 监督者），没人会去重启它们，事件本身也失去了时效。
   * 继续投递只会让新监督者基于陈旧信息做决策（实测踩过：
   * 测试程序换一轮跑，前一轮残留的事件被当成新事件处理）。
   *
   * 生产环境里监督者在创建任何容器之前就注册，队列本来就是空的 ——
   * 这个清理是防御性的。原型期它同时承担"新监督者 = 新任期"的语义。
   */

  g_faultq_total   = 0;
  g_faultq_read    = 0;
  g_faultq_dropped = 0;

  syslog(LOG_INFO, "[ORT] supervisor pinned: pid=%d sig=%d\n",
         pid, ORT_SIGFAULT);
  return OK;
}

#ifdef CONFIG_ORT_SUPERVISOR_RESET
/****************************************************************************
 * Name: ort_supervisor_reset
 *
 * Description:
 *   释放监督者槽位。⚠️ 仅原型测试用，见 sched/Kconfig 的说明。
 *
 *   正式产品必须关闭 CONFIG_ORT_SUPERVISOR_RESET ——
 *   否则这个接口本身就是"任何任务都能顶掉监督者"的后门。
 *
 ****************************************************************************/

int ort_supervisor_reset(void)
{
  _alert("ORT: supervisor slot reset by pid=%d "
         "(PROTOTYPE ONLY — 正式构建不应存在此路径)\n",
         nxsched_self()->pid);
  g_supervisor        = -1;
  g_supervisor_pinned = false;
  return OK;
}
#endif

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

  /* 入队。先算 lost，再写槽位。 */

  {
    uint32_t pending = g_faultq_total - g_faultq_read;
    uint32_t slot;

    if (pending >= ORT_FAULTQ_SIZE)
      {
        /* 队列满 —— 丢掉**最旧的**一条，并推进读游标。
         *
         * 为什么丢最旧的而不是拒绝新的：监督者要处理的是"现在出了什么事"，
         * 陈旧事件的价值最低。丢新的会让监督者永远滞后。
         */

        g_faultq_read++;
        g_faultq_dropped++;
      }

    slot = g_faultq_total % ORT_FAULTQ_SIZE;

    g_faultq_total++;

    g_faultq[slot].seq    = g_faultq_total;
    g_faultq[slot].lost   = g_faultq_dropped;
    g_faultq[slot].victim = victim;
    g_faultq[slot].pc     = pc;
    g_faultq[slot].addr   = addr;
    g_faultq[slot].faults = faults;
  }

  /* 监督者没注册，或故障的就是监督者自己 —— 无人可通知 */

  if (g_supervisor < 0 || g_supervisor == victim)
    {
      return;
    }

  /* 注意：这里不检查 g_supervisor_pinned —— 槽位非空就意味着已钉住，
   * 只需看是否需要通知。
   */

  value.sival_int = (int)victim;

  ret = nxsig_queue(g_supervisor, ORT_SIGFAULT, value);
  if (ret < 0)
    {
      /* 通知失败不能影响隔离动作本身 —— 终止流程必须继续走完。
       * 但要在日志里留下痕迹，否则监督者会「静默失明」。
       */

      /* ★ 不清槽。
       *
       * 槽位保持钉住，后续每次故障都会再报一次 —— 这不是刷屏，
       * 而是**持续暴露降级能力已经失效**这个事实。
       * 清槽会让下一个注册者补位，把一个安全事件变成一次接管机会。
       *
       * 通知失败不能影响隔离动作本身：容器的终止流程必须继续走完
       * （终止权在内核手里，不依赖监督者）。
       */

      _alert("ORT: fault notify to supervisor %d failed: %d "
             "(降级能力失效，槽位保持钉住)\n", g_supervisor, ret);
    }
}

/****************************************************************************
 * Name: ort_fault_record
 *
 * Description:
 *   取回最近一条故障记录。监督者用。
 *
 ****************************************************************************/

int ort_fault_read(FAR struct ort_faultrec_s *rec)
{
  uint32_t pending;

  /* ★ 只有监督者能读。
   *
   * 故障记录是监督者的**私有视图**。放开读会带来两个问题：
   *   ① 容器可以消费掉监督者的事件（让它漏掉故障）
   *   ② 容器可以窥探其它容器的故障地址/PC（信息泄露）
   */

  if (nxsched_self()->pid != g_supervisor)
    {
      return -EPERM;
    }

  if (rec == NULL)
    {
      return -EINVAL;
    }

  pending = g_faultq_total - g_faultq_read;
  if (pending == 0)
    {
      return 0;                 /* 暂无新事件 */
    }

  *rec = g_faultq[g_faultq_read % ORT_FAULTQ_SIZE];

  g_faultq_read++;

  return (int)rec->seq;
}

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

  /* 域是**容器（group）级**的：同一容器的所有线程共享同一个域。
   *
   * to->group 理论上恒非空（idle 走 group_allocate，内核线程共享
   * g_kthread_group），但仍做防御性检查 —— 这里在上下文切换路径上，
   * 一个空指针就是整机 panic。
   */

  group = to != NULL ? to->group : NULL;
  bound = group != NULL ? group->tg_ort_domain : ORT_DOMAIN_UNBOUND;

  /* ★ 默认拒绝：未绑定 / 越界 → 不给任何域块。
   *
   *   这里必须用 ORT_DOMAIN_VALID() 而不是「domain >= 0」——
   *   tg_ort_domain 是 BSS 清零的，未绑定的容器天然是 0，
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

  /* ★ 关键动作：把 incoming 容器的域块编成 user-RW。
   *   池 region 保持 no-access，二者重叠时高编号（own）胜出。
   */

  mpu_modify_region((unsigned int)g_own_region,
                    base, ORT_DOMAIN_BLOCK_SIZE, ORT_FLAGS_ALLOW);
}


#endif /* CONFIG_ORT_MEMDOMAIN */
