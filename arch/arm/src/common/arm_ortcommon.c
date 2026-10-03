/****************************************************************************
 * arch/arm/src/common/arm_ort.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * [ORT] ARM 各架构共用的内核侧机制：监督者槽位 + 故障事件队列
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_ORT_CONTAINER

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

#include "arm_ortcommon.h"

/****************************************************************************
 * [ORT] 容器状态槽：让"被替换的容器"能把状态交给接替者
 *
 * ★ 为什么需要它（见假设审计 H32）：
 *
 *   行业冗余靠 2× 硬件 + 一条**持续同步**的冗余链
 *   （Emerson 连续跟踪、Schneider 每 MAST 周期、ABB RCU-Link），
 *   切换才能做到"输出通道保持状态"的无扰。
 *   ORT 在单板上没有第二个计算域，但**状态延续这件事本身
 *   不依赖第二个计算域** —— 它只需要一个接替者读得到的落点。
 *
 *   所以这个槽是"冗余链"在单板上的对应物：
 *   现任**周期性**发布状态快照（模拟持续跟踪），
 *   接替者在启动时读回 —— 于是替换不再是冷启动。
 *
 * ★ 为什么放在内核而不是共享内存：
 *   - ORT-A（MMU）上两个实例地址空间完全独立，没有共享内存可用；
 *   - ORT-M（MPU）上虽然共用域块，但让容器自己去约定块内偏移
 *     是把耦合埋进容器里，换个平台就废。
 *   放内核则两个 SKU 同一套语义。
 *
 * ★ 访问控制：**只能读写自己域那一格**。
 *   domain 取自调用者的 task_group_s.tg_ort_domain ——
 *   那是监督者绑的，容器改不了。所以容器无法窥探别人的状态。
 *
 * ⚠️ 原型限制：快照只有 64 字节、槽位固定 8 个、无版本号、
 *    无 CRC。正式实现需要（a）按容器配额决定大小，
 *    （b）带 seq + 校验（读到撕裂的快照必须能识别），
 *    （c）跨版本兼容（新旧实例的结构体可能不同）。
 ****************************************************************************/

#define ORT_STATE_SLOTS  8
#define ORT_STATE_MAX    64

struct ort_state_slot_s
{
  uint32_t seq;                      /* 发布序号；0 = 从未发布 */
  uint32_t len;                      /* 有效字节数 */
  uint8_t  data[ORT_STATE_MAX];
};

static struct ort_state_slot_s g_ort_state[ORT_STATE_SLOTS];

/* 取调用者自己的域号，并映射到槽位下标。
 * 未绑定域（tg_ort_domain == 0）的调用者一律拒绝 ——
 * 没有域就没有容器身份，也就没有"该读哪一格"可言。 */

static int ort_state_slot(void)
{
  FAR struct task_group_s *group;
  int domain;

  if (nxsched_self() == NULL)
    {
      return -ESRCH;
    }

  group = nxsched_self()->group;
  if (group == NULL || group->tg_ort_domain == 0)
    {
      return -EPERM;
    }

  domain = (int)group->tg_ort_domain - 1;
  if (domain < 0 || domain >= ORT_STATE_SLOTS)
    {
      return -ERANGE;
    }

  return domain;
}

int ort_state_put(FAR const void *buf, size_t len)
{
  int slot = ort_state_slot();
  irqstate_t flags;

  if (slot < 0)
    {
      return slot;
    }

  if (buf == NULL || len == 0 || len > ORT_STATE_MAX)
    {
      return -EINVAL;
    }

  flags = up_irq_save();

  memcpy(g_ort_state[slot].data, buf, len);
  g_ort_state[slot].len = (uint32_t)len;
  g_ort_state[slot].seq++;

  up_irq_restore(flags);
  return OK;
}

int ort_state_get(FAR void *buf, size_t len)
{
  int slot = ort_state_slot();
  uint32_t n;
  irqstate_t flags;

  if (slot < 0)
    {
      return slot;
    }

  if (buf == NULL || len == 0)
    {
      return -EINVAL;
    }

  flags = up_irq_save();

  /* ★ 从未发布过就明确说"没有"，而不是返回 0 字节当成功。
   *   调用者必须能区分"接续了旧状态"和"没有旧状态可接续" ——
   *   把这两件事混起来，正是 H31/H32 那类错误的温床。 */

  if (g_ort_state[slot].seq == 0)
    {
      up_irq_restore(flags);
      return -ENOENT;
    }

  n = g_ort_state[slot].len < len ? g_ort_state[slot].len : (uint32_t)len;
  memcpy(buf, g_ort_state[slot].data, n);

  up_irq_restore(flags);
  return (int)n;
}

/****************************************************************************
 * Private Data
 ****************************************************************************/

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
 *   lost  —— 本条之前被丢弃的条数（0 = 无丢失），直接放在记录里，
 *            这样监督者读到的每一条都自带"我之前丢过多少"，不会漏判
 *
 * 只有监督者能读：故障记录是**监督者的私有视图**。
 * 放开读会让容器能消费掉监督者的事件、或窥探别的容器的故障地址。
 *
 * 写入方是异常处理上下文（不可阻塞），所以只用最朴素的赋值，
 * 不分配、不等待。生产者 IRQ 上下文 / 消费者任务上下文，
 * 竞争窗口靠"读游标只在消费者侧推进"来约束。
 */

#define ORT_FAULTQ_SIZE  16

static struct ort_faultrec_s g_faultq[ORT_FAULTQ_SIZE];
static uint32_t g_faultq_total;     /* 产生的事件总数（= 最后一条的 seq） */
static uint32_t g_faultq_read;      /* 已被监督者取走的条数 */
static uint32_t g_faultq_dropped;   /* 累计丢弃条数 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

pid_t ort_supervisor_pid(void)
{
  return g_supervisor;
}

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
       *     - 容器的域分配权
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

      _alert("ORT: supervisor slot is pinned to pid=%d, rejecting pid=%d\n",
             g_supervisor, pid);
      return -EBUSY;
    }

  g_supervisor        = pid;
  g_supervisor_pinned = true;

  /* ★ 新一任监督者上任 → 清空事件队列。
   *
   * 事件是属于**某一任监督者**的。上一任缺席期间积压的事件已经没有
   * 正确的处理者（那些容器早被内核终止，没人会重启它们），继续投递
   * 只会让新监督者基于陈旧信息做决策。
   */

  g_faultq_total   = 0;
  g_faultq_read    = 0;
  g_faultq_dropped = 0;

  syslog(LOG_INFO, "[ORT] supervisor pinned: pid=%d sig=%d\n",
         pid, ORT_SIGFAULT);
  return OK;
}

#ifdef CONFIG_ORT_SUPERVISOR_RESET
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
        /* 队列满 —— 丢掉**最旧的**一条。
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

  value.sival_int = (int)victim;

  ret = nxsig_queue(g_supervisor, ORT_SIGFAULT, value);
  if (ret < 0)
    {
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

int ort_fault_read(FAR struct ort_faultrec_s *rec)
{
  uint32_t pending;

  /* ★ 只有监督者能读 */

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

#endif /* CONFIG_ORT_CONTAINER */
