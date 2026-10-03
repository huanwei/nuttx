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

/* ★★ seqlock：seq 兼作"写入中"标志（见 §三·补二十九·四补）
 *
 *   单核时 put/get 都在 up_irq_save() 里，两者不可能交错，所以
 *   "撕裂发生不了"。但 `up_irq_save()` 在 SMP 下**只关本核中断** ——
 *   另一个核照样能跑进同一段 memcpy。
 *
 *   实测（4 核 · 压力模式）：读 17 次就撞上撕裂，w0 与 w5 相差 1。
 *   所以改成 seqlock：
 *
 *     偶数 = 稳定；奇数 = 正在写
 *
 *   写者把 seq 变成奇数、写完变回偶数；读者首尾各读一次 seq，
 *   两次相等才认这次拷贝。
 *
 *   "读到的字节是不是自洽的"这个判据**在这一层就能判**，
 *   不需要容器自己算校验和 —— 内核知道它在跟谁抢。
 */

struct ort_state_slot_s
{
  volatile uint32_t seq;             /* 偶数=稳定 / 奇数=写入中；0 = 无快照 */
  uint32_t pubs;                     /* **累计**发布次数；故障作废时不清零 */
  uint32_t len;                      /* 有效字节数 */
  uint8_t  data[ORT_STATE_MAX];
};

/* 读端重试上限。
 *
 * ★ 必须**有界**：seqlock 的读端在写端持续写入时可以一直重试 ——
 *   而这是一个硬实时系统，无界重试等于无界响应时间。
 *   用完就如实返回 -EAGAIN，让调用者知道"这次没读到一致快照"，
 *   而不是返回一份可能撕裂的数据、也不是假装"没有旧状态"。
 */

#define ORT_SEQLOCK_RETRIES  128

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

/****************************************************************************
 * Name: ort_state_invalidate
 *
 * Description:
 *   作废某个域的状态槽 —— 下一次 GET 会返回 -ENOENT，
 *   于是接替者从冷态开始（COLD_START）。
 *
 *   由 ort_fault_notify() 在容器故障时调用，见那里的说明。
 *
 ****************************************************************************/

void ort_state_invalidate(int domain)
{
  irqstate_t flags;

  if (domain < 0 || domain >= ORT_STATE_SLOTS)
    {
      return;
    }

  flags = up_irq_save();

  g_ort_state[domain].seq = 0;   /* seq == 0 即"当前没有可接续的快照" */
  g_ort_state[domain].len = 0;

  /* ★ pubs（累计发布次数）**刻意不清零**。
   *
   *   它回答的是另一个问题："这个容器到底发布过没有" ——
   *   而作废一个快照并不能让"它发布过"这件事变成没发生过。
   *
   *   把两者混起来会造出一个**假阳性**（实测踩过）：
   *   容器故障 → 内核作废槽 → 监督者还没来得及消费故障事件，
   *   就先看到"槽是空的" → 判定"它没实现发布"。
   *   而它明明一直在发布，只是刚死。
   *
   *   用累计计数之后，判据变成"**从来**没发布过" ——
   *   这个结论不受作废影响，也就不受事件消费顺序影响。 */

  up_irq_restore(flags);
}

/****************************************************************************
 * Name: ort_state_seq
 *
 * Description:
 *   查询**指定域**发布过多少次 —— 只有监督者能调。
 *
 *   ★ 为什么需要它（这是"声明可信度"问题的正解之一）：
 *
 *   manifest 里的 `protocol = 1` 是**声明**，不是事实 ——
 *   容器没实现发布，监督者无从知道，除非它能从外部看见"发布"这件事。
 *
 *   监督者不能读容器内存（读了也不能信，见公理 S1），但它可以问内核：
 *   **那个域的状态槽被写过没有**。槽按域索引、只有该域的容器能写、
 *   监督者能查 —— 于是"这个容器到底发布没发布"变成了一个
 *   **监督者单方面可判定**的事实。这正是公理 S1 要的形态：
 *   不依赖失效组件的自述。
 *
 *   于是 protocol 的声明可以被**反向证伪**：
 *     声明 protocol=1 + 健康运行超过启动窗口 + 槽还是空的  →  没实现。
 *
 * ★ 不给容器这个接口：容器能读别人的发布计数是没必要的旁路，
 *   而它自己的发布它自己清楚。
 *
 * ★ 返回的是**累计**发布次数，故障作废**不清零** —— 见 invalidate 的说明：
 *   判据要的是"**从来**没发布过"，而不是"现在槽是空的"。
 *
 * Returned Value:
 *   累计发布次数（0 = 从来没发布过）；负 errno。
 *
 ****************************************************************************/

int ort_state_seq(int domain)
{
  FAR struct tcb_s *rtcb = nxsched_self();
  irqstate_t flags;
  uint32_t seq;

  if (rtcb == NULL || rtcb->pid != ort_supervisor_pid())
    {
      return -EPERM;
    }

  if (domain < 0 || domain >= ORT_STATE_SLOTS)
    {
      return -EINVAL;
    }

  flags = up_irq_save();
  seq = g_ort_state[domain].pubs;
  up_irq_restore(flags);

  /* 计数只增不减，理论上会绕回；压到 INT_MAX 免得某天变成负数
   * 被调用者当成错误。100 ms 一次发布要跑 6 年才到那里。 */

  return (seq > (uint32_t)INT_MAX) ? INT_MAX : (int)seq;
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

  /* 本核关中断：写者不会在自己的核上被读者打断，
   * 于是"写入中"这个窗口只对**别的核**可见 —— 那正是 seqlock 管的。 */

  flags = up_irq_save();

  g_ort_state[slot].seq++;               /* → 奇数：写入中 */
  SMP_WMB();

  memcpy(g_ort_state[slot].data, buf, len);
  g_ort_state[slot].len = (uint32_t)len;
  g_ort_state[slot].pubs++;

  SMP_WMB();
  g_ort_state[slot].seq++;               /* → 偶数：完成 */

  up_irq_restore(flags);
  return OK;
}

int ort_state_get(FAR void *buf, size_t len)
{
  int slot = ort_state_slot();
  uint32_t n;
  int retry;

  if (slot < 0)
    {
      return slot;
    }

  if (buf == NULL || len == 0)
    {
      return -EINVAL;
    }

  /* ★ 读端**不需要关中断** —— 这是 seqlock 相对于锁的地方：
   *   它不阻塞写者，写者也不阻塞它。代价是可能读到不一致，
   *   而"首尾 seq 相等"就是**内核自己判定的**一致性判据。
   *
   * ★ 三种结果必须互相区分（这是本项目反复踩的那条）：
   *     >0     读到了一致快照
   *     -ENOENT 压根没有快照（从未发布 / 故障作废）
   *     -EAGAIN 有快照，但重试用尽仍没读到一致的
   *   把 -EAGAIN 混进 -ENOENT（当成"冷启动"）就是把
   *   "没读到"说成了"没有" —— 又一个 H31。 */

  for (retry = 0; retry < ORT_SEQLOCK_RETRIES; retry++)
    {
      uint32_t s1 = g_ort_state[slot].seq;

      if (s1 == 0)
        {
          return -ENOENT;          /* 从未发布 / 已被故障作废 */
        }

      if (s1 & 1u)
        {
          continue;                /* 写者正在写这一格 */
        }

      SMP_RMB();

      n = g_ort_state[slot].len < len ? g_ort_state[slot].len
                                      : (uint32_t)len;
      memcpy(buf, g_ort_state[slot].data, n);

      SMP_RMB();

      if (g_ort_state[slot].seq == s1)
        {
          return (int)n;           /* 首尾一致 → 这次拷贝自洽 */
        }
    }

  return -EAGAIN;
}

/****************************************************************************
 * [ORT] 配置槽：部署/O&M 代理 → 监督者的**唯一**配置通道
 *
 * ★ 为什么要有它（设计见 proposals/《部署与 O&M 组件设计》）：
 *
 *   在此之前监督者每 2 秒自己 fopen manifest + 逐行解析 ——
 *   也就是**实时控制环里做文件 I/O**。硬实时关心的是最坏耗时，
 *   而 hostfs / flash 的最坏延迟都不可控。一个 5 ms 周期的控制循环
 *   不能建立在"读文件很快"这个假设上。
 *
 *   切法：把"搬"和"保证实时"分成两个组件。
 *     - 部署/O&M 代理（非实时约束）读文件/收下发，把**原样字节**放进槽；
 *     - 监督者每周期只读一个标量（代数），变了才取回并**自己校验**。
 *
 *   Wind River 三层的答案形状相同：VxWorks 653 是 Module OS 在 init 期装载
 *   配置 + mode manager 分区请求变更；VxWorks 7 是目标上另跑一个 kubelet
 *   （不参与实时调度）；Studio OTA 是设备侧独立的 eSync client/agent。
 *   **共同点是实时控制环从不做 I/O。**
 *
 * ★ 为什么放内核而不是共享内存：
 *   ORT-A（MMU）上代理与监督者**地址空间完全独立**，根本没有共享内存可用。
 *   放内核则监督者侧是一次**有界 memcpy**，无阻塞、大小已知、WCET 可算。
 *   这与容器状态槽是同一个形状 —— 不是新发明。
 *
 * ★ 信任模型：**代理只搬运，校验权在监督者**。
 *   代理是非实时、可重启、可能被降级的组件，按公理 S1 它的输出只能当
 *   **输入**看待。所以这里不做任何格式校验 —— 只存字节。
 *   校验逻辑只有一份，不会出现"代理的规则和监督者的不一致"这种经典漏洞。
 *
 * ★ 权限：**容器写不了配置槽**（一律 -EPERM）。否则一个被攻陷的容器
 *   可以给自己放宽 max_restarts、或把别的 CG 的 critical 改成 false ——
 *   那是把编排层的信任边界交给被编排的对象。
 *
 * ★ 代数与心跳是**两个**计数器（踩过一次的教训的形状，见状态槽的 pubs）：
 *     generation —— 只在**内容真的变了**时才加。监督者据此决定要不要解析。
 *     tick       —— 代理每跑一圈就加，**与内容变没变无关**。
 *   两者混成一个的话，"代理死了"和"配置本来就不用变"看起来一模一样 ——
 *   而后者是正常状态。那正是 H31 那一族。
 *
 * ⚠️ 原型限制：槽是定长 4 KB 单槽、无 CRC。正式实现应按配额定大小、
 *    带校验（识别撕裂的写入）、并考虑 A/B 双槽做原子切换。
 ****************************************************************************/

#define ORT_CFG_MAX  4096

struct ort_cfg_slot_s
{
  volatile uint32_t seq;             /* seqlock：偶数=稳定 / 奇数=写入中 */
  uint32_t generation;               /* 内容变化的次数；0 = 从未写入 */
  uint32_t tick;                     /* 代理心跳；只增不减 */
  uint32_t len;                      /* 有效字节数 */
  uint8_t  data[ORT_CFG_MAX];
};

static struct ort_cfg_slot_s g_ort_cfg;

static pid_t g_deploy = -1;
static bool  g_deploy_pinned;

/* 只有钉住的部署代理能写；只有监督者能读/查 */

static bool ort_is_deploy(void)
{
  FAR struct tcb_s *rtcb = nxsched_self();

  return rtcb != NULL && g_deploy_pinned && rtcb->pid == g_deploy;
}

/****************************************************************************
 * Name: ort_deploy_set
 *
 * Description:
 *   注册部署/O&M 代理。与监督者槽位同一套规则：**首次注册即钉住**，
 *   之后别的任务一律 -EBUSY（除非是同一个任务的重复注册）。
 *
 *   ★ 为什么不能"原代理退出就允许接管"：那等于把"谁能改配置"
 *     变成运行期竞争 —— 任何任务只要等代理退出（或把它耗死）
 *     就能改写整份部署配置，包括别的 CG 的关键等级。
 *     与监督者槽位是同一个理由，见那里的说明。
 *
 ****************************************************************************/

int ort_deploy_set(pid_t pid)
{
  if (g_deploy_pinned)
    {
      if (pid == g_deploy)
        {
          return OK;
        }

      _alert("ORT: deploy slot is pinned to pid=%d, rejecting pid=%d\n",
             g_deploy, pid);
      return -EBUSY;
    }

  g_deploy        = pid;
  g_deploy_pinned = true;

  syslog(LOG_INFO, "[ORT] deploy agent pinned: pid=%d\n", pid);
  return OK;
}

#ifdef CONFIG_ORT_SUPERVISOR_RESET
void ort_deploy_reset(void)
{
  _alert("ORT: deploy slot reset by pid=%d "
         "(PROTOTYPE ONLY — 正式构建不应存在此路径)\n",
         nxsched_self()->pid);
  g_deploy        = -1;
  g_deploy_pinned = false;
}
#endif

/****************************************************************************
 * Name: ort_cfg_put
 *
 * Description:
 *   部署代理把 manifest 的**原样字节**放进槽（不解析、不校验）。
 *
 *   generation **只在内容真的变了**时才加 —— 由内核做比较，
 *   代理侧因此无状态：它不需要记住上次写了什么。
 *
 *   心跳**不在这里**（见 ort_cfg_alive）：代理读不到源文件时没有新内容
 *   可写，但它还活着 —— 两件事混在一起会造出不实的"失联"告警。
 *
 ****************************************************************************/

int ort_cfg_put(FAR const void *buf, size_t len)
{
  irqstate_t flags;
  bool changed;

  if (!ort_is_deploy())
    {
      return -EPERM;
    }

  if (buf == NULL || len == 0 || len > ORT_CFG_MAX)
    {
      return -EINVAL;
    }

  flags = up_irq_save();

  changed = (g_ort_cfg.len != len) ||
            memcmp(g_ort_cfg.data, buf, len) != 0;

  /* 与状态槽同一个 seqlock 形状 —— 代理在某个核上写，
   * 监督者在另一个核上读，两边都不许看到半新半旧。 */

  g_ort_cfg.seq++;
  SMP_WMB();

  memcpy(g_ort_cfg.data, buf, len);
  g_ort_cfg.len  = (uint32_t)len;

  SMP_WMB();
  g_ort_cfg.seq++;

  if (changed)
    {
      g_ort_cfg.generation++;
    }

  up_irq_restore(flags);
  return OK;
}

/****************************************************************************
 * Name: ort_cfg_seq / ort_cfg_tick / ort_cfg_get
 *
 * Description:
 *   监督者侧的三次查询。都是**有界的**（前两个是标量，第三个是有界
 *   memcpy）—— 控制循环里因此不存在任何不可控的最坏耗时。
 *
 *   第三个必须要一个用户缓冲区：槽有 4 KB，塞不进返回值。
 *   ⚠️ 与 PR_GET_ORT_FAULT / PR_ORT_STATE_* 同一个原型债：
 *      内核直接按用户指针写，**没有做指针合法性校验**。
 *
 ****************************************************************************/

/****************************************************************************
 * Name: ort_cfg_alive
 *
 * Description:
 *   代理心跳：**与内容变没变无关**，代理每跑一圈调一次。
 *
 *   ★ 为什么必须与 put 分开：代理读不到源文件（文件被移走、介质出错）
 *     时它**没有新内容可写**，但它**还活着**。如果心跳只挂在 put 上，
 *     这种情形会被上报成"代理失联" —— 那是一条**不实的告警**。
 *     告警必须只由它真正想表达的事实触发。
 *
 ****************************************************************************/

int ort_cfg_alive(void)
{
  irqstate_t flags;

  if (!ort_is_deploy())
    {
      return -EPERM;
    }

  flags = up_irq_save();
  g_ort_cfg.tick++;
  up_irq_restore(flags);
  return OK;
}

int ort_cfg_seq(void)
{
  FAR struct tcb_s *rtcb = nxsched_self();
  irqstate_t flags;
  uint32_t v;

  if (rtcb == NULL || rtcb->pid != ort_supervisor_pid())
    {
      return -EPERM;
    }

  flags = up_irq_save();
  v = g_ort_cfg.generation;
  up_irq_restore(flags);

  return (v > (uint32_t)INT_MAX) ? INT_MAX : (int)v;
}

int ort_cfg_tick(void)
{
  FAR struct tcb_s *rtcb = nxsched_self();
  irqstate_t flags;
  uint32_t v;

  if (rtcb == NULL || rtcb->pid != ort_supervisor_pid())
    {
      return -EPERM;
    }

  flags = up_irq_save();
  v = g_ort_cfg.tick;
  up_irq_restore(flags);

  return (v > (uint32_t)INT_MAX) ? INT_MAX : (int)v;
}

int ort_cfg_get(FAR void *buf, size_t cap)
{
  FAR struct tcb_s *rtcb = nxsched_self();
  uint32_t n;
  int retry;

  if (rtcb == NULL || rtcb->pid != ort_supervisor_pid())
    {
      return -EPERM;
    }

  if (buf == NULL || cap == 0)
    {
      return -EINVAL;
    }

  /* seqlock 读端，与状态槽同一形状。三种结果同样必须区分：
   *   >0 = 一致快照；-ENOENT = 从来没有；-EAGAIN = 有但没读到一致的。 */

  for (retry = 0; retry < ORT_SEQLOCK_RETRIES; retry++)
    {
      uint32_t s1 = g_ort_cfg.seq;

      if (s1 == 0)
        {
          return -ENOENT;          /* 代理还没送来过任何配置 */
        }

      if (s1 & 1u)
        {
          continue;
        }

      SMP_RMB();

      n = g_ort_cfg.len < cap ? g_ort_cfg.len : (uint32_t)cap;
      memcpy(buf, g_ort_cfg.data, n);

      SMP_RMB();

      if (g_ort_cfg.seq == s1)
        {
          return (int)n;
        }
    }

  return -EAGAIN;
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

#ifdef CONFIG_ORT_SUPERVISOR_RESET
/* ⚠️ **仅原型测试**：故障通知信号的总开关（默认开）。
 *
 * ★ 它是**对照实验**用的，不是产品开关。
 *
 *   压故障队列的装置跑起来会让内核 assert（`irq/irq_csection.c:205`，
 *   `enter_critical_section()` 里 `current_task(cpu) == NULL`）。
 *   但那个装置同时压了三样东西：
 *     1. 队列的计数器与环（R1..R5 住的地方）
 *     2. **每条事件一次 `nxsig_queue`** —— 百万级信号投递
 *     3. 百万级 `prctl` 系统调用往返
 *
 *   要归因就得一次只留一样。这个开关把第 2 样单独摘掉：
 *   关掉之后**队列逻辑一字不改**（照常入队、照常计数、照常丢最旧），
 *   只是不再叫醒监督者。
 *
 *   做成**运行期开关**而不是编译期宏：两次对照必须用**同一个二进制**，
 *   否则"换了个编译"和"换了个变量"分不开 —— 那正是本项目反复
 *   栽的那一类。
 *
 *   产品构建里不存在（与 PR_ORT_SUPERVISOR_RESET 同一个门）。 */

static bool g_faultq_nosig;
#endif

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

  /* ── 作废该容器的状态槽 ────────────────────────────────────────────
   *
   * ★ 这一行实现的是 ARINC 653 里 COLD_START 与 WARM_START 的区分。
   *
   *   计划内替换（部署）：前身是**正常退出**的 → 状态可信 → 接替者接续。
   *   故障重启（崩溃后）  ：前身的**状态可能已经被污染**（往往正是它崩的
   *                        原因）→ 接替者必须从冷态开始。
   *
   *   不区分的话，崩溃前的脏状态会被一路传下去 —— 那不是"状态延续"，
   *   那是**故障传播**。
   *
   * ★ 为什么放在这里（而不是监督者里）：
   *
   *   ort_fault_notify() 是**三个架构共用的"某容器故障了"的唯一入口**
   *   （armv7-a 的 abort、armv7-m/armv8-m 的 memfault 都调它）。
   *   放在这里一处改、三个平台同时生效 —— 而且"这个实例死得不正常"
   *   这件事**内核最清楚**（它就在故障现场），监督者只能从事件里推断。
   *
   * ⚠️ 语义上要留意：这个函数的名字是"notify"，却带了副作用。
   *    刻意的 —— 作废状态必须与故障事件同生共死，分开写迟早会漏一处。
   */

  if (victim > 0)
    {
      FAR struct tcb_s *vtcb = nxsched_get_tcb(victim);

      if (vtcb != NULL && vtcb->group != NULL)
        {
          ort_state_invalidate((int)vtcb->group->tg_ort_domain - 1);
        }
    }

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

#ifdef CONFIG_ORT_SUPERVISOR_RESET
  /* ⚠️ 仅原型测试：信号总开关（对照实验用）。见 g_faultq_nosig 的说明。
   *
   * 注意它**只**掐掉通知这一步 —— 上面的入队、计数、丢最旧全都照常走，
   * 所以队列那一侧的负载一点都不少。 */

  if (g_faultq_nosig)
    {
      return;
    }
#endif

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

#ifdef CONFIG_ORT_SUPERVISOR_RESET
/****************************************************************************
 * Name: ort_fault_inject
 *
 * Description:
 *   测试注入点：一口气产生 count 条故障事件，**绕过容器重启**。
 *
 *   ★ 为什么需要它（§三·补三十二·补 §7）：
 *
 *   验收故障队列的竞争，缺的不是"让分支被执行"，而是**速率**。
 *   真实故障率卡在重启路径上（重启一次 = spawn ELF + 等准入 ≈ 2 秒），
 *   而监督者每 5 ms 排空一次 —— 生产者比消费者慢 300 倍，
 *   两者几乎不可能同时待在临界区里，所以 R1/R3 这类竞争
 *   **碰不到**，而"碰不到"看起来和"不存在"完全一样。
 *
 *   这里在**一次系统调用里**连续产生 count 条事件：监督者此刻正阻塞
 *   在这个调用上，根本没机会排空 —— 于是 16 格的环**必然溢出**，
 *   溢出/丢弃/记账那条从来没被走到的路，这一次必然被走到。
 *
 *   ⚠️ 只在 CONFIG_ORT_SUPERVISOR_RESET（仅原型）下编译进来 ——
 *      产品构建里不存在这个符号。与监督者槽位复位同一个门。
 *
 *   victim 用 0：内核查不到对应的 tcb，于是不会去作废谁的状态槽 ——
 *   这条路径**只压事件队列**，不扰动状态机。
 *
 ****************************************************************************/

int ort_fault_inject(int count)
{
  int i;

  /* ★ 权限：**任何任务**都能注入 —— 与其它接口"只有监督者"的规矩相反。
   *
   *   为什么这里要反着来：验收 R3（写者改读者游标）与 R2/R5（读到
   *   写了一半的记录）需要**生产者与消费者同时**在临界区里。
   *   如果注入者只能是监督者，那它注入时消费者必然阻塞着 ——
   *   两条路根本不会交错，也就永远验不到。
   *
   *   所以注入者必须是**另一个任务**。这在一个"仅原型"的门
   *   （CONFIG_ORT_SUPERVISOR_RESET）后面，产品构建里这个符号不存在。 */

  if (nxsched_self() == NULL)
    {
      return -EPERM;
    }

  if (count < 1 || count > 100000)
    {
      return -EINVAL;
    }

  for (i = 0; i < count; i++)
    {
      ort_fault_notify(0, 0xdead0000u + (uintptr_t)i,
                       0xbeef0000u + (uintptr_t)i, (uint32_t)(i + 1));
    }

  return count;
}

/****************************************************************************
 * Name: ort_fault_signal_set
 *
 * Description:
 *   测试对照开关：开关故障通知信号（`nxsig_queue`）的投递。
 *
 *   ★ 为什么必须做成**运行期**开关，而不是编译期宏：
 *     两次对照要用**同一个二进制**。否则"改了个编译选项"和
 *     "改了个变量"分不开 —— 那正是这个项目反复栽的那一类。
 *
 *   ★ 关掉之后队列那一侧的负载**一点没少**：照常入队、照常计数、
 *     照常丢最旧，只有 `nxsig_queue` 不再被调用。所以它把
 *     "ORT 故障路径（队列）"和"百万级信号投递"干净地分开。
 *
 *   返回开关生效后的状态：1 = 信号开，0 = 信号关。
 ****************************************************************************/

int ort_fault_signal_set(int signals)
{
  g_faultq_nosig = (signals == 0);

  _alert("ORT: fault signal %s by pid=%d "
         "(PROTOTYPE ONLY — 对照实验用)\n",
         g_faultq_nosig ? "DISABLED" : "enabled", nxsched_self()->pid);

  return g_faultq_nosig ? 0 : 1;
}
#endif /* CONFIG_ORT_SUPERVISOR_RESET */

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
