/****************************************************************************
 * arch/arm/src/armv8-m/arm_memdomain.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * [ORT] Per-container MPU memory domain support (PROTOTYPE, PMSAv8)
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

/* ── 内核 → 监督者的故障通道 ────────────────────────────────────────────
 *   与 armv7-m 版本同构，见那边的说明。这里只重复一句最关键的理由：
 *   故障任务自己注册的 SIGSEGV 处理器是**容器可控**的，
 *   所以"知道出事了"必须走独立通道。
 */

static pid_t g_supervisor = -1;
static bool  g_supervisor_pinned;

#define ORT_FAULTQ_SIZE  16

static struct ort_faultrec_s g_faultq[ORT_FAULTQ_SIZE];
static uint32_t g_faultq_total;
static uint32_t g_faultq_read;
static uint32_t g_faultq_dropped;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void ort_memdomain_lazyinit(void)
{
  /* 向 MPU 申请一个 region 编号，避免与板级/用户态已用的冲突 */

  g_own_region = (int)mpu_allocregion();

  /* 先编成"不可访问"，直到有容器被绑定 */

  mpu_modify_region((unsigned int)g_own_region,
                    ORT_DOMAIN_POOL_BASE, ORT_DOMAIN_DENY_SIZE,
                    ORT_DENY_FLAGS1, ORT_DENY_FLAGS2);

  syslog(LOG_INFO, "[ORT] memdomain(armv8-m) init: own_region=%d\n",
         g_own_region);
}

/****************************************************************************
 * Public Functions
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

  /* 只有监督者能绑域 —— 域号就是内存块号，容器能自选就能选到别人的块 */

  if (nxsched_self()->pid != g_supervisor)
    {
      return -EPERM;
    }

  flags = enter_critical_section();

  tcb = nxsched_get_tcb(pid);
  if (tcb == NULL || tcb->group == NULL)
    {
      leave_critical_section(flags);
      return -ESRCH;
    }

  group = tcb->group;

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

int ort_supervisor_set(pid_t pid)
{
  if (g_supervisor_pinned)
    {
      /* 已钉住：只接受同一任务的重复注册，其它一律拒绝 ——
       * **无论原监督者是否还活着**。理由见 armv7-m 版本的长注释。
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

  /* 新一任监督者上任 → 清空事件队列（旧事件已无正确处理者） */

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

  {
    uint32_t pending = g_faultq_total - g_faultq_read;
    uint32_t slot;

    if (pending >= ORT_FAULTQ_SIZE)
      {
        /* 队列满 → 丢最旧的（监督者要处理的是"现在出了什么事"） */

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

  if (g_supervisor < 0 || g_supervisor == victim)
    {
      return;
    }

  value.sival_int = (int)victim;

  ret = nxsig_queue(g_supervisor, ORT_SIGFAULT, value);
  if (ret < 0)
    {
      /* ★ 不清槽：清槽会把安全事件变成接管机会。
       * 持续告警才是对的 —— 那是"降级能力已失效"的事实。
       */

      _alert("ORT: fault notify to supervisor %d failed: %d "
             "(降级能力失效，槽位保持钉住)\n", g_supervisor, ret);
    }
}

int ort_fault_read(FAR struct ort_faultrec_s *rec)
{
  uint32_t pending;

  /* 只有监督者能读：故障记录是监督者的私有视图 */

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
      return 0;
    }

  *rec = g_faultq[g_faultq_read % ORT_FAULTQ_SIZE];
  g_faultq_read++;

  return (int)rec->seq;
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

  if (g_own_region < 0)
    {
      ort_memdomain_lazyinit();
    }

  group = to != NULL ? to->group : NULL;
  bound = group != NULL ? group->tg_ort_domain : ORT_DOMAIN_UNBOUND;

  /* 默认拒绝：未绑定 / 越界 → 不给任何域块。
   * 必须用 ORT_DOMAIN_VALID() 而不是「>= 0」，理由见 armv7-m 版本（H28）。
   */

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
