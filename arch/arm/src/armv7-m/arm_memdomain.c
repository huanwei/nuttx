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
#include <debug.h>
#include <syslog.h>

#include <nuttx/sched.h>
#include <nuttx/arch.h>

#include "mpu.h"
#include "arm_memdomain.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* 域管理用的两个 MPU region 编号（惰性分配，-1 表示未初始化） */

static int g_pool_region = -1;   /* 整个池：no-access（低优先级） */
static int g_own_region  = -1;   /* 本任务块：user RW（高优先级） */

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
