/****************************************************************************
 * arch/arm/src/armv8-m/arm_memdomain.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * [ORT] Per-container MPU memory domain support (PROTOTYPE, PMSAv8)
 *
 * ★ 与 armv7-m 版本的核心差异：
 *   PMSAv8 的 region **不允许重叠**（ARM ARM: overlapping regions give
 *   UNPREDICTABLE behaviour）。armv7-m 上那套
 *     region_own  (高编号, 本容器块, user RW)  覆盖
 *     region_pool (低编号, 整个池, no-access)
 *   在 PMSAv8 上不能用。
 *
 *   改用**单 region**：
 *     已绑定 → region 指向本容器的块（user RW）
 *     未绑定 → region 缩成一小块且 AP=RWNO（用户态不可访问）
 *   其余地址不被任何 region 覆盖 —— 非特权访问一律 fault
 *   （MPU_CTRL.PRIVDEFENA 只对**特权**访问回退到默认内存映射）。
 *
 *   结果比 armv7-m 版本更简单：不需要"池 region"，每次切换一次
 *   mpu_modify_region()。
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_ARMV8M_ARM_MEMDOMAIN_H
#define __ARCH_ARM_SRC_ARMV8M_ARM_MEMDOMAIN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_ORT_MEMDOMAIN

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* 域池：一段专用的用户 RAM，按域切块
 *
 * 放在 uocram 的高段（与 mps2-an500 的 ORT-M 原型同一地址，
 * an547 的 uocram 也是 0x60800000 起 8M）。
 *
 * ⚠️ PMSAv8 对块大小**没有 2 的幂要求**（limit = base + size - 1），
 *    但 base 必须按 size 对齐。保持 16KB 只是与 armv7-m 版本一致。
 */

/* 池布局由 Kconfig 决定（§三·补六十：换板必须重设且避开用户堆）。
 * 默认值 = mps 两板的既有布局，行为零漂移。 */

#define ORT_DOMAIN_POOL_BASE    ((uintptr_t)CONFIG_ORT_DOMAIN_POOL_BASE)
#define ORT_DOMAIN_POOL_SIZE    (CONFIG_ORT_DOMAIN_POOL_SIZE)
#define ORT_DOMAIN_BLOCK_SIZE   (CONFIG_ORT_DOMAIN_BLOCK_SIZE)
#define ORT_DOMAIN_COUNT        (ORT_DOMAIN_POOL_SIZE / ORT_DOMAIN_BLOCK_SIZE)

/* 「不可访问」region 的最小尺寸（PMSAv8 对齐要求：base 按 size 对齐） */

#define ORT_DOMAIN_DENY_SIZE    32

/* 域绑定值的编码（存在 group->tg_ort_domain）：
 *
 *     0       = 未绑定
 *     n (>0)  = 域 (n - 1)
 *
 * 与 armv7-m 版本同样的理由：task_group_s 由 kmm_zalloc() 分配、
 * g_kthread_group 是 BSS，0 是唯一天然初值。用 -1 哨兵会 fail-open。
 */

#define ORT_DOMAIN_UNBOUND      0
#define ORT_DOMAIN_ENCODE(d)    ((d) + 1)
#define ORT_DOMAIN_DECODE(v)    ((v) - 1)
#define ORT_DOMAIN_VALID(v)     ((v) > ORT_DOMAIN_UNBOUND && \
                                 (v) <= ORT_DOMAIN_ENCODE(ORT_DOMAIN_COUNT - 1))

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct tcb_s;

void ort_memdomain_switch(FAR struct tcb_s *to);

void ort_fault_notify(pid_t victim, uintptr_t pc, uintptr_t addr,
                      uint32_t faults);

#endif /* CONFIG_ORT_MEMDOMAIN */
#endif /* __ARCH_ARM_SRC_ARMV8M_ARM_MEMDOMAIN_H */
