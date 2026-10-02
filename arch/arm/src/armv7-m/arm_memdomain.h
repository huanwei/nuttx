/****************************************************************************
 * arch/arm/src/armv7-m/arm_memdomain.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * [ORT] Per-task MPU memory domain support (PROTOTYPE)
 *
 * 目的：验证「上下文切换时重编程 MPU region」的可行性，并测量改动规模。
 *
 * 机制：
 *   - 预留两片用户 RAM 池区域（连续切块，每块 = 一个「域」）
 *   - 切换时把「即将运行」的任务的域块编成 user-RW，其余池区编成 no-access
 *   - ARMv7-M MPU 是「高编号 region 优先」，故：
 *       region_own  (高编号, 本任务块, user RW)  覆盖
 *       region_pool (低编号, 整个池, no-access)
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_ARMV7M_ARM_MEMDOMAIN_H
#define __ARCH_ARM_SRC_ARMV7M_ARM_MEMDOMAIN_H

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
 * 放在 uocram (0x60800000 + 8M) 的高段，避开内核/用户已有的堆栈区。
 * 每块必须是 2 的幂且按其大小对齐（MPU 硬约束）。
 */

#define ORT_DOMAIN_POOL_BASE    0x60840000u   /* 池起始（512KB 对齐）      */
#define ORT_DOMAIN_POOL_SIZE    (64 * 1024)   /* 池总大小 64KB             */
#define ORT_DOMAIN_BLOCK_SIZE   (16 * 1024)   /* 每域 16KB（2 的幂）       */
#define ORT_DOMAIN_COUNT        (ORT_DOMAIN_POOL_SIZE / ORT_DOMAIN_BLOCK_SIZE)

/* 返回值：无域 */

#define ORT_DOMAIN_NONE         (-1)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ort_memdomain_switch
 *
 * Description:
 *   上下文切换钩子：为即将运行的任务（to）编程 MPU region。
 *   必须在 arch 层真正切换之前调用（由 nxsched_switch_context() 调用）。
 *
 *   首次调用会完成惰性初始化（分配 region 编号）。
 *
 ****************************************************************************/

void ort_memdomain_switch(FAR struct tcb_s *to);


#endif /* CONFIG_ORT_MEMDOMAIN */
#endif /* __ARCH_ARM_SRC_ARMV7M_ARM_MEMDOMAIN_H */
