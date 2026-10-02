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

/* 域绑定值的编码（存在 tcb->xcp.domain_id）：
 *
 *     0       = 未绑定
 *     n (>0)  = 域 (n - 1)
 *
 * ★ 为什么用「0 = 未绑定」而不是「-1 = 未绑定」哨兵：
 *   TCB 全部来自 kmm_zalloc()（task_spawn.c / task_create.c）或
 *   memset(0)（g_idletcb），BSS 清零是唯一天然存在的默认值。
 *   若用 -1 哨兵，任何一条 TCB 创建路径（fork / 内核线程 / idle）
 *   漏设哨兵，该任务就会落到「域 0」—— 隔离 fail-open。
 *   这不是假设：实测未绑定任务能写 0x60840000，见 H28。
 *
 *   语义上也更自然：域绑定是内核「交给」任务的一个动作，
 *   在动作发生之前任务本来就是未绑定的。
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

/* 注：ort_memdomain_bind() / ort_supervisor_set() / ort_fault_record()
 * 需要被 sched/ 调用，声明放在 arch/arm/include/armv7-m/irq.h
 * （跟着字段走），这里不重复。
 */

/****************************************************************************
 * Name: ort_fault_notify
 *
 * Description:
 *   记录一次容器故障并通知监督者。从 MemManage 异常处理上下文调用。
 *
 ****************************************************************************/

void ort_fault_notify(pid_t victim, uintptr_t pc, uintptr_t addr,
                      uint32_t faults);

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
