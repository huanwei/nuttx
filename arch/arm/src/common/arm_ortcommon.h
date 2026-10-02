/****************************************************************************
 * arch/arm/src/common/arm_ort.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * [ORT] ARM 各架构共用的内部接口
 *
 * 这里放**与 MPU/MMU 无关**的 ORT 内核侧机制：
 *   - 监督者槽位（钉住语义）
 *   - 故障事件队列
 *
 * 为什么单独抽出来：armv7-m / armv8-m / armv7-a 三份拷贝迟早会漂移，
 * 而且这三样东西的语义本来就与隔离机制无关。
 *
 * 各架构自己保留的是：域编码、region/页表操作、故障检测。
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_COMMON_ARM_ORTCOMMON_H
#define __ARCH_ARM_SRC_COMMON_ARM_ORTCOMMON_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_ORT_CONTAINER

#include <sys/types.h>
#include <sys/prctl.h>

struct tcb_s;

/* 记录一次容器故障并通知监督者。从异常处理上下文调用（不可阻塞）。 */

void ort_fault_notify(pid_t victim, uintptr_t pc, uintptr_t addr,
                      uint32_t faults);

/* 取下一条未读事件。>0 = seq；0 = 暂无；-EPERM = 调用者不是监督者 */

int ort_fault_read(FAR struct ort_faultrec_s *rec);

/* 监督者槽位 */

int ort_supervisor_set(pid_t pid);

#ifdef CONFIG_ORT_SUPERVISOR_RESET
int ort_supervisor_reset(void);
#endif

/* 当前监督者 pid（-1 = 无）。给各架构的故障分支用。 */

pid_t ort_supervisor_pid(void);

#endif /* CONFIG_ORT_CONTAINER */
#endif /* __ARCH_ARM_SRC_COMMON_ARM_ORTCOMMON_H */
