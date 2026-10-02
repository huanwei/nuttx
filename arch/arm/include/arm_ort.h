/****************************************************************************
 * arch/arm/include/arm_ort.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * [ORT] 序驰 OrdinRT：ARM 侧共享接口
 *
 * 为什么单独一个头文件：
 *   这些函数要由 sched/ 调用（prctl），但实现分属 armv7-m 与 armv8-m。
 *   两边各写一份声明迟早会漂移，所以放在这里，两个 arch 的 irq.h 都包含它。
 *
 * ⚠️ 调用者不需要知道域是怎么编码的、也不需要知道谁是监督者 ——
 *    这些判断都在实现里（arch/arm/src/armv7-m|armv8-m/arm_memdomain.c）。
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_INCLUDE_ARM_ORT_H
#define __ARCH_ARM_INCLUDE_ARM_ORT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_ORT_CONTAINER

#ifndef __ASSEMBLY__

#include <sys/types.h>
#include <sys/prctl.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct task_group_s;

/* 把指定容器绑定到域。只有监督者能调用，否则 -EPERM。 */

int ort_container_bind(pid_t pid, int domain);

/* 查询容器的域。未绑定返回 -1。 */

int ort_container_domain(FAR struct task_group_s *group);

/* 监督者槽位：首次注册即钉住，之后别的任务一律 -EBUSY。 */

int ort_supervisor_set(pid_t pid);

#ifdef CONFIG_ORT_SUPERVISOR_RESET
int ort_supervisor_reset(void);
#endif

/* 故障事件队列：取下一条未读。>0 = seq；0 = 暂无；-EPERM = 不是监督者 */

int ort_fault_read(FAR struct ort_faultrec_s *rec);

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_ORT_CONTAINER */
#endif /* __ARCH_ARM_INCLUDE_ARM_ORT_H */
