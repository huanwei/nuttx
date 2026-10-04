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

/* 作废某个域的状态槽（域号，不是编码值；-1 表示未绑定，会被忽略）。
 * 由 ort_fault_notify() 在容器故障时调用 —— 故障重启必须是冷启动。 */

void ort_state_invalidate(int domain);

/* [ORT] 容器故障的统一处置结果（M 侧 memfault / usagefault 共用）
 *
 * ★ 为什么要有这个枚举：处置逻辑（通知监督者 → SIGSEGV → 升级 SIGKILL）
 *   原先只长在 arm_memfault.c 里。undefined instruction 那条路需要
 *   一模一样的处置 —— 抄一份必然漂移，而"两处判据不一致"正是这条债
 *   本来的成因（三条故障向量里只有一条被漏掉）。
 *
 * 调用者**必须先自己判定**"这是用户态故障"（判据两边一致：
 * 故障 PC 落在 USERSPACE->us_textstart..us_textend），再调这里。
 * panic 由调用者做 —— PANIC_WITH_REGS 是各架构自己的宏，
 * common/ 里拿不到。
 */

enum ort_fault_action_e
{
  ORT_FAULT_CONTAINED = 0,   /* 已交给信号路径终止该任务 → 正常异常返回 */
  ORT_FAULT_NO_CONTAINER,    /* 找不到容器 → fail-stop */
  ORT_FAULT_UNDELIVERABLE    /* 连 SIGKILL 都投不出去 → fail-stop */
};

enum ort_fault_action_e ort_contain_user_fault(FAR struct tcb_s *ftcb,
                                               uintptr_t pc, uintptr_t addr);

/* 监督者槽位 */

int ort_supervisor_set(pid_t pid);

#ifdef CONFIG_ORT_SUPERVISOR_RESET
int ort_supervisor_reset(void);
#endif

/* 当前监督者 pid（-1 = 无）。给各架构的故障分支用。 */

pid_t ort_supervisor_pid(void);

/* [ARMv7-A / MMU 专用] abort 处理器必须调这个版本，不能直接调
 * ort_handle_user_fault()。
 *
 * 原因见 armv7-a/arm_ort.c 里 ort_fault_on_kstack() 的注释：
 * KERNEL 构建下 abort 向量把寄存器帧建在用户栈上，而异常处理器的
 * 调用帧也在同一张栈上 —— up_schedule_sigaction() 的原地帧下移
 * 会踩掉处理器自己的返回地址。这个包装把终止路径换到内核栈上跑。
 */

#ifdef CONFIG_ORT_MMU
bool ort_handle_user_fault(uintptr_t pc, uintptr_t addr);
bool ort_handle_user_fault_kstack(FAR struct tcb_s *tcb,
                                  uintptr_t pc, uintptr_t addr);
#endif

#endif /* CONFIG_ORT_CONTAINER */
#endif /* __ARCH_ARM_SRC_COMMON_ARM_ORTCOMMON_H */
