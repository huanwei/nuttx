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
#include <stddef.h>
#include <sys/prctl.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct task_group_s;

/* 把指定容器绑定到域。只有监督者能调用，否则 -EPERM。 */

int ort_container_bind(pid_t pid, int domain);

/* 查询容器的域。未绑定返回 -1。 */

int ort_container_domain(FAR struct task_group_s *group);

/* 监督者槽位：首次注册即钉住（且登录名需在构建期白名单内 —— 见
 * task_prctl.c 的资格检查），之后别的任务一律 -EBUSY。 */

int ort_supervisor_set(pid_t pid);

/* 容器组正常退出通知（kind=EXIT 事件）。
 *   pid        = 容器组 pid
 *   ort_domain = 组绑定的域（**原始编码**：tg_ort_domain，域 n 存为 n+1）
 *   code       = 退出码
 * 只有"已绑域且无故障史"的组会走到这里（调用点在 group_leave）。
 * 监督者未注册时静默丢弃 —— 与故障通知同规矩。 */

void ort_group_exit_notify(pid_t pid, uint8_t ort_domain, int code);

/* ③ 准入协议：阻塞等待监督者把**本组**绑好域（手册 §三·补五十）。
 *   返回 0 = 已准入；-ETIMEDOUT = 超时；其它负值 = errno。
 *   唤醒方 = PR_SET_ORT_DOMAIN 处理器（绑域成功即 post）。 */

int ort_wait_admission(unsigned timeout_ms);

/* ④ 就绪上报（手册 §三·补五十一）。
 *   ort_ready_set()        —— 容器侧：标记本组已就绪（OK）。
 *   ort_ready_get(pid)     —— 监督者侧：0/1；非监督者 -EPERM；
 *                             组不存在 0（"不存在"与"未上报"同义：
 *                             都推不出"已就绪"）。 */

int ort_ready_set(void);
int ort_ready_get(pid_t pid);

#ifdef CONFIG_ORT_SUPERVISOR_RESET
int ort_supervisor_reset(void);
#endif

/* 容器状态槽：只能读写**自己域**那一格。
 *
 * 现任周期性发布快照，接替者启动时读回 —— 让替换不再是冷启动，
 * 而不是"新实例什么都不知道"（见假设审计 H32）。
 * 设计与访问控制见 arch/arm/src/common/arm_ortcommon.c。
 *
 * GET 返回实际字节数；-ENOENT = 没有旧状态可接续。
 */

int ort_state_put(FAR const void *buf, size_t len);
int ort_state_get(FAR void *buf, size_t len);

/* 某个域发布过多少次。**只给监督者** —— 用于反向证伪容器的
 * `protocol` 声明（声明了会发布，但槽一直是空的）。
 * 返回发布次数；0 = 从未发布；负 = errno。 */

int ort_state_seq(int domain);

/* 部署/O&M 代理槽：首次注册即钉住（与监督者同规则）。
 * 只有它能写配置槽 —— 容器写一律 -EPERM。 */

int ort_deploy_set(pid_t pid);

#ifdef CONFIG_ORT_SUPERVISOR_RESET
void ort_deploy_reset(void);
#endif

/* 配置槽。**代理只搬运，校验权在监督者** —— 这里存的是原样字节。
 *
 *   put  —— 代理写。generation 只在**内容真变了**时才加。
 *   alive—— 代理心跳，与内容无关。**必须与 put 分开**：代理读不到源文件时
 *           没有内容可写，但它还活着 —— 混在一起会造出不实的失联告警。
 *   seq  —— 监督者读代数：每个控制周期一次标量比较，无 I/O。
 *   tick —— 监督者读心跳：代理失联必须**可检测**（否则和"配置本来
 *           就不用变"看起来一模一样，见假设审计 H31）。
 *   get  —— 监督者取回快照，有界 memcpy。
 */

int ort_cfg_put(FAR const void *buf, size_t len);
int ort_cfg_alive(void);
int ort_cfg_seq(void);
int ort_cfg_tick(void);
int ort_cfg_get(FAR void *buf, size_t cap);

#ifdef CONFIG_ORT_SUPERVISOR_RESET
/* ⚠️ 测试注入点：一口气产生 count 条故障事件，绕过容器重启。
 * 为什么需要速率而不是"把环改小"，见 arm_ortcommon.c 里的长注释。
 * 产品构建里不存在。 */

int ort_fault_inject(int count);

/* ⚠️ 测试对照开关：开关故障通知信号（nxsig_queue）的投递。
 * 关掉后队列逻辑一字不改，只掐通知这一步 —— 用来把
 * "ORT 故障路径"和"百万级信号投递"分开归因。
 * 返回生效后的状态：1 = 开，0 = 关。 */

int ort_fault_signal_set(int signals);
#endif

/* 故障事件队列：取下一条未读。>0 = seq；0 = 暂无；-EPERM = 不是监督者 */

int ort_fault_read(FAR struct ort_faultrec_s *rec);

/* 本平台上 ORT 能提供哪些能力（ORT_CAP_* 位掩码）。
 *
 * ★ 为什么必须有这个东西：
 *
 *   ORT 的两个 SKU 在「容器能不能自己处理故障」上有**真实的能力差异**，
 *   而这个差异此前是**隐式**的 —— 同一份 manifest、同一个容器，
 *   在 ORT-M 上注册 SIGSEGV 处理器能收到通知，在 ORT-A 上却会被
 *   直接跳过、静默升级到 SIGKILL。部署方看到的是"两个 SKU 行为不同"，
 *   却没有任何地方能问出这件事。
 *
 *   与其让它隐式发生，不如让监督者在**准入**时就问清楚：
 *   容器声明了故障处理器而平台不支持 → 拒绝准入，而不是默默降级。
 *
 *   所以这个函数返回的必须是**实际被强制执行的行为**，
 *   不是"理论上想支持的行为" —— 它由各 arch 端口申报，
 *   因为约束就长在那边。
 */

uint32_t ort_caps(void);

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_ORT_CONTAINER */
#endif /* __ARCH_ARM_INCLUDE_ARM_ORT_H */
