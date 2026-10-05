/****************************************************************************
 * include/sys/prctl.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __INCLUDE_SYS_PRCTL_H
#define __INCLUDE_SYS_PRCTL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <signal.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Supported prctl() commands.
 *
 *  PR_SET_NAME
 *    Set the name of the calling thread, using the value in the location
 *    pointed to by (char *) arg2. The name can be up to
 *    CONFIG_TASK_NAME_SIZE long, including the terminating null byte.
 *    (If the length of the string, including the terminating null byte,
 *    exceeds CONFIG_TASK_NAME_SIZE bytes, the string is silently truncated.)
 *    As an example:
 *
 *      prctl(PR_SET_NAME, "MyName");
 *
 *  PR_GET_NAME
 *    Return the name of the calling thread, in the buffer pointed to by
 *    (char *) arg2.  The buffer should allow space for up to
 *    CONFIG_TASK_NAME_SIZE bytes; the returned string will be
 *    null-terminated. As an example:
 *
 *      char myname[CONFIG_TASK_NAME_SIZE];
 *      prctl(PR_GET_NAME, myname);
 *
 *  PR_SET_NAME_EXT
 *    Set the task (or thread) name for the thread whose ID is in required
 *    arg2 (int), using the value in the location pointed to by required arg1
 *    (char*).  The name can be up to CONFIG_TASK_NAME_SIZE long (including
 *    any null termination).  The thread ID of 0 will set the name of the
 *    calling thread. As an example:
 *
 *      prctl(PR_SET_NAME_EXT, "MyName", pid);
 *
 *  PR_GET_NAME_EXT
 *    Return the task (or thread) name for the for the thread whose ID is
 *    optional arg2 (int), in the buffer pointed to by optional arg1
 *    (char *). The buffer must be CONFIG_TASK_NAME_SIZE long (including
 *    any null termination). As an example:
 *
 *      char myname[CONFIG_TASK_NAME_SIZE];
 *      prctl(PR_GET_NAME_EXT, myname, pid);
 */

#define PR_SET_NAME     1
#define PR_GET_NAME     2
#define PR_SET_NAME_EXT 3
#define PR_GET_NAME_EXT 4

#define PR_SET_DUMPABLE 5
#define PR_GET_DUMPABLE 6

/* [ORT] 把指定容器绑定到 MPU 内存域（prototype）
 *
 *   prctl(PR_SET_ORT_DOMAIN, int domain, pid_t pid);
 *       domain < 0  → 解除绑定
 *       pid         → 目标容器中的任一线程
 *
 * ★ 域是**容器级**的（task_group_s）：同一容器的所有线程共享一个域。
 * ★ 只有 ORT 监督者能调用；其它调用者返回 -EPERM。
 *   容器不能自己申报域 —— 域号就是内存块号，能自选就能选到别人的块。
 */
#define PR_SET_ORT_DOMAIN 7

/* [ORT] 查询本容器绑定的域（prototype）
 *
 *   prctl(PR_GET_ORT_DOMAIN);
 *       返回域号；未绑定返回 -1
 *
 * 容器用它等待「准入」：容器创建与监督者绑域之间存在窗口，
 * 在绑好之前容器不应碰任何受控内存。
 */
#define PR_GET_ORT_DOMAIN 10

/* [ORT] 注册 ORT 监督者：内核在容器故障时向该任务发信号（prototype）
 *
 *   prctl(PR_SET_ORT_SUPERVISOR);
 *
 * 只接受首次注册（单槽）。重复注册返回 -EBUSY。
 */
#define PR_SET_ORT_SUPERVISOR 8

/* [ORT] 释放监督者槽位（⚠️ 仅原型测试，需 CONFIG_ORT_SUPERVISOR_RESET）
 *
 *   prctl(PR_ORT_SUPERVISOR_RESET);
 *
 * 监督者槽位是**钉住**的：一旦注册，别的任务永远不能接管 ——
 * 否则"谁能当监督者"就成了运行期竞争。
 *
 * 正式产品必须关闭该配置项：复位应当走「授权复位」
 * （《降级状态机设计》§2.1），而不是一个任何任务都能调的 prctl。
 */
#define PR_ORT_SUPERVISOR_RESET 11

/* [ORT] 取下一条未读的容器故障记录（prototype）
 *
 *   struct ort_faultrec_s rec;
 *   int n = prctl(PR_GET_ORT_FAULT, &rec);
 *       n > 0  取到一条，n 是它的 seq
 *       n == 0 暂无新事件
 *       n < 0  错误（-EPERM = 调用者不是监督者）
 *
 * ★ 是**队列**不是单槽：并发故障时单槽会丢事件、且无法把
 *   "受害 pid"和"故障详情"正确配对。环形缓冲 16 条，
 *   溢出丢最旧的，丢弃条数记在每条的 lost 字段里。
 *
 * ★ 只有监督者能读：故障记录是监督者的私有视图。
 *   放开读会让容器能消费掉监督者的事件、或窥探别的容器的故障地址。
 */
#define PR_GET_ORT_FAULT 9

/* [ORT] 查询本平台的 ORT 能力位（准入检查用）
 *
 *   uint32_t caps = prctl(PR_GET_ORT_CAPS);
 *       >= 0  能力位掩码（ORT_CAP_*）
 *       <  0  错误
 *
 * ★ 直接返回掩码，不写用户指针。
 *   对比 PR_GET_ORT_FAULT 要往用户地址写结构体 —— 那个接口现在还欠着
 *   "用户指针未校验"的债（见 task_prctl.c 里的说明）。
 *   能力位是纯标量，没必要再引入一个待校验的指针。
 *
 * ★ 不需要权限：能力位是平台属性，不是谁的私有信息。
 *   容器自己也可能需要知道 —— 它要据此决定要不要装故障处理器。
 */
#define PR_GET_ORT_CAPS 12

/* [ORT] 能力位定义
 *
 * 每一个位都对应一条**已实测**的行为差异，不是设计愿望。
 * 加新位时必须附上跨平台的对照实验，否则它会慢慢变成谎言。
 */

/* 容器可以注册自己的故障处理器，并在故障发生时被通知到。
 *
 * 未置位时：容器注册的处理器**不会被执行** ——
 *   内核识别出处理器存在后，直接升级到 SIGKILL 终止容器。
 *   （不是"处理器被跳过但还有别的手段"，是真的没有任何通知机会；
 *     监督者仍会通过 ORT_SIGFAULT 独立通道收到事件。）
 *
 * 已置位的平台上，处理器会被调用，但容器**仍然要死**：
 *   处理器返回后会再踩同一条故障指令，内核随即升级 SIGKILL。
 *   即：处理器是"通知"，不是"恢复"。
 */
#define ORT_CAP_FAULT_HANDLER   (1u << 0)

/* [ORT] 容器状态槽：把状态交给接替者（对抗 H32 的"状态不延续"）
 *
 *   prctl(PR_ORT_STATE_PUT, buf, len);   n = prctl(PR_ORT_STATE_GET, buf, cap);
 *       PUT: 返回 OK / 负 errno
 *       GET: 返回实际读到的字节数；-ENOENT = 没有旧状态可接续
 *
 * ★ 只能读写**自己域**那一格，域号由监督者绑定、容器改不了 ——
 *   所以容器无法窥探别的容器的状态。
 *
 * ★ GET 的 -ENOENT 必须与"读到 0 字节"区分开：
 *   "接续了旧状态"和"没有旧状态"是两件不同的事，
 *   混起来正是 H31 / H32 那类错误的温床。
 *
 * ⚠️ 原型限制：64 字节定长、无版本号、无校验。正式实现需要
 *   按容器配额定大小、带 seq + 校验（识别撕裂的快照）、
 *   以及跨版本兼容（新旧实例的结构体可能不同）。
 */
#define PR_ORT_STATE_PUT 13
#define PR_ORT_STATE_GET 14

/* [ORT] 查询某个域的状态发布次数（**只有监督者**能调）
 *
 *   n = prctl(PR_GET_ORT_STATE_SEQ, int domain);
 *       n > 0  该域发布过 n 次
 *       n == 0 从未发布（或故障后被作废）
 *       n < 0  错误（-EPERM = 调用者不是监督者）
 *
 * ★ 存在的理由：manifest 里的 `protocol = 1` 是**声明**，不是事实。
 *   "这个容器到底实现了状态发布没有" —— 容器自己说了不算
 *   （公理 S1：不信任失效组件），监督者也读不到它的内存。
 *
 *   但内核知道：那个域的槽被写过没有。槽按域索引、只有该域的容器
 *   能写 —— 于是这件事变成**监督者单方面可判定**的。
 *
 *   两条反向证伪的判据（见 ortsup 的 audit_protocol）：
 *     1. 健康运行超过启动窗口，槽仍然是空的 → 没实现发布；
 *     2. 撤下它时走了 SIGKILL 兜底 → 没实现 SIGTERM 有界退出。
 *   任一为真 → 声明不可信 → 替换方式降级为冷替换，并明示。
 *
 * ★ 为什么不给容器用：容器知道自己的发布次数，读别人的没有正当用途，
 *   只是多一条旁路。
 */
#define PR_GET_ORT_STATE_SEQ 15

/* [ORT] 部署/O&M 代理：把配置递送从实时控制环里切出去
 *
 *   prctl(PR_SET_ORT_DEPLOY);                     代理注册自己（钉住，同监督者规则）
 *   prctl(PR_ORT_CFG_PUT, const char *buf, size_t len);   代理写入原样字节
 *   prctl(PR_ORT_CFG_ALIVE);                      代理心跳（每跑一圈一次）
 *   n = prctl(PR_GET_ORT_CFG_SEQ);                监督者读**代数**（每个控制周期一次）
 *   n = prctl(PR_GET_ORT_CFG_TICK);               监督者读**心跳**（代理失联检测）
 *   n = prctl(PR_ORT_CFG_GET, char *buf, size_t cap);     监督者取回快照
 *
 * ★ 为什么要切（设计见 proposals/《部署与 O&M 组件设计》）：
 *
 *   在此之前监督者每 2 秒自己 fopen manifest 并逐行解析 ——
 *   也就是**实时控制环里做文件 I/O**。硬实时关心的是最坏耗时，
 *   而 hostfs / flash 的最坏延迟都不可控。5 ms 周期的控制循环
 *   不能建立在"读文件很快"这个假设上。
 *
 *   切分后：代理（非实时约束）负责搬，监督者每周期只做一次
 *   **整数比较**；变了才取回快照（有界 memcpy）并**自己校验**。
 *   控制循环里因此不存在任何不可控的最坏耗时。
 *
 * ★ 信任模型：**代理只搬运，校验权在监督者**。
 *   代理是非实时、可重启、可能被降级的组件 —— 按公理 S1，
 *   它的输出只能当**输入**看待。所以内核侧不做任何格式校验，
 *   只存字节；解析与校验仍然只有监督者那一份。
 *
 * ★ 权限：容器调这四个接口一律 -EPERM。否则一个被攻陷的容器可以
 *   给自己放宽 max_restarts、或把别的 CG 的 critical 改成 false。
 *
 * ★ 代数（generation）与心跳（tick）是**两个**计数器，而且
 *   **心跳必须与"写配置"分开**：
 *     generation —— 只在**内容真变了**时加（内核做比较，代理侧无状态）；
 *     tick       —— 代理每跑一圈加一次，与内容无关。
 *   混成一个的话，"代理死了"和"配置本来就不用变"看起来一模一样 ——
 *   而后者是正常状态。那正是 H31 那一族。
 *   但反过来，把心跳挂在"写配置"上的话，**代理读不到源文件**（文件被移走、
 *   介质出错）这种"活着但没东西可写"的情形会被报成"失联" ——
 *   那是一条不实的告警。告警必须只由它真正想表达的事实触发。
 *
 * ⚠️ PR_ORT_CFG_GET 要往用户地址写（槽有 4 KB，塞不进返回值）——
 *    与 PR_GET_ORT_FAULT / PR_ORT_STATE_* 同一个原型债：
 *    内核直接按用户指针写，**没有做指针合法性校验**。
 */
#define PR_SET_ORT_DEPLOY    16
#define PR_ORT_DEPLOY_RESET  21   /* ⚠️ 仅原型测试，同 PR_ORT_SUPERVISOR_RESET */

/* [ORT] ⚠️ **仅原型测试**：一口气注入 count 条故障事件（绕过容器重启）
 *
 *   n = prctl(PR_ORT_TEST_FAULT, int count);
 *
 * ★ 存在的理由：验收故障事件队列的竞争，缺的是**速率**。
 *   真实故障率卡在重启路径上（≈2 秒一次），而监督者每 5 ms 排空 ——
 *   生产者比消费者慢 300 倍，两者几乎不可能同时进临界区，
 *   于是"碰不到"和"不存在"看起来一模一样。
 *
 *   一次调用里连续产生 count 条 → 监督者此刻正阻塞在这个调用上，
 *   没机会排空 → 16 格的环**必然溢出** → 溢出那条路必然被走到。
 *
 * ⚠️ 与 PR_ORT_SUPERVISOR_RESET 同一个门（CONFIG_ORT_SUPERVISOR_RESET）：
 *    产品构建里不存在这个符号。count 上限 100000。
 */
#define PR_ORT_TEST_FAULT    23

/* [ORT] ⚠️ **仅原型测试**：开关故障通知信号（对照实验用）
 *
 *   n = prctl(PR_ORT_TEST_SIGNOFF, int signals);
 *       signals != 0 → 打开投递；== 0 → 关闭投递
 *       返回生效后的状态（1 = 开，0 = 关）
 *
 * ★ 存在的理由：压故障队列的装置会让内核 assert
 *   （`irq/irq_csection.c:205`）。但那个装置同时压了三样东西 ——
 *   队列计数器/环、**每条事件一次 nxsig_queue**、以及百万级 prctl 往返。
 *   不把它们分开，"是什么压垮了内核"就只能靠猜。
 *
 *   关掉信号后，队列那一侧的负载**一点没少**（照常入队、照常计数、
 *   照常丢最旧），所以这一刀恰好切在"队列逻辑"和"信号投递"之间。
 *
 * ★ 必须是**运行期**开关：两次对照要跑同一个二进制，
 *   否则"改了编译"和"改了变量"分不开。
 *
 * ⚠️ 与 PR_ORT_SUPERVISOR_RESET 同一个门；产品构建里不存在。
 */
#define PR_ORT_TEST_SIGNOFF  24
#define PR_ORT_CFG_ALIVE     22   /* 代理心跳：与内容变没变无关 */
#define PR_ORT_CFG_PUT       17
#define PR_GET_ORT_CFG_SEQ   18
#define PR_GET_ORT_CFG_TICK  19
#define PR_ORT_CFG_GET       20

/* [ORT] 容器故障通知信号
 *
 * 为什么用 SIGUSR1：CONFIG_SIG_SIGUSR1_ACTION 默认为 n，
 * 内核**不会**给它配默认动作 —— 只有主动 sigaction() 挂钩子的监督者
 * 才会收到，普通任务不受影响（也不会被误杀）。
 *
 * ⚠️ 原型期硬编码。正式实现应做成可配置，或改用实时信号（可靠队列）。
 */
#define ORT_SIGFAULT SIGUSR1

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* [ORT] 容器故障记录（prototype）
 *
 * ⚠️ 这是内核与监督者之间的**原型期**接口，不是最终形态：
 *    - 16 格的环形缓冲（不是单槽），溢出时丢**最旧**的一条；
 *      丢了多少记在 `lost` 里
 *    - seq 全局递增且**唯一**，监督者据此判断自己是否漏了事件
 *    - 写入方是**异常上下文**，所以内部不能用锁，也不能阻塞
 *      （改法见 arch/arm/src/common/arm_ortcommon.c 里的说明）
 *
 * ⚠️ 仍然欠着的债：内核直接按**未校验的用户指针**写这个结构体
 *    （与 PR_ORT_STATE_* / PR_ORT_CFG_GET 同一笔）。
 *    正式实现必须校验，或改成只读的内部缓冲 + 文件接口。
 *
 * （本条原先写的是"只有一条单槽记录" —— 那在实现改成环之后就过期了。
 *   接口文档漂移比代码漂移更坏：调用者会按旧语义用它。） */

struct ort_faultrec_s
{
  uint32_t  seq;      /* 递增序号；从 1 开始 */
  uint32_t  lost;     /* 本条之前被丢弃的条数；0 = 无丢失 —— 见下 */
  int       victim;   /* 事件主体（容器组）的 pid */
  uintptr_t pc;       /* FAULT：触发故障的指令地址；EXIT 恒 0 */
  uintptr_t addr;     /* FAULT：被非法访问的地址；EXIT 恒 0 */
  uint32_t  faults;   /* FAULT：该容器累计故障次数；EXIT 恒 0 */
  uint32_t  kind;     /* 事件类型：0 = FAULT（故障）；1 = EXIT（正常退出） */
  int32_t   code;     /* EXIT：进程退出码；FAULT 恒 0 */
};

/* ★ kind 的语义（2026-10-05 加，手册 §三·补四十九）：
 *
 *   kind=0 FAULT —— 原有语义，不变。
 *   kind=1 EXIT  —— **没有故障史**的容器组走到终点（_exit / 主动退出）。
 *                   有故障史的组由 FAULT 事件收尾，不再补发 EXIT
 *                   （重启/降级逻辑由 FAULT 驱动，重复上报只会是噪声）。
 *
 *   ★ 两种事件共用**同一个队列与同一个序号空间** —— 监督者的
 *     连续性/缺口记账对全部事件一体生效，不需要第二套审计。
 *   ⚠️ 本结构是 ABI：扩字段 = 内核与所有读者一起重编（ortsup/orttest
 *     都源码共编，无版本兼容问题；将来若出现**独立固件读者**，
 *      需要先加 version 字段 —— 记在这里，别到时候才发现）。 */

/* ★ `lost` 是**由内核的读者侧填的**，不是由生产者填的（2026-10-04 改）。
 *
 *   原因：生产者数不准。消费者取事件时游标会**跳**（它落到环里最新的
 *   那条），一条被跳过的记录在生产者眼里却是"已消费" —— 它的账会偏小，
 *   而那是**危险的方向**：报告丢得少，监督者会以为事件是连着的。
 *   实测在单生产者对照里就偏了，而对照里丢更新物理上不可能，
 *   所以偏差只能来自"生产者来数"这个做法本身。
 *
 *   "跳过"是**消费者自己做的动作**，只有它数得准，所以改由它累加。
 *
 * ⚠️ 这不改变调用者看到的东西：仍然是"本条之前丢了多少"。
 *    改的是**谁来算**。记录在返回给调用者之前就已经填好了。
 *
 * ★ 可验证的恒等式（实测逐条精确）：
 *       最后一条的 lost  ==  产出总数 − 调用者读到的条数
 *   它同时要求"产出没丢"和"丢弃记账没丢"，而且两者互相吻合到个位数。
 *   验收脚本见手册 §三·补三十二·十补。 */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: prctl
 *
 * Description:
 *   prctl() is called with a first argument describing what to do (with
 *   values PR_* defined above) and with additional arguments depending on
 *   the specific command.
 *
 * Returned Value:
 *   The returned value may depend on the specific command.  For PR_SET_NAME
 *   and PR_GET_NAME, the returned value of 0 indicates successful operation.
 *   On any failure, -1 is retruend and the errno value is set appropriately.
 *
 *     EINVAL The value of 'option' is not recognized.
 *     EFAULT optional arg1 is not a valid address.
 *     ESRCH  No task/thread can be found corresponding to that specified
 *       by optional arg1.
 *
 ****************************************************************************/

int prctl(int option, ...);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_SYS_PRCTL_H */
