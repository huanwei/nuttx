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
 *    - 只有一条单槽记录，会被后来的故障覆盖
 *    - seq 递增，监督者据此判断自己是否漏了事件
 *    - 正式实现应改为环形缓冲 / 事件队列，且需校验用户指针
 */

struct ort_faultrec_s
{
  uint32_t  seq;      /* 递增序号；从 1 开始 */
  uint32_t  lost;     /* 本条之前被丢弃的条数；0 = 无丢失 */
  int       victim;   /* 故障容器的 pid */
  uintptr_t pc;       /* 触发故障的指令地址 */
  uintptr_t addr;     /* 被非法访问的地址 */
  uint32_t  faults;   /* 该容器累计故障次数 */
};

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
