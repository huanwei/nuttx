/****************************************************************************
 * boards/arm/mps/mps3-an547/scripts/memory.ld
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

/* Specify the memory areas
 * see arch/arm/src/mps/hardware/mps_memorymap.h for more details
 */

MEMORY
{

  kflash (rx)  : ORIGIN = 0x00000000, LENGTH = 4M
  uflash (rx)  : ORIGIN = 0x20000000, LENGTH = 4M

  kocram (rwx) : ORIGIN = 0x60000000, LENGTH = 8M
  uocram (rwx) : ORIGIN = 0x60800000, LENGTH = 8M
}

_ram_size = LENGTH(kocram) + LENGTH(uocram);

/* [ORT] 用户态 RAM 的末尾（用户堆一直长到这里）。
 *
 * 与 an500 / risc-v 侧（rv-virt、mpfs）的 __ld_usram_end 同名同义 ——
 * USERSPACE->us_heapend 要它。在此之前 an547 的 us_heapend 一直是 0：
 * 没有代码读它，所以没人发现；直到 ORT 用它划「用户态内存」的窗口
 * （实测：不设它，闸门会把容器的合法栈指针也拒掉）。
 *
 * ★ an500 上先发现的同一个缺口，当时只修了那一块板 —— 这就是
 *   "只修撞到的那一处"的代价。凡是 PROTECTED 的 mps 板都需要它。
 */

__ld_usram_end = ORIGIN(uocram) + LENGTH(uocram);
