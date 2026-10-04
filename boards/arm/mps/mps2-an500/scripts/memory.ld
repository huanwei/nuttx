/****************************************************************************
 * boards/arm/mps/mps2-an500/scripts/memory.ld
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
 * 与 risc-v 侧（rv-virt / mpfs）的 __ld_usram_end 同名同义 ——
 * USERSPACE->us_heapend 要它。在此之前 us_heapend 一直是 0：
 * 没有任何代码读它，所以没人发现；直到 ORT 用它划「用户态内存」的窗口。
 *
 * ⚠️ 注意 uocram 是**用户** RAM，ORT-M 的域池（0x60840000）就在里面 ——
 *    那是容器自己的地，算用户态。
 */

__ld_usram_end = ORIGIN(uocram) + LENGTH(uocram);
