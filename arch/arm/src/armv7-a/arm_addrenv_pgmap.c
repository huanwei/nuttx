/****************************************************************************
 * arch/arm/src/armv7-a/arm_addrenv_pgmap.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>

#include <nuttx/arch.h>
#include <nuttx/cache.h>
#include <nuttx/compiler.h>
#include <nuttx/debug.h>
#include <nuttx/pgalloc.h>

#include "pgalloc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_addrenv_find_page
 *
 * Description:
 *   Find physical page mapped to user virtual address from the address
 *   environment page directory.
 *
 * Input Parameters:
 *   addrenv - The user address environment.
 *   vaddr   - The user virtual address
 *
 * Returned Value:
 *   Page physical address on success; NULL on failure.
 *
 ****************************************************************************/

uintptr_t up_addrenv_find_page(arch_addrenv_t *addrenv, uintptr_t vaddr)
{
  uintptr_t l1entry;
  uintptr_t l2entry;
  FAR uintptr_t *l2table;
  int index;

  DEBUGASSERT(addrenv != NULL);

  /* 遍历两级表。用户区（.text/.data/heap/stack）全部由
   * arm_addrenv_create_region() 建成「L1 → 粗页表 → 4KB 小页」，
   * 所以这里只认 PMD_TYPE_PTE；section 直映不是用户区的形态，
   * 一律视为未映射。
   *
   * ★ 为什么必须真的走表，而不是「地址落在不在用户窗口里」：
   *   create_region() 只映射 **regionsize** 那么多页，而窗口大小是
   *   ARCH_*_NSECTS（上限）。窗口尾部是**没映射**的 ——
   *   只比窗口的话，那些地址会被判成合法，内核照样取数据异常。
   *
   * ★ 为什么不用 arm_addrenv_va_to_pa()：那个函数在没映射时
   *   **返回 vaddr 本身**（arm_physpgaddr.c 末尾），调用者分不出
   *   「真的映射到这个物理页」和「没有映射」。它做不了判据。
   */

  l1entry = mmu_l1table_getentry(addrenv->l1table, vaddr);
  if ((l1entry & PMD_TYPE_MASK) != PMD_TYPE_PTE)
    {
      return 0;
    }

  /* L1 存的是 L2 表的**物理**地址 */

  l2table = (FAR uintptr_t *)arm_pgvaddr(l1entry & PMD_PTE_PADDR_MASK);

  /* D-Cache 里可能是旧的 L2 表，先作废这一行再读 */

  index = (vaddr & SECTION_MASK) >> MM_PGSHIFT;
  up_invalidate_dcache((uintptr_t)&l2table[index],
                       (uintptr_t)&l2table[index] + sizeof(uintptr_t));

  l2entry = l2table[index];
  if ((l2entry & PTE_TYPE_MASK) != PTE_TYPE_SMALL)
    {
      return 0;
    }

  return l2entry & PTE_SMALL_PADDR_MASK;
}

/****************************************************************************
 * Name: up_addrenv_user_vaddr
 *
 * Description:
 *   Check if a virtual address is in user virtual address space.
 *
 * Input Parameters:
 *   vaddr - The virtual address.
 *
 * Returned Value:
 *   True if it is; false if it's not
 *
 ****************************************************************************/

bool up_addrenv_user_vaddr(uintptr_t vaddr)
{
  return arm_uservaddr(vaddr);
}

/****************************************************************************
 * Name: up_addrenv_page_vaddr
 *
 * Description:
 *   Find the kernel virtual address associated with physical page.
 *
 * Input Parameters:
 *   page - The page physical address.
 *
 * Returned Value:
 *   Page kernel virtual address on success; NULL on failure.
 *
 ****************************************************************************/

uintptr_t up_addrenv_page_vaddr(uintptr_t page)
{
  return arm_pgvaddr(page);
}

/****************************************************************************
 * Name: up_addrenv_page_wipe
 *
 * Description:
 *   Wipe a page of physical memory, first mapping it into kernel virtual
 *   memory.
 *
 * Input Parameters:
 *   page - The page physical address.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void up_addrenv_page_wipe(uintptr_t page)
{
  uintptr_t vaddr = arm_pgvaddr(page);
  memset((void *)vaddr, 0, MM_PGSIZE);
}
