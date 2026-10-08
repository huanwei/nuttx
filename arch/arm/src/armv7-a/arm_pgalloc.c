/****************************************************************************
 * arch/arm/src/armv7-a/arm_pgalloc.c
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

#include <string.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/debug.h>
#include <nuttx/irq.h>
#include <nuttx/sched.h>
#include <nuttx/arch.h>
#include <nuttx/addrenv.h>

#include "mmu.h"
#include "pgalloc.h"
#include "sched/sched.h"

#ifdef CONFIG_BUILD_KERNEL

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: alloc_pgtable
 *
 * Description:
 *   Add one page table to a memory region.
 *
 ****************************************************************************/

static uintptr_t alloc_pgtable(void)
{
  uintptr_t paddr;
  uint32_t *l2table;

  /* Allocate one physical page for the L2 page table */

  paddr = mm_pgalloc(1);
  binfo("a new l2 page table (paddr=%x)\n", paddr);
  if (paddr)
    {
      DEBUGASSERT(MM_ISALIGNED(paddr));

      /* Get the virtual address corresponding to the physical page address */

      l2table = (uint32_t *)arm_pgvaddr(paddr);

      /* Initialize the page table */

      memset(l2table, 0, MM_PGSIZE);

      /* Make sure that the initialized L2 table is flushed to physical
       * memory.
       */

      up_flush_dcache((uintptr_t)l2table,
                      (uintptr_t)l2table + MM_PGSIZE);
    }

  return paddr;
}

/****************************************************************************
 * Name: get_pgtable
 *
 * Description:
 *   Return the physical address of the L2 page table corresponding to
 *   'vaddr'
 *
 ****************************************************************************/

static int get_pgtable(arch_addrenv_t *addrenv, uintptr_t vaddr)
{
  uint32_t l1entry;
  uintptr_t paddr;
  unsigned int hpoffset;

  /* The current implementation only supports extending the user heap
   * region as part of the implementation of user sbrk().
   */

  DEBUGASSERT(vaddr >= CONFIG_ARCH_HEAP_VBASE && vaddr < ARCH_HEAP_VEND);

  /* Get the current level 1 entry corresponding to this vaddr */

  hpoffset = vaddr - CONFIG_ARCH_HEAP_VBASE;
  if (hpoffset >= ARCH_HEAP_SIZE)
    {
      return 0;
    }

  l1entry = (uintptr_t)mmu_l1_getentry(vaddr);
  if (l1entry == 0)
    {
      /* No page table has been allocated... allocate one now */

      paddr = alloc_pgtable();
      if (paddr != 0)
        {
          /* Set the new level 1 page table entry in the address
           * environment.
           */

          l1entry = paddr | MMU_L1_PGTABFLAGS;

          /* And instantiate the modified environment */

          up_addrenv_select(addrenv);
        }
    }

  return l1entry & PMD_PTE_PADDR_MASK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pgalloc
 *
 * Description:
 *   If there is a page allocator in the configuration and if and MMU is
 *   available to map physical addresses to virtual address, then function
 *   must be provided by the platform-specific code.  This is part of the
 *   implementation of sbrk().  This function will allocate the requested
 *   number of pages using the page allocator and map them into consecutive
 *   virtual addresses beginning with 'brkaddr'
 *
 *   NOTE:  This function does not use the up_ naming standard because it
 *   is indirectly callable from user-space code via a system trap.
 *   Therefore, it is a system interface and follows a different naming
 *   convention.
 *
 * Input Parameters:
 *   brkaddr - The heap break address.  The next page will be allocated and
 *     mapped to this address.  Must be page aligned.  If the memory manager
 *     has not yet been initialized and this is the first block requested for
 *     the heap, then brkaddr should be zero.  pgalloc will then assigned the
 *     well-known virtual address of the beginning of the heap.
 *   npages - The number of pages to allocate and map.  Mapping of pages
 *     will be contiguous beginning beginning at 'brkaddr'
 *
 * Returned Value:
 *   The (virtual) base address of the mapped page will returned on success.
 *   Normally this will be the same as the 'brkaddr' input. However, if
 *   the 'brkaddr' input was zero, this will be the virtual address of the
 *   beginning of the heap.  Zero is returned on any failure.
 *
 ****************************************************************************/

uintptr_t pgalloc(uintptr_t brkaddr, unsigned int npages)
{
  struct tcb_s *tcb = this_task();
  struct arch_addrenv_s *addrenv;
  uint32_t *l2table;
  uintptr_t paddr;
  uintptr_t base;
  unsigned int index;

  binfo("tcb->pid=%d tcb->group=%p\n", tcb->pid, tcb->addrenv_own);
  binfo("brkaddr=%x npages=%d\n", brkaddr, npages);
  DEBUGASSERT(tcb && tcb->addrenv_own);
  addrenv = &tcb->addrenv_own->addrenv;

  /* The current implementation only supports extending the user heap
   * region as part of the implementation of user sbrk().  This function
   * needs to be expanded to also handle (1) extending the user stack
   * space and (2) extending the kernel memory regions as well.
   */

  /* brkaddr = 0 means that no heap has yet been allocated */

  if (brkaddr == 0)
    {
      brkaddr = CONFIG_ARCH_HEAP_VBASE;
    }

#ifdef CONFIG_ORT_CONTAINER
  /* [ORT §82] 容器内存限额的**增长闸门**：限额要管总量，增长也算。
   *
   *   闸门放这里的原因（盲区实例）：用户 app 的 malloc 走 **app 侧**
   *   libmm 的那份 sbrk（libmm.a 里有 T sbrk/umm_initialize；appa 侧
   *   pgalloc 是 libproxies 的 syscall 代理）—— kernel 侧的 umm_sbrk
   *   改不到那条路径。而**两个世界都必经 pgalloc**（app 经代理进来、
   *   kernel 直调），所以闸门收在这唯一的内核副本上。
   *   0 = 未设 = 区域上限；超限走文档化的 `return 0`（调用方 → EAGAIN/
   *   ENOMEM，优雅失败）。 */

  {
    uint32_t cap = this_task()->group->tg_ort_memcap;

    if (cap > 0 &&
        brkaddr + ((uintptr_t)npages << MM_PGSHIFT) >
        CONFIG_ARCH_HEAP_VBASE + (uintptr_t)cap)
      {
        return 0;
      }
  }
#endif

  /* [ORT] 上游缺陷本地修复（第 7 处，手册 §三·补八十二）：
   *
   *   ① 越界是**合法输入**（默认堆 = 整个 HEAP 区时，"再长一点"
   *      必然在界外——malloc 耗尽时就会走到这），应按本函数的文档
   *      契约走失败路径 `return 0`，而不是断言打停机。
   *   ② 返回**基地址**（文档契约："Normally this will be the same as
   *      the 'brkaddr' input"）。原实现在循环里累加 brkaddr、返回的
   *      是**映射尾** —— mm_extend() 的连续性断言必炸（cap 堆实测
   *      `mm_extend.c:89`）。 */

  if (brkaddr < CONFIG_ARCH_HEAP_VBASE || brkaddr >= ARCH_HEAP_VEND)
    {
      return 0;
    }

  DEBUGASSERT(MM_ISALIGNED(brkaddr));

  base = brkaddr;

  for (; npages > 0; npages--)
    {
      if (brkaddr >= ARCH_HEAP_VEND)
        {
          /* 半途到界：不回收已映射页（原型；随进程退出回收） */

          return 0;
        }

      /* Get the physical address of the level 2 page table */

      paddr = get_pgtable(addrenv, brkaddr);
      binfo("l2 page table (paddr=%x)\n", paddr);
      binfo("brkaddr=%x\n", brkaddr);
      if (paddr == 0)
        {
          return 0;
        }

      /* Get the virtual address corresponding to the physical page address */

      l2table = (uint32_t *)arm_pgvaddr(paddr);

      /* Back up L2 entry with physical memory */

      paddr = mm_pgalloc(1);
      binfo("a new page (paddr=%x)\n", paddr);
      if (paddr == 0)
        {
          return 0;
        }

      /* The table divides a 1Mb address space up into 256 entries, each
       * corresponding to 4Kb of address space.  The page table index is
       * related to the offset from the beginning of 1Mb region.
       */

      index = (brkaddr & 0x000ff000) >> 12;

      /* Map the .text region virtual address to this physical address */

      DEBUGASSERT(l2table[index] == 0);
      l2table[index] = paddr | MMU_L2_UDATAFLAGS;
      brkaddr += MM_PGSIZE;

      /* Make sure that the modified L2 table is flushed to physical
       * memory.
       */

      up_flush_dcache((uintptr_t)&l2table[index],
                      (uintptr_t)&l2table[index] + sizeof(uint32_t));
    }

  return base;
}

#endif /* CONFIG_BUILD_KERNEL */
