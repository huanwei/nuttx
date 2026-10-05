/****************************************************************************
 * boards/arm/stm32f7/nucleo-f767zi/scripts/memory.ld
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * [ORT] 新增（2026-10-05，手册 §三·补六十）：本板原先只有扁平构建的
 * flash.ld —— PROTECTED 构建（ORT-M 真硅）需要 kflash/uflash 与
 * ksram/usram 分区。布局按 STM32F767ZI 的 RM0410 内存图
 * （DTCM 128K@0x20000000 / SRAM1 368K@0x20020000 / SRAM2 16K@0x2007C000 /
 * flash 2M@0x08000000），风格照 nucleo-f746zg 的同名文件。
 *
 * ★ 域池不在这里 —— 它是运行期 MPU 配置的目标，不是链接节。
 *   本板池 = DTCM 顶半 64K（Kconfig ORT_DOMAIN_POOL_BASE=0x20000000），
 *   配合 STM32_DTCMEXCLUDE 把 DTCM 整块排除出堆 —— 板级不授予用户访问、
 *   ORT 按域授予，正是想要的语义（§60）。
 ****************************************************************************/

MEMORY
{
  itcm  (rwx) : ORIGIN = 0x00200000, LENGTH = 16K
  dtcm  (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
  kflash (rx) : ORIGIN = 0x08000000, LENGTH = 512K
  uflash (rx) : ORIGIN = 0x08080000, LENGTH = 512K
  xflash (rx) : ORIGIN = 0x08100000, LENGTH = 1024K
  ksram (rwx) : ORIGIN = 0x20020000, LENGTH = 16K
  usram (rwx) : ORIGIN = 0x20024000, LENGTH = 8K
  xsram (rwx) : ORIGIN = 0x20026000, LENGTH = 344K
  sram2 (rwx) : ORIGIN = 0x2007c000, LENGTH = 16K
}

/* 用户堆上界（us_heapend）：stm32_allocateheap.c 的 PROTECTED 用户堆
 * 到 SRAM1_END (0x2007C000) —— xsram 末端与之对齐（§34·8 的 mps 先例：
 * 同一个符号、同一种用法，ORT 的用户指针闸门在 M 侧需要它）。 */

__ld_usram_end = ORIGIN(xsram) + LENGTH(xsram);
