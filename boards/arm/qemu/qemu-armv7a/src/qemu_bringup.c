/****************************************************************************
 * boards/arm/qemu/qemu-armv7a/src/qemu_bringup.c
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

#include <sys/types.h>
#include <syslog.h>

#include <nuttx/fs/fs.h>
#include <nuttx/fdt.h>
#include <nuttx/rpmsg/rpmsg_port.h>

#ifdef CONFIG_LIBC_FDT
#  include <libfdt.h>
#endif

#include <nuttx/virtio/virtio-mmio.h>

#include "chip.h"
#include "qemu-armv7a.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef QEMU_SPI_IRQ_BASE
#define QEMU_SPI_IRQ_BASE     32
#endif

/* ── [ORT] virtio-mmio 固定地址盲扫（2026-10-06，手册 §三·补六十二）────
 *
 * 为什么需要：FDT 路径要求 qemu 把 DTB 放在 0x40000000 —— 实测
 * （qemu 6.2 + `-kernel` 裸 ELF）该地址是 **NuttX 自己的向量表**，
 * 根本没有 DTB（monitor xp 验明）；于是 virtio 设备永远注册不上，
 * 症状 = **没有 eth0、且没有任何报错**。而设备确实在
 * （0x0a003e00 处读到了 virtio 魔数 "virt"）。
 *
 * 固定地址盲扫与 rv-virt 同法（BASE + 0x200 × 32 槽）；空槽由
 * virtio_register_mmio_device 返回 -ENODEV 跳过 —— 盲扫是安全的。
 *
 * IRQ 换算（**实测校准**）：armv7a 上 NuttX irq 号 == GIC 原始号 ——
 * 校准依据：本板 UART1 的 CONFIG_UART1_IRQ=33，与 qemu virt 设备树里
 * PL011 的 GIC 号 33 直接相等（arm64 侧那套 +32 的偏移**不适用**于
 * 本 arch；照抄会得到 111，实测 ARP 无应答 → connect ENETUNREACH）。
 * 故 virtio 槽位 irq = 48 + 槽号（48 = qemu virt 设备树里 virtio
 * 的起始 GIC 号）。
 */

#ifdef CONFIG_DRIVERS_VIRTIO_MMIO
#define QEMU_VIRTIO_MMIO_BASE    0x0a000000ul
#define QEMU_VIRTIO_MMIO_REGSIZE 0x200
#define QEMU_VIRTIO_MMIO_NUM     32
#define QEMU_VIRTIO_MMIO_DTIRQ   48

static void register_virtio_mmio_fixed(void)
{
  int i;

  for (i = 0; i < QEMU_VIRTIO_MMIO_NUM; i++)
    {
      virtio_register_mmio_device(
        (FAR void *)(QEMU_VIRTIO_MMIO_BASE + QEMU_VIRTIO_MMIO_REGSIZE * i),
        QEMU_VIRTIO_MMIO_DTIRQ + i);
    }
}
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(CONFIG_LIBC_FDT) && defined(CONFIG_DEVICE_TREE)

/****************************************************************************
 * Name: register_devices_from_fdt
 ****************************************************************************/

static void register_devices_from_fdt(void)
{
  const void *fdt = fdt_get();
  int ret;

  if (fdt == NULL)
    {
      return;
    }

#ifdef CONFIG_DRIVERS_VIRTIO_MMIO
  /* ⚠️ 同校准：armv7a 的 irqbase 是 0（arm64 才用 32）；本路径在
   * qemu 6.2 下因无 DTB 是死路，但留着即修正。 */

  ret = fdt_virtio_mmio_devices_register(fdt, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "fdt_virtio_mmio_devices_register failed, ret=%d\n",
             ret);
    }
#endif

#ifdef CONFIG_PCI
  ret = fdt_pci_ecam_register(fdt);
  if (ret < 0)
    {
      syslog(LOG_ERR, "fdt_pci_ecam_register failed, ret=%d\n", ret);
    }
#endif

#ifdef CONFIG_MTD_CFI
  ret = fdt_cfi_register(fdt);
  if (ret < 0)
    {
      syslog(LOG_ERR, "fdt_cfi_register failed, ret=%d\n", ret);
    }
#endif

  UNUSED(ret);
}

#endif

/****************************************************************************
 * Name: rpmsg_port_uart_init
 ****************************************************************************/

#ifdef CONFIG_RPMSG_PORT_UART
static int rpmsg_port_uart_init(void)
{
  const char *remotecpu;
  const char *localcpu;
  int ret;

  if (strcmp(CONFIG_LIBC_HOSTNAME, "server") == 0)
    {
      localcpu = "server2";
      remotecpu = "proxy2";
    }
  else if (strcmp(CONFIG_LIBC_HOSTNAME, "proxy") == 0)
    {
      localcpu = "proxy2";
      remotecpu = "server2";
    }
  else
    {
      syslog(LOG_ERR, "ERROR: hostname must be server or proxy, now: %s\n",
             CONFIG_LIBC_HOSTNAME);
      return -EINVAL;
    }

  const struct rpmsg_port_config_s cfg =
    {
      .remotecpu = remotecpu,
      .txnum = 8,
      .rxnum = 8,
      .txlen = 2048,
      .rxlen = 2048,
    };

  ret = rpmsg_port_uart_initialize(&cfg, "/dev/ttyV0", localcpu);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize rpmsg port uart: %d\n", ret);
    }

  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

int qemu_bringup(void)
{
  int ret;

#ifdef CONFIG_FS_TMPFS
  /* Mount the tmpfs file system */

  ret = nx_mount(NULL, CONFIG_LIBC_TMPDIR, "tmpfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount tmpfs at %s: %d\n",
             CONFIG_LIBC_TMPDIR, ret);
    }
#endif

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

#if defined(CONFIG_LIBC_FDT) && defined(CONFIG_DEVICE_TREE)
  register_devices_from_fdt();

  /* DTB 不在（qemu 6.2 裸 ELF，见上注）→ 走固定地址盲扫 */

#ifdef CONFIG_DRIVERS_VIRTIO_MMIO
  if (fdt_get() == NULL)
    {
      register_virtio_mmio_fixed();
    }
#endif

#elif defined(CONFIG_DRIVERS_VIRTIO_MMIO)
  register_virtio_mmio_fixed();
#endif

#if defined(CONFIG_FS_LITTLEFS) && defined(CONFIG_MTD_CFI)
  /* Mount the procfs file system */

  ret = nx_mount("/dev/cfi-flash1", "/data", "littlefs", 0, "autoformat");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount littlefs at /data: %d\n", ret);
    }
#endif

#ifdef CONFIG_RPMSG_PORT_UART
  ret = rpmsg_port_uart_init();
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize rpmsg port uart: %d\n", ret);
    }
#endif

  UNUSED(ret);
  return OK;
}
