.. SPDX-License-Identifier: GPL-2.0

====================================
Apple J714c initial native Linux boot
====================================

J714c is the 14-inch M5 Max MacBook Pro (T6050). This configuration
provides a RAM-only boot path with AICv3, architectural timers, the
framebuffer left by the boot firmware, and the MTP keyboard and trackpad
over DockChannel. The kernel reaches a BusyBox shell on one CPU with the
built-in keyboard working.

The device tree describes all 18 CPUs in three clusters of six. For this
one-CPU baseline, secondary cores stay stopped in the loader and the kernel
runs with ``maxcpus=1``. The loader removes unavailable CPUs and fills the
release addresses of the remaining nodes.

MMIO addresses in the T6050 device tree are physical: the Apple Device Tree
``arm-io`` child address plus 0x200000000. The AIC is at 0x280400000, the
Samsung UART at 0x505200000, the watchdog at 0x28836c000, the GPIO block
at 0x29c000000 and the PMGR window at 0x280600000. The AIC uses the
``apple,t8142-aic3`` fallback and the 3-cell interrupt form.

SMP bring-up
============

Not done. On this machine the secondary CPUs' RVBAR is locked to the
loader image that the boot firmware started. Writes to ``cpu-impl-reg``
from EL2 raise an SError, so the stock chainload path cannot relocate the
secondaries and the loader does not start them. The ``cpus`` nodes carry
``cpu-release-addr`` for the spin-table path once a loader can start them.

CPU frequency bring-up
======================

Not done. ``apple_soc_cpufreq_init`` reports ``failed to get cluster info:
-19``; the tree has no cluster controller nodes for T6050. The CPUs run at
the frequency the boot firmware left.

Bootloader requirements
=======================

Use the T6050-capable Aurora m1n1. Before entering Linux, the loader must:

* Supply usable memory and reserved regions. T6050 DRAM starts at
  0x10000000000; the DTS memory size is only a loader placeholder.
* Fill CPU release addresses and remove the CPUs it has not started.
* Fill the firmware framebuffer's address, size, geometry and format. The
  ``framebuffer@0`` node under ``/chosen`` is enabled in the DTS because
  the loader's display setup returns early for T6050.
* Program the MTP DMA address filter from the Apple Device Tree
  ``dapf-instance-0`` of ``dart-mtp`` into its filter window, register 1
  of that node (0x294810000). Entry 0 is the PMGR window 0x28830c000 that
  the MTP firmware reads first; without it the firmware never sends HELLO.
  Register 1 of ``dart-aop`` on T6050 is the block shared by every DART at
  0x2890fc000, and writing filter entries there raises an SError, so the
  generic DAPF table must not run on this chip.
* Leave the MTP DART in translate mode. In bypass mode the filter rejects
  the firmware's writes to the kernel's buffers.
* Leave the console, interrupt controller and display powered. The DTS
  has no PMGR power domains; power stays as the firmware left it.

On this machine the MTP coprocessor is stopped at kernel entry
(``CPU_CONTROL`` 0) and its DART holds no valid translation, so the kernel
starts the firmware itself. There is no live RTKit session to adopt.

Build
=====

On an arm64 Linux build host with the usual kernel build dependencies::

  out="$PWD/../j714c-build"
  make ARCH=arm64 O="$out" \
      KCONFIG_ALLCONFIG=arch/arm64/configs/j714c.config allnoconfig
  make ARCH=arm64 O="$out" -j"$(nproc)" Image.gz apple/t6050-j714c.dtb

``j714c.config`` is ``j813.config`` with ``NR_CPUS=18``, ``ARM64_SME``,
``ARM64_LSUI`` and ``ARM64_ERRATUM_4193714``. Add
``CROSS_COMPILE=aarch64-linux-gnu-`` to both commands when cross-compiling
with GCC from another architecture.

Boot and qualification
======================

Use a RAM-only initramfs with a BusyBox shell. Its init mounts proc, sysfs
and devtmpfs and makes no disk mounts.

The command line is::

  maxcpus=1 console=tty0

Load Image.gz, the DTB and the initramfs through the loader's USB proxy
with the proxyclient ``linux.py``; on T6050 it continues when the loader
refuses to start secondaries. When the loader itself is chainloaded over
the proxy, ``chainload.py`` skips the secondary RVBAR writes on T6050.

The qualified console is the firmware framebuffer. The Samsung UART node
records its measured resources; no UART console path has been observed
from this laptop. Before the kernel jump the loader's console is readable
on the USB proxy; after it, the built-in display is the only output.

Verified on 2026-10-23 with kernel ``7.2.10-aurora-t6050-j714c-wip+``:

* T6050/J714c identity, one online CPU, timer interrupts, the shell
  responding to typed commands.
* ``simple-framebuffer`` registered at 0x11fd31e6880, 3024x1890,
  ``x2r10g10b10``, with the console on it.
* The MTP firmware ``AppleMTPFirmwareMac-6300.85.6~243`` starting, and
  ``dockchannel-hid`` enumerating the keyboard, multi-touch, actuator and
  stm interfaces. Keystrokes reach the console.

The same result was reproduced with the loader built from the Aurora m1n1
fork and the kernel built from ``j714c.config`` as above.

These boot tests do not establish SMP, CPU frequency control, PMGR power
domains, suspend, GPU acceleration, USB, storage, networking, audio, SMC
or temperature monitoring. The Samsung UART, the DockChannel UART and the
``gpio0`` block are described but disabled or unverified.

Built-in input
==============

MTP provides keyboard and trackpad HID descriptors at runtime over the
DockChannel fifo at 0x294b14000 with data windows from 0x294b28000. The
``keyboard`` alias lets the loader fill ``hid-country-code`` and
``apple,keyboard-layout-id`` from this machine's own boot environment.

The T6050 MTP takes the same early AP wake as the T8142 MTP in
``rtkit-helper``. The trackpad enumerates as a multi-touch device; its
firmware file is not supplied and the multi-touch node carries no
``firmware-name``. The STM and actuator interfaces report ready without
the AFE and STM reset GPIOs the J514 tree uses; no SMC GPIO is described.

Native display
==============

None. The display stays on the framebuffer the boot firmware left, with no
DCP node and no display reserved-memory setup from the loader.
