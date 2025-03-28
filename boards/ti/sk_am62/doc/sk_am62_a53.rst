.. zephyr:board:: sk_am62

Overview
********

The SK-AM62 board configuration is used by Zephyr applications that run on
the TI AM62x platform. The board configuration provides support for:

- ARM Cortex-A53 core and the following features:
   - General Interrupt Controller (GIC)
   - ARM Generic Timer (arch_timer)
   - On-chip SRAM (oc_sram)
   - UART interfaces (uart0 to uart6)
   - Mailbox interface (mbox0)

See the `TI AM62X Product Page`_ for details.

Hardware
********
The SK-AM62 EVM features the AM62x SoC, which is composed of a quad Cortex-A53
cluster and a single Cortex-M4 core in the MCU domain. Zephyr is ported to run on
the A53 core and the following listed hardware specifications are used:

- High-performance ARM Cortex-A53
- Memory

   - 2GB of DDR4

- Debug

   - XDS110 based JTAG

Supported Features
==================

The sk_am62 configuration supports the following hardware features:

+-----------+------------+-------------------------------------+
| Interface | Controller | Driver/Component                    |
+===========+============+=====================================+
| GIC-v3    | on-chip    | interrupt controller                |
+-----------+------------+-------------------------------------+
| ARM TIMER | on-chip    | system clock                        |
+-----------+------------+-------------------------------------+
| PINCTRL   | on-chip    | pinctrl                             |
+-----------+------------+-------------------------------------+
| UART      | on-chip    | serial port                         |
+-----------+------------+-------------------------------------+
| Mailbox   | on-chip    | IPC Mailbox                         |
+-----------+------------+-------------------------------------+

Other hardware features are not currently supported by the port.

Devices
========
System Clock
------------

This board configuration uses a system clock frequency of 200 MHz.

DDR RAM
-------

The board has 2GB of DDR RAM available. This board configuration
allocates Zephyr 1MB of RAM (0x82000000 to 0x82100000).

Serial Port
-----------

This board configuration uses a single serial communication channel with the
SoC's UART0.

SD Card
*******

Download TI's official `WIC`_ and flash the WIC file with an etching software
onto an SD-card. This will boot Linux on the A53 application cores of the EVM.

The default configuration can be found in
:zephyr_file:`boards/ti/sk_am62/sk_am62_am6234_a53_defconfig`

Building
********


To test the A53 core, we build the :zephyr:code-sample:`hello_world` sample with the following command.

.. zephyr-app-commands::
   :board: sk_am62/am6234/a53
   :zephyr-app: samples/hello_world
   :goals: build
This builds the program and the binary is present in the :file:`build/zephyr` directory as
:file:`zephyr.bin`.

Programming
***********

Copy the compiled ``zephyr.bin`` to the first FAT partition of the SD card and
plug the SD card into the board. Power it up and stop the u-boot execution at
prompt.

Use U-Boot to load and kick zephyr.bin:

.. code-block:: console

    fatload mmc 1:1 0x82000000 zephyr.bin; go 0x82000000

The Zephyr application should start running on the A53 core.


References
**********

AM62x SK EVM TRM:
   https://www.ti.com/lit/ug/spruiv7/spruiv7.pdf

.. _TI AM62X Product Page:
   https://www.ti.com/product/AM625

.. _WIC:
   https://dr-download.ti.com/software-development/software-development-kit-sdk/MD-PvdSyIiioq/10.01.10.04/tisdk-default-image-am62xx-evm-10.01.10.04.rootfs.wic.xz
