Build TF-M enabled application
##############################

This document guides how to build `TF-M`_ enabled application.

.. _TrustedFirmware-M: https://www.trustedfirmware.org/projects/tf-m/
.. _TF-M: `TrustedFirmware-M`_

Updating source code
********************

This `TF-M_` support doesn't upstream yet.
Users need to switch to specific branches for related repositories.

For ``zephyr`` repository, switch to the ``nvt_m3351_enable_tfm`` branch:

.. code-block:: console

    $ cd `west topdir`
    $ cd zephyr
    $ git remote add opennuvoton https://github.com/OpenNuvoton/zephyr
    $ git fetch opennuvoton
    $ git checkout -b nvt_m3351_enable_tfm opennuvoton/nvt_m3351_enable_tfm

Update west modules:

.. code-block:: console

    $ cd `west topdir`
    $ west update

For ``trusted-firmware-m`` repository, switch to the ``nuvoton_zephyr_m3351`` branch:

.. code-block:: console

    $ cd `west topdir`
    $ cd modules/tee/tf-m/trusted-firmware-m
    $ git remote add opennuvoton https://github.com/OpenNuvoton/trusted-firmware-m
    $ git fetch opennuvoton
    $ git checkout -b nuvoton_zephyr_m3351 opennuvoton/nuvoton_zephyr_m3351

For ``hal_nuvoton`` repository, switch to the ``master`` branch:

.. code-block:: console

    $ cd `west topdir`
    $ cd modules/hal/nuvoton
    $ git fetch upstream
    $ git checkout -b master upstream/master

Zephyr board options
********************

The BOARD options are summarized below:

+-----------------------------------+-------------------------------------------+
| BOARD                             | Description                               |
+===================================+===========================================+
| BOARD: numaker_m3351ki/m335xxx    | Building with TrustZone disabled          |
+-----------------------------------+-------------------------------------------+
| BOARD: numaker_m3351ki/m335xxx/ns | Building with TrustZone/TF-M enabled      |
+-----------------------------------+-------------------------------------------+

BOARD: numaker_m3351ki/m335xxx
==============================

The ``numaker_m3351ki/m335xxx`` board target is used to build
TrustZone disabled application as normal.

BOARD: numaker_m3351ki/m335xxx/ns
=================================

The ``numaker_m3351ki/m335xxx/ns`` board target is used to build
TrustZone enabled application.
It will build `TrustedFirmware-M`_ as Secure image 
(``CONFIG_BUILD_WITH_TFM`` to ``y``).
and Zephyr application as Non-secure image
(``CONFIG_TRUSTED_EXECUTION_NONSECURE`` to ``y``).

Memory and Peripheral Ownership
*******************************

Partitioning APROM
==================

APROM is 1024KB in total.
It is partitioned for Secure and Non-secure.
Its address map:

+---------------------+-------------------+------------------------------------+
| Name                | Address[Size]     | Comment                            |
+=====================+===================+====================================+
| APROM Secure        | 0x0[512KB]        | APROM for Secure                   |
+---------------------+-------------------+------------------------------------+
| APROM Non-secure    | 0x10080000[512KB] | APROM for Non-secure               |
+---------------------+-------------------+------------------------------------+

More detailed address map:

+----------------------+-------------------+------------------------------------+
| Name                 | Address[Size]     | Comment                            |
+======================+===================+====================================+
| boot_partition       | 0x0[64KB]         | MCUboot secure image               |
+----------------------+-------------------+------------------------------------+
|                      | 0x10000[64KB]     | Reserved                           |
+----------------------+-------------------+------------------------------------+
| slot0_partition      | 0x20000[192KB]    | Primary slot of secure image       |
+----------------------+-------------------+------------------------------------+
| slot1_partition      | 0x50000[192KB]    | Secondary slot of secure image     |
+----------------------+-------------------+------------------------------------+
| slot0_ns_partition   | 0x10080000[256KB] | Primary slot of non-secure image   |
+----------------------+-------------------+------------------------------------+
| slot1_ns_partition   | 0x100c0000[256KB] | Secondary slot of non-secure image |
+----------------------+-------------------+------------------------------------+

Partitioning LDROM
==================

APROM is 16KB in total.
It is partitioned for Secure wholly.
Its address map:

+----------------------+------------------+------------------------------------+
| Name                 | Address[Size]    | Comment                            |
+======================+==================+====================================+
|                      | 0x0f100000[8KB]  | Unused                             |
+----------------------+------------------+------------------------------------+
| MCUboot Scratch Area | 0x0f102000[8KB]  |                                    |
+----------------------+------------------+------------------------------------+

Partitioning Data Flash
=======================

Data Flash is 64KB in total.
It is partitioned for Secure wholly.
Its address map:

+--------------------------------+-------------------+------------------------------------+
| Name                           | Address[Size]     | Comment                            |
+================================+===================+====================================+
| Protected Storage (PS)         | 0x0f210000[24KB]  | Unused                             |
+--------------------------------+-------------------+------------------------------------+
| Internal Trusted Storage (ITS) | 0x0f216000[24KB]  |                                    |
+--------------------------------+-------------------+------------------------------------+
| OTP / NV Counters              | 0x0f21c000[16KB]  |                                    |
+--------------------------------+-------------------+------------------------------------+

Partitioning SRAM
=================

SRAM is 128KB in total.
It is partitioned for Secure and Non-secure.
Its address map:

+----------------------+------------------+------------------------------------+
| Name                 | Address[Size]    | Comment                            |
+======================+==================+====================================+
| SRAM Secure          | 0x20000000[64KB] | SRAM for Secure                    |
+----------------------+------------------+------------------------------------+
| SRAM Non-secure      | 0x30010000[64KB] | SRAM for Non-secure                |
+----------------------+------------------+------------------------------------+


Partitioning Peripherals
========================

- Always Secure peripherals: CRYPTO, WDT0, and WWDT0
- Secure peripherals with shared registers: CLK, FMC, DFMC, NMI, SCU, and SYS
- Partitioned for Secure: CACHE
- All other peripherals are partitioned for Non-secure.

.. note::

   UART0 is partitioned for Non-secure. However, it is used for console for both
   Secure and Non-secure.
  
Building and Flashing
*********************

Building
========

To build `TF-M`_ enabled application, specify board target
as ``numaker_m3351ki/m335xxx/ns``:

.. code-block:: console

    $ west -v build \
    -b numaker_m3351ki/m335xxx/ns \
    zephyr/samples/hello_world

Flashing
========

For `TF-M`_ enabled build, this will flash 3 images together:

- MCUboot (Secure)
- TF-M (Secure)
- Zephyr (Non-secure)

.. code-block:: console

    $ west flash

Troubleshooting
***************

TrustZone disabled application fail to run after running
`TF-M`_ enabled application before

    Partitioning of Flash and Data Flash will persist across reset cycle.
    If users run `TF-M`_ enabled application and then get back to run
    TrustZone disabled application on the same board hardware,
    the latter will fail because it doesn't re-partition.

    The quick solution would be to do whole chip erase via e.g.
    `NuMicro ICP Programming Tool <https://github.com/OpenNuvoton/Nuvoton_Tools>`_.
