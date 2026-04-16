# SPDX-FileCopyrightText: Copyright (c) 2026 Nuvoton Technology Corporation
# SPDX-License-Identifier: Apache-2.0

if(NOT ZEPHYR_TRUSTED_FIRMWARE_M_MODULE_DIR)
    Message(FATAL_ERROR "\"trusted-firmware-m\" module is unavailable")
endif()

if(CONFIG_ARM_SECURE_FIRMWARE)
  zephyr_library_named(tz_nuvoton_secure)

  # Needed by tfm_hal_device_header.h to pass compile
  zephyr_library_compile_definitions(CMSIS_device_header=\"cmsis_dummy.h\")

  zephyr_library_include_directories(
    include
    ${ZEPHYR_TRUSTED_FIRMWARE_M_MODULE_DIR}/interface/include
    ${ZEPHYR_TRUSTED_FIRMWARE_M_MODULE_DIR}/lib/tfm_helper_lib
    ${ZEPHYR_TRUSTED_FIRMWARE_M_MODULE_DIR}/platform/include
    )

  zephyr_library_sources(
    src/tfm_platform_hal_ioctl.c
    src/tfm_platform_sp.c
    src/tfm_platform_system.c
    src/tfm_veneers.c
    ../src/zephyr_tfm_log.c
    ../interface/interface.c
    )
elseif(CONFIG_ARM_NONSECURE_FIRMWARE)
  zephyr_include_directories(
    include
    ${ZEPHYR_TRUSTED_FIRMWARE_M_MODULE_DIR}/interface/include
    )

  zephyr_library_named(tz_nuvoton_nonsecure)

  zephyr_library_sources(
    src/tfm_platform_hal_ioctl_api.c
    ../src/reboot.c
    ../src/zephyr_tfm_log.c
    ../interface/interface.c
    ${ZEPHYR_TRUSTED_FIRMWARE_M_MODULE_DIR}/interface/src/tfm_platform_api.c
    ${ZEPHYR_TRUSTED_FIRMWARE_M_MODULE_DIR}/interface/src/tfm_tz_psa_ns_api.c
    )

    # Not real TF-M integration, though, pass config option extra
    # to override sys_arch_reboot with tfm_platform_system_reset
    set_source_files_properties(../src/reboot.c
      PROPERTIES
        COMPILE_DEFINITIONS CONFIG_TFM_PARTITION_PLATFORM
      )
endif()
