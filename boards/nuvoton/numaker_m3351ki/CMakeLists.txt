# SPDX-License-Identifier: Apache-2.0

if(CONFIG_ARM_SECURE_FIRMWARE)
  # BSP relies on CHECK_SCU_VIOLATION to print message in SCU_IRQHandler
  zephyr_compile_definitions(CHECK_SCU_VIOLATION)

  # Save veneer for Non-Secure build in the following
  if(CONFIG_ARM_FIRMWARE_HAS_SECURE_ENTRY_FUNCS)
    set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
      COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_BINARY_DIR}/${CONFIG_ARM_ENTRY_VENEERS_LIB_NAME}
        ${CMAKE_CURRENT_SOURCE_DIR}/${CONFIG_ARM_ENTRY_VENEERS_LIB_NAME}
    )
  endif()
endif()

if(CONFIG_ARM_NONSECURE_FIRMWARE)
  # BSP relies on NVT_CMSE_NON_SECURE for Non-Secure
  zephyr_compile_definitions(NVT_CMSE_NON_SECURE)

  # Restore veneer for this Non-Secure build
  if(CONFIG_ARM_FIRMWARE_USES_SECURE_ENTRY_FUNCS)
    set(VENEER_SRC_PATH ${CMAKE_CURRENT_SOURCE_DIR}/${CONFIG_ARM_ENTRY_VENEERS_LIB_NAME})
    set(VENEER_DEST_PATH   ${CMAKE_BINARY_DIR}/${CONFIG_ARM_ENTRY_VENEERS_LIB_NAME})

    add_custom_command(
      OUTPUT ${VENEER_DEST_PATH}
      COMMAND ${CMAKE_COMMAND} -E copy ${VENEER_SRC_PATH} ${VENEER_DEST_PATH}
      DEPENDS ${VENEER_SRC_PATH}
      COMMENT "Copying veneer file to build directory before compilation"
    )
    add_custom_target(restore_veneer DEPENDS ${VENEER_DEST_PATH})

    add_dependencies(zephyr_interface restore_veneer)
  endif()
endif()
