# SPDX-License-Identifier: Apache-2.0

# Per Nuvoton customized OpenOCD, openocd command can be openocd or
# openocd_cmsis-dap, dependent on target board.
if(CONFIG_SOC_SERIES_M46X)
  set(openocd_main "openocd")
else()
  set(openocd_main "openocd_cmsis-dap")
endif()

if(OPENOCD)
  cmake_path(GET OPENOCD PARENT_PATH openocd_parent)
  cmake_path(HAS_EXTENSION OPENOCD openocd_has_ext)
  if(openocd_has_ext)
    cmake_path(GET OPENOCD EXTENSION LAST_ONLY openocd_ext)
  endif()
  cmake_path(APPEND openocd_path ${openocd_parent} "${openocd_main}${openocd_ext}")

  if(EXISTS ${openocd_path})
    set(OPENOCD ${openocd_path} CACHE FILEPATH "" FORCE)
  endif()
else()
  find_program(OPENOCD ${openocd_main})
endif()
