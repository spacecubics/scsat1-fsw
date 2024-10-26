# Copyright (c) 2024 Space Cubics, LLC.
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_SC_LIB_FLASH_DATA)
  zephyr_library()
  zephyr_library_sources_ifdef(CONFIG_SC_LIB_FLASH_DATA data_nor.c)
  zephyr_library_sources_ifdef(CONFIG_SC_LIB_FLASH_DATA config_nor.c)
  zephyr_include_directories(.)
  zephyr_link_libraries(FS)
endif()
