# Copyright (c) 2024 Space Cubics, LLC.
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_SC_LIB_FRAM)
  zephyr_library()
  zephyr_library_sources_ifdef(CONFIG_SC_LIB_FRAM fram.c)
  zephyr_include_directories(.)
endif()
