# Copyright (c) 2024 Space Cubics, LLC.
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_SC_LIB_CSP reply.c)
zephyr_library_sources_ifdef(CONFIG_SC_LIB_CSP file.c)
zephyr_library_sources_ifdef(CONFIG_SC_LIB_CSP flash.c)
zephyr_library_sources_ifdef(CONFIG_SC_LIB_CSP eps.c)
zephyr_library_sources_ifdef(CONFIG_SC_LIB_CSP sys.c)

zephyr_include_directories(.)

zephyr_link_libraries(libcsp)
