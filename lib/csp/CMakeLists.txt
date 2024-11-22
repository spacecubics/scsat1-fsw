# Copyright (c) 2024 Space Cubics, LLC.
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_SC_LIB_CSP reply.c)
zephyr_library_sources_ifdef(CONFIG_SC_LIB_CSP file.c)
zephyr_library_sources_ifdef(CONFIG_SC_LIB_CSP flash.c)
zephyr_library_sources_ifdef(CONFIG_SC_LIB_CSP eps.c)
zephyr_library_sources_ifdef(CONFIG_SC_LIB_CSP sys.c)
zephyr_library_sources_ifdef(CONFIG_SC_LIB_CSP upload.c)

zephyr_include_directories(.)

set(CSP_BUFFER_COUNT 1000 CACHE STRING "Number of total packet buffers")
set(CSP_QFIFO_LEN 1000 CACHE STRING "Length of incoming queue for router task")
zephyr_link_libraries(libcsp)
