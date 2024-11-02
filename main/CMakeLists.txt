# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

set(BOARD scsat1_main)
set(BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR}/..)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(scsat1-main)

zephyr_include_directories(src/tlm)
zephyr_include_directories(src/mon)

target_sources(app PRIVATE src/main.c)
target_sources(app PRIVATE src/csp.c)
target_sources(app PRIVATE src/cmd/handler.c)
target_sources(app PRIVATE src/cmd/pwrctrl.c)
target_sources(app PRIVATE src/tlm/syshk.c)
target_sources(app PRIVATE src/mon/monitor.c)
target_sources(app PRIVATE src/mon/temp.c)
