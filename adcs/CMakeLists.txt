# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

set(BOARD scsat1_adcs)
set(BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR}/..)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(scsat1-adcs)

target_sources(app PRIVATE src/main.c)
target_sources(app PRIVATE src/csp.c)
target_sources(app PRIVATE src/cmd/handler.c)
target_sources(app PRIVATE src/cmd/pwrctrl.c)
