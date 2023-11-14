# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

set(BOARD scsat1_main)

get_filename_component(APPLICATION_PROJECT_DIR
                       ${CMAKE_CURRENT_LIST_DIR}/../../../
                       ABSOLUTE
)
set(BOARD_ROOT ${APPLICATION_PROJECT_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(scsat1-main)

target_sources(app PRIVATE src/main.c)
target_sources(app PRIVATE src/pwrctrl.c)
target_sources(app PRIVATE src/mgnm.c )
target_sources(app PRIVATE src/mgnm_test.c )
