# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

set(BOARD scsat1_main)
set(BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../../)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(scsat1-main)

target_sources(app PRIVATE src/main.c)
target_sources(app PRIVATE src/mgnm_test.c )
target_sources(app PRIVATE src/temp_test.c)
target_sources(app PRIVATE src/cv_test.c)
target_sources(app PRIVATE src/csp.c)
target_sources(app PRIVATE src/csp_test.c)
target_sources(app PRIVATE src/sunsens_test.c)
target_sources(app PRIVATE src/mtq.c)
target_sources(app PRIVATE src/mtq_test.c)
target_sources(app PRIVATE src/dstrx3_test.c)
target_sources(app PRIVATE src/scbus.c)
target_sources(app PRIVATE src/main_init.c)
target_sources(app PRIVATE src/loop_test.c)
target_sources(app PRIVATE src/syshk_test.c)
target_sources(app PRIVATE src/syshk.c)
