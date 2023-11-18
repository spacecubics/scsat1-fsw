# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

set(BOARD scsat1_adcs)

get_filename_component(APPLICATION_PROJECT_DIR
                       ${CMAKE_CURRENT_LIST_DIR}/../../../
                       ABSOLUTE
)
set(BOARD_ROOT ${APPLICATION_PROJECT_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(scsat1-adcs)

target_sources(app PRIVATE src/main.c)
target_sources(app PRIVATE src/pwrctrl.c)
target_sources(app PRIVATE src/sysmon.c)
target_sources(app PRIVATE src/temp.c)
target_sources(app PRIVATE src/temp_test.c)
target_sources(app PRIVATE src/cv.c)
target_sources(app PRIVATE src/cv_test.c)
target_sources(app PRIVATE src/imu.c)
target_sources(app PRIVATE src/imu_test.c)
target_sources(app PRIVATE src/gnss.c)
target_sources(app PRIVATE src/gnss_test.c)
target_sources(app PRIVATE src/csp.c)
target_sources(app PRIVATE src/rw.c)
target_sources(app PRIVATE src/rw_test.c)
