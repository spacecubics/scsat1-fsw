# Copyright (c) 2023 Space Cubics, LLC.
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(include)

add_subdirectory(lib)
add_subdirectory(drivers/can)
add_subdirectory(drivers/i2c)
add_subdirectory(drivers/spi)
add_subdirectory(drivers/pwm)
add_subdirectory(drivers/misc)

execute_process(
	COMMAND git rev-parse --short HEAD
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	OUTPUT_VARIABLE SCSAT1_FSW_GIT_HASH
	OUTPUT_STRIP_TRAILING_WHITESPACE
)
zephyr_compile_definitions(SCSAT1_FSW_GIT_HASH="${SCSAT1_FSW_GIT_HASH}")
