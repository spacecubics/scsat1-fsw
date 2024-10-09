# Copyright (c) 2023 Space Cubics, LLC.
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(include)

add_subdirectory(lib)
add_subdirectory(drivers/can)
add_subdirectory(drivers/i2c)
add_subdirectory(drivers/spi)
add_subdirectory(drivers/pwm)
add_subdirectory(drivers/misc)
