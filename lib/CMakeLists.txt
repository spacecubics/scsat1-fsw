# Copyright (c) 2024 Space Cubics, LLC.
# SPDX-License-Identifier: Apache-2.0

add_subdirectory(core)
add_subdirectory_ifdef(CONFIG_SC_LIB_FLASH flash)
add_subdirectory_ifdef(CONFIG_SC_LIB_CSP csp)
add_subdirectory_ifdef(CONFIG_SC_LIB_SUNSENS sunsens)
add_subdirectory_ifdef(CONFIG_SC_LIB_MGNM mgnm)
add_subdirectory_ifdef(CONFIG_SC_LIB_MTQ mtq)
add_subdirectory_ifdef(CONFIG_SC_LIB_GNSS gnss)
add_subdirectory_ifdef(CONFIG_SC_LIB_IMU imu)
