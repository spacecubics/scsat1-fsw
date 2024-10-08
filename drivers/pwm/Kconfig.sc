# Space Cubics PWM configuration options

# Copyright (c) 2023 Space Cubics,LLC
# SPDX-License-Identifier: Apache-2.0

menuconfig PWM_SC
	bool "Space Cubics PWM Driver"
	default y
	depends on DT_HAS_SC_PWM_ENABLED
	help
	  Enable Space Cubics PWM Driver.
