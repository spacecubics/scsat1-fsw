# Space Cubics I2C configuration options

# Copyright (c) 2023 Space Cubics,LLC
# SPDX-License-Identifier: Apache-2.0

config I2C_SC
	bool "Space Cubics I2C Master Driver"
	default y
	depends on DT_HAS_SC_I2C_ENABLED
	help
	  Enable Space Cubics I2C Master Driver.

config I2C_SC_TRANSFER_TIMEOUT_MS
	int "I2C Transfer timeout [ms]"
	default 100
	help
	  Timeout in milliseconds used for each I2C transfer.
