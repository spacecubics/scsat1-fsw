# Copyright (c) 2023 Space Cubics, LLC.
# SPDX-License-Identifier: Apache-2.0

menuconfig SPI_SC_QSPI
	bool "Space Cubics QSPI driver"
	default y
	depends on DT_HAS_SC_QSPI_ENABLED
	help
	  Enable Space Cubics QSPI driver.

config SPI_SC_IDLE_TIMEOUT_MS
	int "Timeout duration for waiting until the SPI Bus becomes idle.[ms]"
	default 100
	help
	  Timeout duration for waiting until the SPI Bus becomes idle. [ms]
