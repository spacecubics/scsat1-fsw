# SPDX-License-Identifier: Apache-2.0

if(CONFIG_SPI)
  zephyr_library_amend()
  zephyr_library_include_directories(${ZEPHYR_BASE}/drivers/spi)
  zephyr_library_sources_ifdef(CONFIG_SPI_SC_QSPI spi_sc.c)
endif()
