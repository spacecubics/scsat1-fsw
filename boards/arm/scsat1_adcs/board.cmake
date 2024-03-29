# SPDX-License-Identifier: Apache-2.0

if(CONFIG_BOARD_SCSAT1_ADCS)
  board_runner_args(openocd "--use-elf" "--config=${BOARD_DIR}/support/openocd-ftdi.cfg")

  include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
endif()
