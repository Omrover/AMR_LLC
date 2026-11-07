# SPDX-License-Identifier: Apache-2.0
# OpenSDA (K26) de la FRDM-A en modo CMSIS-DAP -> pyocd; J-Link por el conector JTAG.
board_runner_args(pyocd "--target=s32k344")
board_runner_args(jlink "--device=S32K344" "--reset-after-load")

include(${ZEPHYR_BASE}/boards/common/pyocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
