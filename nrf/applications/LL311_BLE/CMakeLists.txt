#
# Copyright (c) 2018 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(LL311_BLE)

# NORDIC SDK APP START
target_sources(app PRIVATE
  src/main.c
  src/my_shell.c
  src/my_ble_core.c
  src/my_lte.c
  #src/my_gsensor.c
  #src/my_nfc.c
  src/my_ctrl.c
)

# NORDIC SDK APP END
