#
# Copyright (c) 2018 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(LL311_BLE)

target_include_directories(app PRIVATE
 ./inc
 ./src
 ./ext_module/nfc/api
 ./ext_module/nfc/drivers/FM175XX/inc
)

# NORDIC SDK APP START
target_sources(app PRIVATE
  src/main.c
  src/my_ring_buf.c
  src/my_shell.c
  src/my_ble_core.c
  src/my_lte.c
  src/my_gsensor.c
  src/my_gsensor_algorithm.c
  src/my_nfc.c
  src/my_ctrl.c
  src/my_motor.c
  src/my_battery.c
#  src/my_wdt.c
  src/my_tool.c
  src/my_cmd_setting.c
  src/my_zms_param.c
  ext_module/nfc/api/nfc_api.c
  ext_module/nfc/drivers/FM175XX/src/fm175xx_driver.c
  ext_module/nfc/drivers/FM175XX/src/nfc_reader_api.c
  ext_module/nfc/drivers/FM175XX/src/nfc_mifare.c
  ext_module/nfc/drivers/FM175XX/src/nfc_ntag.c
  src/my_ble_app.c
  src/my_ble_log.c
  src/my_dfu_jimi.c
  src/my_pm.c
  src/my_ble_scan.c
  src/my_flash_store.c
)

# NORDIC SDK APP END
