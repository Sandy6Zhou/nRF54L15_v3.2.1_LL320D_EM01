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
 ./ext_module/imu/api
 ./ext_module/imu/port
 ./ext_module/imu/vendor
 ./ext_module/temp_humi/api
 ./ext_module/temp_humi/drivers/SHTC3/inc
 ./ext_module/barometer/api
 ./ext_module/barometer/drivers/SPA06/inc
#  ./ext_module/accelerometer/api
#  ./ext_module/accelerometer/drivers/DA213/inc
)

# NORDIC SDK APP START
target_sources(app PRIVATE
  src/main.c
  src/my_ring_buf.c
  src/my_shell.c
  src/my_ble_core.c
  src/my_lte.c
  src/my_magnetic_uart.c
  src/my_gsensor.c
#  src/my_gsensor_algorithm.c
  src/my_ctrl.c
  src/my_battery.c
  src/my_wdt.c
  src/my_tool.c
  src/my_cmd_setting.c
  src/my_zms_param.c
  src/my_ble_app.c
  src/my_ble_log.c
  src/my_dfu_jimi.c
  src/my_pm.c
  src/my_ble_scan.c
  ext_module/imu/api/imu_api.c
  ext_module/imu/port/bmi325_port.c
  ext_module/imu/vendor/bmi325.c
  ext_module/imu/vendor/bmi3.c
  ext_module/temp_humi/api/temp_humi_api.c
  ext_module/temp_humi/drivers/SHTC3/src/shtc3_driver.c
  ext_module/barometer/api/barometer_api.c
  ext_module/barometer/drivers/SPA06/src/spa06_driver.c
  # ext_module/accelerometer/api/accelerometer_api.c
  # ext_module/accelerometer/drivers/DA213/src/da213_driver.c
  src/my_flash_store.c
)

# NORDIC SDK APP END
