/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        my_battery.h
**文件描述:        电池管理模块实现头文件
**当前版本:        V1.0
**作    者:        周森达 (zhousenda@jimiiot.com)
**完成日期:        2026.02.04
*********************************************************************/

#ifndef _MY_BATTERY_H_
#define _MY_BATTERY_H_

int batt_read_mv(int32_t *mv);
int ntc_read_raw(int16_t *raw);
int batt_adc_init(void);
void batt_enable(bool on);
int batt_gpio_init(void);

#endif