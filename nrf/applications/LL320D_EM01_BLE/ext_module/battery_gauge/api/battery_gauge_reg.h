/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        battery_gauge_reg.h
**文件描述:        OM70201WV 电量计私有寄存器定义头文件
**当前版本:        V1.0
**作    者:       周森达 (zhousenda@jimiiot.com)
**完成日期:        2026.07.21
*********************************************************************
** 功能描述:       定义 API 层使用的中断配置寄存器地址
**                 定义中断状态位和中断使能位掩码
*********************************************************************/

#ifndef _BATTERY_GAUGE_REG_H_
#define _BATTERY_GAUGE_REG_H_

#define BATTERY_GAUGE_REG_INTERRUPT_CONFIG 0x0AU
#define BATTERY_GAUGE_REG_CONFIG 0x08U
#define BATTERY_GAUGE_REG_SOC_ALERT 0x0BU
#define BATTERY_GAUGE_REG_TEMP_MAX 0x0CU
#define BATTERY_GAUGE_REG_TEMP_MIN 0x0DU
#define BATTERY_GAUGE_REG_CYCLE_COUNT_HIGH 0xA4U

#define BATTERY_GAUGE_CONFIG_ACTIVE_MODE_MASK 0x02U
#define BATTERY_GAUGE_CONFIG_CYCLE_COUNT_INIT_MASK 0x10U

#define BATTERY_GAUGE_INTERRUPT_STATUS_MASK 0x07U
#define BATTERY_GAUGE_INTERRUPT_LOW_TEMP_ENABLE_MASK 0x08U
#define BATTERY_GAUGE_INTERRUPT_HIGH_TEMP_ENABLE_MASK 0x10U
#define BATTERY_GAUGE_INTERRUPT_SOC_ENABLE_MASK 0x20U
#define BATTERY_GAUGE_INTERRUPT_TEMP_SELECT_MASK 0x80U

#endif
