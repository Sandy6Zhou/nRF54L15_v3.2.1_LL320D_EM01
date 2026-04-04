/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        my_comm.h
**文件描述:        LL311_BLE 工程统一头文件
**当前版本:        V1.0
**作    者:        Harrison Wu (wuyujiao@jimiiot.com)
**完成日期:        2026.01.15
*********************************************************************
** 功能描述:        集中引用所有模块头文件，便于统一管控
**                 包含：Main、BLE、Shell、Ctrl、LTE、NFC、GSensor 模块
*********************************************************************/

#ifndef _MY_COMMON_H_
#define _MY_COMMON_H_

/* ========== 系统头文件引用 ========== */
/* 标准C库 */
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <ctype.h>
#include <psa/crypto.h>

/* Zephyr核心 */
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>

/* Zephyr驱动 */
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/adc.h>

/* Zephyr系统功能 */
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_output_custom.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/poweroff.h>
#include <zephyr/sys/clock.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/zms.h>

/* Zephyr 设备及电源管理 */
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>

/* Zephyr shell */
#include <zephyr/shell/shell.h>

/* Zephyr蓝牙 */
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>

/* Zephyr MCUmgr */
#include <zephyr/mgmt/mcumgr/mgmt/callbacks.h>

/* Nordic SDK/HAL */
#include <hal/nrf_gpio.h>
#include <hal/nrf_reset.h>
#include <bluetooth/services/nus.h>
#include <soc.h>
#include <uart_async_adapter.h>

/* ========== 通用宏定义 ========== */
#define JM_SLEEP(timeout) k_sleep(timeout)
#define MY_MALLOC_BUFFER(PTR, BUFFER_SIZE) \
    {                                      \
        (PTR) = k_malloc((BUFFER_SIZE));   \
    }
#define MY_FREE_BUFFER(PTR) k_free(PTR)
#define MY_ASSERT_INFO(PARAM)  \
    {                          \
        if (!(PARAM))          \
        {                      \
            my_system_reset(); \
        }                      \
    } // TODO

/* ========== 模块类型枚举 ========== */
typedef enum
{
    MOD_MAIN,        // 主处理程序
    MOD_BLE,         // BLE处理程序
    MOD_CTRL,        // Control处理程序
    MOD_LTE,         // LTE处理程序
    MOD_NFC,         // NFC处理程序
    MOD_GSENSOR,     // G-Sensor处理程序
    MOD_FOTA,        // FOTA处理程序
    MAX_MY_MOD_TYPE, // 最大模块类型
} module_type;

/* ========== 告警类型枚举 ========== */
typedef enum
{
    ALARM_OPEN = 0,         // 拆壳告警
    ALARM_ILLEGALUNLOCK,    // 非法解锁告警
    ALARM_LOCK,             // 锁状态告警
    ALARM_MOTION,           // 运动状态告警
    ALARM_BATT,             // 电池状态告警
    ALARM_CHARGE,           // 充电状态告警
    ALARM_IMPACT,           // 撞击检测告警
    ALARM_SEPARATE,         // 蓝牙分离告警
    ALARM_NFC,              // NFC告警
    ALARM_CUT,              // 剪线告警
    ALARM_OTHER,            // 其他类型告警
} alarm_type_t;

/* ========== 定时器相关定义 ========== */
typedef enum
{
    MY_TIMER_ONE_MINUTE = 0, // 最核心定时器，一分钟定时器使用
    MY_TIMER_TEST,           // 1
    // MY_TIMER_WDT_FEED,       /* 看门狗喂狗定时器 */
    MY_TIMER_LTE_POWER,      // LTE电源控制定时器

    MY_TIMER_MAX_ID,
} MY_E_TIMER;

/* 消息ID定义 */
typedef enum
{
    MY_MSG_BASE_MSG = 0,
    MY_MSG_UART_READ_EVENT = MY_MSG_BASE_MSG + 1,
    MY_MSG_TEST,
    MY_MSG_ONE_MINUTE_TIMER,
    MY_MSG_GET_MDIMEI,
    MY_MSG_CLEAR_WDT,
    MY_MSG_POF_EVENT,
    MY_MSG_SYS_SHUTDOWN,
    MY_MSG_POWER_OFF,
    MY_MSG_SYS_REBOOT, // 10
    MY_MSG_BLE_DATA_EVENT,
    MY_MSG_CTRL_LED,    /* LED 控制消息 */
    MY_MSG_CTRL_BUZZER_MODE, /* 蜂鸣器控制消息 */
    MY_MSG_CTRL_BUZZER_ON,
    MY_MSG_CTRL_BUZZER_OFF,
    MY_MSG_WORK_MODE_SWITCH,
    MY_MSG_SHOW_CHARG, // 充电状态显示LED消息
    MY_MSG_UPDATE_BATTERY, // 更新电池状态消息
    MY_MSG_CTRL_LOCK_LED, // 锁LED控制消息
    MY_MSG_CLOSE_LED_SHOW, // 关闭所有LED显示功能消息
    MY_MSG_OPEN_LED_SHOW, // 打开所有LED显示功能消息



    /* LTE处理程序消息 */
    MY_MSG_RESET_LTE_TIMER,
    MY_MSG_LTE_PWRON,
    MY_MSG_LTE_PWROFF,
    MY_MSG_LTE_REV,

    /* G-Sensor处理程序消息 */
    MY_MSG_GSENSOR_PWRON,
    MY_MSG_GSENSOR_PWROFF,
    MY_MSG_GSENSOR_GET_MOTION_STATUS,
    MY_MSG_GSENSOR_INIT,
    MY_MSG_GSENSOR_READ, /* G-Sensor 读取六轴数据 */

    /* NFC处理程序消息 */
    MY_MSG_NFC_START_POLL,   /* 启动NFC轮询 */
    MY_MSG_NFC_STOP_POLL,    /* 停止NFC轮询 */
    MY_MSG_NFC_CARD_EVENT,   /* NFC卡片事件 */
    MY_MSG_NFC_POLL_TIMEOUT, /* NFC轮询超时 */
    MY_MSG_NFC_LED_SHOW,     /* NFC启动LED 显示 */

    /* CTRL处理程序消息 */
    MY_MSG_CTRL_KEY_SHORT_PRESS,       /* 按键短按事件 */
    MY_MSG_CTRL_KEY_LONG_PRESS,        /* 按键长按事件（2秒） */
    MY_MSG_CTRL_LIGHT_SENSOR_DARK,     /* 光传感器检测到黑暗环境 */
    MY_MSG_CTRL_LIGHT_SENSOR_BRIGHT,   /* 光传感器检测到光明环境 */
    MY_MSG_CTRL_LOCK_PIN_INSERTED,     /* 锁销插入检测 */
    MY_MSG_CTRL_LOCK_PIN_DISCONNECTED, /* 锁销断开检测 */
    MY_MSG_CTRL_SHUTDOWN_REQUEST,      /* 关机请求 */

    /* BLE 处理程序消息 */
    MY_MSG_BLE_RX,

    /* DFU OTA 状态消息 */
    MY_MSG_DFU_START,    /* DFU OTA 开始 */
    MY_MSG_DFU_TIMEOUT,  /* DFU OTA 超时退出 */
    MY_MSG_DFU_COMPLETE, /* DFU OTA 完成 */

    /* 开关锁状态消息 */
    MY_MSG_CTRL_OPENLOCKING,    /* 开锁中 */
    MY_MSG_CTRL_CLOSELOCKING,   /* 关锁中 */
    MY_MSG_CTRL_STOPLOCK,       /* 停止开/关锁 */
    MY_MSG_CTRL_OPENLOCKED,     /* 已开锁 */
    MY_MSG_CTRL_CLOSELOCKED,    /* 已关锁 */
    MY_MSG_BLE_LOCK_RESULT,     /* 蓝牙开/关锁结果通知消息 */

    MY_MSG_LTE_BLE_DATA,        /* 蓝牙指令数据 */
} MY_MAIN_TASK_MSG;

/* ========== 集中引用所有模块头文件 ========== */
#include "my_version.h"
#include "my_ring_buf.h"
#include "my_main.h"
#include "my_ble_core.h"
#include "my_shell.h"
#include "my_ctrl.h"
#include "my_lte.h"
#include "my_nfc.h"
#include "my_gsensor.h"
#include "my_motor.h"
#include "my_battery.h"
// #include "my_wdt.h"
#include "my_tool.h"
#include "my_zms_param.h"
#include "my_ble_app.h"
#include "my_cmd_setting.h"
#include "my_dfu_jimi.h"
#include "my_ble_log.h"
#include "my_pm.h"

#endif /* _MY_COMMON_H_ */
