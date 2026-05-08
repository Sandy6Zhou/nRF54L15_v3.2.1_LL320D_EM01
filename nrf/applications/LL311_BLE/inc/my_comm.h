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
#include <zephyr/bluetooth/addr.h>

/* Zephyr MCUmgr */
#include <zephyr/mgmt/mcumgr/mgmt/callbacks.h>

/* Nordic SDK/HAL */
#include <hal/nrf_gpio.h>
#include <hal/nrf_reset.h>
#include <bluetooth/services/nus.h>
#include <soc.h>
#include <uart_async_adapter.h>

// 包含 FM175XX 相关头文件
#include "../inc/fm175xx_driver.h"
#include "../inc/fm175xx_reg.h"

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
    ALARM_OPEN = 1,         // 拆壳告警
    ALARM_ILLEGALUNLOCK,    // 非法解锁告警
    ALARM_LOCK,             // 锁状态告警
    ALARM_MOTION,           // 运动状态告警
    ALARM_BATT,             // 电池状态告警
    ALARM_CHARGE,           // 充电状态告警
    ALARM_IMPACT,           // 撞击检测告警
    ALARM_NFC,              // NFC告警
    ALARM_CUT,              // 剪线告警
    ALARM_LOCKPIN,          // 锁销状态告警
    ALARM_OTHER,            // 其他类型告警
} alarm_type_t;

/* ========== 4G开机/唤醒协议相关枚举 ========== */
typedef enum
{
    LTE_PWR_STATE_NORMAL = 0,   // 正常上电
    LTE_PWR_STATE_ABNORMAL = 1, // 异常重启
} lte_power_state_t;

typedef enum
{
    LTE_PWRON_REASON_BT = 0,    // 蓝牙上电
    LTE_PWRON_REASON_USB = 1,   // USB上电
    LTE_PWRON_REASON_OTHER = 2, // 其他(预留)
} lte_poweron_reason_t;

typedef enum
{
    LTE_BOOT_REASON_CONTINUOUS = 0, // 连续追踪模式间隔定位上报
    LTE_BOOT_REASON_LONG_LIFE = 1,  // 长续航模式间隔定位上报
    LTE_BOOT_REASON_SMART = 2,      // 智能模式间隔定位上报
    LTE_BOOT_REASON_ALARM = 3,      // 告警事件唤醒
    LTE_BOOT_REASON_SCAN = 4,       // 扫描数据上报
    LTE_BOOT_REASON_RESERVED = 255, // 预留(未知原因)
} lte_boot_reason_t;

/* ========== 定时器相关定义 ========== */
typedef enum
{
    MY_TIMER_ONE_MINUTE = 0, // 最核心定时器，一分钟定时器使用
    MY_TIMER_TEST,           // 1
    // MY_TIMER_WDT_FEED,       /* 看门狗喂狗定时器 */
    MY_TIMER_LTE_POWER,      // LTE电源控制定时器
    MY_TIMER_SHUTDOWN,       // 关机定时器
    MY_TIMER_LTE_PULSE,       // LTE脉冲定时器

    /* 扫描定时器 */
    MY_TIMER_SCAN_INTERVAL,   // 周期扫描定时器
    MY_TIMER_SCAN_LENGTH,     // 单次扫描时长定时器
    MY_TIMER_UPLOAD_INTERVAL, // 上报间隔定时器

    // G-Sensor 专用定时器
    MY_TIMER_GSENSOR_SAMPLE,  // G-Sensor 周期采样定时器
    MY_TIMER_GSENSOR_BURST,   // G-Sensor 批量采样定时器

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
    MY_MSG_VERIFY_UNLOCK, // 处理开锁规则



    /* LTE处理程序消息 */
    MY_MSG_RESET_LTE_TIMER,
    MY_MSG_LTE_PWRON,
    MY_MSG_LTE_PWROFF,
    MY_MSG_LTE_REV,
    MY_MSG_RETRANS_CHECK,
    MY_MSG_ADD_RETRANS_QUEUE,

    /* G-Sensor处理程序消息 */
    MY_MSG_GSENSOR_PWRON,
    MY_MSG_GSENSOR_PWROFF,
    MY_MSG_GSENSOR_READ, /* G-Sensor 读取六轴数据 */
    MY_MSG_GSENSOR_WAKEUP_INT, /* G-Sensor 唤醒中断消息 */
    MY_MSG_MODESET_UPDATE,  // 处理智能模式下参数设置更新

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

    // 处理4G过来LTE+CMD数据透传
    MY_MSG_LTE_CMD_RX,
    MY_MSG_LTE_CMD_ASYNC_RESP,
    MY_MSG_BLE_NFCTRIG_EVENT,  // 处理NFC联动指令

    //处理透传mac和tag数据传输（单条上报)
    MY_MSG_UPLOAD_TAG_AND_MAC,

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
    MY_MSG_BLE_PACKET_TIMEOUT,  /* BLE包传输应答超时 */
    MY_MSG_LTE_PULSE,             /* LTE脉冲消息 */

    MY_MSG_SHUTDOWN,            /* 关机消息 */

    /* 扫描处理程序消息 */
    MY_MSG_TAG_SCAN_PROCESS,    /* TAG扫描数据处理消息 */
    MY_MSG_TRAN_MAC_PROCESS,    /* 透传MAC扫描数据处理消息 */
    MY_MSG_SCAN_INTERVAL,       /* 周期扫描定时器消息 */
    MY_MSG_SCAN_LENGTH,         /* 单次扫描时长定时器消息 */
    MY_MSG_SCAN_UPLOAD,         /* 上报间隔定时器消息 */
    MY_MSG_UPLOAD_WAKEUP,       /* 扫描数据在LTE唤醒时顺便上报消息 */
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
#include "my_ble_app.h"
#include "my_cmd_setting.h"
#include "my_dfu_jimi.h"
#include "my_ble_log.h"
#include "my_pm.h"
#include "my_ble_scan.h"
#include "my_zms_param.h"

#endif /* _MY_COMMON_H_ */
