/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        my_common.h
**文件描述:        main.c头文件声明
**当前版本:        V1.0
**作    者:        Harrison Wu (wuyujiao@jimiiot.com)
**完成日期:        2026.01.15
*********************************************************************
** 功能描述:        系统主任务处理
*********************************************************************/

#ifndef _MY_COMMON_H_
#define _MY_COMMON_H_

#include <stddef.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>

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

#define JM_SLEEP  k_sleep
#define JM_MALLOC k_malloc
#define JM_FREE   k_free

typedef enum
{
    MOD_MAIN,        // 主处理程序
    MOD_SHELL,       // Shell处理程序
    MOD_BLE,         // BLE处理程序
    MOD_CTRL,        // Control处理程序
    MOD_LTE,         // LTE处理程序
    MOD_NFC,         // NFC处理程序
    MOD_GSENSOR,     // G-Sensor处理程序
    MOD_FOTA,        // FOTA处理程序
    MAX_MY_MOD_TYPE, // 最大模块类型
} module_type;

/* 定时器ID定义 */
typedef enum
{
    MY_TIMER_ONE_MINUTE = 0, // 最核心定时器，一分钟定时器使用
    MY_TIMER_TEST,           // 1

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

} MY_MAIN_TASK_MSG;

#endif /* _MY_COMMON_H_ */
