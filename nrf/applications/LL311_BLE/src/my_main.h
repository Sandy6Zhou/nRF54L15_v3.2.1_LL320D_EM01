/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        my_main.h
**文件描述:        main.c头文件声明
**当前版本:        V1.0
**作    者:        Harrison Wu (wuyujiao@jimiiot.com)
**完成日期:        2026.01.15
*********************************************************************
** 功能描述:        系统主任务处理
*********************************************************************/

#ifndef _MY_MAIN_H_
#define _MY_MAIN_H_

#include "my_comm.h"
#include <stddef.h>
#include <zephyr/types.h>

/* 任务栈大小定义 */
#define MY_MAIN_TASK_STACK_SIZE    8 * 1024
#define MY_SHELL_TASK_STACK_SIZE   2 * 1024
#define MY_BLE_TASK_STACK_SIZE     CONFIG_BT_NUS_THREAD_STACK_SIZE
#define MY_CTRL_TASK_STACK_SIZE    1 * 1024
#define MY_LTE_TASK_STACK_SIZE     8 * 1024
#define MY_NFC_TASK_STACK_SIZE     8 * 1024
#define MY_GSENSOR_TASK_STACK_SIZE 8 * 1024

/* 任务优先级定义 */
#define MY_MAIN_TASK_PRIORITY    7
#define MY_SHELL_TASK_PRIORITY   7
#define MY_BLE_TASK_PRIORITY     5
#define MY_CTRL_TASK_PRIORITY    5
#define MY_LTE_TASK_PRIORITY     5
#define MY_NFC_TASK_PRIORITY     5
#define MY_GSENSOR_TASK_PRIORITY 5

/* 定时器回调函数类型定义 */
typedef void (*TIMER_FUN)(void *param);

/* 消息结构体定义 */
typedef struct MSG_S
{
    uint32_t msgID;
    void *pData;
    uint32_t DataLen;
} MSG_S;

/* 语言类型定义 */
typedef enum
{
    MY_LANG_ENGLISH,      // 英语
    MY_LANG_SIMP_CHINESE, // 简体中文
    MY_MAX_LANG,
} my_lang_type;

/* 工作模式定义 */
typedef enum
{
    MY_WORK_NORMAL = 0, // 工作模式
    MY_LOW_POWER,       // 低功耗模式
} MY_WORK_MODE;

/*********************************************************************
**函数名称:  my_system_reset
**入口参数:  无
**出口参数:  无
**函数功能:  系统复位函数
*********************************************************************/
void my_system_reset(void);

/*********************************************************************
**函数名称:  custom_task_info_init
**入口参数:  无
**出口参数:  无
**函数功能:  初始化任务数据信息结构
*********************************************************************/
void custom_task_info_init(void);

/*********************************************************************
**函数名称:  my_init_msg_handler
**入口参数:  mod      --  模块类型
**           msgq     --  消息队列
**出口参数:  无
**函数功能:  初始化模块消息处理函数
*********************************************************************/
void my_init_msg_handler(module_type mod, struct k_msgq *msgq);

/*********************************************************************
**函数名称:  my_send_msg
**入口参数:  src_mod_id   --  发送消息的源模块ID
**           dest_mod_id  --  接收消息的目标模块ID
**           msg          --  消息ID
**出口参数:  无
**函数功能:  向指定模块发送简单消息 (不带附加数据)
*********************************************************************/
void my_send_msg(module_type src_mod_id, module_type dest_mod_id, uint32_t msg);

/*********************************************************************
**函数名称:  my_send_msg_data
**入口参数:  src_mod_id   --  发送消息的源模块ID
**           dest_mod_id  --  接收消息的目标模块ID
**           msg          --  消息结构体指针 (MSG_S)
**出口参数:  无
**函数功能:  向指定模块发送包含数据的完整消息结构
*********************************************************************/
void my_send_msg_data(module_type src_mod_id, module_type dest_mod_id, MSG_S *msg);

/*********************************************************************
**函数名称:  my_recv_msg
**入口参数:  msg_queue    --  消息队列
**           msg          --  消息结构体指针 (MSG_S)
**           msg_size     --  消息结构体大小
**           wait_option  --  等待选项
**出口参数:  无
**函数功能:  从指定消息队列接收消息
*********************************************************************/
int my_recv_msg(void *msg_queue, void *msg, uint32_t msg_size, k_timeout_t wait_option);

/*********************************************************************
**函数名称:  my_start_timer
**入口参数:  timerId    --  定时器ID
**           ms         --  定时器超时时间 (单位: 毫秒)
**           isPeriod   --  是否重复定时
**           timer_fun  --  定时器超时回调函数
**出口参数:  无
**函数功能:  启动指定定时器
*********************************************************************/
int my_start_timer(int timerId, uint32_t ms, bool isPeriod, TIMER_FUN timer_fun);

/*********************************************************************
**函数名称:  my_stop_timer
**入口参数:  timerId    --  定时器ID
**出口参数:  无
**函数功能:  停止指定定时器
*********************************************************************/
void my_stop_timer(int timerId);

/*********************************************************************
**函数名称:  my_delete_timer
**入口参数:  timerId    --  定时器ID
**出口参数:  无
**函数功能:  停止并删除指定定时器
*********************************************************************/
void my_delete_timer(int timerId);

/*********************************************************************
**函数名称:  my_time_is_run
**入口参数:  timerId    --  定时器ID
**出口参数:  无
**函数功能:  检查指定定时器是否正在运行
**返 回 值:  true 表示正在运行，false 表示未运行或不存在
*********************************************************************/
bool my_time_is_run(int timerId);

#endif /* _MY_MAIN_H_ */
