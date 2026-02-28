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

/* 任务栈大小定义 */
#define MY_MAIN_TASK_STACK_SIZE    8 * 1024
#define MY_BLE_TASK_STACK_SIZE     CONFIG_BT_NUS_THREAD_STACK_SIZE
#define MY_CTRL_TASK_STACK_SIZE    1 * 1024
#define MY_LTE_TASK_STACK_SIZE     8 * 1024
#define MY_NFC_TASK_STACK_SIZE     8 * 1024
#define MY_GSENSOR_TASK_STACK_SIZE 8 * 1024

/* 任务优先级定义 */
#define MY_MAIN_TASK_PRIORITY    7
#define MY_BLE_TASK_PRIORITY     5
#define MY_CTRL_TASK_PRIORITY    5
#define MY_LTE_TASK_PRIORITY     5
#define MY_NFC_TASK_PRIORITY     5
#define MY_GSENSOR_TASK_PRIORITY 5

/* 定时器回调函数类型定义 */
typedef void (*TIMER_FUN)(void *param);

#define DEFAULT_LONG_LIFE_INTERVAL      (4 * 60)
#define DEFAULT_START_TIME              "0001"

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
    MY_MODE_CONTINUOUS,     // 连续追踪模式
    MY_MODE_LONG_LIFE,      // 长续航模式
    MY_MODE_SMART,          // 智能模式
} MY_WORK_MODE;

// 长续航模式参数结构体
typedef struct {
    uint32_t reporting_interval_min;    // 上传间隔，单位：分钟（非负整数）
    char start_time[5];                 // 开始时间，格式HHMM（24小时制，如"0001"），长度5含字符串结束符
} LongBatteryMode;

// 智能模式参数结构体
typedef struct {
    uint32_t stop_status_interval_sec;  // 停止状态上传间隔，单位：秒（非负整数）
    uint32_t land_status_interval_sec;  // 陆运状态上传间隔，单位：秒（非负整数）
    uint32_t sea_status_interval_sec;   // 海运状态上传间隔，单位：秒（非负整数）
    uint8_t sleep_switch;               // 休眠开关，可设置范围：0/1/2
} IntelligentMode;

// 设备工作模式配置结构体
typedef struct {
    MY_WORK_MODE current_mode;
    // 连续追踪模式的配置信息跟nordic无关,无需保存
    LongBatteryMode long_battery;          // 长续航模式
    IntelligentMode intelligent;           // 智能模式
} DeviceWorkModeConfig;

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

/*********************************************************************
**函数名称:  switch_work_mode
**入口参数:  mode     --  要切换到的工作模式
**出口参数:  无
**函数功能:  切换工作模式，通过消息机制通知主线程
*********************************************************************/
void switch_work_mode(MY_WORK_MODE mode);

/*********************************************************************
**函数名称:  lte_power_check_timer_callback
**入口参数:  timer    --  定时器指针
**出口参数:  无
**函数功能:  LTE电源检测定时器回调函数
*********************************************************************/
void lte_power_check_timer_callback(void *timer);

/*********************************************************************
**函数名称:  handle_long_life_mode
**入口参数:  无
**出口参数:  无
**函数功能:  处理长续航模式逻辑
*********************************************************************/
void handle_long_life_mode(void);

/*********************************************************************
**函数名称:  handle_smart_mode
**入口参数:  无
**出口参数:  无
**函数功能:  处理智能模式逻辑
*********************************************************************/
void handle_smart_mode(void);

/*********************************************************************
**函数名称:  handle_continuous_mode
**入口参数:  无
**出口参数:  无
**函数功能:  处理连续追踪模式逻辑
*********************************************************************/
void handle_continuous_mode(void);

/*********************************************************************
**函数名称:  awaken_lte_timer_callback
**入口参数:  timer    --  定时器指针
**出口参数:  无
**函数功能:  LTE唤醒定时器回调函数，用于唤醒LTE模块
*********************************************************************/
void awaken_lte_timer_callback(void *timer);

/*********************************************************************
**函数名称:  get_workmode_config_ptr
**入口参数:  无
**出口参数:  无
**函数功能:  获取设备工作模式配置结构体指针
**返 回 值:  返回 DeviceWorkModeConfig 结构体指针
*********************************************************************/
DeviceWorkModeConfig* get_workmode_config_ptr(void);

#endif /* _MY_MAIN_H_ */
