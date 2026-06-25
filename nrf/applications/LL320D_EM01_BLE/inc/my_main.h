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

/*直包含必要的头文件，避免循环包含 */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

/* 任务栈大小定义 */
#define MY_MAIN_TASK_STACK_SIZE    4 * 1024 // 先改为4K，未来开发过程中不够再调整
#define MY_BLE_TASK_STACK_SIZE     4 * 1024 // 2K测试空间不够，暂修改为4K
#define MY_CTRL_TASK_STACK_SIZE    1 * 1024 // 先改为4K，未来开发过程中不够再调整
#define MY_LTE_TASK_STACK_SIZE     8 * 1024
#define MY_GSENSOR_TASK_STACK_SIZE 4 * 1024 // 先改为4K，未来开发过程中不够再调整

/* 任务优先级定义 */
#define MY_MAIN_TASK_PRIORITY    7
#define MY_BLE_TASK_PRIORITY     5
#define MY_CTRL_TASK_PRIORITY    5
#define MY_LTE_TASK_PRIORITY     5
#define MY_GSENSOR_TASK_PRIORITY 5

/* 定时器回调函数类型定义 */
typedef void (*TIMER_FUN)(void *param);

/* 消息结构体定义 */
typedef struct
{
    uint32_t msgID;
    void *pData;
    uint32_t DataLen;
} msg_t;

/* 工作模式定义 */
typedef enum
{
    MY_MODE_CONTINUOUS = 0,     // 模式0：连续追踪模式，Cell+GNSS常开，时间+距离双条件上报
    MY_MODE_LONG_LIFE = 1,      // 模式1：长续航模式，定时唤醒定位
    MY_MODE_SMART = 2,          // 模式2：智能模式（默认），G-Sensor区分静止/运动，子模式控制断电策略
    MY_MODE_ALWAYS_ONLINE = 3,  // 模式3：常在线模式，4G永久在线，GNSS仅告警唤醒
    MY_MODE_MAX,
    MY_MODE_SHUTDOWN = 0xFF,    // 关机模式（独立于工作模式），所有功能关闭，长按2秒唤醒
} work_mode_t;

// 连续追踪模式参数结构体
typedef struct
{
    uint32_t reporting_interval_sec;    // 上传间隔，单位：秒（非负整数）
    uint32_t reporting_interval_dis;    // 上传距离，单位：米（非负整数）
} continuous_tracking_mode_t;

// 长续航模式参数结构体
typedef struct {
    uint32_t reporting_interval_min;    // 上传间隔，单位：分钟（5~1440），默认240
    char start_time[5];                 // 开始时间，格式HHMM（24小时制，如"0001"），长度5含字符串结束符
    uint8_t gnss_sw;                    // GNSS开关：1=ON, 0=OFF，默认ON（由4G侧控制GPS启停）
} long_battery_mode_t;

// 智能模式参数结构体
typedef struct {
    uint8_t sub_mode;               // 子模式 0~5，控制Cell/GNSS断电策略和间隔单位，默认5
    uint32_t static_interval;       // 静止状态上报间隔（存储原始值，单位由子模式决定：0~4为分钟3~60，5为秒10~86400），0=静止不上报，默认5
    uint32_t moving_interval;       // 运动状态上报间隔（存储原始值，单位由子模式决定：0~1为分钟3~60，2~5为秒10~86400），默认5
} intelligent_mode_t;

// 设备工作模式配置结构体
typedef struct {
    work_mode_t current_mode;
    continuous_tracking_mode_t continuous_tracking; // 连续追踪模式
    long_battery_mode_t long_battery;          // 长续航模式
    intelligent_mode_t intelligent;           // 智能模式
} device_work_mode_config_t;

/* 上报方式枚举定义 */
typedef enum
{
    REPORT_MODE_NONE = 0,       /* 0-不上报 */
    REPORT_MODE_GPRS,           /* 1-GPRS */
    REPORT_MODE_GPRS_SMS,       /* 2-GPRS+SMS */
    REPORT_MODE_GPRS_SMS_CALL,  /* 3-GPRS+SMS+CALL */
} report_mode_t;

/* 报警模式枚举定义 */
typedef enum
{
    ALARM_NONE = 0,         /* 0-不报警 */
    ALARM_TEMPORARY,    /* 1-报警30s */
    ALARM_CONTINUOUS,   /* 2-持续报警 */
} alarm_mode_t;

/* Empty状态触发方式枚举定义 */
typedef enum
{
    EMPTY_TRIGGER_NONE = 0,         /* 0-不触发 */
    EMPTY_TRIGGER_ONLINE,           /* 1-在线触发 */
    EMPTY_TRIGGER_CHANGE,           /* 2-状态变化触发 */
} empty_trigger_mode_t;

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
**           msg          --  消息结构体指针 (msg_t)
**出口参数:  无
**函数功能:  向指定模块发送包含数据的完整消息结构
*********************************************************************/
void my_send_msg_data(module_type src_mod_id, module_type dest_mod_id, msg_t *msg);

/*********************************************************************
**函数名称:  my_recv_msg
**入口参数:  msg_queue    --  消息队列
**           msg          --  消息结构体指针 (msg_t)
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
void switch_work_mode(work_mode_t mode);

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
**函数名称:  handle_always_online_mode
**入口参数:  无
**出口参数:  无
**函数功能:  处理常在线模式逻辑，4G永久在线，G-Sensor正常采样支持告警
*********************************************************************/
void handle_always_online_mode(void);

/*********************************************************************
**函数名称:  awaken_lte_timer_callback
**入口参数:  timer    --  定时器指针
**出口参数:  无
**函数功能:  LTE唤醒定时器回调函数，用于唤醒LTE模块
*********************************************************************/
void awaken_lte_timer_callback(void *timer);

/*********************************************************************
**函数名称:  send_work_mode_command
**入口参数:  mode     --  要切换到的工作模式
**出口参数:  无
**函数功能:  发送工作模式给LTE模块
*********************************************************************/
void send_work_mode_command(work_mode_t mode);

#endif /* _MY_MAIN_H_ */
