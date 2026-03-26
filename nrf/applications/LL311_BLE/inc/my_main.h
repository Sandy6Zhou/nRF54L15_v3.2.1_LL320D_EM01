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
#define MY_NFC_TASK_STACK_SIZE     4 * 1024 // 先改为4K，未来开发过程中不够再调整
#define MY_GSENSOR_TASK_STACK_SIZE 4 * 1024 // 先改为4K，未来开发过程中不够再调整

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
    MY_MODE_SHUTDOWN,       // 关机模式，该模式下所有功能关闭，只有按键可以唤醒设备，长按2秒开机，开机后智能模式，这种主要应用在仓储模式，进入超低功耗模式
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

/* NFC卡权限结构体 */
typedef struct
{
    char     nfc_no[16];           /* NFC卡号 */
    int32_t  lat;                  /* 纬度，单位：微度 */
    int32_t  lon;                  /* 经度，单位：微度 */
    uint8_t  lat_lon_valid;        /* 经纬度有效标志: 0-不限制, 1-有效 */
    uint32_t radius;               /* 半径，单位米 */
    char     start_time[12];       /* 开始时间，格式YYMMDDHHMM */
    char     end_time[12];         /* 结束时间，格式YYMMDDHHMM */
    uint8_t  time_valid;           /* 起止时间有效标志: 0-不限制, 1-有效 */
    int16_t  unlock_times;         /* 可用次数，-1表示不限次数 */
} NfcAuthCard;

/* 设备指令配置结构体 */
typedef struct
{
    /* REMALM 指令配置 */
    uint8_t remalm_sw;          /* 防拆报警开关: 0-OFF, 1-ON */
    uint8_t remalm_mode;        /* 报警上报方式: 0-GPRS, 1-GPRS+SMS, 2-GPRS+SMS+CALL */

    /* LOCKPINCYT 指令配置 */
    uint8_t lockpincyt_report;   /* 锁销非法拔除上报方式: 0-不上报, 1-GPRS, 2-GPRS+SMS, 3-GPRS+SMS+CALL */
    uint8_t lockpincyt_buzzer;   /* 锁销非法拔除蜂鸣器报警方式: 0-不报警, 1-报警30s, 2-持续报警 */

    /* LOCKERR 指令配置 */
    uint8_t lockerr_report;     /* 锁状态异常上报方式: 0-不上报, 1-GPRS, 2-GPRS+SMS, 3-GPRS+SMS+CALL */
    uint8_t lockerr_buzzer;     /* 锁状态异常蜂鸣器报警方式: 0-不报警, 1-报警30s, 2-持续报警 */

    /* PINSTAT 指令配置 */
    uint8_t pinstat_report;     /* 锁销状态上报方式: 0-不上报, 1-GPRS, 2-GPRS+SMS, 3-GPRS+SMS+CALL */
    uint8_t pinstat_trigger;    /* 锁销状态触发方式: 0-都不触发, 1-插入触发, 2-拔出触发, 3-插入拔出均触发 */

    /* LOCKSTAT 指令配置 */
    uint8_t lockstat_report;    /* 锁状态上报方式: 0-GPRS, 1-GPRS+SMS, 2-GPRS+SMS+CALL */
    uint8_t lockstat_trigger;   /* 锁状态触发方式: 0-都不触发, 1-上锁触发, 2-解锁触发, 3-上锁解锁均触发 */

    /* MOTDET 指令配置 */
    uint16_t motdet_static_g;           /* 静止G值方差阈值 (1-500 mg) */
    uint16_t motdet_land_g;             /* 陆运G值方差阈值 (1-3000 mg) */
    uint16_t motdet_static_land_length; /* 静止进入陆运投票时长 (30-600 s) */
    uint16_t motdet_sea_transport_time; /* 进入海运投票时长 (10-600 s) */
    uint8_t  motdet_report_type;        /* 模式切换上报方式: 0-GPRS, 1-GPRS+SMS, 2-GPRS+SMS+CALL */

    /* BATLEVEL 指令配置 */
    uint8_t batlevel_empty_trg;         /* Empty状态触发方式: 0-不触发, 1-在线触发, 2-状态变化触发 */
    uint8_t batlevel_empty_rpt;         /* Empty状态上报方式: 0-GPRS, 1-GPRS+SMS, 2-GPRS+SMS+CALL */
    uint8_t batlevel_low_trg;           /* Low状态触发方式 */
    uint8_t batlevel_low_rpt;           /* Low状态上报方式 */
    uint8_t batlevel_normal_trg;        /* Normal状态触发方式 */
    uint8_t batlevel_normal_rpt;        /* Normal状态上报方式 */
    uint8_t batlevel_fair_trg;          /* Fair状态触发方式 */
    uint8_t batlevel_fair_rpt;          /* Fair状态上报方式 */
    uint8_t batlevel_high_trg;          /* High状态触发方式 */
    uint8_t batlevel_high_rpt;          /* High状态上报方式 */
    uint8_t batlevel_full_trg;          /* Full状态触发方式 */
    uint8_t batlevel_full_rpt;          /* Full状态上报方式 */

    /* CHARGESTA 指令配置 */
    uint8_t chargesta_report;           /* 充电状态上报方式: 0-GPRS, 1-GPRS+SMS, 2-GPRS+SMS+CALL */

    /* SHOCKALARM 指令配置 */
    uint8_t shockalarm_sw;              /* 撞击报警开关: 0-OFF, 1-ON */
    uint8_t shockalarm_level;           /* 撞击力度阈值: 1-5 (1最不敏感,5最敏感) */
    uint8_t shockalarm_type;            /* 告警上报方式: 0-GPRS, 1-GPRS+SMS, 2-GPRS+SMS+CALL */

    /* STARTR 指令配置 */
    uint8_t startr_sw;                  /* 数据记录功能开关: 0-OFF, 1-ON */

    /* PWRSAVE 指令配置 */
    uint8_t pwsave_sw;                 /* 低功耗运输状态开关: 0-OFF, 1-ON */

    /* BT_CRFPWR 指令配置 */
    int8_t  bt_crfpwr;                  /* 蓝牙发射功率: -8,-4,0,3,5,7,12 dBm */

    /* BT_UPDATA 指令配置 */
    uint8_t  bt_updata_mode;            /* 工作方式: 0-不开启, 1-Cell启动时开启, 2-持续收集Cell启动上传, 3-持续收集Cell启动上传或唤醒上传 */
    uint32_t bt_updata_scan_interval;   /* 蓝牙数据收集间隔: 1-86400秒 */
    uint32_t bt_updata_scan_length;     /* 每次收集搜索时长: 1-86400秒 */
    uint32_t bt_updata_updata_interval; /* 蓝牙唤醒间隔: 1-86400秒 */

    /* TAG 指令配置 */
    uint8_t tag_sw;                     /* Tag定位功能开关: 0-OFF, 1-ON */
    uint16_t tag_interval;              /* 广播间隔: 100-60000ms */

    /* LOCKCD 指令配置 */
    uint16_t lockcd_countdown;          /* 插入后上锁倒计时: 0-3600秒, 0代表不自动上锁 */

    /* LED 指令配置 */
    uint8_t led_display;                /* LED显示开关: 0-OFF, 1-ON */

    /* BUZZER 直接控制指令配置 */
    uint8_t buzzer_operator;            /* 蜂鸣器操作: 0-停止, 1-持续报警, 2-成功提示音, 3-失败提示音, 4-异常提示音, 5-一般报警音 */

    /* NCFTRIG 指令配置 */
    char     ncftrig_nfc_no[16];        /* 联动的NFC卡号 */
    char     ncftrig_command[128];      /* 需要执行的完整可执行指令 */

    /* NFCAUTH 指令配置 */
    NfcAuthCard nfcauth_cards[10];      /* NFC卡权限数组，最多10张卡 */
    uint8_t     nfcauth_card_count;     /* 已授权卡数量 */

    /* BKEY_SET/BKEY_RESET 指令配置 */
    char        bt_key[7];              /* 蓝牙解锁密钥，6位数字 + 结束符 */

} DeviceCmdConfig;

/* 全局指令配置变量声明 */
extern DeviceCmdConfig g_device_cmd_config;

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
