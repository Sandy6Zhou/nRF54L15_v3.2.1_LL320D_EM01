/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        my_cmd_setting.h
**文件描述:        设备命令设置模块头文件
**当前版本:        V1.0
**作    者:        周森达 (zhousenda@jimiiot.com)
**完成日期:        2026.02.12
*********************************************************************/

#ifndef _MY_CMD_SETTING_H_
#define _MY_CMD_SETTING_H_

#define AT_CMD_TABLE_TOTAL (sizeof(at_cmd_attr_table)/sizeof(at_cmd_attr_t))
#define LTE_CMD_TABLE_TOTAL (sizeof(lte_cmd_attr_table)/sizeof(char*))

#define RESP_STRING_LENGTH_MAX              1024                //应答缓冲区大小
#define CMD_STRING_LENGTH_MAX               128                 //暂定可接收cmd缓冲区大小

typedef enum
{
    PARM_1,
    PARM_2,
    PARM_3,
    PARM_4,
    PARM_5,
    PARM_6,
    PARM_7,
    PARM_8,
    PARM_9,
    PARM_10,
    PARM_11,
    PARM_12,
    PARM_13,
    PARM_14,
    PARM_15,
    PARM_16,
    PARM_17,
    PARM_18,
    PARM_19,
    PARM_20,
    PARM_MAX
} cmd_parm_t;

typedef struct {
    char *parm[PARM_MAX];
    uint8_t parm_count;

    char rcv_msg[CMD_STRING_LENGTH_MAX];            //接收到的数据
    uint16_t rcv_length;

    char *resp_msg;                                 //应答数据指针（指向静态缓冲区）
    uint16_t resp_length;
} at_cmd_t;

typedef int(*at_cmd_handler_t)(at_cmd_t *msg);

typedef struct
{
    char *cmd_str;
    at_cmd_handler_t cmd_func;
} at_cmd_attr_t;

// 标记lte_cmd来的,用于区分蓝牙下发的还是lte过来的(某些指令只能网络发蓝牙不能执行)
extern uint8_t g_lte_cmdSource;

// 用于存储整包返回的数据内容(仅在蓝牙线程使用)
extern char g_resp_buf[RESP_STRING_LENGTH_MAX];

/********************************************************************
**函数名称:  set_long_battery_params
**入口参数:  config              ---    指向设备工作模式配置结构体的指针
**           reporting_interval   ---    上报间隔时间（单位：分钟，5~1440）
**           start_time_str      ---    启动时间字符串（格式：HHMM）
**           gnss_sw             ---    GNSS开关（1=ON, 0=OFF）
**出口参数:  config              ---    更新后的长续航模式配置结构体
**函数功能:  设置长续航模式的工作参数，包括上报间隔、启动时间和GNSS开关
**返 回 值:  0 表示成功，负值表示失败（如时间格式错误等）
*********************************************************************/
int set_long_battery_params(device_work_mode_config_t *config, uint16_t reporting_interval, const char *start_time_str, uint8_t gnss_sw);

/********************************************************************
**函数名称:  set_intelligent_params
**入口参数:  config     ---    指向设备工作模式配置结构体的指针
**           sub_mode   ---    子模式（0~5），控制Cell/GNSS休眠策略
**           static_int ---    静止状态上报间隔（原始值，单位由子模式决定）
**           moving_int ---    运动状态上报间隔（原始值，单位由子模式决定）
**出口参数:  config     ---    更新后的智能模式配置结构体
**函数功能:  设置智能模式的工作参数，根据子模式配置静止/运动上报间隔
**返 回 值:  0 表示成功，负值表示失败（如参数非法等）
*********************************************************************/
int set_intelligent_params(device_work_mode_config_t *config, uint8_t sub_mode, uint32_t static_int, uint32_t moving_int);

/********************************************************************
**函数名称:  at_recv_cmd_handler
**入口参数:  at_cmd_msg      ---        AT指令结构体指针，包含接收的指令和响应存储区域(输入/输出)
**出口参数:  at_cmd_msg中更新响应消息内容和响应长度
**函数功能:  解析接收到的AT指令并执行对应的处理函数
**返回值:    成功返回处理函数返回的BLE数据类型，未匹配指令或处理失败返回0
*********************************************************************/
uint16_t at_recv_cmd_handler(at_cmd_t *at_cmd_msg);

/********************************************************************
**函数名称:  run_lte_cmd
**入口参数:  at_cmd_msg      ---   指令结构体指针，包含接收的指令和响应存储区域(输入/输出)
**出口参数:  at_cmd_msg中更新响应消息内容和响应长度
**函数功能:  执行LTE+CMD指令中的command
**返回值:    未匹配指令或命令解析失败返回0
**          返回非0不代表command执行成功，具体看对应的执行函数resp_msg回复
*********************************************************************/
uint16_t run_lte_cmd(at_cmd_t *at_cmd_msg);

/*********************************************************************
**函数名称:  lte_send_command
**入口参数:  cmd_name     --  命令名称
**           param        --  命令参数（可选，NULL 表示无参数）
**出口参数:  无
**函数功能:  用于构建并发送 LTE 命令到 LTE 模块，支持带参数和不带参数的命令。
**           命令格式：BLE+命令名称[=参数]
**返 回 值:  0 表示成功，-1 表示失败（模式非法）
*********************************************************************/
int lte_send_command(const char *cmd_name, const char *param);

#endif /* _MY_CMD_SETTING_H_ */