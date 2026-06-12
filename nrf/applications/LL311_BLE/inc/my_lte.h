/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        my_lte.h
**文件描述:        LTE 模块通讯管理头文件 (XQ200U)
**当前版本:        V1.0
**作    者:        Harrison Wu (wuyujiao@jimiiot.com)
**完成日期:        2026.01.15
*********************************************************************
** 功能描述:        1. 处理与 XQ200U LTE 模块的 UART 通讯
**                 2. 提供 LTE 模块电源控制接口
**                 3. 支持异步 UART 数据收发
*********************************************************************/

#ifndef _MY_LTE_H_
#define _MY_LTE_H_

/* LTE 模块 UART 缓冲区大小 */
#define LTE_UART_BUF_SIZE 256

/* 消息队列项数据结构 */
typedef struct {
    char *msg_content;          /* 消息内容字符串 */
    uint16_t msg_len;           /* 消息长度 */
} lte_pending_msg_t;

// 应答命令类型枚举
typedef enum
{
    BLE_RSP_UNKNOWN = 0,
    BLE_RSP_LOCATION, // BLE+LOCATION=OK,lat,lon
    BLE_RSP_LED,      // BLE+LED=OK
    BLE_RSP_TIME,     // BLE+TIME=OK,<UTC秒数>
    BLE_RSP_TAG,      // BLE+TAG=OK,START/END/seq
    BLE_RSP_OTA,      // BLE+OTA=OK
    BLE_RSP_INFO,     // BLE+INFO=OK
    BLE_RSP_MACINFO,  // BLE+MACINFO=OK,START/END/seq
    BLE_RSP_WMODE,    // BLE+WMODE=OK
    BLE_RSP_POWOFF,   // BLE+POWOFF=OK
    BLE_RSP_PULSE,    // BLE+PULSE=OK,<持续时间(分钟)>
    BLE_RSP_MAX
} ble_rsp_t;

// 应答解析结果结构体
typedef struct
{
    ble_rsp_t type; // 应答类型
    char cmd_name[32]; // 命令名称
    char params[256];  // 参数部分
    int param_count;   // 参数个数
} ble_rsp_result_t;

// 命令映射表
typedef struct
{
    const char *cmd_name;  // 命令名称 (如 "LOCATION")
    ble_rsp_t rsp_type; // 应答类型
} ble_rsp_cmd_map_t;

// 经纬度存储点结构体定义
typedef struct
{
    int32_t lat;         // 纬度（微度，单位：1e-6度）
    int32_t lon;         // 经度（微度，单位：1e-6度）
    float speed;       // GPS速度（单位：m/s）
    int64_t timestamp_s; // 获取时间戳（秒）
} location_storage_t;

typedef struct
{
    char start[16];             //记录开始时间用于查询
    uint16_t delay;             //记录窗口时间（用于查询）
    struct k_timer start_timer; // 网络开锁开始倒计时定时器
    struct k_timer delay_timer; // 网络开锁成功之后窗口期倒计时定时器
    uint16_t delay_sec;         // 窗口期时长（秒）（delay_timer定时器时长）
    uint8_t netunlock_flag;     // 在窗口期时长内开锁标志
    uint8_t start_timer_flag;   // 开始定时器开启过标志位（开启了start定时器意味着不需要回复）
} net_unlock_ctrl_t;

//网络解锁全局变量
extern net_unlock_ctrl_t g_net_unlock;

//全局经纬度存储点
extern location_storage_t g_location_point;

// 4G模块是否就绪标志位
extern bool g_bLteReady;

extern char g_lte4GVersion[32];

/* 循环缓冲区用于存储排队的LTE消息 */
#define LTE_MSG_QUEUE_SIZE    10  /* 可排队的最大消息数 */

//重传检查使能
#define RETRANSMIT_CHECK_ENABLED 0

typedef struct {
    lte_pending_msg_t queue[LTE_MSG_QUEUE_SIZE];  /* 消息的循环缓冲区 */
    uint8_t head;                                 /* 最旧消息的索引 */
    uint8_t tail;                                 /* 下一个空槽的索引 */
    uint8_t count;                                /* 当前队列中的消息数 */
} lte_msg_queue_t;

extern bool g_lte_ota_in_progress;

extern uint8_t g_lte_net_flag; // 网络状态
extern uint8_t g_lte_net_signal_level; // 网络信号强度
extern uint8_t g_lte_gps_state; // GPS状态
extern char g_lte_gps_signal[50]; // GPS信号值

/********************************************************************
**函数名称:  my_lte_init
**入口参数:  tid      ---        指向线程 ID 变量的指针
**出口参数:  tid      ---        存储启动后的线程 ID
**函数功能:  初始化 LTE 模块相关的 UART 设备与电源控制 GPIO，并启动 LTE 线程
**返 回 值:  0 表示成功，其他表示失败
*********************************************************************/
int my_lte_init(k_tid_t *tid);

/********************************************************************
**函数名称:  my_lte_uart_send
**入口参数:  data     ---        待发送数据指针
**            len      ---        数据长度
**出口参数:  无
**函数功能:  通过 UART 向 LTE 模块发送数据
**返 回 值:  0 表示成功，其他表示失败
*********************************************************************/
int my_lte_uart_send(const uint8_t *data, uint16_t len);

/********************************************************************
**函数名称:  get_lte_power_state
**入口参数:  无
**出口参数:  无
**函数功能:  获取LTE模块电源状态
**返 回 值:  true:电源打开，false:电源关闭
*********************************************************************/
bool get_lte_power_state(void);

/********************************************************************
**函数名称:  my_lte_pwr_on
**入口参数:  on       ---        true 为开启电源，false 为关闭
**出口参数:  无
**函数功能:  控制 LTE 模块的电源使能引脚 (P2.02)
**返 回 值:  0 表示成功，其他表示失败
*********************************************************************/
int my_lte_pwr_on(bool on);

/*
 * 处理LTE串口接收到的数据，负责组包拆包成独立指令
 */
void my_lte_handle_recv(uint8_t *pData, uint32_t iLen);

/********************************************************************
**函数名称:  my_at_test
**入口参数:  argc     ---        参数个数
**           argv     ---        参数数组
**出口参数:  无
**函数功能:  处理AT测试命令
**返 回 值:  0表示成功，-1表示参数错误
*********************************************************************/
int my_at_test(int argc, char *argv[]);

int my_lte_parse_cmd(char *cmd, int cmd_len);

/********************************************************************
**函数名称:  set_lte_boot_reason
**入口参数:  reason   ---        要设置的开机原因枚举值
**出口参数:  无
**函数功能:  设置 LTE 开机原因，用于 4G 开机完成握手时回复
**返 回 值:  无
*********************************************************************/
void set_lte_boot_reason(lte_boot_reason_t reason);

/********************************************************************
**函数名称:  get_lte_boot_reason
**入口参数:  无
**出口参数:  无
**函数功能:  获取当前记录的 LTE 开机原因
**返 回 值:  当前开机原因枚举值
*********************************************************************/
lte_boot_reason_t get_lte_boot_reason(void);

/********************************************************************
**函数名称:  my_verify_nfc_permission
**入口参数:  无
**出口参数:  无
**函数功能:  执行刷卡位置校验，主要包括：
**            1. 校验当前储存点位置数据的有效性，有效就根据 NFC 卡片的权限状态执行开锁或关锁规则
**            2.储存点无效发送BLE+LOCATION=seq;seq为刷卡索引，获取经纬度信息
**返 回 值:  无
********************************************************************/
void my_verify_nfc_permission(void);

/********************************************************************
**函数名称:  my_lte_handle_cmd
**入口参数:  data      ---   接收4G的数据
**函数功能:  执行LTE+CMD=<号码>,<指令内容>,并根据指令内容的执行回复对应的结果给4G模块
**            <指令内容>与通过蓝牙指令下发下来的内容一致，按照相关指令格式填写即可
**          如:LTE+CMD=111,VERSION/LTE+CMD=111,VERSION#/末尾加不加#都可以
**          LTE+CMD=111,NFCTRIG,ADD,1234456789,
**          "NFCAUTH,SET,88040FBE99050B,+22277120,13516763,999900,2603200000,2603201200,1"
*********************************************************************/
int my_lte_handle_cmd(char *data);

/********************************************************************
**函数名称:  lte_send_cmd_with_retry
**入口参数:  cmd_name  ---   指令名称
**           param     ---   指令参数 (可为 NULL)
**出口参数:  无
**函数功能:  串口发送指令并加入重传队列
**           1. 尝试发送指令，若失败直接返回
**           2. 发送完发送加入队列消息到LTE
**返 回 值:  无
********************************************************************/
void lte_send_cmd_with_retry(const char *cmd_name, const char *param);

/********************************************************************
**函数名称:  async_match_and_resp
**入口参数:  data      ---        数据（格式：指令头,回复内容）如CUNLOCK,Unlock failed. No unlock state detected.
**出口参数:  无
**函数功能:  解析数据头，在队列中查找匹配项；若匹配成功，将元素前移并发送响应给LTE
**返 回 值:  0表示匹配成功并发送，-1表示未匹配到对应指令
********************************************************************/
int async_match_and_resp(char *data);

/********************************************************************
**函数名称:  send_led_command
**入口参数:  无
**出口参数:  无
**函数功能:  发送LED显示指令
**返 回 值:  无
*********************************************************************/
void send_led_command(void);

#endif /* _MY_LTE_H_ */
