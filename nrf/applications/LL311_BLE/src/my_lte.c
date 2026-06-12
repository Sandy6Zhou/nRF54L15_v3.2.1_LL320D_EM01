/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        my_lte.c
**文件描述:        LTE 模块通讯管理实现文件 (XQ200U)
**当前版本:        V1.0
**作    者:        Harrison Wu (wuyujiao@jimiiot.com)
**完成日期:        2026.01.15
*********************************************************************
** 功能描述:        1. 实现与 XQ200U LTE 模块的 UART 异步通讯
**                 2. 实现电源控制逻辑 (P2.02)
**                 3. 包含串口回环测试逻辑
*********************************************************************/

/* 必须在包含 my_comm.h 之前定义 BLE_LOG_MODULE_ID，避免与 my_ble_log.h 中的默认定义冲突 */
#define BLE_LOG_MODULE_ID BLE_LOG_MOD_LTE

#include "my_comm.h"

#define UART_TX_BUFFER_SIZE    1024

// 默认存储点有效期: 30分钟（秒)
#define LOCATION_VALIDITY_PERIOD_S     (30 * 60)

// LTE UART 单次发送最大等待超时
#define LTE_UART_TX_WAIT_MS         200
// LTE UART 空闲挂起超时：3秒无收发自动进入低功耗挂起态
#define LTE_UART_IDLE_TIMEOUT_MS    3000

// 串口协议报文头定义清单
char LTE_PWRON[] = "LTE+PWRON=";
char LTE_BTSET[] = "LTE+BTSET=";
char LTE_NTCSET[] = "LTE+NTCSET=";
char LTE_TIME[] = "LTE+TIME=";
char LTE_TRANSMIT[] = "LTE+TRANSMIT=";
char LTE_FOTA[] = "LTE+FOTA=";
char LTE_STATE[] = "LTE+STATE=";
char LTE_CMD[] = "LTE+CMD=";
char LTE_LOCATION[] = "LTE+LOCATION=";
char LTE_FACTORY[] = "LTE+FACTORY=";
char LTE_NET[] = "LTE+NET=";
char BLE_CMD[] = "BLE+CMD=";
char BLE[] = "BLE+";

// 经纬度存储点
location_storage_t g_location_point = {0};

/* 串口重发机制：
 * 设备将消息通过串口发出去后，同时记录此消息到重发队列里面并标记状态为等待状态
 * 收到对应消息应答时，将重发队列里面对应的消息的标记位变为非等待状态(即下次可使用这个位置进行存储消息)
 * 当重发队列里面的消息超过ACK_TIMEOUT_S s未收到应答时，执行重发操作并标记重发次数，达到重发上限次数(3次)仍未收到应答时，将消息从重发队列里面清掉。
 */

// 重传机制相关宏和结构体
#define MAX_RETRIES                3   // 最大重试次数
#define ACK_TIMEOUT_S              2   // ACK等待超时时间(秒)
#define RETRANSMISSION_QUEUE_SIZE  10  // 重传队列大小
#define RETRANSMIT_TIMER_PERIOD_MS 500 // 重传定时器周期（毫秒）

typedef enum
{
    MSG_STATE_IDLE = 0, // 空闲可用
    MSG_STATE_PENDING,   // 等待ACK
    MSG_STATE_TIMEOUT    // 超时
} msg_state_enum_t;

typedef struct
{
    char cmd_name[32];  // 指令头
    char *param;        // 发送的参数内容
    int retry_count;    // 当前重试次数
    time_t send_time;   // 最后一次发送的时间(秒时间戳)
    msg_state_enum_t state; // 当前状态
} retransmission_item_t;

static retransmission_item_t s_retrans_queue[RETRANSMISSION_QUEUE_SIZE]; // 重传队列

// ========== 需要特殊处理的指令前缀列表 ==========
static const char *s_special_cmd_prefixes[] = {
    "MACINFO", // BLE+MACINFO=<seq>,... → 存储为 MACINFO_<seq> 或 MACINFO_START/END
    "TAG",     // BLE+TAG=<seq>,... → 存储为 TAG_<seq> 或 TAG_START/END
    NULL
};

// 重传检查定时器
static struct k_timer s_retrans_check_timer;

// LTE UART 空闲挂起定时器（3秒无收发自动挂起UART）
typedef struct
{
    struct k_timer idle_timer;  // 空闲定时器：超时后触发 UART 挂起以节省功耗
    bool active;                // UART 是否处于活跃态（true=已启用RX，false=已挂起）
    bool tx_busy;               // UART 是否正在发送数据（true=发送中，false=空闲）
} lte_uart_ctx_t;

static lte_uart_ctx_t s_lte_uart_ctx = { 0 };

// 电源管理回调函数前置声明
static int lte_pm_init(void);
static int lte_pm_suspend(void);
static int lte_pm_resume(void);

// LTE 电源管理操作回调结构体
static const pm_device_ops_t lte_pm_ops =
{
    .init = lte_pm_init,
    .suspend = lte_pm_suspend,
    .resume = lte_pm_resume,
};

// 命令映射表定义
static const ble_rsp_cmd_map_t ble_rsp_cmd_table[] = {
    {"LOCATION", BLE_RSP_LOCATION},
    {"LED",      BLE_RSP_LED     },
    {"TIME",     BLE_RSP_TIME    },
    {"TAG",      BLE_RSP_TAG     },
    {"MACINFO",  BLE_RSP_MACINFO },
    {"WMODE",    BLE_RSP_WMODE },
    {"POWOFF",   BLE_RSP_POWOFF },
    {"PULSE",    BLE_RSP_PULSE },
    {NULL,       BLE_RSP_UNKNOWN }
};

// 4G透传指令异步回复队列
typedef struct
{
    char cmd_name[16]; // 指令头
    char id[16];       // 发送方号码
    uint8_t used;      // 是否占用（0=空闲，1=使用中）
} async_resp_tiem_t;

// LTE+CMD4G透传指令需要异步回复的队列大小（目前就LTE+CMD=UNLOCK,.../LTE+CMD=CLCOK网络开锁和解锁）
#define ASYNC_QUEUE_SIZE 6

async_resp_tiem_t g_async_queue[ASYNC_QUEUE_SIZE];

/* LTE电源状态跟踪 */
static bool s_lte_power_state = false;  // false=关闭, true=开启

// 4G模块是否完成开机，开机后可以进行正常数据收发
// 0: 未开机； 1: 已开机(并发送了开机消息LTE+PWRON)
bool g_bLteReady = 0;

/* 注册 LTE 模块日志 */
LOG_MODULE_REGISTER(my_lte, LOG_LEVEL_INF);

/* 从设备树获取 UART 与 GPIO 配置 */
#define LTE_UART_NODE DT_ALIAS(lte_uart)
static const struct device *lte_uart_dev = DEVICE_DT_GET(LTE_UART_NODE);

#define LTE_PWR_CTRL_NODE DT_ALIAS(lte_pwr_ctrl)
static const struct gpio_dt_spec lte_pwr_gpio = GPIO_DT_SPEC_GET(LTE_PWR_CTRL_NODE, gpios);

#define LTE_WAKE_NODE DT_ALIAS(lte_wake_ctrl)
static const struct gpio_dt_spec lte_wake_gpio = GPIO_DT_SPEC_GET(LTE_WAKE_NODE, gpios);
static struct gpio_callback lte_wake_cb;

/* 接收LTE+CMD响应回复缓冲区大小 */
#define LTE_CMD_RESPBUF_SIZE 1024
static char s_lte_cmd_resp_buf[LTE_CMD_RESPBUF_SIZE] = {0};

// 定义一个串口发送状态信号量，初始值为1(表示UART空闲)
static struct k_sem s_TxDoneSem;
/* LTE缓存消息队列 */
static lte_msg_queue_t s_lte_msg_queue = {0};

// 4G上电状态，0：蓝牙正常唤醒4G，1：异常重启
static lte_power_state_t s_4GPoweronStatus = 0;
// 4G版本号缓存
char g_lte4GVersion[32] = {0};
// LTE开机原因
static lte_boot_reason_t s_lteBootReason = LTE_BOOT_REASON_RESERVED;

/* UART驱动层使用的接收双缓冲 */
static uint8_t s_lte_rx_buf_1[LTE_UART_BUF_SIZE];
static uint8_t s_lte_rx_buf_2[LTE_UART_BUF_SIZE];
static uint8_t *lte_next_buf = s_lte_rx_buf_2;

// 串口接收循环缓冲区（建议用2的幂，如1024，取模效率更高）
#define LTE_UART_RB_SIZE    512
static uint8_t s_lte_rb_buf[LTE_UART_RB_SIZE];
static ring_buffer_t s_lte_rb;

/* 消息队列定义 */
K_MSGQ_DEFINE(my_lte_msgq, sizeof(msg_t), 10, 4);

/* 线程数据与栈定义 */
K_THREAD_STACK_DEFINE(my_lte_task_stack, MY_LTE_TASK_STACK_SIZE);
static struct k_thread s_my_lte_task_data;

// 产测指令
const char FACTORY_CMD_HEADER[] = "AT^GT_CM=";

// 4G进入产测模式（0 = exit, 1:enter）
static uint8_t s_lte_factory = 0;

/* { visible, command, help, function } */
cmd_struct_t AT_CMD_INNER[] = {

    {1, "TEST",      "AT CMD TEST",             my_at_test},

    {0, NULL,        NULL,                      NULL}
};
// 发送脉冲消息声明
static void send_lte_pulse(void);

// 网络解锁全局变量
net_unlock_ctrl_t g_net_unlock = {0};

// 脉冲消息计数器
static uint16_t s_lte_pulse_count = 0;

// 网络状态
uint8_t g_lte_net_flag = 0;

/********************************************************************
**函数名称:  init_async_queue
**入口参数:  无
**出口参数:  无
**函数功能:  初始化/清空异步回复队列，将所有队列元素内存置零
**返 回 值:  无
********************************************************************/
void init_async_queue(void)
{
    memset(g_async_queue, 0, sizeof(g_async_queue));
}

// lte OTA升级状态
bool g_lte_ota_in_progress = false;

/********************************************************************
**函数名称:  lte_uart_idle_timer_handler
**入口参数:  timer   ---   定时器句柄（输入）
**出口参数:  无
**函数功能:  LTE UART空闲定时器回调，触发UART挂起检查
**返 回 值:  无
**注意事项:  通过消息机制通知LTE线程执行挂起判断，避免在中断上下文操作UART
*********************************************************************/
static void lte_uart_idle_timer_handler(struct k_timer *timer)
{
    ARG_UNUSED(timer);

    my_send_msg(MOD_LTE, MOD_LTE, MY_MSG_LTE_UART_IDLE);
}

/********************************************************************
**函数名称:  lte_pm_init
**入口参数:  无
**出口参数:  无
**函数功能:  LTE电源管理初始化，重置UART上下文状态
**返 回 值:  0 --- 始终成功
*********************************************************************/
static int lte_pm_init(void)
{
    s_lte_uart_ctx.active = false;
    s_lte_uart_ctx.tx_busy = false;
    k_timer_init(&s_lte_uart_ctx.idle_timer, lte_uart_idle_timer_handler, NULL);
    return 0;
}

/********************************************************************
**函数名称:  lte_pm_suspend
**入口参数:  无
**出口参数:  无
**函数功能:  LTE UART挂起处理，关闭RX接收以节省功耗
**返 回 值:  0 --- 挂起成功
**           其他 --- uart_rx_disable返回的错误码
**注意事项:  允许忽略-EFAULT/-EINVAL/-EBUSY等已关闭或忙异常
*********************************************************************/
static int lte_pm_suspend(void)
{
    int ret;

    // 先设置标志，再禁用 RX，防止中断抢占导致重新 enable rx
    s_lte_uart_ctx.active = false;

    ret = uart_rx_disable(lte_uart_dev);
    if ((ret != 0) && (ret != -EFAULT) && (ret != -EINVAL) && (ret != -EBUSY))
    {
        MY_LOG_ERR("LTE UART RX disable failed: %d", ret);
        // 如果失败，恢复 active 标志
        s_lte_uart_ctx.active = true;
        return ret;
    }

    MY_LOG_INF("LTE UART suspended");
    return 0;
}

/********************************************************************
**函数名称:  lte_pm_resume
**入口参数:  无
**出口参数:  无
**函数功能:  LTE UART恢复处理，重新启用RX接收
**返 回 值:  0 --- 恢复成功
**           其他 --- uart_rx_enable返回的错误码
**注意事项:  -EBUSY表示UART已处于活跃态，允许忽略
*********************************************************************/
static int lte_pm_resume(void)
{
    int ret;

    ret = uart_rx_enable(lte_uart_dev, s_lte_rx_buf_1, LTE_UART_BUF_SIZE, 10 * USEC_PER_MSEC);
    if ((ret != 0) && (ret != -EBUSY))
    {
        MY_LOG_ERR("Failed to enable LTE UART RX in resume (err %d)", ret);
        return ret;
    }

    s_lte_uart_ctx.active = true;

    MY_LOG_INF("LTE UART resumed");
    return 0;
}

/********************************************************************
**函数名称:  lte_uart_ensure_active
**入口参数:  无
**出口参数:  无
**函数功能:  确保LTE UART处于活跃态，若已挂起则执行恢复
**返 回 值:  0 --- UART已活跃或恢复成功
**           -ENODEV --- LTE电源未开启
**           其他 --- my_pm_device_resume返回的错误码
*********************************************************************/
static int lte_uart_ensure_active(void)
{
    int ret;

    if (!s_lte_power_state)
    {
        return -ENODEV;
    }

    if (!s_lte_uart_ctx.active)
    {
        ret = my_pm_device_resume(MY_PM_DEV_LTE);
        if (ret < 0)
        {
            MY_LOG_ERR("Failed to resume LTE UART: %d", ret);
            return ret;
        }
    }

    return 0;
}

/********************************************************************
**函数名称:  lte_uart_can_suspend
**入口参数:  无
**出口参数:  无
**函数功能:  检查LTE UART是否满足挂起条件
**返 回 值:  true --- 可以挂起
**           false --- 存在未处理数据或发送中，不允许挂起
**注意事项:  需同时检查消息队列、循环缓冲区、重传队列及发送状态
*********************************************************************/
static bool lte_uart_can_suspend(void)
{
    bool can_suspend;

    can_suspend = (s_lte_msg_queue.count == 0);

    if (my_rb_get_used_size(&s_lte_rb) > 0)
    {
        can_suspend = false;
    }

#if RETRANSMIT_CHECK_ENABLED
    if (!retrans_queue_is_empty())
    {
        can_suspend = false;
    }
#endif

    if (s_lte_uart_ctx.tx_busy)
    {
        can_suspend = false;
    }

    return can_suspend;
}

/********************************************************************
**函数名称:  lte_uart_activity_kick
**入口参数:  无
**出口参数:  无
**函数功能:  刷新LTE UART活跃定时器，延迟挂起以维持通信
**返 回 值:  无
*********************************************************************/
static void lte_uart_activity_kick(void)
{
    if (!s_lte_uart_ctx.active)
    {
        return;
    }

    k_timer_start(&s_lte_uart_ctx.idle_timer, K_MSEC(LTE_UART_IDLE_TIMEOUT_MS), K_NO_WAIT);
}

/********************************************************************
**函数名称:  lte_wake_pin_isr
**入口参数:  port     ---        GPIO端口
**           cb       ---        GPIO回调结构体
**           pins     ---        触发中断的引脚位掩码
**出口参数:  无
**函数功能:  LTE唤醒引脚(P0.04)中断回调，4G模块发数据前拉低此引脚
**           唤醒Nordic，本回调通知LTE线程恢复UART接收
**返 回 值:  无
*********************************************************************/
static void lte_wake_pin_isr(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
{
    ARG_UNUSED(port);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    // 避免频繁发消息到LTE线程
    if (!s_lte_uart_ctx.active)
    {
        my_send_msg(MOD_LTE, MOD_LTE, MY_MSG_LTE_WAKEUP);
    }
}

/********************************************************************
**函数名称:  init_retransmission_queue
**入口参数:  无
**出口参数:  无
**函数功能:  重传队列初始化
**           1. 遍历队列所有槽位，释放动态分配的 param 内存
**           2. 清空 cmd_name、计数器和时间戳
**           3. 重置状态为 0 (空闲)
**返 回 值:  无
********************************************************************/
void init_retransmission_queue(void)
{
    int i = 0;
    // 将整个队列的内存块清零
    for (i = 0; i < RETRANSMISSION_QUEUE_SIZE; i++)
    {
        if (s_retrans_queue[i].param)
        {
            MY_FREE_BUFFER(s_retrans_queue[i].param);
            s_retrans_queue[i].param = NULL;
        }

        memset(s_retrans_queue[i].cmd_name, 0, sizeof(s_retrans_queue[i].cmd_name));
        s_retrans_queue[i].retry_count = 0;
        s_retrans_queue[i].send_time = 0;
        s_retrans_queue[i].state = 0;
    }

    // 停止定时器
    if (k_timer_remaining_get(&s_retrans_check_timer) != 0)
    {
        k_timer_stop(&s_retrans_check_timer);
        MY_LOG_INF("s_retrans_check_timer : STOP");
    }

    MY_LOG_INF("Retransmission queue initialized with size %d", RETRANSMISSION_QUEUE_SIZE);
}

/********************************************************************
**函数名称:  retrans_queue_is_empty
**入口参数:  无
**出口参数:  无
**函数功能:  判断重传队列是否为空
**           遍历队列，检查是否存在状态为 MSG_STATE_PENDING 的消息
**返 回 值:  1: 队列为空
**           0: 队列非空
********************************************************************/
int retrans_queue_is_empty(void)
{
    int i = 0;
    for (i = 0; i < RETRANSMISSION_QUEUE_SIZE; i++)
    {
        if (s_retrans_queue[i].state == MSG_STATE_PENDING)
        {
            return 0; // 非空
        }
    }
    return 1; // 空
}

/********************************************************************
**函数名称:  check_ack
**入口参数:  cmd_name  ---   收到的应答消息对应的命令名称
**出口参数:  无
**函数功能:  检查并处理应答
**           1. 遍历队列，查找处于 PENDING 状态且名称匹配的消息
**           2. 若匹配成功：标记为 ACKED，释放 param 内存，打印日志
**           3. 检查队列是否已空，若空则发送消息停止重传定时器
**返 回 值:  无
********************************************************************/
void check_ack(char *cmd_name)
{
    int i = 0;
    int j = 0;
    for (i = 0; i < RETRANSMISSION_QUEUE_SIZE; i++)
    {
        // 当前收到的应答消息是重传队列中的消息且处于等待应答阶段
        if (s_retrans_queue[i].state == MSG_STATE_PENDING &&
            (strcmp(s_retrans_queue[i].cmd_name, cmd_name) == 0))
        {
            MY_LOG_INF("Received ACK for pending message[%d]:%s", i, s_retrans_queue[i].cmd_name);
            // 释放当前param
            if (s_retrans_queue[i].param)
            {
                MY_FREE_BUFFER(s_retrans_queue[i].param);
                s_retrans_queue[i].param = NULL;
            }
            // 前移
            for (j = i; j < RETRANSMISSION_QUEUE_SIZE - 1 && s_retrans_queue[j + 1].state == MSG_STATE_PENDING; j++)
            {
                memcpy(&s_retrans_queue[j], &s_retrans_queue[j + 1], sizeof(retransmission_item_t));
                s_retrans_queue[j + 1].param = NULL;
            }

            //清空最后一个
            memset(&s_retrans_queue[j], 0, sizeof(retransmission_item_t));
            break;
        }
    }

    // 检查队列是否为空
    if (retrans_queue_is_empty())
    {
        // 停止定时器
        if (k_timer_remaining_get(&s_retrans_check_timer) != 0)
        {
            k_timer_stop(&s_retrans_check_timer);
            MY_LOG_INF("s_retrans_check_timer : STOP");
        }
    }
}

/********************************************************************
**函数名称:  retransmission_check
**入口参数:  无
**出口参数:  无
**函数功能:  重传超时检查与处理 (定时器回调逻辑)
**           1. 获取当前系统时间
**           2. 遍历队列中 PENDING 状态的消息
**           3. 若超时 (当前时间 - 发送时间 >= 超时阈值):
**              - 未达最大重试次数: 重发指令，更新发送时间和重试计数
**              - 已达最大重试次数: 标记超时失败，释放内存，触发 LTE 断电重启
**返 回 值:  无
********************************************************************/
void retransmission_check(void)
{
    int i = 0;
    int j = 0;
    char cmd_name[32];
    int prefix_len = 0;
    time_t current_time = my_get_system_time_sec();

    for (i = 0; i < RETRANSMISSION_QUEUE_SIZE; i++)
    {
        // 只有处于等待阶段的消息才需要进行重传检查
        if (s_retrans_queue[i].state == MSG_STATE_PENDING)
        {
            // 检查是否超时，没超时则不进行重发
            if ((current_time - s_retrans_queue[i].send_time) >= ACK_TIMEOUT_S)
            {
                if (s_retrans_queue[i].retry_count < MAX_RETRIES)
                {
                    MY_LOG_INF("Retransmitting message: %s (Retry %d)", s_retrans_queue[i].cmd_name, s_retrans_queue[i].retry_count + 1);
                    strcpy(cmd_name, s_retrans_queue[i].cmd_name);

                    // 检查是否需要映射回原始指令名
                    for (j = 0; s_special_cmd_prefixes[j] != NULL; j++)
                    {
                        prefix_len = strlen(s_special_cmd_prefixes[j]);

                        // 检查是否以 "PREFIX_" 开头
                        if (strncmp(s_retrans_queue[i].cmd_name, s_special_cmd_prefixes[j],
                            prefix_len) == 0 && s_retrans_queue[i].cmd_name[prefix_len] == '_')
                        {
                            // 提取原始指令名（如 MACINFO_001 → MACINFO）
                            strcpy(cmd_name, s_special_cmd_prefixes[j]);
                            break;
                        }
                    }

                    // 执行重传
                    lte_send_command(cmd_name, s_retrans_queue[i].param);
                    s_retrans_queue[i].retry_count++;
                    s_retrans_queue[i].send_time = current_time;
                }
                else
                {
                    // 超过最大重试次数，标记为超时失败
                    MY_LOG_INF("Message failed after %d retries: %s", MAX_RETRIES, s_retrans_queue[i].cmd_name);
                    s_retrans_queue[i].state = MSG_STATE_TIMEOUT;

                    // 释放param
                    if (s_retrans_queue[i].param)
                    {
                        MY_FREE_BUFFER(s_retrans_queue[i].param);
                        s_retrans_queue[i].param = NULL;
                    }

                    // 清空存储的tag和MAC信息
                    clear_tag_macinfo();

                    // 断电
                    my_send_msg(MOD_LTE, MOD_LTE, MY_MSG_LTE_PWROFF);
                    // 重新上电
                    my_send_msg(MOD_LTE, MOD_LTE, MY_MSG_LTE_PWRON);

                    //防止多条超时造成上下电操作重复
                    break;
                }
            }
        }
    }
}

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
void lte_send_cmd_with_retry(const char *cmd_name, const char *param)
{
    int ret;
    char *command;
    msg_t msg;
    int command_len;

    // 1. 先尝试发送
    ret = lte_send_command(cmd_name, param);

    if (ret == -1)
    {
        return;
    }

    // 动态分配内存
    if (param)
    {
        command_len = strlen(cmd_name) + strlen(param) + 8;
    }
    else
    {
        command_len = strlen(cmd_name) + 1;
    }

    MY_MALLOC_BUFFER(command, command_len);

    if (command == NULL)                        // 内存分配失败
    {
        MY_LOG_ERR("command malloc failed");
        return;
    }

    if (param && strlen(param) > 0) // 有参数的情况
    {
        snprintf(command, command_len, "%s,%s", cmd_name, param);
    }
    else // 无参数的情况
    {
        snprintf(command, command_len, "%s", cmd_name);
    }

    // 发送到LTE线程处理加入重传队列
    msg.msgID = MY_MSG_ADD_RETRANS_QUEUE;
    msg.pData = command;
    my_send_msg_data(MOD_CTRL, MOD_LTE, &msg);
}

/********************************************************************
**函数名称:  add_to_retrans_queue
**入口参数:  command  ---   完整指令字符串 (格式: "CMD_NAME,PARAM")
**出口参数:  无
**函数功能:  解析指令并将其添加到重传队列
**           1. 指令解析：使用逗号分隔，提取 cmd_name 和 param
**           2. 在队列中寻找空闲槽位 (非 PENDING 状态)
**           3. 填充数据：复制 cmd_name，动态分配并复制 param
**           4. 初始化状态为 PENDING，记录发送时间
**           5. 若定时器未启动，则启动重传检查定时器
**返 回 值:  无
*********************************************************************/
void add_to_retrans_queue(char *command)
{
    int cnt = 0; // 记录当前数组已经保存几条消息
    size_t len;
    bool ret;
    char *param = NULL;
    char cmd_name[32];
    char first_param[16];
    int i = 0;

    // 获取第一个参数，指令头
    ret = my_get_str_at_pos(command, 0, ',', cmd_name, sizeof(cmd_name));
    if (ret)
    {
        param = command + strlen(cmd_name) + 1; //+1跳过逗号
    }

    // 检查是否是特殊指令
    for (i = 0; s_special_cmd_prefixes[i] != NULL; i++)
    {
        if (strcmp(cmd_name, s_special_cmd_prefixes[i]) == 0)
        {
            // 匹配到，获取第一个参数（可能是 001/START/END/MILEAGE）
            if (param && param[0] != '\0')
            {
                my_get_str_at_pos(param, 0, ',', first_param, sizeof(first_param));
                strcat(cmd_name, "_");
                strcat(cmd_name, first_param); // 变成 CMD_MILEAGE / MACINFO_START / TAG_001
            }
            break;
        }
    }

    //  将消息添加到重传队列
    for (i = 0; i < RETRANSMISSION_QUEUE_SIZE; i++)
    {
        if (s_retrans_queue[i].state != MSG_STATE_PENDING)
        {
            // 释放旧内存（防止复用时泄漏）
            //  检查是否为NULL
            if (s_retrans_queue[i].param)
            {
                MY_FREE_BUFFER(s_retrans_queue[i].param);
                s_retrans_queue[i].param = NULL;
            }

            // 动态分配 param
            if (param)
            {
                len = strlen(param) + 1;

                MY_MALLOC_BUFFER(s_retrans_queue[i].param, len);
                if (s_retrans_queue[i].param == NULL) // 内存分配失败
                {
                    MY_LOG_ERR("s_retrans_queue[%d].param failed", i);
                    return;
                }

                if (s_retrans_queue[i].param)
                {
                    memcpy(s_retrans_queue[i].param, param, len);
                }
                else
                {
                    MY_LOG_ERR("malloc param failed");
                    return;
                }
            }

            strncpy(s_retrans_queue[i].cmd_name, cmd_name, sizeof(s_retrans_queue[i].cmd_name) - 1);
            s_retrans_queue[i].cmd_name[sizeof(s_retrans_queue[i].cmd_name) - 1] = '\0';

            s_retrans_queue[i].retry_count = 0;
            s_retrans_queue[i].send_time = my_get_system_time_sec();
            s_retrans_queue[i].state = MSG_STATE_PENDING;
            break;
        }
        else
        {
            cnt++;
        }
    }

    // 数组已满
    if (RETRANSMISSION_QUEUE_SIZE == cnt)
    {
        MY_LOG_INF("retransmission queue is full");
    }

    // 如果定时器未启动，开启定时器重传检查
    if (k_timer_remaining_get(&s_retrans_check_timer) == 0)
    {
        k_timer_start(&s_retrans_check_timer, K_MSEC(RETRANSMIT_TIMER_PERIOD_MS), K_MSEC(RETRANSMIT_TIMER_PERIOD_MS));
    }
}

/********************************************************************
**函数名称:  retrans_check_timer_handler
**入口参数:  timer  ---   定时器句柄 (此处未使用)
**出口参数:  无
**函数功能:  重传检查定时器回调函数
**           1. 遍历重传队列，检查是否有消息超时
**           2. 若队列为空，则发送消息停止定时器
**返 回 值:  无
*********************************************************************/
void retrans_check_timer_handler(struct k_timer *timer)
{
    ARG_UNUSED(timer);

    // 发消息去LTE线程处理重传检查
    my_send_msg(MOD_LTE, MOD_LTE, MY_MSG_RETRANS_CHECK);
}

/********************************************************************
**函数名称:  retransmission_poll
**入口参数:  无
**出口参数:  无
**函数功能:  重传轮询主逻辑
**           1. 执行重传检查：遍历队列，处理超时重传或失败逻辑
**           2. 若队列为空，且定时器正在运行，则停止定时器
**返 回 值:  无
*********************************************************************/
void retransmission_poll(void)
{
    retransmission_check();

    // 检查队列是否为空
    if (retrans_queue_is_empty())
    {
        // 停止定时器
        if (k_timer_remaining_get(&s_retrans_check_timer) != 0)
        {
            k_timer_stop(&s_retrans_check_timer);
            MY_LOG_INF("s_retrans_check_timer : STOP");
        }
    }
}

/********************************************************************
**函数名称:  async_add_item
**入口参数:  cmd_name  ---        指令头MODE（LTE+CMD=88888888888,MODE,1,0,1,1,0#）
**            id        ---       关联的ID 88888888888
**出口参数:  无
**函数功能:  向异步队列中添加新的等待项，用于后续匹配蓝牙回复
**返 回 值:  0表示添加成功，-1表示队列已满
********************************************************************/
int async_add_item(char *cmd_name, char *id)
{
    int i = 0;
    for (i = 0; i < ASYNC_QUEUE_SIZE; i++)
    {
        if (!g_async_queue[i].used)
        {
            strcpy(g_async_queue[i].cmd_name, cmd_name);
            strcpy(g_async_queue[i].id, id);
            g_async_queue[i].used = 1;
            return 0;
        }
    }

    return -1; // 队列满
}

/********************************************************************
**函数名称:  async_match_and_resp
**入口参数:  data      ---        数据（格式：指令头,回复内容）如CUNLOCK,Unlock failed. No unlock state detected.
**出口参数:  无
**函数功能:  解析数据头，在队列中查找匹配项；若匹配成功，将元素前移并发送响应给LTE
**返 回 值:  0表示匹配成功并发送，-1表示未匹配到对应指令
********************************************************************/
int async_match_and_resp(char *data)
{
    int i = 0;
    int index = -1;
    char cmd_name[16];
    msg_t msg;
    char *resp;
    char id[16];

    // 拿指令头匹配
    my_get_str_at_pos(data, 0, ',', cmd_name, sizeof(cmd_name));
    for (i = 0; i < ASYNC_QUEUE_SIZE; i++)
    {
        if (g_async_queue[i].used && strcmp(g_async_queue[i].cmd_name, cmd_name) == 0)
        {
            index = i;
            strcpy(id, g_async_queue[i].id);
            break;
        }
    }
    // 成功匹配
    if (index != -1)
    {
        // 将后面的元素依次前移
        for (i = index; i < ASYNC_QUEUE_SIZE - 1 && g_async_queue[i + 1].used; i++)
        {
            memcpy(&g_async_queue[i], &g_async_queue[i + 1], sizeof(async_resp_tiem_t));
        }

        // 清空最后被占用的元素
        memset(&g_async_queue[i], 0, sizeof(async_resp_tiem_t));

        //应答
        MY_MALLOC_BUFFER(resp, strlen(data) + strlen(id) + 20);
        if (resp == NULL)
        {
            MY_LOG_ERR("resp malloc failed");
            return 0;
        }

        sprintf(resp, "LTE+CMD=%s,%s", id, data);

        // 构建消息结构体并发送给LTE模块
        msg.msgID = MY_MSG_LTE_BLE_DATA;
        msg.pData = resp;
        msg.DataLen = strlen(resp);

        my_send_msg_data(MOD_LTE, MOD_LTE, &msg);

        return 0;
    }
    else
    {
        // 未匹配
        return -1;
    }
}

/********************************************************************
 * 函数名称: my_lte_msg_queue_init
 * 入口参数: 无
 * 出口参数: 无
 * 函数功能: 初始化LTE缓存消息队列，包括互斥锁和队列索引
 * 返回值: 0 --- 成功
 * 注意事项: 必须在使用消息队列前调用此函数进行初始化
 ********************************************************************/
static int my_lte_msg_queue_init(void)
{
    s_lte_msg_queue.head = 0;
    s_lte_msg_queue.tail = 0;
    s_lte_msg_queue.count = 0;

    return 0;
}

/********************************************************************
 * 函数名称: my_lte_enqueue_msg
 * 入口参数: msg_content  ---        消息内容指针(输入)
 *           msg_len      ---        消息长度(输入)
 * 出口参数: 无
 * 函数功能: 将消息加入LTE消息队列，队列满时移除最旧消息
 * 返回值: 0 --- 成功
 *         -EINVAL --- 参数无效
 *         -ENOMEM --- 内存分配失败
 * 注意事项: 函数内部会为消息内容动态分配内存，调用者需确保消息有效
 ********************************************************************/
static int my_lte_enqueue_msg(const char *msg_content, uint16_t msg_len)
{
    int ret = 0;
    char *new_msg = NULL;

    if (msg_content == NULL || msg_len == 0)
    {
        return -EINVAL;
    }

    // 如果队列已满，移除最旧的消息（head位置）
    if (s_lte_msg_queue.count >= LTE_MSG_QUEUE_SIZE)
    {
        // 释放最旧消息的内存
        if (s_lte_msg_queue.queue[s_lte_msg_queue.head].msg_content != NULL)
        {
            MY_LOG_INF("Release old message: %s", s_lte_msg_queue.queue[s_lte_msg_queue.head].msg_content);
            MY_FREE_BUFFER(s_lte_msg_queue.queue[s_lte_msg_queue.head].msg_content);
            s_lte_msg_queue.queue[s_lte_msg_queue.head].msg_content = NULL;
        }

        // 移动head指针，移除最旧元素
        s_lte_msg_queue.head = (s_lte_msg_queue.head + 1) % LTE_MSG_QUEUE_SIZE;
        s_lte_msg_queue.count--;
    }

    // 为新消息内容分配内存并复制
    MY_MALLOC_BUFFER(new_msg, msg_len + 1);
    if (new_msg == NULL)
    {
        ret = -ENOMEM;
        goto exit;
    }

    memcpy(new_msg, msg_content, msg_len);
    new_msg[msg_len] = '\0';  // 确保字符串终止

    // 存储到队列尾部
    s_lte_msg_queue.queue[s_lte_msg_queue.tail].msg_content = new_msg;
    s_lte_msg_queue.queue[s_lte_msg_queue.tail].msg_len = msg_len;

    // MY_LOG_INF("Enqueue message: %s", s_lte_msg_queue.queue[s_lte_msg_queue.tail].msg_content);

    // 更新队列指针
    s_lte_msg_queue.tail = (s_lte_msg_queue.tail + 1) % LTE_MSG_QUEUE_SIZE;
    s_lte_msg_queue.count++;

exit:
    return ret;
}

/********************************************************************
 * 函数名称: my_lte_process_queued_msgs
 * 入口参数: 无
 * 出口参数: 无
 * 函数功能: 处理队列中所有排队的消息，发送到LTE模块并释放内存
 * 返回值: 无
 * 注意事项: 函数会清空队列中所有消息，调用时需确保LTE模块已就绪
 ********************************************************************/
static void my_lte_process_queued_msgs(void)
{
    lte_pending_msg_t *pending_msg;

    // 遍历并发送所有排队的消息
    while (s_lte_msg_queue.count > 0)
    {
        pending_msg = &s_lte_msg_queue.queue[s_lte_msg_queue.head];

        // 直接发送到LTE模块，不修改消息内容
        if (pending_msg->msg_content != NULL)
        {
            // 直接发送原始消息内容
            my_lte_uart_send((uint8_t*)pending_msg->msg_content, pending_msg->msg_len);
        }

        // 清理分配的内存
        if (pending_msg->msg_content != NULL)
        {
            MY_FREE_BUFFER(pending_msg->msg_content);
            pending_msg->msg_content = NULL;
        }

        // 移动队列头部指针
        s_lte_msg_queue.head = (s_lte_msg_queue.head + 1) % LTE_MSG_QUEUE_SIZE;
        s_lte_msg_queue.count--;
    }
}

/********************************************************************
 * 函数名称: my_lte_msg_queue_clear
 * 入口参数: 无
 * 出口参数: 无
 * 函数功能: 清空LTE缓存消息队列，释放所有已分配的消息内存并重置队列状态
 * 返回值: 无
 ********************************************************************/
static void my_lte_msg_queue_clear(void)
{
    int i;

    for (i = 0; i < LTE_MSG_QUEUE_SIZE; i++)
    {
        if (s_lte_msg_queue.queue[i].msg_content != NULL)
        {
            MY_FREE_BUFFER(s_lte_msg_queue.queue[i].msg_content);
            s_lte_msg_queue.queue[i].msg_content = NULL;
        }
        s_lte_msg_queue.queue[i].msg_len = 0;
    }

    s_lte_msg_queue.head = 0;
    s_lte_msg_queue.tail = 0;
    s_lte_msg_queue.count = 0;
}

/********************************************************************
**函数名称:  my_lte_pwroff_handle
**入口参数:  无
**出口参数:  无
**函数功能:  LTE模块断电处理，完成发送等待、UART挂起、状态清理及电源关闭
**返 回 值:  无
**注意事项:  1. OTA升级中或产测模式下不执行实际断电，仅清理状态
**           2. 强制重置信号量与TX标志，防止异常阻塞
**           3. 等待发送完成超时后仍强制继续断电流程
*********************************************************************/
static void my_lte_pwroff_handle(void)
{
    int ret;

    // OTA进行中 或 产测模式下，拒绝整个关机流程，保持所有状态不变
    if (g_lte_ota_in_progress || s_lte_factory)
    {
        MY_LOG_INF("LTE pwr off blocked: ota=%d factory=%d",
                    g_lte_ota_in_progress, s_lte_factory);
        return;
    }

    k_timer_stop(&s_lte_uart_ctx.idle_timer);

    // 若 UART 正在发送，利用信号量等待发送完成（中断回调直接释放）
    if (s_lte_uart_ctx.tx_busy)
    {
        ret = k_sem_take(&s_TxDoneSem, K_MSEC(LTE_UART_TX_WAIT_MS));
        if (ret != 0)
        {
            MY_LOG_WRN("Wait TX done timeout, force pwr off");
        }
    }

    if (s_lte_uart_ctx.active)
    {
        my_pm_device_suspend(MY_PM_DEV_LTE);
    }

    // 强制清理状态，防止异常场景下信号量或标志位未复位
    s_lte_uart_ctx.tx_busy = false;
    k_sem_give(&s_TxDoneSem);
    my_rb_clear(&s_lte_rb);
    my_lte_msg_queue_clear();

    // 断LTE的电源
    my_lte_pwr_on(false);
    my_stop_timer(MY_TIMER_LTE_PULSE);
    g_bLteReady = 0;

    #if RETRANSMIT_CHECK_ENABLED
        //清空重传队列
        init_retransmission_queue();
    #endif
}

/********************************************************************
 * 函数名称: my_lte_send_msg
 * 入口参数: msg_content  ---        消息内容指针(输入)
 *           msg_len      ---        消息长度(输入)
 * 出口参数: 无
 * 函数功能: LTE消息发送统一入口，根据模块状态选择直接发送或排队
 * 返回值: 0 --- 成功
 *         -EINVAL --- 参数无效
 *         -ENOMEM --- 内存分配失败
 * 注意事项: LTE未就绪时消息会被加入队列，需调用process函数处理
 ********************************************************************/
static int my_lte_send_msg(const char *msg_content, uint16_t msg_len)
{
    if (msg_content == NULL || msg_len == 0)
    {
        return -EINVAL;
    }

    // 检查LTE模块是否就绪
    if (g_bLteReady)
    {
        // MY_LOG_INF("send uart message: %s", msg_content);

        // LTE已就绪，直接发送原始消息
        return my_lte_uart_send((uint8_t*)msg_content, msg_len);
    }
    else
    {
        // LTE未就绪，将消息排队等待
        return my_lte_enqueue_msg(msg_content, msg_len);
    }
}

/********************************************************************
**函数名称:  set_lte_boot_reason
**入口参数:  reason   ---        要设置的开机原因枚举值
**出口参数:  无
**函数功能:  设置 LTE 开机原因，用于 4G 开机完成握手时回复
**返 回 值:  无
*********************************************************************/
void set_lte_boot_reason(lte_boot_reason_t reason)
{
    s_lteBootReason = reason;
}

/********************************************************************
**函数名称:  get_lte_boot_reason
**入口参数:  无
**出口参数:  无
**函数功能:  获取当前记录的 LTE 开机原因
**返 回 值:  当前开机原因枚举值
*********************************************************************/
lte_boot_reason_t get_lte_boot_reason(void)
{
    return s_lteBootReason;
}

/********************************************************************
**函数名称:  my_lte_task
**入口参数:  无
**出口参数:  无
**函数功能:  LTE 模块主线程，处理来自消息队列的任务
**返 回 值:  无
*********************************************************************/
static void my_lte_task(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    msg_t msg;

    MY_LOG_INF("LTE thread started");

    // 检查重启原因是否为OTA升级
    if (gConfigParam.ota_config.ble_ota_reboot == true)
    {
        lte_send_command("OTA", "EXIT");
        gConfigParam.ota_config.ble_ota_reboot = false;
        my_user_data_write(ZMS_ID_OTA_CONFIG, &gConfigParam.ota_config, sizeof(ota_config_t));
    }

    // 初始化时间指令,从4G网络获取时间同步
    #if RETRANSMIT_CHECK_ENABLED
        lte_send_cmd_with_retry("TIME", "1");
    #else
        lte_send_command("TIME", "1");
    #endif

    for (;;)
    {
        my_recv_msg(&my_lte_msgq, (void *)&msg, sizeof(msg_t), K_FOREVER);

        switch (msg.msgID)
        {
            /* TODO: 添加 LTE 相关的消息处理逻辑 */
            case MY_MSG_LTE_TX_DONE:
            case MY_MSG_LTE_TX_ABORTED:
                s_lte_uart_ctx.tx_busy = false;
                lte_uart_activity_kick();
                break;

            case MY_MSG_LTE_UART_IDLE:
                if (s_lte_uart_ctx.active && lte_uart_can_suspend())
                {
                    my_pm_device_suspend(MY_PM_DEV_LTE);
                }
                break;

            case MY_MSG_LTE_PWRON:
                my_lte_pwr_on(true);
                break;

            case MY_MSG_LTE_PWROFF:
                my_lte_pwroff_handle();
                //2s延时，防止断电后立马上电，导致模块没法真正复位
                k_sleep(K_MSEC(2000));

                break;

            // 收到4G发送的消息,例如返回UTC时间,在里面进行数据解析
            case MY_MSG_LTE_REV:
            {
                static uint8_t read_buf[128];
                int len = 0;

                while (1)
                {
                    memset(read_buf, 0, sizeof(read_buf));

                    // 读取数据（无锁安全）
                    len = my_rb_read(&s_lte_rb, read_buf, sizeof(read_buf));
                    if (len > 0)
                    {
                        my_lte_handle_recv(read_buf, len);
                    }
                    else
                    {
                        break;
                    }
                }
                lte_uart_activity_kick();
            }
                break;

            case MY_MSG_LTE_BLE_DATA:
                // 使用统一的消息发送接口
                my_lte_send_msg((const char*)msg.pData, msg.DataLen);
                // 释放动态分配的内存
                if(msg.pData != NULL)
                {
                    MY_FREE_BUFFER(msg.pData);
                    msg.pData = NULL;
                }
                break;

            case MY_MSG_RETRANS_CHECK:
                retransmission_poll();
                break;

            case MY_MSG_ADD_RETRANS_QUEUE:
                add_to_retrans_queue((char*)msg.pData);

                // 释放动态分配的内存
                if(msg.pData != NULL)
                {
                    MY_FREE_BUFFER(msg.pData);
                    msg.pData = NULL;
                }
                break;

            case MY_MSG_LTE_PULSE:
                // 脉冲消息处理
                send_lte_pulse();
                break;

            case MY_MSG_LTE_WAKEUP:
                MY_LOG_INF("LTE wakeup pin triggered, resuming UART");
                lte_uart_ensure_active();
                break;

            default:
                break;
        }
    }
}

/********************************************************************
**函数名称:  lte_uart_cb
**入口参数:  dev      ---        UART 设备句柄
**            evt      ---        UART 事件结构体
**            user_data ---       用户自定义数据
**出口参数:  无
**函数功能:  LTE 模块 UART 异步回调处理函数，实现回环测试逻辑
**返 回 值:  无
*********************************************************************/
static void lte_uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
    ARG_UNUSED(user_data);

    switch (evt->type)
    {
        case UART_TX_DONE:
            // MY_LOG_INF("LTE UART TX Done");
            my_send_msg(MOD_LTE, MOD_LTE, MY_MSG_LTE_TX_DONE);

            // 传输完成，释放信号量
            k_sem_give(&s_TxDoneSem);
            break;

        case UART_RX_RDY:
            // MY_LOG_INF("LTE UART RX Ready, len: %d", evt->data.rx.len);
#if 0
            /* 串口回环测试：将收到的数据直接原样发回 */
            my_lte_uart_send(&evt->data.rx.buf[evt->data.rx.offset], evt->data.rx.len);
#else
            // 将收到的数据写入循环缓冲区
            my_rb_write(&s_lte_rb, &evt->data.rx.buf[evt->data.rx.offset], evt->data.rx.len);
            // 通知LTE线程读取循环缓冲区数据
            my_send_msg(MOD_MAIN, MOD_LTE, MY_MSG_LTE_REV);
#endif
            break;

        case UART_RX_BUF_REQUEST:
            /* 填充下一个接收缓冲区 */
            uart_rx_buf_rsp(dev, lte_next_buf, LTE_UART_BUF_SIZE);
            break;

        case UART_RX_BUF_RELEASED:
            /* 记录被释放的缓冲区，作为下一次使用的备选 */
            lte_next_buf = evt->data.rx_buf.buf;
            break;

        case UART_RX_DISABLED:
            // uart活跃时，才允许重新开启
            if (s_lte_uart_ctx.active)
            {
                uart_rx_enable(dev, s_lte_rx_buf_1, LTE_UART_BUF_SIZE, 10 * USEC_PER_MSEC);
            }
            break;

        case UART_TX_ABORTED:
            MY_LOG_WRN("LTE UART TX Aborted");
            my_send_msg(MOD_LTE, MOD_LTE, MY_MSG_LTE_TX_ABORTED);
            // 发送异常中止时也要释放等待信号量，避免发送线程永久阻塞
            k_sem_give(&s_TxDoneSem);
            break;

        default:
            break;
    }
}

/********************************************************************
**函数名称:  my_lte_uart_send
**入口参数:  data      ---        发送数据
**            len       ---        发送长度
**出口参数:  无
**函数功能:  LTE 模块发送数据函数，带唤醒前导字节
**返 回 值:  0表示成功，其他表示失败
*********************************************************************/
int my_lte_uart_send(const uint8_t *data, uint16_t len)
{
    static uint8_t wake_byte[3] = {0xAA, 0x0D, 0x0A};  // 唤醒字节
    static uint8_t s_sendDataBuf[UART_TX_BUFFER_SIZE] = {0};
    int ret = 0;
#if 0
    if (len == 0 || data == NULL)
    {
        return -EINVAL;
    }
#endif

    if (!g_bLteReady) return -1;

    if (len > UART_TX_BUFFER_SIZE)
    {
        MY_LOG_INF("uart data is too large:%d", len);
        return -1;
    }

    if (lte_uart_ensure_active() < 0)
    {
        return -1;
    }

    // 刷新LTE UART活跃定时器，延迟挂起以维持通信
    lte_uart_activity_kick();

    // 等待上一次传输完成，增加等待超时避免异常状态下永久阻塞
    ret = k_sem_take(&s_TxDoneSem, K_MSEC(LTE_UART_TX_WAIT_MS));
    if (ret != 0)
    {
        return ret;
    }

    s_lte_uart_ctx.tx_busy = true;

    // 发送唤醒字节：如果4G在休眠状态，这个字节会将其唤醒
    ret = uart_tx(lte_uart_dev, wake_byte, 3, SYS_FOREVER_MS);
    if (ret != 0)
    {
        s_lte_uart_ctx.tx_busy = false;
        k_sem_give(&s_TxDoneSem);
        return ret;
    }

    // 等待唤醒字节发送完成，增加等待超时避免异常状态下永久阻塞
    ret = k_sem_take(&s_TxDoneSem, K_MSEC(LTE_UART_TX_WAIT_MS));
    if (ret != 0)
    {
        s_lte_uart_ctx.tx_busy = false;
        k_sem_give(&s_TxDoneSem);
        return ret;
    }

    // TODO 唤醒时间仅需几百微秒，几乎不影响后续数据传输，待实际测试验证，暂定等待1ms
    k_sleep(K_MSEC(1));
    // ! uart_tx发送是异步处理,传进去的data需要是静态的才能保证数据不丢失或者执行完uart_tx延时一会确保数据传输完.
    memcpy(s_sendDataBuf, data, len);

    // 发送实际数据，此时4G模块已经处于唤醒状态，可以正常接收数据
    ret = uart_tx(lte_uart_dev, s_sendDataBuf, len, SYS_FOREVER_MS);
    if (ret != 0)
    {
        s_lte_uart_ctx.tx_busy = false;
        k_sem_give(&s_TxDoneSem);
        return ret;
    }

    return 0;
}

/********************************************************************
**函数名称:  get_lte_power_state
**入口参数:  无
**出口参数:  无
**函数功能:  获取LTE模块电源状态
**返 回 值:  true:电源打开，false:电源关闭
*********************************************************************/
bool get_lte_power_state(void)
{
    return s_lte_power_state;
}

/********************************************************************
**函数名称:  my_lte_pwr_on
**入口参数:  on      ---        是否开启
**出口参数:  无
**函数功能:  LTE 模块电源控制函数
**返 回 值:  0 表示成功
*********************************************************************/
int my_lte_pwr_on(bool on)
{
    int err;

    /* 检查当前电源状态，避免重复操作 */
    if (s_lte_power_state == on)
    {
        /* 状态相同，无需操作 */
        MY_LOG_INF("LTE Power: already %s", on ? "ON" : "OFF");
        return 0;
    }

    /* 执行电源控制操作 */
    err = gpio_pin_set_dt(&lte_pwr_gpio, on ? 1 : 0);
    if (err == 0)
    {
        /* 操作成功，更新状态 */
        s_lte_power_state = on;
        MY_LOG_INF("LTE Power Control: %s", on ? "Power ON" : "Power OFF");
    }
    else
    {
        MY_LOG_ERR("LTE Power Control failed (err %d)", err);
    }

    return err;
}

/********************************************************************
**函数名称:  my_handle_at_pcba_cmd
**入口参数:  pParam   ---        AT命令参数数组
**           nParam   ---        参数个数
**出口参数:  无
**函数功能:  处理PCBA工厂命令，包括FF/GG/JATAG/JGTAG/MODIFYGV/IMEI/MAC等指令
**返 回 值:  响应字符串指针
*********************************************************************/
static char *my_handle_at_pcba_cmd(char **pParam, int nParam)
{
    static char resp[256];
    const lic_ff_t *lic_ff;
    const lic_gg_t *lic_gg;
    uint16_t ECDH_GValue;
    uint8_t data_buff[64] = {0};
    const gsm_imei_t *gsmImei;
    const uint8_t *edr_addr;
    uint8_t version = 0;
    int ret;

    LOG_INF("%s: %s", __func__, pParam[0]);
    memset(resp, 0, sizeof(resp));

    // 产测指令至少有两个参数，且第一个参数一定是“PCBA”
    // AT^GT_CM=PCBA,<xxx>
    if (nParam < 2 || CMD_MATCHED(pParam[0], "PCBA") == 0)
    {
        sprintf(resp, "Factory CMD ERROR");
        return resp;
    }

    // SMT测试指令
    // AT^GT_CM=PCBA,BT,xxxx
    if (CMD_MATCHED(pParam[1], "BT"))
    {
        LOG_INF("%s: %s", __func__, pParam[1]);
        if (nParam < 3)
        {
            sprintf(resp, "BT params error");
            return resp;
        }
        else if (CMD_MATCHED(pParam[2], "FF"))
        {
            // AT^GT_CM=PCBA,BT,FF
            if (nParam == 3)
            {
                lic_ff = my_param_get_ff();
                if (lic_ff->flag == FLAG_VALID)
                {
                    hex2hexstr(lic_ff->hex, LICENSE_FF_STR_LEN / 2, data_buff, sizeof(data_buff));
                    sprintf(resp, "RETURN_FF:%s", data_buff);
                }
                else
                {
                    sprintf(resp, "RETURN_FF");
                }
            }
            // AT^GT_CM=PCBA,BT,FF,xxxx
            else
            {
                my_param_set_ff(pParam[3], strlen(pParam[3]));
                sprintf(resp, "RETURN_FF_SET_OK");
            }
        }
        else if (CMD_MATCHED(pParam[2], "GG"))
        {
            // AT^GT_CM=PCBA,BT,GG
            if (nParam == 3)
            {
                lic_gg = my_param_get_gg();
                if (lic_gg->flag == FLAG_VALID)
                {
                    hex2hexstr(lic_gg->hex, LICENSE_GG_STR_LEN / 2, data_buff, sizeof(data_buff));
                    sprintf(resp, "RETURN_GG:%s", data_buff);
                }
                else
                {
                    sprintf(resp, "RETURN_GG");
                }
            }
            // AT^GT_CM=PCBA,BT,GG,xxxx
            else
            {
                my_param_set_gg(pParam[3], strlen(pParam[3]));
                sprintf(resp, "RETURN_GG_SET_OK");
            }
        }
        else if (CMD_MATCHED(pParam[2], "JATAG"))
        {
            // AT^GT_CM=PCBA,BT,JATAG,ON/OFF
            if (nParam == 4)
            {
                ret = my_param_set_jatag_or_jgtag(pParam[2], pParam[3]);
                if (ret == 0) {
                    sprintf(resp, "RETURN_JATAG_%s_OK", pParam[3]);
                }else {
                    sprintf(resp, "RETURN_JATAG_%s_FAIL", pParam[3]);
                }
            }
            else
            {
                sprintf(resp, "RETURN_JATAG_SET_FAIL");
            }
        }
        else if (CMD_MATCHED(pParam[2], "JGTAG"))
        {
            // AT^GT_CM=PCBA,BT,JGTAG,ON/OFF
            if (nParam == 4)
            {
                ret = my_param_set_jatag_or_jgtag(pParam[2], pParam[3]);
                if (ret == 0) {
                    sprintf(resp, "RETURN_JGTAG_%s_OK", pParam[3]);
                }else {
                    sprintf(resp, "RETURN_JGTAG_%s_FAIL", pParam[3]);
                }
            }
            else
            {
                sprintf(resp, "RETURN_JGTAG_SET_FAIL");
            }
        }
        else if (CMD_MATCHED(pParam[2], "MODIFYGV"))
        {
            // AT^GT_CM=PCBA,BT,MODIFYGV
            if (nParam == 3)
            {
                ECDH_GValue = my_param_get_Gvalue();
                sprintf(resp, "RETURN_GV:%d (%04X)", ECDH_GValue, ECDH_GValue);
            }
            // AT^GT_CM=PCBA,BT,MODIFYGV,xxxx
            else
            {
                ret = my_param_set_Gvalue(pParam[3]);
                if (ret == 0){
                    sprintf(resp, "RETURN_MODIFYGV_SET_OK");
                } else {
                    sprintf(resp, "RETURN_MODIFYGV_SET_FAIL");
                }
            }
        }
        else if (CMD_MATCHED(pParam[2], "IMEI"))
        {
            // AT^GT_CM=PCBA,BT,IMEI
            if (nParam == 3)
            {
                gsmImei = my_param_get_imei();
                if (gsmImei->flag == FLAG_VALID)
                {
                    memcpy(data_buff, gsmImei->hex, sizeof(gsmImei->hex));
                    sprintf(resp, "RETURN_IMEI:%s", data_buff);
                }
                else
                {
                    sprintf(resp, "RETURN_IMEI");
                }
            }
            // AT^GT_CM=PCBA,BT,IMEI,xxxx
            else
            {
                ret = my_param_set_imei(pParam[3], strlen(pParam[3]));
                if (ret == 0){
                    //TODO 更新广播数据？
                    sprintf(resp, "RETURN_IMEI_SET_OK");
                } else {
                    sprintf(resp, "RETURN_IMEI_SET_FAIL");
                }
            }
        }
        else if (CMD_MATCHED(pParam[2], "MAC"))
        {
            // AT^GT_CM=PCBA,BT,MAC
            if (nParam == 3)
            {
                edr_addr = bt_get_mac_addr();
                sprintf(resp, "RETURN_BT_MAC:%02X%02X%02X%02X%02X%02X",
                edr_addr[5],edr_addr[4],edr_addr[3],edr_addr[2],edr_addr[1],edr_addr[0]);
            }
            // AT^GT_CM=PCBA,BT,MAC,xxxx
            else
            {
                ret = my_param_set_mac(pParam[3], strlen(pParam[3]));
                if (ret == 0){
                    sprintf(resp, "RETURN_BT_MAC_SET_OK");
                } else {
                    sprintf(resp, "RETURN_BT_MAC_SET_FAIL");
                }
            }
        }
    }
    else if (CMD_MATCHED(pParam[1], "MCU")) // AT^GT_CM=PCBA,MCU,xxxxxxxx
    {
         if (nParam < 3)
        {
            sprintf(resp, "MCU params error");
            return resp;
        }
        if (CMD_MATCHED(pParam[2], "LED"))
        {
             if (nParam < 4)
            {
                sprintf(resp, "MCU params error");
                return resp;
            }
            if (CMD_MATCHED(pParam[3], "ON"))
            {
                batt_led_set_level(3);
                lock_led_set(true);
                sprintf(resp, "RETURN_LED_ON");
            }
            else if (CMD_MATCHED(pParam[3], "OFF"))
            {
                batt_led_set_level(0);
                lock_led_set(false);
                sprintf(resp, "RETURN_LED_OFF");
            }
        }
        else if (CMD_MATCHED(pParam[2], "MAC"))
        {
            edr_addr = bt_get_mac_addr();
            if (edr_addr != NULL)
            {
                sprintf(resp, "RETURN_MCU_MAC:%02X%02X%02X%02X%02X%02X",
                        edr_addr[5],edr_addr[4],edr_addr[3],edr_addr[2],edr_addr[1],edr_addr[0]);
            }
        }
        else if (CMD_MATCHED(pParam[2], "GSENSOR"))
        {
            sprintf(resp, "RETURN_MCU_GSENSOR:0x%02X", get_chip_id());
        }
        else if (CMD_MATCHED(pParam[2], "NFC"))
        {
            fm175xx_read_reg(JREG_VERSION, &version);
            sprintf(resp, "RETURN_MCU_NFC:0x%02X", version);
        }

    }

    return resp;
}

/********************************************************************
**函数名称:  my_handle_at_factory_cmd
**入口参数:  pParam   ---        AT命令参数数组
**           nParam   ---        参数个数
**出口参数:  无
**函数功能:  处理AT工厂命令
**返 回 值:  响应字符串指针
*********************************************************************/
static char *my_handle_at_factory_cmd(char **pParam, int nParam)
{
    static char resp[256];

    memset(resp, 0, sizeof(resp));

    LOG_INF("%s: %s", __func__, pParam[0]);

    // SMT相关指令
    if (CMD_MATCHED(pParam[0], "PCBA"))
    {
        return my_handle_at_pcba_cmd(pParam, nParam);
    }
    // TODO

    return resp;
}

/********************************************************************
**函数名称:  my_at_test
**入口参数:  argc     ---        参数个数
**           argv     ---        参数数组
**出口参数:  无
**函数功能:  处理AT测试命令
**返 回 值:  0表示成功，-1表示参数错误
*********************************************************************/
int my_at_test(int argc, char *argv[])
{
    char szValue[30] = {0};

    if (argc < 2) return -1;

    strncpy(szValue, argv[1], 30);

    if (strcmp(szValue, "CPUINFO") == 0)
    {
        MY_LOG_INF("==========>%s", szValue);
    }
    else
    {
        MY_LOG_INF("Unrecognized Testing.");
    }

    return 0;
}

/********************************************************************
**函数名称:  GetCmdMatche
**入口参数:  cmdline  ---        命令行字符串
**出口参数:  无
**函数功能:  匹配并查找命令在命令表中的索引
**返 回 值:  命令索引（成功），-1（未找到）
*********************************************************************/
static int GetCmdMatche(char *cmdline)
{
    int i;

    for (i = 0; AT_CMD_INNER[i].cmd != NULL; i++)
    {
        if (strlen(cmdline) != strlen(AT_CMD_INNER[i].cmd))
            continue;
        if (strncmp(AT_CMD_INNER[i].cmd, cmdline, strlen(AT_CMD_INNER[i].cmd)) == 0)
            return i;
    }

    return -1;
}

/********************************************************************
**函数名称:  ParseArgs
**入口参数:  cmdline  ---        命令行原始内容
**           argc     ---        输出：解析后参数个数
**           argv     ---        输出：参数内容数组
**出口参数:  无
**函数功能:  解析命令行参数，将命令行字符串分解为参数数组
**返 回 值:  无
*********************************************************************/
static void ParseArgs(char *cmdline, int *argc, char **argv)
{
#define STATE_WHITESPACE    0
#define STATE_WORD          1

    char *c;
    int state = STATE_WHITESPACE;
    int i;

    *argc = 0;

    if (strlen(cmdline) == 0)
        return;

    /* convert all tabs into single spaces */
    c = cmdline;
    while (*c != '\0')
    {
        if (*c == '\t')
            *c = ' ';
        c++;
    }

    c = cmdline;
    i = 0;

    /* now find all words on the command line */
    while (*c != '\0')
    {
        if (state == STATE_WHITESPACE)
        {
            if (*c != ' ')
            {
                argv[i] = c;        //将argv[i]指向c
                i++;
                state = STATE_WORD;
            }
        }
        else
        { /* state == STATE_WORD */
            if (*c == ' ')
            {
                *c = '\0';
                state = STATE_WHITESPACE;
            }
        }
        c++;
    }

    *argc = i;

#undef STATE_WHITESPACE
#undef STATE_WORD
}

/********************************************************************
**函数名称:  my_parse_cmd_line
**入口参数:  cmdline  ---        命令行字符串
**           flag     ---        分隔符
**           argc     ---        输出：参数个数
**           argv     ---        输出：参数数组
**出口参数:  无
**函数功能:  按指定分隔符解析命令行参数
**返 回 值:  无
*********************************************************************/
void my_parse_cmd_line(char *cmdline, char flag, int *argc, char **argv)
{
    char *c = cmdline;
    int state = 0;
    int i = 0;
    int max_args = *argc;

    *argc = 0;

    if (strlen(cmdline) == 0)
        return;

    /* now find all words on the command line */
    while (*c != '\0')
    {
        if (state == 0)
        {
            if (*c != flag)
            {
                argv[i] = c;        //将argv[i]指向c
                state = 1;
                i++;

                if (i == max_args) break;
            }
        }
        else
        { /* state == 1*/
            if (*c == flag)
            {
                *c = '\0';
                state = 0;
            }
        }
        c++;
    }

    *argc = i;
}

/********************************************************************
**函数名称:  my_at_factory_cmd
**入口参数:  pfactorycmd ---    产测命令字符串
**出口参数:  无
**函数功能:  解析并处理产测AT命令
**返 回 值:  0表示成功
*********************************************************************/
static int my_at_factory_cmd(char *pfactorycmd)
{
    int argc = 0; // 输入输出参数
    char *argv[MAX_ARGS] = { 0 };
    char *resp_buf;

    LOG_INF("%s: %s", __func__, pfactorycmd);
    my_parse_cmd_line(pfactorycmd + strlen(FACTORY_CMD_HEADER), ',' , &argc, argv);

    resp_buf = my_handle_at_factory_cmd(argv, argc);

    MY_LOG_INF("%s", resp_buf);
    my_lte_send_msg(resp_buf, strlen(resp_buf));

    return 0;
}

/********************************************************************
**函数名称:  send_lte_pulse
**入口参数:  无
**出口参数:  无
**函数功能:  发送脉冲消息
**返 回 值:  无
*********************************************************************/
static void send_lte_pulse(void)
{
    char buf[10] = {0};

    // 脉冲计数器增加
    s_lte_pulse_count++;
    // 构造脉冲消息: LTE+PULSE=<脉冲计数器>
    snprintf(buf, sizeof(buf), "%d", s_lte_pulse_count);

    #if RETRANSMIT_CHECK_ENABLED
        lte_send_cmd_with_retry("PULSE", buf);
    #else
        lte_send_command("PULSE", buf);
    #endif
}

/********************************************************************
**函数名称:  lte_pulse_timer_handler
**入口参数:  param    ---        定时器参数
**出口参数:  无
**函数功能:  脉冲定时器处理函数
*********************************************************************/
void lte_pulse_timer_handler(void *param)
{
    // 发送脉冲消息
    my_send_msg(MOD_LTE, MOD_LTE, MY_MSG_LTE_PULSE);
}

/********************************************************************
**函数名称:  my_lte_handle_power_on
**入口参数:  data     ---        去掉协议头后的参数字符串(输入)
**出口参数:  无
**函数功能:  处理4G模块开机完成指令，解析参数并回复应答报文
**返 回 值:  0 表示成功
**注意事项:  4G异常重启时，开机原因固定返回255
*********************************************************************/
static int my_lte_handle_power_on(char *data)
{
    char power_state_str[4] = {0};
    char power_reason_str[4] = {0};
    lte_boot_reason_t boot_reason;
    char resp_buf[128] = {0};
    int nRespLen;

    // 解析4G发来的参数: <上电状态>,<上电原因>,<4G版本号>
    my_get_str_at_pos(data, 0, ',', power_state_str, sizeof(power_state_str));
    my_get_str_at_pos(data, 1, ',', power_reason_str, sizeof(power_reason_str));
    my_get_str_at_pos(data, 2, ',', g_lte4GVersion, sizeof(g_lte4GVersion));

    s_4GPoweronStatus = atoi(power_state_str);

    MY_LOG_INF("LTE PWRON state=%s reason=%s ver=%s", power_state_str, power_reason_str, g_lte4GVersion);

    // 4G模块已就绪，允许后续数据收发
    g_bLteReady = 1;

    // 构造应答报文: LTE+PWRON=OK,<开机原因>,<蓝牙版本号>
    if (s_4GPoweronStatus == LTE_PWR_STATE_ABNORMAL)
    {
        // 异常重启时，开机原因默认填255
        boot_reason = LTE_BOOT_REASON_RESERVED;
    }
    else
    {
        boot_reason = get_lte_boot_reason();
    }

    if (g_lte_ota_in_progress == true)
    {
        g_lte_ota_in_progress = false;
    }

    nRespLen = snprintf(resp_buf, sizeof(resp_buf), "%sOK,%d,%s\r\n",
                        LTE_PWRON, (int)boot_reason, SOFTWARE_VERSION);

    my_lte_send_msg(resp_buf, (uint16_t)nRespLen);

    // 发送当前工作模式给LTE模块
    send_work_mode_command(gConfigParam.device_workmode_config.workmode_config.current_mode);

    // 处理排队的消息
    my_lte_process_queued_msgs();

    // 启动心跳定时器
    s_lte_pulse_count = 0;
    my_start_timer(MY_TIMER_LTE_PULSE, 60 * 1000, true, lte_pulse_timer_handler);

    return 0;
}

/*
LTE+BTSET=ADVINT,<TC>,<TA>,< TF >
LTE+BTSET=ADVNME,<广播名称>
LTE+BTSET=TAGADV,<开关状态>
LTE+BTSET=CRFPWR,<功率值>
LTE+BTSET=SCANSET,T1,T2
LTE+BTSET=SCANREQ,< 超时时间 >
*/
static int my_lte_handle_bt_set(char *data)
{
    char param_name[16] = {0};
    char value_buff[16] = {0};

    my_get_str_at_pos(data, 0, ',', param_name, sizeof(param_name));

    if (CMD_EQUAL(param_name, "ADVINT"))
    {
        // TODO
        // 继续调用my_get_str_at_pos提取参数
        // my_get_str_at_pos(data, 1, ',', value_buff, sizeof(value_buff));
        // my_get_str_at_pos(data, 2, ',', value_buff, sizeof(value_buff));
    }
    else if (CMD_EQUAL(param_name, "ADVNME"))
    {
        ;
    }
    else if (CMD_EQUAL(param_name, "TAGADV"))
    {
        ;
    }
    else if (CMD_EQUAL(param_name, "CRFPWR"))
    {
        ;
    }
    else if (CMD_EQUAL(param_name, "SCANSET"))
    {
        ;
    }
    else if (CMD_EQUAL(param_name, "SCANREQ"))
    {
        ;
    }

    return 0;
}

/*
 * LTE+TIME=UTC秒数,时区(分钟)
 */
static int my_lte_handle_time(char *data)
{
    // 设置系统时间
    my_set_system_time(atoll(data));

    my_lte_send_msg("LTE+TIME=OK\r\n", strlen("LTE+TIME=OK\r\n"));

    return 0;
}

/*
 * LTE+NTCSET=<SW>,<停止充电低温阈值>,<停止充电高温阈值 >,<恢复充电低温阈值>,<恢复充电高温阈值>
 */
static int my_lte_handle_ntc_set(char *data)
{
    char value_buff[16] = {0};

    // 用value_buff依次提取各个参数
    my_get_str_at_pos(data, 0, ',', value_buff, sizeof(value_buff));

    return 0;
}

/********************************************************************
**函数名称:  netunlock_start_timer_handler
**入口参数:  timer   ---   定时器句柄 (此处未使用)
**出口参数:  无
**函数功能:  网络开锁定时器回调：执行实际的开锁动作
**           1. 状态检查：检测当前是否已处于解锁状态
**           2. 若已解锁，开启窗口定时器倒计时
**           3. 未解锁：更新全局锁状态为 UNLOCKING 并发送控制消息解锁（解锁成功才需要开启窗口定时器）
**返 回 值:  无
*********************************************************************/
void netunlock_start_timer_handler(struct k_timer *timer)
{
    ARG_UNUSED(timer);

    // 检查当前锁状态：通过开锁限位判断是否已解锁
    if (get_openlock_state())
    {
        //窗口定时器 开启标记
        g_net_unlock.netunlock_flag = 1;
        k_timer_start(&g_net_unlock.delay_timer, K_MSEC(g_net_unlock.delay_sec * 1000), K_NO_WAIT);

        return;
    }

    g_netLockState = UNLOCKING;
    //表示到时开启定时器解锁（不需要异步回复）
    g_net_unlock.start_timer_flag = 1;

    my_send_msg(MOD_MAIN, MOD_CTRL, MY_MSG_CTRL_OPENLOCKING);
}

/********************************************************************
**函数名称:  netunlock_delay_timer_handler
**入口参数:  timer   ---   定时器句柄 (此处未使用)
**出口参数:  无
**函数功能:  窗口定时到时，清空标志位（自动上锁恢复）
**返 回 值:  无
*********************************************************************/
void netunlock_delay_timer_handler(struct k_timer *timer)
{
    ARG_UNUSED(timer);

    g_net_unlock.netunlock_flag = 0;
}

static int my_lte_handle_transmit(char *data)
{
    return 0;
}

static int my_lte_handle_fota(char *data)
{
    if (strcmp(data, "ENTER") == 0)
    {
        // lte OTA升级状态设置为true, 表示正在升级中,不允许断电
        g_lte_ota_in_progress = true;

        my_lte_send_msg("LTE+FOTA=OK\r\n", strlen("LTE+FOTA=OK\r\n"));
    }
    else if (strcmp(data, "EXIT") == 0)
    {
        g_lte_ota_in_progress = false;

        my_lte_send_msg("LTE+FOTA=OK\r\n", strlen("LTE+FOTA=OK\r\n"));
    }

    return 0;
}

/********************************************************************
**函数名称:  my_lte_handle_cmd
**入口参数:  data      ---   接收4G的数据
**函数功能:  执行LTE+CMD=<号码>,<指令内容>,并根据指令内容的执行回复对应的结果给4G模块
**            <指令内容>与通过蓝牙指令下发下来的内容一致，按照相关指令格式填写即可
**          如:LTE+CMD=111,VERSION/LTE+CMD=111,VERSION#/末尾加不加#都可以
**          LTE+CMD=111,NFCTRIG,ADD,1234456789,
**          "NFCAUTH,SET,88040FBE99050B,+22277120,13516763,999900,2603200000,2603201200,1"
*********************************************************************/
int my_lte_handle_cmd(char *data)
{
    at_cmd_t at_cmd_msg = {0};
    int len = 0;
    char id[16] = {0};
    int ret = 0;

    memset(s_lte_cmd_resp_buf, 0, sizeof(s_lte_cmd_resp_buf));

    //后续有参数
    if (my_get_str_at_pos(data, 0, ',', id, sizeof(id)))
    {
        MY_LOG_INF("data: %s;len: %d", data, strlen(data));
        //指向command部分的起始位置
        strcpy(at_cmd_msg.rcv_msg, data + strlen(id) + 1); // +1跳过逗号

        //指向全局回复区域
        at_cmd_msg.resp_msg = g_resp_buf;

        //执行指令内容
        ret = run_lte_cmd(&at_cmd_msg);

        //需要异步回复，只做简单应答
        if (ret == 2)
        {
            //添加到异步回复队列
            async_add_item(at_cmd_msg.parm[0], id);
            sprintf(at_cmd_msg.resp_msg, "OK");
        }

        // 拼接回复消息(LTE+CMD=id,cmd,resp)
        snprintf(s_lte_cmd_resp_buf, LTE_CMD_RESPBUF_SIZE, "LTE+CMD=%s,%s,%s\r\n", id, at_cmd_msg.parm[0], at_cmd_msg.resp_msg);
        s_lte_cmd_resp_buf[LTE_CMD_RESPBUF_SIZE - 1] = '\0'; // 确保字符串终止
    }
    else
    {
        sprintf(s_lte_cmd_resp_buf,"LTE+CMD=%s,Missing command parameter\r\n", id);
    }

    MY_LOG_INF("lte handle cmd resp: %s", s_lte_cmd_resp_buf);

    //直接回复
    len = strlen(s_lte_cmd_resp_buf);
    my_lte_send_msg(s_lte_cmd_resp_buf, len);

    return 0;
}

/********************************************************************
**函数名称:  my_lte_handle_location
**入口参数:  data     --- 包含经纬度数据的源字符串 (<纬度>,<经度>,<GPS速度>)
**出口参数:  无
**函数功能:  处理位置信息更新请求，主要包括：
**            1. 从源字符串中分离并提取纬度，经度，GPS速度字段
**            2. 解析并校验经纬度数值的有效性 (遵循全有或全无原则)
**            3. 更新全局位置存储点 (g_location_point) 及时间戳
**            4. 构建应答消息 ("OK" 或 "FAIL") 并通过消息队列发送
**返 回 值:  0      --- 处理成功 (位置已更新并回复 OK)
            -1     --- 处理失败 (参数解析错误或校验未通过，回复 FAIL)
********************************************************************/
static int my_lte_handle_location(char *data)
{
    char lat[16] = {0};
    char lon[16] = {0};
    char speed[16] = {0};
    int32_t lat_value;
    uint8_t lat_valid;
    int32_t lon_value;
    uint8_t lon_valid;
    float speed_value;
    char *resp_data;
    char resp_str[32] = "LTE+LOCATION=FAIL\r\n"; // 默认失败
    int ret = -1;

    my_get_str_at_pos(data, 0, ',', lat, sizeof(lat));
    my_get_str_at_pos(data, 1, ',', lon, sizeof(lon));
    my_get_str_at_pos(data, 2, ',', speed, sizeof(speed));

    //经纬度参数解析和校验
    if (parse_coordinate_value(lat, 1, &lat_value, &lat_valid) != 0)
    {
        LOG_INF("invalid LAT param: %s", lat);
        goto out;
    }

    if (parse_coordinate_value(lon, 0, &lon_value, &lon_valid) != 0)
    {
        LOG_INF("invalid LON param: %s", lon);
        goto out;
    }

    if ((lon_valid == 0) || (lat_valid == 0))
    {
        LOG_INF("invalid LAT or LON param");
        goto out;
    }

    // 速度参数解析和校验
    speed_value = atof(speed);
    LOG_INF("speed: %f", speed_value);

    // 更新存储点
    g_location_point.lat = lat_value;
    g_location_point.lon = lon_value;
    g_location_point.speed = speed_value;
    g_location_point.timestamp_s = my_get_system_time_sec();

    my_send_msg(MOD_LTE, MOD_GSENSOR, MY_MSG_GPS_SPEED_UPDATE);

    strcpy(resp_str, "LTE+LOCATION=OK\r\n");
    ret = 0;

out:

    my_lte_send_msg(resp_str, strlen(resp_str));

    return ret;
}

/********************************************************************
**函数名称:  my_lte_handle_location_rsp
**入口参数:  result   --- 接收应答结构体
**出口参数:  无
**函数功能:  发送消息给MAIN线程去处理开锁相关规则：
**返 回 值:  0      --- 处理完成
*********************************************************************/
//处理获取经纬度
static int my_lte_handle_location_rsp(ble_rsp_result_t *result)
{
    msg_t msg;
    ble_rsp_result_t *result_loc;
    // 动态分配内存存储回复消息
    MY_MALLOC_BUFFER(result_loc, sizeof(ble_rsp_result_t));
    if(result_loc == NULL)
    {
        MY_LOG_ERR("result_loc malloc failed");
        return 0;
    }
    memcpy(result_loc, result, sizeof(ble_rsp_result_t));

    // 构建消息结构体并发送给MAIN模块
    msg.msgID = MY_MSG_VERIFY_UNLOCK;
    msg.pData = result_loc;
    msg.DataLen = sizeof(ble_rsp_result_t);

    my_send_msg_data(MOD_LTE, MOD_MAIN, &msg);

    return 0;
}

/********************************************************************
**函数名称:  ble_rsp_parse
**入口参数:  rsp_str     ---        输入，应答字符串 (如 "LOCATION=OK,seq,22345678,113456789#/LOCATION=OK,seq,22345678,113456789")
**           result      ---        输出，解析结果结构体
**出口参数:  result      ---        填充解析结果
**函数功能:  解析BLE应答字符串
**返 回 值:  0 表示解析成功，-1 表示解析失败
**示例:
**   输入: "LOCATION=OK,seq,N22345678,E113456789"
**   输出: type=BLE_RSP_LOCATION,cmd_name="LOCATION",
**         params="OK,seq,N22345678,E113456789", param_count=4
*********************************************************************/
int ble_rsp_parse(char *rsp_str, ble_rsp_result_t *result)
{
    char *eq_pos;
    char *param_start;
    char *p;
    int i = 0;

    if (rsp_str == NULL || result == NULL)
    {
        return -1;
    }

    memset(result, 0, sizeof(ble_rsp_result_t));

    // 查找 '='
    eq_pos = strchr(rsp_str, '=');
    if (eq_pos == NULL)
    {
        MY_LOG_INF("Invalid format(no '='): %s", rsp_str);
        return -1;
    }

    // 将 '=' 替换为\0,原始数据rsp_str会修改
    *eq_pos = '\0';
    // 提取 cmd_name
    strcpy(result->cmd_name, rsp_str);

    // 查表获取 type
    for (i = 0; ble_rsp_cmd_table[i].cmd_name != NULL; i++)
    {
        if (strcmp(result->cmd_name, ble_rsp_cmd_table[i].cmd_name) == 0)
        {
            result->type = ble_rsp_cmd_table[i].rsp_type;
            break;
        }
    }

    // 参数起始位置
    param_start = eq_pos + 1;

    // 只截断参数中的 '#'
    char *hash_pos = strchr(param_start, '#');
    if (hash_pos != NULL)
    {
        *hash_pos = '\0';
    }

    // 保存完整参数串
    strncpy(result->params, param_start, sizeof(result->params) - 1);
    result->params[sizeof(result->params) - 1] = '\0';

    // 计算参数个数
    if (*param_start == '\0')
    {
        result->param_count = 0;
    }
    else
    {
        result->param_count = 1;

        p = param_start;
        while ((p = strchr(p, ',')) != NULL)
        {
            result->param_count++;
            p++;
        }
    }

    MY_LOG_INF("BLE RSP: type=%d, cmd=%s, params=%s, count=%d",
               result->type,
               result->cmd_name,
               result->params,
               result->param_count);

    return 0;
}

/********************************************************************
**函数名称:  my_ble_handle
**入口参数:  data     --- 接收到的应答原始数据字符串指针（如 "LOCATION=OK,22345678,113456789#）
**出口参数:  无
**函数功能:  处理4G模块返回的BLE格式应答数据：
**            1. 调用ble_rsp_parse解析应答字符串，获取类型、参数等信息
**            2. 根据解析结果的类型，调用对应的处理函数（如my_lte_handle_location_rsp处理LOCATION类型的应答）
**返 回 值:  0      --- 处理完成
**            -1     --- 解析失败
*********************************************************************/
//BLE+LOCATION =OK,<维度>,<经度>
static int my_ble_handle(char *data)
{
    ble_rsp_result_t rsp_result;
    int ret = 0;
    int i = 0;
    char subcmd[16] = {0};
    char cmd_name[32];
    bool need_subcmd = false;

    //解析数据
    ret = ble_rsp_parse(data, &rsp_result);
    if (ret != 0)
    {
        MY_LOG_ERR("Failed to parse BLE response: %s", data);
        return -1;
    }

    // 查找是否有特殊指令
    for (i = 0; s_special_cmd_prefixes[i] != NULL; i++)
    {
        if (strcmp(rsp_result.cmd_name, s_special_cmd_prefixes[i]) == 0)
        {
            need_subcmd = true;
            break;
        }
    }

    if (need_subcmd)
    {
        // 取第二个参数（子命令或序号）
        my_get_str_at_pos(rsp_result.params, 1, ',', subcmd, sizeof(subcmd));
        sprintf(cmd_name, "%s_%s", rsp_result.cmd_name, subcmd);
    }
    else
    {
        strcpy(cmd_name, rsp_result.cmd_name);
    }

    //收到应答，移出重传队列
    check_ack(cmd_name);

    //处理数据
    switch (rsp_result.type)
    {
        case BLE_RSP_LOCATION:
            ret = my_lte_handle_location_rsp(&rsp_result);
            break;

        case BLE_RSP_LED:
            break;

        case BLE_RSP_TIME:
            // 提取UTC秒数（应4G网络时间同步）
            my_get_str_at_pos(rsp_result.params, 1, ',', subcmd, sizeof(subcmd));
            // 设置系统时间
            my_set_system_time(atoll(subcmd));
            break;

        //如果是tag/macinfo指令，发消息到蓝牙线性通知可以接着发下一条
        case BLE_RSP_TAG:
        case BLE_RSP_MACINFO:
            my_send_msg(MOD_LTE, MOD_BLE, MY_MSG_UPLOAD_TAG_AND_MAC);
            break;

        default:
            MY_LOG_INF("Unhandled BLE TYPE: %d", rsp_result.type);
            break;
    }
    return 0;
}

/********************************************************************
**函数名称:  my_lte_handle_factory
**入口参数:  data      ---        接收到的原始指令字符串 (如 "ENTER" 或 "EXIT")LTE+FACTORY=ENTER/EXIT
**出口参数:  无
**函数功能:  处理产测模式切换指令
**返 回 值:  0表示处理成功，-1表示内存分配失败
********************************************************************/
static int my_lte_handle_factory(char *data)
{
    char result[16] = {0};
    bool ret = false;
    char resp_buf[24] = "LTE+FACTORY=FAIL\r\n";

    ret = my_get_str_at_pos(data, 0, ',', result, sizeof(result));

    // 后续无参数
    if (!ret)
    {
        if (CMD_EQUAL(result, "ENTER"))
        {
            s_lte_factory = 1;
        }
        else if (CMD_EQUAL(result, "EXIT"))
        {
            s_lte_factory = 0;
        }
        memset(resp_buf, 0, sizeof(resp_buf));
        sprintf(resp_buf, "LTE+FACTORY=OK\r\n");
    }

    // 直接应答
    my_lte_send_msg(resp_buf, strlen(resp_buf));

    return 0;
}

/********************************************************************
**函数名称:  my_lte_handle_net
**入口参数:  data      ---        接收到的原始指令字符串 (如 "0" 或 "1")LTE+NET=0/1
**出口参数:  无
**函数功能:  处理网络状态切换指令
**返 回 值:  0表示处理成功，-1表示无效参数
********************************************************************/
static int my_lte_handle_net(char *data)
{
    char result[16] = {0};
    uint8_t net = 0;

    my_get_str_at_pos(data, 0, ',', result, sizeof(result));
    net = atoi(result);

    // 检查参数是否有效
    if (net != 0 && net != 1)
    {
        MY_LOG_ERR("Invalid network flag: %d", net);
        return -1;
    }

    // 切换网络状态
    g_lte_net_flag = net;
    LOG_INF("net: %d", net);

    // 发送应答
    my_lte_uart_send("LTE+NET=OK\r\n", strlen("LTE+NET=OK\r\n"));

    return 0;
}

/********************************************************************
**函数名称:  my_check_location_valid
**入口参数:  point       ---        存储点指针
**出口参数:  无
**函数功能:  验证经纬度存储点是否在有效期内
**           1. 检查存储点是否已初始化（经纬度是否为有效值）
**           2. 检查是否超过30分钟有效期
**返 回 值:  true 表示有效，false 表示无效或过期
*********************************************************************/
bool my_check_location_valid(location_storage_t *point)
{
    int64_t current_time;
    int64_t elapsed_time;

    // 参数检查
    if (point == NULL)
    {
        return false;
    }

    // 检查时间戳是否有效
    if (point->timestamp_s == 0)
    {
        return false;
    }

    // 获取当前时间
    current_time = my_get_system_time_sec();

    // 计算已过去的时间
    elapsed_time = current_time - point->timestamp_s;

    // 检查是否超过30分钟有效期
    if (elapsed_time < 0 || elapsed_time > LOCATION_VALIDITY_PERIOD_S)
    {
        return false;
    }

    return true;
}

/********************************************************************
**函数名称:  my_verify_nfc_permission
**入口参数:  无
**出口参数:  无
**函数功能:  执行刷卡位置校验，主要包括：
**            1. 校验当前储存点位置数据的有效性，有效就根据 NFC 卡片的权限状态执行开锁或关锁规则
**            2.储存点无效发送BLE+LOCATION=seq;seq为刷卡索引，获取经纬度信息
**返 回 值:  无
********************************************************************/
void my_verify_nfc_permission(void)
{
    char card_index[10]={0};
    //先验证存储点是否有效
    if (my_check_location_valid(&g_location_point))
    {
        //再验证是否在电子围栏范围内
        if (is_point_in_circle(g_location_point.lat, g_location_point.lon,
            gConfigParam.nfcauth_config.nfcauth_cards[g_nfc_card_index].lat,
            gConfigParam.nfcauth_config.nfcauth_cards[g_nfc_card_index].lon,
            gConfigParam.nfcauth_config.nfcauth_cards[g_nfc_card_index].radius))
        {
            if (gConfigParam.nfcauth_config.nfcauth_cards[g_nfc_card_index].unlock_times != 0 && get_openlock_state() == false)
            {
                if (gConfigParam.nfcauth_config.nfcauth_cards[g_nfc_card_index].unlock_times > 0)
                {
                    gConfigParam.nfcauth_config.nfcauth_cards[g_nfc_card_index].unlock_times--;
                    my_user_data_write(ZMS_ID_NFCAUTH_CONFIG, &gConfigParam.nfcauth_config, sizeof(nfcauth_config_t));
                }
                my_send_msg(MOD_CTRL, MOD_CTRL, MY_MSG_CTRL_OPENLOCKING);
                MY_LOG_INF("start to openlock");
            }
            else if (gConfigParam.nfcauth_config.nfcauth_cards[g_nfc_card_index].lock_times != 0 && get_closelock_state() == false)
            {
                if (gConfigParam.nfcauth_config.nfcauth_cards[g_nfc_card_index].lock_times > 0)
                {
                    gConfigParam.nfcauth_config.nfcauth_cards[g_nfc_card_index].lock_times--;
                    my_user_data_write(ZMS_ID_NFCAUTH_CONFIG, &gConfigParam.nfcauth_config, sizeof(nfcauth_config_t));
                }
                my_send_msg(MOD_CTRL, MOD_CTRL, MY_MSG_CTRL_CLOSELOCKING);
                MY_LOG_INF("start to closelock");
            }
        }
        else
        {
            LOG_INF("device is out of allowed area");
        }

        g_last_card_index = -1;
    }
    else
    {
        // 通过发消息通知4G需要获取经纬度信息
        sprintf(card_index, "%d", g_nfc_card_index);
        #if RETRANSMIT_CHECK_ENABLED
            lte_send_cmd_with_retry("LOCATION", card_index);
        #else
            lte_send_command("LOCATION", card_index);
        #endif
    }
    return;
}

/*
 * 处理各个协议指令
 * cmd为已经拆分好的单条指令
 */
int my_lte_parse_cmd(char *cmd, int cmd_len)
{
    char *argv[MAX_ARGS];
    int ret = 0;
    int i;
    char *p = cmd;
    int argc, num_commands;
    msg_t msg;
    char *lte_cmd;
    char *ble_cmd;

    if (0 == strlen(cmd) || 0 == cmd_len)
    {
        return -1;
    }

    MY_LOG_INF("%s: %s", __func__, cmd);

    // 按使用频次由高到低排序?
    if (CMD_MATCHED(cmd, LTE_PWRON))
    {
        ret = my_lte_handle_power_on(p + strlen(LTE_PWRON));
        goto END;
    }
    else if (CMD_MATCHED(cmd, LTE_BTSET))
    {
        ret = my_lte_handle_bt_set(p + strlen(LTE_BTSET));
        goto END;
    }
    else if (CMD_MATCHED(cmd, LTE_TIME))
    {
        ret = my_lte_handle_time(p + strlen(LTE_TIME));
        goto END;
    }
    else if (CMD_MATCHED(cmd, LTE_NTCSET))
    {
        ret = my_lte_handle_ntc_set(p + strlen(LTE_NTCSET));
        goto END;
    }
    else if (CMD_MATCHED(cmd, LTE_TRANSMIT))
    {
        ret = my_lte_handle_transmit(p + strlen(LTE_TRANSMIT));
        goto END;
    }
    else if (CMD_MATCHED(cmd, LTE_FOTA))
    {
        ret = my_lte_handle_fota(p + strlen(LTE_FOTA));
        goto END;
    }
    else if (CMD_MATCHED(cmd, LTE_CMD))
    {
        if (CMD_MATCHED(cmd + strlen(LTE_CMD), FACTORY_CMD_HEADER))
        {
            my_at_factory_cmd(cmd + strlen(LTE_CMD));
            return -1;
        }

        MY_MALLOC_BUFFER(lte_cmd, strlen(cmd) + 1 - strlen(LTE_CMD));
        if (lte_cmd == NULL)
        {
            MY_LOG_ERR("lte_cmd malloc failed");
            return 0;
        }

        strcpy(lte_cmd, cmd + strlen(LTE_CMD));

        // 将数据透传指令放到与蓝牙同线程
        msg.msgID = MY_MSG_LTE_CMD_RX;
        msg.pData = lte_cmd;
        my_send_msg_data(MOD_LTE, MOD_BLE, &msg);

        goto END;
    }
    else if (CMD_MATCHED(cmd, LTE_LOCATION))
    {
        ret = my_lte_handle_location(p + strlen(LTE_LOCATION));
        goto END;
    }
    else if (CMD_MATCHED(cmd, LTE_FACTORY))
    {
        ret = my_lte_handle_factory(p + strlen(LTE_FACTORY));
        goto END;
    }
    else if (CMD_MATCHED(cmd, LTE_NET))
    {
        ret = my_lte_handle_net(p + strlen(LTE_NET));
        goto END;
    }
    else if (CMD_MATCHED(cmd, BLE_CMD))
    {
        MY_MALLOC_BUFFER(ble_cmd, strlen(cmd) + 1 - strlen(BLE_CMD));
        if (ble_cmd == NULL)
        {
            MY_LOG_ERR("ble_cmd malloc failed");
            return 0;
        }

        strcpy(ble_cmd, cmd + strlen(BLE_CMD));

        msg.msgID = MY_MSG_BLE_CMD;
        msg.pData = ble_cmd;
        msg.DataLen = strlen(ble_cmd);
        my_send_msg_data(MOD_LTE, MOD_BLE, &msg);
        goto END;
    }
    //处理4G应答
    else if (CMD_MATCHED(cmd, BLE))
    {
        ret = my_ble_handle(p + strlen(BLE));
        goto END;
    }

    // 检查是否是产测指令
    if (strncmp(cmd, FACTORY_CMD_HEADER, strlen(FACTORY_CMD_HEADER)) == 0)
    {
        my_at_factory_cmd(cmd);

        return -1;
    }

    ParseArgs(cmd, &argc, argv);

    /* only whitespace */
    if (argc == 0) {
        return -1;
    }

    num_commands = GetCmdMatche(argv[0]);
    if (num_commands < 0) {
        MY_LOG_INF("No '%s' command", argv[0]);
        return -1;
    }

    if (AT_CMD_INNER[num_commands].proc != NULL) {
        AT_CMD_INNER[num_commands].proc(argc, argv);
    }
END:
    return ret;
}

void my_lte_handle_recv(uint8_t *pData, uint32_t iLen)
{
    static char command[MAX_CMD_LEN] = {0};
    static uint32_t index = 0;
    uint32_t i;

    for (i = 0; i < iLen; i++)
    {
        if (pData[i] == '\r' || pData[i] == '\n') // 回车是\r 为了兼容同时处理 \n
        {
            my_lte_parse_cmd(command, index);

            command[0] = 0;
            index = 0;

            // 如果下个字符是\n，跳过
            if (pData[i + 1] == '\n')
            {
                i++;
            }
        }
        else if (index < (MAX_CMD_LEN - 1))
        {
            command[index++] = pData[i];
            command[index] = '\0';
        }
    }
}

/********************************************************************
**函数名称:  my_lte_init
**入口参数:  tid      ---        线程ID
**出口参数:  无
**函数功能:  LTE 模块初始化函数，配置 UART 与 GPIO，启动线程
**返 回 值:  无
*********************************************************************/
int my_lte_init(k_tid_t *tid)
{
    int err;

    /* 检查硬件设备是否就绪 */
    if (!device_is_ready(lte_uart_dev))
    {
        MY_LOG_ERR("LTE UART device not ready");
        return -ENODEV;
    }

    if (!gpio_is_ready_dt(&lte_pwr_gpio))
    {
        MY_LOG_ERR("LTE Power GPIO not ready");
        return -ENODEV;
    }

    if (!gpio_is_ready_dt(&lte_wake_gpio))
    {
        MY_LOG_ERR("LTE Wake GPIO not ready");
        return -ENODEV;
    }

    /* 配置电源控制引脚为输出，默认低电平（不使能） */
    err = gpio_pin_configure_dt(&lte_pwr_gpio, GPIO_OUTPUT_INACTIVE);
    if (err)
    {
        MY_LOG_ERR("Failed to configure LTE Power GPIO (err %d)", err);
        return err;
    }

    // 配置唤醒引脚(P0.04)为输入，外部上拉，下降沿中断
    err = gpio_pin_configure_dt(&lte_wake_gpio, GPIO_INPUT);
    if (err)
    {
        MY_LOG_ERR("Failed to configure LTE Wake GPIO (err %d)", err);
        return err;
    }

    err = gpio_pin_interrupt_configure_dt(&lte_wake_gpio, GPIO_INT_EDGE_TO_ACTIVE);
    if (err)
    {
        MY_LOG_ERR("Failed to configure LTE Wake interrupt (err %d)", err);
        return err;
    }

    gpio_init_callback(&lte_wake_cb, lte_wake_pin_isr, BIT(lte_wake_gpio.pin));
    err = gpio_add_callback(lte_wake_gpio.port, &lte_wake_cb);
    if (err)
    {
        MY_LOG_ERR("Failed to add LTE Wake callback (err %d)", err);
        return err;
    }

    // 初始化串口接收循环缓冲区
    my_rb_init(&s_lte_rb, s_lte_rb_buf, LTE_UART_RB_SIZE);

    // 初始值为1(表示UART空闲)
    k_sem_init(&s_TxDoneSem, 1, 1);

    /* 设置 UART 异步回调 */
    err = uart_callback_set(lte_uart_dev, lte_uart_cb, NULL);
    if (err)
    {
        MY_LOG_ERR("Failed to set LTE UART callback (err %d)", err);
        return err;
    }

    // 初始化 LTE 到统一 PM 框架，默认保持 UART 挂起态
    err = my_pm_device_register(MY_PM_DEV_LTE, &lte_pm_ops);
    if (err < 0)
    {
        MY_LOG_ERR("LTE PM registration failed: %d", err);
        return err;
    }

    /* 初始化消息队列 */
    my_init_msg_handler(MOD_LTE, &my_lte_msgq);

    /* 初始化LTE缓存消息队列, 用于存储BLE指令数据 */
    my_lte_msg_queue_init();

    /* 启动 LTE 线程 */
    *tid = k_thread_create(&s_my_lte_task_data, my_lte_task_stack,
                           K_THREAD_STACK_SIZEOF(my_lte_task_stack),
                           my_lte_task, NULL, NULL, NULL,
                           MY_LTE_TASK_PRIORITY, 0, K_NO_WAIT);

    /* 设置线程名称 */
    k_thread_name_set(*tid, "MY_LTE");

    MY_LOG_INF("LTE module initialized successfully (Loopback mode)");

#if RETRANSMIT_CHECK_ENABLED
    //初始化队列
    init_retransmission_queue();
#endif

    //重传检查定时器
    k_timer_init(&s_retrans_check_timer, retrans_check_timer_handler, NULL);

    // 清空异步回复队列
    init_async_queue();

    /* 初始化完成后默认开启模块电源 */
    my_lte_pwr_on(true);

    // 网络解锁相关定时器
    k_timer_init(&g_net_unlock.start_timer, netunlock_start_timer_handler, NULL);
    k_timer_init(&g_net_unlock.delay_timer, netunlock_delay_timer_handler, NULL);

    return 0;
}
