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

// 串口协议报文头定义清单
char LTE_PWRON[] = "LTE+PWRON=";
char LTE_BTSET[] = "LTE+BTSET=";
char LTE_NTCSET[] = "LTE+NTCSET=";
char LTE_TIME[] = "LTE+TIME=";
char LTE_TRANSMIT[] = "LTE+TRANSMIT=";
char LTE_LOCK[] = "LTE+LOCK=";
char LTE_FOTA[] = "LTE+FOTA=";
char LTE_STATE[] = "LTE+STATE=";
char LTE_CMD[] = "LTE+CMD=";
char LTE_LOCATION[] = "LTE+LOCATION=";
char BLE[] = "BLE+";

// 经纬度存储点
location_storage_t g_location_point = {0};

// 命令映射表定义
static const ble_rsp_cmd_map_t ble_rsp_cmd_table[] = {
    {"LOCATION", BLE_RSP_LOCATION},
    {"LED",      BLE_RSP_LED     },
    {NULL,       BLE_RSP_UNKNOWN }
};

/* LTE电源状态跟踪 */
static bool g_lte_power_state = false;  // false=关闭, true=开启

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

/* 接收LTE+CMD响应回复缓冲区大小 */
#define LTE_CMD_RESPBUF_SIZE 1024
char g_lte_cmd_resp_buf[LTE_CMD_RESPBUF_SIZE] = {0};

// 定义一个串口发送状态信号量，初始值为1(表示UART空闲)
static struct k_sem s_TxDoneSem;
/* LTE缓存消息队列 */
static lte_msg_queue_t g_lte_msg_queue = {0};

// 4G上电状态，0：蓝牙正常唤醒4G，1：异常重启
lte_power_state_t g_4GPoweronStatus = 0;
// 4G版本号缓存
char g_lte4GVersion[32] = {0};
// LTE开机原因
lte_boot_reason_t g_lteBootReason = LTE_BOOT_REASON_RESERVED;

/* UART驱动层使用的接收双缓冲 */
static uint8_t lte_rx_buf_1[LTE_UART_BUF_SIZE];
static uint8_t lte_rx_buf_2[LTE_UART_BUF_SIZE];
static uint8_t *lte_next_buf = lte_rx_buf_2;

// 串口接收循环缓冲区（建议用2的幂，如1024，取模效率更高）
#define LTE_UART_RB_SIZE    512
static uint8_t g_lte_rb_buf[LTE_UART_RB_SIZE];
static ring_buffer_t g_lte_rb;

/* 消息队列定义 */
K_MSGQ_DEFINE(my_lte_msgq, sizeof(MSG_S), 10, 4);

/* 线程数据与栈定义 */
K_THREAD_STACK_DEFINE(my_lte_task_stack, MY_LTE_TASK_STACK_SIZE);
static struct k_thread my_lte_task_data;

// 产测指令
const char FACTORY_CMD_HEADER[] = "AT^GT_CM=";

/* { visible, command, help, function } */
CMD_STRUC AT_CMD_INNER[] = {

    {1, "TEST",      "AT CMD TEST",             my_at_test},

    {0, NULL,        NULL,                      NULL}
};

// 网络解锁全局变量
net_unlock_ctrl_t g_net_unlock;

// lte+cmd异步回复需要id号
char g_lte_cmd_id[16];

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
    k_mutex_init(&g_lte_msg_queue.queue_mutex);

    g_lte_msg_queue.head = 0;
    g_lte_msg_queue.tail = 0;
    g_lte_msg_queue.count = 0;

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

    k_mutex_lock(&g_lte_msg_queue.queue_mutex, K_FOREVER);

    // 如果队列已满，移除最旧的消息（head位置）
    if (g_lte_msg_queue.count >= LTE_MSG_QUEUE_SIZE)
    {
        // 释放最旧消息的内存
        if (g_lte_msg_queue.queue[g_lte_msg_queue.head].msg_content != NULL)
        {
            MY_LOG_INF("Release old message: %s", g_lte_msg_queue.queue[g_lte_msg_queue.head].msg_content);
            MY_FREE_BUFFER(g_lte_msg_queue.queue[g_lte_msg_queue.head].msg_content);
            g_lte_msg_queue.queue[g_lte_msg_queue.head].msg_content = NULL;
        }

        // 移动head指针，移除最旧元素
        g_lte_msg_queue.head = (g_lte_msg_queue.head + 1) % LTE_MSG_QUEUE_SIZE;
        g_lte_msg_queue.count--;
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
    g_lte_msg_queue.queue[g_lte_msg_queue.tail].msg_content = new_msg;
    g_lte_msg_queue.queue[g_lte_msg_queue.tail].msg_len = msg_len;

    // MY_LOG_INF("Enqueue message: %s", g_lte_msg_queue.queue[g_lte_msg_queue.tail].msg_content);

    // 更新队列指针
    g_lte_msg_queue.tail = (g_lte_msg_queue.tail + 1) % LTE_MSG_QUEUE_SIZE;
    g_lte_msg_queue.count++;

exit:
    k_mutex_unlock(&g_lte_msg_queue.queue_mutex);
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

    k_mutex_lock(&g_lte_msg_queue.queue_mutex, K_FOREVER);

    // 遍历并发送所有排队的消息
    while (g_lte_msg_queue.count > 0)
    {
        pending_msg = &g_lte_msg_queue.queue[g_lte_msg_queue.head];

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
        g_lte_msg_queue.head = (g_lte_msg_queue.head + 1) % LTE_MSG_QUEUE_SIZE;
        g_lte_msg_queue.count--;
    }

    k_mutex_unlock(&g_lte_msg_queue.queue_mutex);
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
    g_lteBootReason = reason;
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
    return g_lteBootReason;
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

    MSG_S msg;

    MY_LOG_INF("LTE thread started");

    for (;;)
    {
        my_recv_msg(&my_lte_msgq, (void *)&msg, sizeof(MSG_S), K_FOREVER);

        switch (msg.msgID)
        {
            /* TODO: 添加 LTE 相关的消息处理逻辑 */
            case MY_MSG_LTE_PWRON:
                my_lte_pwr_on(true);
                break;

            case MY_MSG_LTE_PWROFF:
                my_lte_pwr_on(false);
                g_bLteReady = 0;
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
                    len = my_rb_read(&g_lte_rb, read_buf, sizeof(read_buf));
                    if (len > 0)
                    {
                        my_lte_handle_recv(read_buf, len);
                    }
                    else
                    {
                        break;
                    }
                }
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
            my_rb_write(&g_lte_rb, &evt->data.rx.buf[evt->data.rx.offset], evt->data.rx.len);
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
            /* 如果接收被禁用，尝试重新开启 */
            uart_rx_enable(dev, lte_rx_buf_1, LTE_UART_BUF_SIZE, 10 * USEC_PER_MSEC);
            break;

        case UART_TX_ABORTED:
            MY_LOG_WRN("LTE UART TX Aborted");
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

    // 等待上一次传输完成
    k_sem_take(&s_TxDoneSem, K_FOREVER);

    // 发送唤醒字节：如果4G在休眠状态，这个字节会将其唤醒
    uart_tx(lte_uart_dev, wake_byte, 3, SYS_FOREVER_MS);

    // 等待上一次传输完成
    k_sem_take(&s_TxDoneSem, K_FOREVER);

    // TODO 唤醒时间仅需几百微秒，几乎不影响后续数据传输，待实际测试验证，暂定等待1ms
    k_sleep(K_MSEC(1));
    // ! uart_tx发送是异步处理,传进去的data需要是静态的才能保证数据不丢失或者执行完uart_tx延时一会确保数据传输完.
    memcpy(s_sendDataBuf, data, len);

    // 发送实际数据，此时4G模块已经处于唤醒状态，可以正常接收数据
    return uart_tx(lte_uart_dev, s_sendDataBuf, len, SYS_FOREVER_MS);
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
    return g_lte_power_state;
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
    if (g_lte_power_state == on)
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
        g_lte_power_state = on;
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
    const lic_ff_struct *lic_ff;
    const lic_gg_struct *lic_gg;
    uint16_t ECDH_GValue;
    uint8_t data_buff[64] = {0};
    const GsmImei_t *gsmImei;
    const uint8_t *edr_addr;
    int ret;

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

    my_parse_cmd_line(pfactorycmd + strlen(FACTORY_CMD_HEADER), ',' , &argc, argv);

    MY_LOG_INF("%s", my_handle_at_factory_cmd(argv, argc));

    return 0;
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
    time_t utc_sec;

    // 解析4G发来的参数: <上电状态>,<上电原因>,<4G版本号>
    my_get_str_at_pos(data, 0, ',', power_state_str, sizeof(power_state_str));
    my_get_str_at_pos(data, 1, ',', power_reason_str, sizeof(power_reason_str));
    my_get_str_at_pos(data, 2, ',', g_lte4GVersion, sizeof(g_lte4GVersion));

    g_4GPoweronStatus = atoi(power_state_str);

    MY_LOG_INF("LTE PWRON state=%s reason=%s ver=%s", power_state_str, power_reason_str, g_lte4GVersion);

    // 4G模块已就绪，允许后续数据收发
    g_bLteReady = 1;

    // 构造应答报文: LTE+PWRON=OK,<开机原因>,<蓝牙版本号>,<UTC>
    if (g_4GPoweronStatus == LTE_PWR_STATE_ABNORMAL)
    {
        // 异常重启时，开机原因默认填255
        boot_reason = LTE_BOOT_REASON_RESERVED;
    }
    else
    {
        boot_reason = get_lte_boot_reason();
    }

    utc_sec = my_get_system_time_sec();
    nRespLen = snprintf(resp_buf, sizeof(resp_buf), "%sOK,%d,%s,%lld\r\n",
                        LTE_PWRON, (int)boot_reason, SOFTWARE_VERSION, (long long)utc_sec);

    my_lte_send_msg(resp_buf, (uint16_t)nRespLen);

    // 处理排队的消息
    my_lte_process_queued_msgs();

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
    char utc_seconds[16] = {0};
    char zone_in_min[16] = {0};

    my_get_str_at_pos(data, 0, ',', utc_seconds, sizeof(utc_seconds));
    my_get_str_at_pos(data, 1, ',', zone_in_min, sizeof(zone_in_min));

    // 设置系统时间
    my_set_system_time(atoll(utc_seconds));

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

    at_cmd_struc at_cmd_msg = {0};
    int len = 0;
    char id[16] = {0};
    MSG_S msg;
    char *resp_msg;
    int ret = 0;

    memset(g_lte_cmd_resp_buf, 0, sizeof(g_lte_cmd_resp_buf));

    //后续有参数
    if (my_get_str_at_pos(data, 0, ',', id, sizeof(id)))
    {
        MY_LOG_INF("data: %s;len: %d", data, strlen(data));
        //指向command部分的起始位置
        strcpy(at_cmd_msg.rcv_msg, data + strlen(id) + 1); // +1跳过逗号

        //执行指令内容
        ret = run_lte_cmd(&at_cmd_msg);

        //需要异步回复，只做简单应答
        if (ret == 2)
        {
            //TODO 可能存在连续2条需要异步回复指令过来，后续需要修改做匹配和回复内容需增加指令头
            strcpy(g_lte_cmd_id, id);

            sprintf(at_cmd_msg.resp_msg, "OK");
        }

        // 拼接回复消息
        snprintf(g_lte_cmd_resp_buf, LTE_CMD_RESPBUF_SIZE, "LTE+CMD=%s,%s\r\n", id, at_cmd_msg.resp_msg);
        g_lte_cmd_resp_buf[LTE_CMD_RESPBUF_SIZE - 1] = '\0'; // 确保字符串终止
    }
    else
    {
        sprintf(g_lte_cmd_resp_buf,"LTE+CMD=%s,Missing command parameter\r\n", id);
    }

    MY_LOG_INF("lte handle cmd resp: %s", g_lte_cmd_resp_buf);

    // 动态分配内存存储回复消息
    len = strlen(g_lte_cmd_resp_buf);
    MY_MALLOC_BUFFER(resp_msg, len + 1);
    if(resp_msg == NULL)
    {
        MY_LOG_ERR("resp_msg malloc failed");
        return 0;
    }

    memcpy(resp_msg, g_lte_cmd_resp_buf, len);
    resp_msg[len] = '\0';  // 确保字符串终止

    // 构建消息结构体并发送给LTE模块
    msg.msgID = MY_MSG_LTE_BLE_DATA;
    msg.pData = resp_msg;
    msg.DataLen = len;
    my_send_msg_data(MOD_LTE,MOD_LTE, &msg);

    return 0;
}

/********************************************************************
**函数名称:  my_lte_handle_location
**入口参数:  data     --- 包含经纬度数据的源字符串 (<纬度>,<经度>)
**出口参数:  无
**函数功能:  处理位置信息更新请求，主要包括：
**            1. 从源字符串中分离并提取纬度和经度字段
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
    int32_t lat_value;
    uint8_t lat_valid;
    int32_t lon_value;
    uint8_t lon_valid;
    char *resp_data;
    char resp_str[32] = "LTE+LOCATION=FAIL\r\n"; // 默认失败
    int ret = -1;
    int len;
    MSG_S msg;

    my_get_str_at_pos(data, 0, ',', lat, sizeof(lat));
    my_get_str_at_pos(data, 1, ',', lon, sizeof(lon));

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

    // 更新存储点
    g_location_point.lat = lat_value;
    g_location_point.lon = lon_value;
    g_location_point.timestamp_s = my_get_system_time_sec();

    strcpy(resp_str, "LTE+LOCATION=OK\r\n");
    ret = 0;

out:
    // 统一回复逻辑
    MY_MALLOC_BUFFER(resp_data, strlen(resp_str) + 1);
    if (resp_data == NULL)
    {
        MY_LOG_ERR("resp_data malloc failed");
        ret = -1;
        return ret;
    }

    strcpy(resp_data, resp_str);
    len = strlen(resp_data);

    // 构建消息结构体并发送给LTE模块
    msg.msgID = MY_MSG_LTE_BLE_DATA;
    msg.pData = resp_data;
    msg.DataLen = len;

    my_send_msg_data(MOD_LTE, MOD_LTE, &msg);

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
    MSG_S msg;
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

    //解析数据
    ret = ble_rsp_parse(data, &rsp_result);
    if (ret != 0)
    {
        MY_LOG_ERR("Failed to parse BLE response: %s", data);
        return -1;
    }

    switch (rsp_result.type)
    {
        case BLE_RSP_LOCATION:
            ret = my_lte_handle_location_rsp(&rsp_result);
            break;

        case BLE_RSP_LED:
            break;

        default:
            MY_LOG_INF("Unhandled BLE TYPE: %d", rsp_result.type);
            break;
    }
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
**函数名称:  my_verify_openlock
**入口参数:  无
**出口参数:  无
**函数功能:  执行刷卡位置校验，主要包括：
**            1. 校验当前储存点位置数据的有效性，有效就执行开锁规则
**            2.储存点无效发送BLE+LOCATION=seq;seq为刷卡索引，获取经纬度信息
**返 回 值:  无
********************************************************************/
void my_verify_openlock(void)
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
            // 若卡的次数有限,需要消耗次数(-1为无限次数)
            if (gConfigParam.nfcauth_config.nfcauth_cards[g_nfc_card_index].unlock_times > 0)
            {
                gConfigParam.nfcauth_config.nfcauth_cards[g_nfc_card_index].unlock_times--;
            }
            // 启动开锁操作
            my_send_msg(MOD_CTRL, MOD_CTRL, MY_MSG_CTRL_OPENLOCKING);
            MY_LOG_INF("start to openlock");
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
        lte_send_command("LOCATION", card_index);
    }
    return;
}

/*
 * 处理各个协议指令
 * cmd为已经拆分好的单条指令
 */
int my_lte_parse_cmd(char *cmd, int cmd_len)
{
    int ret = 0;
    char *p = cmd;
    int argc, num_commands;
    char *argv[MAX_ARGS];
    MSG_S msg;
    char *lte_cmd;

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

    /* 配置电源控制引脚为输出，默认低电平（不使能） */
    err = gpio_pin_configure_dt(&lte_pwr_gpio, GPIO_OUTPUT_INACTIVE);
    if (err)
    {
        MY_LOG_ERR("Failed to configure LTE Power GPIO (err %d)", err);
        return err;
    }

    // 初始化串口接收循环缓冲区
    my_rb_init(&g_lte_rb, g_lte_rb_buf, LTE_UART_RB_SIZE);

    // 初始值为1(表示UART空闲)
    k_sem_init(&s_TxDoneSem, 1, 1);

    /* 设置 UART 异步回调 */
    err = uart_callback_set(lte_uart_dev, lte_uart_cb, NULL);
    if (err)
    {
        MY_LOG_ERR("Failed to set LTE UART callback (err %d)", err);
        return err;
    }

    /* 开启 UART 接收 */
    err = uart_rx_enable(lte_uart_dev, lte_rx_buf_1, LTE_UART_BUF_SIZE, 10 * USEC_PER_MSEC);
    if (err)
    {
        MY_LOG_ERR("Failed to enable LTE UART RX (err %d)", err);
        return err;
    }

    /* 初始化消息队列 */
    my_init_msg_handler(MOD_LTE, &my_lte_msgq);

    /* 初始化LTE缓存消息队列, 用于存储BLE指令数据 */
    my_lte_msg_queue_init();

    /* 启动 LTE 线程 */
    *tid = k_thread_create(&my_lte_task_data, my_lte_task_stack,
                           K_THREAD_STACK_SIZEOF(my_lte_task_stack),
                           my_lte_task, NULL, NULL, NULL,
                           MY_LTE_TASK_PRIORITY, 0, K_NO_WAIT);

    /* 设置线程名称 */
    k_thread_name_set(*tid, "MY_LTE");

    MY_LOG_INF("LTE module initialized successfully (Loopback mode)");

    /* 初始化完成后默认开启模块电源 */
    my_lte_pwr_on(true);

    // 网络解锁相关定时器
    k_timer_init(&g_net_unlock.start_timer, netunlock_start_timer_handler, NULL);
    k_timer_init(&g_net_unlock.delay_timer, netunlock_delay_timer_handler, NULL);

    return 0;
}
