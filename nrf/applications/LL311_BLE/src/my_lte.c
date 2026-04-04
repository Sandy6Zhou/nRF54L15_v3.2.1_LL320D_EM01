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

// 串口协议报文头定义清单
char LTE_PWRON[] = "LTE+PWRON=";
char LTE_BTSET[] = "LTE+BTSET=";
char LTE_NTCSET[] = "LTE+NTCSET=";
char LTE_TIME[] = "LTE+TIME=";
char LTE_NFCAUTH[] = "LTE+NFCAUTH=";
char LTE_NFCTRIG[] = "LTE+NFCTRIG=";
char LTE_TRANSMIT[] = "LTE+TRANSMIT=";
char LTE_LOCK[] = "LTE+LOCK=";
char LTE_BUZZER[] = "LTE+BUZZER=";
char LTE_LED[] = "LTE+LED=";
char LTE_FOTA[] = "LTE+FOTA=";
char LTE_STATE[] = "LTE+STATE=";

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

// 定义一个串口发送状态信号量，初始值为1(表示UART空闲)
static struct k_sem s_TxDoneSem;
/* LTE缓存消息队列 */
static lte_msg_queue_t g_lte_msg_queue = {0};

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
**函数功能:  LTE 模块发送数据函数
**返 回 值:  无
*********************************************************************/
int my_lte_uart_send(const uint8_t *data, uint16_t len)
{
#if 0
    if (len == 0 || data == NULL)
    {
        return -EINVAL;
    }
#endif

    if (!g_bLteReady) return -1;

    // 等待上一次传输完成,等待时间基本忽略不计
    k_sem_take(&s_TxDoneSem, K_FOREVER);

    return uart_tx(lte_uart_dev, data, len, SYS_FOREVER_MS);
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

/*
 * 4G模块上电后发送的第一条指令
 * BLE收到此条报文后才能给4G模块发送指令
 *
 * LTE+PWRON=<上电原因>,<固件版本号>
 */
static int my_lte_handle_power_on(char *data)
{
    g_bLteReady = 1;

    // TODO 4G完成开机逻辑处理
    // 处理任何排队的消息
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

/*
LTE+NFCAUTH=SET,<NFC.NO>,<OP>,<LAT>,<LON>,<半径>,<startTime>,<endTime>,<Unlock Times>
LTE+NFCAUTH=PSET,<NFC.NO>
LTE+NFCAUTH=DEL,ALL
LTE+NFCAUTH=DEL,<NFC.NO>
LTE+NFCAUTH=CHECK,<NFC.NO>
*/
static int my_lte_handle_nfc_auth(char *data)
{
    char cmd_name[16] = {0};
    char val_buff[16] = {0};

    my_get_str_at_pos(data, 0, ',', cmd_name, sizeof(cmd_name));

    if (CMD_EQUAL(cmd_name, "SET"))
    {
    }
    else if (CMD_EQUAL(cmd_name, "PSET"))
    {
    }
    else if (CMD_EQUAL(cmd_name, "DEL"))
    {
    }
    else if (CMD_EQUAL(cmd_name, "CHECK"))
    {
    }

    return 0;
}

/*
 * LTE+NFCTRIG=<联动卡号>,<Command>
 */
static int my_lte_handle_nfc_trig(char *data)
{
    return 0;
}

/*
0: 开锁
LTE+LOCK=0,<Time>,<Delay>

1: 上锁
LTE+LOCK=1,<Time>,<Delay> // Time, Delay 可选

DT: 设置延迟上锁时间
LTE+LOCK=DT,<延迟时间>
*/
static int my_lte_handle_lock(char *data)
{
    char cmd_name[16] = {0};
    char val_buff[16] = {0};

    my_get_str_at_pos(data, 0, ',', cmd_name, sizeof(cmd_name));

    // 开锁
    if (CMD_EQUAL(cmd_name, "0"))
    {
    }
    // 上锁
    else if (CMD_EQUAL(cmd_name, "1"))
    {
    }
    // 自动上锁延迟时间
    else if (CMD_EQUAL(cmd_name, "DT"))
    {
    }

    return 0;
}

static int my_lte_handle_transmit(char *data)
{
    return 0;
}

static int my_lte_handle_buzzer(char *data)
{
    return 0;
}

static int my_lte_handle_led(char *data)
{
    return 0;
}

static int my_lte_handle_fota(char *data)
{
    return 0;
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
    else if (CMD_MATCHED(cmd, LTE_NFCAUTH))
    {
        ret = my_lte_handle_nfc_auth(p + strlen(LTE_NFCAUTH));
        goto END;
    }
    else if (CMD_MATCHED(cmd, LTE_NFCTRIG))
    {
        ret = my_lte_handle_nfc_trig(p + strlen(LTE_NFCTRIG));
        goto END;
    }
    else if (CMD_MATCHED(cmd, LTE_LOCK))
    {
        ret = my_lte_handle_lock(p + strlen(LTE_LOCK));
        goto END;
    }
    else if (CMD_MATCHED(cmd, LTE_TRANSMIT))
    {
        ret = my_lte_handle_transmit(p + strlen(LTE_TRANSMIT));
        goto END;
    }
    else if (CMD_MATCHED(cmd, LTE_BUZZER))
    {
        ret = my_lte_handle_buzzer(p + strlen(LTE_BUZZER));
        goto END;
    }
    else if (CMD_MATCHED(cmd, LTE_LED))
    {
        ret = my_lte_handle_led(p + strlen(LTE_LED));
        goto END;
    }
    else if (CMD_MATCHED(cmd, LTE_FOTA))
    {
        ret = my_lte_handle_fota(p + strlen(LTE_FOTA));
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

    return 0;
}
