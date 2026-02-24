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

#include "my_comm.h"

/* LTE电源状态跟踪 */
static bool g_lte_power_state = false;  // false=关闭, true=开启

/* 注册 LTE 模块日志 */
LOG_MODULE_REGISTER(my_lte, LOG_LEVEL_INF);

/* 从设备树获取 UART 与 GPIO 配置 */
#define LTE_UART_NODE DT_ALIAS(lte_uart)
static const struct device *lte_uart_dev = DEVICE_DT_GET(LTE_UART_NODE);

#define LTE_PWR_CTRL_NODE DT_ALIAS(lte_pwr_ctrl)
static const struct gpio_dt_spec lte_pwr_gpio = GPIO_DT_SPEC_GET(LTE_PWR_CTRL_NODE, gpios);

/* UART 接收双缓冲 */
static uint8_t lte_rx_buf_1[LTE_UART_BUF_SIZE];
static uint8_t lte_rx_buf_2[LTE_UART_BUF_SIZE];
static uint8_t *lte_next_buf = lte_rx_buf_2;

/* 消息队列定义 */
K_MSGQ_DEFINE(my_lte_msgq, sizeof(MSG_S), 10, 4);

/* 线程数据与栈定义 */
K_THREAD_STACK_DEFINE(my_lte_task_stack, MY_LTE_TASK_STACK_SIZE);
static struct k_thread my_lte_task_data;

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

    LOG_INF("LTE thread started");

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
                break;

            // 收到4G发送的消息,例如返回UTC时间,在里面进行数据解析
            case MY_MSG_LTE_REV:
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
            LOG_DBG("LTE UART TX Done");
            break;

        case UART_RX_RDY:
            LOG_DBG("LTE UART RX Ready, len: %d", evt->data.rx.len);
            /* 串口回环测试：将收到的数据直接原样发回 */
            my_lte_send(&evt->data.rx.buf[evt->data.rx.offset], evt->data.rx.len);
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
            LOG_WRN("LTE UART TX Aborted");
            break;

        default:
            break;
    }
}

/********************************************************************
**函数名称:  my_lte_send
**入口参数:  data      ---        发送数据
**            len       ---        发送长度
**出口参数:  无
**函数功能:  LTE 模块发送数据函数
**返 回 值:  无
*********************************************************************/
int my_lte_send(const uint8_t *data, uint16_t len)
{
    if (len == 0 || data == NULL)
    {
        return -EINVAL;
    }

    return uart_tx(lte_uart_dev, data, len, SYS_FOREVER_MS);
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
        LOG_INF("LTE Power: already %s", on ? "ON" : "OFF");
        return 0;
    }

    /* 执行电源控制操作 */
    err = gpio_pin_set_dt(&lte_pwr_gpio, on ? 1 : 0);
    if (err == 0)
    {
        /* 操作成功，更新状态 */
        g_lte_power_state = on;
        LOG_INF("LTE Power Control: %s", on ? "Power ON" : "Power OFF");
    }
    else
    {
        LOG_ERR("LTE Power Control failed (err %d)", err);
    }

    return err;
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
        LOG_ERR("LTE UART device not ready");
        return -ENODEV;
    }

    if (!gpio_is_ready_dt(&lte_pwr_gpio))
    {
        LOG_ERR("LTE Power GPIO not ready");
        return -ENODEV;
    }

    /* 配置电源控制引脚为输出，默认低电平（不使能） */
    err = gpio_pin_configure_dt(&lte_pwr_gpio, GPIO_OUTPUT_INACTIVE);
    if (err)
    {
        LOG_ERR("Failed to configure LTE Power GPIO (err %d)", err);
        return err;
    }

    /* 设置 UART 异步回调 */
    err = uart_callback_set(lte_uart_dev, lte_uart_cb, NULL);
    if (err)
    {
        LOG_ERR("Failed to set LTE UART callback (err %d)", err);
        return err;
    }

    /* 开启 UART 接收 */
    err = uart_rx_enable(lte_uart_dev, lte_rx_buf_1, LTE_UART_BUF_SIZE, 10 * USEC_PER_MSEC);
    if (err)
    {
        LOG_ERR("Failed to enable LTE UART RX (err %d)", err);
        return err;
    }

    /* 初始化消息队列 */
    my_init_msg_handler(MOD_LTE, &my_lte_msgq);

    /* 启动 LTE 线程 */
    *tid = k_thread_create(&my_lte_task_data, my_lte_task_stack,
                           K_THREAD_STACK_SIZEOF(my_lte_task_stack),
                           my_lte_task, NULL, NULL, NULL,
                           MY_LTE_TASK_PRIORITY, 0, K_NO_WAIT);

    /* 设置线程名称 */
    k_thread_name_set(*tid, "MY_LTE");

    LOG_INF("LTE module initialized successfully (Loopback mode)");

    /* 初始化完成后默认开启模块电源 */
    my_lte_pwr_on(true);

    return 0;
}
