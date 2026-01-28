/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        my_shell.c
**文件描述:        Shell 命令行交互模块
**当前版本:        V1.0
**作    者:        Harrison Wu (wuyujiao@jimiiot.com)
**完成日期:        2026.01.15
*********************************************************************
** 功能描述:        Shell 人机交互与蓝牙透传模块
**                 1. 通过 UART 接收 ASCII 码指令
**                 2. 蓝牙连接时透传 UART 数据到蓝牙
**                 3. 蓝牙断开时丢弃 UART 数据
**                 4. 将蓝牙数据透传到 UART 输出
*********************************************************************/

#include "my_shell.h"
#include "my_comm.h"
#include "my_main.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <uart_async_adapter.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(my_shell, LOG_LEVEL_INF);

#ifdef CONFIG_UART_ASYNC_ADAPTER
UART_ASYNC_ADAPTER_INST_DEFINE(async_adapter);
#else
#define async_adapter NULL
#endif

struct my_shell_context
{
    const struct device *uart;          /* 当前使用的 UART 设备（可能为 async_adapter） */
    struct k_work_delayable uart_work;  /* 用于重新启动 RX 的延时工作 */
    struct k_fifo *uart_rx_to_ble_fifo; /* UART RX -> BLE 方向 FIFO */
    struct k_fifo *ble_tx_to_uart_fifo; /* BLE -> UART 方向 FIFO */
    bool ble_connected;                 /* 蓝牙连接状态 */
};

static struct my_shell_context shell_ctx = {
    .ble_connected = false,
};

/* 消息队列定义 */
K_MSGQ_DEFINE(my_shell_msgq, sizeof(MSG_S), 10, 4);

/* 线程数据与栈定义 */
K_THREAD_STACK_DEFINE(my_shell_task_stack, MY_SHELL_TASK_STACK_SIZE);
static struct k_thread my_shell_task_data;

/* UART 异步 API 是否可用 */
/********************************************************************
**函数名称:  uart_test_async_api
**入口参数:  dev      ---        UART 设备指针
**出口参数:  无
**函数功能:  检查指定 UART 设备是否支持异步回调 API
**返 回 值:  true 表示支持异步 API，false 表示不支持
*********************************************************************/
static bool uart_test_async_api(const struct device *dev)
{
    const struct uart_driver_api *api = (const struct uart_driver_api *)dev->api;

    return (api->callback_set != NULL);
}

/* UART 回调函数：发送完成、中断接收等处理 */
/********************************************************************
**函数名称:  uart_cb
**入口参数:  dev      ---        触发回调的 UART 设备指针
**            evt      ---        UART 事件结构体指针
**            user_data ---       用户上下文指针（本工程未使用）
**出口参数:  无
**函数功能:  处理 UART 发送完成、接收就绪、缓冲区申请与释放等异步事件
**返 回 值:  无
*********************************************************************/
static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(user_data);

    static size_t aborted_len;
    struct shell_uart_data_t *buf;
    static uint8_t *aborted_buf;
    static bool disable_req;

    switch (evt->type)
    {
        case UART_TX_DONE:
            LOG_INF("UART_TX_DONE");
            if ((evt->data.tx.len == 0) ||
                (!evt->data.tx.buf))
            {
                return;
            }

            if (aborted_buf)
            {
                buf = CONTAINER_OF(aborted_buf, struct shell_uart_data_t, data[0]);
                aborted_buf = NULL;
                aborted_len = 0;
            }
            else
            {
                buf = CONTAINER_OF(evt->data.tx.buf, struct shell_uart_data_t, data[0]);
            }

            k_free(buf);

            buf = k_fifo_get(shell_ctx.ble_tx_to_uart_fifo, K_NO_WAIT);
            if (!buf)
            {
                return;
            }

            if (uart_tx(shell_ctx.uart, buf->data, buf->len, SYS_FOREVER_MS))
            {
                LOG_WRN("Failed to send data over UART");
            }

            break;

        case UART_RX_RDY:
            LOG_DBG("UART_RX_RDY");
            buf = CONTAINER_OF(evt->data.rx.buf, struct shell_uart_data_t, data[0]);
            buf->len += evt->data.rx.len;

            if (disable_req)
            {
                return;
            }

            if ((evt->data.rx.buf[buf->len - 1] == '\n') ||
                (evt->data.rx.buf[buf->len - 1] == '\r'))
            {
                disable_req = true;
                uart_rx_disable(shell_ctx.uart);
            }

            break;

        case UART_RX_DISABLED:
            LOG_DBG("UART_RX_DISABLED");
            disable_req = false;

            buf = k_malloc(sizeof(*buf));
            if (buf)
            {
                buf->len = 0;
            }
            else
            {
                LOG_WRN("Not able to allocate UART receive buffer");
                k_work_reschedule(&shell_ctx.uart_work, SHELL_UART_WAIT_FOR_BUF_DELAY);
                return;
            }

            uart_rx_enable(shell_ctx.uart, buf->data, sizeof(buf->data), SHELL_UART_WAIT_FOR_RX);

            break;

        case UART_RX_STOPPED:
            break;

        case UART_RX_BUF_REQUEST:
            LOG_DBG("UART_RX_BUF_REQUEST");
            buf = k_malloc(sizeof(*buf));
            if (buf)
            {
                buf->len = 0;
                uart_rx_buf_rsp(shell_ctx.uart, buf->data, sizeof(buf->data));
            }
            else
            {
                LOG_WRN("Not able to allocate UART receive buffer");
            }

            break;

        case UART_RX_BUF_RELEASED:
            LOG_DBG("UART_RX_BUF_RELEASED");
            buf = CONTAINER_OF(evt->data.rx_buf.buf, struct shell_uart_data_t, data[0]);

            if (buf->len > 0)
            {
                /* 只有在蓝牙已连接时才将数据放入 FIFO 进行透传 */
                if (shell_ctx.ble_connected)
                {
                    k_fifo_put(shell_ctx.uart_rx_to_ble_fifo, buf);
                }
                else
                {
                    /* 蓝牙未连接，丢弃数据 */
                    LOG_DBG("BLE not connected, discarding UART RX data");
                    k_free(buf);
                }
            }
            else
            {
                k_free(buf);
            }

            break;

        case UART_TX_ABORTED:
            LOG_DBG("UART_TX_ABORTED");
            if (!aborted_buf)
            {
                aborted_buf = (uint8_t *)evt->data.tx.buf;
            }

            aborted_len += evt->data.tx.len;
            buf = CONTAINER_OF((void *)aborted_buf, struct shell_uart_data_t, data);

            uart_tx(shell_ctx.uart, &buf->data[aborted_len],
                    buf->len - aborted_len, SYS_FOREVER_MS);

            break;

        default:
            break;
    }
}

/* 当 RX buffer 释放或错误时，重新启动接收 */
/********************************************************************
**函数名称:  uart_work_handler
**入口参数:  item     ---        Zephyr 工作队列任务指针
**出口参数:  无
**函数功能:  在接收缓冲不足等情况下重新分配 RX 缓冲并启动 UART 接收
**返 回 值:  无
*********************************************************************/
static void uart_work_handler(struct k_work *item)
{
    ARG_UNUSED(item);

    struct shell_uart_data_t *buf;

    buf = k_malloc(sizeof(*buf));
    if (buf)
    {
        buf->len = 0;
    }
    else
    {
        LOG_WRN("Not able to allocate UART receive buffer");
        k_work_reschedule(&shell_ctx.uart_work, SHELL_UART_WAIT_FOR_BUF_DELAY);
        return;
    }

    uart_rx_enable(shell_ctx.uart, buf->data, sizeof(buf->data), SHELL_UART_WAIT_FOR_RX);
}

/* 内部 UART 初始化实现 */
/********************************************************************
**函数名称:  uart_low_level_init
**入口参数:  无
**出口参数:  无
**函数功能:  完成 UART 设备就绪检测、行控制配置、欢迎信息发送及接收启动
**返 回 值:  0 表示成功，负值表示失败（如设备未就绪或配置错误）
*********************************************************************/
static int uart_low_level_init(void)
{
    int err;
    int pos;
    struct shell_uart_data_t *rx;
    struct shell_uart_data_t *tx;

    if (!device_is_ready(shell_ctx.uart))
    {
        return -ENODEV;
    }

    rx = k_malloc(sizeof(*rx));
    if (rx)
    {
        rx->len = 0;
    }
    else
    {
        return -ENOMEM;
    }

    k_work_init_delayable(&shell_ctx.uart_work, uart_work_handler);

    if (IS_ENABLED(CONFIG_UART_ASYNC_ADAPTER) && !uart_test_async_api(shell_ctx.uart))
    {
        /* Implement API adapter */
        uart_async_adapter_init(async_adapter, shell_ctx.uart);
        shell_ctx.uart = async_adapter;
    }

    err = uart_callback_set(shell_ctx.uart, uart_cb, NULL);
    if (err)
    {
        k_free(rx);
        LOG_ERR("Cannot initialize UART callback");
        return err;
    }

    if (IS_ENABLED(CONFIG_UART_LINE_CTRL))
    {
        LOG_INF("Wait for DTR");
        while (true)
        {
            uint32_t dtr = 0;

            uart_line_ctrl_get(shell_ctx.uart, UART_LINE_CTRL_DTR, &dtr);
            if (dtr)
            {
                break;
            }
            /* Give CPU resources to low priority threads. */
            k_sleep(K_MSEC(100));
        }
        LOG_INF("DTR set");
        err = uart_line_ctrl_set(shell_ctx.uart, UART_LINE_CTRL_DCD, 1);
        if (err)
        {
            LOG_WRN("Failed to set DCD, ret code %d", err);
        }
        err = uart_line_ctrl_set(shell_ctx.uart, UART_LINE_CTRL_DSR, 1);
        if (err)
        {
            LOG_WRN("Failed to set DSR, ret code %d", err);
        }
    }

    tx = k_malloc(sizeof(*tx));

    if (tx)
    {
        pos = snprintf(tx->data, sizeof(tx->data),
                       "Starting Shell Command Interface\r\n");

        if ((pos < 0) || (pos >= sizeof(tx->data)))
        {
            k_free(rx);
            k_free(tx);
            LOG_ERR("snprintf returned %d", pos);
            return -ENOMEM;
        }

        tx->len = pos;
    }
    else
    {
        k_free(rx);
        return -ENOMEM;
    }

    err = uart_tx(shell_ctx.uart, tx->data, tx->len, SYS_FOREVER_MS);
    LOG_INF("Welcome msg TX: err %d", err);
    if (err)
    {
        k_free(rx);
        k_free(tx);
        LOG_ERR("Cannot display welcome message (err: %d)", err);
        return err;
    }

    err = uart_rx_enable(shell_ctx.uart, rx->data, sizeof(rx->data), SHELL_UART_WAIT_FOR_RX);
    if (err)
    {
        LOG_ERR("Cannot enable uart reception (err: %d)", err);
        /* Free the rx buffer only because the tx buffer will be handled in the callback */
        k_free(rx);
    }

    return err;
}

/********************************************************************
**函数名称:  my_shell_send_from_ble
**入口参数:  data     ---        待发送的数据缓冲区指针
**            len      ---        待发送的数据长度
**出口参数:  无
**函数功能:  将 BLE 收到的数据通过 UART 发送，必要时缓存到 FIFO 等待 UART 空闲
**返 回 值:  0 表示成功，负值表示失败（如内存不足或参数非法）
*********************************************************************/
int my_shell_send_from_ble(const uint8_t *data, uint16_t len)
{
    if ((data == NULL) || (len == 0U))
    {
        return -EINVAL;
    }

    for (uint16_t pos = 0; pos != len;)
    {
        struct shell_uart_data_t *tx = k_malloc(sizeof(*tx));

        if (!tx)
        {
            LOG_WRN("Not able to allocate UART send data buffer");
            return -ENOMEM;
        }

        /* Keep the last byte of TX buffer for potential LF char. */
        size_t tx_data_size = sizeof(tx->data) - 1U;

        if ((len - pos) > tx_data_size)
        {
            tx->len = tx_data_size;
        }
        else
        {
            tx->len = (len - pos);
        }

        memcpy(tx->data, &data[pos], tx->len);

        pos += tx->len;

        /* Append the LF character when the CR character triggered
         * transmission from the peer.
         */
        if ((pos == len) && (data[len - 1U] == '\r'))
        {
            tx->data[tx->len] = '\n';
            tx->len++;
        }

        int err = uart_tx(shell_ctx.uart, tx->data, tx->len, SYS_FOREVER_MS);
        if (err)
        {
            /* 如果当前 UART 正在发送，则放入 FIFO，等待 TX_DONE 回调继续发送 */
            k_fifo_put(shell_ctx.ble_tx_to_uart_fifo, tx);
        }
    }

    return 0;
}

/********************************************************************
**函数名称:  my_shell_set_ble_connected
**入口参数:  connected ---       蓝牙连接状态（true: 已连接，false: 未连接）
**出口参数:  无
**函数功能:  设置蓝牙连接状态，控制 UART RX 数据是否透传到蓝牙
**返 回 值:  无
*********************************************************************/
void my_shell_set_ble_connected(bool connected)
{
    shell_ctx.ble_connected = connected;
    LOG_INF("Shell BLE connection status: %s", connected ? "Connected" : "Disconnected");
}

/********************************************************************
**函数名称:  my_shell_task
**入口参数:  无
**出口参数:  无
**函数功能:  Shell 模块主线程，处理来自消息队列的任务
**返 回 值:  无
*********************************************************************/
static void my_shell_task(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    MSG_S msg;

    LOG_INF("Shell thread started");

    for (;;)
    {
        my_recv_msg(&my_shell_msgq, (void *)&msg, sizeof(MSG_S), K_FOREVER);

        switch (msg.msgID)
        {
            /* TODO: 添加 Shell 相关的消息处理逻辑 */
            default:
                break;
        }
    }
}

/********************************************************************
**函数名称:  my_shell_init
**入口参数:  param    ---        Shell 模块初始化参数结构体
**           tid      ---        指向线程 ID 变量的指针
**出口参数:  tid      ---        存储启动后的线程 ID
**函数功能:  绑定底层 UART 设备和透传 FIFO，初始化消息队列并启动 Shell 线程
**返 回 值:  0 表示成功，负值表示失败（如设备未就绪等错误）
*********************************************************************/
int my_shell_init(const struct my_shell_init_param *param, k_tid_t *tid)
{
    int err;

    if (param == NULL)
    {
        return -EINVAL;
    }

    shell_ctx.uart = param->uart_dev;
    shell_ctx.uart_rx_to_ble_fifo = param->uart_rx_to_ble_fifo;
    shell_ctx.ble_tx_to_uart_fifo = param->ble_tx_to_uart_fifo;
    shell_ctx.ble_connected = false;

    err = uart_low_level_init();
    if (err)
    {
        return err;
    }

    /* 初始化消息队列 */
    my_init_msg_handler(MOD_SHELL, &my_shell_msgq);

    /* 启动 Shell 线程 */
    *tid = k_thread_create(&my_shell_task_data, my_shell_task_stack,
                           K_THREAD_STACK_SIZEOF(my_shell_task_stack),
                           my_shell_task, NULL, NULL, NULL,
                           MY_SHELL_TASK_PRIORITY, 0, K_NO_WAIT);

    /* 设置线程名称 */
    k_thread_name_set(*tid, "my_shell_task");

    LOG_INF("Shell module initialized");
    return 0;
}