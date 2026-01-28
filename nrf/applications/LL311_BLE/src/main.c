/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Nordic UART Bridge Service (NUS) sample
 */
#include <uart_async_adapter.h>

#include <errno.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/types.h>

#include "my_ble_core.h"
#include "my_comm.h"
#include "my_ctrl.h"
// #include "my_gsensor.h"
#include "my_lte.h"
#include "my_main.h"
// #include "my_nfc.h"
#include "my_shell.h"

#include <soc.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#include <dk_buttons_and_leds.h>

#include <stdio.h>
#include <string.h>

#include <zephyr/logging/log.h>

#define LOG_MODULE_NAME peripheral_uart
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

/* 线程 ID 声明 */
k_tid_t my_main_task_id = NULL;
k_tid_t my_shell_task_id = NULL;
k_tid_t my_ble_task_id = NULL;
k_tid_t my_ctrl_task_id = NULL;
k_tid_t my_lte_task_id = NULL;
k_tid_t my_nfc_task_id = NULL;
k_tid_t my_gsensor_task_id = NULL;
k_tid_t g_my_task_info[MAX_MY_MOD_TYPE] = {NULL};

/* 消息队列声明 */
K_MSGQ_DEFINE(my_main_msgq, sizeof(MSG_S), 10, 4);
struct k_msgq *g_my_msg_info[MAX_MY_MOD_TYPE] = {NULL};

/* 定时器声明 */
struct k_timer g_my_timer_info[MY_TIMER_MAX_ID];
bool g_my_timer_init_status[MY_TIMER_MAX_ID] = {false};

/* 运行状态 LED 定义 */
#define RUN_STATUS_LED         DK_LED1
#define RUN_LED_BLINK_INTERVAL 1000

/* Shell UART 透传 FIFO：
 * - fifo_uart_rx_data: UART RX -> BLE 方向的数据缓存
 * - fifo_uart_tx_data: BLE -> UART 方向的数据缓存
 */
static K_FIFO_DEFINE(fifo_uart_tx_data);
static K_FIFO_DEFINE(fifo_uart_rx_data);

/********************************************************************
**函数名称:  error
**入口参数:  无
**出口参数:  无
**函数功能:  进入系统错误状态，点亮所有 LED 并阻塞在死循环中
**返 回 值:  无
*********************************************************************/
void error(void)
{
    /* 所有 LED 亮，表示系统错误状态 */
    dk_set_leds_state(DK_ALL_LEDS_MSK, DK_NO_LEDS_MSK);

    while (true)
    {
        /* Spin for ever */
        k_sleep(K_MSEC(1000));
    }
}

/********************************************************************
**函数名称:  configure_gpio
**入口参数:  无
**出口参数:  无
**函数功能:  初始化按键与 LED 相关的 GPIO 资源，并注册 BLE 配对按键回调
**返 回 值:  无
*********************************************************************/
static void configure_gpio(void)
{
    int err;

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
    /* 按键事件交给 BLE 模块处理，用于配对确认/拒绝 */
    err = dk_buttons_init(my_ble_button_changed);
    if (err)
    {
        LOG_ERR("Cannot init buttons (err: %d)", err);
    }
#endif /* CONFIG_BT_NUS_SECURITY_ENABLED */

    err = dk_leds_init();
    if (err)
    {
        LOG_ERR("Cannot init LEDs (err: %d)", err);
    }
}

/*********************************************************************
**函数名称:  my_system_reset
**入口参数:  无
**出口参数:  无
**函数功能:  系统复位函数
*********************************************************************/
void my_system_reset(void)
{
    LOG_ERR("System reset");
    JM_SLEEP(K_SECONDS(1));
    sys_reboot(SYS_REBOOT_WARM);
}

/*********************************************************************
**函数名称:  custom_task_info_init
**入口参数:  无
**出口参数:  无
**函数功能:  初始化任务数据信息结构
*********************************************************************/
void custom_task_info_init(void)
{
    g_my_task_info[MOD_MAIN] = my_main_task_id;
    g_my_task_info[MOD_SHELL] = my_shell_task_id;
    g_my_task_info[MOD_BLE] = my_ble_task_id;
    g_my_task_info[MOD_CTRL] = my_ctrl_task_id;
    g_my_task_info[MOD_LTE] = my_lte_task_id;
    g_my_task_info[MOD_NFC] = my_nfc_task_id;
    g_my_task_info[MOD_GSENSOR] = my_gsensor_task_id;
}

/*********************************************************************
**函数名称:  my_init_msg_handler
**入口参数:  mod - 任务类型，msgq - 消息队列
**出口参数:  无
**函数功能:  初始化任务数据信息结构
*********************************************************************/
void my_init_msg_handler(module_type mod, struct k_msgq *msgq)
{
    if (msgq == NULL)
    {
        LOG_ERR("Invalid message queue (mod: %d)", mod);
        return;
    }

    /* 保存消息队列指针 */
    g_my_msg_info[mod] = msgq;
}

/*********************************************************************
**函数名称:  my_send_msg
**入口参数:  src_mod_id   --  发送消息的源模块ID
**           dest_mod_id  --  接收消息的目标模块ID
**           msg          --  消息ID
**出口参数:  无
**函数功能:  向指定模块发送简单消息 (不带附加数据)
*********************************************************************/
void my_send_msg(module_type src_mod_id, module_type dest_mod_id, uint32_t msg)
{
    MSG_S sendMsg = {.msgID = msg, .pData = NULL, .DataLen = 0};
    struct k_msgq *destHdl = g_my_msg_info[dest_mod_id];

    if (destHdl == NULL)
    {
#if 0 // NOTE: 在定时器回调中调用打印接口设备会死机
        LOG_ERR("dest thread is not ready! dest_mod_id=%d msgid=%d", dest_mod_id, msg);
#endif
        return;
    }

    /* 将消息放入目标队列 */
    k_msgq_put(destHdl, (void *)(&sendMsg), K_NO_WAIT);
}

/*********************************************************************
**函数名称:  my_send_msg_data
**入口参数:  src_mod_id   --  发送消息的源模块ID
**           dest_mod_id  --  接收消息的目标模块ID
**           msg          --  消息结构体指针 (MSG_S)
**出口参数:  无
**函数功能:  向指定模块发送包含数据的完整消息结构
*********************************************************************/
void my_send_msg_data(module_type src_mod_id, module_type dest_mod_id, MSG_S *msg)
{
    struct k_msgq *destHdl = g_my_msg_info[dest_mod_id];

    if (destHdl == NULL)
    {
#if 0 // NOTE: 在定时器回调中调用打印接口设备会死机
        LOG_ERR("dest thread is not ready!");
#endif
        return;
    }

    /* 将消息放入目标队列 */
    k_msgq_put(destHdl, (void *)msg, K_NO_WAIT);
}

/*********************************************************************
**函数名称:  my_recv_msg
**入口参数:  msg_queue    --  消息队列句柄
**           msg          --  存储接收数据的缓冲区指针
**           msg_size     --  消息大小 (由队列定义决定，此处仅作预留)
**           wait_option  --  等待选项 (K_NO_WAIT, K_FOREVER等)
**出口参数:  msg          --  接收到的消息内容
**函数功能:  从指定消息队列接收消息
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
int my_recv_msg(void *msg_queue, void *msg, uint32_t msg_size, k_timeout_t wait_option)
{
    return (int)k_msgq_get(msg_queue, msg, wait_option);
}

/*********************************************************************
**函数名称:  my_timer_expiry_function
**入口参数:  timer    --  Zephyr 定时器结构体指针
**出口参数:  无
**函数功能:  Zephyr 定时器超时回调包装函数
*********************************************************************/
static void my_timer_expiry_function(struct k_timer *timer)
{
    TIMER_FUN fun = (TIMER_FUN)k_timer_user_data_get(timer);
    if (fun)
    {
        fun(timer);
    }
}
/*********************************************************************
**函数名称:  my_stop_timer
**入口参数:  timerId    --  定时器ID
**出口参数:  无
**函数功能:  停止指定定时器
*********************************************************************/
void my_stop_timer(int timerId)
{
    if (timerId < 0 || timerId >= MY_TIMER_MAX_ID)
    {
        return;
    }

    if (g_my_timer_init_status[timerId])
    {
        k_timer_stop(&g_my_timer_info[timerId]);
    }
}

/*********************************************************************
**函数名称:  my_start_timer
**入口参数:  timerId    --  定时器ID
**           ms         --  定时器超时时间 (单位: 毫秒)
**           isPeriod   --  是否重复定时
**           timer_fun  --  定时器超时回调函数
**出口参数:  无
**函数功能:  启动指定定时器
*********************************************************************/
int my_start_timer(int timerId, uint32_t ms, bool isPeriod, TIMER_FUN timer_fun)
{
    if (timerId < 0 || timerId >= MY_TIMER_MAX_ID)
    {
        return -EINVAL;
    }

    /* 如果定时器未初始化，则先执行初始化并标记状态 */
    if (!g_my_timer_init_status[timerId])
    {
        k_timer_init(&g_my_timer_info[timerId], my_timer_expiry_function, NULL);
        g_my_timer_init_status[timerId] = true;
    }

    /* 停止旧的定时器 (现在已确保初始化，可以安全调用) */
    k_timer_stop(&g_my_timer_info[timerId]);

    /* 把用户回调函数指针存到 user_data */
    k_timer_user_data_set(&g_my_timer_info[timerId],
                          (void *)timer_fun);

    /* 启动定时器 */
    k_timer_start(&g_my_timer_info[timerId],
                  K_MSEC(ms),
                  isPeriod ? K_MSEC(ms) : K_NO_WAIT);

    return 0;
}

/*********************************************************************
**函数名称:  my_delete_timer
**入口参数:  timerId    --  定时器ID
**出口参数:  无
**函数功能:  停止定时器（静态方式不释放内存）
*********************************************************************/
void my_delete_timer(int timerId)
{
    if (timerId < 0 || timerId >= MY_TIMER_MAX_ID)
    {
        return;
    }

    if (g_my_timer_init_status[timerId])
    {
        k_timer_stop(&g_my_timer_info[timerId]);
        /* 静态分配下不置回状态，以便下次 start 时直接复用 */
    }
}

/*********************************************************************
**函数名称:  my_time_is_run
**入口参数:  timerId    --  定时器ID
**出口参数:  无
**函数功能:  检查指定定时器是否正在运行
**返 回 值:  true 表示正在运行，false 表示未运行或不存在
*********************************************************************/
bool my_time_is_run(int timerId)
{
    if (timerId < 0 || timerId >= MY_TIMER_MAX_ID)
    {
        return false;
    }

    if (!g_my_timer_init_status[timerId])
    {
        return false;
    }

    /* 如果剩余时间大于 0，说明定时器正在运行 */
    return (k_timer_remaining_get(&g_my_timer_info[timerId]) > 0);
}

/********************************************************************
**函数名称:  main
**入口参数:  无
**出口参数:  无
**函数功能:  作为系统入口，依次完成 GPIO、UART、BLE 模块初始化并运行主指示灯循环
**返 回 值:  0 表示程序正常运行（理论上不返回）
*********************************************************************/
int main(void)
{
    int err = 0;

    configure_gpio();

    /* 获取当前线程 ID 并保存 */
    my_main_task_id = k_current_get();

    /* 初始化 Shell 模块，并将透传 FIFO 交给 Shell 管理 */
    struct my_shell_init_param shell_param = {
        .uart_dev = DEVICE_DT_GET(DT_CHOSEN(nordic_nus_uart)),
        .uart_rx_to_ble_fifo = &fifo_uart_rx_data,
        .ble_tx_to_uart_fifo = &fifo_uart_tx_data,
    };

    err = my_shell_init(&shell_param, &my_shell_task_id);
    if (err)
    {
        error();
    }

    /* 初始化 BLE 核心模块，将同一组 FIFO 交给 BLE 使用 */
    struct my_ble_core_init_param ble_param = {
        .uart_rx_to_ble_fifo = &fifo_uart_rx_data,
        .ble_tx_to_uart_fifo = &fifo_uart_tx_data,
    };

    err = my_ble_core_init(&ble_param, &my_ble_task_id);
    if (err)
    {
        error();
    }

    /* 启动 BLE 协议栈、NUS 服务、广播以及 BLE 写线程 */
    err = my_ble_core_start();
    if (err)
    {
        error();
    }

    /* 初始化 LTE 模块 */
    err = my_lte_init(&my_lte_task_id);
    if (err)
    {
        LOG_ERR("Failed to initialize LTE module (err %d)", err);
        /* LTE 初始化失败可以选择不进入 error() 阻塞，视具体需求而定 */
    }

    /* 初始化 G-Sensor 模块 */
#if 0
    err = my_gsensor_init(&my_gsensor_task_id);
    if (err)
    {
        LOG_ERR("Failed to initialize G-Sensor (err %d)", err);
    }

    /* 初始化 NFC 模块 */
    err = my_nfc_init(&my_nfc_task_id);
    if (err)
    {
        LOG_ERR("Failed to initialize NFC (err %d)", err);
    }
#endif

    /* 初始化系统控制模块 (LED, Buzzer, Key) */
    err = my_ctrl_init(&my_ctrl_task_id);
    if (err)
    {
        LOG_ERR("Failed to initialize Control module (err %d)", err);
    }
    else
    {
        /* 启动时响一声提示音 */
        my_ctrl_buzzer_play_tone(2000, 100);
    }

    /* 初始化自定义任务信息 */
    custom_task_info_init();

    /* 初始化主线程消息队列 */
    my_init_msg_handler(MOD_MAIN, &my_main_msgq);

    /* 主循环：等待并处理消息，逻辑已迁移至各线程 */
    MSG_S msg;
    for (;;)
    {
        my_recv_msg(&my_main_msgq, (void *)&msg, sizeof(MSG_S), K_FOREVER);

        switch (msg.msgID)
        {
                /* TODO: 处理主任务相关消息 */

            default:
                break;
        }
    }

    return 0;
}
