/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        my_nfc.c
**文件描述:        NFC 读卡管理模块实现文件
**当前版本:        V1.0
**作    者:        Harrison Wu (wuyujiao@jimiiot.com)
**完成日期:        2026.01.15
*********************************************************************
** 功能描述:        1. 实现 NFC 模块的 I2C 初始化与轮询逻辑
**                 2. 实现电源控制逻辑 (P1.07)
**                 3. 包含延时自动关闭功能
*********************************************************************/

#include "my_nfc.h"
#include "my_comm.h"
#include "my_main.h"
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

/* 注册 NFC 模块日志 */
LOG_MODULE_REGISTER(my_nfc, LOG_LEVEL_INF);

/* 从设备树获取硬件配置 */
#define NFC_I2C_NODE DT_ALIAS(nfc_i2c)
static const struct device *nfc_i2c_dev = DEVICE_DT_GET(NFC_I2C_NODE);

#define NFC_PWR_NODE DT_ALIAS(nfc_pwr_ctrl)
static const struct gpio_dt_spec nfc_pwr_gpio = GPIO_DT_SPEC_GET(NFC_PWR_NODE, gpios);

/* 内部状态管理 */
struct my_nfc_context
{
    struct k_work_delayable poll_work; /* 轮询工作项 */
    struct k_timer stop_timer;         /* 超时停止定时器 */
    bool is_working;                   /* 是否处于工作状态 */
    uint32_t work_duration_ms;         /* 当前设定的工作时长 */
};

static struct my_nfc_context nfc_ctx;

/* 消息队列定义 */
K_MSGQ_DEFINE(my_nfc_msgq, sizeof(MSG_S), 10, 4);

/* 线程数据与栈定义 */
K_THREAD_STACK_DEFINE(my_nfc_task_stack, MY_NFC_TASK_STACK_SIZE);
static struct k_thread my_nfc_task_data;

/********************************************************************
**函数名称:  my_nfc_task
**入口参数:  无
**出口参数:  无
**函数功能:  NFC 模块主线程，处理来自消息队列的任务
**返 回 值:  无
*********************************************************************/
static void my_nfc_task(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    MSG_S msg;

    LOG_INF("NFC thread started");

    for (;;)
    {
        my_recv_msg(&my_nfc_msgq, (void *)&msg, sizeof(MSG_S), K_FOREVER);

        switch (msg.msgID)
        {
            /* TODO: 添加 NFC 相关的消息处理逻辑 */
            default:
                break;
        }
    }
}

/* NFC 模块假定的 I2C 地址 (实际移植时根据厂商手册修改) */
#define NFC_DEVICE_ADDR 0x28

/********************************************************************
**函数名称:  nfc_poll_handler
**入口参数:  work     ---        工作项指针
**出口参数:  无
**函数功能:  NFC 轮询任务处理函数，周期性读取 I2C 数据
**返 回 值:  无
*********************************************************************/
static void nfc_poll_handler(struct k_work *work)
{
    if (!nfc_ctx.is_working)
    {
        return;
    }

    uint8_t dummy_data[4];
    int err;

    /* 简单的 I2C 读取测试 (stub) */
    err = i2c_read(nfc_i2c_dev, dummy_data, sizeof(dummy_data), NFC_DEVICE_ADDR);
    if (err == 0)
    {
        LOG_INF("NFC Data detected (stub)");
        /* 此处后续移植厂商的解析代码 */
    }

    /* 继续下一次轮询 (例如每 500ms 一次) */
    k_work_reschedule(&nfc_ctx.poll_work, K_MSEC(500));
}

/********************************************************************
**函数名称:  nfc_stop_timer_handler
**入口参数:  timer    ---        定时器指针
**出口参数:  无
**函数功能:  超时自动关闭处理
**返 回 值:  无
*********************************************************************/
static void nfc_stop_timer_handler(struct k_timer *timer)
{
    LOG_INF("NFC work timeout, powering off...");
    my_nfc_stop_work();
}

/********************************************************************
**函数名称:  my_nfc_pwr_on
**入口参数:  on      ---        是否开启
**出口参数:  无
**函数功能:  NFC 模块电源控制函数
**返 回 值:  无
*********************************************************************/
int my_nfc_pwr_on(bool on)
{
    LOG_INF("NFC Power: %s", on ? "ON" : "OFF");
    return gpio_pin_set_dt(&nfc_pwr_gpio, on ? 1 : 0);
}

/********************************************************************
**函数名称:  my_nfc_start_work
**入口参数:  timeout_s    ---        工作时长 (秒)
**出口参数:  无
**函数功能:  启动 NFC 模块工作，开始轮询任务并启动电源
**返 回 值:  无
*********************************************************************/
int my_nfc_start_work(uint32_t timeout_s)
{
    if (nfc_ctx.is_working)
    {
        LOG_WRN("NFC is already working");
        return -EBUSY;
    }

    uint32_t duration = (timeout_s == 0) ? NFC_DEFAULT_WORK_TIME_S : timeout_s;

    /* 开启电源 */
    my_nfc_pwr_on(true);

    /* 等待模块启动稳定 */
    k_sleep(K_MSEC(100));

    nfc_ctx.is_working = true;
    nfc_ctx.work_duration_ms = duration * 1000;

    /* 启动超时定时器 */
    k_timer_start(&nfc_ctx.stop_timer, K_SECONDS(duration), K_NO_WAIT);

    /* 立即启动第一次轮询 */
    k_work_reschedule(&nfc_ctx.poll_work, K_NO_WAIT);

    LOG_INF("NFC polling started for %d seconds", duration);
    return 0;
}

/********************************************************************
**函数名称:  my_nfc_stop_work
**入口参数:  无
**出口参数:  无
**函数功能:  停止 NFC 模块工作，取消轮询任务并关闭电源
**返 回 值:  无
*********************************************************************/
void my_nfc_stop_work(void)
{
    nfc_ctx.is_working = false;

    /* 停止所有待处理的任务 */
    k_timer_stop(&nfc_ctx.stop_timer);
    k_work_cancel_delayable(&nfc_ctx.poll_work);

    /* 关闭电源 */
    my_nfc_pwr_on(false);

    LOG_INF("NFC work stopped");
}

/********************************************************************
**函数名称:  my_nfc_init
**入口参数:  tid      ---        线程ID
**出口参数:  无
**函数功能:  NFC 模块初始化函数，配置 I2C 与 GPIO，启动线程
**返 回 值:  无
*********************************************************************/
int my_nfc_init(k_tid_t *tid)
{
    int err;

    /* 1. 检查硬件设备是否就绪 */
    if (!device_is_ready(nfc_i2c_dev))
    {
        LOG_ERR("NFC I2C device not ready");
        return -ENODEV;
    }

    if (!gpio_is_ready_dt(&nfc_pwr_gpio))
    {
        LOG_ERR("NFC Power GPIO not ready");
        return -ENODEV;
    }

    /* 2. 配置电源引脚 */
    err = gpio_pin_configure_dt(&nfc_pwr_gpio, GPIO_OUTPUT_INACTIVE);
    if (err)
    {
        LOG_ERR("Failed to configure NFC Power GPIO (err %d)", err);
        return err;
    }

    /* 3. 初始化工作项与定时器 */
    k_work_init_delayable(&nfc_ctx.poll_work, nfc_poll_handler);
    k_timer_init(&nfc_ctx.stop_timer, nfc_stop_timer_handler, NULL);

    nfc_ctx.is_working = false;

    /* 4. 初始化消息队列 */
    my_init_msg_handler(MOD_NFC, &my_nfc_msgq);

    /* 5. 启动 NFC 线程 */
    *tid = k_thread_create(&my_nfc_task_data, my_nfc_task_stack,
                           K_THREAD_STACK_SIZEOF(my_nfc_task_stack),
                           my_nfc_task, NULL, NULL, NULL,
                           MY_NFC_TASK_PRIORITY, 0, K_NO_WAIT);

    /* 6. 设置线程名称 */
    k_thread_name_set(*tid, "MY_NFC");

    LOG_INF("NFC module initialized");
    return 0;
}