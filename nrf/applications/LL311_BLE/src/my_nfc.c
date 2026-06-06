/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        my_nfc.c
**文件描述:        NFC 读卡管理模块实现文件
**当前版本:        V3.0
**作    者:        Harrison Wu (wuyujiao@jimiiot.com)
**完成日期:        2026.02.28
*********************************************************************
** 功能描述:        1. 通过 nfc_api 接口操作 NFC 功能
**                 2. 支持 Mifare Classic 和 NTag 卡片 (现在卡片类型为TYPE-A)，UUID 读取
**                 3. 初始化后进入 HPD 模式，等待启动轮询
*********************************************************************/

/* 必须在包含 my_comm.h 之前定义 BLE_LOG_MODULE_ID，避免与 my_ble_log.h 中的默认定义冲突 */
#define BLE_LOG_MODULE_ID BLE_LOG_MOD_NFC

#include "my_comm.h"
#include "nfc_api.h"

/* 注册 NFC 模块日志 */
LOG_MODULE_REGISTER(my_nfc, LOG_LEVEL_INF);

/* NFC NPD 控制（低功耗模式控制引脚，P2.04）
 * 硬件设计：NFC 常供电，通过 NPD 引脚控制芯片低功耗模式（600nA）
 */
#define NFC_NPD_NODE DT_ALIAS(nfc_npd_ctrl)
static const struct gpio_dt_spec nfc_npd_gpio = GPIO_DT_SPEC_GET(NFC_NPD_NODE, gpios);

/* 内部状态管理 */
struct my_nfc_context_t
{
    bool is_working;
    bool card_present;
    bool in_hpd_mode;        /* 当前是否处于 HPD 模式 */
    uint32_t poll_timeout_s; /* 轮询超时时间（秒） */
};

/* 默认轮询超时时间：10秒 */
#define NFC_DEFAULT_POLL_TIMEOUT_S 10

/* 定时器用于轮询超时 */
static struct k_timer s_nfc_poll_timer;

static struct my_nfc_context_t s_nfc_ctx;

/* 消息队列定义 */
K_MSGQ_DEFINE(my_nfc_msgq, sizeof(msg_t), 10, 4);

/* 线程数据与栈定义 */
K_THREAD_STACK_DEFINE(my_nfc_task_stack, MY_NFC_TASK_STACK_SIZE);
static struct k_thread s_my_nfc_task_data;

/* 卡片信息缓冲区 */
static struct nfc_card_info s_card_info_buffer;

/* 电源管理回调函数前置声明 */
static int nfc_pm_init(void);
static int nfc_pm_suspend(void);
static int nfc_pm_resume(void);

/* NFC 电源管理操作回调结构体 */
static const pm_device_ops_t s_nfc_pm_ops = {
    .init = nfc_pm_init,
    .suspend = nfc_pm_suspend,
    .resume = nfc_pm_resume,
};

/********************************************************************
**函数名称:  my_nfc_poll_timeout_handler
**入口参数:  timer    ---        定时器指针
**出口参数:  无
**函数功能:  轮询超时定时器回调，发送消息到 NFC 线程处理
**返 回 值:  无
*********************************************************************/
static void my_nfc_poll_timeout_handler(struct k_timer *timer)
{
    ARG_UNUSED(timer);

    /* 发送超时消息到 NFC 线程，避免在中断中执行睡眠操作 */
    my_send_msg(MOD_NFC, MOD_NFC, MY_MSG_NFC_POLL_TIMEOUT);
}

/********************************************************************
**函数名称:  my_nfc_card_detected_cb
**入口参数:  type     ---        卡片类型
**           uid      ---        UID 数据
**           uid_len  ---        UID 长度
**           data     ---        卡片数据
**           data_len ---        数据长度
**出口参数:  无
**函数功能:  卡片检测回调函数，发送卡片信息到主任务处理
**返 回 值:  无
*********************************************************************/
static void my_nfc_card_detected_cb(nfc_card_type_t type,
                                    uint8_t *uid, uint8_t uid_len,
                                    uint8_t *data, uint8_t data_len)
{
    MY_LOG_DBG("Card detected, type: %d", type);
    MY_LOG_DBG("UID: %02X%02X%02X%02X", uid[0], uid[1], uid[2], uid[3]);
    s_nfc_ctx.card_present = true;

    /* 保存卡片信息 */
    s_card_info_buffer.type = type;
    s_card_info_buffer.uid_len = uid_len;
    memcpy(s_card_info_buffer.uid, uid, uid_len);
    s_card_info_buffer.data_len = data_len;
    if (data_len > 0)
    {
        memcpy(s_card_info_buffer.data, data, data_len);
    }

    /* 发送卡片事件消息到主任务 */
    msg_t msg;
    msg.msgID = MY_MSG_NFC_CARD_EVENT;
    msg.pData = &s_card_info_buffer;
    msg.DataLen = sizeof(struct nfc_card_info);
    my_send_msg_data(MOD_NFC, MOD_MAIN, &msg);

    /* 发送消息到 NFC 线程自身，由线程上下文执行 stop 和 suspend
     * 避免在中断/回调上下文中直接操作总线，防止与蓝牙冲突
     */
    my_send_msg(MOD_NFC, MOD_NFC, MY_MSG_NFC_STOP_POLL);
}

/********************************************************************
**函数名称:  my_nfc_task
**入口参数:  p1, p2, p3
**出口参数:  无
**函数功能:  NFC 模块主线程
**返 回 值:  无
*********************************************************************/
static void my_nfc_task(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    msg_t msg;
    uint32_t timeout_ms;
    int ret;

    MY_LOG_INF("NFC thread started");

    /* 初始化定时器 */
    k_timer_init(&s_nfc_poll_timer, my_nfc_poll_timeout_handler, NULL);

    MY_LOG_INF("NFC timer initialized");

    /* 初始化状态, 默认不工作 */
    s_nfc_ctx.is_working = false;
    s_nfc_ctx.card_present = false;
    s_nfc_ctx.in_hpd_mode = false;

    /* 注册 NFC 到电源管理模块 */
    ret = my_pm_device_register(MY_PM_DEV_NFC, &s_nfc_pm_ops);
    if (ret < 0)
    {
        MY_LOG_ERR("NFC PM registration failed");
        /* 注册失败，线程继续运行但无法使用 PM 功能 */
    }
    else
    {
        MY_LOG_INF("NFC PM registered successfully");
    }

    for (;;)
    {
        my_recv_msg(&my_nfc_msgq, (void *)&msg, sizeof(msg_t), K_FOREVER);

        switch (msg.msgID)
        {
            case MY_MSG_NFC_START_POLL:
                if (s_nfc_ctx.is_working)
                {
                    MY_LOG_INF("NFC polling already running");
                    break;
                }

                /* 通过电源管理模块恢复 NFC 设备（会自动恢复 I2C22 总线），会调用 nfc_pm_resume
                 * 在线程上下文中执行，避免与蓝牙冲突
                 */
                ret = my_pm_device_resume(MY_PM_DEV_NFC);
                if (ret < 0)
                {
                    MY_LOG_ERR("Failed to resume NFC device: %d", ret);
                    break;
                }

                /* 从消息中获取超时时间（秒） */
                if (msg.DataLen >= sizeof(uint32_t))
                {
                    s_nfc_ctx.poll_timeout_s = *(uint32_t *)msg.pData;
                }
                else
                {
                    s_nfc_ctx.poll_timeout_s = NFC_DEFAULT_POLL_TIMEOUT_S;
                }
                timeout_ms = s_nfc_ctx.poll_timeout_s * 1000;
                MY_LOG_INF("Starting NFC polling for %d s", s_nfc_ctx.poll_timeout_s);
                s_nfc_ctx.is_working = true;
                s_nfc_ctx.card_present = false;
                nfc_api_poll_start();
                k_timer_start(&s_nfc_poll_timer, K_MSEC(timeout_ms), K_NO_WAIT);

                /* 开始轮询后闪烁LED*/
                my_lock_led_msg_send(LOCK_LED_NFC_START);
                break;

            case MY_MSG_NFC_STOP_POLL:
            case MY_MSG_NFC_POLL_TIMEOUT:
                if (!s_nfc_ctx.is_working)
                {
                    MY_LOG_INF("NFC polling already stopped");
                    return;
                }

                /* 停止轮询定时器 */
                k_timer_stop(&s_nfc_poll_timer);

                /* 关闭 NFC 指示灯 */
                my_lock_led_msg_send(LOCK_LED_CLOSE);

                /* 通过电源管理模块挂起 NFC 设备（会自动挂起 I2C22 总线），会调用 nfc_pm_suspend */
                my_pm_device_suspend(MY_PM_DEV_NFC);

                MY_LOG_INF("NFC stopped and suspended");
                break;

            default:
                break;
        }
    }
}

/********************************************************************
**函数名称:  nfc_pm_resume
**入口参数:  无
**出口参数:  无
**函数功能:  NFC 电源管理恢复回调
**           1. NPD 拉高，唤醒芯片
**           2. 延时等待芯片稳定
**           3. 重新初始化 FM175XX
**返 回 值:  0 表示成功，负值表示错误码
*********************************************************************/
static int nfc_pm_resume(void)
{
    nfc_result_t result;
    int retry_count = 0;

    /* NPD 拉高，唤醒芯片 */
    gpio_pin_set_dt(&nfc_npd_gpio, 1);
    s_nfc_ctx.in_hpd_mode = false;

    /* 等待芯片稳定（datasheet 要求至少 5ms） */
    k_msleep(5);

    /* 尝试重新初始化 FM175XX 寄存器（带重试） */
    do
    {
        result = nfc_api_init(my_nfc_card_detected_cb);
        if (result == NFC_SUCCESS)
        {
            break;
        }

        retry_count++;
        MY_LOG_WRN("NFC init attempt %d failed: %d", retry_count, result);
        k_msleep(10); /* 等待 I2C 总线稳定 */
    } while (retry_count < 3);

    if (result != NFC_SUCCESS)
    {
        MY_LOG_ERR("Failed to reinitialize NFC API after %d attempts: %d", retry_count, result);
        /* 恢复失败，重新进入低功耗 */
        gpio_pin_set_dt(&nfc_npd_gpio, 0);
        s_nfc_ctx.in_hpd_mode = true;
        return -EIO;
    }

    MY_LOG_INF("NFC resumed (NPD high, FM175XX reinitialized, retries: %d)", retry_count);
    return 0;
}

/********************************************************************
**函数名称:  nfc_pm_suspend
**入口参数:  无
**出口参数:  无
**函数功能:  NFC 电源管理挂起回调
**           1. 停止 POLL
**           2. NPD 拉低，芯片进入 600nA 模式
**返 回 值:  0 表示成功，负值表示错误码
*********************************************************************/
static int nfc_pm_suspend(void)
{
    /* 停止轮询 */
    if (s_nfc_ctx.is_working)
    {
        k_timer_stop(&s_nfc_poll_timer);
        nfc_api_poll_stop();
        s_nfc_ctx.is_working = false;
    }

    /* NPD 拉低，芯片进入 600nA 低功耗模式 */
    gpio_pin_set_dt(&nfc_npd_gpio, 0);
    s_nfc_ctx.in_hpd_mode = true;

    MY_LOG_INF("NFC suspended (NPD low, ~600nA)");
    return 0;
}

/********************************************************************
**函数名称:  nfc_pm_init
**入口参数:  无
**出口参数:  无
**函数功能:  NFC 电源管理初始化回调（首次启动时调用）
**           配置 NPD 引脚，默认进入低功耗模式
**返 回 值:  0 表示成功，负值表示错误码
*********************************************************************/
static int nfc_pm_init(void)
{
    int err;

    /* 检查 NPD GPIO 就绪状态 */
    if (!gpio_is_ready_dt(&nfc_npd_gpio))
    {
        MY_LOG_ERR("NPD GPIO not ready");
        return -ENODEV;
    }

    /* 延时100ms等待硬件稳定 */
    k_msleep(100);

    /* 配置 NPD 引脚，默认拉低（进入 600nA 低功耗模式） */
    err = gpio_pin_configure_dt(&nfc_npd_gpio, GPIO_OUTPUT_INACTIVE);
    if (err)
    {
        MY_LOG_ERR("Failed to configure NPD GPIO: %d", err);
        return err;
    }

    /* 标记为已进入低功耗模式 */
    s_nfc_ctx.in_hpd_mode = true;

    MY_LOG_INF("NFC initialized in low power mode (600nA)");
    return 0;
}

/********************************************************************
**函数名称:  my_nfc_start_poll
**入口参数:  timeout_s ---       轮询超时时间（秒），0 表示使用默认值
**出口参数:  无
**函数功能:  启动 NFC 轮询功能
**返 回 值:  0 表示成功
*********************************************************************/
int my_nfc_start_poll(uint32_t timeout_s)
{
    msg_t msg;
    static uint32_t timeout_storage; /* 静态存储，确保消息处理时有效 */

    if (timeout_s == 0)
    {
        timeout_s = NFC_DEFAULT_POLL_TIMEOUT_S;
    }
    timeout_storage = timeout_s;
    msg.msgID = MY_MSG_NFC_START_POLL;
    msg.pData = &timeout_storage;
    msg.DataLen = sizeof(uint32_t);

    /* 只发送消息，由线程上下文执行 resume 和启动轮询
     * 避免在中断/回调上下文中直接操作总线，防止与蓝牙冲突
     */
    my_send_msg_data(MOD_NFC, MOD_NFC, &msg);

    return 0;
}

/********************************************************************
**函数名称:  my_nfc_stop_poll
**入口参数:  无
**出口参数:  无
**函数功能:  停止 NFC 轮询功能
**返 回 值:  0 表示成功
*********************************************************************/
int my_nfc_stop_poll(void)
{
    my_send_msg(MOD_NFC, MOD_NFC, MY_MSG_NFC_STOP_POLL);

    return 0;
}

/********************************************************************
**函数名称:  my_nfc_init
**入口参数:  tid      ---        线程 ID 指针
**出口参数:  tid      ---        线程 ID
**函数功能:  NFC 模块初始化函数
**返 回 值:  0 表示成功
*********************************************************************/
int my_nfc_init(k_tid_t *tid)
{
    /* 初始化状态 */
    s_nfc_ctx.is_working = false;
    s_nfc_ctx.card_present = false;
    s_nfc_ctx.in_hpd_mode = false;
    s_nfc_ctx.poll_timeout_s = NFC_DEFAULT_POLL_TIMEOUT_S;

    /* 初始化消息队列 */
    my_init_msg_handler(MOD_NFC, &my_nfc_msgq);

    /* 启动 NFC 线程 */
    *tid = k_thread_create(&s_my_nfc_task_data, my_nfc_task_stack,
                           K_THREAD_STACK_SIZEOF(my_nfc_task_stack),
                           my_nfc_task, NULL, NULL, NULL,
                           MY_NFC_TASK_PRIORITY, 0, K_NO_WAIT);

    k_thread_name_set(*tid, "MY_NFC");

    MY_LOG_INF("NFC module initialized (via nfc_api)");
    return 0;
}
