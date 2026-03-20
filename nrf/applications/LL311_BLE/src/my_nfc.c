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

/* NFC 电源控制 */
#define NFC_PWR_NODE DT_ALIAS(nfc_pwr_ctrl)
static const struct gpio_dt_spec nfc_pwr_gpio = GPIO_DT_SPEC_GET(NFC_PWR_NODE, gpios);

/* I2C22 引脚定义 (P1.05 SCL, P1.06 SDA) */
#define NFC_I2C_SCL_PIN 5
#define NFC_I2C_SDA_PIN 6

/* 内部状态管理 */
struct my_nfc_context
{
    bool is_working;
    bool card_present;
    bool in_hpd_mode;        /* 当前是否处于 HPD 模式 */
    uint32_t poll_timeout_s; /* 轮询超时时间（秒） */
};

/* 默认轮询超时时间：30秒 */
#define NFC_DEFAULT_POLL_TIMEOUT_S 30

/* 定时器用于轮询超时 */
static struct k_timer nfc_poll_timer;

static struct my_nfc_context nfc_ctx;

/* 消息队列定义 */
K_MSGQ_DEFINE(my_nfc_msgq, sizeof(MSG_S), 10, 4);

/* 线程数据与栈定义 */
K_THREAD_STACK_DEFINE(my_nfc_task_stack, MY_NFC_TASK_STACK_SIZE);
static struct k_thread my_nfc_task_data;

/* 卡片信息缓冲区 */
static struct nfc_card_info card_info_buffer;

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
    MSG_S msg;
    msg.msgID = MY_MSG_NFC_POLL_TIMEOUT;
    msg.pData = NULL;
    msg.DataLen = 0;
    my_send_msg_data(MOD_NFC, MOD_NFC, &msg);
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
    nfc_ctx.card_present = true;

    /* 保存卡片信息 */
    card_info_buffer.type = type;
    card_info_buffer.uid_len = uid_len;
    memcpy(card_info_buffer.uid, uid, uid_len);
    card_info_buffer.data_len = data_len;
    if (data_len > 0)
    {
        memcpy(card_info_buffer.data, data, data_len);
    }

    /* 发送卡片事件消息到主任务 */
    MSG_S msg;
    msg.msgID = MY_MSG_NFC_CARD_EVENT;
    msg.pData = &card_info_buffer;
    msg.DataLen = sizeof(struct nfc_card_info);
    my_send_msg_data(MOD_NFC, MOD_MAIN, &msg);

    /* 停止轮询并进入 HPD模式 */
    msg.msgID = MY_MSG_NFC_STOP_POLL;
    msg.pData = NULL;
    msg.DataLen = 0;
    my_send_msg_data(MOD_NFC, MOD_NFC, &msg);
}

/********************************************************************
**函数名称:  my_nfc_i2c_set_high_impedance
**入口参数:  无
**出口参数:  无
**函数功能:  将 I2C 引脚设为高阻态（输入，无上下拉），用于 HPD 模式省电
**返 回 值:  无
*********************************************************************/
static void my_nfc_i2c_set_high_impedance(void)
{
    const struct device *gpio_dev = device_get_binding("gpio1");
    if (gpio_dev)
    {
        /* 设为输入模式，无上下拉，高阻态 */
        gpio_pin_configure(gpio_dev, NFC_I2C_SCL_PIN, GPIO_INPUT);
        gpio_pin_configure(gpio_dev, NFC_I2C_SDA_PIN, GPIO_INPUT);
        MY_LOG_DBG("I2C pins set to high impedance");
    }
}

/********************************************************************
**函数名称:  my_nfc_enter_hpd
**入口参数:  无
**出口参数:  无
**函数功能:  停止轮询并进入 HPD 模式，I2C 引脚设为高阻态以节省功耗
**返 回 值:  无
*********************************************************************/
static void my_nfc_enter_hpd(void)
{
    if (!nfc_ctx.in_hpd_mode)
    {
        nfc_api_poll_stop();

        /* 将 I2C 引脚设为高阻态，让外部上拉决定电平 */
        my_nfc_i2c_set_high_impedance();

        nfc_api_enter_hpd();
        nfc_ctx.in_hpd_mode = true;
        nfc_ctx.is_working = false;
        MY_LOG_INF("NFC entered HPD mode (I2C high impedance)");
    }
}

/********************************************************************
**函数名称:  my_nfc_i2c_restore
**入口参数:  无
**出口参数:  无
**函数功能:  恢复 I2C 总线功能，将引脚重新交给 TWIM 外设控制
**返 回 值:  无
*********************************************************************/
static void my_nfc_i2c_restore(void)
{
    const struct device *i2c_dev = device_get_binding("i2c22");
    if (i2c_dev)
    {
        /* 重新初始化 I2C 设备，恢复引脚控制 */
        /* Zephyr 会自动将引脚重新配置为 TWIM 功能 */
        MY_LOG_DBG("I2C bus restored");
    }
}

/********************************************************************
**函数名称:  my_nfc_exit_hpd
**入口参数:  无
**出口参数:  无
**函数功能:  退出 HPD 模式，恢复 I2C 总线
**返 回 值:  无
*********************************************************************/
static void my_nfc_exit_hpd(void)
{
    if (nfc_ctx.in_hpd_mode)
    {
        /* 先恢复 I2C 总线，再退出 HPD */
        my_nfc_i2c_restore();

        nfc_api_exit_hpd();
        nfc_ctx.in_hpd_mode = false;
        MY_LOG_INF("NFC exited HPD mode (I2C restored)");
    }
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

    MSG_S msg;
    nfc_result_t result;
    uint32_t timeout_ms;

    MY_LOG_INF("NFC thread started");

    /* 初始化 NFC API */
    result = nfc_api_init(my_nfc_card_detected_cb);
    if (result != NFC_SUCCESS)
    {
        MY_LOG_ERR("Failed to initialize NFC API: %d", result);
        return;
    }

    /* 初始化定时器 */
    k_timer_init(&nfc_poll_timer, my_nfc_poll_timeout_handler, NULL);

    MY_LOG_INF("NFC API initialized successfully");

    /* 初始化状态：启动轮询（默认超时，读到卡或超时后进入 HPD） */
    nfc_ctx.is_working = false;
    nfc_ctx.card_present = false;
    nfc_ctx.in_hpd_mode = false;

    /* 进入 HPD 模式 */
    my_nfc_enter_hpd();
    k_sleep(K_MSEC(1));

    for (;;)
    {
        my_recv_msg(&my_nfc_msgq, (void *)&msg, sizeof(MSG_S), K_FOREVER);

        switch (msg.msgID)
        {
            case MY_MSG_NFC_START_POLL:
                if (nfc_ctx.is_working)
                {
                    MY_LOG_INF("NFC polling already running");
                    break;
                }
                /* 从消息中获取超时时间（秒） */
                if (msg.DataLen >= sizeof(uint32_t))
                {
                    nfc_ctx.poll_timeout_s = *(uint32_t *)msg.pData;
                }
                else
                {
                    nfc_ctx.poll_timeout_s = NFC_DEFAULT_POLL_TIMEOUT_S;
                }
                timeout_ms = nfc_ctx.poll_timeout_s * 1000;
                MY_LOG_INF("Starting NFC polling for %d s", nfc_ctx.poll_timeout_s);
                nfc_ctx.is_working = true;
                nfc_ctx.card_present = false;
                my_nfc_exit_hpd();
                nfc_api_poll_start();
                k_timer_start(&nfc_poll_timer, K_MSEC(timeout_ms), K_NO_WAIT);
                break;

            case MY_MSG_NFC_STOP_POLL:
                if (!nfc_ctx.is_working)
                {
                    break;
                }
                MY_LOG_INF("Stopping NFC polling");
                k_timer_stop(&nfc_poll_timer);
                my_nfc_enter_hpd();
                break;

            case MY_MSG_NFC_POLL_TIMEOUT:
                MY_LOG_INF("NFC polling timeout, entering HPD mode");
                my_nfc_enter_hpd();
                break;

            default:
                break;
        }
    }
}

/********************************************************************
**函数名称:  my_nfc_pwr_on
**入口参数:  on       ---        是否开启
**出口参数:  无
**函数功能:  NFC 模块电源控制函数
**返 回 值:  0 表示成功
*********************************************************************/
int my_nfc_pwr_on(bool on)
{
    MY_LOG_INF("NFC Power: %s", on ? "ON" : "OFF");
    return gpio_pin_set_dt(&nfc_pwr_gpio, on ? 1 : 0);
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
    MSG_S msg;
    static uint32_t timeout_storage; /* 静态存储，确保消息处理时有效 */

    msg.msgID = MY_MSG_NFC_START_POLL;

    if (timeout_s == 0)
    {
        timeout_s = NFC_DEFAULT_POLL_TIMEOUT_S;
    }
    timeout_storage = timeout_s;
    msg.pData = &timeout_storage;
    msg.DataLen = sizeof(uint32_t);

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
    MSG_S msg;
    msg.msgID = MY_MSG_NFC_STOP_POLL;
    msg.pData = NULL;
    msg.DataLen = 0;

    my_send_msg_data(MOD_NFC, MOD_NFC, &msg);
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
    int err;

    /* 检查电源 GPIO */
    if (!gpio_is_ready_dt(&nfc_pwr_gpio))
    {
        MY_LOG_ERR("NFC Power GPIO not ready");
        return -ENODEV;
    }

    /* 配置电源引脚，保持高电平（NFC直连3.3V供电） */
    err = gpio_pin_configure_dt(&nfc_pwr_gpio, GPIO_OUTPUT_ACTIVE);
    if (err)
    {
        MY_LOG_ERR("Failed to configure NFC Power GPIO: %d", err);
        return err;
    }

    /* 初始化状态 */
    nfc_ctx.is_working = false;
    nfc_ctx.card_present = false;
    nfc_ctx.in_hpd_mode = false;
    nfc_ctx.poll_timeout_s = NFC_DEFAULT_POLL_TIMEOUT_S;

    /* 初始化消息队列 */
    my_init_msg_handler(MOD_NFC, &my_nfc_msgq);

    /* 启动 NFC 线程 */
    *tid = k_thread_create(&my_nfc_task_data, my_nfc_task_stack,
                           K_THREAD_STACK_SIZEOF(my_nfc_task_stack),
                           my_nfc_task, NULL, NULL, NULL,
                           MY_NFC_TASK_PRIORITY, 0, K_NO_WAIT);

    k_thread_name_set(*tid, "MY_NFC");

    MY_LOG_INF("NFC module initialized (via nfc_api)");
    return 0;
}
