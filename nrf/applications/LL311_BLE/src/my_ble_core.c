/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        my_ble_core.c
**文件描述:        蓝牙核心任务处理模块
**当前版本:        V1.0
**作    者:        Harrison Wu (wuyujiao@jimiiot.com)
**完成日期:        2026.01.13
*********************************************************************
** 功能描述:        蓝牙核心任务处理模块
**                 1. 蓝牙低功耗连接管理
**                 2. Nordic UART服务实现
**                 3. 通过消息队列与UART任务进行数据交换
**                 4. 连接状态指示和安全管理
*********************************************************************/

#include "my_ble_core.h"
#include "my_comm.h"
#include "my_main.h"
#include "my_shell.h"

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>

#include <soc.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>

#include <bluetooth/services/nus.h>

#include <dk_buttons_and_leds.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

LOG_MODULE_REGISTER(my_ble_core, LOG_LEVEL_INF);

#define CONNECTABLE_ADV_IDX       0
#define NON_CONNECTABLE_ADV_1_IDX 1
#define NON_CONNECTABLE_ADV_2_IDX 2

#define STACKSIZE CONFIG_BT_NUS_THREAD_STACK_SIZE
#define PRIORITY  7

#define DEVICE_NAME     CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define CON_STATUS_LED     DK_LED2
#define KEY_PASSKEY_ACCEPT DK_BTN1_MSK
#define KEY_PASSKEY_REJECT DK_BTN2_MSK

/* BLE 初始化完成信号量，供写线程等待 */
K_SEM_DEFINE(ble_init_ok, 0, 1);

/* 连接对象与广播管理变量 */
static struct bt_conn *current_conn;
static struct bt_conn *auth_conn;
static struct k_work adv_work;

#ifndef CONFIG_BT_EXT_ADV_MAX_ADV_SET
#define CONFIG_BT_EXT_ADV_MAX_ADV_SET 3
#endif
static struct bt_le_ext_adv *ext_adv[CONFIG_BT_EXT_ADV_MAX_ADV_SET];

/* 广播间隔（单位 0.625ms） */
static uint32_t beacon1_adv_interval = 160;
static uint32_t beacon2_adv_interval = 160;

/* 可连接广播与 beacon 的 AD/SD 数据，沿用原 main.c 实现 */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

/* Non-connectable advertising data for Beacon 1 */
static const struct bt_data beacon1_ad_data[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
    BT_DATA_BYTES(BT_DATA_MANUFACTURER_DATA,
                  0xFF, 0xFF,
                  0x01, 0x02, 0x03, 0x04, 0x05, 0x06)};

static const struct bt_data beacon1_sd_data[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, "Harrison_Beacon_1", 17),
};

/* Non-connectable advertising data for Beacon 2 */
static const struct bt_data beacon2_ad_data[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
    BT_DATA_BYTES(BT_DATA_URI,
                  0x17,
                  '/', '/', 'e', 'x', 'a', 'm', 'p', 'l', 'e', '.',
                  'c', 'o', 'm'),
};

static const struct bt_data beacon2_sd_data[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, "Harrison_Beacon_2", 17),
};

/*
 * BLE 核心模块上下文：
 *  - uart_rx_to_ble_fifo: UART RX -> BLE 发送方向的数据 FIFO
 *  - ble_tx_to_uart_fifo: BLE -> UART 发送方向的数据 FIFO
 */
struct my_ble_core_context
{
    struct k_fifo *uart_rx_to_ble_fifo; /* UART -> BLE: 等待通过 BLE 发送的数据 */
    struct k_fifo *ble_tx_to_uart_fifo; /* BLE -> UART: 等待通过 UART 发送的数据 */
};

static struct my_ble_core_context g_ble_ctx;

/* 消息队列定义 */
K_MSGQ_DEFINE(my_ble_msgq, sizeof(MSG_S), 10, 4);

/* 线程数据与栈定义 */
K_THREAD_STACK_DEFINE(my_ble_task_stack, MY_BLE_TASK_STACK_SIZE);
static struct k_thread my_ble_task_data;

/********************************************************************
**函数名称:  my_ble_task
**入口参数:  无
**出口参数:  无
**函数功能:  BLE 模块主线程，处理来自消息队列的任务
**返 回 值:  无
*********************************************************************/
static void my_ble_task(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    MSG_S msg;

    LOG_INF("BLE task thread started");

    for (;;)
    {
        my_recv_msg(&my_ble_msgq, (void *)&msg, sizeof(MSG_S), K_FOREVER);

        switch (msg.msgID)
        {
            /* TODO: 添加 BLE 相关的消息处理逻辑 */
            default:
                break;
        }
    }
}

/********************************************************************
**函数名称:  adv_work_handler
**入口参数:  work     ---        Zephyr 工作队列任务指针
**出口参数:  无
**函数功能:  在连接对象回收后重新启动可连接广播（用于断开重连场景）
**返 回 值:  无
*********************************************************************/
static void adv_work_handler(struct k_work *work)
{
    int err;

    /* Restart the connectable advertising set */
    if (ext_adv[CONNECTABLE_ADV_IDX])
    {
        err = bt_le_ext_adv_start(ext_adv[CONNECTABLE_ADV_IDX],
                                  BT_LE_EXT_ADV_START_DEFAULT);
        if (err)
        {
            LOG_ERR("Failed to restart advertising (err %d)", err);
        }
        else
        {
            LOG_INF("Advertising successfully restarted");
        }
    }
}

/********************************************************************
**函数名称:  adv_connected_cb
**入口参数:  adv      ---        当前广播实例句柄
**          info     ---        广播建立连接时的连接信息指针
**出口参数:  无
**函数功能:  在通过扩展广播建立连接时输出调试日志，便于跟踪连接来源
**返 回 值:  无
*********************************************************************/
static void adv_connected_cb(struct bt_le_ext_adv *adv,
                             struct bt_le_ext_adv_connected_info *info)
{
    LOG_INF("Advertiser[%d] %p connected conn %p",
            bt_le_ext_adv_get_index(adv), adv, info->conn);
}

static const struct bt_le_ext_adv_cb adv_cb = {
    .connected = adv_connected_cb};

/********************************************************************
**函数名称:  advertising_set_create
**入口参数:  adv      ---        输出参数，返回创建好的扩展广播句柄指针
**          param    ---        广播参数配置（是否可连接、间隔等）
**          ad_data  ---        广播数据指针
**          ad_len   ---        广播数据元素个数
**          sd_data  ---        扫描响应数据指针
**          sd_len   ---        扫描响应数据元素个数
**出口参数:  无
**函数功能:  创建并配置一个扩展广播实例，并立即按照默认启动参数开始广播
**返 回 值:  0 表示成功，负值表示失败（如参数非法或底层错误）
*********************************************************************/
static int advertising_set_create(struct bt_le_ext_adv **adv,
                                  const struct bt_le_adv_param *param,
                                  const struct bt_data *ad_data, size_t ad_len,
                                  const struct bt_data *sd_data, size_t sd_len)
{
    int err;
    struct bt_le_ext_adv *adv_set;

    err = bt_le_ext_adv_create(param, &adv_cb, adv);
    if (err)
    {
        return err;
    }

    adv_set = *adv;
    LOG_INF("Created adv: %p", adv_set);

    err = bt_le_ext_adv_set_data(adv_set, ad_data, ad_len, sd_data, sd_len);
    if (err)
    {
        LOG_ERR("Failed to set advertising data (err %d)", err);
        return err;
    }

    return bt_le_ext_adv_start(adv_set, BT_LE_EXT_ADV_START_DEFAULT);
}

/********************************************************************
**函数名称:  beacon1_adv_create
**入口参数:  无
**出口参数:  无
**函数功能:  创建并启动 Beacon1 的不可连接广播（使用厂家自定义数据）
**返 回 值:  0 表示成功，负值表示失败（如广播创建或启动失败）
*********************************************************************/
static int beacon1_adv_create(void)
{
    int err;
    struct bt_le_adv_param param = {
        .options = BT_LE_ADV_OPT_SCANNABLE,
        .interval_min = beacon1_adv_interval,
        .interval_max = beacon1_adv_interval,
        .peer = NULL,
    };

    err = advertising_set_create(&ext_adv[NON_CONNECTABLE_ADV_1_IDX], &param,
                                 beacon1_ad_data, ARRAY_SIZE(beacon1_ad_data),
                                 beacon1_sd_data, ARRAY_SIZE(beacon1_sd_data));
    if (err)
    {
        LOG_ERR("Failed to create beacon 1 advertising set (err %d)", err);
    }

    return err;
}

/********************************************************************
**函数名称:  beacon2_adv_create
**入口参数:  无
**出口参数:  无
**函数功能:  创建并启动 Beacon2 的不可连接广播（使用 URI 数据）
**返 回 值:  0 表示成功，负值表示失败（如广播创建或启动失败）
*********************************************************************/
static int beacon2_adv_create(void)
{
    int err;
    struct bt_le_adv_param param = {
        .options = BT_LE_ADV_OPT_SCANNABLE,
        .interval_min = beacon2_adv_interval,
        .interval_max = beacon2_adv_interval,
        .peer = NULL,
    };

    err = advertising_set_create(&ext_adv[NON_CONNECTABLE_ADV_2_IDX], &param,
                                 beacon2_ad_data, ARRAY_SIZE(beacon2_ad_data),
                                 beacon2_sd_data, ARRAY_SIZE(beacon2_sd_data));
    if (err)
    {
        LOG_ERR("Failed to create beacon 2 advertising set (err %d)", err);
    }

    return err;
}

/********************************************************************
**函数名称:  connectable_adv_create
**入口参数:  无
**出口参数:  无
**函数功能:  创建并启动可连接的 NUS 广播，用于手机等设备建立 BLE 连接
**返 回 值:  0 表示成功，负值表示失败（如广播创建或启动失败）
*********************************************************************/
static int connectable_adv_create(void)
{
    int err;
    struct bt_le_adv_param param = {
        .options = BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_USE_IDENTITY,
        .interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
        .interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
        .peer = NULL,
    };

    err = advertising_set_create(&ext_adv[CONNECTABLE_ADV_IDX], &param,
                                 ad, ARRAY_SIZE(ad),
                                 sd, ARRAY_SIZE(sd));
    if (err)
    {
        LOG_ERR("Failed to create connectable advertising set (err %d)", err);
    }

    return err;
}

/********************************************************************
**函数名称:  advertising_start
**入口参数:  无
**出口参数:  无
**函数功能:  将重新启动可连接广播的工作提交到系统工作队列中异步执行
**返 回 值:  无
*********************************************************************/
static void advertising_start(void)
{
    k_work_submit(&adv_work);
}

/********************************************************************
**函数名称:  connected
**入口参数:  conn     ---        新建立连接的连接句柄
**           err      ---        连接完成状态码（0 表示成功）
**出口参数:  无
**函数功能:  处理 BLE 连接建立事件，保存连接句柄并点亮连接状态 LED
**返 回 值:  无
*********************************************************************/
static void connected(struct bt_conn *conn, uint8_t err)
{
    char addr[BT_ADDR_LE_STR_LEN];

    if (err)
    {
        LOG_ERR("Connection failed, err 0x%02x %s", err, bt_hci_err_to_str(err));
        return;
    }

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    LOG_INF("Connected %s", addr);

    current_conn = bt_conn_ref(conn);

    dk_set_led_on(CON_STATUS_LED);

    /* 通知 Shell 模块蓝牙已连接 */
    my_shell_set_ble_connected(true);
}

/********************************************************************
**函数名称:  disconnected
**入口参数:  conn     ---        断开的连接句柄
**           reason   ---        断开原因码
**出口参数:  无
**函数功能:  处理 BLE 断开事件，释放连接和认证句柄并熄灭连接状态 LED
**返 回 值:  无
*********************************************************************/
static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_INF("Disconnected: %s, reason 0x%02x %s", addr, reason, bt_hci_err_to_str(reason));

    if (auth_conn)
    {
        bt_conn_unref(auth_conn);
        auth_conn = NULL;
    }

    if (current_conn)
    {
        bt_conn_unref(current_conn);
        current_conn = NULL;
        dk_set_led_off(CON_STATUS_LED);

        /* 通知 Shell 模块蓝牙已断开 */
        my_shell_set_ble_connected(false);
    }
}

/********************************************************************
**函数名称:  recycled_cb
**入口参数:  无
**出口参数:  无
**函数功能:  在连接对象完全回收后被调用，用于重新启动可连接广播
**返 回 值:  无
*********************************************************************/
static void recycled_cb(void)
{
    LOG_INF("Connection object available from previous conn. Disconnect is complete!");
    advertising_start();
}

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
/********************************************************************
**函数名称:  security_changed
**入口参数:  conn     ---        当前连接句柄
**          level    ---        更新后的安全级别
**          err      ---        安全更新结果错误码（0 表示成功）
**出口参数:  无
**函数功能:  记录 BLE 连接安全级别变更结果，便于调试和问题定位
**返 回 值:  无
*********************************************************************/
static void security_changed(struct bt_conn *conn, bt_security_t level,
                             enum bt_security_err err)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (!err)
    {
        LOG_INF("Security changed: %s level %u", addr, level);
    }
    else
    {
        LOG_WRN("Security failed: %s level %u err %d %s", addr, level, err,
                bt_security_err_to_str(err));
    }
}
#endif

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
    .recycled = recycled_cb,
#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
    .security_changed = security_changed,
#endif
};

#if defined(CONFIG_BT_NUS_SECURITY_ENABLED)
/********************************************************************
**函数名称:  auth_passkey_display
**入口参数:  conn     ---        当前连接句柄
**          passkey  ---        待显示的配对数字密码
**出口参数:  无
**函数功能:  在配对过程中输出需要在外设侧显示的数字密码
**返 回 值:  无
*********************************************************************/
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_INF("Passkey for %s: %06u", addr, passkey);
}

/********************************************************************
**函数名称:  auth_passkey_confirm
**入口参数:  conn     ---        当前连接句柄
**          passkey  ---        待确认的配对数字密码
**出口参数:  无
**函数功能:  在数值比较配对模式下保存待确认连接句柄并提示用户确认/拒绝
**返 回 值:  无
*********************************************************************/
static void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey)
{
    char addr[BT_ADDR_LE_STR_LEN];

    auth_conn = bt_conn_ref(conn);

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_INF("Passkey for %s: %06u", addr, passkey);

    if (IS_ENABLED(CONFIG_SOC_SERIES_NRF54HX) || IS_ENABLED(CONFIG_SOC_SERIES_NRF54LX))
    {
        LOG_INF("Press Button 0 to confirm, Button 1 to reject.");
    }
    else
    {
        LOG_INF("Press Button 1 to confirm, Button 2 to reject.");
    }
}

/********************************************************************
**函数名称:  auth_cancel
**入口参数:  conn     ---        当前连接句柄
**出口参数:  无
**函数功能:  处理配对取消事件，并输出取消日志信息
**返 回 值:  无
*********************************************************************/
static void auth_cancel(struct bt_conn *conn)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_INF("Pairing cancelled: %s", addr);
}

/********************************************************************
**函数名称:  pairing_complete
**入口参数:  conn     ---        当前连接句柄
**          bonded   ---        是否建立了配对绑定关系
**出口参数:  无
**函数功能:  处理配对完成事件并记录配对结果
**返 回 值:  无
*********************************************************************/
static void pairing_complete(struct bt_conn *conn, bool bonded)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_INF("Pairing completed: %s, bonded: %d", addr, bonded);
}

/********************************************************************
**函数名称:  pairing_failed
**入口参数:  conn     ---        当前连接句柄
**          reason   ---        配对失败原因码
**出口参数:  无
**函数功能:  处理配对失败事件并输出失败原因日志
**返 回 值:  无
*********************************************************************/
static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_INF("Pairing failed conn: %s, reason %d %s", addr, reason,
            bt_security_err_to_str(reason));
}

static struct bt_conn_auth_cb conn_auth_callbacks = {
    .passkey_display = auth_passkey_display,
    .passkey_confirm = auth_passkey_confirm,
    .cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
    .pairing_complete = pairing_complete,
    .pairing_failed = pairing_failed};
#else
static struct bt_conn_auth_cb conn_auth_callbacks;
static struct bt_conn_auth_info_cb conn_auth_info_callbacks;
#endif

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
/********************************************************************
**函数名称:  num_comp_reply
**入口参数:  accept   ---        是否接受当前数值比较配对（true 接受，false 拒绝）
**出口参数:  无
**函数功能:  根据用户按键选择确认或取消数值比较配对，并释放认证连接引用
**返 回 值:  无
*********************************************************************/
static void num_comp_reply(bool accept)
{
    if (accept)
    {
        bt_conn_auth_passkey_confirm(auth_conn);
        LOG_INF("Numeric Match, conn %p", (void *)auth_conn);
    }
    else
    {
        bt_conn_auth_cancel(auth_conn);
        LOG_INF("Numeric Reject, conn %p", (void *)auth_conn);
    }

    bt_conn_unref(auth_conn);
    auth_conn = NULL;
}

/********************************************************************
**函数名称:  my_ble_button_changed
**入口参数:  button_state ---    当前按键状态位图
**            has_changed  ---    本次中断中发生变化的按键位图
**出口参数:  无
**函数功能:  处理 BLE 配对过程中的按键确认/拒绝事件（数值比较确认）
*********************************************************************/
void my_ble_button_changed(uint32_t button_state, uint32_t has_changed)
{
    uint32_t buttons = button_state & has_changed;

    if (auth_conn)
    {
        if (buttons & KEY_PASSKEY_ACCEPT)
        {
            num_comp_reply(true);
        }

        if (buttons & KEY_PASSKEY_REJECT)
        {
            num_comp_reply(false);
        }
    }
}
#endif /* CONFIG_BT_NUS_SECURITY_ENABLED */

/********************************************************************
**函数名称:  bt_receive_cb
**入口参数:  conn     ---        当前 BLE 连接句柄
**          data     ---        收到的数据缓冲区指针
**          len      ---        收到的数据长度
**出口参数:  无
**函数功能:  处理来自对端的 NUS 数据，并转发给 UART 模块发送到串口
**返 回 值:  无
*********************************************************************/
static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data,
                          uint16_t len)
{
    int err;
    char addr[BT_ADDR_LE_STR_LEN] = {0};

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));

    LOG_INF("Received data from: %s", addr);

    err = my_shell_send_from_ble(data, len);
    if (err)
    {
        LOG_WRN("Failed to forward data from BLE to UART (err: %d)", err);
    }
}

static struct bt_nus_cb nus_cb = {
    .received = bt_receive_cb,
};

/********************************************************************
**函数名称:  ble_write_thread
**入口参数:  无
**出口参数:  无
**函数功能:  从 UART→BLE 透传 FIFO 读取数据，组帧后通过 NUS 接口发送到对端
**返 回 值:  无（线程函数，不返回）
*********************************************************************/
static void ble_write_thread(void)
{
    /* 必须等待蓝牙初始化完成 */
    k_sem_take(&ble_init_ok, K_FOREVER);

    LOG_INF("BLE write thread started");

    for (;;)
    {
        /* 从 FIFO 获取 UART 接收到的数据块 */
        struct shell_uart_data_t *buf = k_fifo_get(g_ble_ctx.uart_rx_to_ble_fifo,
                                                   K_FOREVER);

        /*
         * 检查当前是否有活跃的蓝牙连接：
         * - current_conn 是在 connected 回调中赋值的句柄
         */
        if (current_conn)
        {
            int err = bt_nus_send(current_conn, buf->data, buf->len);
            if (err)
            {
                /*
                 * 发送失败的常见原因：
                 * -EBUSY: 缓冲区满
                 * -EMSGSIZE: 数据包超过 MTU (NUS 默认会处理分包，但此处需注意)
                 * -ENOTCONN: 连接已断开
                 */
                LOG_WRN("Failed to send data over BLE connection (err %d, len %d)",
                        err, buf->len);
            }
        }
        else
        {
            LOG_DBG("No active BLE connection, discarding UART data");
        }

        /* 释放由 UART 模块分配的内存 */
        k_free(buf);
    }
}

K_THREAD_DEFINE(ble_write_thread_id, STACKSIZE, ble_write_thread, NULL, NULL,
                NULL, PRIORITY, 0, 0);

/********************************************************************
**函数名称:  my_ble_core_start
**入口参数:  无
**出口参数:  无
**函数功能:  初始化并启动 BLE 协议栈、NUS 服务、连接安全与广播，以及 BLE 写线程
**返 回 值:  0 表示成功，负值表示失败（如协议栈初始化失败等）
*********************************************************************/
int my_ble_core_start(void)
{
    int err;

#if defined(CONFIG_BT_NUS_SECURITY_ENABLED)
    err = bt_conn_auth_cb_register(&conn_auth_callbacks);
    if (err)
    {
        LOG_ERR("Failed to register authorization callbacks. (err: %d)", err);
        return err;
    }

    err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
    if (err)
    {
        LOG_ERR("Failed to register authorization info callbacks. (err: %d)", err);
        return err;
    }
#endif

    err = bt_enable(NULL);
    if (err)
    {
        LOG_ERR("bt_enable failed (err: %d)", err);
        return err;
    }

    LOG_DBG("Bluetooth initialized");

    k_sem_give(&ble_init_ok);

    if (IS_ENABLED(CONFIG_SETTINGS))
    {
        settings_load();
    }

    err = bt_nus_init(&nus_cb);
    if (err)
    {
        LOG_ERR("Failed to initialize UART service (err: %d)", err);
        return err;
    }

    k_work_init(&adv_work, adv_work_handler);

    /* Create and start connectable advertising (NUS) */
    err = connectable_adv_create();
    if (err)
    {
        LOG_ERR("Failed to create connectable advertising");
        return err;
    }

    /* Create and start non-connectable beacon 1 */
    err = beacon1_adv_create();
    if (err)
    {
        LOG_ERR("Failed to create beacon 1 advertising");
        return err;
    }

    /* Create and start non-connectable beacon 2 */
    err = beacon2_adv_create();
    if (err)
    {
        LOG_ERR("Failed to create beacon 2 advertising");
        return err;
    }

    return 0;
}

/********************************************************************
**函数名称:  my_ble_core_init
**入口参数:  param    ---        BLE 核心初始化参数结构体（包含 UART-BLE 透传 FIFO ）
**           tid      ---        指向线程 ID 变量的指针
**出口参数:  tid      ---        存储启动后的线程 ID
**函数功能:  保存 UART 与 BLE 透传 FIFO 指针，初始化消息队列并启动 BLE 任务线程
**返回值:  0 表示成功，负值表示失败（如参数非法等）
*********************************************************************/
int my_ble_core_init(const struct my_ble_core_init_param *param, k_tid_t *tid)
{
    if (param == NULL)
    {
        return -EINVAL;
    }

    /* 保存 UART 与 BLE 透传 FIFO 指针 */
    g_ble_ctx.uart_rx_to_ble_fifo = param->uart_rx_to_ble_fifo;
    g_ble_ctx.ble_tx_to_uart_fifo = param->ble_tx_to_uart_fifo;

    /* 初始化消息队列 */
    my_init_msg_handler(MOD_BLE, &my_ble_msgq);

    /* 启动 BLE 任务线程 */
    *tid = k_thread_create(&my_ble_task_data, my_ble_task_stack,
                           K_THREAD_STACK_SIZEOF(my_ble_task_stack),
                           my_ble_task, NULL, NULL, NULL,
                           MY_BLE_TASK_PRIORITY, 0, K_NO_WAIT);

    /* 设置线程名称 */
    k_thread_name_set(*tid, "MY_BLE");

    return 0;
}