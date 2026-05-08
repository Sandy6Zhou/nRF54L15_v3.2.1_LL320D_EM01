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
**                 2. 自定义 GATT 服务接收数据
**                 3. 通过消息队列将收到数据转发至 main 任务
**                 4. 连接状态指示和安全管理
*********************************************************************/

/* 必须在包含 my_comm.h 之前定义 BLE_LOG_MODULE_ID，避免与 my_ble_log.h 中的默认定义冲突 */
#define BLE_LOG_MODULE_ID BLE_LOG_MOD_BLE

#include "my_comm.h"

LOG_MODULE_REGISTER(my_ble_core, LOG_LEVEL_INF);

/* TODO
 * 关于蓝牙广播与连接这里，之前设计时的想法是三路广播，两路不可连接用来做tag，另一路可连接的是与JIMI 蓝牙app连接用，
 * 这样的好处是三路各自独立，不依赖，而且我们自己可连接的那一路广播也可以直接走标准广播而不用走scan response，
 * 如果有需要多余的扩展数据，那scan response用起来可以在广播里携带更多的数据以备扩展，
 * 坏处是，多一路广播，在广播时，广播量更密集，在功耗上会多点损耗，后续可以讨论看看如何取舍，进行优化。
 */

/* 消息队列定义 */
K_MSGQ_DEFINE(my_ble_msgq, sizeof(MSG_S), 10, 4);
/* 线程数据与栈定义 */
K_THREAD_STACK_DEFINE(my_ble_task_stack, MY_BLE_TASK_STACK_SIZE);
/* BLE 初始化完成信号量，供写线程等待 */
K_SEM_DEFINE(ble_init_ok, 0, 1);
static struct k_thread my_ble_task_data;

#define CONNECTABLE_APPLE_ADV_IDX       0
#define CONNECTABLE_GOOGLE_ADV_IDX      1
#define NON_CONNECTABLE_APPLE_ADV_IDX   2
#define NON_CONNECTABLE_GOOGLE_ADV_IDX  3

#define DEV_CUST_UUID                   0xFEE5  // 自定义UUID
#define FLAG_TYPE_VALUE                 0x06    // google的flag值
#define CON_ADV_OBJ_MAX_NUM             2       // 0:GOOGLE, 1:APPLE
#define ADV_INTERVAL                    3200    // 3200*0.625ms=2000ms
#define BLE_NOTIFY_SEND_BUF_MAX_SIZE    1024

typedef struct {
    struct bt_le_ext_adv *handle;
    bool valid;
} ADV_HDL_S;

static struct bt_conn *current_conn;
static struct k_work adv_work;
static struct bt_le_ext_adv *ext_adv[CONFIG_BT_EXT_ADV_MAX_ADV_SET];

ADV_HDL_S con_adv_obj_hdl =
{                  // 可连接广播句柄（APPLE）
    .handle = NULL,
    .valid = true,
};
ADV_HDL_S no_con_adv_obj_hdl[2] = {               // 不可连接广播句柄（GOOGLE/APPLE）
    {.handle = NULL, .valid = true},
    {.handle = NULL, .valid = true}
};

static uint8_t battery_capacity = 100;            // 预留电量
static uint16_t connect_id = 0xff;                // 连接ID
static uint16_t ble_server_rx_index = 0;          // 发送缓存索引
static bool ble_server_send_done = true;          // 发送完成标志
static bool ble_data_send_enable[2] = {false};    // GOOGLE/APPLE发送使能
static uint8_t uart_ble_server_buf[BLE_NOTIFY_SEND_BUF_MAX_SIZE]; // 发送缓存

static void connected(struct bt_conn *conn, uint8_t err);
static void disconnected(struct bt_conn *conn, uint8_t reason);
static void recycled_cb(void);
static void adv_connected_cb(struct bt_le_ext_adv *adv,
                             struct bt_le_ext_adv_connected_info *info);
static int custom_char_write(struct bt_conn *conn,
                                 const struct bt_gatt_attr *attr,
                                 const void *buf, uint16_t len,
                                 uint16_t offset, uint8_t flags);

static void custom_char_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                        uint16_t value);

static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
    .recycled = recycled_cb
};

// 监测MTU变化的回调
static void att_mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx)
{
    if (tx >= CONFIG_BT_L2CAP_TX_MTU)
        ble_server_mtu = CONFIG_BT_L2CAP_TX_MTU;
    else
        ble_server_mtu = tx;

    LOG_INF("ble_server_mtu: %d, rx:%d", ble_server_mtu, rx);
}

static struct bt_gatt_cb gatt_callbacks = {
    .att_mtu_updated = att_mtu_updated, // 监测MTU变化的回调
};

static const struct bt_le_ext_adv_cb adv_cb = {
    .connected = adv_connected_cb
};

/* ========== GATT服务自定义 ========== */
static struct bt_uuid_16 my_service_uuid = BT_UUID_INIT_16(DEV_CUST_UUID);
static struct bt_uuid_16 char_write_uuid = BT_UUID_INIT_16(0xFEB5);
static struct bt_uuid_16 char_notify_uuid = BT_UUID_INIT_16(0xFEB6);

BT_GATT_SERVICE_DEFINE(my_gatt_svc,
    BT_GATT_PRIMARY_SERVICE(&my_service_uuid),
    BT_GATT_CHARACTERISTIC(&char_write_uuid,
                           BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_WRITE,
                           NULL, custom_char_write, NULL),
    BT_GATT_CHARACTERISTIC(&char_notify_uuid,
                           BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           NULL, NULL, NULL),
    BT_GATT_CCC(custom_char_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

/* OTA 状态回调结构体 */
static struct mgmt_callback ota_chunk_callback;
static struct mgmt_callback ota_started_callback;
static struct mgmt_callback ota_stopped_callback;
static struct mgmt_callback ota_pending_callback;

/* 配对密钥（IMEI 后 6 位）*/
static uint32_t ota_passkey = 0;

/********************************************************************
**函数名称:  auth_passkey_entry
**入口参数:  conn     ---   BLE 连接句柄
**出口参数:  无
**函数功能:  输入配对密钥回调
**返 回 值:  0 表示成功
**功能描述:  1. 在需要输入配对密钥时触发
**           2. 自动回复 IMEI 后 6 位作为配对密钥
*********************************************************************/
static int auth_passkey_entry(struct bt_conn *conn)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    LOG_INF("Passkey entry for %s: %06u", addr, ota_passkey);

    /* 自动回复配对密钥 */
    bt_conn_auth_passkey_entry(conn, ota_passkey);

    return 0;
}

/********************************************************************
**函数名称:  auth_passkey_display
**入口参数:  conn     ---   BLE 连接句柄
**           passkey  ---   配对密钥
**出口参数:  无
**函数功能:  显示配对密钥回调
**返 回 值:  无
**功能描述:  1. 在配对过程中显示密钥
**           2. 使用 IMEI 后 6 位作为配对密钥
*********************************************************************/
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    LOG_INF("Passkey for %s: %06u (display only, actual: %06u)", addr, passkey, ota_passkey);
}

/********************************************************************
**函数名称:  auth_cancel
**入口参数:  conn     ---   BLE 连接句柄
**出口参数:  无
**函数功能:  取消配对回调
**返 回 值:  无
**功能描述:  配对取消时触发
*********************************************************************/
static void auth_cancel(struct bt_conn *conn)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    LOG_INF("Pairing cancelled: %s", addr);
}

/* 配对信息回调 - 配对完成通知 */
static void auth_pairing_complete(struct bt_conn *conn, bool bonded)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    LOG_INF("Pairing complete: %s, bonded: %d", addr, bonded);
}

static void auth_pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    LOG_ERR("Pairing failed: %s, reason: %d", addr, reason);
}

static struct bt_conn_auth_info_cb auth_info_cb = {
    .pairing_complete = auth_pairing_complete,
    .pairing_failed = auth_pairing_failed,
};

/* 配对回调结构体 - 设备自动输入静态密钥（IMEI后6位） */
static struct bt_conn_auth_cb auth_cb = {
    .passkey_display = auth_passkey_display,
    .passkey_entry = auth_passkey_entry,
    .cancel = auth_cancel,
};

/********************************************************************
**函数名称:  custom_char_write
**入口参数:  conn     ---        BLE连接句柄
**           attr     ---        GATT属性指针
**           buf      ---        写入数据缓冲区
**           len      ---        数据长度
**           offset   ---        写入偏移量
**           flags    ---        写入标志
**出口参数:  无
**函数功能:  GATT自定义写回调函数，处理客户端写入请求
**返 回 值:  写入长度
*********************************************************************/
static int custom_char_write(struct bt_conn *conn,
                                 const struct bt_gatt_attr *attr,
                                 const void *buf, uint16_t len, uint16_t offset,
                                 uint8_t flags)
{
    ARG_UNUSED(attr);
    ARG_UNUSED(offset);
    ARG_UNUSED(flags);

    LOG_INF("ble received %d bytes data", len);

    /* 数据转发到 BLE APP 层处理（包含解密流程） */
    BLE_DataInputBuffer((uint8_t *)buf, len);

    return len;
}

/********************************************************************
**函数名称:  custom_char_ccc_cfg_changed
**入口参数:  attr     ---        GATT属性指针
**           value    ---        CCC配置值
**出口参数:  无
**函数功能:  GATT通知CCC配置改变回调函数，处理客户端通知使能请求
**返 回 值:  无
*********************************************************************/
static void custom_char_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                        uint16_t value)
{
    LOG_INF("CCC config changed: 0x%04x", value);

    if (value & BT_GATT_CCC_NOTIFY)
    {
        ble_data_send_enable[GOOGLE_ADV_TYPE] = true;
        ble_data_send_enable[APPLE_ADV_TYPE] = true;
    }
    else
    {
        ble_data_send_enable[GOOGLE_ADV_TYPE] = false;
        ble_data_send_enable[APPLE_ADV_TYPE] = false;
    }
}

/********************************************************************
**函数名称:  bt_get_mac_addr
**入口参数:  无
**出口参数:  无
**函数功能:  获取设备蓝牙MAC地址
**返 回 值:  指向MAC地址缓冲区的指针（6字节）
*********************************************************************/
const uint8_t *bt_get_mac_addr(void)
{
    static uint8_t mac[6];
    bt_addr_le_t addrs[CONFIG_BT_ID_MAX];
    size_t count = CONFIG_BT_ID_MAX;

    /* 确保在此之前已经调用过 bt_enable() */
    bt_id_get(addrs, &count);

    /* 默认 identity 是索引 0（BT_ID_DEFAULT） */
    bt_addr_le_t *id_addr = &addrs[0];

    /* 只取 6 字节地址部分 */
    memcpy(mac, id_addr->a.val, 6);

    return mac;
}

/********************************************************************
**函数名称:  ble_set_tx_power
**入口参数:  tx_power ---        发射功率(dBm)，范围: -10 ~ +7(NRF54L15 QFN封装)
**出口参数:  无
**函数功能:  设置蓝牙发射功率
**返 回 值:  0表示成功，负值表示失败
**说    明:  必须在 bt_enable() 之后调用
*********************************************************************/
int ble_set_tx_power(int8_t tx_power)
{
    int err;
    struct net_buf *buf, *rsp = NULL;

    /* 限制功率范围: nRF54L15 支持 -40 ~ +8 dBm */
    if (tx_power < -10)
    {
        tx_power = -10;
    }
    else if (tx_power > 7)
    {
        tx_power = 7;
    }

    /* 使用 Nordic VS HCI 命令设置发射功率
     * Opcode: 0xFC0F (VS_WRITE_TX_POWER_LEVEL)
     * 参数: handle_type(1B) + handle(2B) + tx_power_level(1B)
     * bt_hci_cmd_create(0xFC0F, 4)函数已弃用。替换为bt_hci_cmd_alloc(K_FOREVER)
     */
    buf = bt_hci_cmd_alloc(K_FOREVER);
    if (!buf)
    {
        LOG_ERR("Failed to create HCI command buffer");
        return -ENOMEM;
    }

    /* handle_type = 0 (Advertising), handle = 0, tx_power_level */
    net_buf_add_u8(buf, 0);      /* handle_type: Advertising */
    net_buf_add_le16(buf, 0);    /* handle: 0 */
    net_buf_add_u8(buf, tx_power);  /* tx_power_level */

    err = bt_hci_cmd_send_sync(0xFC0F, buf, &rsp);
    if (err)
    {
        LOG_ERR("Failed to set TX power (err %d)", err);
        return err;
    }

    if (rsp)
    {
        net_buf_unref(rsp);
    }

    LOG_INF("BLE TX power set to %d dBm", tx_power);
    return 0;
}

/********************************************************************
**函数名称:  ble_set_tx_power_by_param
**入口参数:  无
**出口参数:  无
**函数功能:  根据ZMS存储的参数设置蓝牙发射功率
**返 回 值:  0表示成功，负值表示失败
*********************************************************************/
int ble_set_tx_power_by_param(void)
{
    int8_t tx_power = my_param_get_ble_tx_power();
    return ble_set_tx_power(tx_power);
}

/********************************************************************
**函数名称:  get_adv_data
**入口参数:  adv_type ---        广播类型（GOOGLE_ADV_TYPE/APPLE_ADV_TYPE）
**           adv_data  ---        输出：广播数据指针
**           adv_len   ---        输出：广播数据长度
**           scan_data ---        输出：扫描响应数据指针
**           scan_len  ---        输出：扫描响应数据长度
**出口参数:  无
**函数功能:  获取指定类型的广播数据和扫描响应数据
**返 回 值:  无
*********************************************************************/
static void get_adv_data(MY_ADV_TYPE adv_type,
                         struct bt_data **adv_data, uint8_t *adv_len,
                         struct bt_data **scan_data, uint8_t *scan_len)
{
    static struct bt_data advertising_data[2];
    static struct bt_data scan_response_data[3];
    uint8_t adv_idx = 0, scan_idx = 0;
    static uint8_t name_buf[31] = {0};
    static uint8_t uuid_le[2] = {0};
    static uint8_t mac_val_and_bat[8] = {0};
    static uint8_t flags_value;
    uint8_t name_len, name_total_len;

    const char *local_name = bt_get_name();
    const GsmImei_t *gsmImei = my_param_get_imei();
    const uint8_t *edr_addr = bt_get_mac_addr();
    uint16_t uuid_user = DEV_CUST_UUID;

    // scan_response_data---------------------------------------------------------------

    // device name(LL311-XXXXX)
    name_len = strlen(local_name);
    memcpy(name_buf, local_name, name_len);
    name_buf[name_len] = '-';
    memcpy(&name_buf[name_len + 1], &gsmImei->hex[10], 5);  // 使用IMEI后5位作为广播名字的后缀
    name_total_len = name_len + 1 + 5;

    // uuid
    memcpy(uuid_le, &uuid_user, sizeof(uuid_le));

    // mac+batt_percent
    mac_val_and_bat[0] = edr_addr[5];
    mac_val_and_bat[1] = edr_addr[4];
    mac_val_and_bat[2] = edr_addr[3];
    mac_val_and_bat[3] = edr_addr[2];
    mac_val_and_bat[4] = edr_addr[1];
    mac_val_and_bat[5] = edr_addr[0];
    mac_val_and_bat[6] = 0x02;              // 数据类型
    mac_val_and_bat[7] = battery_capacity;  // 电量百分比

    // pack
    scan_response_data[scan_idx].type     = BT_DATA_NAME_SHORTENED;
    scan_response_data[scan_idx].data_len = name_total_len;
    scan_response_data[scan_idx].data     = name_buf;
    scan_idx++;

    scan_response_data[scan_idx].type     = BT_DATA_UUID16_SOME;
    scan_response_data[scan_idx].data_len = sizeof(uuid_le);
    scan_response_data[scan_idx].data     = uuid_le;
    scan_idx++;

    scan_response_data[scan_idx].type     = BT_DATA_UUID32_SOME;
    scan_response_data[scan_idx].data_len = sizeof(mac_val_and_bat);
    scan_response_data[scan_idx].data     = mac_val_and_bat;
    scan_idx++;

    *scan_data = scan_response_data;
    *scan_len  = scan_idx;   /* 扫描响应里有3个元素 */

    // advertising_data-----------------------------------------------------------------
    if (GOOGLE_ADV_TYPE == adv_type)
    {
        flags_value = FLAG_TYPE_VALUE;

        advertising_data[adv_idx].type     = BT_DATA_FLAGS;
        advertising_data[adv_idx].data_len = sizeof(flags_value);
        advertising_data[adv_idx].data     = &flags_value;
        adv_idx++;

        // google类型0x16
        advertising_data[adv_idx].type     = BT_DATA_SVC_DATA16;
        advertising_data[adv_idx].data_len = sizeof(gConfigParam.lic_gg.hex);
        advertising_data[adv_idx].data     = gConfigParam.lic_gg.hex;
        adv_idx++;

    }
    else
    {
        // TODO 预留电量值
        if (battery_capacity >= 80) {
            gConfigParam.lic_ff.hex[6] = 0x20;
        } else if (battery_capacity >= 20) {
            gConfigParam.lic_ff.hex[6] = 0x60;
        } else if (battery_capacity >= 5) {
            gConfigParam.lic_ff.hex[6] = 0xA0;
        } else {
            gConfigParam.lic_ff.hex[6] = 0xE0;
        }

        // ios类型0xff
        advertising_data[adv_idx].type     = BT_DATA_MANUFACTURER_DATA;
        advertising_data[adv_idx].data_len = sizeof(gConfigParam.lic_ff.hex);
        advertising_data[adv_idx].data     = gConfigParam.lic_ff.hex;
        adv_idx++;
    }

    *adv_data = advertising_data;
    *adv_len  = adv_idx;
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

/********************************************************************
**函数名称:  start_adv
**入口参数:  adv_obj_hdl ---    广播对象句柄指针
**           enable     ---    启动/停止标志（true:启动, false:停止）
**出口参数:  无
**函数功能:  启动或停止指定的广播对象
**返 回 值:  无
*********************************************************************/
static void start_adv(ADV_HDL_S *adv_obj_hdl, bool enable)
{
    int err;
    struct bt_data *adv_data = NULL;
    struct bt_data *scan_data = NULL;
    uint8_t adv_len = 0, scan_len = 0;
    struct bt_le_ext_adv_info adv_info;
    bool is_running;
    MY_ADV_TYPE adv_type;
    const char *adv_type_str;

    /* 判断是Google还是Apple */
    if (adv_obj_hdl == &no_con_adv_obj_hdl[GOOGLE_ADV_TYPE])
    {
        adv_type_str = "GOOGLE";
        adv_type = GOOGLE_ADV_TYPE;
    }
    else if (adv_obj_hdl == &no_con_adv_obj_hdl[APPLE_ADV_TYPE])
    {
        adv_type_str = "APPLE";
        adv_type = APPLE_ADV_TYPE;
    }
    else
    {
        adv_type_str = "UNKNOWN";
    }

    LOG_INF("%s called: [%s] enable=%d", __func__, adv_type_str, enable);

    if (!adv_obj_hdl->valid || adv_obj_hdl->handle == NULL)
    {
        LOG_WRN("Adv handle invalid: %p", adv_obj_hdl->handle);
        return;
    }

    // 获取广播句柄此时的广播状态(开启/关闭)
    err = bt_le_ext_adv_get_info(adv_obj_hdl->handle, &adv_info);
    if (err) {
        LOG_ERR("Get adv state fail: %d", err);
        return;
    }

    is_running = (adv_info.ext_adv_state == BT_LE_EXT_ADV_STATE_ENABLED);

    if (enable == is_running) {
        LOG_INF("Adv %p already %s", adv_obj_hdl->handle, enable ? "started" : "stopped");
        return;
    }

    if (enable)
    {
        get_adv_data(adv_type, &adv_data, &adv_len, &scan_data, &scan_len);

        err = bt_le_ext_adv_set_data(adv_obj_hdl->handle, adv_data, adv_len, scan_data, scan_len);
        if (err) {
            LOG_ERR("Set adv data fail: %d", err);
            return;
        }
    }

    if (enable) {
        err = bt_le_ext_adv_start(adv_obj_hdl->handle, BT_LE_EXT_ADV_START_DEFAULT);
    } else {
        err = bt_le_ext_adv_stop(adv_obj_hdl->handle);
    }

    if (err) {
        LOG_ERR("Adv %s fail: %d", enable ? "start" : "stop", err);
    } else {
        LOG_INF("Adv %s success: %p", enable ? "start" : "stop", adv_obj_hdl->handle);
    }
}

/********************************************************************
**函数名称:  set_adv_valid_status
**入口参数:  index    ---        广播类型索引（0:APPLE, 1:GOOGLE）
**           status   ---        广播状态（0:无效, 1:有效）
**出口参数:  无
**函数功能:  设置指定类型广播的有效状态
**返 回 值:  无
*********************************************************************/
void set_adv_valid_status(MY_ADV_TYPE index, int status)
{
    // 无效状态时，先停止广播，再设置无效状态
    if (status == 0)
    {
        start_adv(&no_con_adv_obj_hdl[index], false);
    }
    no_con_adv_obj_hdl[index].valid = status;
}

/********************************************************************
**函数名称:  adv_work_handler
**入口参数:  work      ---        工作结构体指针
**出口参数:  无
**函数功能:  广播工作处理函数，用于延迟启动可连接广播
**返 回 值:  无
*********************************************************************/
static void adv_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    LOG_INF("Restart connectable advertising");
    start_adv(&con_adv_obj_hdl, true);
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
**函数功能:  处理 BLE 连接建立事件，保存连接句柄
**返 回 值:  无
*********************************************************************/
static void connected(struct bt_conn *conn, uint8_t err)
{
    char addr[BT_ADDR_LE_STR_LEN] = {0};

    if (err)
    {
        LOG_ERR("Connection failed, err 0x%02x %s", err, bt_hci_err_to_str(err));
        return;
    }

    // 获取当前对端连接的蓝牙地址
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    LOG_INF("Connected %s", addr);

    current_conn = bt_conn_ref(conn);
    connect_id = bt_conn_index(conn);
    LOG_INF("connect_id %d", connect_id);

    /* 蓝牙连接上后，默认被连接的广播会自动停止*/
    start_adv(&con_adv_obj_hdl, false);

    /* 清除蓝牙日志断开标志，允许日志发送
     * 注意：必须在连接成功后调用，确保日志可以正常发送 */
    ble_log_connect_init();
}

/********************************************************************
**函数名称:  disconnected
**入口参数:  conn     ---        断开的连接句柄
**           reason   ---        断开原因码
**出口参数:  无
**函数功能:  处理 BLE 断开事件，释放连接
**返 回 值:  无
*********************************************************************/
static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    char addr[BT_ADDR_LE_STR_LEN] = {0};

    // 获取当前对端断开的蓝牙地址
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    LOG_INF("Disconnected: %s, reason 0x%02x %s", addr, reason, bt_hci_err_to_str(reason));

    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }

    // 对端断开后，清除连接ID并关闭数据发送标志位
    connect_id = 0xff;
    ble_data_send_enable[GOOGLE_ADV_TYPE] = false;
    ble_data_send_enable[APPLE_ADV_TYPE] = false;

    /* 清除蓝牙日志就绪标志，防止下次连接前发送日志 */
    ble_log_set_ready(false);

    /* 清理日志互斥锁，防止死锁
     * 场景：日志发送任务持有锁时蓝牙断开，其他任务阻塞等待 */
    ble_log_disconnect_cleanup();

    /* 清理BLE单包/分包回复状态，避免断链后busy残留 */
    ble_packet_trans_disconnect_cleanup();
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
    LOG_INF("%s called", __func__);

    if (connect_id == 0xff)
    {
        LOG_INF("Connection object recycled. Restart connectable adv!");

        advertising_start();
    }
}

/********************************************************************
**函数名称:  ble_server_send_notification
**入口参数:  data     ---        要发送的数据缓冲区指针
**           tx_len   ---        数据长度
**出口参数:  无
**函数功能:  BLE服务器发送通知函数，处理GATT通知数据发送
**返 回 值:  无
*********************************************************************
**注意事项:
**           1. 本函数内不允许调用任何可能触发BLE发送的日志函数，避免递归调用
**           2. 禁用MY_LOG_INF、MY_LOG_WRN、MY_LOG_ERR、MY_LOG_DBG
**           3. 日志输出只允许调用LOG_INF、LOG_WRN、LOG_ERR、LOG_DBG、LOG_HEXDUMP_INF
*********************************************************************/
void ble_server_send_notification(uint8_t *data, uint16_t tx_len)
{
    int err;
    uint16_t _tx_len;

    if (connect_id == 0xff || current_conn == NULL)
    {
        LOG_WRN("BLE disconnected, skip send");
        return ;
    }

    if (!ble_server_send_done)
    {
        if (tx_len > 0 && (ble_server_rx_index + tx_len) <= BLE_NOTIFY_SEND_BUF_MAX_SIZE)
        {
            memcpy(&uart_ble_server_buf[ble_server_rx_index], data, tx_len);
            ble_server_rx_index += tx_len;
        }
        LOG_INF("Cache data: total %d, new %d", ble_server_rx_index, tx_len);
        return ;
    }

    ble_server_send_done = false;
    if (tx_len > 0 && (ble_server_rx_index + tx_len) <= BLE_NOTIFY_SEND_BUF_MAX_SIZE)
    {
        memcpy(&uart_ble_server_buf[ble_server_rx_index], data, tx_len);
        ble_server_rx_index += tx_len;
    }

    _tx_len = (ble_server_rx_index > MIN(BLE_SERVER_MAX_DATA_LEN, BLE_SVC_TX_MAX_LEN))
              ? MIN(BLE_SERVER_MAX_DATA_LEN, BLE_SVC_TX_MAX_LEN) : ble_server_rx_index;

    if (ble_data_send_enable[GOOGLE_ADV_TYPE] || ble_data_send_enable[APPLE_ADV_TYPE])
    {
        // LOG_HEXDUMP_INF(uart_ble_server_buf, _tx_len, "bt_gatt_notify:");

        // my_gatt_svc.attrs[4]对应notify特征值
        err = bt_gatt_notify(current_conn, &my_gatt_svc.attrs[4], uart_ble_server_buf, _tx_len);
        if (err)
        {
            LOG_ERR("Notify send fail: %d", err);
            ble_server_send_done = true;
            return ;
        }
    }
    else
    {
        LOG_WRN("BLE send enable not set");
        ble_server_send_done = true;
        return ;
    }

    ble_server_rx_index -= _tx_len;
    memcpy(&uart_ble_server_buf[0], &uart_ble_server_buf[_tx_len], ble_server_rx_index);
    LOG_INF("Send %d bytes, remain %d bytes", _tx_len, ble_server_rx_index);
    ble_server_send_done = true;
}

/********************************************************************
**函数名称:  ble_is_connected
**入口参数:  无
**出口参数:  无
**函数功能:  检查BLE是否已连接
**返 回 值:  true=已连接，false=未连接
*********************************************************************/
bool ble_is_connected(void)
{
    return (connect_id != 0xff && current_conn != NULL);
}

/********************************************************************
**函数名称:  ble_is_data_channel_ready
**入口参数:  无
**出口参数:  无
**函数功能:  检查BLE数据通道是否就绪（连接建立+CCC使能）
**返 回 值:  true=就绪，false=未就绪
**注意事项:  仅当数据通道就绪后才能发送日志，避免在连接初期发送导致崩溃
**           1. 物理连接已建立
**           2. APP已使能CCC通知（0xFEB6特征值）
**           3. 此时才能安全调用bt_gatt_notify发送数据
*********************************************************************/
bool ble_is_data_channel_ready(void)
{
    /* 检查物理连接 */
    if (connect_id == 0xff || current_conn == NULL)
    {
        return false;
    }

    /* 检查CCC通知是否已使能（APP已订阅通知） */
    if (!ble_data_send_enable[GOOGLE_ADV_TYPE] && !ble_data_send_enable[APPLE_ADV_TYPE])
    {
        return false;
    }

    return true;
}

/********************************************************************
**函数名称:  connectable_adv_create
**入口参数:  无
**出口参数:  无
**函数功能:  创建可连接广播对象，用于BLE设备发现和连接
**返 回 值:  0表示成功，负值表示失败
*********************************************************************/
static int connectable_adv_create(void)
{
    int err;
    struct bt_le_adv_param param = {
        .options = BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_USE_IDENTITY,
        .interval_min = ADV_INTERVAL,
        .interval_max = ADV_INTERVAL,
        .peer = NULL,
    };

    err = bt_le_ext_adv_create(&param, &adv_cb, &ext_adv);
    if (err)
    {
        LOG_ERR("Create adv set fail: %d", err);
        return err;
    }
    con_adv_obj_hdl.handle = ext_adv[0];

    return err;
}

/********************************************************************
**函数名称:  non_connectable_adv_create
**入口参数:  无
**出口参数:  无
**函数功能:  创建不可连接广播对象，用于设备发现但不支持连接
**返 回 值:  0表示成功，负值表示失败
*********************************************************************/
static int non_connectable_adv_create(void)
{
    int err, i;
    struct bt_le_adv_param param = {
        .options = BT_LE_ADV_OPT_SCANNABLE | BT_LE_ADV_OPT_USE_IDENTITY,
        .interval_min = ADV_INTERVAL,
        .interval_max = ADV_INTERVAL,
        .peer = NULL,
    };

    for (i = 0; i < CON_ADV_OBJ_MAX_NUM; i++)
    {
        err = bt_le_ext_adv_create(&param, &adv_cb, &ext_adv[1+i]);
        if (err) {
            LOG_ERR("Create adv set fail: %d", err);
            return err;
        }
        no_con_adv_obj_hdl[i].handle = ext_adv[1+i];
    }

    return err;
}

/********************************************************************
**函数名称:  my_ble_updata_adv_param
**入口参数:  ms_data --- 广播间隔时间，单位为毫秒(ms)
**出口参数:  无
**函数功能:  更新BLE非连接广播参数
**返 回 值:  无
*********************************************************************/
void my_ble_updata_adv_param(uint16_t ms_data)
{
    /**
     * 配置新的广告参数
     * - options: 设置为可扫描广告(BT_LE_ADV_OPT_SCANNABLE)和使用身份地址(BT_LE_ADV_OPT_USE_IDENTITY)
     * - interval_min/max: 将毫秒转换为BLE广告间隔单位(0.625ms/单位)
     * - peer: 设置为NULL，表示非定向广告
     */
    struct bt_le_adv_param new_param =
    {
        .options = BT_LE_ADV_OPT_SCANNABLE | BT_LE_ADV_OPT_USE_IDENTITY,
        .interval_min = ms_data/0.625,  // 转换为BLE广告间隔单位(0.625ms)
        .interval_max = ms_data/0.625,  // 转换为BLE广告间隔单位(0.625ms)
        .peer = NULL,                   // 非定向广告
    };

    // 先停止非连接广告,传入参数0表示停止所有非连接广告
    my_no_con_start_adv(0);

    // 更新Apple类型非连接广告的参数,使用新配置的广告参数
    bt_le_ext_adv_update_param(no_con_adv_obj_hdl[APPLE_ADV_TYPE].handle, &new_param);
    // 更新Google类型非连接广告的参数,使用相同的新配置广告参数
    bt_le_ext_adv_update_param(no_con_adv_obj_hdl[GOOGLE_ADV_TYPE].handle, &new_param);

    // 重新启动非连接广告,使用设备命令配置中的tag_sw值
    my_no_con_start_adv(gConfigParam.tag_config.tag_sw);
}

/********************************************************************
**函数名称:  ota_chunk_cb
**入口参数:  event       ---   事件 ID
**           prev_status ---   上一个处理状态
**           rc          ---   返回码指针
**           group       ---   组 ID 指针
**           abort_more  ---   中止标志指针
**           data        ---   事件数据
**           data_size   ---   数据大小
**出口参数:  rc          ---   返回码
**           group       ---   组 ID
**           abort_more  ---   中止标志
**函数功能:  OTA 数据块上传回调
**返 回 值:  MGMT_CB_OK 表示处理成功
**功能描述:  1. 每次镜像数据块上传时触发
**           2. 输出 OTA 上传进度日志
*********************************************************************/
static enum mgmt_cb_return ota_chunk_cb(uint32_t event, enum mgmt_cb_return prev_status,
                                        int32_t *rc, uint16_t *group, bool *abort_more,
                                        void *data, size_t data_size)
{
    ARG_UNUSED(event);
    ARG_UNUSED(prev_status);
    ARG_UNUSED(rc);
    ARG_UNUSED(group);
    ARG_UNUSED(abort_more);
    ARG_UNUSED(data);
    ARG_UNUSED(data_size);

    LOG_INF("OTA: chunk upload in progress");

    return MGMT_CB_OK;
}

/********************************************************************
**函数名称:  ota_started_cb
**入口参数:  event       ---   事件 ID
**           prev_status ---   上一个处理状态
**           rc          ---   返回码指针
**           group       ---   组 ID 指针
**           abort_more  ---   中止标志指针
**           data        ---   事件数据
**           data_size   ---   数据大小
**出口参数:  rc          ---   返回码
**           group       ---   组 ID
**           abort_more  ---   中止标志
**函数功能:  OTA 开始回调
**返 回 值:  MGMT_CB_OK 表示处理成功
**功能描述:  1. DFU 操作开始时触发
**           2. 输出 OTA 开始日志
*********************************************************************/
static enum mgmt_cb_return ota_started_cb(uint32_t event, enum mgmt_cb_return prev_status,
                                          int32_t *rc, uint16_t *group, bool *abort_more,
                                          void *data, size_t data_size)
{
    ARG_UNUSED(event);
    ARG_UNUSED(prev_status);
    ARG_UNUSED(rc);
    ARG_UNUSED(group);
    ARG_UNUSED(abort_more);
    ARG_UNUSED(data);
    ARG_UNUSED(data_size);

    LOG_INF("OTA: DFU started");

    return MGMT_CB_OK;
}

/********************************************************************
**函数名称:  ota_stopped_cb
**入口参数:  event       ---   事件 ID
**           prev_status ---   上一个处理状态
**           rc          ---   返回码指针
**           group       ---   组 ID 指针
**           abort_more  ---   中止标志指针
**           data        ---   事件数据
**           data_size   ---   数据大小
**出口参数:  rc          ---   返回码
**           group       ---   组 ID
**           abort_more  ---   中止标志
**函数功能:  OTA 停止回调
**返 回 值:  MGMT_CB_OK 表示处理成功
**功能描述:  1. DFU 操作停止时触发
**           2. 输出 OTA 停止日志
*********************************************************************/
static enum mgmt_cb_return ota_stopped_cb(uint32_t event, enum mgmt_cb_return prev_status,
                                          int32_t *rc, uint16_t *group, bool *abort_more,
                                          void *data, size_t data_size)
{
    ARG_UNUSED(event);
    ARG_UNUSED(prev_status);
    ARG_UNUSED(rc);
    ARG_UNUSED(group);
    ARG_UNUSED(abort_more);
    ARG_UNUSED(data);
    ARG_UNUSED(data_size);

    LOG_INF("OTA: DFU stopped");

    return MGMT_CB_OK;
}

/********************************************************************
**函数名称:  ota_pending_cb
**入口参数:  event       ---   事件 ID
**           prev_status ---   上一个处理状态
**           rc          ---   返回码指针
**           group       ---   组 ID 指针
**           abort_more  ---   中止标志指针
**           data        ---   事件数据
**           data_size   ---   数据大小
**出口参数:  rc          ---   返回码
**           group       ---   组 ID
**           abort_more  ---   中止标志
**函数功能:  OTA 传输完成回调
**返 回 值:  MGMT_CB_OK 表示处理成功
**功能描述:  1. DFU 传输完成时触发
**           2. 输出 OTA 传输完成日志
*********************************************************************/
static enum mgmt_cb_return ota_pending_cb(uint32_t event, enum mgmt_cb_return prev_status,
                                          int32_t *rc, uint16_t *group, bool *abort_more,
                                          void *data, size_t data_size)
{
    ARG_UNUSED(event);
    ARG_UNUSED(prev_status);
    ARG_UNUSED(rc);
    ARG_UNUSED(group);
    ARG_UNUSED(abort_more);
    ARG_UNUSED(data);
    ARG_UNUSED(data_size);

    LOG_INF("OTA: DFU transfer complete, pending confirmation");

    return MGMT_CB_OK;
}

/********************************************************************
**函数名称:  my_no_con_start_adv
**入口参数:  tag_sw ---   广播控制开关：1-开启广播，0-关闭广播
**出口参数:  无
**函数功能:  同时控制苹果和谷歌两种类型的广播打开和关闭
**返 回 值:  无
*********************************************************************/
void my_no_con_start_adv(uint8_t tag_sw)
{
    if (tag_sw == 1)
    {
        // 开启苹果类型的广播
        start_adv(&no_con_adv_obj_hdl[APPLE_ADV_TYPE], true);
        // 开启谷歌类型的广播
        start_adv(&no_con_adv_obj_hdl[GOOGLE_ADV_TYPE], true);
    }
    else
    {
        // 关闭苹果类型的广播
        start_adv(&no_con_adv_obj_hdl[APPLE_ADV_TYPE], false);
        // 关闭谷歌类型的广播
        start_adv(&no_con_adv_obj_hdl[GOOGLE_ADV_TYPE], false);
    }
}

/********************************************************************
**函数名称:  my_ble_core_start
**入口参数:  无
**出口参数:  无
**函数功能:  初始化并启动 BLE 协议栈、相关广播及自定义 GATT 服务
**返 回 值:  0 表示成功，负值表示失败（如协议栈初始化失败等）
*********************************************************************/
int my_ble_core_start(void)
{
    int err;
    const macaddr_t *my_macaddr;

    // 每次设备启动时，需先获取自定义mac地址，设置完后再开启蓝牙(bt_enable)，自定义mac地址才会生效
    my_macaddr = my_param_get_macaddr();
    bt_ctlr_set_public_addr(my_macaddr->hex);

    err = bt_enable(NULL);
    if (err)
    {
        LOG_ERR("bt_enable failed (err: %d)", err);
        return err;
    }

    LOG_DBG("Bluetooth initialized");

    /* 根据ZMS参数设置蓝牙发射功率 */
    ble_set_tx_power_by_param();

    k_sem_give(&ble_init_ok);

    if (IS_ENABLED(CONFIG_SETTINGS))
    {
        settings_load();
    }

    /* 获取 IMEI 并设置配对密钥为后 6 位 */
    {
        const GsmImei_t *imei = my_param_get_imei();
        char passkey_str[7] = {0};

        /* 提取 IMEI 后 6 位 */
        memcpy(passkey_str, &imei->hex[GSM_IMEI_LENGTH - 6], 6);
        passkey_str[6] = '\0';

        /* 转换为数字并保存配对密钥 */
        ota_passkey = atoi(passkey_str);
        LOG_INF("OTA pairing passkey set to IMEI last 6 digits: %06u", ota_passkey);
    }

    /* 注册配对回调 */
    err = bt_conn_auth_cb_register(&auth_cb);
    if (err)
    {
        LOG_ERR("Failed to register auth callbacks (err %d)", err);
        return err;
    }
    LOG_INF("Pairing callbacks registered");

    /* 注册配对信息回调 */
    bt_conn_auth_info_cb_register(&auth_info_cb);
    LOG_INF("Pairing info callbacks registered");

    /* 注册连接回调 */
    bt_conn_cb_register(&conn_callbacks);

    /* 注册 GATT 回调, 处理 GATT 事件 */
    bt_gatt_cb_register(&gatt_callbacks);

    k_work_init(&adv_work, adv_work_handler);

    /* 注册 OTA 状态监听回调 */
    ota_chunk_callback.callback = ota_chunk_cb;
    ota_chunk_callback.event_id = MGMT_EVT_OP_IMG_MGMT_DFU_CHUNK;
    mgmt_callback_register(&ota_chunk_callback);

    ota_started_callback.callback = ota_started_cb;
    ota_started_callback.event_id = MGMT_EVT_OP_IMG_MGMT_DFU_STARTED;
    mgmt_callback_register(&ota_started_callback);

    ota_stopped_callback.callback = ota_stopped_cb;
    ota_stopped_callback.event_id = MGMT_EVT_OP_IMG_MGMT_DFU_STOPPED;
    mgmt_callback_register(&ota_stopped_callback);

    ota_pending_callback.callback = ota_pending_cb;
    ota_pending_callback.event_id = MGMT_EVT_OP_IMG_MGMT_DFU_PENDING;
    mgmt_callback_register(&ota_pending_callback);

    LOG_INF("OTA status hooks registered");

    /* 初始化 Jimi DFU */
    jimi_dfu_timer_init();
    LOG_INF("Jimi DFU initialized");

    // 创建可连接及不可连接的广播
    err = connectable_adv_create();
    if (err) return err;

    err = non_connectable_adv_create();
    if (err) return err;

    // 初始状态：根据tag_sw判断是否开启不可连接广播
    my_no_con_start_adv(gConfigParam.tag_config.tag_sw);

    start_adv(&con_adv_obj_hdl, true);

    LOG_INF("BLE core start success");
    return 0;
}

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
    int ret;

    LOG_INF("BLE task thread started");

    /* 初始化蓝牙日志模块 */
    ble_log_init();

    /* 初始化BLE单包/分包回复超时重发模块 */
    ble_packet_trans_init();

    /* 初始化扫描模块 */
    my_scan_init();

    for (;;)
    {
        my_recv_msg(&my_ble_msgq, (void *)&msg, sizeof(MSG_S), K_FOREVER);

        switch (msg.msgID)
        {
            /* TODO: 添加 BLE 相关的消息处理逻辑 */
            case MY_MSG_BLE_RX:
                ble_rx_proc_handle();
                break;
            /* 暂时保留测试消息,后续测试验证会频繁使用到 */
            case MY_MSG_TEST:
                ble_comu_at_cmd_handle(shell_test_buff, strlen(shell_test_buff));
                break;

            // 处理BLE+CMD回复指令
            case MY_MSG_BLE_CMD:
                ble_packet_trans_send(msg.pData, msg.DataLen);
                // 释放动态分配的内存
                if(msg.pData != NULL)
                {
                    MY_FREE_BUFFER(msg.pData);
                    msg.pData = NULL;
                }
                break;

            /* 蓝牙开/关锁结果通知 */
            case MY_MSG_BLE_LOCK_RESULT:
                /* 根据数据大小动态使用单包或分包 */
                ble_packet_trans_send(msg.pData, msg.DataLen);
                break;

            /* BLE包传输应答超时处理 */
            case MY_MSG_BLE_PACKET_TIMEOUT:
                ble_packet_trans_timeout_handler();
                break;

            //LTE+CMD数据透传指令
            case MY_MSG_LTE_CMD_RX:
                my_lte_handle_cmd((char *)msg.pData);

                // 释放动态分配的内存
                if(msg.pData != NULL)
                {
                    MY_FREE_BUFFER(msg.pData);
                    msg.pData = NULL;
                }
                break;

            /* 扫描消息处理 */
            case MY_MSG_TAG_SCAN_PROCESS:
            case MY_MSG_SCAN_INTERVAL:
            case MY_MSG_SCAN_LENGTH:
            case MY_MSG_SCAN_UPLOAD:
            case MY_MSG_UPLOAD_WAKEUP:
            case MY_MSG_TRAN_MAC_PROCESS:
                my_scan_msg_handler(&msg);
                break;

            //处理透传mac和tag数据传输（单条上报）
            case MY_MSG_UPLOAD_TAG_AND_MAC:
                tag_mac_scan_upload_data();

                break;
            //异步回复给4G
            case MY_MSG_LTE_CMD_ASYNC_RESP:
                async_match_and_resp((char*)msg.pData);

                // 释放动态分配的内存
                if(msg.pData != NULL)
                {
                    MY_FREE_BUFFER(msg.pData);
                    msg.pData = NULL;
                }
                break;

            case MY_MSG_BLE_NFCTRIG_EVENT:
                if (msg.pData)
                {
                    nfctrig_cmd_t *card = (nfctrig_cmd_t *)msg.pData;
                    //执行NFC联动指令
                    ret = run_nfc_cmd(card->card_id, &card->card_index);
                    if (ret)
                    {
                        //命令匹配成功，具体执行结果看命令响应
                        MY_LOG_INF("Command matched success: cmd_type:%d; command:%s", ret, gConfigParam.nfctrig_config.nfctrig_table.nfctrig_rule[card->card_index].nfctrig_command);
                    }
                    else
                    {
                        MY_LOG_INF("Command not matched or failed to parse: cmd_type:%d; card_id:%s", ret, card->card_id);
                    }

                    // 处理完毕后释放消息数据内存
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
**函数名称:  my_ble_core_init
**入口参数:  param    ---        BLE 核心初始化参数结构体
**           tid      ---        指向线程 ID 变量的指针
**出口参数:  tid      ---        存储启动后的线程 ID
**函数功能:  初始化消息队列并启动 BLE 任务线程
**返回值:  0 表示成功，负值表示失败（如参数非法等）
*********************************************************************/
int my_ble_core_init(const struct my_ble_core_init_param *param, k_tid_t *tid)
{
    if (param == NULL)
    {
        return -EINVAL;
    }

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