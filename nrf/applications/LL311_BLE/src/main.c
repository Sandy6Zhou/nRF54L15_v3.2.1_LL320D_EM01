/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Nordic UART Bridge Service (NUS) sample
 */
#include "my_comm.h"

DeviceWorkModeConfig g_workmode_config = {0};

#define LOG_MODULE_NAME my_main
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

/* 线程 ID 声明 */
k_tid_t my_main_task_id = NULL;
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
    // TODO

    while (true)
    {
        /* Spin for ever */
        k_sleep(K_MSEC(1000));
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

/*********************************************************************
**函数名称:  get_workmode_config_ptr
**入口参数:  无
**出口参数:  无
**函数功能:  获取工作模式配置结构体的指针
**返 回 值:  DeviceWorkModeConfig* - 指向g_workmode_config的指针（非NULL）
**           该函数返回全局变量的指针，无需手动释放，且指针始终有效
*********************************************************************/
DeviceWorkModeConfig* get_workmode_config_ptr(void)
{
    return &g_workmode_config;
}

/*********************************************************************
**函数名称:  switch_work_mode
**入口参数:  mode     --  要切换到的工作模式
**出口参数:  无
**函数功能:  切换工作模式，通过消息机制通知主线程
*********************************************************************/
void switch_work_mode(MY_WORK_MODE mode)
{
    /* 切换工作模式 */
    g_workmode_config.current_mode = mode;

    my_send_msg(MOD_MAIN, MOD_MAIN, MY_MSG_WORK_MODE_SWITCH);

    LOG_INF("Work mode switch request sent: %d", g_workmode_config.current_mode);
}

/*********************************************************************
**函数名称:  awaken_lte_timer_callback
**入口参数:  timer  --  定时器指针
**出口参数:  无
**函数功能:  LTE唤醒定时器超时回调函数
**           1. 向LTE线程发送上电消息，开启4G电源
**           2. 向主线程发送消息触发重置LTE定时器（用于长续航模式下,下次唤醒）
**返 回 值:  无
*********************************************************************/
void awaken_lte_timer_callback(void *timer)
{
    /* 长续航模式下才需要去重置LTE定时器 */
    if (g_workmode_config.current_mode == MY_MODE_LONG_LIFE)
    {
        my_send_msg(MOD_MAIN, MOD_MAIN, MY_MSG_RESET_LTE_TIMER);
    }

    /* 开启LTE */
    my_send_msg(MOD_MAIN, MOD_LTE, MY_MSG_LTE_PWRON);
}

/*********************************************************************
**函数名称:  handle_long_life_mode
**入口参数:  无
**出口参数:  无
**函数功能:  处理长续航模式（省电模式）
**           1. 关闭GSENSOR传感器以降低功耗
**           2. 向LTE线程发送上电消息，开启4G电源
**返 回 值:  无
*********************************************************************/
void handle_long_life_mode(void)
{
    /* 关闭GSENSOR */
    my_send_msg(MOD_MAIN, MOD_GSENSOR, MY_MSG_GSENSOR_PWROFF);

    /* 开启LTE */
    my_send_msg(MOD_MAIN, MOD_LTE, MY_MSG_LTE_PWRON);
}

/*********************************************************************
**函数名称:  handle_smart_mode
**入口参数:  无
**出口参数:  无
**函数功能:  处理智能模式（自动根据状态切换）,发消息给GSENSOR线程处理以下步骤
**           1. 开启GSENSOR模块
**           2. GSENSOR模块负责初始化配置、检测设备状态（静止/陆地运输/海运）
**           3. 根据GSENSOR状态智能开启LTE并设置间隔唤醒定时器
**返 回 值:  无
*********************************************************************/
void handle_smart_mode(void)
{
    my_send_msg(MOD_MAIN, MOD_GSENSOR, MY_MSG_GSENSOR_PWRON);
}

/*********************************************************************
**函数名称:  handle_continuous_mode
**入口参数:  无
**出口参数:  无
**函数功能:  处理连续模式（实时监控模式）
**           1. 停止LTE间隔唤醒定时器（保持LTE常开）
**           2. 关闭GSENSOR传感器
**           3. 开启LTE模块保持持续连接
**返 回 值:  无
*********************************************************************/
void handle_continuous_mode(void)
{
    my_stop_timer(MY_TIMER_LTE_POWER);

    /* 关闭GSENSOR */
    my_send_msg(MOD_MAIN, MOD_GSENSOR, MY_MSG_GSENSOR_PWROFF);

    /* 开启LTE */
    my_send_msg(MOD_MAIN, MOD_LTE, MY_MSG_LTE_PWRON);
}

/*********************************************************************
**函数名称:  set_reset_lte_timer
**入口参数:  无
**出口参数:  无
**函数功能:  计算并重置LTE间隔唤醒定时器
**           1. 根据配置的上传开始时间和间隔，计算距离下次唤醒的时间
**           2. 增加0-120秒的随机偏移量（防止多设备同时上传造成服务器压力）
**           3. 启动LTE电源定时器，到期后触发 awaken_lte_timer_callback
**返 回 值:  无
*********************************************************************/
void set_reset_lte_timer(void)
{
    int timer_interval;
    int timer_interval_random = 0;
    int ret;
    time_t current_time;

    current_time = my_get_system_time_sec();

    /* timer_interval不可能为0 */ 
    timer_interval = calculate_remaining_seconds(g_workmode_config.long_battery.start_time,
                        g_workmode_config.long_battery.reporting_interval_min, current_time);

    LOG_INF("current_time:%llu,timer_interval:%d", current_time, timer_interval);

    if (timer_interval == -1)
        return;

    ret = rand_0_to_120_seconds(&timer_interval_random);
    if (ret == PSA_SUCCESS)
    {
        timer_interval += timer_interval_random;
    }

    LOG_INF("timer_interval_random:%d", timer_interval_random);

    my_start_timer(MY_TIMER_LTE_POWER, timer_interval * 1000, false, awaken_lte_timer_callback);
}

/********************************************************************
**函数名称:  device_config_init
**入口参数:  p_workmode - 设备工作模式配置结构体指针
**出口参数:  无
**函数功能:  初始化设备工作模式配置，设置各模式默认参数（智能模式和长续航模式）
**          连续追踪模式的配置跟nordic无关,无需配置
**返 回 值:  无
*********************************************************************/
void device_config_init(DeviceWorkModeConfig *p_workmode)
{
    if (p_workmode == NULL)
        return;

    /* 默认设置为智能模式 */ 
    p_workmode->current_mode = MY_MODE_SMART;

    /* 长电池模式配置：设置上传时间间隔和起始时间 */ 
    p_workmode->long_battery.reporting_interval_min = DEFAULT_LONG_LIFE_INTERVAL;   // 上传间隔（分钟）
    strcpy(p_workmode->long_battery.start_time, DEFAULT_START_TIME);                // 每日上传起始时间

    /* 智能模式配置：设置不同状态下的上传时间间隔 */ 
    p_workmode->intelligent.stop_status_interval_sec = STATIC_INTERVAL;             // 静止状态上传间隔（秒）
    p_workmode->intelligent.land_status_interval_sec = LAND_TRANSPORT_INTERVAL;     // 陆运状态上传间隔（秒）
    p_workmode->intelligent.sea_status_interval_sec = SEA_TRANSPORT_INTERVAL;       // 海运状态上传间隔（秒）
    p_workmode->intelligent.sleep_switch = 0;                                       // 休眠开关模式(0默认不休眠)
}

/********************************************************************
**函数名称:  print_app_info
**入口参数:  无
**出口参数:  无
**函数功能:  打印应用信息
**返 回 值:  无
**功能描述:  1. 打印软件版本信息
**           2. 打印蓝牙 MAC 地址
**           3. 打印 IMEI 信息
*********************************************************************/
static void print_app_info(void)
{
    const macaddr_t *mac_addr = my_param_get_macaddr();
    const GsmImei_t *imei = my_param_get_imei();

    LOG_INF("============================================");
    LOG_INF("App Info:");
    LOG_INF("  Version    : %s", SOFTWARE_VERSION);
    LOG_INF("  BLE MAC    : %02X:%02X:%02X:%02X:%02X:%02X",
            mac_addr->hex[0], mac_addr->hex[1], mac_addr->hex[2],
            mac_addr->hex[3], mac_addr->hex[4], mac_addr->hex[5]);
    LOG_INF("  IMEI       : %c%c%c%c%c%c%c%c%c%c%c%c%c%c%c",
            imei->hex[0], imei->hex[1], imei->hex[2], imei->hex[3], imei->hex[4],
            imei->hex[5], imei->hex[6], imei->hex[7], imei->hex[8], imei->hex[9],
            imei->hex[10], imei->hex[11], imei->hex[12], imei->hex[13], imei->hex[14]);
    LOG_INF("============================================");
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
    MSG_S msg;

    my_param_load_config();

    psa_crypto_init();  // PSA库初始化
    device_config_init(&g_workmode_config);

    /* 打印应用信息 */
    print_app_info();

    /* 获取当前线程 ID 并保存 */
    my_main_task_id = k_current_get();

    /* 初始化 Shell 模块 */
    err = my_shell_init();
    if (err)
    {
        LOG_ERR("Failed to initialize Shell module (err %d)", err);
    }

    /* 初始化 BLE 核心模块 */
    struct my_ble_core_init_param ble_param = {
        .reserved = 0,
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

    /* 初始化系统控制模块 (LED, Buzzer, Key) */
    err = my_ctrl_init(&my_ctrl_task_id);
    if (err)
    {
        LOG_ERR("Failed to initialize Control module (err %d)", err);
    }

    /* 初始化自定义任务信息 */
    custom_task_info_init();

    /* 初始化主线程消息队列 */
    my_init_msg_handler(MOD_MAIN, &my_main_msgq);

    /* 主循环：等待并处理消息，逻辑已迁移至各线程 */
    for (;;)
    {
        memset(&msg, 0, sizeof(MSG_S));

        my_recv_msg(&my_main_msgq, (void *)&msg, sizeof(MSG_S), K_FOREVER);

        switch (msg.msgID)
        {
            case MY_MSG_BLE_DATA_EVENT:
                if (msg.pData && msg.DataLen > 0)
                {
                    LOG_INF("BLE Rx (len %d): %s", msg.DataLen, (char *)msg.pData);
                    LOG_HEXDUMP_INF(msg.pData, msg.DataLen, "BLE RAW");
                    MY_FREE_BUFFER(msg.pData);
                }
                break;

            case MY_MSG_NFC_CARD_EVENT:
                LOG_INF("NFC event received, DataLen=%d, expected=%d", msg.DataLen, sizeof(struct nfc_card_info));
                if (msg.pData && msg.DataLen == sizeof(struct nfc_card_info))
                {
                    struct nfc_card_info *card = (struct nfc_card_info *)msg.pData;
                    LOG_INF("NFC Card detected, type: %d", card->type);
                    LOG_INF("NFC Card UID (%d bytes): %02X%02X%02X%02X", card->uid_len, card->uid[0], card->uid[1], card->uid[2], card->uid[3]);
                }
                break;

            case MY_MSG_CTRL_KEY_SHORT_PRESS:
                LOG_INF("KEY EVENT: Short press detected");
                /* 启动 NFC 轮询 */
                my_nfc_start_poll(30);
                break;

            case MY_MSG_CTRL_KEY_LONG_PRESS:
                if (g_workmode_config.current_mode == MY_MODE_SHUTDOWN)
                {
                    /* 关机模式下长按唤醒 */
                    LOG_INF("KEY EVENT: Long press detected in SHUTDOWN mode, waking up...");
                    g_workmode_config.current_mode = MY_MODE_SMART;
                    LOG_INF("System waken up, entering SMART mode");
                    handle_smart_mode();
                }
                else
                {
                    LOG_INF("KEY EVENT: Long press detected (2s)");
                }
                break;

            case MY_MSG_CTRL_LIGHT_SENSOR_DARK:
                LOG_INF("Light sensor detected: DARK");
                break;

            case MY_MSG_CTRL_LIGHT_SENSOR_BRIGHT:
                LOG_INF("Light sensor detected: BRIGHT");
                break;

            case MY_MSG_CTRL_LOCK_PIN_INSERTED:
                LOG_INF("Lock pin detected: INSERTED");
                break;

            case MY_MSG_CTRL_LOCK_PIN_DISCONNECTED:
                LOG_INF("Lock pin detected: DISCONNECTED");
                break;

            case MY_MSG_CTRL_SHUTDOWN_REQUEST:
                LOG_INF("Shutdown request received, entering SHUTDOWN mode");
                /* 切换到关机模式 */
                g_workmode_config.current_mode = MY_MODE_SHUTDOWN;
                LOG_INF("System shutdown complete. Press FUN_KEY for 2s to wakeup.");
                break;

            case MY_MSG_WORK_MODE_SWITCH:
                /* 根据当前切换的模式处理对应的逻辑 */
                switch (g_workmode_config.current_mode)
                {
                    case MY_MODE_LONG_LIFE:
                        LOG_INF("Switched to LONG_LIFE mode");
                        handle_long_life_mode();
                        break;

                    case MY_MODE_SMART:
                        LOG_INF("Switched to SMART mode");
                        handle_smart_mode();
                        break;

                    case MY_MODE_CONTINUOUS:
                        LOG_INF("Switched to CONTINUOUS mode");
                        handle_continuous_mode();
                        break;

                    case MY_MODE_SHUTDOWN:
                        LOG_INF("System is in SHUTDOWN mode (ultra-low power)");
                        /* 关机模式下不执行任何操作 */
                        break;

                    default:
                        LOG_INF("Switched to NORMAL mode");
                        break;
                }
                break;

            case MY_MSG_RESET_LTE_TIMER:
                set_reset_lte_timer();
                break;

            case MY_MSG_DFU_START:
                LOG_INF("DFU start received");
                break;

            case MY_MSG_DFU_TIMEOUT:
                LOG_INF("DFU timeout received");
                break;

            case MY_MSG_DFU_COMPLETE:
                LOG_INF("DFU complete received");
                break;

            default:
                break;
        }
    }

    return 0;
}