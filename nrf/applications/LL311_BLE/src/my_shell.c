/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        my_shell.c
**文件描述:        Shell 命令行交互模块实现（基于 RTT）
**当前版本:        V1.0
**作    者:        Harrison Wu (wuyujiao@jimiiot.com)
**完成日期:        2026.01.22
*********************************************************************
** 功能描述:        注册自定义 Shell 命令，用于系统诊断和设备控制
*********************************************************************/

#include "my_comm.h"

#define LOG_MODULE_NAME my_shell
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

/********************************************************************
**函数名称:  cmd_system_info
**入口参数:  shell   ---        Shell 实例指针
**           argc    ---        参数数量
**           argv    ---        参数数组
**出口参数:  无
**函数功能:  输出系统信息（示例命令）
**返 回 值:  0 表示成功
*********************************************************************/
static int cmd_system_info(const struct shell *shell, size_t argc, char **argv)
{
    shell_print(shell, "=== System Information ===");
    shell_print(shell, "Device: nRF54L15");
    shell_print(shell, "Build Time: %s %s", __DATE__, __TIME__);
    shell_print(shell, "Uptime: %lld ms", k_uptime_get());
    return 0;
}

/********************************************************************
**函数名称:  cmd_ble_info
**入口参数:  shell   ---        Shell 实例指针
**           argc    ---        参数数量
**           argv    ---        参数数组
**出口参数:  无
**函数功能:  输出蓝牙状态信息（示例命令）
**返 回 值:  0 表示成功
*********************************************************************/
static int cmd_ble_info(const struct shell *shell, size_t argc, char **argv)
{
    shell_print(shell, "=== BLE Status ===");
    shell_print(shell, "Device Name: Harrison_UART_Service");
    shell_print(shell, "Advertising: Active");
    return 0;
}

/********************************************************************
**函数名称:  cmd_mem_stat
**入口参数:  shell   ---        Shell 实例指针
**           argc    ---        参数数量
**           argv    ---        参数数组
**出口参数:  无
**函数功能:  输出内存使用统计（示例命令）
**返 回 值:  0 表示成功
*********************************************************************/
static int cmd_mem_stat(const struct shell *shell, size_t argc, char **argv)
{
    shell_print(shell, "=== Memory Statistics ===");
    shell_print(shell, "Heap Size: %d bytes", CONFIG_HEAP_MEM_POOL_SIZE);
    shell_print(shell, "Stack Usage: Check with 'kernel stacks'");
    return 0;
}

/********************************************************************
**函数名称:  cmd_reboot
**入口参数:  shell   ---        Shell 实例指针
**           argc    ---        参数数量
**           argv    ---        参数数组
**出口参数:  无
**函数功能:  系统重启命令
**返 回 值:  0 表示成功
*********************************************************************/
static int cmd_reboot(const struct shell *shell, size_t argc, char **argv)
{
    shell_print(shell, "System rebooting...");
    k_sleep(K_MSEC(500));
    sys_reboot(SYS_REBOOT_WARM);
    return 0;
}

/********************************************************************
**函数名称:  cmd_gsensor_read
**入口参数:  shell   ---        Shell 实例指针
**           argc    ---        参数数量
**           argv    ---        参数数组
**出口参数:  无
**函数功能:  触发 G-Sensor 读取六轴数据
**返 回 值:  0 表示成功
*********************************************************************/
static int cmd_gsensor_read(const struct shell *shell, size_t argc, char **argv)
{
    MSG_S msg;
    msg.msgID = MY_MSG_GSENSOR_READ;
    msg.pData = NULL;
    msg.DataLen = 0;

    my_send_msg_data(MOD_MAIN, MOD_GSENSOR, &msg);
    shell_print(shell, "G-Sensor read command sent");

    return 0;
}

/********************************************************************
**函数名称:  cmd_switch_mode
**入口参数:  sh      ---        Shell 实例指针
**           argc    ---        参数数量
**           argv    ---        参数数组
**出口参数:  无
**函数功能:  切换工作模式（longlife/smart/continuous）
**返 回 值:  0 表示成功，负数表示失败
*********************************************************************/
static int cmd_switch_mode(const struct shell *sh, size_t argc, char **argv)
{
    gsensor_state_t gsensor_state;
    DeviceWorkModeConfig* p_workmode;
    const char *mode_str;
    const char *state_str;

    p_workmode = get_workmode_config_ptr();

    if (argc < 2)
    {
        shell_error(sh, "Missing <mode> argument");
        shell_print(sh, "Usage: app switchmode <mode> <state>");
        shell_print(sh, "  mode  : longlife | smart | continuous");
        shell_print(sh, "  state : still | land | sea (only for smart mode)");
        return -EINVAL;
    }

    mode_str = argv[1];
    state_str = (argc >= 3) ? argv[2] : NULL;

    /* 解析工作模式 */
    if (strcmp(mode_str, "longlife") == 0)
    {
        p_workmode->current_mode = MY_MODE_LONG_LIFE;
        gsensor_state = STATE_UNKNOWN;
    }
    else if (strcmp(mode_str, "smart") == 0)
    {
        p_workmode->current_mode = MY_MODE_SMART;

        /* 智能模式下需要 state 参数 */
        if (state_str == NULL)
        {
            shell_error(sh, "Smart mode requires <state> argument");
            shell_print(sh, "Valid states: still | land | sea");
            return -EINVAL;
        }

        if (strcmp(state_str, "static") == 0)
        {
            gsensor_state = STATE_STATIC;
        }
        else if (strcmp(state_str, "land") == 0)
        {
            gsensor_state = STATE_LAND_TRANSPORT;
        }
        else if (strcmp(state_str, "sea") == 0)
        {
            gsensor_state = STATE_SEA_TRANSPORT;
        }
        else
        {
            shell_error(sh, "Invalid state '%s' for smart mode", state_str);
            shell_print(sh, "Valid states: still | land | sea");
            return -EINVAL;
        }
    }
    else if (strcmp(mode_str, "continuous") == 0)
    {
        p_workmode->current_mode = MY_MODE_CONTINUOUS;
        gsensor_state = STATE_UNKNOWN;
    }
    else
    {
        shell_error(sh, "Invalid mode '%s'", mode_str);
        shell_print(sh, "Valid modes: longlife | smart | continuous");
        return -EINVAL;
    }

    /* 无限等待直到拿到互斥锁 */
    k_mutex_lock(&gsensor_mutex, K_FOREVER);

    g_current_gsensor_state = gsensor_state;

    /* 退出临界区，释放互斥锁 */
    k_mutex_unlock(&gsensor_mutex);

    /* 调用实际的业务函数去切换模式/状态 */
    switch_work_mode(p_workmode->current_mode);

    shell_print(sh, "Switch mode OK:");
    shell_print(sh, "  mode  = %s", mode_str);
    if (p_workmode->current_mode == MY_MODE_SMART)
    {
        shell_print(sh, "  state = %s", state_str);
    }

    return 0;
}

/********************************************************************
**函数名称:  cmd_set_time
**入口参数:  sh      ---        Shell 实例指针
**           argc    ---        参数数量
**           argv    ---        参数数组
**出口参数:  无
**函数功能:  设置系统时间（Unix 时间戳）
**返 回 值:  0 表示成功，负数表示失败
*********************************************************************/
static int cmd_set_time(const struct shell *sh, size_t argc, char **argv)
{
    DeviceWorkModeConfig *p_workmode;
    int ret;

    p_workmode = get_workmode_config_ptr();

    if (argc != 2)
    {
        shell_error(sh, "Usage: app settime <epoch_sec>");
        shell_print(sh, "  <epoch_sec>: seconds since 1970-01-01 00:00:00 UTC");
        return -EINVAL;
    }

    errno = 0;
    unsigned long long epoch = strtoull(argv[1], NULL, 10);
    if (errno != 0)
    {
        shell_error(sh, "Invalid number: %s", argv[1]);
        return -EINVAL;
    }

    ret = my_set_system_time((time_t)epoch);
    if (ret < 0)
    {
        shell_error(sh, "app_set_system_time failed, ret=%d", ret);
        return ret;
    }

    if (p_workmode->current_mode == MY_MODE_LONG_LIFE)
    {
        my_send_msg(MOD_MAIN, MOD_MAIN, MY_MSG_RESET_LTE_TIMER);
    }

    shell_print(sh, "System time set to epoch: %llu", epoch);
    return 0;
}

/********************************************************************
**函数名称:  cmd_get_time
**入口参数:  sh      ---        Shell 实例指针
**           argc    ---        参数数量
**           argv    ---        参数数组
**出口参数:  无
**函数功能:  获取当前系统时间（Unix 时间戳）
**返 回 值:  0 表示成功
*********************************************************************/
static int cmd_get_time(const struct shell *sh, size_t argc, char **argv)
{
    time_t second = my_get_system_time_sec();
    shell_print(sh, "System time set to epoch: %llu", second);
    return 0;
}

/********************************************************************
**函数名称:  cmd_modeset
**入口参数:  sh      ---        Shell 实例指针
**           argc    ---        参数数量
**           argv    ---        参数数组
**出口参数:  无
**函数功能:  配置长续航或智能模式的各项参数
**返 回 值:  0 表示成功，-1 表示失败
*********************************************************************/
static int cmd_modeset(const struct shell *sh, size_t argc, char **argv)
{
    DeviceWorkModeConfig *p_workmode;
    char *endptr;
    uint32_t mode_flag;
    uint32_t report_interval, static_int, land_int, land_distance, sea_int;
    uint8_t sleep_sw;

    p_workmode = get_workmode_config_ptr();

    /* 第一步：解析模式标识（mode_flag），校验是否为有效数字 */
    mode_flag = strtoul(argv[1], &endptr, 10);
    if (*endptr != '\0')
    {
        shell_print(sh, "Error: mode_flag must be a number (1 or 2)!");
        return -1;
    }

    /* 第二步：根据模式标识校验参数个数，并处理对应逻辑 */
    switch (mode_flag)
    {
        case 1: // 长续航模式
        {
            // 校验参数个数（app modeset 1 10 0001 → argc=4）
            if (argc != 4)
            {
                shell_print(sh, "Invalid argument count for mode 1!");
                shell_print(sh, "Usage: app modeset 1 <report_interval> <start_time>");
                shell_print(sh, "  report_interval: 5-1440 minutes (range limit)");
                shell_print(sh, "  start_time    : HHMM (24h format, 0000-2359)");
                return -1;
            }

            // 解析并校验上报间隔（5-1440分钟）
            report_interval = strtoul(argv[2], &endptr, 10);
            if (*endptr != '\0' || report_interval < 5 || report_interval > 1440)
            {
                shell_print(sh, "Error: report_interval must be 5-1440 minutes!");
                return -1;
            }

            // 校验启动时间格式（HHMM，4位数字）
            if (strlen(argv[3]) != 4)
            {
                shell_print(sh, "Error: start_time must be 4 digits (HHMM)!");
                return -1;
            }
            for (int i = 0; i < 4; i++)
            {
                if (!isdigit(argv[3][i]))
                {
                    shell_print(sh, "Error: start_time must be numeric (0000-2359)!");
                    return -1;
                }
            }

            // 设置长续航模式参数
            set_long_battery_params(p_workmode, report_interval, argv[3]);
            shell_print(sh, "Longlife mode config success!");

            break;
        }
        case 2: // 智能模式
        {
            // 校验参数个数（app modeset 2 s l ld se sw → argc=7）
            if (argc != 7)
            {
                shell_print(sh, "Invalid argument count for mode 2!");
                shell_print(sh, "Usage: app modeset 2 <static_int> <land_int> <land_distance> <sea_int> <sleep_sw>");
                shell_print(sh, "  static_int    : Static state threshold (integer)");
                shell_print(sh, "  land_int      : Land transport threshold (integer)");
                shell_print(sh, "  land_distance : Land distance threshold (integer)");
                shell_print(sh, "  sea_int       : Sea transport threshold (integer)");
                shell_print(sh, "  sleep_sw      : Sleep switch (0/1/2)");
                return -1;
            }

            // 解析并校验static_int（正整数）
            static_int = strtoul(argv[2], &endptr, 10);
            if (*endptr != '\0')
            {
                shell_print(sh, "Error: static_int must be an integer!");
                return -1;
            }

            // 解析并校验land_int（正整数）
            land_int = strtoul(argv[3], &endptr, 10);
            if (*endptr != '\0')
            {
                shell_print(sh, "Error: land_int must be an integer!");
                return -1;
            }

            // 解析并校验land_distance（正整数）
            land_distance = strtoul(argv[4], &endptr, 10);
            if (*endptr != '\0')
            {
                shell_print(sh, "Error: land_distance must be an integer!");
                return -1;
            }

            // 解析并校验sea_int（正整数）
            sea_int = strtoul(argv[5], &endptr, 10);
            if (*endptr != '\0')
            {
                shell_print(sh, "Error: sea_int must be an integer!");
                return -1;
            }

            // 解析并校验sleep_sw（仅支持0或1或2）
            sleep_sw = (uint8_t)strtoul(argv[6], &endptr, 10);
            if (*endptr != '\0' || (sleep_sw != 0 && sleep_sw != 1 && sleep_sw != 2))
            {
                shell_print(sh, "Error: sleep_sw must be 0 or 1 or 2!");
                return -1;
            }

            // 设置智能模式参数
            set_intelligent_params(p_workmode, static_int, land_int, sea_int, sleep_sw);
            shell_print(sh, "Sensor mode config success!");
            break;
        }
        default: // 不支持的模式标识
            shell_print(sh, "Error: Unsupported mode_flag! Only 1 (longlife) or 2 (smart) are supported!");
            return -1;
    }

    return 0;
}

/********************************************************************
**函数名称:  my_shell_handle_rx
**入口参数:  pData    ---        接收到的数据缓冲区
**           iLen     ---        数据长度
**出口参数:  无
**函数功能:  处理接收到的字符串，解析并执行命令
**返 回 值:  无
*********************************************************************/
static void my_shell_handle_rx(uint8_t *pData, uint32_t iLen)
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
**函数名称:  shell_at_test
**入口参数:  sh       ---        shell结构体指针
**           argc     ---        参数个数
**           argv     ---        参数数组
**出口参数:  无
**函数功能:  AT测试命令处理函数
**返 回 值:  0表示成功，-EINVAL表示参数错误
*********************************************************************/
static int shell_at_test(const struct shell *sh, size_t argc, char **argv)
{
    uint8_t rx_buff[512] = {0};
    int len;

    if (argc < 2)
    {
        shell_error(sh, "Missing parameter");
        return -EINVAL;
    }

    len = strlen(argv[1]);
    memcpy(rx_buff, argv[1], len);
    // 手动增加\r\n，使得my_shell_handle_rx能识别到
    rx_buff[len++] = '\r';
    rx_buff[len++] = '\n';
    rx_buff[len] = 0;

    shell_print(sh, "param: %s, len: %d", argv[1], len);

    my_shell_handle_rx(rx_buff, len);

    return 0;
}

/********************************************************************
**函数名称:  cmd_nfc_poll
**入口参数:  shell   ---        Shell 实例指针
**           argc    ---        参数数量
**           argv    ---        参数数组
**出口参数:  无
**函数功能:  启动或停止 NFC 轮询（读到卡或超时后自动进入 HPD）
**返 回 值:  0 表示成功
*********************************************************************/
static int cmd_nfc_poll(const struct shell *shell, size_t argc, char **argv)
{
    if (argc < 2)
    {
        shell_print(shell, "Usage: app nfc_poll <start|stop> [timeout_sec]");
        shell_print(shell, "  start [timeout_sec] - Start NFC polling (default 30s, auto HPD after card detected or timeout)");
        shell_print(shell, "  stop                - Stop NFC polling and enter HPD mode");
        return -EINVAL;
    }

    if (strcmp(argv[1], "start") == 0)
    {
        uint32_t timeout_s = 30; /* 默认30秒 */
        if (argc >= 3)
        {
            timeout_s = atoi(argv[2]);
            if (timeout_s == 0)
            {
                timeout_s = 30; /* 如果输入无效，使用默认值 */
            }
        }
        shell_print(shell, "Starting NFC polling for %d seconds...", timeout_s);
        my_nfc_start_poll(timeout_s);
        shell_print(shell, "NFC polling started: %ds timeout, will enter HPD mode when card detected or timeout", timeout_s);
    }
    else if (strcmp(argv[1], "stop") == 0)
    {
        shell_print(shell, "Stopping NFC polling...");
        my_nfc_stop_poll();
        shell_print(shell, "NFC polling stopped, entered HPD mode");
    }
    else
    {
        shell_error(shell, "Invalid parameter: %s", argv[1]);
        shell_print(shell, "Usage: app nfc_poll <start|stop> [timeout_sec]");
        return -EINVAL;
    }

    return 0;
}

// 外部电池状态结构体声明, 用于在 shell 命令中修改电池状态, 调试用
extern Batt_LED_Ctrl_S g_batt_led_ctrl;

/**
 * @brief Shell 命令：设置电池状态
 * 
 * 该函数处理 "app batt_state" 命令，用于手动设置电池状态
 * 
 * @param shell  shell 实例指针
 * @param argc   参数个数
 * @param argv   参数数组
 * 
 * @return 成功返回 0，失败返回 -EINVAL
 */
/********************************************************************
**函数名称:  cmd_batt_stateconst
**入口参数:  shell   ---        Shell 实例指针
**           argc    ---        参数数量
**           argv    ---        参数数组
**出口参数:  无
**函数功能:  命令：设置电池状态，调试用
**返 回 值:  0 表示成功
*********************************************************************/
static int cmd_batt_stateconst(struct shell *shell, size_t argc, char **argv)
{
    if (strcmp(argv[1], "Empty") == 0)
    {
        g_batt_led_ctrl.state = BATT_EMPTY;  // 设置电池状态为空
        shell_print(shell, "battery state: Empty");  // 输出状态信息
    }
    else if (strcmp(argv[1], "Low") == 0)
    {
        g_batt_led_ctrl.state = BATT_LOW;  // 设置电池状态为低
        shell_print(shell, "battery state: Low");  // 输出状态信息
    }
    else if (strcmp(argv[1], "Normal") == 0)
    {
        g_batt_led_ctrl.state = BATT_NORMAL;  // 设置电池状态为正常
        shell_print(shell, "battery state: Normal");  // 输出状态信息
    }
    else if (strcmp(argv[1], "Fair") == 0)
    {
        g_batt_led_ctrl.state = BATT_FAIR;  // 设置电池状态为良好
        shell_print(shell, "battery state: Fair");  // 输出状态信息
    }
    else if (strcmp(argv[1], "High") == 0)
    {
        g_batt_led_ctrl.state = BATT_HIGH;  // 设置电池状态为高
        shell_print(shell, "battery state: High");  // 输出状态信息
    }
    else if (strcmp(argv[1], "Full") == 0)
    {
        g_batt_led_ctrl.state = BATT_FULL;  // 设置电池状态为满
        shell_print(shell, "battery state: Full");  // 输出状态信息
    }
    else
    {
        shell_error(shell, "Invalid parameter: %s", argv[1]);  // 输出错误信息
        shell_print(shell, "Usage: app batt_state_change <Empty|Low|Normal|Fair|High|Full>");  // 输出使用方法
        return -EINVAL;  // 返回错误代码
    }

    return 0;
}

/********************************************************************
**函数名称：cmd_shutdown
**入口参数：shell   ---        Shell 实例指针
**           argc    ---        参数数量
**           argv    ---        参数数组
**出口参数：无
**函数功能：执行系统关机（进入超低功耗模式，仅按键可唤醒）
**返 回 值：0 表示成功
*********************************************************************/
static int cmd_shutdown(const struct shell *shell, size_t argc, char **argv)
{
    MSG_S msg;

    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    shell_print(shell, "System shutdown request...");
    shell_print(shell, "Entering SHUTDOWN mode (ultra-low power, only KEY can wakeup)");

    /* 发送关机请求到主任务 */
    msg.msgID = MY_MSG_CTRL_SHUTDOWN_REQUEST;
    msg.pData = NULL;
    msg.DataLen = 0;
    my_send_msg_data(MOD_MAIN, MOD_MAIN, &msg);

    return 0;
}

/********************************************************************
**函数名称:  cmd_ble_log_test
**入口参数:  shell    ---        Shell 句柄
**           argc     ---        参数个数
**           argv     ---        参数数组
**出口参数:  无
**函数功能:  测试蓝牙日志发送功能
**返 回 值:  0 表示成功
**使用示例:  app blog "test message"
*********************************************************************/
static int cmd_ble_log_test(const struct shell *shell, size_t argc, char **argv)
{
    const char *msg;
    size_t len;
    uint8_t send_len;

    if (argc < 2)
    {
        shell_print(shell, "Usage: app blog \"<message>\"");
        shell_print(shell, "Example: app blog \"Hello BLE Log\"");
        shell_print(shell, "Note: Message must be enclosed in quotes");
        return -1;
    }

    msg = argv[1];
    len = strlen(msg);

    /* 检查参数是否包含空格（带引号的参数在argc=2时是一个整体）
     * 如果argc>2，说明参数被空格分割，用户可能忘记加引号 */
    if (argc > 2)
    {
        shell_print(shell, "Error: Message must be enclosed in quotes");
        shell_print(shell, "Usage: app blog \"<message>\"");
        return -1;
    }

    /* 检查长度限制 */
    if (len == 0)
    {
        shell_print(shell, "Message is empty");
        return -1;
    }

    if (len > 512)
    {
        shell_print(shell, "Message too long (max 512 bytes)");
        return -1;
    }

    /* 限制发送长度为 255 字节（ble_log_send 参数类型为 uint8_t） */
    send_len = (len > 255) ? 255 : (uint8_t)len;

    shell_print(shell, "Sending BLE log: %s", msg);
    ble_log_send((uint8_t *)msg, send_len);
    shell_print(shell, "BLE log sent, length: %d", send_len);

    return 0;
}

/********************************************************************
**函数名称:  cmd_ble_log_config
**入口参数:  shell    ---        Shell 句柄
**           argc     ---        参数个数
**           argv     ---        参数数组
**出口参数:  无
**函数功能:  蓝牙日志配置命令
**返 回 值:  0 表示成功
**使用示例:  app blogcfg global 1          (开启总开关)
**           app blogcfg mod BLE 1         (开启BLE模块)
**           app blogcfg level BLE 3       (BLE模块INF等级)
**           app blogcfg show              (显示配置)
*********************************************************************/
static int cmd_ble_log_config(const struct shell *shell, size_t argc, char **argv)
{
    BleLogConfig_t *config;
    int ret;
    uint8_t level;
    uint8_t mod_id;
    uint8_t en;

    if (argc < 2)
    {
        shell_print(shell, "Usage:");
        shell_print(shell, "  app blogcfg global <0|1>        - Set global enable");
        shell_print(shell, "  app blogcfg mod <name> <0|1>    - Set module enable");
        shell_print(shell, "  app blogcfg level <name> <0-4>  - Set module level");
        shell_print(shell, "  app blogcfg show                - Show configuration");
        shell_print(shell, "Module names:");
        shell_print(shell, "  MAIN, BLE, DFU, SENSOR, LTE, CTRL, SHELL, NFC,");
        shell_print(shell, "  BATTERY, MOTOR, CMD, TOOL, PARAM, WDT, OTHER");
        shell_print(shell, "Level: 0=NONE, 1=ERR, 2=WRN, 3=INF, 4=DBG");
        return -1;
    }

    config = my_param_get_ble_log_config();

    if (strcmp(argv[1], "global") == 0)
    {
        if (argc < 3)
        {
            shell_print(shell, "Current global enable: %d", config->global_en);
            return 0;
        }

        en = atoi(argv[2]) ? 1 : 0; // 支持非0为1，否则为0

        ret = my_param_set_ble_log_global(en);  // 保存全局使能参数
        if (ret == 0)
        {
            shell_print(shell, "BLE log global enable set to: %d", en);
        }
        else
        {
            shell_print(shell, "Failed to set global enable");
        }
    }
    else if (strcmp(argv[1], "mod") == 0)
    {
        if (argc < 4)
        {
            shell_print(shell, "Usage: app blogcfg mod <name> <0|1>");
            return -1;
        }

        en = atoi(argv[3]) ? 1 : 0; // 支持非0为1，否则为0

        if (strcmp(argv[2], "MAIN") == 0)
            mod_id = BLE_LOG_MOD_MAIN;
        else if (strcmp(argv[2], "BLE") == 0) // BLE 模块不支持 BLE 日志（递归风险），初始化时已禁用
        {
            shell_print(shell, "BLE module does not support BLE log (recursive risk)");
            return -1;
        }
        else if (strcmp(argv[2], "DFU") == 0) // DFU 模块不支持 BLE 日志（递归风险），初始化时已禁用
        {
            shell_print(shell, "DFU module does not support BLE log (recursive risk)");
            return -1;
        }
        else if (strcmp(argv[2], "SENSOR") == 0)
            mod_id = BLE_LOG_MOD_SENSOR;
        else if (strcmp(argv[2], "LTE") == 0)
            mod_id = BLE_LOG_MOD_LTE;
        else if (strcmp(argv[2], "CTRL") == 0)
            mod_id = BLE_LOG_MOD_CTRL;
        else if (strcmp(argv[2], "SHELL") == 0) // SHELL 模块不支持 BLE 日志（递归风险），初始化时已禁用
        {
            shell_print(shell, "SHELL module does not support BLE log (recursive risk)");
            return -1;
        }
        else if (strcmp(argv[2], "NFC") == 0)
            mod_id = BLE_LOG_MOD_NFC;
        else if (strcmp(argv[2], "BATTERY") == 0)
            mod_id = BLE_LOG_MOD_BATTERY;
        else if (strcmp(argv[2], "MOTOR") == 0)
            mod_id = BLE_LOG_MOD_MOTOR;
        else if (strcmp(argv[2], "CMD") == 0) // CMD 模块不支持 BLE 日志（递归风险），初始化时已禁用
        {
            shell_print(shell, "CMD module does not support BLE log (recursive risk)");
            return -1;
        }
        else if (strcmp(argv[2], "TOOL") == 0)
            mod_id = BLE_LOG_MOD_TOOL;
        else if (strcmp(argv[2], "PARAM") == 0)
            mod_id = BLE_LOG_MOD_PARAM;
        else if (strcmp(argv[2], "WDT") == 0)
            mod_id = BLE_LOG_MOD_WDT;
        else if (strcmp(argv[2], "OTHER") == 0)
            mod_id = BLE_LOG_MOD_OTHER;
        else
        {
            shell_print(shell, "Unknown module: %s", argv[2]);
            return -1;
        }

        ret = my_param_set_ble_log_mod(mod_id, en);
        if (ret == 0)
        {
            shell_print(shell, "BLE log module %s enable set to: %d", argv[2], en);
        }
        else
        {
            shell_print(shell, "Failed to set module enable");
        }
    }
    else if (strcmp(argv[1], "level") == 0)
    {
        if (argc < 4)
        {
            shell_print(shell, "Usage: app blogcfg level <name> <0-4>");
            return -1;
        }

        level = atoi(argv[3]);

        if (strcmp(argv[2], "MAIN") == 0)
            mod_id = BLE_LOG_MOD_MAIN;
        else if (strcmp(argv[2], "BLE") == 0) // BLE 模块不支持 BLE 日志（递归风险），初始化时已禁用
        {
            shell_print(shell, "BLE module does not support BLE log (recursive risk)");
            return -1;
        }
        else if (strcmp(argv[2], "DFU") == 0) // DFU 模块不支持 BLE 日志（递归风险），初始化时已禁用
        {
            shell_print(shell, "DFU module does not support BLE log (recursive risk)");
            return -1;
        }
        else if (strcmp(argv[2], "SENSOR") == 0)
            mod_id = BLE_LOG_MOD_SENSOR;
        else if (strcmp(argv[2], "LTE") == 0)
            mod_id = BLE_LOG_MOD_LTE;
        else if (strcmp(argv[2], "CTRL") == 0)
            mod_id = BLE_LOG_MOD_CTRL;
        else if (strcmp(argv[2], "SHELL") == 0) // SHELL 模块不支持 BLE 日志（递归风险），初始化时已禁用
        {
            shell_print(shell, "SHELL module does not support BLE log (recursive risk)");
            return -1;
        }
        else if (strcmp(argv[2], "NFC") == 0)
            mod_id = BLE_LOG_MOD_NFC;
        else if (strcmp(argv[2], "BATTERY") == 0)
            mod_id = BLE_LOG_MOD_BATTERY;
        else if (strcmp(argv[2], "MOTOR") == 0)
            mod_id = BLE_LOG_MOD_MOTOR;
        else if (strcmp(argv[2], "CMD") == 0) // CMD 模块不支持 BLE 日志（递归风险），初始化时已禁用
        {
            shell_print(shell, "CMD module does not support BLE log (recursive risk)");
            return -1;
        }
        else if (strcmp(argv[2], "TOOL") == 0)
            mod_id = BLE_LOG_MOD_TOOL;
        else if (strcmp(argv[2], "PARAM") == 0)
            mod_id = BLE_LOG_MOD_PARAM;
        else if (strcmp(argv[2], "WDT") == 0)
            mod_id = BLE_LOG_MOD_WDT;
        else if (strcmp(argv[2], "OTHER") == 0)
            mod_id = BLE_LOG_MOD_OTHER;
        else
        {
            shell_print(shell, "Unknown module: %s", argv[2]);
            return -1;
        }

        ret = my_param_set_ble_log_level(mod_id, level);
        if (ret == 0)
        {
            shell_print(shell, "BLE log module %s level set to: %d", argv[2], level);
        }
        else
        {
            shell_print(shell, "Failed to set module level");
        }
    }
    else if (strcmp(argv[1], "show") == 0)
    {
        shell_print(shell, "BLE Log Configuration:");
        shell_print(shell, "  Global enable: %d", config->global_en);
        shell_print(shell, "  Module status (ON/OFF + level(0:NONE 1:ERR 2:WRN 3:INF 4:DBG)):");
        shell_print(shell, "    MAIN:   %s  %d",
                    BLE_LOG_MOD_IS_ENABLED(config, BLE_LOG_MOD_MAIN) ? "ON" : "OFF",
                    config->mod_level[BLE_LOG_MOD_MAIN]);
        shell_print(shell, "    BLE:    %s  %d",
                    BLE_LOG_MOD_IS_ENABLED(config, BLE_LOG_MOD_BLE) ? "ON" : "OFF",
                    config->mod_level[BLE_LOG_MOD_BLE]);
        shell_print(shell, "    DFU:    %s  %d",
                    BLE_LOG_MOD_IS_ENABLED(config, BLE_LOG_MOD_DFU) ? "ON" : "OFF",
                    config->mod_level[BLE_LOG_MOD_DFU]);
        shell_print(shell, "    SENSOR: %s  %d",
                    BLE_LOG_MOD_IS_ENABLED(config, BLE_LOG_MOD_SENSOR) ? "ON" : "OFF",
                    config->mod_level[BLE_LOG_MOD_SENSOR]);
        shell_print(shell, "    LTE:    %s  %d",
                    BLE_LOG_MOD_IS_ENABLED(config, BLE_LOG_MOD_LTE) ? "ON" : "OFF",
                    config->mod_level[BLE_LOG_MOD_LTE]);
        shell_print(shell, "    CTRL:   %s  %d",
                    BLE_LOG_MOD_IS_ENABLED(config, BLE_LOG_MOD_CTRL) ? "ON" : "OFF",
                    config->mod_level[BLE_LOG_MOD_CTRL]);
        shell_print(shell, "    SHELL:  %s  %d",
                    BLE_LOG_MOD_IS_ENABLED(config, BLE_LOG_MOD_SHELL) ? "ON" : "OFF",
                    config->mod_level[BLE_LOG_MOD_SHELL]);
        shell_print(shell, "    NFC:    %s  %d",
                    BLE_LOG_MOD_IS_ENABLED(config, BLE_LOG_MOD_NFC) ? "ON" : "OFF",
                    config->mod_level[BLE_LOG_MOD_NFC]);
        shell_print(shell, "    BATTERY: %s  %d",
                    BLE_LOG_MOD_IS_ENABLED(config, BLE_LOG_MOD_BATTERY) ? "ON" : "OFF",
                    config->mod_level[BLE_LOG_MOD_BATTERY]);
        shell_print(shell, "    MOTOR:  %s  %d",
                    BLE_LOG_MOD_IS_ENABLED(config, BLE_LOG_MOD_MOTOR) ? "ON" : "OFF",
                    config->mod_level[BLE_LOG_MOD_MOTOR]);
        shell_print(shell, "    CMD:    %s  %d",
                    BLE_LOG_MOD_IS_ENABLED(config, BLE_LOG_MOD_CMD) ? "ON" : "OFF",
                    config->mod_level[BLE_LOG_MOD_CMD]);
        shell_print(shell, "    TOOL:   %s  %d",
                    BLE_LOG_MOD_IS_ENABLED(config, BLE_LOG_MOD_TOOL) ? "ON" : "OFF",
                    config->mod_level[BLE_LOG_MOD_TOOL]);
        shell_print(shell, "    PARAM:  %s  %d",
                    BLE_LOG_MOD_IS_ENABLED(config, BLE_LOG_MOD_PARAM) ? "ON" : "OFF",
                    config->mod_level[BLE_LOG_MOD_PARAM]);
        shell_print(shell, "    WDT:    %s  %d",
                    BLE_LOG_MOD_IS_ENABLED(config, BLE_LOG_MOD_WDT) ? "ON" : "OFF",
                    config->mod_level[BLE_LOG_MOD_WDT]);
        shell_print(shell, "    OTHER:  %s  %d",
                    BLE_LOG_MOD_IS_ENABLED(config, BLE_LOG_MOD_OTHER) ? "ON" : "OFF",
                    config->mod_level[BLE_LOG_MOD_OTHER]);
    }
    else
    {
        shell_print(shell, "Unknown command: %s", argv[1]);
        return -1;
    }

    return 0;
}

/* 注册自定义命令到 Shell 子系统 */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_app,
    SHELL_CMD(sysinfo, NULL, "Display system information", cmd_system_info),
    SHELL_CMD(bleinfo, NULL, "Display BLE status", cmd_ble_info),
    SHELL_CMD(memstat, NULL, "Display memory statistics", cmd_mem_stat),
    SHELL_CMD(gsensor, NULL, "Read G-Sensor 6-axis data", cmd_gsensor_read),
    SHELL_CMD(reboot, NULL, "Reboot system", cmd_reboot),
    SHELL_CMD(switchmode, NULL, "Switch work mode", cmd_switch_mode),
    SHELL_CMD(settime, NULL, "settime unix seconds ", cmd_set_time),
    SHELL_CMD(gettime, NULL, "gettime unix seconds", cmd_get_time),
    SHELL_CMD(modeset, NULL, "Configure longlife or smart mode parameters", cmd_modeset),
    SHELL_CMD(AT_TEST, NULL, "Usage:app AT_TEST \"TEST xxxx(AT^GT_CM=xxxx)\"", shell_at_test),
    SHELL_CMD(nfc_poll, NULL, "NFC polling: app nfc_poll <start|stop>", cmd_nfc_poll),
    SHELL_CMD(shutdown, NULL, "Shutdown system (enter ultra-low power mode)", cmd_shutdown),
    SHELL_CMD(batt_state_change, NULL, "battery state change", cmd_batt_stateconst),//手动改变电池状态，调试用
    SHELL_CMD(blog, NULL, "Send BLE log test message: app blog <message>", cmd_ble_log_test),
    SHELL_CMD(blogcfg, NULL, "BLE log config: app blogcfg <global|mod|level|show>", cmd_ble_log_config),
    SHELL_SUBCMD_SET_END
);
/* Zephyr Shell 子系统提供的宏，随 nRF Connect SDK一起提供，用来在 Shell里注册一个“根命令”
 * 这个宏在头文件zephyr/shell/shell.h里定义，是Zephyr的Shell API的一部分
 */
SHELL_CMD_REGISTER(app, &sub_app, "Application commands", NULL);

/********************************************************************
**函数名称:  my_shell_init
**入口参数:  无
**出口参数:  无
**函数功能:  初始化 Shell 模块（Zephyr Shell 自动初始化，此处仅做日志输出）
**返 回 值:  0 表示成功
*********************************************************************/
int my_shell_init(void)
{
    LOG_INF("Shell module initialized (RTT backend)");
    LOG_INF("Use 'app sysinfo', 'app bleinfo', etc. to interact");
    return 0;
}