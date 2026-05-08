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

/* 必须在包含 my_comm.h 之前定义 BLE_LOG_MODULE_ID，避免与 my_ble_log.h 中的默认定义冲突 */
#define BLE_LOG_MODULE_ID BLE_LOG_MOD_SHELL

#include "my_comm.h"

#define LOG_MODULE_NAME my_shell
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

uint8_t shell_test_buff[256] = {0};

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
    my_send_msg(MOD_MAIN, MOD_GSENSOR, MY_MSG_GSENSOR_READ);
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
    MY_WORK_MODE current_workmode;
    const char *mode_str;
    const char *state_str;

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
        current_workmode = MY_MODE_LONG_LIFE;
        gsensor_state = STATE_UNKNOWN;
    }
    else if (strcmp(mode_str, "smart") == 0)
    {
        current_workmode = MY_MODE_SMART;

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
        current_workmode = MY_MODE_CONTINUOUS;
        gsensor_state = STATE_UNKNOWN;
    }
    else
    {
        shell_error(sh, "Invalid mode '%s'", mode_str);
        shell_print(sh, "Valid modes: longlife | smart | continuous");
        return -EINVAL;
    }

    g_gsensor_runtime_ctx.current_gsensor_state = gsensor_state;

    /* 调用实际的业务函数去切换模式/状态 */
    switch_work_mode(current_workmode);
    if (current_workmode == MY_MODE_SMART)
    {
        //TODO 测试调用，后续补充完gsensor完删掉此处
        my_send_msg(MOD_GSENSOR, MOD_GSENSOR, MY_MSG_MODESET_UPDATE);
    }

    shell_print(sh, "Switch mode OK:");
    shell_print(sh, "  mode  = %s", mode_str);
    if (current_workmode == MY_MODE_SMART)
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
    int ret;

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

    if (gConfigParam.device_workmode_config.workmode_config.current_mode == MY_MODE_LONG_LIFE)
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

    p_workmode = &gConfigParam.device_workmode_config.workmode_config;

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
            set_intelligent_params(p_workmode, static_int, land_int, land_distance, sea_int, sleep_sw);
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
    int len;

    if (argc < 2)
    {
        shell_error(sh, "Missing parameter");
        return -EINVAL;
    }

    memset(shell_test_buff, 0, sizeof(shell_test_buff));

    len = strlen(argv[1]);
    memcpy(shell_test_buff, argv[1], len);
    // 手动增加\r\n，使得my_shell_handle_rx能识别到
    shell_test_buff[len++] = '\r';
    shell_test_buff[len++] = '\n';
    shell_test_buff[len] = 0;

    shell_print(sh, "param: %s, len: %d", argv[1], len);

    my_shell_handle_rx(shell_test_buff, len);

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

/********************************************************************
**函数名称:  cmd_ble_test
**入口参数:  sh    ---        Shell句柄，用于输出信息
            argc  ---        参数个数
            argv  ---        参数数组，argv[1]为测试参数字符串
**出口参数:  无
**函数功能:  处理BLE测试命令，接收参数并发送测试消息到BLE模块
**返 回 值:  0表示成功，-EINVAL表示参数错误
*********************************************************************/
static int cmd_ble_test(const struct shell *sh, size_t argc, char **argv)
{
    int len;

    if (argc < 2)
    {
        shell_error(sh, "Missing parameter");
        return -EINVAL;
    }

    memset(shell_test_buff, 0, sizeof(shell_test_buff));

    len = strlen(argv[1]);
    memcpy(shell_test_buff, argv[1], len);
    shell_test_buff[len] = 0;

    shell_print(sh, "param: %s, len: %d", argv[1], len);

    my_send_msg(MOD_MAIN, MOD_BLE, MY_MSG_TEST);

    return 0;
}

/********************************************************************
**函数名称:  cmd_buzzer_test
**入口参数:  sh    ---        Shell句柄，用于输出信息
            argc  ---        参数个数
            argv  ---        参数数组，argv[1]为测试参数字符串
**出口参数:  无
**函数功能:  处理Buzzer测试命令，接收参数并发送测试消息到Buzzer模块
**返 回 值:  0表示成功
*********************************************************************/
static int cmd_buzzer_test(const struct shell *sh, size_t argc, char **argv)
{
    int len;

    if (argc < 2)
    {
        shell_error(sh, "Missing parameter");
        return -EINVAL;
    }

    memset(shell_test_buff, 0, sizeof(shell_test_buff));

    len = strlen(argv[1]);
    memcpy(shell_test_buff, argv[1], len);
    shell_test_buff[len] = 0;

    shell_print(sh, "param: %s, len: %d", argv[1], len);
    my_set_buzzer_mode(atoi(argv[1]));

    return 0;
}

/********************************************************************
**函数名称:  cmd_nfctrig_test
**入口参数:  sh    ---        Shell句柄，用于输出信息
            argc  ---        参数个数
            argv  ---        参数数组，argv[1]为测试参数字符串
**出口参数:  无
**函数功能:  测试NFC联动指令，接收参数并执行对应的NFC联动命令
**返 回 值:  0表示成功
*********************************************************************/
static int cmd_nfctrig_test(const struct shell *sh, size_t argc, char **argv)
{
    int len;
    uint8_t card_index = 0;
    uint16_t ret = 0;

    if (argc < 2)
    {
        shell_error(sh, "Missing parameter");
        return -EINVAL;
    }

    memset(shell_test_buff, 0, sizeof(shell_test_buff));

    len = strlen(argv[1]);
    memcpy(shell_test_buff, argv[1], len);
    shell_test_buff[len] = 0;

    shell_print(sh, "param: %s, len: %d", argv[1], len);

    //执行NFC联动指令
    ret = run_nfc_cmd(argv[1], &card_index);
    if (ret)
    {
        MY_LOG_INF("Command executed success: cmd_type:%d; command:%s", ret, gConfigParam.nfctrig_config.nfctrig_table.nfctrig_rule[card_index].nfctrig_command);
    }
    else
    {
        MY_LOG_INF("Command executed fail: cmd_type:%d", ret);
    }


    return 0;
}

/********************************************************************
**函数名称:  cmd_nfc_swipe_test
**入口参数:  sh    ---        Shell句柄，用于输出信息
            argc  ---        参数个数
            argv  ---        参数数组，argv[1]为测试参数字符串
**出口参数:  无
**函数功能:  处理NFC刷卡测试命令，执行对应的NFC刷卡测试逻辑（模拟刷卡事件）
**返 回 值:  0表示成功
*********************************************************************/
static int cmd_nfc_swip_test(const struct shell *sh, size_t argc, char **argv)
{
    int len;
    uint8_t select = 0;
    // 定义 10 个测试二进制卡号
    uint8_t test_data[10][7] = {
        {0x88, 0x04, 0x0F, 0xBE, 0x99, 0x05, 0x0B}, // 1
        {0x88, 0x04, 0x0F, 0xBE, 0x99, 0x05, 0xCC}, // 2
        {0x88, 0x04, 0x0F, 0xBE}, // 3
        {0x00}, // 4 (预留位置，可根据需要填充)
        {0x00}, // 5
        {0x00}, // 6
        {0x00}, // 7
        {0x00}, // 8
        {0x00}, // 9
        {0x00}  // 10
    };
    //对应的卡号长度，按序
    uint8_t test_lens[] = {7, 7, 4, 1, 1, 1, 1, 1, 1, 1};

    if (argc < 2)
    {
        shell_error(sh, "Missing parameter");
        return -EINVAL;
    }

    memset(shell_test_buff, 0, sizeof(shell_test_buff));

    len = strlen(argv[1]);
    select = atoi(argv[1]);
    memcpy(shell_test_buff, argv[1], len);
    shell_test_buff[len] = 0;

    shell_print(sh, "param: %s, len: %d", argv[1], len);
    if (select < 1 || select > 10)
    {
        shell_error(sh, "Invalid selection: %d (Range: 1-10)", select);
        return -EINVAL;
    }

    //刷卡处理
    handle_nfc_card_event(test_data[select - 1], test_lens[select-1]);
    return 0;
}

/********************************************************************
**函数名称:  cmd_tag_scan_set_config
**入口参数:  shell   ---        Shell 实例指针
**           argc    ---        参数数量
**           argv    ---        参数数组
**出口参数:  无
**函数功能:  配置 Tag 扫描参数
**返 回 值:  0 表示成功，负数表示失败
**使用示例:  app tagscan <mode> <scan_interval> <scan_length> <upload_interval>
*********************************************************************/
static int cmd_tag_scan_set_config(const struct shell *shell, size_t argc, char **argv)
{
    uint8_t mode;
    uint32_t scan_interval;
    uint32_t scan_length;
    uint32_t upload_interval;
    char *p_endptr;

    if (argc != 5)
    {
        shell_print(shell, "Usage: app tagscan <mode> <scan_interval> <scan_length> <upload_interval>");
        shell_print(shell, "  mode            : Scan mode (0-3, 0=off 1=wakeup 2=period_cache 3=period_upload)");
        shell_print(shell, "  scan_interval   : Scan interval (s), must > scan_length");
        shell_print(shell, "  scan_length     : Scan length (s), must > 0");
        shell_print(shell, "  upload_interval : Upload interval (s), mode 3 required");
        return -EINVAL;
    }

    // 解析 mode 参数
    mode = (uint8_t)strtoul(argv[1], &p_endptr, 10);
    if (*p_endptr != '\0' || mode > 3)
    {
        shell_error(shell, "Invalid mode: %s, mode must be 0-3", argv[1]);
        shell_print(shell, "  0: off");
        shell_print(shell, "  1: wakeup_scan (scan on LTE wakeup)");
        shell_print(shell, "  2: period_cache (periodic scan, upload on LTE wakeup)");
        shell_print(shell, "  3: period_upload (periodic scan + periodic upload)");
        return -EINVAL;
    }

    // 解析 scan_interval 参数
    scan_interval = strtoul(argv[2], &p_endptr, 10);
    if (*p_endptr != '\0' || scan_interval == 0 || scan_interval > 86400)
    {
        shell_error(shell, "Invalid scan_interval: %s, must be 1-86400 seconds", argv[2]);
        return -EINVAL;
    }

    // 解析 scan_length 参数
    scan_length = strtoul(argv[3], &p_endptr, 10);
    if (*p_endptr != '\0' || scan_length == 0 || scan_length > 86400)
    {
        shell_error(shell, "Invalid scan_length: %s, must be 1-86400 seconds", argv[3]);
        return -EINVAL;
    }

    // 解析 upload_interval 参数
    upload_interval = strtoul(argv[4], &p_endptr, 10);
    if (*p_endptr != '\0' || upload_interval > 86400)
    {
        shell_error(shell, "Invalid upload_interval: %s, must be 0-86400 seconds", argv[4]);
        return -EINVAL;
    }

    // 校验 scan_interval 必须大于 scan_length
    if (scan_interval <= scan_length)
    {
        shell_error(shell, "scan_interval (%u) must be greater than scan_length (%u)", scan_interval, scan_length);
        return -EINVAL;
    }

    // 模式 3 要求 upload_interval 必须大于 0
    if (mode == 3 && upload_interval == 0)
    {
        shell_error(shell, "Mode 3 (period_upload) requires upload_interval > 0");
        return -EINVAL;
    }

    // 模式 3 要求 upload_interval 应该大于 scan_interval（避免频繁上报）
    if (mode == 3 && upload_interval <= scan_interval)
    {
        shell_warn(shell, "Warning: upload_interval (%u) should be greater than scan_interval (%u) in mode 3", upload_interval, scan_interval);
    }

    my_scan_set_config(mode, scan_interval, scan_length, upload_interval);

    shell_print(shell, "Tag scan config set OK:");
    shell_print(shell, "  mode            = %d (%s)", mode,
                mode == 0 ? "off" : (mode == 1 ? "wakeup_scan" : (mode == 2 ? "period_cache" : "period_upload")));
    shell_print(shell, "  scan_interval   = %u s", scan_interval);
    shell_print(shell, "  scan_length     = %u s", scan_length);
    shell_print(shell, "  upload_interval = %u s", upload_interval);

    return 0;
}

/********************************************************************
**函数名称:  cmd_alarm_test
**入口参数:  sh      ---        Shell 实例指针
**           argc    ---        参数数量
**           argv    ---        参数数组
**出口参数:  无
**函数功能:  测试发送告警消息到 LTE 模块
**返 回 值:  0 表示成功，负数表示失败
**使用示例:  app alarmtest <type> [info]
*********************************************************************/
static int cmd_alarm_test(const struct shell *sh, size_t argc, char **argv)
{
    int n_alarm_type;
    const char *p_additional_info;

    if (argc < 2)
    {
        shell_print(sh, "Usage: app alarmtest <type> [info]");
        shell_print(sh, "  type: 0=OPEN, 1=ILLEGALUNLOCK, 2=LOCK, 3=MOTION,");
        shell_print(sh, "        4=BATT, 5=CHARGE, 6=IMPACT, 7=SEPARATE,");
        shell_print(sh, "        8=NFC, 9=CUT, 10=OTHER");
        shell_print(sh, "  info: optional additional info string");
        return -EINVAL;
    }

    // 解析告警类型
    n_alarm_type = atoi(argv[1]);
    if (n_alarm_type < 0 || n_alarm_type > ALARM_OTHER)
    {
        shell_error(sh, "Invalid alarm type: %d", n_alarm_type);
        return -EINVAL;
    }

    // 获取附加信息（可选）
    p_additional_info = (argc >= 3) ? argv[2] : NULL;

    // 发送告警消息
    send_alarm_message_to_lte((alarm_type_t)n_alarm_type, p_additional_info);

    shell_print(sh, "Alarm message sent to LTE:");
    shell_print(sh, "  type: %d", n_alarm_type);
    shell_print(sh, "  info: %s", p_additional_info ? p_additional_info : "(none)");

    return 0;
}
/********************************************************************
**函数名称:  cmd_retransmit_check_test
**入口参数:  sh    ---   Shell 实例句柄
**           argc  ---   参数个数
**           argv  ---   参数数组 (argv[1]: 指令内容, argv[2]: 可选参数)
**出口参数:  无
**函数功能:  Shell 测试命令：手动触发 LTE 重传检查机制
**           调用 lte_send_cmd_with_retry 发送指令并启动重传
**指令格式:  retransmit_test [cmd_string] [optional_param]
**返回值说明:  0:      成功
**           -EINVAL: 参数缺失
**返 回 值:  int
*********************************************************************/
static int cmd_retransmit_check_test(const struct shell *sh, size_t argc, char **argv)
{
    int len;
    char *p = NULL;

    if (argc < 2)
    {
        shell_error(sh, "Missing parameter");
        return -EINVAL;
    }

    memset(shell_test_buff, 0, sizeof(shell_test_buff));

    len = strlen(argv[1]);
    memcpy(shell_test_buff, argv[1], len);
    shell_test_buff[len] = 0;

    // 检查是否有第二个参数
    if (argc >= 3)
    {
        p = argv[2];
        shell_print(sh, "param2: %s", p);
    }
    shell_print(sh, "param1: %s, len: %d", argv[1], len);

    lte_send_cmd_with_retry(argv[1], p);

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
    SHELL_CMD(blog, NULL, "Send BLE log test message: app blog <message>", cmd_ble_log_test),
    SHELL_CMD(blogcfg, NULL, "BLE log config: app blogcfg <global|mod|level|show>", cmd_ble_log_config),
    SHELL_CMD(ble_test, NULL, "test", cmd_ble_test),
    SHELL_CMD(buzzer_test, NULL, "Run Buzzer test", cmd_buzzer_test),
    SHELL_CMD(nfctrig_test, NULL, "Run nfctrig test", cmd_nfctrig_test),
    SHELL_CMD(nfc_swip_test, NULL, "Run nfc swipe test", cmd_nfc_swip_test),
    SHELL_CMD(tagscan, NULL, "Set tag scan config: app tagscan <mode> <scan_interval> <scan_length> <upload_interval>", cmd_tag_scan_set_config),
    SHELL_CMD(alarmtest, NULL, "Test alarm message: app alarmtest <type> [info]", cmd_alarm_test),
    SHELL_CMD(retransmit_check_test, NULL, "Run retransmit_check_test test", cmd_retransmit_check_test),
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