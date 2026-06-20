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

uint8_t g_shell_test_buff[256] = {0};

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
    work_mode_t current_workmode;
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
    device_work_mode_config_t *p_workmode;
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

    memset(g_shell_test_buff, 0, sizeof(g_shell_test_buff));

    len = strlen(argv[1]);
    memcpy(g_shell_test_buff, argv[1], len);
    // 手动增加\r\n，使得my_shell_handle_rx能识别到
    g_shell_test_buff[len++] = '\r';
    g_shell_test_buff[len++] = '\n';
    g_shell_test_buff[len] = 0;

    shell_print(sh, "param: %s, len: %d", argv[1], len);

    my_shell_handle_rx(g_shell_test_buff, len);

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
    msg_t msg;

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
    ble_log_config_t *config;
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
        shell_print(shell, "  MAIN, BLE, DFU, SENSOR, LTE, CTRL, SHELL,");
        shell_print(shell, "  BATTERY, CMD, TOOL, PARAM, WDT, OTHER");
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
        else if (strcmp(argv[2], "BATTERY") == 0)
            mod_id = BLE_LOG_MOD_BATTERY;
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
        else if (strcmp(argv[2], "BATTERY") == 0)
            mod_id = BLE_LOG_MOD_BATTERY;
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
        shell_print(shell, "    BATTERY: %s  %d",
                    BLE_LOG_MOD_IS_ENABLED(config, BLE_LOG_MOD_BATTERY) ? "ON" : "OFF",
                    config->mod_level[BLE_LOG_MOD_BATTERY]);
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

    memset(g_shell_test_buff, 0, sizeof(g_shell_test_buff));

    len = strlen(argv[1]);
    memcpy(g_shell_test_buff, argv[1], len);
    g_shell_test_buff[len] = 0;

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

    memset(g_shell_test_buff, 0, sizeof(g_shell_test_buff));

    len = strlen(argv[1]);
    memcpy(g_shell_test_buff, argv[1], len);
    g_shell_test_buff[len] = 0;

    shell_print(sh, "param: %s, len: %d", argv[1], len);
    my_set_buzzer_mode(atoi(argv[1]));

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
        shell_print(sh, "  type: 1=OPEN, 2=STILL, 3=SEA, 4=LAND,");
        shell_print(sh, "        5=BATT, 6=CHARGE_FULL, 7=IMPACT, 8=CUT, 9=OTHER");
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

    memset(g_shell_test_buff, 0, sizeof(g_shell_test_buff));

    len = strlen(argv[1]);
    memcpy(g_shell_test_buff, argv[1], len);
    g_shell_test_buff[len] = 0;

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

/********************************************************************
**函数名称:  cmd_hardware_test
**入口参数:  sh    ---   Shell 实例句柄
**           argc  ---   参数个数
**           argv  ---   参数数组 (argv[1]: 指令内容, argv[2]: 可选参数)
**出口参数:  无
**函数功能:  Shell 测试命令：手动触发硬件测试
**指令格式:  hardware_ware [cmd_string] [optional_param]
**返回值说明:  0:      成功
**           -EINVAL: 参数缺失
**返 回 值:  int
*********************************************************************/
static int cmd_hardware_test(const struct shell *sh, size_t argc, char **argv)
{
    int mode = 0;

    // G-Sensor电源使能测试
    if (strcmp(argv[1], "gsensorpwren") == 0)
    {
        //gsensorpwren 0/1      (G-Sensor控制电源关/开)
        mode = atoi(argv[2]);

        my_gsensor_pwr_on(mode);
    }
    // 充电电路开关
    else if (strcmp(argv[1], "v_chg_en") == 0)
    {
        //v_chg_en 0/1      (充电电路关/开)
        mode = atoi(argv[2]);

        batt_enable(mode);
    }
    // 4G电源使能测试
    else if (strcmp(argv[1], "4GPOWER") == 0)
    {
        //4GPOWER 0/1      (4G控制电源关/开)
        mode = atoi(argv[2]);

        my_lte_pwr_on(mode);
    }

    return 0;
}

#if FS_STORE_TEST_ENABLE
/********************************************************************
**函数名称:  fs_test_parse_type
**入口参数:  str       ---        类型字符串("tag"或"mac")（输入）
**           type_ptr  ---        接收解析结果的指针（输出）
**出口参数:  type_ptr  ---        存储解析出的数据类型
**函数功能:  将shell输入的类型字符串解析为fs_data_type_t枚举
**返 回 值:  0表示成功，-EINVAL表示非法类型
*********************************************************************/
static int fs_test_parse_type(const char *str, fs_data_type_t *type_ptr)
{
    if (strcmp(str, "tag") == 0)
    {
        *type_ptr = FS_TYPE_TAG;
        return 0;
    }
    else if (strcmp(str, "mac") == 0)
    {
        *type_ptr = FS_TYPE_MAC;
        return 0;
    }

    return -EINVAL;
}

/********************************************************************
**函数名称:  fs_test_make_tag
**入口参数:  rec_ptr   ---        待填充的TAG记录指针（输出）
**           seq       ---        测试序号(编码进MAC地址用于回读校验)（输入）
**出口参数:  rec_ptr   ---        存储构造好的可识别测试TAG记录
**函数功能:  构造一条带唯一序号的测试TAG记录，序号小端编码进MAC地址前4字节
**返 回 值:  无
*********************************************************************/
static void fs_test_make_tag(tag_scan_result_t *rec_ptr, uint32_t seq)
{
    memset(rec_ptr, 0, sizeof(tag_scan_result_t));

    // 序号小端编码进MAC地址前4字节，回读时据此校验顺序与内容
    rec_ptr->addr.a.val[0] = (uint8_t)(seq & 0xFF);
    rec_ptr->addr.a.val[1] = (uint8_t)((seq >> 8) & 0xFF);
    rec_ptr->addr.a.val[2] = (uint8_t)((seq >> 16) & 0xFF);
    rec_ptr->addr.a.val[3] = (uint8_t)((seq >> 24) & 0xFF);
    rec_ptr->addr.a.val[4] = 0xAA;          // 固定标记，便于人工辨识
    rec_ptr->addr.a.val[5] = 0x55;

    rec_ptr->battery_percent = (uint8_t)(seq & 0x7F);
    rec_ptr->rssi = (int8_t)(-40 - (int)(seq & 0x1F));
    snprintf(rec_ptr->name, sizeof(rec_ptr->name), "TAG%u", seq);
    rec_ptr->uuid_len = 0;
    rec_ptr->ff_data_len = 0;
}

/********************************************************************
**函数名称:  fs_test_make_mac
**入口参数:  rec_ptr   ---        待填充的MAC记录指针（输出）
**           seq       ---        测试序号(编码进MAC地址用于回读校验)（输入）
**出口参数:  rec_ptr   ---        存储构造好的可识别测试MAC记录
**函数功能:  构造一条带唯一序号的测试透传MAC记录，序号小端编码进MAC地址前4字节
**返 回 值:  无
*********************************************************************/
static void fs_test_make_mac(tran_mac_result_item_t *rec_ptr, uint32_t seq)
{
    memset(rec_ptr, 0, sizeof(tran_mac_result_item_t));

    // 序号小端编码进MAC地址前4字节
    rec_ptr->addr.a.val[0] = (uint8_t)(seq & 0xFF);
    rec_ptr->addr.a.val[1] = (uint8_t)((seq >> 8) & 0xFF);
    rec_ptr->addr.a.val[2] = (uint8_t)((seq >> 16) & 0xFF);
    rec_ptr->addr.a.val[3] = (uint8_t)((seq >> 24) & 0xFF);
    rec_ptr->addr.a.val[4] = 0xBB;          // 固定标记，便于人工辨识
    rec_ptr->addr.a.val[5] = 0x66;

    // 广播数据填2字节，内容为序号低16位，便于回读校验
    rec_ptr->adv_data_len = 2;
    rec_ptr->adv_data[0] = (uint8_t)(seq & 0xFF);
    rec_ptr->adv_data[1] = (uint8_t)((seq >> 8) & 0xFF);
}

/********************************************************************
**函数名称:  fs_test_decode_seq
**入口参数:  val_ptr   ---        MAC地址字节数组指针（输入）
**出口参数:  无
**函数功能:  从MAC地址前4字节小端解码出测试序号，用于回读校验
**返 回 值:  解码出的测试序号
*********************************************************************/
static uint32_t fs_test_decode_seq(const uint8_t *val_ptr)
{
    return (uint32_t)val_ptr[0] | ((uint32_t)val_ptr[1] << 8) |
           ((uint32_t)val_ptr[2] << 16) | ((uint32_t)val_ptr[3] << 24);
}

/********************************************************************
**函数名称:  cmd_fs_test
**入口参数:  sh      ---        Shell 实例指针
**           argc    ---        参数数量
**           argv    ---        参数数组
**出口参数:  无
**函数功能:  FLASH循环存储模块测试命令，支持落盘/读取/提交/回退/清空/状态查看
**           等子命令，用于验证整个存储-上报闭环及循环覆盖等边界场景
**返 回 值:  0表示成功，负值表示失败
**使用示例:  app fs push tag 100   app fs info tag   app fs read mac 0
*********************************************************************/
static int cmd_fs_test(const struct shell *sh, size_t argc, char **argv)
{
    fs_data_type_t type;
    fs_debug_info_t info;
    tag_scan_result_t tag_rec;
    tran_mac_result_item_t mac_rec;
    uint8_t read_buf[128];      // 读取缓冲(取两类记录中较大者)
    uint32_t count;
    uint32_t i;
    uint32_t seq;
    uint32_t prev_seq;
    bool sorted_ok;
    int ret;
    int read_cnt;
    int rd;

    if (argc < 2)
    {
        shell_print(sh, "Usage: app fs <subcmd> [args]");
        shell_print(sh, "  init                     - Init flash store module");
        shell_print(sh, "  info  <tag|mac>          - Show region internal state");
        shell_print(sh, "  count <tag|mac>          - Show pending record count");
        shell_print(sh, "  push  <tag|mac> <n>      - Push n test records");
        shell_print(sh, "  fill  <tag|mac> <secs>   - Push records to fill <secs> sectors");
        shell_print(sh, "  begin <tag|mac>          - Begin an upload session");
        shell_print(sh, "  read  <tag|mac> [n]      - Read n records (0=read all), no commit");
        shell_print(sh, "  commit <tag|mac>         - Commit current read batch");
        shell_print(sh, "  rewind <tag|mac>         - Rewind read cursor to last commit");
        shell_print(sh, "  clear <tag|mac>          - Clear region (wipe pointers)");
        shell_print(sh, "  sorttest <tag|mac> <n>   - Verify timestamp asc sort on flush");
        return -EINVAL;
    }

    /* init子命令无type参数，单独处理 */
    if (strcmp(argv[1], "init") == 0)
    {
        ret = my_flash_store_init();
        shell_print(sh, "flash store init ret=%d", ret);
        return ret;
    }

    /* 其余子命令均需type参数 */
    if (argc < 3 || fs_test_parse_type(argv[2], &type) != 0)
    {
        shell_error(sh, "need type arg: tag | mac");
        return -EINVAL;
    }

    if (strcmp(argv[1], "info") == 0)
    {
        ret = my_flash_store_get_debug_info(type, &info);
        if (ret != 0)
        {
            shell_error(sh, "get debug info fail ret=%d", ret);
            return ret;
        }

        shell_print(sh, "=== FS %s region state ===", argv[2]);
        shell_print(sh, "  wr_sector       = %u", info.wr_sector);
        shell_print(sh, "  rd_sector       = %u", info.rd_sector);
        shell_print(sh, "  valid_sectors   = %u / %u", info.valid_sectors, info.region_sectors);
        shell_print(sh, "  rd_rec_idx      = %u", info.rd_rec_idx);
        shell_print(sh, "  seq_counter     = %u", info.seq_counter);
        shell_print(sh, "  staging_count   = %u / %u", info.staging_count, info.rec_per_sector);
        shell_print(sh, "  upload_active   = %d", info.upload_active);
        shell_print(sh, "  staging_snap    = %u", info.upload_staging_snap);
        shell_print(sh, "  read_offset     = %u", info.read_offset);
        shell_print(sh, "  staging_read    = %u", info.staging_read_idx);
        shell_print(sh, "  pending_count   = %u", info.pending_count);
        return 0;
    }
    else if (strcmp(argv[1], "count") == 0)
    {
        count = my_flash_store_pending_count(type);
        shell_print(sh, "FS %s pending count = %u", argv[2], count);
        return 0;
    }
    else if (strcmp(argv[1], "push") == 0)
    {
        if (argc < 4)
        {
            shell_error(sh, "Usage: app fs push <tag|mac> <n>");
            return -EINVAL;
        }

        count = strtoul(argv[3], NULL, 10);
        // 用模块内seq_counter作为起始序号，保证多次push序号连续不重复
        my_flash_store_get_debug_info(type, &info);
        seq = my_flash_store_pending_count(type) + info.rd_rec_idx;

        for (i = 0; i < count; i++)
        {
            if (type == FS_TYPE_TAG)
            {
                fs_test_make_tag(&tag_rec, seq + i);
                ret = my_flash_store_push_tag(&tag_rec);
            }
            else
            {
                fs_test_make_mac(&mac_rec, seq + i);
                ret = my_flash_store_push_mac(&mac_rec);
            }

            if (ret != 0)
            {
                shell_warn(sh, "push #%u ret=%d (stop)", i, ret);
                break;
            }
        }

        shell_print(sh, "pushed %u/%u %s records (start seq=%u)", i, count, argv[2], seq);
        return 0;
    }
    else if (strcmp(argv[1], "fill") == 0)
    {
        if (argc < 4)
        {
            shell_error(sh, "Usage: app fs fill <tag|mac> <sectors>");
            return -EINVAL;
        }

        // 计算需要push的记录条数 = 目标扇区数 × 每扇区记录数
        my_flash_store_get_debug_info(type, &info);
        count = strtoul(argv[3], NULL, 10) * info.rec_per_sector;
        seq = my_flash_store_pending_count(type) + info.rd_rec_idx;

        for (i = 0; i < count; i++)
        {
            if (type == FS_TYPE_TAG)
            {
                fs_test_make_tag(&tag_rec, seq + i);
                ret = my_flash_store_push_tag(&tag_rec);
            }
            else
            {
                fs_test_make_mac(&mac_rec, seq + i);
                ret = my_flash_store_push_mac(&mac_rec);
            }

            if (ret != 0)
            {
                shell_warn(sh, "fill #%u ret=%d (stop)", i, ret);
                break;
            }
        }

        shell_print(sh, "filled %u %s records (%s sectors, start seq=%u)",
                    i, argv[2], argv[3], seq);
        return 0;
    }
    else if (strcmp(argv[1], "begin") == 0)
    {
        my_flash_store_upload_begin(type);
        shell_print(sh, "upload session begin for %s", argv[2]);
        return 0;
    }
    else if (strcmp(argv[1], "read") == 0)
    {
        // 读取条数：0或缺省表示读到无数据为止
        count = (argc >= 4) ? strtoul(argv[3], NULL, 10) : 0;
        read_cnt = 0;

        while (count == 0 || (uint32_t)read_cnt < count)
        {
            ret = my_flash_store_read_next(type, read_buf);
            if (ret <= 0)
            {
                // 0表示读尽，负值表示错误
                if (ret < 0)
                {
                    shell_error(sh, "read_next ret=%d", ret);
                }
                break;
            }

            // 从回读记录的MAC地址解码序号，校验顺序与内容
            if (type == FS_TYPE_TAG)
            {
                seq = fs_test_decode_seq(((tag_scan_result_t *)read_buf)->addr.a.val);
                shell_print(sh, "  [%d] TAG seq=%u rssi=%d batt=%u",
                            read_cnt, seq,
                            ((tag_scan_result_t *)read_buf)->rssi,
                            ((tag_scan_result_t *)read_buf)->battery_percent);
            }
            else
            {
                seq = fs_test_decode_seq(((tran_mac_result_item_t *)read_buf)->addr.a.val);
                shell_print(sh, "  [%d] MAC seq=%u adv_len=%u",
                            read_cnt, seq,
                            ((tran_mac_result_item_t *)read_buf)->adv_data_len);
            }

            read_cnt++;
        }

        shell_print(sh, "read %d %s records (call 'commit' to confirm, 'rewind' to resend)",
                    read_cnt, argv[2]);
        return 0;
    }
    else if (strcmp(argv[1], "commit") == 0)
    {
        ret = my_flash_store_commit(type);
        shell_print(sh, "commit %s ret=%d", argv[2], ret);
        return ret;
    }
    else if (strcmp(argv[1], "rewind") == 0)
    {
        my_flash_store_rewind(type);
        shell_print(sh, "rewind %s done", argv[2]);
        return 0;
    }
    else if (strcmp(argv[1], "clear") == 0)
    {
        my_flash_store_clear(type);
        shell_print(sh, "clear %s done", argv[2]);
        return 0;
    }
    else if (strcmp(argv[1], "sorttest") == 0)
    {
        if (argc < 4)
        {
            shell_error(sh, "Usage: app fs sorttest <tag|mac> <n>");
            return -EINVAL;
        }

        count = strtoul(argv[3], NULL, 10);

        // 清空目标区，注入count条乱序时间戳记录并经真实落盘接口排序入FLASH
        my_flash_store_clear(type);
        ret = my_scan_test_flush_sort((uint8_t)type, (uint16_t)count);
        if (ret < 0)
        {
            shell_error(sh, "flush sort inject fail ret=%d", ret);
            return ret;
        }

        // 回读校验：注入时序号与时间戳反序，排序正确则回读序号应严格递减
        my_flash_store_upload_begin(type);
        read_cnt = 0;
        prev_seq = 0;
        sorted_ok = true;

        while (1)
        {
            rd = my_flash_store_read_next(type, read_buf);
            if (rd <= 0)
            {
                break;
            }

            if (type == FS_TYPE_TAG)
            {
                seq = fs_test_decode_seq(((tag_scan_result_t *)read_buf)->addr.a.val);
            }
            else
            {
                seq = fs_test_decode_seq(((tran_mac_result_item_t *)read_buf)->addr.a.val);
            }

            shell_print(sh, "  [%d] seq=%u", read_cnt, seq);

            // 从第二条起，序号应小于前一条(降序)，否则排序未生效
            if (read_cnt > 0 && seq >= prev_seq)
            {
                sorted_ok = false;
            }
            prev_seq = seq;
            read_cnt++;
        }

        // 退回读游标，本测试数据不算正式上报
        my_flash_store_rewind(type);

        if (read_cnt != (int)ret)
        {
            shell_warn(sh, "read back %d but injected %d", read_cnt, ret);
            sorted_ok = false;
        }

        shell_print(sh, "sorttest %s: injected %d, read %d, sorted=%s",
                    argv[2], ret, read_cnt, sorted_ok ? "PASS" : "FAIL");
        return sorted_ok ? 0 : -EIO;
    }

    shell_error(sh, "unknown subcmd: %s", argv[1]);
    return -EINVAL;
}
#endif /* FS_STORE_TEST_ENABLE */

/* 注册自定义命令到 Shell 子系统 */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_app,
    SHELL_CMD(sysinfo, NULL, "Display system information", cmd_system_info),
    SHELL_CMD(bleinfo, NULL, "Display BLE status", cmd_ble_info),
    SHELL_CMD(memstat, NULL, "Display memory statistics", cmd_mem_stat),
    SHELL_CMD(reboot, NULL, "Reboot system", cmd_reboot),
    SHELL_CMD(switchmode, NULL, "Switch work mode", cmd_switch_mode),
    SHELL_CMD(settime, NULL, "settime unix seconds ", cmd_set_time),
    SHELL_CMD(gettime, NULL, "gettime unix seconds", cmd_get_time),
    SHELL_CMD(modeset, NULL, "Configure longlife or smart mode parameters", cmd_modeset),
    SHELL_CMD(AT_TEST, NULL, "Usage:app AT_TEST \"TEST xxxx(AT^GT_CM=xxxx)\"", shell_at_test),
    SHELL_CMD(shutdown, NULL, "Shutdown system (enter ultra-low power mode)", cmd_shutdown),
    SHELL_CMD(blog, NULL, "Send BLE log test message: app blog <message>", cmd_ble_log_test),
    SHELL_CMD(blogcfg, NULL, "BLE log config: app blogcfg <global|mod|level|show>", cmd_ble_log_config),
    SHELL_CMD(ble_test, NULL, "test", cmd_ble_test),
    SHELL_CMD(buzzer_test, NULL, "Run Buzzer test", cmd_buzzer_test),
    SHELL_CMD(tagscan, NULL, "Set tag scan config: app tagscan <mode> <scan_interval> <scan_length> <upload_interval>", cmd_tag_scan_set_config),
    SHELL_CMD(alarmtest, NULL, "Test alarm message: app alarmtest <type> [info]", cmd_alarm_test),
    SHELL_CMD(retransmit_check_test, NULL, "Run retransmit_check_test test", cmd_retransmit_check_test),
    SHELL_CMD(hardware_test, NULL, "Run hardware test", cmd_hardware_test),
#if FS_STORE_TEST_ENABLE
    SHELL_CMD(fs, NULL, "Flash store test: app fs <init|info|count|push|fill|begin|read|commit|rewind|clear|sorttest>", cmd_fs_test),
#endif
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