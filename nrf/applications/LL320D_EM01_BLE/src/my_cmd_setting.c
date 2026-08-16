/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        my_cmd_setting.c
**文件描述:        设备命令设置模块实现文件
**当前版本:        V1.0
**作    者:        周森达 (zhousenda@jimiiot.com)
**完成日期:        2026.01.22
*********************************************************************
** 功能描述:        1. 设备工作模式设置
**                 2. 命令参数配置管理
**                 3. 配置验证与存储
**
** 日志输出规范（重要）:
**   - 本模块所有日志统一使用 LOG_INF/LOG_ERR/LOG_WRN/LOG_DBG
**   - 禁止使用 MY_LOG_INF/MY_LOG_ERR/MY_LOG_WRN/MY_LOG_DBG 可输出蓝牙日志宏
**
** 原因说明:
**   1. 本模块为蓝牙指令处理模块，指令响应已通过 BLE 通道返回给 APP
**   2. 蓝牙连接建立后，指令响应数据通过 0xFEB5 特征值主动回传
**   3. 如使用蓝牙日志宏，会导致日志递归发送（日志发送本身又产生日志）
**   4. 统一使用 RTT 日志，既满足调试需求，又避免蓝牙通道冗余
**
** 示例:
**   LOG_INF("BTLOG enabled");        // 正确 - 仅 RTT 输出
**   MY_LOG_INF("BTLOG enabled");     // 错误 - 会触发蓝牙日志递归
*********************************************************************/

/* 必须在包含 my_comm.h 之前定义 BLE_LOG_MODULE_ID，避免与 my_ble_log.h 中的默认定义冲突 */
#define BLE_LOG_MODULE_ID BLE_LOG_MOD_CMD

#include "my_comm.h"

#define LTE_CMD_BUF_SIZE CMD_STRING_LENGTH_MAX          /* LTE透传最大命令字符串长度 */

LOG_MODULE_REGISTER(my_cmd_setting, LOG_LEVEL_INF);

// 出厂关机标志位
bool g_factory_mode = false;

// 标记lte_cmd来的,用于区分蓝牙下发的还是lte过来的(某些指令只能网络发蓝牙不能执行)
uint8_t g_lte_cmdSource = 0;

// 用于存储整包返回的数据内容(仅在蓝牙线程使用)
char g_resp_buf[RESP_STRING_LENGTH_MAX];

static int remalm_cmd_handler(at_cmd_t* msg);
static int pullalm_cmd_handler(at_cmd_t* msg);
static int patalm_cmd_handler(at_cmd_t* msg);
static int tempalm_cmd_handler(at_cmd_t* msg);
static int motdet_cmd_handler(at_cmd_t* msg);
static int motdetalm_cmd_handler(at_cmd_t* msg);
static int batlevel_cmd_handler(at_cmd_t* msg);
static int chargesta_cmd_handler(at_cmd_t* msg);
static int pwsave_cmd_handler(at_cmd_t* msg);
static int pwrlimit_cmd_handler(at_cmd_t* msg);
static int lprunning_cmd_handler(at_cmd_t* msg);
static int startr_cmd_handler(at_cmd_t* msg);
static int cbmt_cmd_handler(at_cmd_t* msg);
static int bt_mac_cmd_handler(at_cmd_t* msg);
static int bt_crfpwr_cmd_handler(at_cmd_t* msg);
static int bt_updata_cmd_handler(at_cmd_t* msg);
static int bluetooth_cmd_handler(at_cmd_t* msg);
static int btconnect_cmd_handler(at_cmd_t* msg);
static int tag_cmd_handler(at_cmd_t* msg);
static int jatag_cmd_handler(at_cmd_t* msg);
static int jgtag_cmd_handler(at_cmd_t* msg);
static int taginit_param_cmd_handler(at_cmd_t* msg);
static int led_cmd_handler(at_cmd_t* msg);
static int ltint_cmd_handler(at_cmd_t* msg);
static int buzzer_cmd_handler(at_cmd_t* msg);
static int btlog_cmd_handler(at_cmd_t* msg);
static int version_cmd_handler(at_cmd_t* msg);
static int modeset_cmd_handler(at_cmd_t* msg);
static int modeget_cmd_handler(at_cmd_t* msg);
static int modeparam_cmd_handler(at_cmd_t* msg);
static int bt_parmac_cmd_handler(at_cmd_t* msg);
static int status_cmd_handler(at_cmd_t* msg);
static int patmtimer_cmd_handler(at_cmd_t* msg);
static int patm_cmd_handler(at_cmd_t* msg);
static int temptimer_cmd_handler(at_cmd_t* msg);
static int temp_cmd_handler(at_cmd_t* msg);
static int imu_alm_cmd_handler(at_cmd_t* msg);
static int factory_cmd_handler(at_cmd_t* msg);
static int factoryall_cmd_handler(at_cmd_t* msg);

static const at_cmd_attr_t at_cmd_attr_table[] =
{
    {"REMALM",         remalm_cmd_handler},
    {"PULLALM",        pullalm_cmd_handler},
    {"PATMALM",        patalm_cmd_handler},
    {"TEMPALM",        tempalm_cmd_handler},
    {"MOTDET",         motdet_cmd_handler},
    {"MOTDETALM",      motdetalm_cmd_handler},
    {"BATLEVEL",       batlevel_cmd_handler},
    {"CHARGESTA",      chargesta_cmd_handler},
    {"PWRSAVE",        pwsave_cmd_handler},
    {"PWRLIMIT",       pwrlimit_cmd_handler},
    {"LPSLEEP",        lprunning_cmd_handler},
    {"STARTR",         startr_cmd_handler},
    {"CBMT",           cbmt_cmd_handler},
    {"BT_MAC",         bt_mac_cmd_handler},
    {"BT_CRFPWR",      bt_crfpwr_cmd_handler},
    {"BT_UPDATA",      bt_updata_cmd_handler},
    {"BLUETOOTH",      bluetooth_cmd_handler},
    {"BTCONNECT",      btconnect_cmd_handler},
    {"TAG",            tag_cmd_handler},
    {"JATAG",          jatag_cmd_handler},
    {"JGTAG",          jgtag_cmd_handler},
    {"TAGINIT_PARAM",  taginit_param_cmd_handler},
    {"LED",            led_cmd_handler},
    {"LTINT",          ltint_cmd_handler},
    {"BUZZER",         buzzer_cmd_handler},
    {"BTLOG",          btlog_cmd_handler},
    {"VERSION",        version_cmd_handler},
    {"MODESET",        modeset_cmd_handler},
    {"MODEGET",        modeget_cmd_handler},
    {"MODEPARAM",      modeparam_cmd_handler},
    {"BT_PARMAC",      bt_parmac_cmd_handler},
    {"STATUS",         status_cmd_handler},
    {"PATMTIMER",      patmtimer_cmd_handler},
    {"PATM",           patm_cmd_handler},
    {"TEMPTIMER",      temptimer_cmd_handler},
    {"TEMP",           temp_cmd_handler},
    {"IMUALM",        imu_alm_cmd_handler},
    {"FACTORY",        factory_cmd_handler},
    {"FACTORYALL",     factoryall_cmd_handler},
};

static const char* lte_cmd_attr_table[] =
{
    "MILEAGE",
    "TRIP",
    "BOOTLOC",
    "SF",
    "GFENCE",
    "APN",
    "HBT",
    "SERVER",
    "SIMPRI",
    "DEEPSLEEPDT",
    "CENTER",
    "SECOND_SERVER",
    "CHECK",
    "GPSDMODE",
    "POSITIONIMMED",
    "BATCH",
    "ANGLEREP",
    "GPSCOLD",
    "NETSET",
    "ICCID",
    "IMSI",
    "ASETGMT",
    "GMT",
    "AMSSERVER",
    "NETLOG",
};

/*********************************************************************
**函数名称:  lte_send_command
**入口参数:  cmd_name     --  命令名称
**           param        --  命令参数（可选，NULL 表示无参数）
**出口参数:  无
**函数功能:  用于构建并发送 LTE 命令到 LTE 模块，支持带参数和不带参数的命令。
**           命令格式：BLE+命令名称[=参数]
**返 回 值:  0 表示成功，-1 表示失败（模式非法）
*********************************************************************/
int lte_send_command(const char *cmd_name, const char *param)
{
    char *p_msg = NULL;  // 动态分配的消息内存
    msg_t msg;  // 消息结构体
    int buf_len;

    // 检查LTE模块电源状态,如果关闭则先开启
    if (!get_lte_power_state())
    {
        my_send_msg(MOD_CTRL, MOD_LTE, MY_MSG_LTE_PWRON);  // 发送开启 LTE 电源的消息
    }

    if(param)
    {
        buf_len = strlen(cmd_name) + strlen(param) + 16;
    }
    else
    {
        buf_len = strlen(cmd_name) + 16;
    }

    MY_MALLOC_BUFFER(p_msg, buf_len);  // 分配内存

    if(p_msg == NULL)  // 内存分配失败
    {
        MY_LOG_ERR("Failed to allocate memory for LTE command message");  // 输出错误信息
        return -1;  // 退出函数
    }

    if (param && strlen(param) > 0)  // 有参数的情况
    {
        snprintf(p_msg, buf_len, "BLE+%s=%s\r\n", cmd_name, param);  // 构建带参数的命令
    }
    else  // 无参数的情况
    {
        snprintf(p_msg, buf_len, "BLE+%s\r\n", cmd_name);  // 构建不带参数的命令
    }

   // 构建消息结构体并发送给LTE模块
    msg.msgID = MY_MSG_LTE_BLE_DATA;  // 设置消息 ID 为 LTE BLE 数据消息
    msg.pData = p_msg;  // 设置消息数据为命令字符串
    msg.DataLen = strlen(p_msg);  // 设置消息长度
    my_send_msg_data(MOD_CTRL, MOD_LTE, &msg);  // 发送消息到 LTE 模块

    return 0;  // 返回成功
}

//TODO: 不知道指令透传数据参数检查是否需要，后续再看
#if 0

static bool validate_lte_cmd_params(at_cmd_t* msg)
{
    // 根据命令名称验证参数
    if (strcmp(msg->parm[0], "MILEAGE") == 0)
    {
        if (msg->parm_count != 2)
        {
            LOG_ERR("MILEAGE command requires exactly 2 parameter");
            return false;
        }
    }
    else if (strcmp(msg->parm[0], "TRIP") == 0)
    {
        if (msg->parm_count != 1)
        {
            LOG_ERR("TRIP command requires exactly 1 parameter");
            return false;
        }
    }
    else if (strcmp(msg->parm[0], "BOOTLOC") == 0)
    {
        if (msg->parm_count != 1)
        {
            LOG_ERR("BOOTLOC command requires exactly 1 parameter");
            return false;
        }
    }
    else if (strcmp(msg->parm[0], "SF") == 0)
    {
        if (msg->parm_count != 3)
        {
            LOG_ERR("SF command requires exactly 3 parameter");
            return false;
        }
    }
    else if (strcmp(msg->parm[0], "GFENCE") == 0)
    {
        if (msg->parm_count != 8)
        {
            LOG_ERR("GFENCE command requires exactly 8 parameter");
            return false;
        }
    }
    else if (strcmp(msg->parm[0], "APN") == 0)
    {
        if (msg->parm_count == 0)
        {
            LOG_ERR("APN command requires more than 0 parameter");
            return false;
        }
    }
    else if (strcmp(msg->parm[0], "HBT") == 0)
    {
        if (msg->parm_count != 2)
        {
            LOG_ERR("HBT command requires exactly 2 parameter");
            return false;
        }
    }
    else if (strcmp(msg->parm[0], "SERVER") == 0)
    {
         if (msg->parm_count == 0)
        {
            LOG_ERR("SERVER command requires more than 0 parameter");
            return false;
        }
    }
    else if (strcmp(msg->parm[0], "SIMPRI") == 0)
    {
        // SIMPRI 命令需要 1 个参数（SIM 卡优先级）
        if (msg->parm_count != 1)
        {
            LOG_ERR("SIMPRI command requires exactly 1 parameter");
            return false;
        }
    }

    return true;
}

#endif

/*********************************************************************
**函数名称:  lte_cmd_handler
**入口参数:  msg     --  AT命令消息结构体指针
**出口参数:  无
**函数功能:  LTE透传命令处理
**返 回 值:  BLE_DATA_TYPE_PACKET_MULTIPLE 表示返回分包传输类型的数据
*********************************************************************/
static int lte_cmd_handler(at_cmd_t* msg)
{
    char* lte_cmd_msg = NULL;    // LTE命令消息缓冲区
    uint16_t remaining;            // 响应消息缓冲区的剩余空间
    int offset = 0;             // 命令消息偏移量，用于追加参数
    int ret = -1;                  // 函数返回值，默认为-1表示失败

    // 动态分配内存存储告警消息
    MY_MALLOC_BUFFER(lte_cmd_msg, LTE_CMD_BUF_SIZE);  // 分配内存，加 1 用于存储终止符
    if(lte_cmd_msg == NULL)  // 内存分配失败
    {
        MY_LOG_ERR("Failed to allocate memory for LTE command message");  // 输出错误信息
        return -1;  // 退出函数
    }

    remaining = RESP_STRING_LENGTH_MAX;  // 计算响应消息缓冲区的大小

    LOG_INF("%s=>%s", __func__, msg->parm[0]);  // 输出函数名和命令名

#if 0
    // 参数判断逻辑（暂时注释掉）
    if (!validate_lte_cmd_params(msg))
    {
        // 参数验证失败，生成错误响应
        ret = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
        if (ret > 0 && ret < remaining)
        {
            msg->resp_length = ret;
        }
        LOG_INF("LTE command parameter validation failed");
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
    }
#endif

    // 构建LTE命令消息，透传命令头
    offset = snprintf(lte_cmd_msg, LTE_CMD_BUF_SIZE, "%s", msg->parm[0]);

    // 追加命令参数
    for (int i = 0; i < msg->parm_count; i++)
    {
        offset += snprintf(lte_cmd_msg + offset, LTE_CMD_BUF_SIZE, ",%s", msg->parm[i+1]);
    }

    // 追加命令结束符
    snprintf(lte_cmd_msg + offset, LTE_CMD_BUF_SIZE, "#");

    // 发送LTE命令
    ret = lte_send_command("CMD", lte_cmd_msg);

    // 释放动态分配的内存
    if(lte_cmd_msg != NULL)
    {
        MY_FREE_BUFFER(lte_cmd_msg);
        lte_cmd_msg = NULL;
    }

    // 检查命令发送是否成功
    if(ret < 0)
    {
        LOG_ERR("Failed to allocate memory for LTE command message");
        snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
    }

    // 生成成功响应消息
    ret = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);

    // 检查响应消息是否生成成功
    if (ret > 0 && ret < remaining)
    {
        msg->resp_length = ret;  // 设置响应消息的长度
        LOG_INF("RETURN_%s_OK", msg->parm[0]);
    }

    // TODO: 后续修改回传数据
    return BLE_DATA_TYPE_PACKET_MULTIPLE;
}

/*********************************************************************
**函数名称:  set_long_battery_params
**入口参数:  p_workmode        --  设备工作模式配置结构体指针
**           reporting_interval --  上报间隔（分钟，5~1440）
**           start_time_str     --  首次唤醒基准时间（字符指针，HHMM格式，如"0800"）
**           gnss_sw            --  GNSS开关（1=ON, 0=OFF）
**出口参数:  无
**函数功能:  设置长续航模式参数
**返 回 值:  0 表示成功，-1 表示失败（参数非法）
*********************************************************************/
int set_long_battery_params(device_work_mode_config_t *p_workmode,
                     uint16_t reporting_interval, const char *start_time_str, uint8_t gnss_sw)
{
    int str_len;
    int i;
    uint16_t start_time;
    uint8_t hh, mm;

    if (p_workmode == NULL)
    {
        LOG_INF("Error: p_workmode pointer is NULL");
        return -1;
    }

    // ========== 1. 校验上报间隔范围 ==========
    if (reporting_interval < 5 || reporting_interval > 1440)
    {
        LOG_INF("Error: long_battery reporting_interval %u out of range (5~1440)", reporting_interval);
        return -1;
    }

    // ========== 2. 字符指针入参的基础校验 ==========
    // 校验字符串是否为NULL
    if (start_time_str == NULL)
    {
        LOG_INF("Error: start_time_str is NULL");
        return -1;
    }
    // 校验字符串长度是否为4位（HHMM必须是4位）
    str_len = strlen(start_time_str);
    if (str_len != 4)
    {
        LOG_INF("Error: start_time_str %s length is %d (must be 4)", start_time_str, str_len);
        return -1;
    }
    // 校验字符串是否全为数字
    for (i = 0; i < 4; i++)
    {
        if (!isdigit((unsigned char)start_time_str[i]))
        {
            LOG_INF("Error: start_time_str %s contains non-digit character at position %d", start_time_str, i);
            return -1;
        }
    }

    // ========== 3. 字符串转数值并拆分HH/MM ==========
    // 先转成16位整数（如"0800"→800，"2400"→2400）
    start_time = (uint16_t)atoi(start_time_str);
    // 拆分小时和分钟
    hh = start_time / 100;
    mm = start_time % 100;

    // ========== 4. 时间范围校验 ==========
    if (!((hh >= 0 && hh <= 24) && (mm >= 0 && mm <= 59) && !(hh == 24 && mm != 0)))
    {
        LOG_INF("Error: long_battery start_time %s invalid (HHMM 0000~2400)", start_time_str);
        return -1;
    }

    // ========== 5. 校验GNSS开关 ==========
    if (gnss_sw > 1)
    {
        LOG_INF("Error: gnss_sw %u out of range (0/1)", gnss_sw);
        return -1;
    }

    // ========== 6. 赋值到工作模式配置结构体 ==========
    p_workmode->long_battery.reporting_interval_min = reporting_interval;
    strcpy(p_workmode->long_battery.start_time, start_time_str);
    p_workmode->long_battery.gnss_sw = gnss_sw;

    LOG_INF("Set long_battery: reporting_interval=%u, start_time=%s, gnss_sw=%u", reporting_interval, start_time_str, gnss_sw);
    return 0;
}

/*********************************************************************
**函数名称:  set_intelligent_params
**入口参数:  p_workmode  --  设备工作模式配置结构体指针
**           sub_mode    --  子模式（0~5）
**           static_int  --  静止状态上报间隔（单位:秒，0=不上报；sub0~4: 180~3600，sub5: 10~86400）
**           moving_int  --  运动状态上报间隔（单位:秒；sub0,1,3: 180~3600，sub2,4,5: 10~86400）
**出口参数:  无
**函数功能:  设置智能模式参数，校验间隔范围
**返 回 值:  0 表示成功，-1 表示失败（参数非法）
**注意事项:  所有子模式间隔单位统一为秒
*********************************************************************/
int set_intelligent_params(device_work_mode_config_t *p_workmode, uint8_t sub_mode,
                     uint32_t static_int, uint32_t moving_int)
{
    if (p_workmode == NULL) return -1;

    // 校验子模式范围
    if (sub_mode > 5)
    {
        LOG_INF("Error: intelligent sub_mode %u out of range (0~5)", sub_mode);
        return -1;
    }

    // 校验静止间隔（0=静止不上报，允许）
    if (static_int != 0)
    {
        if (sub_mode <= 4)
        {
            // 子模式0~4：静止间隔范围180~3600秒
            if (static_int < 180 || static_int > 3600)
            {
                LOG_INF("Error: intelligent static_int %u out of range (0/180~3600 sec) for sub_mode %u", static_int, sub_mode);
                return -1;
            }
        }
        else
        {
            // 子模式5：静止间隔范围10~86400秒
            if (static_int < 10 || static_int > 86400)
            {
                LOG_INF("Error: intelligent static_int %u out of range (0/10~86400 sec) for sub_mode 5", static_int);
                return -1;
            }
        }
    }

    // 校验运动间隔
    if (sub_mode == 0 || sub_mode == 1 || sub_mode == 3)
    {
        // 子模式0,1,3：运动间隔范围180~3600秒
        if (moving_int < 180 || moving_int > 3600)
        {
            LOG_INF("Error: intelligent moving_int %u out of range (180~3600 sec) for sub_mode %u", moving_int, sub_mode);
            return -1;
        }
    }
    else
    {
        // 子模式2,4,5：运动间隔范围10~86400秒
        if (moving_int < 10 || moving_int > 86400)
        {
            LOG_INF("Error: intelligent moving_int %u out of range (10~86400 sec) for sub_mode %u", moving_int, sub_mode);
            return -1;
        }
    }

    p_workmode->intelligent.sub_mode = sub_mode;
    p_workmode->intelligent.static_interval[sub_mode] = static_int;
    p_workmode->intelligent.moving_interval[sub_mode] = moving_int;
    LOG_INF("Set intelligent: sub_mode=%u, static_int=%u, moving_int=%u",
           sub_mode, static_int, moving_int);
    return 0;
}

/********************************************************************
**函数名称:  at_cmd_str_analyse
**入口参数:  str_data      ---        待解析的AT指令字符串(输入)
**         :  tar_data      ---        输出参数数组，存储拆分后的指令参数(输出)
**         :  limit         ---        参数数组最大长度（限制拆分数量）(输入)
**         :  startChar     ---        指令起始字符（NULL表示无起始字符）(输入)
**         :  endChars      ---        指令结束字符集（如"\r\n"）(输入)
**         :  splitChar     ---        参数分隔字符（如','）(输入)
**出口参数:  tar_data中存储解析出的参数字符串
**函数功能:  解析AT指令字符串，按指定分隔符拆分参数到目标数组
**返回值:    成功返回实际拆分的参数数量，失败返回负值错误码(-1入参异常/-2超上限/-3分隔符后超上限/-4未找到结束符)
*********************************************************************/
int at_cmd_str_analyse(char *str_data, char **tar_data, int limit, char startChar, char *endChars, char splitChar)
{
    static char *blank = "";
    int len, i = 0, j = 0, status = 0;
    char *p;
    uint8_t in_quote = 0;   //是否在引号内
    int found_endChar = 0;   //是否找到结束符

    if (str_data == NULL)
    {
        return -1;
    }

    len = strlen(str_data);
    for (i = 0, j = 0, p = str_data; i < len; i++, p++)
    {
        // 处理引号状态切换
        if (*p == '"')
        {
            if (in_quote)
            {
                // 结束引号 → 截断字符串
                *p = '\0';
            }
            else
            {
                // 起始引号 , 参数起点后移
                if (status == 1 && j > 0)
                {
                    tar_data[j - 1] = p + 1;
                }
            }

            in_quote = !in_quote;
            continue;
        }

        if (status == 0 && (*p == startChar || startChar == NULL))
        {
            status = 1;
            if (j >= limit)
            {
                return -2;
            }

            if (startChar == NULL)
            {
                // 如果是引号开头，跳过
                if (*p == '"')
                {
                    tar_data[j++] = p + 1;
                }
                else
                {
                    tar_data[j++] = p;
                }
            }
            else if (*(p + 1) == splitChar)
            {
                tar_data[j++] = blank;
            }
            else
            {
                tar_data[j++] = p + 1;
            }
        }

        if (status == 0)
        {
            continue;
        }

        // 只有不在引号内才判断结束符
        if (!in_quote && strchr(endChars, *p) != NULL)
        {
            *p = 0;
            found_endChar = 1;   //是否找到结束符
            break;
        }

        // 只有不在引号内才按 splitChar 分割
        if (!in_quote && *p == splitChar)
        {
            *p = 0;

            if (j >= limit)
            {
                return -3;
            }

            if (strchr(endChars, *(p + 1)) != NULL || *(p + 1) == splitChar)
            {
                tar_data[j++] = blank;
            }
            else
            {
                tar_data[j++] = p + 1;
            }
        }
    }

    // 新增：检查是否找到结束符
    if (!found_endChar)
    {
        return -4;  // 未找到结束符
    }
    for (i = j; i < limit; i++)
    {
        tar_data[i] = blank;
    }

    //检测引号是否闭合
    if (in_quote)
    {
        return -1;
    }

    return j;
}

/********************************************************************
**函数名称:  at_recv_cmd_handler
**入口参数:  at_cmd_msg      ---        AT指令结构体指针，包含接收的指令和响应存储区域(输入/输出)
**出口参数:  at_cmd_msg中更新响应消息内容和响应长度
**函数功能:  解析接收到的AT指令并执行对应的处理函数
**返回值:    成功返回处理函数返回的BLE数据类型，未匹配指令或处理失败返回0
*********************************************************************/
uint16_t at_recv_cmd_handler(at_cmd_t *at_cmd_msg)
{
    char *data_ptr, split_ch = ',';
    int par_len;
    uint16_t cmd_type = 0;
    uint8_t index;

    data_ptr = at_cmd_msg->rcv_msg;

    // 解析AT指令参数
    par_len = at_cmd_str_analyse(data_ptr, at_cmd_msg->parm, PARM_MAX, NULL, "#", split_ch);
    if (par_len > PARM_MAX || par_len <= 0)
    {
        LOG_INF("at_cmd_analyse_par_len error, len=%d", par_len);
        return cmd_type;
    }
    at_cmd_msg->parm_count = par_len - 1;
#if 0
    if (at_cmd_msg->parm_count)
    {
        LOG_INF("recv_cmd:par_num=%d,%s,%s", at_cmd_msg->parm_count, at_cmd_msg->parm[PARM_1], at_cmd_msg->parm[PARM_2]);
    }
    else
    {
        LOG_INF("recv_cmd:par_num=%d,%s", at_cmd_msg->parm_count, at_cmd_msg->parm[PARM_1]);
    }
#endif
    // 遍历 AT 命令表，查找匹配的命令
    for (index = 0; index < AT_CMD_TABLE_TOTAL; index++)
    {
        if (my_strcasecmp(at_cmd_attr_table[index].cmd_str, at_cmd_msg->parm[PARM_1]) == 0)
        {
            if (at_cmd_attr_table[index].cmd_func != NULL)
            {
                cmd_type = at_cmd_attr_table[index].cmd_func(at_cmd_msg);
                return cmd_type;
            }
        }
    }

    // 遍历 LTE 命令表，查找匹配的命令
    for (index = 0; index < LTE_CMD_TABLE_TOTAL; index++)
    {
        if (my_strcasecmp(lte_cmd_attr_table[index], at_cmd_msg->parm[PARM_1]) == 0)
        {
            cmd_type = lte_cmd_handler(at_cmd_msg);
            return cmd_type;
        }
    }

    // 未匹配指令，返回错误回复
    at_cmd_msg->resp_length = snprintf(at_cmd_msg->resp_msg, RESP_STRING_LENGTH_MAX, "Set Fail! UnknownCmd");

    return cmd_type;
}

/********************************************************************
**函数名称:  lte_cmd_source_clear_callback
**入口参数:  param      ---        无
**出口参数:  无
**函数功能:  清除lte_cmdSource标志位
*********************************************************************/
static void lte_cmd_source_clear_callback(void *param)
{
    g_lte_cmdSource = 0;
}

/********************************************************************
**函数名称:  run_lte_cmd
**入口参数:  at_cmd_msg      ---   指令结构体指针，包含接收的指令和响应存储区域(输入/输出)
**出口参数:  at_cmd_msg中更新响应消息内容和响应长度
**函数功能:  执行LTE+CMD指令中的command
**返回值:    未匹配指令或命令解析失败返回0
**          返回非0不代表command执行成功，具体看对应的执行函数resp_msg回复
**          返回2代表需要异步回复
*********************************************************************/
uint16_t run_lte_cmd(at_cmd_t *at_cmd_msg)
{

    uint16_t cmd_type = 0;

    MY_LOG_INF("at_cmd_msg->rcv_msg:%s", at_cmd_msg->rcv_msg);
    MY_LOG_INF("at_cmd_msg->rcv_msglen:%d", strlen(at_cmd_msg->rcv_msg));

    //标记网络指令
    g_lte_cmdSource = 1;

    //执行命令
    cmd_type = at_recv_cmd_handler(at_cmd_msg);

    // 2秒后清除lte_cmdSource标志位
    my_start_timer(MY_TIMER_LTE_CMDSOURCE_CLEAR, 2000, false, lte_cmd_source_clear_callback);

    return cmd_type;
}

/********************************************************************
**函数名称:  remalm_cmd_handler
**入口参数:  msg      ---        AT指令结构体指针
**出口参数:  msg->resp_msg  ---  响应消息
**           msg->resp_length --- 响应长度
**函数功能:  处理REMALM指令：设置设备防拆报警功能
**指令格式:  REMALM,<SW>,<M>#
**参数说明:  <SW> - 功能开关: ON/OFF
**           <M> - 报警上报方式: 0-不上报，1-GPRS, 2-GPRS+SMS, 3-GPRS+SMS+CALL
**返 回 值:  BLE数据类型
*********************************************************************/
static int remalm_cmd_handler(at_cmd_t* msg)
{
    uint16_t remaining;
    uint8_t no_count = 0;
    int m_value;
    int sw_value;

    remaining = RESP_STRING_LENGTH_MAX;

    //无参数即查询
    if (msg->parm_count == 0)
    {
        // 根据 remalm_sw 的值选择 "ON" 或 "OFF"
        const char* state_str = gConfigParam.remalm_config.remalm_sw ? "ON" : "OFF";

        msg->resp_length = snprintf(msg->resp_msg, remaining, "%s:%s,%d", msg->parm[0],
                                    state_str, gConfigParam.remalm_config.remalm_mode);
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
    }

    /* 检查参数数量 */
    if (msg->parm_count != 2)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        goto param_invalid;
    }

    /* 解析SW参数 */
    if (my_strcasecmp(msg->parm[1], "ON") == 0)
    {
        sw_value = 1;
    }
    else if (my_strcasecmp(msg->parm[1], "OFF") == 0)
    {
        sw_value = 0;
    }
    else
    {
        LOG_INF("%s=>invalid SW param: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }

    no_count = string_check_is_number(0, msg->parm[2]);
    if (no_count == 0 || no_count > 9)
    {
        LOG_INF("%s=>invalid M param: %s", __func__, msg->parm[2]);
        goto param_invalid;
    }

    /* 解析M参数 */
    m_value = atoi(msg->parm[2]);
    if (m_value < 0 || m_value > 3)
    {
        LOG_INF("%s=>invalid M param: %s", __func__, msg->parm[2]);
        goto param_invalid;
    }

    /* 所有参数验证通过,统一赋值 */
    gConfigParam.remalm_config.flag = FLAG_VALID;
    gConfigParam.remalm_config.remalm_sw = (uint8_t)sw_value;
    gConfigParam.remalm_config.remalm_mode = (uint8_t)m_value;

    /* 保存配置 */
    my_user_data_write(ZMS_ID_REM_ALM_CONFIG, &gConfigParam.remalm_config, sizeof(remalm_config_t));

    LOG_INF("%s=>%s,%s,%s", __func__, msg->parm[0], msg->parm[1], msg->parm[2]);

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set OK");
    LOG_INF("REMALM: SW=%d, M=%d", gConfigParam.remalm_config.remalm_sw, gConfigParam.remalm_config.remalm_mode);

    return BLE_DATA_TYPE_PACKET_MULTIPLE;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set Fail! InvalidParam");
    return BLE_DATA_TYPE_PACKET_MULTIPLE;
}

/********************************************************************
**函数名称:  pullalm_cmd_handler
**入口参数:  msg      ---        AT指令结构体指针
**出口参数:  msg->resp_msg  ---  响应消息
**           msg->resp_length --- 响应长度
**函数功能:  处理PULLALM指令：设置设备防拆卸报警功能
**指令格式:  PULLALM,<SW>,<M>#
**参数说明:  <SW> - 功能开关: ON/OFF
**           <M> - 报警上报方式: 0-不上报，1-GPRS, 2-GPRS+SMS, 3-GPRS+SMS+CALL
**返 回 值:  BLE数据类型
*********************************************************************/
static int pullalm_cmd_handler(at_cmd_t* msg)
{
    uint16_t remaining;
    uint8_t no_count = 0;
    int m_value;
    int sw_value;

    remaining = RESP_STRING_LENGTH_MAX;

    //无参数即查询
    if (msg->parm_count == 0)
    {
        // 根据 pullalm_sw 的值选择 "ON" 或 "OFF"
        const char* state_str = gConfigParam.pullalm_config.pullalm_sw ? "ON" : "OFF";

        msg->resp_length = snprintf(msg->resp_msg, remaining, "%s:%s,%d", msg->parm[0],
                                    state_str, gConfigParam.pullalm_config.pullalm_mode);
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
    }

    /* 检查参数数量 */
    if (msg->parm_count != 2)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        goto param_invalid;
    }

    /* 解析SW参数 */
    if (my_strcasecmp(msg->parm[1], "ON") == 0)
    {
        sw_value = 1;
    }
    else if (my_strcasecmp(msg->parm[1], "OFF") == 0)
    {
        sw_value = 0;
    }
    else
    {
        LOG_INF("%s=>invalid SW param: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }

    no_count = string_check_is_number(0, msg->parm[2]);
    if (no_count == 0 || no_count > 9)
    {
        LOG_INF("%s=>invalid M param: %s", __func__, msg->parm[2]);
        goto param_invalid;
    }

    /* 解析M参数 */
    m_value = atoi(msg->parm[2]);
    if (m_value < 0 || m_value > 3)
    {
        LOG_INF("%s=>invalid M param: %s", __func__, msg->parm[2]);
        goto param_invalid;
    }

    /* 所有参数验证通过,统一赋值 */
    gConfigParam.pullalm_config.flag = FLAG_VALID;
    gConfigParam.pullalm_config.pullalm_sw = (uint8_t)sw_value;
    gConfigParam.pullalm_config.pullalm_mode = (uint8_t)m_value;

    /* 保存配置 */
    my_user_data_write(ZMS_ID_PULL_ALM_CONFIG, &gConfigParam.pullalm_config, sizeof(pullalm_config_t));

    LOG_INF("%s=>%s,%s,%s", __func__, msg->parm[0], msg->parm[1], msg->parm[2]);

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set OK");
    LOG_INF("PULLALM: SW=%d, M=%d", gConfigParam.pullalm_config.pullalm_sw, gConfigParam.pullalm_config.pullalm_mode);

    return BLE_DATA_TYPE_PACKET_MULTIPLE;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set Fail! InvalidParam");
    return BLE_DATA_TYPE_PACKET_MULTIPLE;
}

/********************************************************************
**函数名称:  patalm_cmd_handler
**入口参数:  msg      ---        AT指令结构体指针
**出口参数:  msg->resp_msg  ---  响应消息
**           msg->resp_length --- 响应长度
**函数功能:  处理PATMALM指令：设置气压报警功能
**指令格式:  PATMALM,<SW>,<LOW_THRESHOLD>,<HIGH_THRESHOLD>,<REPORT_TYPE>,<REPORT_INTERVAL>#
**参数说明:  <SW> - 功能开关: ON/OFF
**           <LOW_THRESHOLD> - 低压报警阈值: 30-250, 255-不报警
**           <HIGH_THRESHOLD> - 高压报警阈值: 30-250, 255-不报警
**           <REPORT_TYPE> - 报警上报方式: 0-不上报，1-GPRS, 2-GPRS+SMS, 3-GPRS+SMS+CALL
**           <REPORT_INTERVAL> - 重复报警上报间隔: 0-60分钟
**返 回 值:  BLE数据类型
*********************************************************************/
static int patalm_cmd_handler(at_cmd_t* msg)
{
    uint16_t remaining;
    uint8_t no_count = 0;
    int sw_value;
    int low_threshold;
    int high_threshold;
    int report_type;
    int report_interval;

    remaining = RESP_STRING_LENGTH_MAX;

    //无参数即查询
    if (msg->parm_count == 0)
    {
        // 根据 patalm_sw 的值选择 "ON" 或 "OFF"
        const char* state_str = gConfigParam.patalm_config.patalm_sw ? "ON" : "OFF";

        msg->resp_length = snprintf(msg->resp_msg, remaining, "%s:%s,%d,%d,%d,%d,%d", msg->parm[0],
                                    state_str,
                                    gConfigParam.patalm_config.patalm_low_threshold,
                                    gConfigParam.patalm_config.patalm_high_threshold,
                                    gConfigParam.patalm_config.patalm_report_type,
                                    gConfigParam.patalm_config.patalm_report_interval);
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
    }

    /* 检查参数数量 */
    if (msg->parm_count != 5)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        goto param_invalid;
    }

    /* 解析SW参数 */
    if (my_strcasecmp(msg->parm[1], "ON") == 0)
    {
        sw_value = 1;
    }
    else if (my_strcasecmp(msg->parm[1], "OFF") == 0)
    {
        sw_value = 0;
    }
    else
    {
        LOG_INF("%s=>invalid SW param: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }

    no_count = string_check_is_number(0, msg->parm[2]);
    if (no_count == 0 || no_count > 9)
    {
        LOG_INF("%s=>invalid LOW_THRESHOLD param: %s", __func__, msg->parm[2]);
        goto param_invalid;
    }

    /* 解析LOW_THRESHOLD参数 */
    low_threshold = atoi(msg->parm[2]);
    if ((low_threshold < 30 || low_threshold > 250) && low_threshold != 255)
    {
        LOG_INF("%s=>invalid LOW_THRESHOLD param: %s", __func__, msg->parm[2]);
        goto param_invalid;
    }

    no_count = string_check_is_number(0, msg->parm[3]);
    if (no_count == 0 || no_count > 9)
    {
        LOG_INF("%s=>invalid HIGH_THRESHOLD param: %s", __func__, msg->parm[3]);
        goto param_invalid;
    }

    /* 解析HIGH_THRESHOLD参数 */
    high_threshold = atoi(msg->parm[3]);
    if ((high_threshold < 30 || high_threshold > 250) && high_threshold != 255)
    {
        LOG_INF("%s=>invalid HIGH_THRESHOLD param: %s", __func__, msg->parm[3]);
        goto param_invalid;
    }

    if (high_threshold < low_threshold && low_threshold != 255)
    {
        LOG_INF("The low pressure alarm threshold must be less than the high pressure alarm threshold, except 255.");
        msg->resp_length = snprintf(msg->resp_msg, remaining, "The low pressure alarm threshold must be less than the high pressure alarm threshold, except 255.");
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
    }

    no_count = string_check_is_number(0, msg->parm[4]);
    if (no_count == 0 || no_count > 9)
    {
        LOG_INF("%s=>invalid REPORT_TYPE param: %s", __func__, msg->parm[4]);
        goto param_invalid;
    }

    /* 解析REPORT_TYPE参数 */
    report_type = atoi(msg->parm[4]);
    if (report_type < 0 || report_type > 3)
    {
        LOG_INF("%s=>invalid REPORT_TYPE param: %s", __func__, msg->parm[4]);
        goto param_invalid;
    }

    no_count = string_check_is_number(0, msg->parm[5]);
    if (no_count == 0 || no_count > 9)
    {
        LOG_INF("%s=>invalid REPORT_INTERVAL param: %s", __func__, msg->parm[5]);
        goto param_invalid;
    }

    /* 解析REPORT_INTERVAL参数 */
    report_interval = atoi(msg->parm[5]);
    if (report_interval < 0 || report_interval > 60)
    {
        LOG_INF("%s=>invalid REPORT_INTERVAL param: %s", __func__, msg->parm[5]);
        goto param_invalid;
    }

    /* 所有参数验证通过,统一赋值 */
    gConfigParam.patalm_config.flag = FLAG_VALID;
    gConfigParam.patalm_config.patalm_sw = (uint8_t)sw_value;
    gConfigParam.patalm_config.patalm_low_threshold = (uint8_t)low_threshold;
    gConfigParam.patalm_config.patalm_high_threshold = (uint8_t)high_threshold;
    gConfigParam.patalm_config.patalm_report_type = (uint8_t)report_type;
    gConfigParam.patalm_config.patalm_report_interval = (uint8_t)report_interval;

    /* 保存配置 */
    my_user_data_write(ZMS_ID_PATMALM_CONFIG, &gConfigParam.patalm_config, sizeof(patalm_config_t));

    LOG_INF("%s=>%s,%s,%s", __func__, msg->parm[0], msg->parm[1], msg->parm[2]);

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set OK");
    LOG_INF("PATMALM: SW=%d, LOW_THRESHOLD=%d, HIGH_THRESHOLD=%d, REPORT_TYPE=%d, REPORT_INTERVAL=%d",
            gConfigParam.patalm_config.patalm_sw,
            gConfigParam.patalm_config.patalm_low_threshold,
            gConfigParam.patalm_config.patalm_high_threshold,
            gConfigParam.patalm_config.patalm_report_type,
            gConfigParam.patalm_config.patalm_report_interval);

    return BLE_DATA_TYPE_PACKET_MULTIPLE;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set Fail! InvalidParam");
    return BLE_DATA_TYPE_PACKET_MULTIPLE;
}

/********************************************************************
**函数名称:  tempalm_cmd_handler
**入口参数:  msg      ---        AT指令结构体指针
**出口参数:  msg->resp_msg  ---  响应消息
**           msg->resp_length --- 响应长度
**函数功能:  处理TEMPALM指令：设置温湿度报警功能
**指令格式:  TEMPALM,<SW>,<TEMP_LOW_THRESHOLD>,<TEMP_HIGH_THRESHOLD>,<HUMI_LOW_THRESHOLD>,<HUMI_HIGH_THRESHOLD>,<REPORT_TYPE>,<REPORT_INTERVAL>#
**参数说明:  <SW> - 功能开关: ON/OFF
**           <TEMP_LOW_THRESHOLD> - 低温报警阈值: -30~100℃, 255-不报警
**           <TEMP_HIGH_THRESHOLD> - 高温报警阈值: 30~100℃, 255-不报警
**           <HUMI_LOW_THRESHOLD> - 低湿度报警阈值: 0-100%, 255-不报警
**           <HUMI_HIGH_THRESHOLD> - 高湿度报警阈值: 0-100%, 255-不报警
**           <REPORT_TYPE> - 报警上报方式: 0-不上报，1-GPRS, 2-GPRS+SMS, 3-GPRS+SMS+CALL
**           <REPORT_INTERVAL> - 重复报警上报间隔: 0-60分钟
**返 回 值:  BLE数据类型
*********************************************************************/
static int tempalm_cmd_handler(at_cmd_t* msg)
{
    uint16_t remaining;
    uint8_t no_count = 0;
    int sw_value;
    int temp_low_threshold;
    int temp_high_threshold;
    int humi_low_threshold;
    int humi_high_threshold;
    int report_type;
    int report_interval;

    remaining = RESP_STRING_LENGTH_MAX;

    //无参数即查询
    if (msg->parm_count == 0)
    {
        const char* state_str = gConfigParam.tempalm_config.tempalm_sw ? "ON" : "OFF";

        msg->resp_length = snprintf(msg->resp_msg, remaining, "%s:%s,%d,%d,%d,%d,%d,%d", msg->parm[0],
                                    state_str,
                                    gConfigParam.tempalm_config.temp_low_threshold,
                                    gConfigParam.tempalm_config.temp_high_threshold,
                                    gConfigParam.tempalm_config.humi_low_threshold,
                                    gConfigParam.tempalm_config.humi_high_threshold,
                                    gConfigParam.tempalm_config.tempalm_report_type,
                                    gConfigParam.tempalm_config.tempalm_report_interval);
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
    }

    /* 检查参数数量 */
    if (msg->parm_count != 7)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        goto param_invalid;
    }

    /* 解析SW参数 */
    if (my_strcasecmp(msg->parm[1], "ON") == 0)
    {
        sw_value = 1;
    }
    else if (my_strcasecmp(msg->parm[1], "OFF") == 0)
    {
        sw_value = 0;
    }
    else
    {
        LOG_INF("%s=>invalid SW param: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }

    no_count = string_check_is_number(1, msg->parm[2]);
    if (no_count == 0 || no_count > 9)
    {
        LOG_INF("%s=>invalid TEMP_LOW_THRESHOLD param: %s", __func__, msg->parm[2]);
        goto param_invalid;
    }

    /* 解析TEMP_LOW_THRESHOLD参数 */
    temp_low_threshold = atoi(msg->parm[2]);
    if ((temp_low_threshold < -30 || temp_low_threshold > 100) && temp_low_threshold != 255)
    {
        LOG_INF("%s=>invalid TEMP_LOW_THRESHOLD param: %s", __func__, msg->parm[2]);
        goto param_invalid;
    }

    no_count = string_check_is_number(1, msg->parm[3]);
    if (no_count == 0 || no_count > 9)
    {
        LOG_INF("%s=>invalid TEMP_HIGH_THRESHOLD param: %s", __func__, msg->parm[3]);
        goto param_invalid;
    }

    /* 解析TEMP_HIGH_THRESHOLD参数 */
    temp_high_threshold = atoi(msg->parm[3]);
    if ((temp_high_threshold < 30 || temp_high_threshold > 100) && temp_high_threshold != 255)
    {
        LOG_INF("%s=>invalid TEMP_HIGH_THRESHOLD param: %s", __func__, msg->parm[3]);
        goto param_invalid;
    }

    if (temp_high_threshold < temp_low_threshold && temp_low_threshold != 255)
    {
        LOG_INF("The low temperature alarm threshold must be less than the high temperature alarm threshold, except 255.");
        msg->resp_length = snprintf(msg->resp_msg, remaining, "The low temperature alarm threshold must be less than the high temperature alarm threshold, except 255.");
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
    }

    no_count = string_check_is_number(0, msg->parm[4]);
    if (no_count == 0 || no_count > 9)
    {
        LOG_INF("%s=>invalid HUMI_LOW_THRESHOLD param: %s", __func__, msg->parm[4]);
        goto param_invalid;
    }

    /* 解析HUMI_LOW_THRESHOLD参数 */
    humi_low_threshold = atoi(msg->parm[4]);
    if ((humi_low_threshold < 0 || humi_low_threshold > 100) && humi_low_threshold != 255)
    {
        LOG_INF("%s=>invalid HUMI_LOW_THRESHOLD param: %s", __func__, msg->parm[4]);
        goto param_invalid;
    }

    no_count = string_check_is_number(0, msg->parm[5]);
    if (no_count == 0 || no_count > 9)
    {
        LOG_INF("%s=>invalid HUMI_HIGH_THRESHOLD param: %s", __func__, msg->parm[5]);
        goto param_invalid;
    }

    /* 解析HUMI_HIGH_THRESHOLD参数 */
    humi_high_threshold = atoi(msg->parm[5]);
    if ((humi_high_threshold < 0 || humi_high_threshold > 100) && humi_high_threshold != 255)
    {
        LOG_INF("%s=>invalid HUMI_HIGH_THRESHOLD param: %s", __func__, msg->parm[5]);
        goto param_invalid;
    }

    if (humi_high_threshold < humi_low_threshold && humi_low_threshold != 255)
    {
        LOG_INF("The low humidity alarm threshold must be less than the high humidity alarm threshold, except 255.");
        msg->resp_length = snprintf(msg->resp_msg, remaining, "The low humidity alarm threshold must be less than the high humidity alarm threshold, except 255.");
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
    }

    no_count = string_check_is_number(0, msg->parm[6]);
    if (no_count == 0 || no_count > 9)
    {
        LOG_INF("%s=>invalid REPORT_TYPE param: %s", __func__, msg->parm[6]);
        goto param_invalid;
    }

    /* 解析REPORT_TYPE参数 */
    report_type = atoi(msg->parm[6]);
    if (report_type < 0 || report_type > 3)
    {
        LOG_INF("%s=>invalid REPORT_TYPE param: %s", __func__, msg->parm[6]);
        goto param_invalid;
    }

    no_count = string_check_is_number(0, msg->parm[7]);
    if (no_count == 0 || no_count > 9)
    {
        LOG_INF("%s=>invalid REPORT_INTERVAL param: %s", __func__, msg->parm[7]);
        goto param_invalid;
    }

    /* 解析REPORT_INTERVAL参数 */
    report_interval = atoi(msg->parm[7]);
    if (report_interval < 0 || report_interval > 60)
    {
        LOG_INF("%s=>invalid REPORT_INTERVAL param: %s", __func__, msg->parm[7]);
        goto param_invalid;
    }

    /* 所有参数验证通过,统一赋值 */
    gConfigParam.tempalm_config.flag = FLAG_VALID;
    gConfigParam.tempalm_config.tempalm_sw = sw_value;
    gConfigParam.tempalm_config.temp_low_threshold = temp_low_threshold;
    gConfigParam.tempalm_config.temp_high_threshold = temp_high_threshold;
    gConfigParam.tempalm_config.humi_low_threshold = humi_low_threshold;
    gConfigParam.tempalm_config.humi_high_threshold = humi_high_threshold;
    gConfigParam.tempalm_config.tempalm_report_type = report_type;
    gConfigParam.tempalm_config.tempalm_report_interval = report_interval;

    /* 保存配置 */
    my_user_data_write(ZMS_ID_TEMPALM_CONFIG, &gConfigParam.tempalm_config, sizeof(tempalm_config_t));

    LOG_INF("%s=>%s,%s,%s", __func__, msg->parm[0], msg->parm[1], msg->parm[2]);

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set OK");
    LOG_INF("TEMPALM: SW=%d, TEMP_LOW=%d, TEMP_HIGH=%d, HUMI_LOW=%d, HUMI_HIGH=%d, REPORT_TYPE=%d, REPORT_INTERVAL=%d",
            gConfigParam.tempalm_config.tempalm_sw,
            gConfigParam.tempalm_config.temp_low_threshold,
            gConfigParam.tempalm_config.temp_high_threshold,
            gConfigParam.tempalm_config.humi_low_threshold,
            gConfigParam.tempalm_config.humi_high_threshold,
            gConfigParam.tempalm_config.tempalm_report_type,
            gConfigParam.tempalm_config.tempalm_report_interval);

    return BLE_DATA_TYPE_PACKET_MULTIPLE;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set Fail! InvalidParam");
    return BLE_DATA_TYPE_PACKET_MULTIPLE;
}

/********************************************************************
**函数名称:  motdet_cmd_handler
**入口参数:  msg      ---        AT指令结构体指针
**出口参数:  msg->resp_msg  ---  响应消息
**           msg->resp_length --- 响应长度
**函数功能:  处理MOTDET指令：设置运动检测参数
**指令格式:  MOTDET,[Vibration],[Duration]#
**参数说明:  [Vibration] - 震动次数: 1-500 (默认5)
**           [Duration] - 检测间隔: 1-3600 s (默认10)
**返 回 值:  BLE数据类型
*********************************************************************/
static int motdet_cmd_handler(at_cmd_t* msg)
{
    uint16_t remaining;
    uint8_t no_count = 0;
    int vibration_value;
    int duration_value;

    remaining = RESP_STRING_LENGTH_MAX;

    //无参数即查询
    if (msg->parm_count == 0)
    {
        msg->resp_length = snprintf(msg->resp_msg, remaining, "%s:%d,%d,%d",
                            msg->parm[0],
                            gConfigParam.motdet_config.motdet_vibration,
                            gConfigParam.motdet_config.motdet_duration);
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
    }

    /* 检查参数数量 */
    if (msg->parm_count != 2)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        goto param_invalid;
    }

    no_count = string_check_is_number(0, msg->parm[1]);
    if (no_count == 0 || no_count > 9)
    {
        LOG_INF("%s=>invalid Vibration param: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }
    /* 解析Vibration参数 */
    vibration_value = atoi(msg->parm[1]);
    if (vibration_value < 1 || vibration_value > 500)
    {
        LOG_INF("%s=>invalid Vibration param: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }

    no_count = string_check_is_number(0, msg->parm[2]);
    if (no_count == 0 || no_count > 9)
    {
        LOG_INF("%s=>invalid Duration param: %s", __func__, msg->parm[2]);
        goto param_invalid;
    }
    /* 解析Duration参数 */
    duration_value = atoi(msg->parm[2]);
    if (duration_value < 1 || duration_value > 3600)
    {
        LOG_INF("%s=>invalid Duration param: %s", __func__, msg->parm[2]);
        goto param_invalid;
    }

#if GSENSOR_DUTY_PROJECT
    if (vibration_value > duration_value)
    {
        goto param_invalid;
    }
#endif

    /* 所有参数验证通过,统一赋值 */
    gConfigParam.motdet_config.flag = FLAG_VALID;
    gConfigParam.motdet_config.motdet_vibration = (uint16_t)vibration_value;
    gConfigParam.motdet_config.motdet_duration = (uint16_t)duration_value;

    /* 保存配置 */
    my_user_data_write(ZMS_ID_MOT_DET_CONFIG, &gConfigParam.motdet_config, sizeof(mot_det_config_t));

    LOG_INF("%s=>%s,%s,%s,%s,%s,%s", __func__, msg->parm[0], msg->parm[1],
           msg->parm[2], msg->parm[3], msg->parm[4], msg->parm[5]);

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set OK");
    LOG_INF("MOTDET: Vibration=%d, Duration=%d",
           gConfigParam.motdet_config.motdet_vibration,
           gConfigParam.motdet_config.motdet_duration);

    //TODO 具体逻辑处理

    return BLE_DATA_TYPE_PACKET_MULTIPLE;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set Fail! InvalidParam");
    return BLE_DATA_TYPE_PACKET_MULTIPLE;
}

/********************************************************************
**函数名称:  motdetalm_cmd_handler
**入口参数:  msg      ---        AT指令结构体指针
**出口参数:  msg->resp_msg  ---  响应消息
**           msg->resp_length --- 响应长度
**函数功能:  处理MOTDETALM指令：设置运动检测报警报警功能
**指令格式:  MOTDETALM,<SW>,<M>#
**参数说明:  <SW> - 功能开关: ON/OFF
**           <M> - 报警上报方式: 0-不上报，1-GPRS, 2-GPRS+SMS, 3-GPRS+SMS+CALL
**返 回 值:  BLE数据类型
*********************************************************************/
static int motdetalm_cmd_handler(at_cmd_t* msg)
{
    uint16_t remaining;
    uint8_t no_count = 0;
    int sw_value;
    int report_type_value;

    remaining = RESP_STRING_LENGTH_MAX;

    //无参数即查询
    if (msg->parm_count == 0)
    {
        const char* state_str = gConfigParam.motdetalm_config.motdetalm_sw ? "ON" : "OFF";

        msg->resp_length = snprintf(msg->resp_msg, remaining, "%s:%s,%d", msg->parm[0],
                                    state_str, gConfigParam.motdetalm_config.motdetalm_report_type);
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
    }

    /* 检查参数数量 */
    if (msg->parm_count != 2)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        goto param_invalid;
    }

    /* 解析SW参数 */
    if (my_strcasecmp(msg->parm[1], "ON") == 0)
    {
        sw_value = 1;
    }
    else if (my_strcasecmp(msg->parm[1], "OFF") == 0)
    {
        sw_value = 0;
    }
    else
    {
        LOG_INF("%s=>invalid SW param: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }

    no_count = string_check_is_number(0, msg->parm[2]);
    if (no_count == 0 || no_count > 9)
    {
        LOG_INF("%s=>invalid Report Type param: %s", __func__, msg->parm[2]);
        goto param_invalid;
    }

    /* 解析Report Type参数 */
    report_type_value = atoi(msg->parm[2]);
    if (report_type_value < 0 || report_type_value > 3)
    {
        LOG_INF("%s=>invalid Report Type param: %s", __func__, msg->parm[2]);
        goto param_invalid;
    }

    /* 所有参数验证通过,统一赋值 */
    gConfigParam.motdetalm_config.flag = FLAG_VALID;
    gConfigParam.motdetalm_config.motdetalm_sw = (uint8_t)sw_value;
    gConfigParam.motdetalm_config.motdetalm_report_type = (uint8_t)report_type_value;

    /* 保存配置 */
    my_user_data_write(ZMS_ID_MOTDETALM_CONFIG, &gConfigParam.motdetalm_config, sizeof(motdetalm_config_t));

    LOG_INF("%s=>%s,%s,%s", __func__, msg->parm[0], msg->parm[1], msg->parm[2]);

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set OK");
    LOG_INF("MOTDETALM: SW=%d, Report Type=%d", gConfigParam.motdetalm_config.motdetalm_sw, gConfigParam.motdetalm_config.motdetalm_report_type);

    return BLE_DATA_TYPE_PACKET_MULTIPLE;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set Fail! InvalidParam");
    return BLE_DATA_TYPE_PACKET_MULTIPLE;
}

/********************************************************************
**函数名称:  batlevel_cmd_handler
**入口参数:  msg      ---        AT指令结构体指针
**出口参数:  msg->resp_msg  ---  响应消息
**           msg->resp_length --- 响应长度
**函数功能:  处理BATLEVEL指令：设置电池电量状态触发与上报配置
**指令格式:  BATLEVEL,[Empty RPT],[LOW RPT],[Normal RPT],[Fair RPT],[High RPT],[Full RPT]#
**参数说明:  共6个参数，每个参数对应一个电量状态的上报方式
**           RPT参数: 0-不上报, 1-GPRS, 2-GPRS+SMS, 3-GPRS+SMS+CALL
**默认设置: BATLEVEL,1,1,1,1,1,1#
**返 回 值:  BLE数据类型
*********************************************************************/
static int batlevel_cmd_handler(at_cmd_t* msg)
{
    uint16_t remaining;
    uint8_t no_count = 0;
    int param_values[6];
    int i;

    remaining = RESP_STRING_LENGTH_MAX;

    //无参数即查询
    if (msg->parm_count == 0)
    {
        msg->resp_length = snprintf(msg->resp_msg, remaining, "%s:%d,%d,%d,%d,%d,%d",
                            msg->parm[0],
                            gConfigParam.batlevel_config.batlevel_empty_rpt,
                            gConfigParam.batlevel_config.batlevel_low_rpt,
                            gConfigParam.batlevel_config.batlevel_normal_rpt,
                            gConfigParam.batlevel_config.batlevel_fair_rpt,
                            gConfigParam.batlevel_config.batlevel_high_rpt,
                            gConfigParam.batlevel_config.batlevel_full_rpt);
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
    }

    /* 检查参数数量 */
    if (msg->parm_count != 6)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        goto param_invalid;
    }

    /* 解析所有6个参数 */
    for (i = 0; i < 6; i++)
    {
        no_count = string_check_is_number(0, msg->parm[i + 1]);
        if (no_count == 0 || no_count > 9)
        {
            LOG_INF("%s=>invalid RPT param: %s", __func__, msg->parm[i + 1]);
            goto param_invalid;
        }
        param_values[i] = atoi(msg->parm[i + 1]);
        if (param_values[i] < REPORT_MODE_NONE || param_values[i] > REPORT_MODE_GPRS_SMS_CALL)
        {
            LOG_INF("%s=>invalid RPT param: %s", __func__, msg->parm[i + 1]);
            goto param_invalid;
        }
    }

    /* 所有参数验证通过,统一赋值 */
    gConfigParam.batlevel_config.flag = FLAG_VALID;
    gConfigParam.batlevel_config.batlevel_empty_rpt = (uint8_t)param_values[0];
    gConfigParam.batlevel_config.batlevel_low_rpt = (uint8_t)param_values[1];
    gConfigParam.batlevel_config.batlevel_normal_rpt = (uint8_t)param_values[2];
    gConfigParam.batlevel_config.batlevel_fair_rpt = (uint8_t)param_values[3];
    gConfigParam.batlevel_config.batlevel_high_rpt = (uint8_t)param_values[4];
    gConfigParam.batlevel_config.batlevel_full_rpt = (uint8_t)param_values[5];

    /* 保存配置 */
    my_user_data_write(ZMS_ID_BAT_LEVEL_CONFIG, &gConfigParam.batlevel_config, sizeof(bat_level_config_t));

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set OK");
    LOG_INF("BATLEVEL: Empty RPT=%d, Low RPT=%d, Normal RPT=%d, Fair RPT=%d, High RPT=%d, Full RPT=%d",
           gConfigParam.batlevel_config.batlevel_empty_rpt, gConfigParam.batlevel_config.batlevel_low_rpt,
           gConfigParam.batlevel_config.batlevel_normal_rpt, gConfigParam.batlevel_config.batlevel_fair_rpt,
           gConfigParam.batlevel_config.batlevel_high_rpt, gConfigParam.batlevel_config.batlevel_full_rpt);

    //TODO 具体逻辑处理

    return BLE_DATA_TYPE_PACKET_MULTIPLE;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set Fail! InvalidParam");
    return BLE_DATA_TYPE_PACKET_MULTIPLE;
}

/********************************************************************
**函数名称:  chargesta_cmd_handler
**入口参数:  msg      ---        AT指令结构体指针
**出口参数:  msg->resp_msg  ---  响应消息
**           msg->resp_length --- 响应长度
**函数功能:  处理CHARGESTA指令：设置充电状态变化上报方式
**指令格式:  CHARGESTA,[RPT]#
**参数说明:  [RPT] - 状态变化时的上报方式: 0-不上报, 1-GPRS(默认), 2-GPRS+SMS, 3-GPRS+SMS+CALL
**返 回 值:  BLE数据类型
*********************************************************************/
static int chargesta_cmd_handler(at_cmd_t* msg)
{
    uint16_t remaining;
    uint8_t no_count = 0;
    int report_value;

    remaining = RESP_STRING_LENGTH_MAX;

    //无参数即查询
    if (msg->parm_count == 0)
    {
        msg->resp_length = snprintf(msg->resp_msg, remaining, "%s:%d",
                            msg->parm[0],
                            gConfigParam.batlevel_config.chargesta_report);
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
    }

    /* 检查参数数量 */
    if (msg->parm_count != 1)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        goto param_invalid;
    }

    no_count = string_check_is_number(0, msg->parm[1]);
    if (no_count == 0 || no_count > 9)
    {
        LOG_INF("%s=>invalid RPT param: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }
    /* 解析RPT参数 */
    report_value = atoi(msg->parm[1]);
    if (report_value < REPORT_MODE_NONE || report_value > REPORT_MODE_GPRS_SMS_CALL)
    {
        LOG_INF("%s=>invalid RPT param: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }

    /* 所有参数验证通过,统一赋值 */
    gConfigParam.batlevel_config.flag = FLAG_VALID;
    gConfigParam.batlevel_config.chargesta_report = (uint8_t)report_value;

    /* 保存配置 */
    my_user_data_write(ZMS_ID_BAT_LEVEL_CONFIG, &gConfigParam.batlevel_config, sizeof(bat_level_config_t));

    LOG_INF("%s=>%s,%s", __func__, msg->parm[0], msg->parm[1]);

    /* 所有参数验证通过,生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set OK");
    LOG_INF("CHARGESTA: Report=%d", gConfigParam.batlevel_config.chargesta_report);

    //TODO 具体逻辑处理

    return BLE_DATA_TYPE_PACKET_MULTIPLE;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set Fail! InvalidParam");
    return BLE_DATA_TYPE_PACKET_MULTIPLE;
}

/********************************************************************
**函数名称:  pwsave_cmd_handler
**入口参数:  msg      ---        AT指令结构体指针
**出口参数:  msg->resp_msg  ---  响应消息
**           msg->resp_length --- 响应长度
**函数功能:  处理PWRSAVE指令：设备进入低功耗运输状态
**指令格式:  PWRSAVE,ON#
**参数说明:  ON - 开启低功耗运输状态
**返 回 值:  BLE数据类型
*********************************************************************/
static int pwsave_cmd_handler(at_cmd_t* msg)
{
    uint16_t remaining;

    remaining = RESP_STRING_LENGTH_MAX;

    /* 检查参数数量 (应为1，指令格式为PWRSAVE,ON#) */
    if (msg->parm_count == 1)
    {
        /* 解析参数 */
        if (my_strcasecmp(msg->parm[1], "ON") == 0)
        {
            LOG_INF("%s=>%s,%s", __func__, msg->parm[0], msg->parm[1]);

            // 关机系统
            if (go_to_shutdown() == 0)
            {
                /* 根据指令说明，立即回复 "Poweroff OK" */
                msg->resp_length = snprintf(msg->resp_msg, remaining, "Poweroff OK");
            }
            else
            {
                msg->resp_length = snprintf(msg->resp_msg, remaining, "Poweroff error! Device is charging.");
                return BLE_DATA_TYPE_PACKET_MULTIPLE;
            }

            LOG_INF("PWRSAVE: Device will enter low-power transport state");
        }
        else
        {
            LOG_INF("%s=>invalid param: %s", __func__, msg->parm[1]);
            msg->resp_length = snprintf(msg->resp_msg, remaining, "Set Fail! InvalidParam");
        }
    }
    else
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        msg->resp_length = snprintf(msg->resp_msg, remaining, "Set Fail! InvalidParam");
    }
    return BLE_DATA_TYPE_PACKET_MULTIPLE;
}

/********************************************************************
**函数名称:  pwrlimit_cmd_handler
**入口参数:  msg      ---        AT指令结构体指针
**出口参数:  msg->resp_msg  ---  响应消息
**           msg->resp_length --- 响应长度
**函数功能:  处理PWRLIMIT指令：限制按钮关机功能
**指令格式:  PWRLIMIT,[SW]#
**参数说明:  ON - 开启限制按钮关机功能
**          OFF - 关闭限制按钮关机功能
**返 回 值:  BLE数据类型
*********************************************************************/
static int pwrlimit_cmd_handler(at_cmd_t* msg)
{
    uint16_t remaining;

    remaining = RESP_STRING_LENGTH_MAX;

    // 无参数即查询
    if (msg->parm_count == 0)
    {
        const char* state_str = gConfigParam.pwrlimit_config.pwrlimit_sw ? "ON" : "OFF";
        msg->resp_length = snprintf(msg->resp_msg, remaining, "%s:%s", msg->parm[0], state_str);
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
    }

    /* 检查参数数量 */
    if (msg->parm_count != 1)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        goto param_invalid;
    }

    if (my_strcasecmp(msg->parm[1], "ON") == 0)
    {
        gConfigParam.pwrlimit_config.pwrlimit_sw = 1;
    }
    else if (my_strcasecmp(msg->parm[1], "OFF") == 0)
    {
        gConfigParam.pwrlimit_config.pwrlimit_sw = 0;
    }
    else
    {
        LOG_INF("%s=>invalid A param: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }

    /* 所有参数验证通过,统一赋值 */
    gConfigParam.pwrlimit_config.flag = FLAG_VALID;

    /* 保存配置 */
    my_user_data_write(ZMS_ID_PWRLIMIT_CONFIG, &gConfigParam.pwrlimit_config, sizeof(pwrlimit_config_t));

    LOG_INF("%s=>%s,%s", __func__, msg->parm[0], msg->parm[1]);

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "set OK");
    LOG_INF("LED: Display=%d", gConfigParam.led_config.led_display);

    return BLE_DATA_TYPE_PACKET_MULTIPLE;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set Fail! InvalidParam");
    return BLE_DATA_TYPE_PACKET_MULTIPLE;
}

/********************************************************************
**函数名称:  lprunning_cmd_handler
**入口参数:  msg      ---        AT指令结构体指针（输入）
**出口参数:  msg->resp_msg  ---  响应消息（输出）
**           msg->resp_length --- 响应长度（输出）
**函数功能:  处理LPSLEEP指令：低功耗运行功能配置
**指令格式:  查询指令: LPSLEEP#
**           设置指令: LPSLEEP,SW,B,T#
**           关闭指令: LPSLEEP,OFF#
**参数说明:  SW  - ON/OFF 功能总开关
**           B   - 进入低功耗运行的电量百分比阈值 (10~50, 默认20)
**           T   - 低功耗运行下定时唤醒间隔 (1~48小时, 默认24)
**返 回 值:  BLE数据类型
*********************************************************************/
static int lprunning_cmd_handler(at_cmd_t* msg)
{
    uint16_t remaining;
    int threshold;
    int interval;
    uint8_t no_count = 0;

    remaining = RESP_STRING_LENGTH_MAX;

    // 无参数即查询
    if (msg->parm_count == 0)
    {
        const char *state_str = gConfigParam.lprunning_config.lprunning_sw ? "ON" : "OFF";
        msg->resp_length = snprintf(msg->resp_msg, remaining, "LPSLEEP:%s,%d,%d",
                                    state_str,
                                    gConfigParam.lprunning_config.lprunning_threshold,
                                    gConfigParam.lprunning_config.lprunning_interval);
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
    }

    // 关闭指令: LPSLEEP,OFF#
    if ((msg->parm_count == 1) && (my_strcasecmp(msg->parm[1], "OFF") == 0))
    {
        gConfigParam.lprunning_config.lprunning_sw = 0;
        gConfigParam.lprunning_config.flag = FLAG_VALID;
        my_user_data_write(ZMS_ID_LPSLEEP_CONFIG, &gConfigParam.lprunning_config, sizeof(lprunning_config_t));

        // 退出请求统一交由main线程串行判断处理
        my_send_msg(MOD_MAIN, MOD_MAIN, MY_MSG_LPSLEEP_EXIT);

        LOG_INF("%s=>%s,OFF", __func__, msg->parm[0]);
        msg->resp_length = snprintf(msg->resp_msg, remaining, "Set OK");
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
    }
    // 缺省开启指令: LPSLEEP,ON# （使用已存储的阈值和间隔参数）
    else if (msg->parm_count == 1 && my_strcasecmp(msg->parm[1], "ON") == 0)
    {
        gConfigParam.lprunning_config.lprunning_sw = 1;
        gConfigParam.lprunning_config.flag = FLAG_VALID;
        my_user_data_write(ZMS_ID_LPSLEEP_CONFIG, &gConfigParam.lprunning_config, sizeof(lprunning_config_t));

        // 清除暂缓标志，允许下次电量低于阈值时立即进入低功耗运行
        my_send_msg(MOD_MAIN, MOD_MAIN, MY_MSG_LPSLEEP_CLEAR_HOLD_OFF);

        LOG_INF("%s=>%s,ON (threshold=%d, interval=%d)", __func__, msg->parm[0],
                gConfigParam.lprunning_config.lprunning_threshold,
                gConfigParam.lprunning_config.lprunning_interval);
        msg->resp_length = snprintf(msg->resp_msg, remaining, "Set OK");
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
    }
    // 设置指令: LPSLEEP,ON,B,T#
    else if (msg->parm_count == 3 && my_strcasecmp(msg->parm[1], "ON") == 0)
    {
        // 校验参数是否为纯数字
        no_count = string_check_is_number(0, msg->parm[2]);
        if (no_count == 0 || no_count > 9)
        {
            LOG_INF("%s=>threshold is not a number: %s", __func__, msg->parm[2]);
            goto param_invalid;
        }

        no_count = string_check_is_number(0, msg->parm[3]);
        if (no_count == 0 || no_count > 9)
        {
            LOG_INF("%s=>interval is not a number: %s", __func__, msg->parm[3]);
            goto param_invalid;
        }

        threshold = (uint8_t)atoi(msg->parm[2]);
        interval = (uint8_t)atoi(msg->parm[3]);

        // 参数范围校验
        if (threshold < 10 || threshold > 50)
        {
            LOG_INF("%s=>threshold out of range: %d", __func__, threshold);
            goto param_invalid;
        }

        if (interval < 1 || interval > 48)
        {
            LOG_INF("%s=>interval out of range: %d", __func__, interval);
            goto param_invalid;
        }

        gConfigParam.lprunning_config.lprunning_sw = 1;
        gConfigParam.lprunning_config.lprunning_threshold = threshold;
        gConfigParam.lprunning_config.lprunning_interval = interval;
        gConfigParam.lprunning_config.flag = FLAG_VALID;
        my_user_data_write(ZMS_ID_LPSLEEP_CONFIG, &gConfigParam.lprunning_config, sizeof(lprunning_config_t));

        // 清除暂缓标志，允许下次电量低于阈值时立即进入低功耗运行
        my_send_msg(MOD_MAIN, MOD_MAIN, MY_MSG_LPSLEEP_CLEAR_HOLD_OFF);

        LOG_INF("%s=>%s,ON,%d,%d", __func__, msg->parm[0], threshold, interval);
        msg->resp_length = snprintf(msg->resp_msg, remaining, "Set OK");
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
    }
param_invalid:
    // 参数格式错误
    LOG_INF("%s=>%s, param error: count=%d", __func__, msg->parm[0], msg->parm_count);
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set Fail! InvalidParam");
    return BLE_DATA_TYPE_PACKET_MULTIPLE;
}

/********************************************************************
**函数名称:  startr_cmd_handler
**入口参数:  msg      ---        AT指令结构体指针
**出口参数:  msg->resp_msg  ---  响应消息
**           msg->resp_length --- 响应长度
**函数功能:  处理STARTR指令：设置数据记录功能开关
**指令格式:  查询指令: STARTR#
**           设置指令: STARTR,[A]#
**参数说明:  [A] - ON/OFF; ON:开启数据记录功能; OFF:关闭数据记录功能(默认)
**返 回 值:  BLE数据类型
*********************************************************************/
static int startr_cmd_handler(at_cmd_t* msg)
{
    uint16_t remaining;

    remaining = RESP_STRING_LENGTH_MAX;

    /* 检查参数数量：0表示查询，1表示设置 */
    if (msg->parm_count == 0)
    {
        /* 查询指令：返回当前状态 */
        LOG_INF("%s=>%s (query)", __func__, msg->parm[0]);
        if (gConfigParam.startr_config.startr_sw == 1)
        {
            msg->resp_length = snprintf(msg->resp_msg, remaining, "STARTR:ON");
        }
        else
        {
            msg->resp_length = snprintf(msg->resp_msg, remaining, "STARTR:OFF");
        }
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
    }

    if (msg->parm_count != 1)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        goto param_invalid;
    }

    /* 设置指令 */
    LOG_INF("%s=>%s,%s", __func__, msg->parm[0], msg->parm[1]);

    /* 解析A参数 */
    if (my_strcasecmp(msg->parm[1], "ON") == 0)
    {
        gConfigParam.startr_config.startr_sw = 1;
    }
    else if (my_strcasecmp(msg->parm[1], "OFF") == 0)
    {
        gConfigParam.startr_config.startr_sw = 0;
    }
    else
    {
        LOG_INF("%s=>invalid A param: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }

    /* 所有参数验证通过,生成成功响应 */
    gConfigParam.startr_config.flag = FLAG_VALID;
    /* 保存配置 */
    my_user_data_write(ZMS_ID_STARTR_CONFIG, &gConfigParam.startr_config, sizeof(startr_config_t));

    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set OK");
    LOG_INF("STARTR: SW=%d", gConfigParam.startr_config.startr_sw);

    //TODO 具体逻辑处理

    return BLE_DATA_TYPE_PACKET_MULTIPLE;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set Fail! InvalidParam");
    return BLE_DATA_TYPE_PACKET_MULTIPLE;
}

/********************************************************************
**函数名称:  cbmt_cmd_handler
**入口参数:  msg      ---        AT指令结构体指针
**出口参数:  msg->resp_msg  ---  响应消息
**           msg->resp_length --- 响应长度
**函数功能:  处理CBMT指令：查询内置电池电量、充电状态和温度
**指令格式:  CBMT#
**返回值说明: RETURN_CBMT:CHARGNIG=CHARGE_IN,VBAT=3000,VBATTEMP=37.50
**           CHARGNIG: 外电状态(CHARGE_IN为外电连接,CHARGE_OUT为外电断开)
**           VBAT: 读取电池本身电压(单位: mV)
**           VBATTEMP: 电池温度(单位: ℃)
**返 回 值:  BLE数据类型
*********************************************************************/
static int cbmt_cmd_handler(at_cmd_t* msg)
{
    uint16_t remaining;
    uint16_t battery_voltage_mv;
    const char* charge_status;
    int ret;

    remaining = RESP_STRING_LENGTH_MAX;

    /* 检查参数数量：应为0 */
    if (msg->parm_count == 0)
    {
        LOG_INF("%s=>%s", __func__, msg->parm[0]);

        battery_voltage_mv = get_batt_mv();
        if(g_charg_state == NO_CHARGING)
        {
            charge_status = "CHARGE_OUT";
        }
        else
        {
            charge_status = "CHARGE_IN";
        }

        /* 生成响应消息，格式：RETURN CBMT:CHARGNIG=XXX,VBAT=XXXX*/
        ret = snprintf(msg->resp_msg, remaining, "RETURN_CBMT:CHARGNIG=%s,VBAT=%u",
                      charge_status, battery_voltage_mv);

        if (ret > 0 && ret < remaining)
        {
            msg->resp_length = ret;
            LOG_INF("CBMT: %s", msg->resp_msg);
        }
        else
        {
            msg->resp_length = snprintf(msg->resp_msg, remaining, "Set Fail! InvalidParam");
        }
    }
    else
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        msg->resp_length = snprintf(msg->resp_msg, remaining, "Set Fail! InvalidParam");
    }
    return BLE_DATA_TYPE_PACKET_MULTIPLE;
}

/********************************************************************
**函数名称:  bt_mac_cmd_handler
**入口参数:  msg      ---        AT指令结构体指针
**出口参数:  msg->resp_msg  ---  响应消息
**          msg->resp_length --- 响应长度
**函数功能:  处理BT_MAC指令：查询蓝牙MAC地址
**指令格式:  BT_MAC#            查询蓝牙MAC地址
**返回值说明: BT_MAC:00:00:00:00:00:00  (MAC地址示例)
**返 回 值:  BLE数据类型
*********************************************************************/
static int bt_mac_cmd_handler(at_cmd_t* msg)
{
    uint16_t remaining;  // 响应消息缓冲区的剩余空间
    int ret;             // snprintf 函数的返回值
    uint8_t data_buff[6];

    remaining = RESP_STRING_LENGTH_MAX;  // 计算响应消息缓冲区的大小

    LOG_INF("%s=>%s", __func__, msg->parm[0]);
    if (msg->parm_count == 0)
    {
        memcpy(data_buff, gConfigParam.my_macaddr.hex, sizeof(gConfigParam.my_macaddr.hex));
        ret = snprintf(msg->resp_msg, remaining, "%s:%02x:%02x:%02x:%02x:%02x:%02x",
        msg->parm[0], data_buff[5], data_buff[4], data_buff[3], data_buff[2], data_buff[1], data_buff[0]);
        if (ret > 0 && ret < remaining)  // 检查响应消息是否生成成功
        {
            msg->resp_length = ret;  // 设置响应消息的长度
            LOG_INF("%s", msg->resp_msg);  // 输出版本号信息
        }
        else  // 响应消息生成失败
        {
            // 生成失败响应消息
            msg->resp_length = snprintf(msg->resp_msg, remaining, "Set Fail! InvalidParam");
        }
    }
    else
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        goto param_invalid;
    }

    return BLE_DATA_TYPE_PACKET_MULTIPLE;  // 返回 BLE 数据类型

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set Fail! InvalidParam");
    return BLE_DATA_TYPE_PACKET_MULTIPLE;
}

/********************************************************************
**函数名称:  bt_crfpwr_cmd_handler
**入口参数:  msg      ---        AT指令结构体指针
**出口参数:  msg->resp_msg  ---  响应消息
**           msg->resp_length --- 响应长度
**函数功能:  处理BT_CRFPWR指令：设置设备蓝牙发射功率
**指令格式:  BT_CRFPWR,[A]#
**参数说明:  A - 功率值(默认：0)，单位dBm，可选值：-8,-4,0,3,5,7,12
**返 回 值:  BLE数据类型
*********************************************************************/
static int bt_crfpwr_cmd_handler(at_cmd_t* msg)
{
    uint16_t remaining;
    uint8_t no_count = 0;
    int a_value;

    remaining = RESP_STRING_LENGTH_MAX;

    // 无参数即查询
    if (msg->parm_count == 0)
    {
        msg->resp_length = snprintf(msg->resp_msg, remaining, "%s:%d", msg->parm[0],
                                    gConfigParam.ble_tx_power.tx_power);
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
    }

    /* 检查参数数量 */
    if (msg->parm_count != 1)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        goto param_invalid;
    }

    no_count = string_check_is_number(1, msg->parm[1]);
    if (no_count == 0 || no_count > 9)
    {
        LOG_INF("%s=>invalid A param: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }
    /* 解析A参数 */
    a_value = atoi(msg->parm[1]);
    if (a_value != -8 && a_value != -4 && a_value != 0 && a_value != 3
        && a_value != 5 && a_value != 7)
    {
        LOG_INF("%s=>invalid A param: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }

    /* 所有参数验证通过,统一赋值 */
    gConfigParam.ble_tx_power.flag = FLAG_VALID;
    gConfigParam.ble_tx_power.tx_power = (int8_t)a_value;
    ble_set_tx_power(gConfigParam.ble_tx_power.tx_power);

    /* 保存配置 */
    my_user_data_write(ZMS_ID_BLE_TX_POWER, &gConfigParam.ble_tx_power, sizeof(ble_tx_power_t));

    LOG_INF("%s=>%s,%s", __func__, msg->parm[0], msg->parm[1]);

    /* 所有参数验证通过,生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set OK!");
    LOG_INF("BT_CRFPWR: A=%d",gConfigParam.ble_tx_power.tx_power);

    return BLE_DATA_TYPE_PACKET_MULTIPLE;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set Fail! InvalidParam");
    return BLE_DATA_TYPE_PACKET_MULTIPLE;
}

/********************************************************************
**函数名称:  bt_updata_cmd_handler
**入口参数:  msg      ---        AT指令结构体指针
**出口参数:  msg->resp_msg  ---  响应消息
**           msg->resp_length --- 响应长度
**函数功能:  处理BT_UPDATA指令：设置蓝牙数据收集上传策略
**指令格式:  BT_UPDATA,[Mode],[Scan Interval],[Scan Length],[Updata interval]#
**参数说明:  Mode - 工作方式(默认：0)
**           0：不开启蓝牙搜索收集功能
**           1：设备持续按[Scan Interval]和[Scan Length]收集数据，Cell启动时上传，未启动时仅存储
**           2：设备持续收集数据，Cell启动时上传；未启动时若距离上次上传达到[Updata interval]，则唤醒Cell和GNSS上传
**           Scan Interval - 蓝牙数据收集间隔(默认：600秒)，范围：5-86400秒
**           Scan Length - 每次收集的搜索时长(默认：10秒)，范围：5-86400秒
**           Updata interval - 蓝牙唤醒间隔(默认：14400秒)，范围：120-86400秒
**返 回 值:  BLE数据类型
*********************************************************************/
static int bt_updata_cmd_handler(at_cmd_t* msg)
{
    uint16_t remaining;
    uint8_t no_count = 0;
    int mode_value;
    int scan_interval_value;
    int scan_length_value;
    int updata_interval_value;

    remaining = RESP_STRING_LENGTH_MAX;

    // 无参数即查询
    if (msg->parm_count == 0)
    {
        msg->resp_length = snprintf(msg->resp_msg, remaining, "%s:%d,%d,%d,%d",
                                    msg->parm[0],
                                    gConfigParam.bt_updata_config.bt_updata_mode,
                                    gConfigParam.bt_updata_config.bt_updata_scan_interval,
                                    gConfigParam.bt_updata_config.bt_updata_scan_length,
                                    gConfigParam.bt_updata_config.bt_updata_updata_interval
        );
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
    }

    if (get_lprunning_active() == true)
    {
        LOG_INF("%s=>%s, LPRunning active, not support BT_UPDATA", __func__, msg->parm[0]);
        msg->resp_length = snprintf(msg->resp_msg, remaining, "Set Fail! Unauthorized");
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
    }

    /* 检查参数数量 */
    if (msg->parm_count != 4)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        goto param_invalid;
    }

    no_count = string_check_is_number(0, msg->parm[1]);
    if (no_count == 0 || no_count > 9)
    {
        LOG_INF("%s=>invalid Mode param: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }
    /* 解析Mode参数 */
    mode_value = atoi(msg->parm[1]);
    if (mode_value < 0 || mode_value > 2)
    {
        LOG_INF("%s=>invalid Mode param: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }

    no_count = string_check_is_number(0, msg->parm[2]);
    if (no_count == 0 || no_count > 9)
    {
        LOG_INF("%s=>invalid Scan Interval param: %s", __func__, msg->parm[2]);
        goto param_invalid;
    }
    /* 解析Scan Interval参数 */
    scan_interval_value = atol(msg->parm[2]);
    if (scan_interval_value < 5 || scan_interval_value > 86400)
    {
        LOG_INF("%s=>invalid Scan Interval param: %s", __func__, msg->parm[2]);
        goto param_invalid;
    }

    no_count = string_check_is_number(0, msg->parm[3]);
    if (no_count == 0 || no_count > 9)
    {
        LOG_INF("%s=>invalid Scan Length param: %s", __func__, msg->parm[3]);
        goto param_invalid;
    }
    /* 解析Scan Length参数 */
    scan_length_value = atol(msg->parm[3]);
    if (scan_length_value < 5 || scan_length_value > 86400)
    {
        LOG_INF("%s=>invalid Scan Length param: %s", __func__, msg->parm[3]);
        goto param_invalid;
    }

    no_count = string_check_is_number(0, msg->parm[4]);
    if (no_count == 0 || no_count > 9)
    {
        LOG_INF("%s=>invalid Updata interval param: %s", __func__, msg->parm[4]);
        goto param_invalid;
    }
    /* 解析Updata interval参数 */
    updata_interval_value = atol(msg->parm[4]);
    if (updata_interval_value < 120 || updata_interval_value > 86400)
    {
        LOG_INF("%s=>invalid Updata interval param: %s", __func__, msg->parm[4]);
        goto param_invalid;
    }

    // 检查Scan Interval是否大于Scan Length
    if (scan_interval_value <= scan_length_value)
    {
        LOG_INF("%s=>interval must be greater than length", __func__);
        goto param_invalid;
    }

    // 建议upload_interval应该大于scan_interval（避免频繁上报）
    if (mode_value == 3 && updata_interval_value <= scan_interval_value)
    {
        LOG_INF("%s=>Warning: upload_interval(%u) should be greater than scan_interval(%u)",
                __func__, updata_interval_value, scan_interval_value);
        goto param_invalid;
    }

    /* 所有参数验证通过,统一赋值 */
    gConfigParam.bt_updata_config.flag = FLAG_VALID;
    gConfigParam.bt_updata_config.bt_updata_mode = (uint8_t)mode_value;
    gConfigParam.bt_updata_config.bt_updata_scan_interval = scan_interval_value;
    gConfigParam.bt_updata_config.bt_updata_scan_length = scan_length_value;
    gConfigParam.bt_updata_config.bt_updata_updata_interval = updata_interval_value;

    /* 保存配置 */
    my_user_data_write(ZMS_ID_BT_UPDATA_CONFIG, &gConfigParam.bt_updata_config, sizeof(bt_updata_config_t));

    LOG_INF("%s=>%s,%s,%s,%s,%s", __func__, msg->parm[0], msg->parm[1],
           msg->parm[2], msg->parm[3], msg->parm[4]);

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set OK!");
    LOG_INF("BT_UPDATA: Mode=%d, ScanInterval=%u, ScanLength=%u, UpdataInterval=%u",
           gConfigParam.bt_updata_config.bt_updata_mode,
           gConfigParam.bt_updata_config.bt_updata_scan_interval,
           gConfigParam.bt_updata_config.bt_updata_scan_length,
           gConfigParam.bt_updata_config.bt_updata_updata_interval);

    // 应用TAG扫描配置
    my_scan_set_config(gConfigParam.bt_updata_config.bt_updata_mode,
                           gConfigParam.bt_updata_config.bt_updata_scan_interval,
                           gConfigParam.bt_updata_config.bt_updata_scan_length,
                           gConfigParam.bt_updata_config.bt_updata_updata_interval);

    return BLE_DATA_TYPE_PACKET_MULTIPLE;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set Fail! InvalidParam");
    return BLE_DATA_TYPE_PACKET_MULTIPLE;
}

/********************************************************************
**函数名称:  bluetooth_cmd_handler
**入口参数:  msg      ---        AT指令结构体指针
**出口参数:  msg->resp_msg  ---  响应消息
**           msg->resp_length --- 响应长度
**函数功能:  处理blueuetooth指令：设置蓝牙广播是否开启和开启间隔
**指令格式:  blueuetooth,[SW],[A],[B]#
**参数说明:  SW - 广播状态
**              ON - 开启广播
**              OFF - 关闭广播
**          A - 广播模式
**          B - 广播间隔
**返 回 值:  BLE数据类型
*********************************************************************/
static int bluetooth_cmd_handler(at_cmd_t* msg)
{
    uint16_t remaining;
    uint8_t no_count = 0;
    int sw = 0;
    int a = 0;
    int b = 0;

    remaining = RESP_STRING_LENGTH_MAX;

    // 无参数即查询
    if (msg->parm_count == 0)
    {
        const char* state_str = gConfigParam.bluetooth_config.bluetooth_sw ? "ON" : "OFF";
        if (gConfigParam.bluetooth_config.bluetooth_flag)
        {
            msg->resp_length = snprintf(msg->resp_msg, remaining, "%s:%s,%d,%d", msg->parm[0], state_str, gConfigParam.bluetooth_config.bluetooth_a, gConfigParam.bluetooth_config.bluetooth_b);
        }
        else
        {
            msg->resp_length = snprintf(msg->resp_msg, remaining, "%s:%s", msg->parm[0], state_str);
        }
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
    }

    //不携带参数
    if (msg->parm_count == 1)
    {
        if (my_strcasecmp(msg->parm[1], "ON") == 0)
        {
            gConfigParam.bluetooth_config.bluetooth_sw = 1;
        }
        else if (my_strcasecmp(msg->parm[1], "OFF") == 0)
        {
            gConfigParam.bluetooth_config.bluetooth_sw = 0;
        }
        else
        {
            LOG_INF("%s=>invalid param: %s", __func__, msg->parm[1]);
            goto param_invalid;
        }

        gConfigParam.bluetooth_config.flag = FLAG_VALID;
        gConfigParam.bluetooth_config.bluetooth_flag = 0;

        if (gConfigParam.bluetooth_config.bluetooth_sw)
        {
            my_send_msg(MOD_BLE, MOD_BLE, MY_MSG_BLE_OPEN_ADV);
        }
        else
        {
            my_send_msg(MOD_BLE, MOD_BLE, MY_MSG_BLE_CLOSE_ADV);
        }

        LOG_INF("%s=>%s:%s", __func__, msg->parm[0], msg->parm[1]);
    }
    //携带参数
    else if (msg->parm_count == 3)
    {
        if (my_strcasecmp(msg->parm[1], "ON") == 0)
        {
            sw = 1;
        }
        else
        {
            LOG_INF("%s=>invalid param: %s", __func__, msg->parm[1]);
            goto param_invalid;
        }

        no_count = string_check_is_number(0, msg->parm[2]);
        if (no_count == 0 || no_count > 9)
        {
            LOG_INF("%s=>invalid param: %s", __func__, msg->parm[2]);
            goto param_invalid;
        }
        a = atoi(msg->parm[2]);
        if (a != 5)
        {
            LOG_INF("%s=>invalid param: %s", __func__, msg->parm[2]);
            goto param_invalid;
        }

        no_count = string_check_is_number(0, msg->parm[3]);
        if (no_count == 0 || no_count > 9)
        {
            LOG_INF("%s=>invalid param: %s", __func__, msg->parm[3]);
            goto param_invalid;
        }
        b = atoi(msg->parm[3]);
        if (b > 30)
        {
            LOG_INF("%s=>invalid param: %s", __func__, msg->parm[3]);
            goto param_invalid;
        }

        gConfigParam.bluetooth_config.flag = FLAG_VALID;
        gConfigParam.bluetooth_config.bluetooth_sw = sw;
        gConfigParam.bluetooth_config.bluetooth_a = a;
        gConfigParam.bluetooth_config.bluetooth_b = b;
        gConfigParam.bluetooth_config.bluetooth_flag = 1;

        my_send_msg(MOD_BLE, MOD_BLE, MY_MSG_BLE_CLOSE_ADV);

        LOG_INF("%s=>%s:%s,%s,%s", __func__, msg->parm[0], msg->parm[1], msg->parm[2], msg->parm[3]);
    }
    else
    {
        LOG_INF("%s=>param count error: %d", __func__, msg->parm_count);
        goto param_invalid;
    }

    /* 保存配置 */
    my_user_data_write(ZMS_ID_BLUETOOTH_CONFIG, &gConfigParam.bluetooth_config, sizeof(bluetooth_config_t));

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set OK!");

    return BLE_DATA_TYPE_PACKET_MULTIPLE;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set Fail! InvalidParam");
    return BLE_DATA_TYPE_PACKET_MULTIPLE;
}

/********************************************************************
**函数名称:  btconnect_cmd_handler
**入口参数:  msg      ---        AT指令结构体指针
**出口参数:  msg->resp_msg  ---  响应消息
**           msg->resp_length --- 响应长度
**函数功能:  处理BTCONNECT指令：设置蓝牙连接功能开关和参数
**指令格式:  BTCONNECT,[SW],[Interval],[Report]#
**参数说明:  SW - 功能开关(默认：OFF)
**              ON - 开启蓝牙连接功能
**              OFF - 关闭蓝牙连接功能
**           Interval - 连接间隔(默认：300秒)，范围：300-43200秒
**           Report - 上报模式(默认：0)，范围：0-3
**返 回 值:  BLE数据类型
*********************************************************************/
static int btconnect_cmd_handler(at_cmd_t* msg)
{
    uint16_t remaining;
    uint8_t no_count = 0;
    int sw_value;
    int interval_value;
    int report_value;

    remaining = RESP_STRING_LENGTH_MAX;

    //无参数即查询
    if (msg->parm_count == 0)
    {
        // 根据 btconnect_sw 的值选择 "ON" 或 "OFF"
        const char* state_str = gConfigParam.btconnect_config.btconnect_sw ? "ON" : "OFF";

        msg->resp_length = snprintf(msg->resp_msg, remaining, "%s:%s,%d,%d", msg->parm[0],
                                    state_str, gConfigParam.btconnect_config.btconnect_interval, gConfigParam.btconnect_config.btconnect_report);
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
    }

    /* 检查参数数量 */
    if (msg->parm_count != 3)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        goto param_invalid;
    }

    /* 解析SW参数 */
    if (my_strcasecmp(msg->parm[1], "ON") == 0)
    {
        sw_value = 1;
    }
    else if (my_strcasecmp(msg->parm[1], "OFF") == 0)
    {
        sw_value = 0;
    }
    else
    {
        LOG_INF("%s=>invalid SW param: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }

    no_count = string_check_is_number(0, msg->parm[2]);
    if (no_count == 0 || no_count > 9)
    {
        LOG_INF("%s=>invalid Interval param: %s", __func__, msg->parm[2]);
        goto param_invalid;
    }

    /* 解析Interval参数 */
    interval_value = atoi(msg->parm[2]);
    if (interval_value < 300 || interval_value > 43200)
    {
        LOG_INF("%s=>invalid Interval param: %s", __func__, msg->parm[2]);
        goto param_invalid;
    }

    no_count = string_check_is_number(0, msg->parm[3]);
    if (no_count == 0 || no_count > 9)
    {
        LOG_INF("%s=>invalid Report param: %s", __func__, msg->parm[3]);
        goto param_invalid;
    }

    /* 解析Report参数 */
    report_value = atoi(msg->parm[3]);
    if (report_value < 0 || report_value > 3)
    {
        LOG_INF("%s=>invalid Report param: %s", __func__, msg->parm[3]);
        goto param_invalid;
    }

    /* 所有参数验证通过,统一赋值 */
    gConfigParam.btconnect_config.flag = FLAG_VALID;
    gConfigParam.btconnect_config.btconnect_sw = (uint8_t)sw_value;
    gConfigParam.btconnect_config.btconnect_interval = (uint16_t)interval_value;
    gConfigParam.btconnect_config.btconnect_report = (uint8_t)report_value;

    /* 保存配置 */
    my_user_data_write(ZMS_ID_BTCONNECT_CONFIG, &gConfigParam.btconnect_config, sizeof(btconnect_config_t));

    LOG_INF("%s=>%s,%s,%s,%s", __func__, msg->parm[0], msg->parm[1], msg->parm[2], msg->parm[3]);

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set OK");
    LOG_INF("BTCONNECT: SW=%d, Interval=%d, Report=%d", gConfigParam.btconnect_config.btconnect_sw, gConfigParam.btconnect_config.btconnect_interval, gConfigParam.btconnect_config.btconnect_report);

    return BLE_DATA_TYPE_PACKET_MULTIPLE;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set Fail! InvalidParam");
    return BLE_DATA_TYPE_PACKET_MULTIPLE;
}

/********************************************************************
**函数名称:  tag_cmd_handler
**入口参数:  msg      ---        AT指令结构体指针
**出口参数:  msg->resp_msg  ---  响应消息
**           msg->resp_length --- 响应长度
**函数功能:  处理TAG指令：设置Tag定位功能和详细参数
**指令格式:  TAG,[SW],[Interval]#
**兼容指令:  TAG,ON#（按默认或已设置参数开启功能）
**参数说明:  SW - 功能开关(默认：OFF)
**           ON：开启
**           OFF：关闭
**           Interval - 广播间隔(默认：2000ms)，范围：100ms-60000ms(分辨率100ms)
**返 回 值:  BLE数据类型
*********************************************************************/
static int tag_cmd_handler(at_cmd_t* msg)
{
    uint16_t remaining;
    uint8_t no_count = 0;
    int interval_value;
    int sw_value;

    remaining = RESP_STRING_LENGTH_MAX;

    // 无参数即查询
    if (msg->parm_count == 0)
    {
        // 根据 tag_sw 的值选择 "ON" 或 "OFF"
        const char* state_str = gConfigParam.tag_config.tag_sw ? "ON" : "OFF";
        msg->resp_length = snprintf(msg->resp_msg, remaining, "%s:%s,%d",
                                    msg->parm[0],
                                    state_str,
                                    gConfigParam.tag_config.tag_interval
        );
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
    }


    /* 检查参数数量：支持1个参数(TAG,ON)或2个参数(TAG,SW,Interval) */
    if (msg->parm_count != 1 && msg->parm_count != 2)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        goto param_invalid;
    }

    /* 解析SW参数 */
    if (my_strcasecmp(msg->parm[1], "ON") == 0)
    {
        sw_value = 1;
    }
    else if (my_strcasecmp(msg->parm[1], "OFF") == 0)
    {
        sw_value = 0;
    }
    else
    {
        LOG_INF("%s=>invalid SW param: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }

    /* 如果有Interval参数，则解析 */
    if (msg->parm_count == 2)
    {
        no_count = string_check_is_number(0, msg->parm[2]);
        if (no_count == 0 || no_count > 9)
        {
            LOG_INF("%s=>invalid Interval param: %s", __func__, msg->parm[2]);
            goto param_invalid;
        }
        interval_value = atoi(msg->parm[2]);
        if (interval_value < 100 || interval_value > 60000)
        {
            LOG_INF("%s=>invalid Interval param: %s", __func__, msg->parm[2]);
            goto param_invalid;
        }
    }

    /* 所有参数验证通过,统一赋值 */
    gConfigParam.tag_config.flag = FLAG_VALID;
    gConfigParam.tag_config.tag_sw = (uint8_t)sw_value;
    if (msg->parm_count == 2)
    {
        gConfigParam.tag_config.tag_interval = (uint16_t)interval_value;
    }

    /* 保存配置 */
    my_user_data_write(ZMS_ID_TAG_CONFIG, &gConfigParam.tag_config, sizeof(tag_config_t));

    LOG_INF("%s=>%s,%s", __func__, msg->parm[0], msg->parm[1]);
    if (msg->parm_count == 2)
    {
        LOG_INF("%s=>%s", __func__, msg->parm[2]);
    }

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set OK!");
    LOG_INF("TAG: SW=%d, Interval=%u", gConfigParam.tag_config.tag_sw, gConfigParam.tag_config.tag_interval);

    //更新非连接广播参数，里面会按配置打开或关闭广播，根据tag_sw的值
    my_ble_updata_adv_param(gConfigParam.tag_config.tag_interval);

    return BLE_DATA_TYPE_PACKET_MULTIPLE;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set Fail! InvalidParam");
    return BLE_DATA_TYPE_PACKET_MULTIPLE;
}

/********************************************************************
**函数名称:  jatag_cmd_handler
**入口参数:  msg      ---        AT指令结构体指针
**出口参数:  msg->resp_msg  ---  响应消息
**           msg->resp_length --- 响应长度
**函数功能:  处理JATAG指令：设置JATag定位功能开关
**指令格式:  JATAG,[SW]#
**兼容指令:  JATAG,ON#（按默认或已设置参数开启功能）
**参数说明:  SW - 功能开关(默认：OFF)
**           ON：开启
**           OFF：关闭
**返 回 值:  BLE数据类型
*********************************************************************/
static int jatag_cmd_handler(at_cmd_t* msg)
{
    uint16_t remaining;
    int sw_value;

    remaining = RESP_STRING_LENGTH_MAX;

    // 无参数即查询
    if (msg->parm_count == 0)
    {
        const char* state_str = gConfigParam.adv_valid_value.AppleValid ? "ON" : "OFF";
        msg->resp_length = snprintf(msg->resp_msg, remaining, "%s:%s",
                                    msg->parm[0],
                                    state_str
        );
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
    }

    /* 检查参数数量：支持1个参数(JATAG,ON)*/
    if (msg->parm_count != 1)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        goto param_invalid;
    }

    /* 解析SW参数 */
    if (my_strcasecmp(msg->parm[1], "ON") == 0)
    {
        sw_value = 1;
    }
    else if (my_strcasecmp(msg->parm[1], "OFF") == 0)
    {
        sw_value = 0;
    }
    else
    {
        LOG_INF("%s=>invalid SW param: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }

    if (gConfigParam.adv_valid_value.GoogleValid == 0)
    {
        LOG_INF("JATAG: GoogleValid is 0");
        goto param_invalid;
    }
    /* 所有参数验证通过,统一赋值 */
    gConfigParam.adv_valid_value.flag = FLAG_VALID;
    gConfigParam.adv_valid_value.AppleValid = (uint8_t)sw_value;
    set_adv_valid_status(APPLE_ADV_TYPE, gConfigParam.adv_valid_value.AppleValid);
    my_no_con_start_adv(gConfigParam.tag_config.tag_sw);

    /* 保存配置 */
    my_user_data_write(ZMS_ID_ADV_VALID, &gConfigParam.adv_valid_value, sizeof(adv_valid_value_t));

    LOG_INF("%s=>%s,%s", __func__, msg->parm[0], msg->parm[1]);

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set OK!");
    LOG_INF("JATAG: SW=%d, Interval=%u", gConfigParam.adv_valid_value.AppleValid, gConfigParam.tag_config.tag_interval);

    return BLE_DATA_TYPE_PACKET_MULTIPLE;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set Fail! InvalidParam");
    return BLE_DATA_TYPE_PACKET_MULTIPLE;
}

/********************************************************************
**函数名称:  jgtag_cmd_handler
**入口参数:  msg      ---        AT指令结构体指针
**出口参数:  msg->resp_msg  ---  响应消息
**           msg->resp_length --- 响应长度
**函数功能:  处理JGTAG指令：设置JGTAG定位功能开关
**指令格式:  JGTAG,[SW]#
**兼容指令:  JGTAG,ON#（按默认或已设置参数开启功能）
**参数说明:  SW - 功能开关(默认：OFF)
**           ON：开启
**           OFF：关闭
**返 回 值:  BLE数据类型
*********************************************************************/
static int jgtag_cmd_handler(at_cmd_t* msg)
{
    uint16_t remaining;
    int sw_value;

    remaining = RESP_STRING_LENGTH_MAX;

    // 无参数即查询
    if (msg->parm_count == 0)
    {
        const char* state_str = gConfigParam.adv_valid_value.GoogleValid ? "ON" : "OFF";
        msg->resp_length = snprintf(msg->resp_msg, remaining, "%s:%s",
                                    msg->parm[0],
                                    state_str
        );
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
    }

    /* 检查参数数量：支持1个参数(JGTAG,ON)*/
    if (msg->parm_count != 1)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        goto param_invalid;
    }

    /* 解析SW参数 */
    if (my_strcasecmp(msg->parm[1], "ON") == 0)
    {
        sw_value = 1;
    }
    else if (my_strcasecmp(msg->parm[1], "OFF") == 0)
    {
        sw_value = 0;
    }
    else
    {
        LOG_INF("%s=>invalid SW param: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }

    if (gConfigParam.adv_valid_value.AppleValid == 0)
    {
        LOG_INF("JGTAG: AppleValid is 0");
        goto param_invalid;
    }
    /* 所有参数验证通过,统一赋值 */
    gConfigParam.adv_valid_value.flag = FLAG_VALID;
    gConfigParam.adv_valid_value.GoogleValid = (uint8_t)sw_value;
    set_adv_valid_status(GOOGLE_ADV_TYPE, gConfigParam.adv_valid_value.GoogleValid);
    my_no_con_start_adv(gConfigParam.tag_config.tag_sw);

    /* 保存配置 */
    my_user_data_write(ZMS_ID_ADV_VALID, &gConfigParam.adv_valid_value, sizeof(adv_valid_value_t));

    LOG_INF("%s=>%s,%s", __func__, msg->parm[0], msg->parm[1]);

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set OK!");
    LOG_INF("JGTAG: SW=%d, Interval=%u", gConfigParam.adv_valid_value.GoogleValid, gConfigParam.tag_config.tag_interval);

    return BLE_DATA_TYPE_PACKET_MULTIPLE;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set Fail! InvalidParam");
    return BLE_DATA_TYPE_PACKET_MULTIPLE;
}

/********************************************************************
**函数名称:  taginit_param_cmd_handler
**入口参数:  msg      ---        AT指令结构体指针
**出口参数:  msg->resp_msg  ---  响应消息
**          msg->resp_length --- 响应长度
**函数功能:  处理TAGINIT指令：远程下发FF和SN
**指令格式:  TAGINIT_PARAM#
**          TAGINIT_PARAM,FF,SN#
**参数说明:  FF - 许证FF
**          SN - 许证SN
**返 回 值:  BLE数据类型
*********************************************************************/
static int taginit_param_cmd_handler(at_cmd_t* msg)
{
    uint16_t remaining;
    uint8_t ff_buff[LICENSE_FF_STR_LEN + 1] = {0};
    uint8_t sn_buff[GSM_SN_LENGTH + 1] = {0};

    remaining = RESP_STRING_LENGTH_MAX;

    // 无参数即查询
    if (msg->parm_count == 0)
    {
        hex2hexstr(gConfigParam.lic_ff.hex, sizeof(gConfigParam.lic_ff.hex), ff_buff, sizeof(ff_buff));
        memcpy(sn_buff, gConfigParam.gsm_sn.hex, sizeof(gConfigParam.gsm_sn.hex));
        msg->resp_length = snprintf(msg->resp_msg, remaining, "TAG:FF:%s;SN:%s", ff_buff, sn_buff);
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
    }

    /* 检查参数数量 */
    if (msg->parm_count != 2)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        goto param_invalid;
    }

    if (!my_param_check_license(msg->parm[1], strlen(msg->parm[1]), ZMS_ID_FF))
    {
        LOG_INF("my_param_check_license FF error!");
        goto param_invalid;
    }
    if (!my_param_check_license(msg->parm[2], strlen(msg->parm[2]), ZMS_ID_SN))
    {
        LOG_INF("my_param_check_license SN error!");
        goto param_invalid;
    }

    gConfigParam.lic_ff.flag = FLAG_VALID;
    hexstr_to_hex((uint8_t *)gConfigParam.lic_ff.hex, sizeof(gConfigParam.lic_ff.hex), msg->parm[1]);
    my_user_data_write(ZMS_ID_FF, &gConfigParam.lic_ff, sizeof(lic_ff_t));

    gConfigParam.gsm_sn.flag = FLAG_VALID;
    memcpy(gConfigParam.gsm_sn.hex, msg->parm[2], sizeof(gConfigParam.gsm_sn.hex));
    my_user_data_write(ZMS_ID_SN, &gConfigParam.gsm_sn, sizeof(gsm_sn_t));

    LOG_INF("%s=>%s,%s,%s", __func__, msg->parm[0], msg->parm[1], msg->parm[2]);

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set OK!");

    return BLE_DATA_TYPE_PACKET_MULTIPLE;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set Fail! InvalidParam");
    return BLE_DATA_TYPE_PACKET_MULTIPLE;
}

/********************************************************************
**函数名称:  led_cmd_handler
**入口参数:  msg      ---        AT指令结构体指针
**出口参数:  msg->resp_msg  ---  响应消息
**           msg->resp_length --- 响应长度
**函数功能:  处理LED指令：控制设备LED指示灯的显示状态
**指令格式:  LED,A#
**参数说明:  A - 设备LED是否全时显示，可选值：OFF(关闭，默认)、ON(开启)
**返 回 值:  BLE数据类型
*********************************************************************/
static int led_cmd_handler(at_cmd_t* msg)
{
    uint16_t remaining;
    uint8_t no_count = 0;
    int mode = 0;

    remaining = RESP_STRING_LENGTH_MAX;

    // 无参数即查询
    if (msg->parm_count == 0)
    {
        msg->resp_length = snprintf(msg->resp_msg, remaining, "%s:%d", msg->parm[0], gConfigParam.led_config.led_display);
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
    }

    /* 检查参数数量 */
    if (msg->parm_count != 1)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        goto param_invalid;
    }

    no_count = string_check_is_number(0, msg->parm[1]);

    if (no_count == 0 || no_count > 9)
    {
        LOG_INF("%s=>invalid A param: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }

    mode = atoi(msg->parm[1]);

    if (mode > 2)
    {
         LOG_INF("%s=>invalid A param: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }

    /* 所有参数验证通过,统一赋值 */
    gConfigParam.led_config.flag = FLAG_VALID;
    gConfigParam.led_config.led_display = (uint8_t)mode;
    if (gConfigParam.led_config.led_display == 2)
    {
        my_send_msg(MOD_BLE, MOD_CTRL, MY_MSG_LED_ENABLE);
    }
    else
    {
        my_send_msg(MOD_BLE, MOD_CTRL, MY_MSG_LED_DISABLE);
    }

    LOG_INF("LED: Display=%d", gConfigParam.led_config.led_display);

    // 只有在LTE就绪状态下才发送LED指令
    if (g_bLteReady == true)
    {
        send_led_command();
    }

    /* 保存配置 */
    my_user_data_write(ZMS_ID_LED_CONFIG, &gConfigParam.led_config, sizeof(led_config_t));

    LOG_INF("%s=>%s,%s", __func__, msg->parm[0], msg->parm[1]);

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set OK!");
    LOG_INF("LED: Display=%d", gConfigParam.led_config.led_display);

    return BLE_DATA_TYPE_PACKET_MULTIPLE;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set Fail! InvalidParam");
    return BLE_DATA_TYPE_PACKET_MULTIPLE;
}

/*********************************************************************
**函数名称:  ltint_cmd_handler
**入口参数:  msg      ---        AT指令结构体指针
**出口参数:  msg->resp_msg  ---  响应消息
**           msg->resp_length --- 响应长度
**函数功能:  处理LTINT指令：控制光感过滤
**指令格式:  LTINT,[T1],[T2]#    - 开启光感过滤
**           LTINT#       - 查询光感过滤参数
**参数说明:  T1  - 检测到光的连续时间超过T1时，切换为“Light”状态 100~5000ms
**          T2  - 检测到暗的连续时间超过T2时，切换为“Dark”状态 100~5000ms
**          无参数 - 查询当前状态
**返 回 值:  BLE数据类型
*********************************************************************/
static int ltint_cmd_handler(at_cmd_t* msg)
{
    uint16_t remaining;
    int T1, T2;
    uint8_t no_count = 0;

    remaining = RESP_STRING_LENGTH_MAX;

    // 无参数即查询
    if (msg->parm_count == 0)
    {
        msg->resp_length = snprintf(msg->resp_msg, remaining, "%s:%d,%d",msg->parm[0],gConfigParam.ltint_config.T1,gConfigParam.ltint_config.T2);
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
    }

    /* 检查参数数量 */
    if (msg->parm_count != 2)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        goto param_invalid;
    }

    no_count = string_check_is_number(0, msg->parm[1]);
    if (no_count == 0 || no_count > 9)
    {
        LOG_INF("%s=>invalid T1 param: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }

    no_count = string_check_is_number(0, msg->parm[2]);
    if (no_count == 0 || no_count > 9)
    {
        LOG_INF("%s=>invalid T2 param: %s", __func__, msg->parm[2]);
        goto param_invalid;
    }

    T1 = atoi(msg->parm[1]);
    T2 = atoi(msg->parm[2]);

    if (T1 < 100 || T1 > 5000 || T2 < 100 || T2 > 5000)
    {
        LOG_INF("%s=>invalid T1 or T2 param: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }

    gConfigParam.ltint_config.flag = FLAG_VALID;
    gConfigParam.ltint_config.T1 = T1;
    gConfigParam.ltint_config.T2 = T2;

    /* 保存配置 */
    my_user_data_write(ZMS_ID_LTINT_CONFIG, &gConfigParam.ltint_config, sizeof(ltint_config_t));

    LOG_INF("%s=>%s,%s", __func__, msg->parm[0], msg->parm[1]);

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set OK!");
    LOG_INF("LTINT: T1=%d,T2=%d", gConfigParam.ltint_config.T1,gConfigParam.ltint_config.T2);

    return BLE_DATA_TYPE_PACKET_MULTIPLE;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set Fail! InvalidParam");
    return BLE_DATA_TYPE_PACKET_MULTIPLE;
}

/********************************************************************
**函数名称:  buzzer_cmd_handler
**入口参数:  msg      ---        AT指令结构体指针
**出口参数:  msg->resp_msg  ---  响应消息
**           msg->resp_length --- 响应长度
**函数功能:  处理BUZZER指令：直接控制设备蜂鸣器的不同提示音模式
**指令格式:  BUZZER,[Operater]#
**参数说明:  Operater - 蜂鸣器操作
**           0：停止蜂鸣器
**           1：持续报警(200ms ON，500ms OFF，不停止)
**           2：成功提示音(500ms ON)
**           3：失败提示音(200ms ON，200ms OFF，响3声)
**           4：异常提示音(100ms ON，100ms OFF，持续1s)
**           5：一般报警音(200ms ON，300ms OFF，持续30s)
**返 回 值:  BLE数据类型
*********************************************************************/
static int buzzer_cmd_handler(at_cmd_t* msg)
{
    uint16_t remaining;
    uint8_t no_count = 0;
    int operator_value;

    remaining = RESP_STRING_LENGTH_MAX;

    // 无参数即查询
    if (msg->parm_count == 0)
    {
        msg->resp_length = snprintf(msg->resp_msg, remaining, "%s:%d",
                                    msg->parm[0],
                                    gConfigParam.buzzer_config.buzzer_operator
        );
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
    }

    /* 检查参数数量 */
    if (msg->parm_count != 1)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        goto param_invalid;
    }

    no_count = string_check_is_number(0, msg->parm[1]);
    if (no_count == 0 || no_count > 9)
    {
        LOG_INF("%s=>invalid Operater param: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }
    /* 解析Operater参数 */
    operator_value = atoi(msg->parm[1]);
    if (operator_value < 0 || operator_value > 5)
    {
        LOG_INF("%s=>invalid Operater param: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }

    /* 所有参数验证通过,统一赋值 */
    gConfigParam.buzzer_config.flag = FLAG_VALID;
    gConfigParam.buzzer_config.buzzer_operator = (uint8_t)operator_value;

    /* 保存配置 */
    my_user_data_write(ZMS_ID_BUZZER_CONFIG, &gConfigParam.buzzer_config, sizeof(buzzer_config_t));

    LOG_INF("%s=>%s,%s", __func__, msg->parm[0], msg->parm[1]);

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set OK!");
    LOG_INF("BUZZER: Operator=%d", gConfigParam.buzzer_config.buzzer_operator);

    //TODO 具体逻辑处理
    my_set_buzzer_mode(operator_value);

    return BLE_DATA_TYPE_PACKET_MULTIPLE;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set Fail! InvalidParam");
    return BLE_DATA_TYPE_PACKET_MULTIPLE;
}

/*********************************************************************
**函数名称:  btlog_cmd_handler
**入口参数:  msg      ---        AT指令结构体指针
**出口参数:  msg->resp_msg  ---  响应消息
**           msg->resp_length --- 响应长度
**函数功能:  处理BTLOG指令：控制蓝牙日志开关
**指令格式:  BTLOG,ON#    - 开启蓝牙日志
**           BTLOG,OFF#   - 关闭蓝牙日志
**           BTLOG#       - 查询蓝牙日志状态
**参数说明:  ON  - 开启蓝牙日志总开关
**           OFF - 关闭蓝牙日志总开关
**           无参数 - 查询当前状态
**返 回 值:  BLE数据类型
*********************************************************************/
static int btlog_cmd_handler(at_cmd_t* msg)
{
    uint16_t remaining;
    ble_log_config_t *config;

    remaining = RESP_STRING_LENGTH_MAX;
    config = my_param_get_ble_log_config();

    /* 无参数 - 查询状态 */
    if (msg->parm_count == 0)
    {
        msg->resp_length = snprintf(msg->resp_msg, remaining, "BTLOG:%s",
                                    config->global_en ? "ON" : "OFF");
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
    }

    /* 有参数 - 设置状态 */
    if (msg->parm_count == 1)
    {
        if (my_strcasecmp(msg->parm[1], "ON") == 0)
        {
            config->global_en = 1;
            if (my_param_set_ble_log_config(config) == 0)
            {
                msg->resp_length = snprintf(msg->resp_msg, remaining, "Set OK!");
                LOG_INF("BTLOG enabled");
            }
            else
            {
                msg->resp_length = snprintf(msg->resp_msg, remaining, "Set Fail! Unauthorized");
            }
            return BLE_DATA_TYPE_PACKET_MULTIPLE;
        }
        else if (my_strcasecmp(msg->parm[1], "OFF") == 0)
        {
            config->global_en = 0;
            if (my_param_set_ble_log_config(config) == 0)
            {
                msg->resp_length = snprintf(msg->resp_msg, remaining, "Set OK!");
                LOG_INF("BTLOG disabled");
            }
            else
            {
                msg->resp_length = snprintf(msg->resp_msg, remaining, "Set Fail! Unauthorized");
            }
            return BLE_DATA_TYPE_PACKET_MULTIPLE;
        }
        else
        {
            LOG_INF("BTLOG invalid param: %s", msg->parm[1]);
            goto param_invalid;
        }
    }

    /* 参数数量错误 */
    LOG_INF("BTLOG param count error: %d", msg->parm_count);

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set Fail! InvalidParam");
    return BLE_DATA_TYPE_PACKET_MULTIPLE;
}

/********************************************************************
**函数名称:  version_cmd_handler
**入口参数:  msg   ---   AT 命令消息结构体
**出口参数:  msg   ---   填充响应消息
**函数功能:  处理VERSION指令：查询版本号
**指令格式:  VERSION#
**返回值说明: [VERSION] [版本号]
**返 回 值:  BLE数据类型
*********************************************************************/
static int version_cmd_handler(at_cmd_t* msg)
{
    uint16_t remaining;  // 响应消息缓冲区的剩余空间
    int ret;             // snprintf 函数的返回值

    remaining = RESP_STRING_LENGTH_MAX;  // 计算响应消息缓冲区的大小

    /* 检查参数数量：应为0 */
    if (msg->parm_count == 0)  // 检查命令是否有参数
    {
        LOG_INF("%s=>%s", __func__, msg->parm[0]);  // 输出函数名和命令名

        /* 生成响应消息，格式：[VERSION]%s*/
        ret = snprintf(msg->resp_msg, remaining, "[Cell VERSION]%s;\n[BT VERSION]%s", g_lte4GVersion, SOFTWARE_VERSION);  // 生成包含版本号的响应消息

        if (ret > 0 && ret < remaining)  // 检查响应消息是否生成成功
        {
            msg->resp_length = ret;  // 设置响应消息的长度
            LOG_INF("VERSION: %s", msg->resp_msg);  // 输出版本号信息
        }
        else  // 响应消息生成失败
        {
            // 生成失败响应消息
            msg->resp_length = snprintf(msg->resp_msg, remaining, "Set Fail! InvalidParam");
        }
    }
    else  // 参数数量错误
    {
        // 输出参数数量错误信息
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        // 生成失败响应消息
        msg->resp_length = snprintf(msg->resp_msg, remaining, "Set Fail! InvalidParam");
    }
    return BLE_DATA_TYPE_PACKET_MULTIPLE;  // 返回 BLE 数据类型
}

/********************************************************************
**函数名称:  modeset_cmd_handler
**入口参数:  msg   ---   AT 命令消息结构体
**出口参数:  msg   ---   填充响应消息
**函数功能:  处理MODESET指令：设置设备工作模式
**指令格式:  MODESET,[Work Mode],[参数...]#
**参数说明:  模式0: MODESET,0,[Reporting INT],[Distance INT]#
**           模式1: MODESET,1,[Reporting Interval],[Start Time],[GNSS SW]#
**           模式2: MODESET,2,[Sub Mode],[Static INT],[MOVING INT]# 或 MODESET,2,[Sub Mode]#
**           模式3: MODESET,3#
**返 回 值:  BLE数据类型
*********************************************************************/
static int modeset_cmd_handler(at_cmd_t* msg)
{
    uint8_t no_count = 0;
    uint16_t remaining;
    device_work_mode_config_t param_work_mode_config;
    int gnss_sw;
    int sub_mode_val;
    int static_int_val;
    int moving_int_val;

    remaining = RESP_STRING_LENGTH_MAX;  // 计算响应消息缓冲区的大小

    if(msg->parm_count == 0)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        goto param_invalid;
    }

    no_count = string_check_is_number(0, msg->parm[1]);
    if (no_count == 0 || no_count > 9)
    {
        LOG_INF("%s=>invalid Work Mode param: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }
    // 解析当前模式参数（parm[1]）
    param_work_mode_config.current_mode = atoi(msg->parm[1]);
    if (param_work_mode_config.current_mode >= MY_MODE_MAX)
    {
        LOG_INF("%s=>invalid mode: %d", __func__, param_work_mode_config.current_mode);
        goto param_invalid;
    }

    // 只有模式参数的情况（仅切换模式，不改参数）
    if (msg->parm_count == 1)
    {

        if (get_last_work_mode() != param_work_mode_config.current_mode)
        {
            gConfigParam.device_workmode_config.flag = FLAG_VALID;
            gConfigParam.device_workmode_config.workmode_config.current_mode = param_work_mode_config.current_mode;
            /* 保存配置 */
            my_user_data_write(ZMS_ID_WORK_MODE_CONFIG, &gConfigParam.device_workmode_config, sizeof(work_mode_config_t));
        }

        // 切换到指定工作模式
        switch_work_mode(param_work_mode_config.current_mode);

        /* 生成成功响应 */
        msg->resp_length = snprintf(msg->resp_msg, remaining, "Set OK!");
        LOG_INF("MODESET: current_mode:%d", param_work_mode_config.current_mode);

        return BLE_DATA_TYPE_PACKET_MULTIPLE;
    }

    // 连续追踪模式处理: MODESET,0,[Reporting INT],[Distance INT]#
    if (param_work_mode_config.current_mode == MY_MODE_CONTINUOUS)
    {
        /* 检查参数数量 */
        if (msg->parm_count != 3)
        {
            LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
            goto param_invalid;
        }

        no_count = string_check_is_number(0, msg->parm[2]);
        if (no_count == 0 || no_count > 9)
        {
            LOG_INF("%s=>invalid Reporting Interval Sec param: %s", __func__, msg->parm[2]);
            goto param_invalid;
        }
        no_count = string_check_is_number(0, msg->parm[3]);
        if (no_count == 0 || no_count > 9)
        {
            LOG_INF("%s=>invalid Reporting Interval Dis param: %s", __func__, msg->parm[3]);
            goto param_invalid;
        }
        // 解析连续追踪模式参数
        param_work_mode_config.continuous_tracking.reporting_interval_sec = atoi(msg->parm[2]);
        param_work_mode_config.continuous_tracking.reporting_interval_dis = atoi(msg->parm[3]);

        // 检查参数是否有效
        if (param_work_mode_config.continuous_tracking.reporting_interval_sec < 5 || param_work_mode_config.continuous_tracking.reporting_interval_sec > 86400)
        {
            LOG_INF("%s=>Reporting INT %u out of range (5~86400)", __func__, param_work_mode_config.continuous_tracking.reporting_interval_sec);
            goto param_invalid;
        }
        if (param_work_mode_config.continuous_tracking.reporting_interval_dis != 0 &&
            (param_work_mode_config.continuous_tracking.reporting_interval_dis < 5 ||
            param_work_mode_config.continuous_tracking.reporting_interval_dis > 1000))
        {
            LOG_INF("%s=>Distance INT %u out of range (0/5~1000)", __func__, param_work_mode_config.continuous_tracking.reporting_interval_dis);
            goto param_invalid;
        }

        // 设置连续追踪模式参数
        gConfigParam.device_workmode_config.workmode_config.continuous_tracking.reporting_interval_sec = param_work_mode_config.continuous_tracking.reporting_interval_sec;
        gConfigParam.device_workmode_config.workmode_config.continuous_tracking.reporting_interval_dis = param_work_mode_config.continuous_tracking.reporting_interval_dis;
        gConfigParam.device_workmode_config.flag = FLAG_VALID;
        /* 保存配置 */
        my_user_data_write(ZMS_ID_WORK_MODE_CONFIG, &gConfigParam.device_workmode_config, sizeof(work_mode_config_t));

        if (gConfigParam.device_workmode_config.workmode_config.current_mode == MY_MODE_CONTINUOUS)
        {
            send_work_mode_command(param_work_mode_config.current_mode);
        }

        LOG_INF("%s,%s,%s,%s#", msg->parm[0], msg->parm[1], msg->parm[2], msg->parm[3]);
    }
    // 长续航模式处理: MODESET,1,[Reporting Interval],[Start Time],[GNSS SW]#
    else if (param_work_mode_config.current_mode == MY_MODE_LONG_LIFE)
    {
        /* 检查参数数量 */
        if (msg->parm_count != 4)
        {
            LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
            goto param_invalid;
        }

        no_count = string_check_is_number(0, msg->parm[2]);
        if (no_count == 0 || no_count > 9)
        {
            LOG_INF("%s=>invalid Reporting Interval Min param: %s", __func__, msg->parm[2]);
            goto param_invalid;
        }

        // 解析GNSS SW参数（ON/OFF字符串）
        if (my_strcasecmp(msg->parm[4], "ON") == 0)
        {
            gnss_sw = 1;
        }
        else if (my_strcasecmp(msg->parm[4], "OFF") == 0)
        {
            gnss_sw = 0;
        }
        else
        {
            LOG_INF("%s=>invalid GNSS SW param: %s (expect ON/OFF)", __func__, msg->parm[4]);
            goto param_invalid;
        }

        // 解析长续航模式参数
        param_work_mode_config.long_battery.reporting_interval_min = atoi(msg->parm[2]);
        // 设置长续航模式参数
        if (set_long_battery_params(&gConfigParam.device_workmode_config.workmode_config,
            param_work_mode_config.long_battery.reporting_interval_min, msg->parm[3], gnss_sw) < 0)
        {
            LOG_INF("%s=>set_long_battery_params failed", __func__);
            goto param_invalid;
        }

        gConfigParam.device_workmode_config.flag = FLAG_VALID;
        /* 保存配置 */
        my_user_data_write(ZMS_ID_WORK_MODE_CONFIG, &gConfigParam.device_workmode_config, sizeof(work_mode_config_t));

        if (gConfigParam.device_workmode_config.workmode_config.current_mode == MY_MODE_LONG_LIFE)
        {
            send_work_mode_command(param_work_mode_config.current_mode);
            // 重新开启LTE定时器
            my_send_msg(MOD_MAIN, MOD_MAIN, MY_MSG_RESET_LTE_TIMER);
        }

        LOG_INF("%s,%s,%s,%s,%s#", msg->parm[0], msg->parm[1], msg->parm[2], msg->parm[3], msg->parm[4]);
    }
    // 智能模式处理: MODESET,2,[Sub Mode],[Static INT],[MOVING INT]# 或 MODESET,2,[Sub Mode]#
    else if (param_work_mode_config.current_mode == MY_MODE_SMART)
    {
        /* 检查参数数量：2=仅切换子模式，4=完整设置子模式+间隔 */
        if (msg->parm_count != 2 && msg->parm_count != 4)
        {
            LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
            goto param_invalid;
        }

        no_count = string_check_is_number(0, msg->parm[2]);
        if (no_count == 0 || no_count > 9)
        {
            LOG_INF("%s=>invalid Sub Mode param: %s", __func__, msg->parm[2]);
            goto param_invalid;
        }

        sub_mode_val = atoi(msg->parm[2]);

        if (msg->parm_count == 4)
        {
            /* 完整模式：校验并设置 Sub Mode + Static INT + Moving INT */
            no_count = string_check_is_number(0, msg->parm[3]);
            if (no_count == 0 || no_count > 9)
            {
                LOG_INF("%s=>invalid Static INT param: %s", __func__, msg->parm[3]);
                goto param_invalid;
            }
            no_count = string_check_is_number(0, msg->parm[4]);
            if (no_count == 0 || no_count > 9)
            {
                LOG_INF("%s=>invalid Moving INT param: %s", __func__, msg->parm[4]);
                goto param_invalid;
            }

            static_int_val = atoi(msg->parm[3]);
            moving_int_val = atoi(msg->parm[4]);

            // 设置智能模式参数（内部完成参数校验）
            if (set_intelligent_params(&gConfigParam.device_workmode_config.workmode_config,
                sub_mode_val, static_int_val, moving_int_val) < 0)
            {
                LOG_INF("%s=>set_intelligent_params failed", __func__);
                goto param_invalid;
            }
        }
        else
        {
            /* 仅切换子模式：校验 sub_mode 范围，保留该子模式已有间隔 */
            if (sub_mode_val > 5)
            {
                LOG_INF("%s=>invalid Sub Mode %d (0~5)", __func__, sub_mode_val);
                goto param_invalid;
            }
            gConfigParam.device_workmode_config.workmode_config.intelligent.sub_mode = sub_mode_val;
        }

        gConfigParam.device_workmode_config.flag = FLAG_VALID;
        /* 保存配置 */
        my_user_data_write(ZMS_ID_WORK_MODE_CONFIG, &gConfigParam.device_workmode_config, sizeof(work_mode_config_t));

        if (gConfigParam.device_workmode_config.workmode_config.current_mode == MY_MODE_SMART)
        {
            send_work_mode_command(param_work_mode_config.current_mode);
            // 参数变更后，基于当前运动状态重新应用LTE唤醒策略
            smart_mode_apply_lte_policy();
        }

        if (msg->parm_count == 4)
        {
            LOG_INF("%s,%s,%s,%s,%s#", msg->parm[0], msg->parm[1], msg->parm[2], msg->parm[3], msg->parm[4]);
        }
        else
        {
            LOG_INF("%s,%s,%s#", msg->parm[0], msg->parm[1], msg->parm[2]);
        }
    }
    else
    {
        LOG_INF("%s=>invalid current_mode %d", __func__, param_work_mode_config.current_mode);
        goto param_invalid;
    }


    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set OK!");
    LOG_INF("MODESET: current_mode:%d", param_work_mode_config.current_mode);

    return BLE_DATA_TYPE_PACKET_MULTIPLE;

param_invalid:
    // 生成失败响应
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set Fail! InvalidParam");
    return BLE_DATA_TYPE_PACKET_MULTIPLE;
}

/********************************************************************
**函数名称:  modeget_cmd_handler
**入口参数:  msg   ---   AT 命令消息结构体
**出口参数:  msg   ---   填充响应消息
**函数功能:  处理MODEGET指令：查询设备当前工作模式参数
**指令格式:  MODEGET#
**返 回 值:  BLE数据类型
*********************************************************************/
static int modeget_cmd_handler(at_cmd_t* msg)
{
    uint16_t remaining;  // 响应消息缓冲区的剩余空间
    int ret = -1;        // snprintf 函数的返回值

    remaining = RESP_STRING_LENGTH_MAX;  // 计算响应消息缓冲区的大小

    /* 检查参数数量：应为0 */
    if (msg->parm_count == 0)  // 检查命令是否有参数
    {
        LOG_INF("%s=>%s", __func__, msg->parm[0]);  // 输出函数名和命令名

        switch (gConfigParam.device_workmode_config.workmode_config.current_mode)
        {
            case MY_MODE_CONTINUOUS:
                ret = snprintf(msg->resp_msg, remaining, "MODE:%d,%d,%d",
                    gConfigParam.device_workmode_config.workmode_config.current_mode,
                    gConfigParam.device_workmode_config.workmode_config.continuous_tracking.reporting_interval_sec,
                    gConfigParam.device_workmode_config.workmode_config.continuous_tracking.reporting_interval_dis);
                break;

            case MY_MODE_LONG_LIFE:
                ret = snprintf(msg->resp_msg, remaining, "MODE:%d,%d,%s,%s",
                    gConfigParam.device_workmode_config.workmode_config.current_mode,
                    gConfigParam.device_workmode_config.workmode_config.long_battery.reporting_interval_min,
                    gConfigParam.device_workmode_config.workmode_config.long_battery.start_time,
                    gConfigParam.device_workmode_config.workmode_config.long_battery.gnss_sw ? "ON" : "OFF");
                break;

            case MY_MODE_SMART:
                ret = snprintf(msg->resp_msg, remaining, "MODE:%d,%d,%d,%d",
                    gConfigParam.device_workmode_config.workmode_config.current_mode,
                    gConfigParam.device_workmode_config.workmode_config.intelligent.sub_mode,
                    gConfigParam.device_workmode_config.workmode_config.intelligent.static_interval[gConfigParam.device_workmode_config.workmode_config.intelligent.sub_mode],
                    gConfigParam.device_workmode_config.workmode_config.intelligent.moving_interval[gConfigParam.device_workmode_config.workmode_config.intelligent.sub_mode]);
                break;

            case MY_MODE_ALWAYS_ONLINE:
                ret = snprintf(msg->resp_msg, remaining, "MODE:%d",
                    gConfigParam.device_workmode_config.workmode_config.current_mode);
                break;

            default:
                break;
        }

        if (ret > 0 && ret < remaining)  // 检查响应消息是否生成成功
        {
            msg->resp_length = ret;  // 设置响应消息的长度
            LOG_INF("MODEGET: %s", msg->resp_msg);  // 输出状态信息
        }
        else  // 响应消息生成失败
        {
            // 生成失败响应消息
            msg->resp_length = snprintf(msg->resp_msg, remaining, "Set Fail! InvalidParam");
        }
    }
    else  // 参数数量错误
    {
        // 输出参数数量错误信息
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        // 生成失败响应消息
        msg->resp_length = snprintf(msg->resp_msg, remaining, "Set Fail! InvalidParam");
    }
    return BLE_DATA_TYPE_PACKET_MULTIPLE;  // 返回 BLE 数据类型
}

/********************************************************************
**函数名称:  modeparam_cmd_handler
**入口参数:  msg   ---   AT 命令消息结构体
**出口参数:  msg   ---   填充响应消息
**函数功能:  处理MODEPARAM指令：查询设备所有工作模式参数
**指令格式:  MODEPARAM#
**返 回 值:  BLE数据类型
*********************************************************************/
static int modeparam_cmd_handler(at_cmd_t* msg)
{
    uint16_t remaining;  // 响应消息缓冲区的剩余空间
    int ret = -1;        // snprintf 函数的返回值

    remaining = RESP_STRING_LENGTH_MAX;  // 计算响应消息缓冲区的大小

    /* 检查参数数量：应为0 */
    if (msg->parm_count == 0)  // 检查命令是否有参数
    {
        LOG_INF("%s=>%s", __func__, msg->parm[0]);  // 输出函数名和命令名

        ret = snprintf(msg->resp_msg, remaining,
                    "MODE:%d,%d,%d\nMODE:%d,%d,%s,%s\nMODE:%d,%d,%d,%d\nMODE:%d",
                    MY_MODE_CONTINUOUS,
                    gConfigParam.device_workmode_config.workmode_config.continuous_tracking.reporting_interval_sec,
                    gConfigParam.device_workmode_config.workmode_config.continuous_tracking.reporting_interval_dis,
                    MY_MODE_LONG_LIFE,
                    gConfigParam.device_workmode_config.workmode_config.long_battery.reporting_interval_min,
                    gConfigParam.device_workmode_config.workmode_config.long_battery.start_time,
                    gConfigParam.device_workmode_config.workmode_config.long_battery.gnss_sw ? "ON" : "OFF",
                    MY_MODE_SMART,
                    gConfigParam.device_workmode_config.workmode_config.intelligent.sub_mode,
                    gConfigParam.device_workmode_config.workmode_config.intelligent.static_interval[gConfigParam.device_workmode_config.workmode_config.intelligent.sub_mode],
                    gConfigParam.device_workmode_config.workmode_config.intelligent.moving_interval[gConfigParam.device_workmode_config.workmode_config.intelligent.sub_mode],
                    MY_MODE_ALWAYS_ONLINE);

        if (ret > 0 && ret < remaining)  // 检查响应消息是否生成成功
        {
            msg->resp_length = ret;  // 设置响应消息的长度
            LOG_INF("MODEPARAM: %s", msg->resp_msg);  // 输出状态信息
        }
        else  // 响应消息生成失败
        {
            // 生成失败响应消息
            msg->resp_length = snprintf(msg->resp_msg, remaining, "Set Fail! InvalidParam");
        }
    }
    else  // 参数数量错误
    {
        // 输出参数数量错误信息
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        // 生成失败响应消息
        msg->resp_length = snprintf(msg->resp_msg, remaining, "Set Fail! InvalidParam");
    }
    return BLE_DATA_TYPE_PACKET_MULTIPLE;  // 返回 BLE 数据类型
}

/********************************************************************
**函数名称:  bt_parmac_cmd_handler
**入口参数:  msg      ---        AT指令结构体指针
**出口参数:  无
**函数功能:  处理透传MAC地址管理指令
**           支持：ADD/DEL/CHECK
**指令格式:  BT_PARMAC,ADD,[MAC1]...[MAC6]#
**           BT_PARMAC,DEL,[MAC]#
**           BT_PARMAC,DEL,ALL#
**           BT_PARMAC,CHECK#
**参数说明:  [MAC1]...[MAC6] - 待添加的MAC地址，支持同时添加1~6个
**           [MAC]           - 待删除的指定MAC地址
**           ALL             - 删除全部已配置MAC地址
**返 回 值:  BLE_DATA_TYPE_AT_CMD
*********************************************************************/
static int bt_parmac_cmd_handler(at_cmd_t* msg)
{
    uint16_t remaining;
    uint8_t hex_buf[6];
    uint8_t reversed_buf[6];
    char mac_str[13];
    bt_addr_le_t temp_addr;
    int i, add_count;

    remaining = RESP_STRING_LENGTH_MAX;

    if (msg->parm_count < 1)
    {
        goto param_invalid;
    }

    // BT_PARMAC,ADD,[MAC1],[MAC2]...[MAC6]#
    if (my_strcasecmp(msg->parm[1], "ADD") == 0)
    {
        add_count = msg->parm_count - 1;  // 减去"ADD"自身
        if (add_count < 1 || add_count > 6)
        {
            goto param_invalid;
        }

        // 检查总容量是否足够
        if (gConfigParam.bparmac_config.bt_parmac_mac_count + add_count > TRAN_MAC_MAX_NUM)
        {
            LOG_INF("max count exceeded, current=%d, add=%d",
                    gConfigParam.bparmac_config.bt_parmac_mac_count, add_count);
            goto param_invalid;
        }

        for (i = 0; i < add_count; i++)
        {
            // 将MAC字符串转换为HEX数组
            if (!macstr_to_hex(msg->parm[2 + i], hex_buf))
            {
                LOG_INF("invalid MAC: %s", msg->parm[2 + i]);
                goto param_invalid;
            }
            // 字节序反转（大端转小端）
            char_array_reverse(hex_buf, sizeof(hex_buf), reversed_buf, sizeof(reversed_buf));
            memset(&temp_addr, 0, sizeof(temp_addr));
            temp_addr.type = BT_ADDR_LE_PUBLIC;
            memcpy(temp_addr.a.val, reversed_buf, 6);

            if (my_tran_mac_add(&temp_addr) != 0)
            {
                LOG_WRN("duplicate mac or not enough space");
            }
        }

        // 更新配置参数
        gConfigParam.bparmac_config.flag = FLAG_VALID;
        // 保存配置参数到flash
        my_user_data_write(ZMS_ID_BT_PARMAC_CONFIG, &gConfigParam.bparmac_config, sizeof(bparmac_config_t));

        LOG_INF("ADD %d MACs, total: %d", add_count, gConfigParam.bparmac_config.bt_parmac_mac_count);
        msg->resp_length = snprintf(msg->resp_msg, remaining, "Set OK!");
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
    }
    else if (my_strcasecmp(msg->parm[1], "DEL") == 0)
    {
        if (msg->parm_count != 2)
        {
            goto param_invalid;
        }

        // BT_PARMAC,DEL,ALL#
        if (my_strcasecmp(msg->parm[2], "ALL") == 0)
        {
            my_tran_mac_del_all();
            LOG_INF("DEL ALL");
            msg->resp_length = snprintf(msg->resp_msg, remaining, "Set OK!");
            // 更新配置参数
            gConfigParam.bparmac_config.flag = FLAG_VALID;
            // 保存配置参数到flash
            my_user_data_write(ZMS_ID_BT_PARMAC_CONFIG, &gConfigParam.bparmac_config, sizeof(bparmac_config_t));
            return BLE_DATA_TYPE_PACKET_MULTIPLE;
        }
        // BT_PARMAC,DEL,[MAC]#
        else
        {
            if (!macstr_to_hex(msg->parm[2], hex_buf))
            {
                goto param_invalid;
            }
            char_array_reverse(hex_buf, 6, reversed_buf, 6);
            memset(&temp_addr, 0, sizeof(temp_addr));
            temp_addr.type = BT_ADDR_LE_PUBLIC;
            memcpy(temp_addr.a.val, reversed_buf, 6);

            if (my_tran_mac_del(&temp_addr) == 0)
            {
                LOG_INF("DEL MAC success");
                msg->resp_length = snprintf(msg->resp_msg, remaining, "Set OK!");
                return BLE_DATA_TYPE_PACKET_MULTIPLE;
            }
            else
            {
                LOG_INF("DEL MAC not found");
                goto param_invalid;
            }
            // 更新配置参数
            gConfigParam.bparmac_config.flag = FLAG_VALID;
            // 保存配置参数到flash
            my_user_data_write(ZMS_ID_BT_PARMAC_CONFIG, &gConfigParam.bparmac_config, sizeof(bparmac_config_t));
        }
    }
    else if (my_strcasecmp(msg->parm[1], "CHECK") == 0)
    {
        // BT_PARMAC,CHECK#
        if (msg->parm_count != 1)
        {
            goto param_invalid;
        }

        for (i = 0; i < gConfigParam.bparmac_config.bt_parmac_mac_count; i++)
        {
            snprintf(mac_str, sizeof(mac_str), "%02X%02X%02X%02X%02X%02X",
                     gConfigParam.bparmac_config.bt_parmac_macs[i].a.val[5],
                     gConfigParam.bparmac_config.bt_parmac_macs[i].a.val[4],
                     gConfigParam.bparmac_config.bt_parmac_macs[i].a.val[3],
                     gConfigParam.bparmac_config.bt_parmac_macs[i].a.val[2],
                     gConfigParam.bparmac_config.bt_parmac_macs[i].a.val[1],
                     gConfigParam.bparmac_config.bt_parmac_macs[i].a.val[0]);

            if (i < gConfigParam.bparmac_config.bt_parmac_mac_count - 1)
            {
                msg->resp_length += snprintf(msg->resp_msg + msg->resp_length,
                    RESP_STRING_LENGTH_MAX - msg->resp_length, "%s;", mac_str);
            }
            else
            {
                msg->resp_length += snprintf(msg->resp_msg + msg->resp_length,
                    RESP_STRING_LENGTH_MAX - msg->resp_length, "%s", mac_str);
            }
        }

        if (i == 0)
        {
            msg->resp_length = snprintf(msg->resp_msg, remaining, "Set Fail! Unauthorized.");
        }

        LOG_INF("CHECK, count=%d", gConfigParam.bparmac_config.bt_parmac_mac_count);
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
    }
    else
    {
        goto param_invalid;
    }

param_invalid:
    LOG_INF("%s=>%s, param error or set fail", __func__, msg->parm[0]);
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set Fail! InvalidParam");
    return BLE_DATA_TYPE_PACKET_MULTIPLE;
}

/********************************************************************
**函数名称:  patmtimer_cmd_handler
**入口参数:  msg      ---        AT指令结构体指针
**出口参数:  无
**函数功能:  处理气压定时上传配置指令
**返 回 值:  BLE_DATA_TYPE_PACKET_MULTIPLE
*********************************************************************/
static int patmtimer_cmd_handler(at_cmd_t* msg)
{
    uint16_t remaining;
    uint8_t no_count;
    int interval_min;
    int wakeup_sw;

    remaining = RESP_STRING_LENGTH_MAX;

    if (msg->parm_count == 0)
    {
        msg->resp_length = snprintf(msg->resp_msg, remaining, "PATMTIMER:%d,%s",
                                    gConfigParam.patm_timer_config.interval_min,
                                    gConfigParam.patm_timer_config.wakeup_cell_sw ? "ON" : "OFF");
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
    }

    if (msg->parm_count != 2)
    {
        goto param_invalid;
    }

    no_count = string_check_is_number(0, msg->parm[1]);
    if (no_count == 0 || no_count > 9)
    {
        goto param_invalid;
    }

    interval_min = (uint16_t)atoi(msg->parm[1]);
    if (!(interval_min == 0 || (interval_min >= 10 && interval_min <= 1440)))
    {
        goto param_invalid;
    }

    if (my_strcasecmp(msg->parm[2], "ON") == 0)
    {
        wakeup_sw = 1;
    }
    else if (my_strcasecmp(msg->parm[2], "OFF") == 0)
    {
        wakeup_sw = 0;
    }
    else
    {
        goto param_invalid;
    }

    gConfigParam.patm_timer_config.flag = FLAG_VALID;
    gConfigParam.patm_timer_config.interval_min = interval_min;
    gConfigParam.patm_timer_config.wakeup_cell_sw = wakeup_sw;
    my_user_data_write(ZMS_ID_PATM_TIMER_CONFIG, &gConfigParam.patm_timer_config, sizeof(patm_timer_config_t));
    // 通知CTRL线程仅重装气压上传定时器
    my_send_msg(MOD_BLE, MOD_CTRL, MY_MSG_CTRL_PATM_RELOAD);

    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set OK!");
    return BLE_DATA_TYPE_PACKET_MULTIPLE;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set Fail! InvalidParam");
    return BLE_DATA_TYPE_PACKET_MULTIPLE;
}

/********************************************************************
**函数名称:  patm_cmd_handler
**入口参数:  msg      ---        AT指令结构体指针
**出口参数:  无
**函数功能:  处理气压读取指令
**指令格式:  PATM#
**返 回 值:  BLE_DATA_TYPE_PACKET_MULTIPLE
*********************************************************************/
static int patm_cmd_handler(at_cmd_t* msg)
{
    uint16_t remaining;  // 响应消息缓冲区的剩余空间

    remaining = RESP_STRING_LENGTH_MAX;  // 计算响应消息缓冲区的大小

    /* 检查参数数量：应为0 */
    if (msg->parm_count == 0)  // 检查命令是否有参数
    {
        LOG_INF("%s=>%s", __func__, msg->parm[0]);  // 输出函数名和命令名
        // 通知CTRL线程读取气压数据
        my_send_msg(MOD_BLE, MOD_CTRL, MY_MSG_CTRL_PATM_READ);

        return 1; // 在其它线程处理后续任务
    }
    else  // 参数数量错误
    {
        // 输出参数数量错误信息
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        // 生成失败响应消息
        msg->resp_length = snprintf(msg->resp_msg, remaining, "Set Fail! InvalidParam");
    }
    return BLE_DATA_TYPE_PACKET_MULTIPLE;  // 返回 BLE 数据类型
}

/********************************************************************
**函数名称:  temptimer_cmd_handler
**入口参数:  msg      ---        AT指令结构体指针
**出口参数:  无
**函数功能:  处理温湿度定时上传配置指令
**返 回 值:  BLE_DATA_TYPE_PACKET_MULTIPLE
*********************************************************************/
static int temptimer_cmd_handler(at_cmd_t* msg)
{
    uint16_t remaining;
    uint8_t no_count;
    int interval_min;
    int wakeup_sw;

    remaining = RESP_STRING_LENGTH_MAX;

    if (msg->parm_count == 0)
    {
        msg->resp_length = snprintf(msg->resp_msg, remaining, "TEMPTIMER:%d,%s",
                                    gConfigParam.temp_timer_config.interval_min,
                                    gConfigParam.temp_timer_config.wakeup_cell_sw ? "ON" : "OFF");
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
    }

    if (msg->parm_count != 2)
    {
        goto param_invalid;
    }

    no_count = string_check_is_number(0, msg->parm[1]);
    if (no_count == 0 || no_count > 9)
    {
        goto param_invalid;
    }

    interval_min = (uint16_t)atoi(msg->parm[1]);
    if (!(interval_min == 0 || (interval_min >= 10 && interval_min <= 1440)))
    {
        goto param_invalid;
    }

    if (my_strcasecmp(msg->parm[2], "ON") == 0)
    {
        wakeup_sw = 1;
    }
    else if (my_strcasecmp(msg->parm[2], "OFF") == 0)
    {
        wakeup_sw = 0;
    }
    else
    {
        goto param_invalid;
    }

    gConfigParam.temp_timer_config.flag = FLAG_VALID;
    gConfigParam.temp_timer_config.interval_min = interval_min;
    gConfigParam.temp_timer_config.wakeup_cell_sw = wakeup_sw;
    my_user_data_write(ZMS_ID_TEMP_TIMER_CONFIG, &gConfigParam.temp_timer_config, sizeof(temp_timer_config_t));
    // 通知CTRL线程仅重装温湿度上传定时器
    my_send_msg(MOD_BLE, MOD_CTRL, MY_MSG_CTRL_TEMP_RELOAD);

    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set OK!");
    return BLE_DATA_TYPE_PACKET_MULTIPLE;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set Fail! InvalidParam");
    return BLE_DATA_TYPE_PACKET_MULTIPLE;
}

/********************************************************************
**函数名称:  temp_cmd_handler
**入口参数:  msg      ---        AT指令结构体指针
**出口参数:  无
**函数功能:  处理温湿度读取指令
**指令格式:  TEMP#
**返 回 值:  BLE_DATA_TYPE_PACKET_MULTIPLE
*********************************************************************/
static int temp_cmd_handler(at_cmd_t* msg)
{
    uint16_t remaining;  // 响应消息缓冲区的剩余空间

    remaining = RESP_STRING_LENGTH_MAX;  // 计算响应消息缓冲区的大小

    /* 检查参数数量：应为0 */
    if (msg->parm_count == 0)  // 检查命令是否有参数
    {
        LOG_INF("%s=>%s", __func__, msg->parm[0]);  // 输出函数名和命令名
        // 通知CTRL线程读取温湿度数据
        my_send_msg(MOD_BLE, MOD_CTRL, MY_MSG_CTRL_TEMP_READ);

        return 1; // 在其它线程处理后续任务
    }
    else  // 参数数量错误
    {
        // 输出参数数量错误信息
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        // 生成失败响应消息
        msg->resp_length = snprintf(msg->resp_msg, remaining, "Set Fail! InvalidParam");
    }
    return BLE_DATA_TYPE_PACKET_MULTIPLE;  // 返回 BLE 数据类型
}

/********************************************************************
**函数名称:  status_cmd_handler
**入口参数:  msg      ---        AT指令结构体指针
**出口参数:  无
**函数功能:  处理状态查询指令
**指令格式:  STATUS#
**返 回 值:  BLE_DATA_TYPE_AT_CMD
*********************************************************************/
static int status_cmd_handler(at_cmd_t* msg)
{
    uint16_t remaining;     // 响应消息缓冲区的剩余空间

    remaining = RESP_STRING_LENGTH_MAX;  // 计算响应消息缓冲区的大小

    /* 检查参数数量：应为0 */
    if (msg->parm_count == 0)  // 检查命令是否有参数
    {
        LOG_INF("%s=>%s", __func__, msg->parm[0]);  // 输出函数名和命令名

        // 通知CTRL线程读取查询status#状态
        my_send_msg(MOD_BLE, MOD_CTRL, MY_MSG_CTRL_STATUS_READ);

        return 1; // 在其它线程处理后续任务
    }
    else  // 参数数量错误
    {
        // 输出参数数量错误信息
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        // 生成失败响应消息
        msg->resp_length = snprintf(msg->resp_msg, remaining, "Set Fail! InvalidParam");
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
    }
    return BLE_DATA_TYPE_PACKET_MULTIPLE;  // 返回 BLE 数据类型
}

/********************************************************************
**函数名称:  imu_alm_cmd_handler
**入口参数:  msg      ---        AT指令结构体指针
**出口参数:  msg->resp_msg  ---  响应消息
**           msg->resp_length --- 响应长度
**函数功能:  处理IMU_ALM指令：设置IMU倾角报警功能
**指令格式:  IMU_ALM,<SW>,<REPORT>,<ROLL_THRESHOLD>,<PITCH_THRESHOLD>,<YAW_THRESHOLD>,<DURATION_TIME>,<RECOVER_TIME>#
**参数说明:  <SW> - 功能开关: ON/OFF
**           <REPORT> - 报警上报方式: 0-不上报，1-GPRS, 2-GPRS+SMS, 3-GPRS+SMS+CALL
**           <ROLL_THRESHOLD> - Roll轴倾角阈值: 5-60度, 255-不报警
**           <PITCH_THRESHOLD> - Pitch轴倾角阈值: 5-60度, 255-不报警
**           <YAW_THRESHOLD> - Yaw轴倾角阈值: 5-60度, 255-不报警
**           <DURATION_TIME> - 持续时间阈值: 1-180秒
**           <RECOVER_TIME> - 恢复时间: 1-30秒
**返 回 值:  BLE数据类型
*********************************************************************/
static int imu_alm_cmd_handler(at_cmd_t* msg)
{
    uint16_t remaining;
    uint8_t no_count = 0;
    int sw_value;
    int imu_alm_report;
    int roll_threshold;
    int pitch_threshold;
    int yaw_threshold;
    int imu_duration_time;
    int recover_time;

    remaining = RESP_STRING_LENGTH_MAX;

    //无参数即查询
    if (msg->parm_count == 0)
    {
        // 根据 imu_alm_sw 的值选择 "ON" 或 "OFF"
        const char* state_str = gConfigParam.imu_alm_config.imu_alm_sw ? "ON" : "OFF";

        msg->resp_length = snprintf(msg->resp_msg, remaining, "%s:%s,%d,%d,%d,%d,%d,%d", msg->parm[0],
                                    state_str,
                                    gConfigParam.imu_alm_config.imu_alm_report,
                                    gConfigParam.imu_alm_config.imu_roll_threshold,
                                    gConfigParam.imu_alm_config.imu_pitch_threshold,
                                    gConfigParam.imu_alm_config.imu_yaw_threshold,
                                    gConfigParam.imu_alm_config.imu_duration_time,
                                    gConfigParam.imu_alm_config.recover_time);
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
    }

    /* 检查参数数量 */
    if (msg->parm_count != 7)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        goto param_invalid;
    }

    /* 解析SW参数 */
    if (my_strcasecmp(msg->parm[1], "ON") == 0)
    {
        sw_value = 1;
    }
    else if (my_strcasecmp(msg->parm[1], "OFF") == 0)
    {
        sw_value = 0;
    }
    else
    {
        LOG_INF("%s=>invalid SW param: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }

    no_count = string_check_is_number(0, msg->parm[2]);
    if (no_count == 0 || no_count > 9)
    {
        LOG_INF("%s=>invalid REPORT param: %s", __func__, msg->parm[2]);
        goto param_invalid;
    }

    /* 解析REPORT参数 */
    imu_alm_report = atoi(msg->parm[2]);
    if (imu_alm_report < 0 || imu_alm_report > 3)
    {
        LOG_INF("%s=>invalid REPORT param: %s", __func__, msg->parm[2]);
        goto param_invalid;
    }

    no_count = string_check_is_number(0, msg->parm[3]);
    if (no_count == 0 || no_count > 9)
    {
        LOG_INF("%s=>invalid ROLL_THRESHOLD param: %s", __func__, msg->parm[3]);
        goto param_invalid;
    }

    /* 解析ROLL_THRESHOLD参数 */
    roll_threshold = atoi(msg->parm[3]);
    if ((roll_threshold < 5 || roll_threshold > 60) && roll_threshold != 255)
    {
        LOG_INF("%s=>invalid ROLL_THRESHOLD param: %s", __func__, msg->parm[3]);
        goto param_invalid;
    }

    no_count = string_check_is_number(0, msg->parm[4]);
    if (no_count == 0 || no_count > 9)
    {
        LOG_INF("%s=>invalid PITCH_THRESHOLD param: %s", __func__, msg->parm[4]);
        goto param_invalid;
    }

    /* 解析PITCH_THRESHOLD参数 */
    pitch_threshold = atoi(msg->parm[4]);
    if ((pitch_threshold < 5 || pitch_threshold > 60) && pitch_threshold != 255)
    {
        LOG_INF("%s=>invalid PITCH_THRESHOLD param: %s", __func__, msg->parm[4]);
        goto param_invalid;
    }

    no_count = string_check_is_number(0, msg->parm[5]);
    if (no_count == 0 || no_count > 9)
    {
        LOG_INF("%s=>invalid YAW_THRESHOLD param: %s", __func__, msg->parm[5]);
        goto param_invalid;
    }

    /* 解析YAW_THRESHOLD参数 */
    yaw_threshold = atoi(msg->parm[5]);
    if ((yaw_threshold < 5 || yaw_threshold > 60) && yaw_threshold != 255)
    {
        LOG_INF("%s=>invalid YAW_THRESHOLD param: %s", __func__, msg->parm[5]);
        goto param_invalid;
    }

    no_count = string_check_is_number(0, msg->parm[6]);
    if (no_count == 0 || no_count > 9)
    {
        LOG_INF("%s=>invalid DURATION_TIME param: %s", __func__, msg->parm[6]);
        goto param_invalid;
    }

    /* 解析DURATION_TIME参数 */
    imu_duration_time = atoi(msg->parm[6]);
    if (imu_duration_time < 1 || imu_duration_time > 180)
    {
        LOG_INF("%s=>invalid DURATION_TIME param: %s", __func__, msg->parm[6]);
        goto param_invalid;
    }

    no_count = string_check_is_number(0, msg->parm[7]);
    if (no_count == 0 || no_count > 9)
    {
        LOG_INF("%s=>invalid RECOVER_TIME param: %s", __func__, msg->parm[7]);
        goto param_invalid;
    }

    /* 解析RECOVER_TIME参数 */
    recover_time = atoi(msg->parm[7]);
    if (recover_time < 1 || recover_time > 30)
    {
        LOG_INF("%s=>invalid RECOVER_TIME param: %s", __func__, msg->parm[7]);
        goto param_invalid;
    }

    /* 所有参数验证通过,统一赋值 */
    gConfigParam.imu_alm_config.flag = FLAG_VALID;
    gConfigParam.imu_alm_config.imu_alm_sw = (uint8_t)sw_value;
    gConfigParam.imu_alm_config.imu_alm_report = (uint8_t)imu_alm_report;
    gConfigParam.imu_alm_config.imu_roll_threshold = (uint8_t)roll_threshold;
    gConfigParam.imu_alm_config.imu_pitch_threshold = (uint8_t)pitch_threshold;
    gConfigParam.imu_alm_config.imu_yaw_threshold = (uint8_t)yaw_threshold;
    gConfigParam.imu_alm_config.imu_duration_time = (uint8_t)imu_duration_time;
    gConfigParam.imu_alm_config.recover_time = (uint8_t)recover_time;

    /* 保存配置 */
    my_user_data_write(ZMS_ID_IMU_ALM_CONFIG, &gConfigParam.imu_alm_config, sizeof(imu_alm_config_t));

    LOG_INF("%s=>%s,%s,%s", __func__, msg->parm[0], msg->parm[1], msg->parm[2]);

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set OK");

    return BLE_DATA_TYPE_PACKET_MULTIPLE;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Set Fail! InvalidParam");
    return BLE_DATA_TYPE_PACKET_MULTIPLE;
}

/*********************************************************************
**函数名称:  factory_cmd_handler
**入口参数:  msg              ---    指向AT_cmd_t结构体的指针
**出口参数:  无
**函数功能:  处理FACTORY指令
*********************************************************************/
static int factory_cmd_handler(at_cmd_t* msg)
{
    uint16_t remaining;

    remaining = RESP_STRING_LENGTH_MAX;
    /* 检查参数数量 (应为0，指令格式为FACTORY#) */
    if (msg->parm_count == 0)
    {
        my_param_factory_reset();
        lte_send_command("FACTORY", "0");
        g_factory_mode = true;
        msg->resp_length = snprintf(msg->resp_msg, remaining, "Set OK");
    }
    else
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        msg->resp_length = snprintf(msg->resp_msg, remaining, "Set Fail! InvalidParam");
    }
    return BLE_DATA_TYPE_PACKET_MULTIPLE;
}

/*********************************************************************
**函数名称:  factoryall_cmd_handler
**入口参数:  msg              ---    指向AT_cmd_t结构体的指针
**出口参数:  无
**函数功能:  处理FACTORYALL指令
*********************************************************************/
static int factoryall_cmd_handler(at_cmd_t* msg)
{
    uint16_t remaining;

    remaining = RESP_STRING_LENGTH_MAX;
     /* 检查参数数量 (应为0，指令格式为FACTORYALL#) */
    if (msg->parm_count == 0)
    {
        my_param_factory_reset();
        lte_send_command("FACTORY", "1");
        g_factory_mode = true;
        msg->resp_length = snprintf(msg->resp_msg, remaining, "Set OK");
    }
    else
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        msg->resp_length = snprintf(msg->resp_msg, remaining, "Set Fail! InvalidParam");
    }
    return BLE_DATA_TYPE_PACKET_MULTIPLE;
}
