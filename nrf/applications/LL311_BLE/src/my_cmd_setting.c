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
#define LTE_SEND_BUF_LEN (CMD_STRING_LENGTH_MAX + 20)  /* LTE最大发送缓冲区长度 */

LOG_MODULE_REGISTER(my_cmd_setting, LOG_LEVEL_INF);

// 标记lte_cmd来的,用于区分蓝牙下发的还是lte过来的(某些指令只能网络发蓝牙不能执行)
uint8_t g_lte_cmdSource;
static int remalm_cmd_handler(at_cmd_struc* msg);
static int lockpincyt_cmd_handler(at_cmd_struc* msg);
static int lockerr_cmd_handler(at_cmd_struc* msg);
static int pinstat_cmd_handler(at_cmd_struc* msg);
static int lockstat_cmd_handler(at_cmd_struc* msg);
static int motdet_cmd_handler(at_cmd_struc* msg);
static int batlevel_cmd_handler(at_cmd_struc* msg);
static int chargesta_cmd_handler(at_cmd_struc* msg);
static int shockalarm_cmd_handler(at_cmd_struc* msg);
static int pwsave_cmd_handler(at_cmd_struc* msg);
static int startr_cmd_handler(at_cmd_struc* msg);
static int cbmt_cmd_handler(at_cmd_struc* msg);
static int bt_crfpwr_cmd_handler(at_cmd_struc* msg);
static int bt_updata_cmd_handler(at_cmd_struc* msg);
static int tag_cmd_handler(at_cmd_struc* msg);
static int lockcd_cmd_handler(at_cmd_struc* msg);
static int led_cmd_handler(at_cmd_struc* msg);
static int buzzer_cmd_handler(at_cmd_struc* msg);
static int nfctrig_cmd_handler(at_cmd_struc* msg);
static int nfcauth_cmd_handler(at_cmd_struc* msg);
static int btlog_cmd_handler(at_cmd_struc* msg);
static int bkey_cmd_handler(at_cmd_struc* msg);
static int bunlock_cmd_handler(at_cmd_struc* msg);
static int block_cmd_handler(at_cmd_struc* msg);
static int version_cmd_handler(at_cmd_struc* msg);
static int modeset_cmd_handler(at_cmd_struc* msg);
static int cunlock_cmd_handler(at_cmd_struc* msg);
static int clock_cmd_handler(at_cmd_struc* msg);
static int jatag_cmd_handler(at_cmd_struc* msg);
static int jgtag_cmd_handler(at_cmd_struc* msg);

static const at_cmd_attr_t at_cmd_attr_table[] =
{
    {"REMALM",         remalm_cmd_handler},
    {"LOCKPINCYT",     lockpincyt_cmd_handler},
    {"LOCKERR",        lockerr_cmd_handler},
    {"PINSTAT",        pinstat_cmd_handler},
    {"LOCKSTAT",       lockstat_cmd_handler},
    {"MOTDET",         motdet_cmd_handler},
    {"BATLEVEL",       batlevel_cmd_handler},
    {"CHARGESTA",      chargesta_cmd_handler},
    {"SHOCKALARM",     shockalarm_cmd_handler},
    {"PWRSAVE",        pwsave_cmd_handler},
    {"STARTR",         startr_cmd_handler},
    {"CBMT",           cbmt_cmd_handler},
    {"BT_CRFPWR",      bt_crfpwr_cmd_handler},
    {"BT_UPDATA",      bt_updata_cmd_handler},
    {"TAG",            tag_cmd_handler},
    {"JATAG",          jatag_cmd_handler},
    {"JGTAG",          jgtag_cmd_handler},
    {"LOCKCD",         lockcd_cmd_handler},
    {"LED",            led_cmd_handler},
    {"BUZZER",         buzzer_cmd_handler},
    {"NFCTRIG",        nfctrig_cmd_handler},
    {"NFCAUTH",        nfcauth_cmd_handler},
    {"BTLOG",          btlog_cmd_handler},
    {"BKEY_SET",       bkey_cmd_handler},
    {"BKEY_RESET",     bkey_cmd_handler},
    {"BUNLOCK",        bunlock_cmd_handler},
    {"BLOCK",          block_cmd_handler},
    {"VERSION",        version_cmd_handler},
    {"MODESET",        modeset_cmd_handler},
    {"CUNLOCK",        cunlock_cmd_handler},
    {"CLOCK",          clock_cmd_handler},
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
    MSG_S msg;  // 消息结构体

    // 检查LTE模块电源状态,如果关闭则先开启
    if (!get_lte_power_state())
    {
        my_send_msg(MOD_CTRL, MOD_LTE, MY_MSG_LTE_PWRON);  // 发送开启 LTE 电源的消息
    }

    // 动态分配内存存储告警消息
    MY_MALLOC_BUFFER(p_msg, LTE_SEND_BUF_LEN);  // 分配内存
    if(p_msg == NULL)  // 内存分配失败
    {
        MY_LOG_ERR("Failed to allocate memory for LTE command message");  // 输出错误信息
        return -1;  // 退出函数
    }

    if (param && strlen(param) > 0)  // 有参数的情况
    {
        snprintf(p_msg, LTE_SEND_BUF_LEN, "BLE+%s=%s\r\n", cmd_name, param);  // 构建带参数的命令
    }
    else  // 无参数的情况
    {
        snprintf(p_msg, LTE_SEND_BUF_LEN, "BLE+%s\r\n", cmd_name);  // 构建不带参数的命令
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

static bool validate_lte_cmd_params(at_cmd_struc* msg)
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
**返 回 值:  BLE_DATA_TYPE_AT_CMD 表示返回AT命令类型的数据
*********************************************************************/
static int lte_cmd_handler(at_cmd_struc* msg)
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

    remaining = sizeof(msg->resp_msg);  // 计算响应消息缓冲区的大小

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
        return BLE_DATA_TYPE_AT_CMD;
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
        return BLE_DATA_TYPE_AT_CMD;
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
    return BLE_DATA_TYPE_AT_CMD;
}

/*********************************************************************
**函数名称:  set_work_mode
**入口参数:  p_workmode  --  设备工作模式配置结构体指针
**           mode        --  目标工作模式
**出口参数:  无
**函数功能:  设置工作模式
**返 回 值:  0 表示成功，-1 表示失败（模式非法）
*********************************************************************/
int set_work_mode(DeviceWorkModeConfig *p_workmode, MY_WORK_MODE mode)
{
    if (p_workmode == NULL) return -1;

    if (mode < MY_MODE_CONTINUOUS || mode > MY_MODE_SMART)
    {
        LOG_INF("Error: Invalid work mode %d", mode);
        return -1;
    }

    p_workmode->current_mode = mode;
    LOG_INF("Set work mode to %d success", mode);
    return 0;
}

/*********************************************************************
**函数名称:  set_long_battery_params
**入口参数:  p_workmode        --  设备工作模式配置结构体指针
**           reporting_interval --  静止上报间隔（分钟）
**           start_time_str     --  首次唤醒基准时间（字符指针，HHMM格式，如"0800"、"2400"）
**出口参数:  无
**函数功能:  设置长续航模式参数
**返 回 值:  0 表示成功，-1 表示失败（参数非法）
*********************************************************************/
int set_long_battery_params(DeviceWorkModeConfig *p_workmode,
                     uint16_t reporting_interval, const char *start_time_str)
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

    // ========== 5. 赋值到工作模式配置结构体 ==========
    p_workmode->long_battery.reporting_interval_min = reporting_interval;
    strcpy(p_workmode->long_battery.start_time, start_time_str);

    LOG_INF("Set long_battery: reporting_interval=%u, start_time=%s", reporting_interval, start_time_str);
    return 0;
}

/*********************************************************************
**函数名称:  set_intelligent_params
**入口参数:  p_workmode  --  设备工作模式配置结构体指针
**           static_int  --  停止状态上报间隔（秒）
**           land_int    --  陆运状态上报间隔（秒）
**           land_distance --  陆运距离（米）
**           sea_int     --  海运状态上报间隔（秒）
**           sleep_sw    --  休眠开关（0/1/2）
**出口参数:  无
**函数功能:  设置智能模式参数
**返 回 值:  0 表示成功，-1 表示失败（参数非法）
*********************************************************************/
int set_intelligent_params(DeviceWorkModeConfig *p_workmode, uint32_t static_int,
                     uint32_t land_int, uint32_t land_distance, uint32_t sea_int, uint8_t sleep_sw)
{
    if (p_workmode == NULL) return -1;

    // 校验停止状态上报间隔
    if (static_int < 10 || static_int > 86400)
    {
        LOG_INF("Error: intelligent static_int %u out of range (10~86400)", static_int);
        return -1;
    }

    // 校验陆运状态上报间隔
    if (land_int < 10 || land_int > 86400)
    {
        LOG_INF("Error: intelligent land_int %u out of range (10~86400)", land_int);
        return -1;
    }

    // 校验陆运距离
    if (land_distance != 0 && (land_distance < 5 || land_distance > 1000))
    {
        LOG_INF("Error: intelligent land_distance %u out of range 0 or (5~1000)", land_distance);
        return -1;
    }

    // 校验海运状态上报间隔
    if (sea_int < 10 || sea_int > 86400)
    {
        LOG_INF("Error: intelligent sea_int %u out of range (10~86400)", sea_int);
        return -1;
    }

    // 校验休眠开关
    if (sleep_sw > 2)
    {
        LOG_INF("Error: intelligent sleep_sw %u out of range (0/1/2)", sleep_sw);
        return -1;
    }

    p_workmode->intelligent.stop_status_interval_sec = static_int;
    p_workmode->intelligent.land_status_interval_sec = land_int;
    p_workmode->intelligent.land_status_interval_dis = land_distance;
    p_workmode->intelligent.sea_status_interval_sec = sea_int;
    p_workmode->intelligent.sleep_switch = sleep_sw;
    LOG_INF("Set intelligent: static_int=%u, land_int=%u, sea_int=%u, sleep_sw=%u",
           static_int, land_int, sea_int, sleep_sw);
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
**返回值:    成功返回实际拆分的参数数量，失败返回负值错误码(-1入参异常/-2超上限/-3分隔符后超上限)
*********************************************************************/
int at_cmd_str_analyse(char *str_data, char **tar_data, int limit, char startChar, char *endChars, char splitChar)
{
    static char *blank = "";
    int len, i = 0, j = 0, status = 0;
    char *p;
    uint8_t in_quote = 0;   //是否在引号内

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
uint16_t at_recv_cmd_handler(at_cmd_struc *at_cmd_msg)
{
    char *data_ptr, split_ch = ',';
    uint8_t par_len;
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
        if (strcmp(at_cmd_attr_table[index].cmd_str, at_cmd_msg->parm[PARM_1]) == 0)
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
        if (strcmp(lte_cmd_attr_table[index], at_cmd_msg->parm[PARM_1]) == 0)
        {
            cmd_type = lte_cmd_handler(at_cmd_msg);
            return cmd_type;
        }
    }
    return cmd_type;
}

/********************************************************************
**函数名称:  run_nfc_cmd
**入口参数:  card_id      ---      输入，NFC卡号指针
**           index      ---        匹配到卡号的索引
**出口参数:  at_cmd_msg中更新响应消息内容和响应长度
**函数功能:  执行NFC联动指令
**返回值:    成功返回处理函数返回的BLE数据类型，未匹配指令或命令解析失败返回0
**          返回非0不代表命令执行成功，具体看对应的执行函数resp_msg回复
*********************************************************************/
uint16_t run_nfc_cmd(char *card_id, uint8_t *index)
{
    at_cmd_struc at_cmd_msg;
    uint16_t cmd_type = 0;
    int i = 0;
    uint8_t found = 0;

    //遍历匹配卡号
    for (i = 0; i < gConfigParam.nfctrig_config.nfctrig_table.count; i++)
    {
        /* 卡号匹配 */
        if (strcmp(gConfigParam.nfctrig_config.nfctrig_table.nfctrig_rule[i].nfctrig_nfc_no, card_id) == 0)
        {
            found = 1;
            break;
        }
    }

    if (found)
    {
        *index = i;
        //复制
        strcpy(at_cmd_msg.rcv_msg, gConfigParam.nfctrig_config.nfctrig_table.nfctrig_rule[i].nfctrig_command);

        MY_LOG_INF("at_cmd_msg.rcv_msg:%s", at_cmd_msg.rcv_msg);
        MY_LOG_INF("at_cmd_msg.rcv_msglen:%d", strlen(at_cmd_msg.rcv_msg));
        cmd_type = at_recv_cmd_handler(&at_cmd_msg);
    }

    return cmd_type;
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
uint16_t run_lte_cmd(at_cmd_struc *at_cmd_msg)
{

    uint16_t cmd_type = 0;

    MY_LOG_INF("at_cmd_msg->rcv_msg:%s", at_cmd_msg->rcv_msg);
    MY_LOG_INF("at_cmd_msg->rcv_msglen:%d", strlen(at_cmd_msg->rcv_msg));

    //标记网络指令
    g_lte_cmdSource = 1;

    //执行命令
    cmd_type = at_recv_cmd_handler(at_cmd_msg);

    //执行完清除
    g_lte_cmdSource = 0;

    if (!cmd_type)
    {
        //构造回复(命令无效)
        sprintf(at_cmd_msg->resp_msg, "Invalid command parameter");
    }

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
**           <M> - 报警上报方式: 0-GPRS, 1-GPRS+SMS, 2-GPRS+SMS+CALL
**返 回 值:  BLE数据类型
*********************************************************************/
static int remalm_cmd_handler(at_cmd_struc* msg)
{
    uint16_t remaining;
    int m_value;
    int sw_value;

    remaining = sizeof(msg->resp_msg);

    /* 检查参数数量 */
    if (msg->parm_count != 2)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
        return BLE_DATA_TYPE_AT_CMD;
    }

    /* 解析SW参数 */
    if (strcmp(msg->parm[1], "ON") == 0)
    {
        sw_value = 1;
    }
    else if (strcmp(msg->parm[1], "OFF") == 0)
    {
        sw_value = 0;
    }
    else
    {
        LOG_INF("%s=>invalid SW param: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }

    /* 解析M参数 */
    m_value = atoi(msg->parm[2]);
    if (m_value < 0 || m_value > 2)
    {
        LOG_INF("%s=>invalid M param: %s", __func__, msg->parm[2]);
        goto param_invalid;
    }

    /* 所有参数验证通过,统一赋值 */
    gConfigParam.remalm_config.remalm_sw = (uint8_t)sw_value;
    gConfigParam.remalm_config.remalm_mode = (uint8_t)m_value;

    LOG_INF("%s=>%s,%s,%s", __func__, msg->parm[0], msg->parm[1], msg->parm[2]);

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
    LOG_INF("REMALM: SW=%d, M=%d", gConfigParam.remalm_config.remalm_sw, gConfigParam.remalm_config.remalm_mode);

    //TODO 具体逻辑处理

    return BLE_DATA_TYPE_AT_CMD;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
    return BLE_DATA_TYPE_AT_CMD;
}

/********************************************************************
**函数名称:  lockpincyt_cmd_handler
**入口参数:  msg      ---        AT指令结构体指针
**出口参数:  msg->resp_msg  ---  响应消息
**           msg->resp_length --- 响应长度
**函数功能:  处理LOCKPINCYT指令：设置锁销非法拔除事件上报与响应
**指令格式:  LOCKPINCYT,[Report],[Buzzer]#
**参数说明:  [Report] - 上报方式: 0-不上报, 1-GPRS, 2-GPRS+SMS, 3-GPRS+SMS+CALL
**           [Buzzer] - 蜂鸣器报警方式: 0-不报警, 1-报警30s, 2-持续报警
**返 回 值:  BLE数据类型
*********************************************************************/
static int lockpincyt_cmd_handler(at_cmd_struc* msg)
{
    uint16_t remaining;
    int report_value;
    int buzzer_value;

    remaining = sizeof(msg->resp_msg);

    /* 检查参数数量 */
    if (msg->parm_count != 2)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
        return BLE_DATA_TYPE_AT_CMD;
    }

    /* 解析Report参数 */
    report_value = atoi(msg->parm[1]);
    if (report_value < 0 || report_value > 3)
    {
        LOG_INF("%s=>invalid Report param: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }

    /* 解析Buzzer参数 */
    buzzer_value = atoi(msg->parm[2]);
    if (buzzer_value < 0 || buzzer_value > 2)
    {
        LOG_INF("%s=>invalid Buzzer param: %s", __func__, msg->parm[2]);
        goto param_invalid;
    }

    /* 所有参数验证通过,统一赋值 */
    gConfigParam.lockpincyt_config.lockpincyt_report = (uint8_t)report_value;
    gConfigParam.lockpincyt_config.lockpincyt_buzzer = (uint8_t)buzzer_value;

    LOG_INF("%s=>%s,%s,%s", __func__, msg->parm[0], msg->parm[1], msg->parm[2]);

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
    LOG_INF("LOCKPINCYT: Report=%d, Buzzer=%d",
           gConfigParam.lockpincyt_config.lockpincyt_report,
           gConfigParam.lockpincyt_config.lockpincyt_buzzer);

    //TODO 具体逻辑处理

    return BLE_DATA_TYPE_AT_CMD;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
    return BLE_DATA_TYPE_AT_CMD;
}

/********************************************************************
**函数名称:  lockerr_cmd_handler
**入口参数:  msg      ---        AT指令结构体指针
**出口参数:  msg->resp_msg  ---  响应消息
**           msg->resp_length --- 响应长度
**函数功能:  处理LOCKERR指令：设置锁状态异常事件上报与响应
**指令格式:  LOCKERR,[参数1],[参数2]#
**参数说明:  [参数1](Report) - 上报方式: 0-不上报, 1-GPRS(默认), 2-GPRS+SMS, 3-GPRS+SMS+CALL
**           [参数2](Buzzer) - 蜂鸣器报警方式: 0-不报警(默认), 1-报警30s, 2-持续报警
**返 回 值:  BLE数据类型
*********************************************************************/
static int lockerr_cmd_handler(at_cmd_struc* msg)
{
    uint16_t remaining;
    int report_value;
    int buzzer_value;

    remaining = sizeof(msg->resp_msg);

    /* 检查参数数量 */
    if (msg->parm_count != 2)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
        return BLE_DATA_TYPE_AT_CMD;
    }

    /* 解析Report参数 */
    report_value = atoi(msg->parm[1]);
    if (report_value < 0 || report_value > 3)
    {
        LOG_INF("%s=>invalid Report param: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }

    /* 解析Buzzer参数 */
    buzzer_value = atoi(msg->parm[2]);
    if (buzzer_value < 0 || buzzer_value > 2)
    {
        LOG_INF("%s=>invalid Buzzer param: %s", __func__, msg->parm[2]);
        goto param_invalid;
    }

    /* 所有参数验证通过,统一赋值 */
    gConfigParam.lockerr_config.lockerr_report = (uint8_t)report_value;
    gConfigParam.lockerr_config.lockerr_buzzer = (uint8_t)buzzer_value;

    LOG_INF("%s=>%s,%s,%s", __func__, msg->parm[0], msg->parm[1], msg->parm[2]);

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
    LOG_INF("LOCKERR: Report=%d, Buzzer=%d",
           gConfigParam.lockerr_config.lockerr_report,
           gConfigParam.lockerr_config.lockerr_buzzer);

    //TODO 具体逻辑处理

    return BLE_DATA_TYPE_AT_CMD;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
    return BLE_DATA_TYPE_AT_CMD;
}

/********************************************************************
**函数名称:  pinstat_cmd_handler
**入口参数:  msg      ---        AT指令结构体指针
**出口参数:  msg->resp_msg  ---  响应消息
**           msg->resp_length --- 响应长度
**函数功能:  处理PINSTAT指令：设置锁销状态检测与上报
**指令格式:  PINSTAT,[Report],[Trigger]#
**参数说明:  [Report] - 上报方式: 0-不上报, 1-GPRS, 2-GPRS+SMS, 3-GPRS+SMS+CALL
**           [Trigger] - 触发上报方式: 0-都不触发, 1-插入触发, 2-拔出触发, 3-插入拔出均触发
**返 回 值:  BLE数据类型
*********************************************************************/
static int pinstat_cmd_handler(at_cmd_struc* msg)
{
    uint16_t remaining;
    int report_value;
    int trigger_value;

    remaining = sizeof(msg->resp_msg);

    /* 检查参数数量 */
    if (msg->parm_count != 2)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
        return BLE_DATA_TYPE_AT_CMD;
    }

    /* 解析Report参数 */
    report_value = atoi(msg->parm[1]);
    if (report_value < 0 || report_value > 3)
    {
        LOG_INF("%s=>invalid Report param: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }

    /* 解析Trigger参数 */
    trigger_value = atoi(msg->parm[2]);
    if (trigger_value < 0 || trigger_value > 3)
    {
        LOG_INF("%s=>invalid Trigger param: %s", __func__, msg->parm[2]);
        goto param_invalid;
    }

    /* 所有参数验证通过,统一赋值 */
    gConfigParam.pinstat_config.pinstat_report = (uint8_t)report_value;
    gConfigParam.pinstat_config.pinstat_trigger = (uint8_t)trigger_value;

    LOG_INF("%s=>%s,%s,%s", __func__, msg->parm[0], msg->parm[1], msg->parm[2]);

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
    LOG_INF("PINSTAT: Report=%d, Trigger=%d",
           gConfigParam.pinstat_config.pinstat_report,
           gConfigParam.pinstat_config.pinstat_trigger);

    //TODO 具体逻辑处理

    return BLE_DATA_TYPE_AT_CMD;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
    return BLE_DATA_TYPE_AT_CMD;
}

/********************************************************************
**函数名称:  lockstat_cmd_handler
**入口参数:  msg      ---        AT指令结构体指针
**出口参数:  msg->resp_msg  ---  响应消息
**           msg->resp_length --- 响应长度
**函数功能:  处理LOCKSTAT指令：设置锁状态检测与上报
**指令格式:  LOCKSTAT,[Report],[Trigger]#
**参数说明:  [Report] - 上报方式: 0-GPRS, 1-GPRS+SMS, 2-GPRS+SMS+CALL
**           [Trigger] - 触发上报方式: 0-都不触发, 1-上锁触发, 2-解锁触发, 3-上锁解锁均触发
**返 回 值:  BLE数据类型
*********************************************************************/
static int lockstat_cmd_handler(at_cmd_struc* msg)
{
    uint16_t remaining;
    int report_value;
    int trigger_value;

    remaining = sizeof(msg->resp_msg);

    /* 检查参数数量 */
    if (msg->parm_count != 2)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
        return BLE_DATA_TYPE_AT_CMD;
    }

    /* 解析Report参数 */
    report_value = atoi(msg->parm[1]);
    if (report_value < 0 || report_value > 2)
    {
        LOG_INF("%s=>invalid Report param: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }

    /* 解析Trigger参数 */
    trigger_value = atoi(msg->parm[2]);
    if (trigger_value < 0 || trigger_value > 3)
    {
        LOG_INF("%s=>invalid Trigger param: %s", __func__, msg->parm[2]);
        goto param_invalid;
    }

    /* 所有参数验证通过,统一赋值 */
    gConfigParam.lockstat_config.lockstat_report = (uint8_t)report_value;
    gConfigParam.lockstat_config.lockstat_trigger = (uint8_t)trigger_value;

    LOG_INF("%s=>%s,%s,%s", __func__, msg->parm[0], msg->parm[1], msg->parm[2]);

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
    LOG_INF("LOCKSTAT: Report=%d, Trigger=%d",
           gConfigParam.lockstat_config.lockstat_report,
           gConfigParam.lockstat_config.lockstat_trigger);

    //TODO 具体逻辑处理

    return BLE_DATA_TYPE_AT_CMD;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
    return BLE_DATA_TYPE_AT_CMD;
}

/********************************************************************
**函数名称:  motdet_cmd_handler
**入口参数:  msg      ---        AT指令结构体指针
**出口参数:  msg->resp_msg  ---  响应消息
**           msg->resp_length --- 响应长度
**函数功能:  处理MOTDET指令：设置运动检测参数
**指令格式:  MOTDET,[Static G],[Land G],[Static-Land Length],[Sea Tranceport time],[Report Type]#
**参数说明:  [Static G] - 静止G值方差阈值: 1-500 mg (默认10)
**           [Land G] - 陆运G值方差阈值: 1-3000 mg (默认2000)
**           [Static-Land Length] - 静止进入陆运投票时长: 30-600 s (默认50)
**           [Sea Tranceport time] - 进入海运投票时长: 10-600 s (默认10)
**           [Report Type] - 模式切换上报方式: 0-GPRS, 1-GPRS+SMS, 2-GPRS+SMS+CALL
**返 回 值:  BLE数据类型
*********************************************************************/
static int motdet_cmd_handler(at_cmd_struc* msg)
{
    uint16_t remaining;
    int static_g_value;
    int land_g_value;
    int static_land_length_value;
    int sea_transport_time_value;
    int report_type_value;

    remaining = sizeof(msg->resp_msg);

    /* 检查参数数量 */
    if (msg->parm_count != 5)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
        return BLE_DATA_TYPE_AT_CMD;
    }

    /* 解析Static G参数 */
    static_g_value = atoi(msg->parm[1]);
    if (static_g_value < 1 || static_g_value > 500)
    {
        LOG_INF("%s=>invalid Static G param: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }

    /* 解析Land G参数 */
    land_g_value = atoi(msg->parm[2]);
    if (land_g_value < 1 || land_g_value > 3000)
    {
        LOG_INF("%s=>invalid Land G param: %s", __func__, msg->parm[2]);
        goto param_invalid;
    }

    /* 解析Static-Land Length参数 */
    static_land_length_value = atoi(msg->parm[3]);
    if (static_land_length_value < 30 || static_land_length_value > 600)
    {
        LOG_INF("%s=>invalid Static-Land Length param: %s", __func__, msg->parm[3]);
        goto param_invalid;
    }

    /* 解析Sea Transport time参数 */
    sea_transport_time_value = atoi(msg->parm[4]);
    if (sea_transport_time_value < 10 || sea_transport_time_value > 600)
    {
        LOG_INF("%s=>invalid Sea Transport time param: %s", __func__, msg->parm[4]);
        goto param_invalid;
    }

    /* 解析Report Type参数 */
    report_type_value = atoi(msg->parm[5]);
    if (report_type_value < 0 || report_type_value > 2)
    {
        LOG_INF("%s=>invalid Report Type param: %s", __func__, msg->parm[5]);
        goto param_invalid;
    }

    /* 所有参数验证通过,统一赋值 */
    gConfigParam.motdet_config.motdet_static_g = (uint16_t)static_g_value;
    gConfigParam.motdet_config.motdet_land_g = (uint16_t)land_g_value;
    gConfigParam.motdet_config.motdet_static_land_length = (uint16_t)static_land_length_value;
    gConfigParam.motdet_config.motdet_sea_transport_time = (uint16_t)sea_transport_time_value;
    gConfigParam.motdet_config.motdet_report_type = (uint8_t)report_type_value;

    LOG_INF("%s=>%s,%s,%s,%s,%s,%s", __func__, msg->parm[0], msg->parm[1],
           msg->parm[2], msg->parm[3], msg->parm[4], msg->parm[5]);

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
    LOG_INF("MOTDET: StaticG=%d, LandG=%d, StaticLandLen=%d, SeaTime=%d, ReportType=%d",
           gConfigParam.motdet_config.motdet_static_g,
           gConfigParam.motdet_config.motdet_land_g,
           gConfigParam.motdet_config.motdet_static_land_length,
           gConfigParam.motdet_config.motdet_sea_transport_time,
           gConfigParam.motdet_config.motdet_report_type);

    //TODO 具体逻辑处理

    return BLE_DATA_TYPE_AT_CMD;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
    return BLE_DATA_TYPE_AT_CMD;
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
static int batlevel_cmd_handler(at_cmd_struc* msg)
{
    uint16_t remaining;
    int param_values[6];
    int i;

    remaining = sizeof(msg->resp_msg);

    /* 检查参数数量 */
    if (msg->parm_count != 6)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        goto param_invalid;
    }

    /* 解析所有6个参数 */
    for (i = 0; i < 6; i++)
    {
        param_values[i] = atoi(msg->parm[i + 1]);
        if (param_values[i] < REPORT_MODE_NONE || param_values[i] > REPORT_MODE_GPRS_SMS_CALL)
        {
            LOG_INF("%s=>invalid RPT param: %s", __func__, msg->parm[i]);
            goto param_invalid;
        }
    }

    /* 所有参数验证通过,统一赋值 */
    gConfigParam.batlevel_config.batlevel_empty_rpt = (uint8_t)param_values[0];
    gConfigParam.batlevel_config.batlevel_low_rpt = (uint8_t)param_values[1];
    gConfigParam.batlevel_config.batlevel_normal_rpt = (uint8_t)param_values[2];
    gConfigParam.batlevel_config.batlevel_fair_rpt = (uint8_t)param_values[3];
    gConfigParam.batlevel_config.batlevel_high_rpt = (uint8_t)param_values[4];
    gConfigParam.batlevel_config.batlevel_full_rpt = (uint8_t)param_values[5];

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
    LOG_INF("BATLEVEL: Empty RPT=%d, Low RPT=%d, Normal RPT=%d, Fair RPT=%d, High RPT=%d, Full RPT=%d",
           gConfigParam.batlevel_config.batlevel_empty_rpt, gConfigParam.batlevel_config.batlevel_low_rpt,
           gConfigParam.batlevel_config.batlevel_normal_rpt, gConfigParam.batlevel_config.batlevel_fair_rpt,
           gConfigParam.batlevel_config.batlevel_high_rpt, gConfigParam.batlevel_config.batlevel_full_rpt);

    //TODO 具体逻辑处理

    return BLE_DATA_TYPE_AT_CMD;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
    return BLE_DATA_TYPE_AT_CMD;
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
static int chargesta_cmd_handler(at_cmd_struc* msg)
{
    uint16_t remaining;
    int report_value;

    remaining = sizeof(msg->resp_msg);

    /* 检查参数数量 */
    if (msg->parm_count != 1)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
        return BLE_DATA_TYPE_AT_CMD;
    }

    /* 解析RPT参数 */
    report_value = atoi(msg->parm[1]);
    if (report_value < REPORT_MODE_NONE || report_value > REPORT_MODE_GPRS_SMS_CALL)
    {
        LOG_INF("%s=>invalid RPT param: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }
    gConfigParam.batlevel_config.chargesta_report = (uint8_t)report_value;

    LOG_INF("%s=>%s,%s", __func__, msg->parm[0], msg->parm[1]);

    /* 所有参数验证通过,生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
    LOG_INF("CHARGESTA: Report=%d", gConfigParam.batlevel_config.chargesta_report);

    //TODO 具体逻辑处理

    return BLE_DATA_TYPE_AT_CMD;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
    return BLE_DATA_TYPE_AT_CMD;
}

/********************************************************************
**函数名称:  shockalarm_cmd_handler
**入口参数:  msg      ---        AT指令结构体指针
**出口参数:  msg->resp_msg  ---  响应消息
**           msg->resp_length --- 响应长度
**函数功能:  处理SHOCKALARM指令：设置撞击检测报警功能
**指令格式:  SHOCKALARM,[SW],[Level],[Type of Alarm]#
**参数说明:  [SW] - 功能开关: ON/OFF (默认OFF)
**           [Level] - 撞击力度阈值: 1-5 (默认3; 5最敏感,1最不敏感)
**           [Type of Alarm] - 告警上报方式: 0-GPRS, 1-GPRS+SMS, 2-GPRS+SMS+CALL
**返 回 值:  BLE数据类型
*********************************************************************/
static int shockalarm_cmd_handler(at_cmd_struc* msg)
{
    uint16_t remaining;
    int level_value;
    int type_value;
    int sw_value;

    remaining = sizeof(msg->resp_msg);

    /* 检查参数数量 */
    if (msg->parm_count != 3)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
        return BLE_DATA_TYPE_AT_CMD;
    }

    /* 解析SW参数 */
    if (strcmp(msg->parm[1], "ON") == 0)
    {
        sw_value = 1;
    }
    else if (strcmp(msg->parm[1], "OFF") == 0)
    {
        sw_value = 0;
    }
    else
    {
        LOG_INF("%s=>invalid SW param: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }

    /* 解析Level参数 */
    level_value = atoi(msg->parm[2]);
    if (level_value < 1 || level_value > 5)
    {
        LOG_INF("%s=>invalid Level param: %s", __func__, msg->parm[2]);
        goto param_invalid;
    }

    /* 解析Type of Alarm参数 */
    type_value = atoi(msg->parm[3]);
    if (type_value < 0 || type_value > 2)
    {
        LOG_INF("%s=>invalid Type of Alarm param: %s", __func__, msg->parm[3]);
        goto param_invalid;
    }

    /* 所有参数验证通过,统一赋值 */
    gConfigParam.shockalarm_config.shockalarm_sw = (uint8_t)sw_value;
    gConfigParam.shockalarm_config.shockalarm_level = (uint8_t)level_value;
    gConfigParam.shockalarm_config.shockalarm_type = (uint8_t)type_value;

    LOG_INF("%s=>%s,%s,%s,%s", __func__, msg->parm[0], msg->parm[1], msg->parm[2], msg->parm[3]);

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
    LOG_INF("SHOCKALARM: SW=%d, Level=%d, Type=%d",
           gConfigParam.shockalarm_config.shockalarm_sw,
           gConfigParam.shockalarm_config.shockalarm_level,
           gConfigParam.shockalarm_config.shockalarm_type);

    //TODO 具体逻辑处理

    return BLE_DATA_TYPE_AT_CMD;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
    return BLE_DATA_TYPE_AT_CMD;
}

void shutdown_timeout_timer(void *param)
{
    // 关机定时器到期，发送关机消息
       my_send_msg(MOD_MAIN, MOD_MAIN, MY_MSG_SHUTDOWN);
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
static int pwsave_cmd_handler(at_cmd_struc* msg)
{
    uint16_t remaining;

    remaining = sizeof(msg->resp_msg);

    /* 检查参数数量 (应为1，指令格式为PWRSAVE,ON#) */
    if (msg->parm_count == 1)
    {
        /* 解析参数 */
        if (strcmp(msg->parm[1], "ON") == 0)
        {
            gConfigParam.pwsave_config.pwsave_sw = 1;
            LOG_INF("%s=>%s,%s", __func__, msg->parm[0], msg->parm[1]);

            /* 根据指令说明，立即回复 "Poweroff OK" */
            msg->resp_length = snprintf(msg->resp_msg, remaining, "Poweroff OK");

            // 启动关机定时器，让蓝牙接收到回复，10毫秒后触发关机
            my_start_timer(MY_TIMER_SHUTDOWN, 10, false, shutdown_timeout_timer);

            LOG_INF("PWRSAVE: Device will enter low-power transport state");
        }
        else
        {
            LOG_INF("%s=>invalid param: %s", __func__, msg->parm[1]);
            msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
        }
    }
    else
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
    }
    return BLE_DATA_TYPE_AT_CMD;
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
static int startr_cmd_handler(at_cmd_struc* msg)
{
    uint16_t remaining;

    remaining = sizeof(msg->resp_msg);

    /* 检查参数数量：0表示查询，1表示设置 */
    if (msg->parm_count == 0)
    {
        /* 查询指令：返回当前状态 */
        LOG_INF("%s=>%s (query)", __func__, msg->parm[0]);
        if (gConfigParam.startr_config.startr_sw == 1)
        {
            msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN STARTR:ON");
        }
        else
        {
            msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN STARTR:OFF");
        }
        return BLE_DATA_TYPE_AT_CMD;
    }

    if (msg->parm_count != 1)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
        return BLE_DATA_TYPE_AT_CMD;
    }

    /* 设置指令 */
    LOG_INF("%s=>%s,%s", __func__, msg->parm[0], msg->parm[1]);

    /* 解析A参数 */
    if (strcmp(msg->parm[1], "ON") == 0)
    {
        gConfigParam.startr_config.startr_sw = 1;
    }
    else if (strcmp(msg->parm[1], "OFF") == 0)
    {
        gConfigParam.startr_config.startr_sw = 0;
    }
    else
    {
        LOG_INF("%s=>invalid A param: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }

    /* 所有参数验证通过,生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
    LOG_INF("STARTR: SW=%d", gConfigParam.startr_config.startr_sw);

    //TODO 具体逻辑处理

    return BLE_DATA_TYPE_AT_CMD;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
    return BLE_DATA_TYPE_AT_CMD;
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
static int cbmt_cmd_handler(at_cmd_struc* msg)
{
    uint16_t remaining;
    uint16_t battery_voltage_mv;
    const char* charge_status;
    int ret;

    remaining = sizeof(msg->resp_msg);

    /* 检查参数数量：应为0 */
    if (msg->parm_count == 0)
    {
        LOG_INF("%s=>%s", __func__, msg->parm[0]);

        my_battery_read_mv(&battery_voltage_mv);
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
            msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
        }
    }
    else
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
    }
    return BLE_DATA_TYPE_AT_CMD;
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
static int bt_crfpwr_cmd_handler(at_cmd_struc* msg)
{
    uint16_t remaining;
    int a_value;

    remaining = sizeof(msg->resp_msg);

    /* 检查参数数量 */
    if (msg->parm_count != 1)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
        return BLE_DATA_TYPE_AT_CMD;
    }

    /* 解析A参数 */
    a_value = atoi(msg->parm[1]);
    if (a_value != -8 && a_value != -4 && a_value != 0 && a_value != 3
        && a_value != 5 && a_value != 7)
    {
        LOG_INF("%s=>invalid A param: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }

    gConfigParam.ble_tx_power.tx_power = (int8_t)a_value;
    ble_set_tx_power(gConfigParam.ble_tx_power.tx_power);

    LOG_INF("%s=>%s,%s", __func__, msg->parm[0], msg->parm[1]);

    /* 所有参数验证通过,生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
    LOG_INF("BT_CRFPWR: A=%d",gConfigParam.ble_tx_power.tx_power);

    return BLE_DATA_TYPE_AT_CMD;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
    return BLE_DATA_TYPE_AT_CMD;
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
**           1：当设备需要开启Cell时才开启收集上传
**           2：设备持续按[Scan Interval]和[Scan Length]收集数据，Cell启动时上传，未启动时仅存储
**           3：设备持续收集数据，Cell启动时上传；未启动时若距离上次上传达到[Updata interval]，则唤醒Cell和GNSS上传
**           Scan Interval - 蓝牙数据收集间隔(默认：600秒)，范围：1-86400秒
**           Scan Length - 每次收集的搜索时长(默认：10秒)，范围：1-86400秒
**           Updata interval - 蓝牙唤醒间隔(默认：14400秒)，范围：1-86400秒
**返 回 值:  BLE数据类型
*********************************************************************/
static int bt_updata_cmd_handler(at_cmd_struc* msg)
{
    uint16_t remaining;
    int mode_value;
    uint32_t scan_interval_value;
    uint32_t scan_length_value;
    uint32_t updata_interval_value;

    remaining = sizeof(msg->resp_msg);

    /* 检查参数数量 */
    if (msg->parm_count != 4)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
        return BLE_DATA_TYPE_AT_CMD;
    }

    /* 解析Mode参数 */
    mode_value = atoi(msg->parm[1]);
    if (mode_value < 0 || mode_value > 3)
    {
        LOG_INF("%s=>invalid Mode param: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }

    /* 解析Scan Interval参数 */
    scan_interval_value = atoi(msg->parm[2]);
    if (scan_interval_value < 1 || scan_interval_value > 86400)
    {
        LOG_INF("%s=>invalid Scan Interval param: %s", __func__, msg->parm[2]);
        goto param_invalid;
    }

    /* 解析Scan Length参数 */
    scan_length_value = atoi(msg->parm[3]);
    if (scan_length_value < 1 || scan_length_value > 86400)
    {
        LOG_INF("%s=>invalid Scan Length param: %s", __func__, msg->parm[3]);
        goto param_invalid;
    }

    /* 解析Updata interval参数 */
    updata_interval_value = atoi(msg->parm[4]);
    if (updata_interval_value < 1 || updata_interval_value > 86400)
    {
        LOG_INF("%s=>invalid Updata interval param: %s", __func__, msg->parm[4]);
        goto param_invalid;
    }

    /* 所有参数验证通过,统一赋值 */
    gConfigParam.bt_updata_config.bt_updata_mode = (uint8_t)mode_value;
    gConfigParam.bt_updata_config.bt_updata_scan_interval = scan_interval_value;
    gConfigParam.bt_updata_config.bt_updata_scan_length = scan_length_value;
    gConfigParam.bt_updata_config.bt_updata_updata_interval = updata_interval_value;

    LOG_INF("%s=>%s,%s,%s,%s,%s", __func__, msg->parm[0], msg->parm[1],
           msg->parm[2], msg->parm[3], msg->parm[4]);

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
    LOG_INF("BT_UPDATA: Mode=%d, ScanInterval=%u, ScanLength=%u, UpdataInterval=%u",
           gConfigParam.bt_updata_config.bt_updata_mode,
           gConfigParam.bt_updata_config.bt_updata_scan_interval,
           gConfigParam.bt_updata_config.bt_updata_scan_length,
           gConfigParam.bt_updata_config.bt_updata_updata_interval);

    //TODO 具体逻辑处理

    return BLE_DATA_TYPE_AT_CMD;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
    return BLE_DATA_TYPE_AT_CMD;
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
static int tag_cmd_handler(at_cmd_struc* msg)
{
    uint16_t remaining;
    int interval_value;
    int sw_value;

    remaining = sizeof(msg->resp_msg);

    /* 检查参数数量：支持1个参数(TAG,ON)或2个参数(TAG,SW,Interval) */
    if (msg->parm_count != 1 && msg->parm_count != 2)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
        return BLE_DATA_TYPE_AT_CMD;
    }

    /* 解析SW参数 */
    if (strcmp(msg->parm[1], "ON") == 0)
    {
        sw_value = 1;
    }
    else if (strcmp(msg->parm[1], "OFF") == 0)
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
        interval_value = atoi(msg->parm[2]);
        if (interval_value < 100 || interval_value > 60000)
        {
            LOG_INF("%s=>invalid Interval param: %s", __func__, msg->parm[2]);
            goto param_invalid;
        }
    }

    /* 所有参数验证通过,统一赋值 */
    gConfigParam.tag_config.tag_sw = (uint8_t)sw_value;
    if (msg->parm_count == 2)
    {
        gConfigParam.tag_config.tag_interval = (uint16_t)interval_value;
    }

    LOG_INF("%s=>%s,%s", __func__, msg->parm[0], msg->parm[1]);
    if (msg->parm_count == 2)
    {
        LOG_INF("%s=>%s", __func__, msg->parm[2]);
    }

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
    LOG_INF("TAG: SW=%d, Interval=%u", gConfigParam.tag_config.tag_sw, gConfigParam.tag_config.tag_interval);

    //更新非连接广播参数，里面会按配置打开或关闭广播，根据tag_sw的值
    my_ble_updata_adv_param(gConfigParam.tag_config.tag_interval);

    return BLE_DATA_TYPE_AT_CMD;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
    return BLE_DATA_TYPE_AT_CMD;
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
static int jatag_cmd_handler(at_cmd_struc* msg)
{
    uint16_t remaining;
    int sw_value;

    remaining = sizeof(msg->resp_msg);

    /* 检查参数数量：支持1个参数(JATAG,ON)*/
    if (msg->parm_count != 1)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
        return BLE_DATA_TYPE_AT_CMD;
    }

    /* 解析SW参数 */
    if (strcmp(msg->parm[1], "ON") == 0)
    {
        sw_value = 1;
    }
    else if (strcmp(msg->parm[1], "OFF") == 0)
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
    gConfigParam.adv_valid_value.AppleValid = (uint8_t)sw_value;
    set_adv_valid_status(APPLE_ADV_TYPE, gConfigParam.adv_valid_value.AppleValid);
    my_no_con_start_adv(gConfigParam.tag_config.tag_sw);

    LOG_INF("%s=>%s,%s", __func__, msg->parm[0], msg->parm[1]);

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
    LOG_INF("JATAG: SW=%d, Interval=%u", gConfigParam.adv_valid_value.AppleValid, gConfigParam.tag_config.tag_interval);

    return BLE_DATA_TYPE_AT_CMD;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
    return BLE_DATA_TYPE_AT_CMD;
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
static int jgtag_cmd_handler(at_cmd_struc* msg)
{
    uint16_t remaining;
    int sw_value;

    remaining = sizeof(msg->resp_msg);

    /* 检查参数数量：支持1个参数(JGTAG,ON)*/
    if (msg->parm_count != 1)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
        return BLE_DATA_TYPE_AT_CMD;
    }

    /* 解析SW参数 */
    if (strcmp(msg->parm[1], "ON") == 0)
    {
        sw_value = 1;
    }
    else if (strcmp(msg->parm[1], "OFF") == 0)
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
    gConfigParam.adv_valid_value.GoogleValid = (uint8_t)sw_value;
    set_adv_valid_status(GOOGLE_ADV_TYPE, gConfigParam.adv_valid_value.GoogleValid);
    my_no_con_start_adv(gConfigParam.tag_config.tag_sw);

    LOG_INF("%s=>%s,%s", __func__, msg->parm[0], msg->parm[1]);

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
    LOG_INF("JGTAG: SW=%d, Interval=%u", gConfigParam.adv_valid_value.GoogleValid, gConfigParam.tag_config.tag_interval);

    return BLE_DATA_TYPE_AT_CMD;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
    return BLE_DATA_TYPE_AT_CMD;
}

/********************************************************************
**函数名称:  lockcd_cmd_handler
**入口参数:  msg      ---        AT指令结构体指针
**出口参数:  msg->resp_msg  ---  响应消息
**           msg->resp_length --- 响应长度
**函数功能:  处理LOCKCD指令：设置锁销插入后自动上锁倒计时
**指令格式:  LOCKCD,[Count down time]#
**参数说明:  Count down time - 插入后上锁倒计时(单位：s)，范围：0-3600秒；设置为0代表不自动上锁，默认值：3秒
**返 回 值:  BLE数据类型
*********************************************************************/
static int lockcd_cmd_handler(at_cmd_struc* msg)
{
    uint16_t remaining;
    int countdown_value;

    remaining = sizeof(msg->resp_msg);

    /* 检查参数数量 */
    if (msg->parm_count != 1)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
        return BLE_DATA_TYPE_AT_CMD;
    }

    /* 解析Count down time参数 */
    countdown_value = atoi(msg->parm[1]);
    if (countdown_value < 0 || countdown_value > 3600)
    {
        LOG_INF("%s=>invalid Count down time param: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }

    /* 所有参数验证通过,统一赋值 */
    gConfigParam.locked_config.lockcd_countdown = (uint16_t)countdown_value;

    LOG_INF("%s=>%s,%s", __func__, msg->parm[0], msg->parm[1]);

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
    LOG_INF("LOCKCD: Countdown=%u", gConfigParam.locked_config.lockcd_countdown);

    //TODO 具体逻辑处理

    return BLE_DATA_TYPE_AT_CMD;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
    return BLE_DATA_TYPE_AT_CMD;
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
static int led_cmd_handler(at_cmd_struc* msg)
{
    uint16_t remaining;
    int display_value;

    remaining = sizeof(msg->resp_msg);

    /* 检查参数数量 */
    if (msg->parm_count != 1)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
        return BLE_DATA_TYPE_AT_CMD;
    }

    /* 解析A参数 */
    if (strcmp(msg->parm[1], "ON") == 0)
    {
        display_value = 1;
        lte_send_command(msg->parm[1], "1");
        my_send_msg(MOD_BLE, MOD_CTRL, MY_MSG_OPEN_LED_SHOW);
    }
    else if (strcmp(msg->parm[1], "OFF") == 0)
    {
        display_value = 0;
        lte_send_command(msg->parm[1], "0");
        my_send_msg(MOD_BLE, MOD_CTRL, MY_MSG_CLOSE_LED_SHOW);
    }
    else
    {
        LOG_INF("%s=>invalid A param: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }

    /* 所有参数验证通过,统一赋值 */
    gConfigParam.led_config.led_display = (uint8_t)display_value;

    LOG_INF("%s=>%s,%s", __func__, msg->parm[0], msg->parm[1]);

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
    LOG_INF("LED: Display=%d", gConfigParam.led_config.led_display);

    //TODO 具体逻辑处理

    return BLE_DATA_TYPE_AT_CMD;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
    return BLE_DATA_TYPE_AT_CMD;
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
static int buzzer_cmd_handler(at_cmd_struc* msg)
{
    uint16_t remaining;
    int operator_value;

    remaining = sizeof(msg->resp_msg);

    /* 检查参数数量 */
    if (msg->parm_count != 1)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
        return BLE_DATA_TYPE_AT_CMD;
    }

    /* 解析Operater参数 */
    operator_value = atoi(msg->parm[1]);
    if (operator_value < 0 || operator_value > 5)
    {
        LOG_INF("%s=>invalid Operater param: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }

    /* 所有参数验证通过,统一赋值 */
    gConfigParam.buzzer_config.buzzer_operator = (uint8_t)operator_value;

    LOG_INF("%s=>%s,%s", __func__, msg->parm[0], msg->parm[1]);

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
    LOG_INF("BUZZER: Operator=%d", gConfigParam.buzzer_config.buzzer_operator);

    //TODO 具体逻辑处理

    return BLE_DATA_TYPE_AT_CMD;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
    return BLE_DATA_TYPE_AT_CMD;
}

/********************************************************************
** 函数名称:  nfctrig_cmd_handler
** 入口参数:  msg      ---  AT指令结构体指针
** 出口参数:  msg->resp_msg   ---  响应消息
**           msg->resp_length --- 响应长度
** 函数功能:  处理 NFCTRIG 指令，实现 NFC 卡号与执行指令的关联功能
**
** 指令格式说明:
** 1. 添加/设置: NFCTRIG,ADD,[NFC NO.],"[Command]"#
**    - 功能: 绑定卡号与指令。
** 2. 查询:     NFCTRIG,CHECK#
**    - 功能: 查询当前所有已绑定的规则。
**    - 回复格式: [NFC NO.]_[Command];[NFC NO.]_[Command]...
** 3. 删除:     NFCTRIG,DEL,ALL#
**    - 功能: 删除所有绑定。
** 4. 删除特定: NFCTRIG,DEL,[NFC NO.]#
**    - 功能: 删除指定卡号的绑定。
**
** 返 回 值:  BLE数据类型
*********************************************************************/
static int nfctrig_cmd_handler(at_cmd_struc* msg)
{
    uint16_t remaining;
    int nfc_no_len;
    int command_len;
    int i = 0;
    int index = 0;
    uint8_t found = 0;
    char temp_buf[256];
    int temp_len = 0;

    remaining = sizeof(msg->resp_msg);

    /* 参数数量基础校验 */
    if (msg->parm_count < 1)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        goto param_invalid;
    }

    if (strcmp(msg->parm[1], "ADD") == 0)
    {
        /* 检查参数数量 */
        if (msg->parm_count != 3)
        {
            LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
            goto param_invalid;
        }

        /* 校验NFC NO.参数 */
        nfc_no_len = strlen(msg->parm[2]);
        if (nfc_no_len == 0 || nfc_no_len >= sizeof(gConfigParam.nfctrig_config.nfctrig_table.nfctrig_rule[0].nfctrig_nfc_no))
        {
            LOG_INF("%s=>invalid NFC NO. param: %s", __func__, msg->parm[2]);
            goto param_invalid;
        }

        /* 校验Command参数 */
        command_len = strlen(msg->parm[3]);
        if (command_len == 0 || command_len >= sizeof(gConfigParam.nfctrig_config.nfctrig_table.nfctrig_rule[0].nfctrig_command))
        {
            LOG_INF("%s=>invalid Command param: %s", __func__, msg->parm[3]);
            goto param_invalid;
        }

        /* 检查Command参数是否为NFCTRIG,ADD */
        if (strstr(msg->parm[3], "NFCTRIG,ADD") != NULL)
        {
            LOG_INF("%s=>Command param cannot be 'NFCTRIG,ADD'", __func__);
            goto param_invalid;
        }

        /* 1. 判满 */
        if (gConfigParam.nfctrig_config.nfctrig_table.count >= NFCTRIG_MAX_RULES)
        {
            LOG_INF("table full");
            msg->resp_length = snprintf(msg->resp_msg, remaining, "Card limit reached. Please delete before adding.");
            return BLE_DATA_TYPE_AT_CMD;
        }

        /* 2. 查重 */
        for (i = 0; i < gConfigParam.nfctrig_config.nfctrig_table.count; i++)
        {
            //比较卡号
            if (strcmp((char*)gConfigParam.nfctrig_config.nfctrig_table.nfctrig_rule[i].nfctrig_nfc_no, msg->parm[2]) == 0)
            {
                LOG_INF("The card number already exists.Please delete it before setting again.");
                msg->resp_length = snprintf(msg->resp_msg, remaining, "The card number already exists.Please delete it before setting again.");
                return BLE_DATA_TYPE_AT_CMD;
            }
        }
            /* 3. 插入 */
        index = gConfigParam.nfctrig_config.nfctrig_table.count;

        strcpy(gConfigParam.nfctrig_config.nfctrig_table.nfctrig_rule[index].nfctrig_nfc_no, msg->parm[2]);
        strcpy(gConfigParam.nfctrig_config.nfctrig_table.nfctrig_rule[index].nfctrig_command, msg->parm[3]);
        gConfigParam.nfctrig_config.nfctrig_table.count++;

        LOG_INF("%s=>%s,%s,%s,%s", __func__, msg->parm[0], msg->parm[1], msg->parm[2], msg->parm[3]);


        LOG_INF("NFCTRIG_ADD: NFC_NO=%s, Command=%s",
            gConfigParam.nfctrig_config.nfctrig_table.nfctrig_rule[index].nfctrig_nfc_no,
            gConfigParam.nfctrig_config.nfctrig_table.nfctrig_rule[index].nfctrig_command);

        /* 生成成功响应 */
        msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_%s_OK", msg->parm[0], msg->parm[1]);
        return BLE_DATA_TYPE_AT_CMD;

    }
    else if (strcmp(msg->parm[1], "CHECK") == 0)
    {
        /* CHECK指令参数数量校验：需要1个参数 */
        if (msg->parm_count != 1)
        {
            LOG_INF("%s=>DEL param count error: %d", __func__, msg->parm_count);
            goto param_invalid;
        }

        for (i = 0; i < gConfigParam.nfctrig_config.nfctrig_table.count; i++)
        {
            if (i == gConfigParam.nfctrig_config.nfctrig_table.count - 1)
            {
                temp_len += snprintf(temp_buf + temp_len, sizeof(temp_buf) - temp_len,
                                "%s_%s", gConfigParam.nfctrig_config.nfctrig_table.nfctrig_rule[i].nfctrig_nfc_no,
                                gConfigParam.nfctrig_config.nfctrig_table.nfctrig_rule[i].nfctrig_command);
            }
            else
            {
                temp_len += snprintf(temp_buf + temp_len, sizeof(temp_buf) - temp_len,
                                "%s_%s;", gConfigParam.nfctrig_config.nfctrig_table.nfctrig_rule[i].nfctrig_nfc_no,
                                gConfigParam.nfctrig_config.nfctrig_table.nfctrig_rule[i].nfctrig_command);
            }
            found = 1;
        }

        if (found)
        {
            //TODO 回传的数据有可能太大返回有问题，后续需要分包回传处理
            msg->resp_length = snprintf(msg->resp_msg, remaining, "%s", temp_buf);
            LOG_INF("%s=>CHECK,COUNT=%d", __func__, gConfigParam.nfctrig_config.nfctrig_table.count);
            return BLE_DATA_TYPE_AT_CMD;
        }
        else
        {
            LOG_INF("%s=>CHECK not found", __func__);
            goto param_invalid;
        }

    }
    else if (strcmp(msg->parm[1], "DEL") == 0)
    {
        /* DEL指令参数数量校验 */
        if (msg->parm_count != 2)
        {
            LOG_INF("%s=>DEL param count error: %d", __func__, msg->parm_count);
            goto param_invalid;
        }

        /* 删除所有已授权卡号 */
        if (strcmp(msg->parm[2], "ALL") == 0)
        {
            memset(gConfigParam.nfctrig_config.nfctrig_table.nfctrig_rule, 0, sizeof(gConfigParam.nfctrig_config.nfctrig_table.nfctrig_rule));
            gConfigParam.nfctrig_config.nfctrig_table.count = 0;
            LOG_INF("%s=>DEL ALL", __func__);
            msg->resp_length = snprintf(msg->resp_msg, remaining, "NFCTRIG Delete ALL Success.");
            return BLE_DATA_TYPE_AT_CMD;
        }
        /* 删除指定卡号 */
        else
        {
            for (i = 0; i < gConfigParam.nfctrig_config.nfctrig_table.count; i++)
            {
                if (strcmp(gConfigParam.nfctrig_config.nfctrig_table.nfctrig_rule[i].nfctrig_nfc_no, msg->parm[2]) == 0)
                {
                    found = 1;
                    /* 将后面的卡前移，覆盖被删除的卡 */
                    for (index = i; index < gConfigParam.nfctrig_config.nfctrig_table.count - 1; index++)
                    {
                        memcpy(&gConfigParam.nfctrig_config.nfctrig_table.nfctrig_rule[index],
                               &gConfigParam.nfctrig_config.nfctrig_table.nfctrig_rule[index + 1],
                               sizeof(nfctrig_rule_t));
                    }
                    /* 清空最后一个位置 */
                    memset(&gConfigParam.nfctrig_config.nfctrig_table.nfctrig_rule[gConfigParam.nfctrig_config.nfctrig_table.count - 1],
                           0, sizeof(nfctrig_rule_t));
                    gConfigParam.nfctrig_config.nfctrig_table.count--;
                    break;
                }
            }
            if (found)
            {
                LOG_INF("%s=>DEL,%s", __func__, msg->parm[2]);
                msg->resp_length = snprintf(msg->resp_msg, remaining, "NFCTRIG %s Delete Success.", msg->parm[2]);
                return BLE_DATA_TYPE_AT_CMD;
            }
            else
            {
                LOG_INF("%s=>DEL,%s not found", __func__, msg->parm[2]);
                goto param_invalid;
            }
        }
    }
    /* 无效的操作类型 */
    else
    {
        LOG_INF("%s=>invalid operation: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }

param_invalid:
    if (msg->parm_count >= 1)
    {
        msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_%s_FAIL", msg->parm[0], msg->parm[1]);
    }
    else
    {
        msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
    }

    return BLE_DATA_TYPE_AT_CMD;
}

/********************************************************************
**函数名称:  nfcauth_cmd_handler
**入口参数:  msg      ---        AT指令结构体指针
**出口参数:  msg->resp_msg  ---  响应消息
**           msg->resp_length --- 响应长度
**函数功能:  处理NFCAUTH指令：NFC卡权限管理
**指令格式:  NFCAUTH,SET,[NFC NO],[LAT],[LON],[半径],[Start Time],[End Time],[Unlock Times]#
**           NFCAUTH,PSET,[NFC NO]#
**           NFCAUTH,DEL,[ALL]#
**           NFCAUTH,DEL,[NFC NO]#
**           NFCAUTH,CHECK#
**           NFCAUTH,CHECK,[NFC NO]#
**参数说明:  SET - 设置NFC卡权限
**           PSET - 快速设置管理员卡（等价于SET，任意地点、时间、不限次数）
**           DEL - 删除已授权卡号
**           CHECK - 查询已设置卡号
**           NFC NO - NFC卡号
**           LAT/LON - 经纬度，均为空字符串表示不限制
**           半径 - 可操作地址半径，单位米，范围50~999900
**           Start Time/End Time - 起止时间，格式必须为YYMMDDHHMM，均为空字符串表示不限制
**           Unlock Times - 可用次数，0表示不限次数，范围0/1~999
**返 回 值:  BLE数据类型
*********************************************************************/
static int nfcauth_cmd_handler(at_cmd_struc* msg)
{
    uint16_t remaining;
    int i;
    int index;
    int found;
    int nfc_no_len;
    int32_t lat_value;
    uint8_t lat_valid;
    int32_t lon_value;
    uint8_t lon_valid;
    int32_t radius;
    uint8_t starttime_valid;
    uint8_t endtime_valid;
    int32_t unlock_times;
    char temp_buf[256];
    int temp_len;

    remaining = sizeof(msg->resp_msg);

#if 0
    for(int j = 0; j < msg->parm_count + 1; j++)
    {
        LOG_INF("=====>%s", msg->parm[j]);
    }
#endif

    /* 参数数量基础校验 */
    if (msg->parm_count < 1)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        goto param_invalid;
    }

    /* 处理SET指令：设置NFC卡权限 */
    if (strcmp(msg->parm[1], "SET") == 0)
    {
        /* SET指令参数数量校验：需要8个参数 */
        if (msg->parm_count != 8)
        {
            LOG_INF("%s=>SET param count error: %d", __func__, msg->parm_count);
            goto param_invalid;
        }

        /* NFC卡号参数校验 */
        nfc_no_len = strlen(msg->parm[2]);
        if (nfc_no_len == 0 || nfc_no_len >= sizeof(gConfigParam.nfcauth_config.nfcauth_cards[0].nfc_no))
        {
            LOG_INF("%s=>invalid NFC NO. param: %s", __func__, msg->parm[2]);
            goto param_invalid;
        }

        /* 经纬度参数解析和校验 */
        if (parse_coordinate_value(msg->parm[3], 1, &lat_value, &lat_valid) != 0)
        {
            LOG_INF("%s=>invalid LAT param: %s", __func__, msg->parm[3]);
            goto param_invalid;
        }

        if (parse_coordinate_value(msg->parm[4], 0, &lon_value, &lon_valid) != 0)
        {
            LOG_INF("%s=>invalid LON param: %s", __func__, msg->parm[4]);
            goto param_invalid;
        }

        if ((lat_valid == 1 && lon_valid == 0) || (lat_valid == 0 && lon_valid == 1))
        {
            LOG_INF("%s=>invalid LAT or LON param", __func__);
            goto param_invalid;
        }

        /* 半径参数校验：非空时范围50~999900米，空字符串表示无效 */
        temp_len = strlen(msg->parm[5]);
        if (temp_len == 0)
        {
            LOG_INF("%s=>invalid radius param", __func__);
            goto param_invalid;
        }

        if (!is_integer_array(msg->parm[5], temp_len))
        {
            LOG_INF("%s=>invalid radius param: %s (not pure digit)", __func__, msg->parm[5]);
            goto param_invalid;
        }
        else
        {
            radius = atoi(msg->parm[5]);
            if (radius < 50 || radius > 999900)
            {
                LOG_INF("%s=>invalid radius param: %s", __func__, msg->parm[5]);
                goto param_invalid;
            }
        }

        /* 起止时间参数格式校验：必须为YYMMDDHHMM或空字符串 */
        if (validate_time_format(msg->parm[6], &starttime_valid) != 0)
        {
            LOG_INF("%s=>invalid start_time param format: %s", __func__, msg->parm[6]);
            goto param_invalid;
        }

        if (validate_time_format(msg->parm[7], &endtime_valid) != 0)
        {
            LOG_INF("%s=>invalid end_time param format: %s", __func__, msg->parm[7]);
            goto param_invalid;
        }

        if ((starttime_valid == 1 && endtime_valid == 0)
        || (starttime_valid == 0 && endtime_valid == 1))
        {
            LOG_INF("%s=>invalid start_time or end_time param", __func__);
            goto param_invalid;
        }

        if (starttime_valid == 1 && endtime_valid == 1)
        {
            /* 比较开始时间和结束时间，开始时间必须小于结束时间 */
            if (strcmp(msg->parm[6], msg->parm[7]) >= 0)
            {
                LOG_INF("%s=>start_time must be less than end_time: %s >= %s", __func__, msg->parm[6], msg->parm[7]);
                goto param_invalid;
            }
        }

        /* 可用次数参数校验：非空时范围1~999 */
        temp_len = strlen(msg->parm[8]);
        if (temp_len != 0)
        {
            if (!is_integer_array(msg->parm[8], temp_len))
            {
                LOG_INF("%s=>invalid unlock_times param: %s (not pure digit)", __func__, msg->parm[8]);
                goto param_invalid;
            }
            else
            {
                unlock_times = atoi(msg->parm[8]);
                // 仅允许 unlock_times 为 -1 或 1~999
                if (!(unlock_times == -1 || (unlock_times >= 1 && unlock_times <= 999)))
                {
                    LOG_INF("%s=>invalid unlock_times param: %s", __func__, msg->parm[8]);
                    goto param_invalid;
                }
            }
        }
        else
        {
            unlock_times = -1;// 空字符串时，标记为-1，表示不限次数
        }

        /* 查找是否已存在该卡号，存在则更新，不存在则新增 */
        found = 0;
        for (i = 0; i < gConfigParam.nfcauth_config.nfcauth_card_count; i++)
        {
            if (strcmp(gConfigParam.nfcauth_config.nfcauth_cards[i].nfc_no, msg->parm[2]) == 0)
            {
                index = i;
                found = 1;
                break;
            }
        }

        /* 新增卡号时检查是否已达到最大数量限制 */
        if (!found)
        {
            if (gConfigParam.nfcauth_config.nfcauth_card_count >= 10)
            {
                LOG_INF("%s=>card list full", __func__);
                msg->resp_length = snprintf(msg->resp_msg, remaining, "Card limit reached. Please delete before adding.");
                return BLE_DATA_TYPE_AT_CMD;
            }
            index = gConfigParam.nfcauth_config.nfcauth_card_count;
            gConfigParam.nfcauth_config.nfcauth_card_count++;
        }

        /* 保存卡权限信息 */
        strcpy(gConfigParam.nfcauth_config.nfcauth_cards[index].nfc_no, msg->parm[2]);
        gConfigParam.nfcauth_config.nfcauth_cards[index].lat = lat_value;
        gConfigParam.nfcauth_config.nfcauth_cards[index].lon = lon_value;
        if (lat_valid == 1 && lon_valid == 1)
        {
            gConfigParam.nfcauth_config.nfcauth_cards[index].lat_lon_valid = 1;
        }
        else
        {
            gConfigParam.nfcauth_config.nfcauth_cards[index].lat_lon_valid = 0;
        }
        gConfigParam.nfcauth_config.nfcauth_cards[index].radius = radius;
        strcpy(gConfigParam.nfcauth_config.nfcauth_cards[index].start_time, msg->parm[6]);
        strcpy(gConfigParam.nfcauth_config.nfcauth_cards[index].end_time, msg->parm[7]);
        if (starttime_valid == 1 && endtime_valid == 1)
        {
            gConfigParam.nfcauth_config.nfcauth_cards[index].time_valid = 1;
        }
        else
        {
            gConfigParam.nfcauth_config.nfcauth_cards[index].time_valid = 0;
        }
        gConfigParam.nfcauth_config.nfcauth_cards[index].unlock_times = unlock_times;

        LOG_INF("%s=>SET,%s,%d,%d,%d,%u,%s,%s,%d,%u", __func__,
                gConfigParam.nfcauth_config.nfcauth_cards[index].nfc_no,
                gConfigParam.nfcauth_config.nfcauth_cards[index].lat,
                gConfigParam.nfcauth_config.nfcauth_cards[index].lon,
                gConfigParam.nfcauth_config.nfcauth_cards[index].lat_lon_valid,
                gConfigParam.nfcauth_config.nfcauth_cards[index].radius,
                gConfigParam.nfcauth_config.nfcauth_cards[index].start_time,
                gConfigParam.nfcauth_config.nfcauth_cards[index].end_time,
                gConfigParam.nfcauth_config.nfcauth_cards[index].time_valid,
                gConfigParam.nfcauth_config.nfcauth_cards[index].unlock_times);

        msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
        return BLE_DATA_TYPE_AT_CMD;
    }
    /* 处理PSET指令：快速设置管理员卡（任意地点、时间、不限次数） */
    else if (strcmp(msg->parm[1], "PSET") == 0)
    {
        /* PSET指令参数数量校验：需要2个参数 */
        if (msg->parm_count != 2)
        {
            LOG_INF("%s=>PSET param count error: %d", __func__, msg->parm_count);
            goto param_invalid;
        }

        /* NFC卡号参数校验 */
        nfc_no_len = strlen(msg->parm[2]);
        if (nfc_no_len == 0 || nfc_no_len >= sizeof(gConfigParam.nfcauth_config.nfcauth_cards[0].nfc_no))
        {
            LOG_INF("%s=>invalid NFC NO. param: %s", __func__, msg->parm[2]);
            goto param_invalid;
        }

        /* 查找是否已存在该卡号，存在则更新，不存在则新增 */
        found = 0;
        for (i = 0; i < gConfigParam.nfcauth_config.nfcauth_card_count; i++)
        {
            if (strcmp(gConfigParam.nfcauth_config.nfcauth_cards[i].nfc_no, msg->parm[2]) == 0)
            {
                index = i;
                found = 1;
                break;
            }
        }

        /* 新增卡号时检查是否已达到最大数量限制 */
        if (!found)
        {
            if (gConfigParam.nfcauth_config.nfcauth_card_count >= 10)
            {
                LOG_INF("%s=>card list full", __func__);
                msg->resp_length = snprintf(msg->resp_msg, remaining, "Card limit reached. Please delete before adding.");
                return BLE_DATA_TYPE_AT_CMD;
            }
            index = gConfigParam.nfcauth_config.nfcauth_card_count;
            gConfigParam.nfcauth_config.nfcauth_card_count++;
        }

        /* 设置管理员卡权限：所有限制项均设为0 */
        strcpy(gConfigParam.nfcauth_config.nfcauth_cards[index].nfc_no, msg->parm[2]);
        gConfigParam.nfcauth_config.nfcauth_cards[index].lat = 0;
        gConfigParam.nfcauth_config.nfcauth_cards[index].lon = 0;
        gConfigParam.nfcauth_config.nfcauth_cards[index].lat_lon_valid = 0;
        gConfigParam.nfcauth_config.nfcauth_cards[index].radius = 0;
        strcpy(gConfigParam.nfcauth_config.nfcauth_cards[index].start_time, "");
        strcpy(gConfigParam.nfcauth_config.nfcauth_cards[index].end_time, "");
        gConfigParam.nfcauth_config.nfcauth_cards[index].time_valid = 0;
        gConfigParam.nfcauth_config.nfcauth_cards[index].unlock_times = 0;

        LOG_INF("%s=>PSET,%s", __func__, gConfigParam.nfcauth_config.nfcauth_cards[index].nfc_no);

        msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
        return BLE_DATA_TYPE_AT_CMD;
    }
    /* 处理DEL指令：删除已授权卡号 */
    else if (strcmp(msg->parm[1], "DEL") == 0)
    {
        /* DEL指令参数数量校验：需要2个参数 */
        if (msg->parm_count != 2)
        {
            LOG_INF("%s=>DEL param count error: %d", __func__, msg->parm_count);
            goto param_invalid;
        }

        /* 删除所有已授权卡号 */
        if (strcmp(msg->parm[2], "ALL") == 0)
        {
            memset(gConfigParam.nfcauth_config.nfcauth_cards, 0, sizeof(gConfigParam.nfcauth_config.nfcauth_cards));
            gConfigParam.nfcauth_config.nfcauth_card_count = 0;
            LOG_INF("%s=>DEL ALL", __func__);
            msg->resp_length = snprintf(msg->resp_msg, remaining, "NFC Delete ALL Success.");
            return BLE_DATA_TYPE_AT_CMD;
        }
        /* 删除指定卡号 */
        else
        {
            found = 0;
            for (i = 0; i < gConfigParam.nfcauth_config.nfcauth_card_count; i++)
            {
                if (strcmp(gConfigParam.nfcauth_config.nfcauth_cards[i].nfc_no, msg->parm[2]) == 0)
                {
                    found = 1;
                    /* 将后面的卡前移，覆盖被删除的卡 */
                    for (index = i; index < gConfigParam.nfcauth_config.nfcauth_card_count - 1; index++)
                    {
                        memcpy(&gConfigParam.nfcauth_config.nfcauth_cards[index],
                               &gConfigParam.nfcauth_config.nfcauth_cards[index + 1],
                               sizeof(NfcAuthCard));
                    }
                    /* 清空最后一个位置 */
                    memset(&gConfigParam.nfcauth_config.nfcauth_cards[gConfigParam.nfcauth_config.nfcauth_card_count - 1],
                           0, sizeof(NfcAuthCard));
                    gConfigParam.nfcauth_config.nfcauth_card_count--;
                    break;
                }
            }

            if (found)
            {
                LOG_INF("%s=>DEL,%s", __func__, msg->parm[2]);
                msg->resp_length = snprintf(msg->resp_msg, remaining, "NFC %s Delete Success.", msg->parm[2]);
                return BLE_DATA_TYPE_AT_CMD;
            }
            else
            {
                LOG_INF("%s=>DEL,%s not found", __func__, msg->parm[2]);
                goto param_invalid;
            }
        }
    }
    /* 处理CHECK指令：查询已设置卡号 */
    else if (strcmp(msg->parm[1], "CHECK") == 0)
    {
        /* 查询所有已设置卡号列表 */
        if (msg->parm_count == 1)
        {
            for (i = 0; i < gConfigParam.nfcauth_config.nfcauth_card_count; i++)
            {
                if (i == gConfigParam.nfcauth_config.nfcauth_card_count - 1)
                {
                    temp_len += snprintf(temp_buf + temp_len, sizeof(temp_buf) - temp_len,
                                    "%s", gConfigParam.nfcauth_config.nfcauth_cards[i].nfc_no);
                }
                else
                {
                    temp_len += snprintf(temp_buf + temp_len, sizeof(temp_buf) - temp_len,
                                    "%s;", gConfigParam.nfcauth_config.nfcauth_cards[i].nfc_no);
                }
            }

            msg->resp_length = snprintf(msg->resp_msg, remaining, "%s", temp_buf);
            LOG_INF("%s=>CHECK,COUNT=%d", __func__, gConfigParam.nfcauth_config.nfcauth_card_count);
            return BLE_DATA_TYPE_AT_CMD;
        }
        /* 查询指定卡号的详细权限信息 */
        else if (msg->parm_count == 2)
        {
            found = 0;
            for (i = 0; i < gConfigParam.nfcauth_config.nfcauth_card_count; i++)
            {
                if (strcmp(gConfigParam.nfcauth_config.nfcauth_cards[i].nfc_no, msg->parm[2]) == 0)
                {
                    found = 1;
                    msg->resp_length = snprintf(msg->resp_msg, remaining,
                        "RETURN_%s,%d,%d,%u,%s,%s,%u",
                        gConfigParam.nfcauth_config.nfcauth_cards[i].nfc_no,
                        gConfigParam.nfcauth_config.nfcauth_cards[i].lat,
                        gConfigParam.nfcauth_config.nfcauth_cards[i].lon,
                        gConfigParam.nfcauth_config.nfcauth_cards[i].radius,
                        gConfigParam.nfcauth_config.nfcauth_cards[i].start_time,
                        gConfigParam.nfcauth_config.nfcauth_cards[i].end_time,
                        gConfigParam.nfcauth_config.nfcauth_cards[i].unlock_times);
                    LOG_INF("%s=>CHECK,%s", __func__, msg->parm[2]);
                    return BLE_DATA_TYPE_AT_CMD;
                }
            }

            if (!found)
            {
                LOG_INF("%s=>CHECK,%s not found", __func__, msg->parm[2]);
                goto param_invalid;
            }
        }
        else
        {
            LOG_INF("%s=>CHECK param count error: %d", __func__, msg->parm_count);
            goto param_invalid;
        }
    }
    /* 无效的操作类型 */
    else
    {
        LOG_INF("%s=>invalid operation: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }

/* 参数错误统一处理 */
param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
    return BLE_DATA_TYPE_AT_CMD;
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
static int btlog_cmd_handler(at_cmd_struc* msg)
{
    uint16_t remaining;
    BleLogConfig_t *config;

    remaining = sizeof(msg->resp_msg);
    config = my_param_get_ble_log_config();

    /* 无参数 - 查询状态 */
    if (msg->parm_count == 0)
    {
        msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_BTLOG_%s",
                                    config->global_en ? "ON" : "OFF");
        return BLE_DATA_TYPE_AT_CMD;
    }

    /* 有参数 - 设置状态 */
    if (msg->parm_count == 1)
    {
        if (strcmp(msg->parm[1], "ON") == 0)
        {
            config->global_en = 1;
            if (my_param_set_ble_log_config(config) == 0)
            {
                msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_BTLOG_ON_OK");
                LOG_INF("BTLOG enabled");
            }
            else
            {
                msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_BTLOG_ON_FAIL");
            }
            return BLE_DATA_TYPE_AT_CMD;
        }
        else if (strcmp(msg->parm[1], "OFF") == 0)
        {
            config->global_en = 0;
            if (my_param_set_ble_log_config(config) == 0)
            {
                msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_BTLOG_OFF_OK");
                LOG_INF("BTLOG disabled");
            }
            else
            {
                msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_BTLOG_OFF_FAIL");
            }
            return BLE_DATA_TYPE_AT_CMD;
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
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_BTLOG_FAIL");
    return BLE_DATA_TYPE_AT_CMD;
}

/*********************************************************************
**函数名称:  bkey_cmd_handler
**入口参数:  msg      ---        AT指令结构体指针
**出口参数:  msg->resp_msg  ---  响应消息
**           msg->resp_length --- 响应长度
**函数功能:  处理BKEY_SET/BKEY_RESET指令：设置/重置蓝牙解锁密钥
**指令格式:  BKEY_SET,[Old key],[New key]#  - 修改密钥
**           BKEY_RESET#                    - 重置为默认密钥
**参数说明:  [Old key] - 当前密钥，6位数字（默认000000）
**           [New key] - 新密钥，6位数字，不可与当前密钥相同
**返 回 值:  BLE数据类型
*********************************************************************/
static int bkey_cmd_handler(at_cmd_struc* msg)
{
    uint16_t remaining;
    uint8_t digit_len;
    const char *default_key = "000000";

    remaining = sizeof(msg->resp_msg);

    /* 判断指令类型 */
    if (strcmp(msg->parm[0], "BKEY_RESET") == 0)
    {
        /* BKEY_RESET# - 重置为默认密钥 */
        if (msg->parm_count != 0)
        {
            LOG_INF("%s=>BKEY_RESET param count error: %d", __func__, msg->parm_count);
            msg->resp_length = snprintf(msg->resp_msg, remaining, "Key reset failed. Invalid param.");
            return BLE_DATA_TYPE_AT_CMD;
        }

        strcpy(gConfigParam.bkey_config.bt_key, default_key);
        LOG_INF("%s=>RESET to default key", __func__);
        msg->resp_length = snprintf(msg->resp_msg, remaining, "Key reset success.");
        return BLE_DATA_TYPE_AT_CMD;
    }
    else if (strcmp(msg->parm[0], "BKEY_SET") == 0)
    {
        /* BKEY_SET,[Old key],[New key]# - 修改密钥 */
        if (msg->parm_count != 2)
        {
            LOG_INF("%s=>param count error: %d", __func__, msg->parm_count);
            msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_BKEY_SET_FAIL");
            return BLE_DATA_TYPE_AT_CMD;
        }

        /* 验证旧密钥长度是否为6位数字 */
        if (strlen(msg->parm[1]) != 6)
        {
            LOG_INF("%s=>old key length error: %s", __func__, msg->parm[1]);
            goto invalid_key;
        }

        digit_len = string_check_is_number(0, msg->parm[1]);
        if (digit_len != 6)
        {
            LOG_INF("%s=>old key format error: %s", __func__, msg->parm[1]);
            goto invalid_key;
        }

        /* 验证新密钥长度是否为6位数字 */
        if (strlen(msg->parm[2]) != 6)
        {
            LOG_INF("%s=>new key length error: %s", __func__, msg->parm[2]);
            goto invalid_key;
        }
        digit_len = string_check_is_number(0, msg->parm[2]);
        if (digit_len != 6)
        {
            LOG_INF("%s=>new key format error: %s", __func__, msg->parm[2]);
            goto invalid_key;
        }

        /* 验证旧密钥是否正确 */
        if (strcmp(gConfigParam.bkey_config.bt_key, msg->parm[1]) != 0)
        {
            LOG_INF("%s=>old key mismatch", __func__);
            goto invalid_key;
        }

        /* 验证新密钥是否与旧密钥相同 */
        if (strcmp(msg->parm[1], msg->parm[2]) == 0)
        {
            LOG_INF("%s=>new key same as old key", __func__);
            msg->resp_length = snprintf(msg->resp_msg, remaining, "Key update failed. New key must be different.");
            return BLE_DATA_TYPE_AT_CMD;
        }

        /* 更新密钥 */
        strcpy(gConfigParam.bkey_config.bt_key, msg->parm[2]);
        LOG_INF("%s=>key updated:%s", __func__, gConfigParam.bkey_config.bt_key);
        msg->resp_length = snprintf(msg->resp_msg, remaining, "Key update success.");
        return BLE_DATA_TYPE_AT_CMD;
    }

    /* 无效指令 */
    LOG_INF("%s=>invalid cmd: %s", __func__, msg->parm[0]);

    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_BKEY_FAIL");
    return BLE_DATA_TYPE_AT_CMD;

invalid_key:
    /* 蜂鸣器未授权提示音 (异常提示音)*/
    my_set_buzzer_mode(BUZZER_ERROR_TONE);
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Key update failed. Invalid key.");
    return BLE_DATA_TYPE_AT_CMD;
}

/*********************************************************************
**函数名称:  bunlock_cmd_handler
**入口参数:  msg      ---        AT指令结构体指针
**出口参数:  msg->resp_msg  ---  响应消息
**           msg->resp_length --- 响应长度
**函数功能:  处理BUNLOCK指令：蓝牙解锁
**指令格式:  BUNLOCK,[key]#
**参数说明:  [key] - 解锁固定密钥，由6位数字组成
**返 回 值:  BLE数据类型
*********************************************************************/
static int bunlock_cmd_handler(at_cmd_struc* msg)
{
    uint16_t remaining;
    bool already_unlocked;
    uint8_t digit_len;

    remaining = sizeof(msg->resp_msg);

    /* 检查参数数量 */
    if (msg->parm_count != 1)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        msg->resp_length = snprintf(msg->resp_msg, remaining, "Unlock failed. Invalid param.");
        return BLE_DATA_TYPE_AT_CMD;
    }

    /* 验证密钥长度是否为6位数字 */
    if (strlen(msg->parm[1]) != 6)
    {
        LOG_INF("%s=>key length error: %s", __func__, msg->parm[1]);
        goto key_error;
    }

    digit_len = string_check_is_number(0, msg->parm[1]);
    if (digit_len != 6)
    {
        LOG_INF("%s=>old key format error: %s", __func__, msg->parm[1]);
        goto key_error;
    }

    /* 验证密钥是否正确 */
    if (strcmp(gConfigParam.bkey_config.bt_key, msg->parm[1]) != 0)
    {
        LOG_INF("%s=>key mismatch", __func__);
        goto key_error;
    }

    /* 检查当前锁状态：通过开锁限位判断是否已解锁 */
    already_unlocked = get_openlock_state();
    if (already_unlocked)
    {
        LOG_INF("%s=>already in unlock state", __func__);
        msg->resp_length = snprintf(msg->resp_msg, remaining, "Unlock failed. Device already in unlock state.");
        return BLE_DATA_TYPE_AT_CMD;
    }

    /* 执行开锁动作，开锁模块会在完成或超时后通过ble_comu_response_or_expansion_cmd回复结果 */
    LOG_INF("%s=>executing unlock action", __func__);

    /* 标记当前是处于蓝牙解锁 */
    g_bBleLockState = UNLOCKING;
    my_send_msg(MOD_MAIN, MOD_CTRL, MY_MSG_CTRL_OPENLOCKING);

    /* 不返回响应消息，由ble模块异步回复 */
    msg->resp_length = 0;
    return BLE_DATA_TYPE_AT_CMD;

key_error:
    // Buzzer 未授权提示
    my_set_buzzer_mode(BUZZER_ERROR_TONE);
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Unlock failed. Unlock key error");
    return BLE_DATA_TYPE_AT_CMD;
}

/*********************************************************************
**函数名称:  block_cmd_handler
**入口参数:  msg      ---        AT指令结构体指针
**出口参数:  msg->resp_msg  ---  响应消息
**           msg->resp_length --- 响应长度
**函数功能:  处理BLOCK指令：蓝牙上锁
**指令格式:  BLOCK,[key]#
**参数说明:  [key] - 上锁固定密钥，由6位数字组成
**返 回 值:  BLE数据类型
*********************************************************************/
static int block_cmd_handler(at_cmd_struc* msg)
{
    uint16_t remaining;
    bool already_locked;
    bool pin_inserted;
    uint8_t digit_len;

    remaining = sizeof(msg->resp_msg);

    /* 检查参数数量 */
    if (msg->parm_count != 1)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        msg->resp_length = snprintf(msg->resp_msg, remaining, "Lock failed. Invalid param.");
        return BLE_DATA_TYPE_AT_CMD;
    }

    /* 验证密钥长度是否为6位数字 */
    if (strlen(msg->parm[1]) != 6)
    {
        LOG_INF("%s=>key length error: %s", __func__, msg->parm[1]);
        goto key_error;
    }

    digit_len = string_check_is_number(0, msg->parm[1]);
    if (digit_len != 6)
    {
        LOG_INF("%s=>key format error: %s", __func__, msg->parm[1]);
        goto key_error;
    }

    /* 验证密钥是否正确 */
    if (strcmp(gConfigParam.bkey_config.bt_key, msg->parm[1]) != 0)
    {
        LOG_INF("%s=>key mismatch", __func__);
        goto key_error;
    }

    /* 检查锁销是否插入 */
    pin_inserted = get_lockpin_insert_state();
    if (!pin_inserted)
    {
        LOG_INF("%s=>lock pin not detected", __func__);
        // Buzzer 上锁失败提示
        my_set_buzzer_mode(BUZZER_EVENT_LOCK_FAIL);
        msg->resp_length = snprintf(msg->resp_msg, remaining, "Lock failed. Lock pin not detected.");
        return BLE_DATA_TYPE_AT_CMD;
    }

    /* 检查当前锁状态：通过关锁限位判断是否已上锁 */
    already_locked = get_closelock_state();
    if (already_locked)
    {
        LOG_INF("%s=>already in lock state", __func__);
        //Buzzer 上锁失败提示
        my_set_buzzer_mode(BUZZER_EVENT_LOCK_FAIL);
        msg->resp_length = snprintf(msg->resp_msg, remaining, "Lock failed. Device already in lock state.");
        return BLE_DATA_TYPE_AT_CMD;
    }

    /* 执行上锁动作，上锁模块会在完成或超时后通过ble_comu_response_or_expansion_cmd回复结果 */
    LOG_INF("%s=>executing lock action", __func__);

    /* 标记当前是处于蓝牙上锁 */
    g_bBleLockState = LOCKING;
    my_send_msg(MOD_MAIN, MOD_CTRL, MY_MSG_CTRL_CLOSELOCKING);

    /* 不返回响应消息，由ble模块异步回复 */
    msg->resp_length = 0;
    return BLE_DATA_TYPE_AT_CMD;

key_error:
    //Buzzer 未授权提示
    my_set_buzzer_mode(BUZZER_ERROR_TONE);
    msg->resp_length = snprintf(msg->resp_msg, remaining, "Lock failed. key error");
    return BLE_DATA_TYPE_AT_CMD;
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
static int version_cmd_handler(at_cmd_struc* msg)
{
    uint16_t remaining;  // 响应消息缓冲区的剩余空间
    int ret;             // snprintf 函数的返回值

    remaining = sizeof(msg->resp_msg);  // 计算响应消息缓冲区的大小

    /* 检查参数数量：应为0 */
    if (msg->parm_count == 0)  // 检查命令是否有参数
    {
        LOG_INF("%s=>%s", __func__, msg->parm[0]);  // 输出函数名和命令名

        /* 生成响应消息，格式：[VERSION]%s*/
        ret = snprintf(msg->resp_msg, remaining, "[VERSION]%s", SOFTWARE_VERSION);  // 生成包含版本号的响应消息

        if (ret > 0 && ret < remaining)  // 检查响应消息是否生成成功
        {
            msg->resp_length = ret;  // 设置响应消息的长度
            LOG_INF("VERSION: %s", msg->resp_msg);  // 输出版本号信息
        }
        else  // 响应消息生成失败
        {
            // 生成失败响应消息
            msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
        }
    }
    else  // 参数数量错误
    {
        // 输出参数数量错误信息
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        // 生成失败响应消息
        msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
    }
    return BLE_DATA_TYPE_AT_CMD;  // 返回 BLE 数据类型
}

/********************************************************************
**函数名称:  modeset_cmd_handler
**入口参数:  msg   ---   AT 命令消息结构体
**出口参数:  msg   ---   填充响应消息
**函数功能:  处理MODESET指令：设置设备工作模式
**指令格式:  MODESET,[Work Mode]#
**参数说明:  [Work Mode] - 工作模式
**返 回 值:  BLE数据类型
**使用示例:  AT+MODESET=1,10,0800     // 设置长续航模式，上报间隔10分钟，启动时间08:00
**           AT+MODESET=2,3600,60,500,14400,2  // 设置智能模式，各状态间隔和睡眠开关
**           AT+MODESET=3             // 切换到连续模式
*********************************************************************/
static int modeset_cmd_handler(at_cmd_struc* msg)
{
    uint16_t remaining;  // 响应消息缓冲区的剩余空间
    DeviceWorkModeConfig param_work_mode_config;  // 工作模式配置结构体

    remaining = sizeof(msg->resp_msg);  // 计算响应消息缓冲区的大小

    if(msg->parm_count == 0)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        goto param_invalid;
    }
    // 解析当前模式参数（parm[1]）
    param_work_mode_config.current_mode = atoi(msg->parm[1]);
    if (param_work_mode_config.current_mode < MY_MODE_CONTINUOUS || param_work_mode_config.current_mode > MY_MODE_SMART)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[1]);
        goto param_invalid;
    }

    // 只有模式参数的情况（切换模式）
    if (msg->parm_count == 1)
    {
        // 切换到指定工作模式
        switch_work_mode(param_work_mode_config.current_mode);

        /* 生成成功响应 */
        msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
        LOG_INF("MODESET: current_mode:%d", param_work_mode_config.current_mode);

        return BLE_DATA_TYPE_AT_CMD;
    }

    // 连续模式处理
    if (param_work_mode_config.current_mode == MY_MODE_CONTINUOUS)
    {
        /* 检查参数数量 */
        if (msg->parm_count != 3)
        {
            LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
            goto param_invalid;
        }
        LOG_INF("%s,%s,%s,%s#", msg->parm[0], msg->parm[1], msg->parm[2], msg->parm[3]);
    }
    // 长续航模式处理
    else if (param_work_mode_config.current_mode == MY_MODE_LONG_LIFE)
    {
        /* 检查参数数量 */
        if (msg->parm_count != 3)
        {
            LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
            goto param_invalid;
        }

        // 解析长续航模式参数
        param_work_mode_config.long_battery.reporting_interval_min = atoi(msg->parm[2]);
        // 设置长续航模式参数
        if (set_long_battery_params(&gConfigParam.device_workmode_config.workmode_config,
            param_work_mode_config.long_battery.reporting_interval_min, msg->parm[3]) < 0)
        {
            LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
            goto param_invalid;
        }

        LOG_INF("%s,%s,%s,%s#", msg->parm[0], msg->parm[1], msg->parm[2], msg->parm[3]);
    }
    // 智能模式处理
    else if (param_work_mode_config.current_mode == MY_MODE_SMART)
    {
        /* 检查参数数量 */
        if (msg->parm_count != 6)
        {
            LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
            goto param_invalid;
        }

        // 解析智能模式参数
        param_work_mode_config.intelligent.stop_status_interval_sec = atoi(msg->parm[2]);      // 静止状态间隔（秒）
        param_work_mode_config.intelligent.land_status_interval_sec = atoi(msg->parm[3]);      // 陆运状态间隔（秒）
        param_work_mode_config.intelligent.land_status_interval_dis = atoi(msg->parm[4]);      // 陆运状态距离（米）
        param_work_mode_config.intelligent.sea_status_interval_sec = atoi(msg->parm[5]);      // 海运状态间隔（秒）
        param_work_mode_config.intelligent.sleep_switch = atoi(msg->parm[6]);                 // 睡眠开关（0-2）

        // 设置智能模式参数
        if (set_intelligent_params(&gConfigParam.device_workmode_config.workmode_config,
            param_work_mode_config.intelligent.stop_status_interval_sec,
            param_work_mode_config.intelligent.land_status_interval_sec,
            param_work_mode_config.intelligent.land_status_interval_dis,
            param_work_mode_config.intelligent.sea_status_interval_sec,
            param_work_mode_config.intelligent.sleep_switch) < 0)
        {
            LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
            goto param_invalid;
        }

        LOG_INF("%s,%s,%s,%s,%s,%s,%s#", msg->parm[0], msg->parm[1], msg->parm[2], msg->parm[3], msg->parm[4], msg->parm[5], msg->parm[6]);
    }

    // 命令透传，发送个LTE模块
    lte_cmd_handler(msg);

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
    LOG_INF("MODESET: current_mode:%d", param_work_mode_config.current_mode);

    return BLE_DATA_TYPE_AT_CMD;

param_invalid:
    // 生成失败响应
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
    return BLE_DATA_TYPE_AT_CMD;
}

/********************************************************************
**函数名称:  cunlock_cmd_handler
**入口参数:  msg   ---   AT 命令消息结构体
**出口参数:  msg   ---   填充响应消息
**函数功能:  处理 UNLOCK 指令：执行带时间窗口的远程开锁操作
**           1. 权限检查：仅允许 LTE 网络来源，拒绝蓝牙指令
**           2. 参数校验：检查参数数量(2个)、时间格式(12位)、延迟时间范围(<=3600s)
**           3. 指令更新：重复发直接更新指令
**           4. 时间窗口逻辑：
**                 delay_time > 0:
**              - start_time + delay_time < current_ts,时间过期：返回失败
**              - start_time < current_ts < start_time + delay_time：立即执行开锁
**              - start_time > current_ts：启动启动定时器等待
**                 delay_time = 0:
**                 start_time < current_ts:立即执行开锁
**                 start_time > current_ts：启动启动定时器等待
**指令格式:  UNLOCK,[YYMMDDHHMMSS],[delay_time]
**返回值说明: 0: 来源错误（非 LTE 来源）
**             1: 直接回复
**             2: 需异步回复
**返 回 值:  int
*********************************************************************/
static int cunlock_cmd_handler(at_cmd_struc* msg)
{
    uint16_t remaining;
    int delay_time = 0;
    time_t start_ts = 0;
    time_t current_ts = 0;
    char buf[12] = {0};
    uint8_t sec = 0;

    remaining = sizeof(msg->resp_msg);

    // 此指令不允许蓝牙发
    if (!g_lte_cmdSource)
    {
        return 0;
    }

    // 参数数量基础校验
    if (msg->parm_count != 2)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        goto param_invalid;
    }

    // YYMMDDHHMMSS  12个字符
    if (strlen(msg->parm[1]) != 12)
    {
        MY_LOG_INF("Time setting error");
        msg->resp_length = snprintf(msg->resp_msg, remaining, "Time setting error");
        return 1;
    }

    //获取delay数据
    delay_time = atoi(msg->parm[2]);

    // 检查delay_time
    if (delay_time > 3600 || delay_time < 0)
    {
        MY_LOG_INF("Timeout exceeds 3600s or delay_time < 0");
        msg->resp_length = snprintf(msg->resp_msg, remaining, "Timeout exceeds 3600s or delay_time < 0");
        return 1;
    }

    // 指令更新
    if (k_timer_remaining_get(&g_net_unlock.start_timer) != 0)
    {
        k_timer_stop(&g_net_unlock.start_timer);
    }
    if (k_timer_remaining_get(&g_net_unlock.delay_timer) != 0)
    {
        k_timer_stop(&g_net_unlock.delay_timer);
    }
    g_net_unlock.netunlock_flag = 0;
    g_net_unlock.start_timer_flag = 0;


    g_net_unlock.delay_sec = delay_time;

    // 取前10个字符
    strncpy(buf, msg->parm[1], 10);
    buf[10] = '\0';

    sec = atoi(msg->parm[1] + 10);
    start_ts = time_str_to_timestamp(buf) + sec;
    current_ts = my_get_system_time_sec();
    MY_LOG_INF("current_ts = %d", current_ts);

    if (delay_time > 0)
    {
        // 判断起始时间+窗口期时间是否过期，过期返回失败
        if (start_ts + delay_time <= current_ts)
        {
            // 不在窗口期内
            MY_LOG_INF("Command received, but not within the valid time window.1");
            msg->resp_length = snprintf(msg->resp_msg, remaining, "Command received, but not within the valid time window.");
            return 1;
        }
        else
        {
            // 当前是否处于当前时间范围内
            if (start_ts > current_ts)
            {
                // 启动定时器等到达start_ts时间后执行开锁命令
                k_timer_start(&g_net_unlock.start_timer, K_MSEC((start_ts - current_ts) * 1000), K_NO_WAIT);
                MY_LOG_INF("start_ts - current_ts = %d", (start_ts - current_ts));
                // 不在窗口期内
                MY_LOG_INF("Command received, but not within the valid time window.2");
                msg->resp_length = snprintf(msg->resp_msg, remaining, "Command received, but not within the valid time window.");
                return 1;
            }
            else
            {
                /*说明时间在当前时间之后且+delay没过期，直接执行开锁命令
                检查当前锁状态：通过开锁限位判断是否已解锁*/
                if (get_openlock_state())
                {
                    MY_LOG_INF("Unlock failed. Device already in unlock state.");
                    msg->resp_length = snprintf(msg->resp_msg, remaining, "Unlock failed. Device already in unlock state.");

                    //当时锁是打开的，窗口期内不允许自动关锁(计算剩余窗口期)
                    g_net_unlock.delay_sec = delay_time + start_ts - current_ts;
                    g_net_unlock.netunlock_flag = 1;
                    k_timer_start(&g_net_unlock.delay_timer, K_MSEC(g_net_unlock.delay_sec * 1000), K_NO_WAIT);

                    return 1;
                }

                // 标记当前是处于网络解锁
                g_netLockState = UNLOCKING;
                my_send_msg(MOD_MAIN, MOD_CTRL, MY_MSG_CTRL_OPENLOCKING);
                MY_LOG_INF("start to openlock");

                // 需要异步回复
                return 2;
            }
        }
    }
    else // 没有窗口期
    {
        // 判断是否到达开锁时间
        if (start_ts <= current_ts)
        {
            /*到达开锁时间，执行开锁命令
            检查当前锁状态：通过开锁限位判断是否已解锁 */
            if (get_openlock_state())
            {
                MY_LOG_INF("Unlock failed. Device already in unlock state.");
                msg->resp_length = snprintf(msg->resp_msg, remaining, "Unlock failed. Device already in unlock state.");
                return 1;
            }

            g_net_unlock.netunlock_flag = 1;

            // 标记当前是处于网络解锁
            g_netLockState = UNLOCKING;
            my_send_msg(MOD_MAIN, MOD_CTRL, MY_MSG_CTRL_OPENLOCKING);
            MY_LOG_INF("start to openlock");

            // 需要异步回复
            return 2;
        }
        else
        {
            // 未到达开锁时间，启动定时器等到达开锁时间后执行开锁命令
            k_timer_start(&g_net_unlock.start_timer, K_MSEC((start_ts - current_ts) * 1000), K_NO_WAIT);

            // 不在窗口期内
            MY_LOG_INF("Command received, but not within the valid time window.");
            msg->resp_length = snprintf(msg->resp_msg, remaining, "Command received, but not within the valid time window.");
            return 1;
        }
    }

param_invalid:
    // 生成失败响应
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
    return 1;
}

/********************************************************************
**函数名称:  clock_cmd_handler
**入口参数:  msg   ---   AT 命令消息结构体
**出口参数:  msg   ---   填充响应消息
**函数功能:  处理 CLOCK 指令：执行远程上锁操作
**           1. 权限检查：仅允许 LTE 网络来源，拒绝蓝牙指令
**           2. 参数检查：校验参数数量必须为 0
**           3. 硬件检查：检测锁销是否插入、检测当前是否已处于上锁状态
**           4. 执行逻辑：若检查通过，更新全局锁状态为 LOCKING 并发送控制消息
**指令格式:  CLOCK#
**返回值说明: 0: 来源错误（非 LTE 来源）
**             1: 直接回复
**             2: 需异步回复
**返 回 值:  int
*********************************************************************/
static int clock_cmd_handler(at_cmd_struc *msg)
{
    uint16_t remaining;
    bool pin_inserted;

    remaining = sizeof(msg->resp_msg);

    // 此指令不允许蓝牙发
    if (!g_lte_cmdSource)
    {
        return 0;
    }

    // 检查参数数量
    if (msg->parm_count != 0)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        msg->resp_length = snprintf(msg->resp_msg, remaining, "Lock failed. Invalid param.");
        return 1;
    }

    // 检查锁销是否插入
    pin_inserted = get_lockpin_insert_state();
    // 锁销未插入
    if (!pin_inserted)
    {
        // Buzzer 上锁失败提示
        my_set_buzzer_mode(BUZZER_EVENT_LOCK_FAIL);
        MY_LOG_INF("Lock failed. Lock pin not detected.");
        msg->resp_length = snprintf(msg->resp_msg, remaining, "Lock failed. Lock pin not detected.");
        return 1;
    }

    // 检查当前锁状态：通过关锁限位判断是否已上锁
    if (get_closelock_state())
    {
        // Buzzer 上锁失败提示
        my_set_buzzer_mode(BUZZER_EVENT_LOCK_FAIL);
        MY_LOG_INF("Lock failed. Device already in lock state.");
        msg->resp_length = snprintf(msg->resp_msg, remaining, "Lock failed. Device already in lock state.");
        return 1;
    }

    // 标记当前是处于网络上锁
    g_netLockState = LOCKING;
    my_send_msg(MOD_MAIN, MOD_CTRL, MY_MSG_CTRL_CLOSELOCKING);

    // 需要异步回复
    return 2;
}
