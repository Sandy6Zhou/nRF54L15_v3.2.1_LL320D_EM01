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

// 标记lte_cmd来的,用于区分蓝牙下发的还是lte过来的(某些指令只能网络发蓝牙不能执行)
uint8_t g_lte_cmdSource = 0;

// 用于存储整包返回的数据内容(仅在蓝牙线程使用)
char g_resp_buf[RESP_STRING_LENGTH_MAX];

static int remalm_cmd_handler(at_cmd_t* msg);
static int motdet_cmd_handler(at_cmd_t* msg);
static int batlevel_cmd_handler(at_cmd_t* msg);
static int chargesta_cmd_handler(at_cmd_t* msg);
static int shockalarm_cmd_handler(at_cmd_t* msg);
static int pwsave_cmd_handler(at_cmd_t* msg);
static int startr_cmd_handler(at_cmd_t* msg);
static int cbmt_cmd_handler(at_cmd_t* msg);
static int bt_crfpwr_cmd_handler(at_cmd_t* msg);
static int bt_updata_cmd_handler(at_cmd_t* msg);
static int tag_cmd_handler(at_cmd_t* msg);
static int jatag_cmd_handler(at_cmd_t* msg);
static int jgtag_cmd_handler(at_cmd_t* msg);
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

static const at_cmd_attr_t at_cmd_attr_table[] =
{
    {"REMALM",         remalm_cmd_handler},
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
    "SECOND_SERVER"
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
**           static_int  --  静止状态上报间隔（原始值，单位由子模式决定）
**           moving_int  --  运动状态上报间隔（原始值，单位由子模式决定）
**出口参数:  无
**函数功能:  设置智能模式参数，根据子模式校验间隔范围
**返 回 值:  0 表示成功，-1 表示失败（参数非法）
**注意事项:  子模式0~4静止间隔单位为分钟(0/3~60)，子模式5为秒(0/10~86400)
**           子模式0~1运动间隔单位为分钟(3~60)，子模式2~5为秒(10~86400)
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
            // 子模式0~4：静止间隔单位为分钟，范围3~60
            if (static_int < 3 || static_int > 60)
            {
                LOG_INF("Error: intelligent static_int %u out of range (0/3~60 min) for sub_mode %u", static_int, sub_mode);
                return -1;
            }
        }
        else
        {
            // 子模式5：静止间隔单位为秒，范围10~86400
            if (static_int < 10 || static_int > 86400)
            {
                LOG_INF("Error: intelligent static_int %u out of range (0/10~86400 sec) for sub_mode 5", static_int);
                return -1;
            }
        }
    }

    // 校验运动间隔
    if (sub_mode <= 1)
    {
        // 子模式0~1：运动间隔单位为分钟，范围3~60
        if (moving_int < 3 || moving_int > 60)
        {
            LOG_INF("Error: intelligent moving_int %u out of range (3~60 min) for sub_mode %u", moving_int, sub_mode);
            return -1;
        }
    }
    else
    {
        // 子模式2~5：运动间隔单位为秒，范围10~86400
        if (moving_int < 10 || moving_int > 86400)
        {
            LOG_INF("Error: intelligent moving_int %u out of range (10~86400 sec) for sub_mode %u", moving_int, sub_mode);
            return -1;
        }
    }

    p_workmode->intelligent.sub_mode = sub_mode;
    p_workmode->intelligent.static_interval = static_int;
    p_workmode->intelligent.moving_interval = moving_int;
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

    // 未匹配指令，返回错误回复
    at_cmd_msg->resp_length = snprintf(at_cmd_msg->resp_msg, RESP_STRING_LENGTH_MAX, "CMD Error");

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
uint16_t run_lte_cmd(at_cmd_t *at_cmd_msg)
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
        msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
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

    no_count = string_check_is_number(0, msg->parm[2]);
    if (no_count == 0)
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
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
    LOG_INF("REMALM: SW=%d, M=%d", gConfigParam.remalm_config.remalm_sw, gConfigParam.remalm_config.remalm_mode);

    //TODO 具体逻辑处理

    return BLE_DATA_TYPE_PACKET_MULTIPLE;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
    return BLE_DATA_TYPE_PACKET_MULTIPLE;
}

/********************************************************************
**函数名称:  motdet_cmd_handler
**入口参数:  msg      ---        AT指令结构体指针
**出口参数:  msg->resp_msg  ---  响应消息
**           msg->resp_length --- 响应长度
**函数功能:  处理MOTDET指令：设置运动检测参数
**指令格式:  MOTDET,[Transition Count],[Detection Interval],[Report Type]#
**参数说明:  [Transition Count] - 运动切换次数: 1-10 (默认5)
**           [Detection Interval] - 检测间隔: 5-3600 s (默认300)
**           [Report Type] - 模式切换上报方式: 0-不上报，1-GPRS, 2-GPRS+SMS, 3-GPRS+SMS+CALL
**返 回 值:  BLE数据类型
*********************************************************************/
static int motdet_cmd_handler(at_cmd_t* msg)
{
    uint16_t remaining;
    uint8_t no_count = 0;
    int transition_count_value;
    int detection_interval_value;
    int report_type_value;

    remaining = RESP_STRING_LENGTH_MAX;

    //无参数即查询
    if (msg->parm_count == 0)
    {
        msg->resp_length = snprintf(msg->resp_msg, remaining, "%s:%d,%d,%d",
                            msg->parm[0],
                            gConfigParam.motdet_config.motdet_transition_count,
                            gConfigParam.motdet_config.motdet_detection_interval,
                            gConfigParam.motdet_config.motdet_report_type);
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
    }

    /* 检查参数数量 */
    if (msg->parm_count != 3)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
    }

    no_count = string_check_is_number(0, msg->parm[1]);
    if (no_count == 0)
    {
        LOG_INF("%s=>invalid Transition Count param: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }
    /* 解析Transition Count参数 */
    transition_count_value = atoi(msg->parm[1]);
    if (transition_count_value < 1 || transition_count_value > 10)
    {
        LOG_INF("%s=>invalid Transition Count param: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }

    no_count = string_check_is_number(0, msg->parm[2]);
    if (no_count == 0)
    {
        LOG_INF("%s=>invalid Detection Interval param: %s", __func__, msg->parm[2]);
        goto param_invalid;
    }
    /* 解析Detection Interval参数 */
    detection_interval_value = atoi(msg->parm[2]);
    if (detection_interval_value < 5 || detection_interval_value > 3600)
    {
        LOG_INF("%s=>invalid Detection Interval param: %s", __func__, msg->parm[2]);
        goto param_invalid;
    }

    no_count = string_check_is_number(0, msg->parm[3]);
    if (no_count == 0)
    {
        LOG_INF("%s=>invalid Report Type param: %s", __func__, msg->parm[3]);
        goto param_invalid;
    }
    /* 解析Report Type参数 */
    report_type_value = atoi(msg->parm[3]);
    if (report_type_value < 0 || report_type_value > 3)
    {
        LOG_INF("%s=>invalid Report Type param: %s", __func__, msg->parm[3]);
        goto param_invalid;
    }

    /* 所有参数验证通过,统一赋值 */
    gConfigParam.motdet_config.flag = FLAG_VALID;
    gConfigParam.motdet_config.motdet_transition_count = (uint16_t)transition_count_value;
    gConfigParam.motdet_config.motdet_detection_interval = (uint16_t)detection_interval_value;
    gConfigParam.motdet_config.motdet_report_type = (uint8_t)report_type_value;

    /* 保存配置 */
    my_user_data_write(ZMS_ID_MOT_DET_CONFIG, &gConfigParam.motdet_config, sizeof(mot_det_config_t));

    LOG_INF("%s=>%s,%s,%s,%s,%s,%s", __func__, msg->parm[0], msg->parm[1],
           msg->parm[2], msg->parm[3], msg->parm[4], msg->parm[5]);

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
    LOG_INF("MOTDET: TransitionCount=%d, DetectionInterval=%d, ReportType=%d",
           gConfigParam.motdet_config.motdet_transition_count,
           gConfigParam.motdet_config.motdet_detection_interval,
           gConfigParam.motdet_config.motdet_report_type);

    //TODO 具体逻辑处理

    return BLE_DATA_TYPE_PACKET_MULTIPLE;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
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
        if (no_count == 0)
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
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
    LOG_INF("BATLEVEL: Empty RPT=%d, Low RPT=%d, Normal RPT=%d, Fair RPT=%d, High RPT=%d, Full RPT=%d",
           gConfigParam.batlevel_config.batlevel_empty_rpt, gConfigParam.batlevel_config.batlevel_low_rpt,
           gConfigParam.batlevel_config.batlevel_normal_rpt, gConfigParam.batlevel_config.batlevel_fair_rpt,
           gConfigParam.batlevel_config.batlevel_high_rpt, gConfigParam.batlevel_config.batlevel_full_rpt);

    //TODO 具体逻辑处理

    return BLE_DATA_TYPE_PACKET_MULTIPLE;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
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
        msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
    }

    no_count = string_check_is_number(0, msg->parm[1]);
    if (no_count == 0)
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
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
    LOG_INF("CHARGESTA: Report=%d", gConfigParam.batlevel_config.chargesta_report);

    //TODO 具体逻辑处理

    return BLE_DATA_TYPE_PACKET_MULTIPLE;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
    return BLE_DATA_TYPE_PACKET_MULTIPLE;
}

/********************************************************************
**函数名称:  shockalarm_cmd_handler
**入口参数:  msg      ---        AT指令结构体指针
**出口参数:  msg->resp_msg  ---  响应消息
**           msg->resp_length --- 响应长度
**函数功能:  处理SHOCKALARM指令：设置撞击检测报警功能
**指令格式:  SHOCKALARM,[SW],[Level],[Type of Alarm],[Silence duration]#
**参数说明:  [SW] - 功能开关: ON/OFF (默认OFF)
**           [Level] - 撞击力度阈值: 1-5 (默认3; 5最敏感,1最不敏感)
**           [Type of Alarm] - 告警上报方式: 0-不上报, 1-GPRS, 2-GPRS+SMS, 3-GPRS+SMS+CALL
**           [Silence duration] - 静默时间: 10-600秒 (默认60秒)
**返 回 值:  BLE数据类型
*********************************************************************/
static int shockalarm_cmd_handler(at_cmd_t* msg)
{
    uint16_t remaining;
    uint8_t no_count = 0;
    int level_value;
    int type_value;
    int sw_value;
    int time_value;

    remaining = RESP_STRING_LENGTH_MAX;

    //无参数即查询
    if (msg->parm_count == 0)
    {
        // 根据 shockalarm_sw 的值选择 "ON" 或 "OFF"
        const char* state_str = gConfigParam.shockalarm_config.shockalarm_sw ? "ON" : "OFF";
        msg->resp_length = snprintf(msg->resp_msg, remaining, "%s:%s,%d,%d,%d",
                            msg->parm[0],
                            state_str,
                            gConfigParam.shockalarm_config.shockalarm_level,
                            gConfigParam.shockalarm_config.shockalarm_type,
                            gConfigParam.shockalarm_config.shockalarm_time
        );
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
    }

    /* 检查参数数量 */
    if (msg->parm_count != 4)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
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

    no_count = string_check_is_number(0, msg->parm[2]);
    if (no_count == 0)
    {
        LOG_INF("%s=>invalid Level param: %s", __func__, msg->parm[2]);
        goto param_invalid;
    }
    /* 解析Level参数 */
    level_value = atoi(msg->parm[2]);
    if (level_value < 1 || level_value > 5)
    {
        LOG_INF("%s=>invalid Level param: %s", __func__, msg->parm[2]);
        goto param_invalid;
    }

    no_count = string_check_is_number(0, msg->parm[3]);
    if (no_count == 0)
    {
        LOG_INF("%s=>invalid Type param: %s", __func__, msg->parm[3]);
        goto param_invalid;
    }
    /* 解析Type of Alarm参数 */
    type_value = atoi(msg->parm[3]);
    if (type_value < 0 || type_value > 3)
    {
        LOG_INF("%s=>invalid Type of Alarm param: %s", __func__, msg->parm[3]);
        goto param_invalid;
    }

    no_count = string_check_is_number(0, msg->parm[4]);
    if (no_count == 0)
    {
        LOG_INF("%s=>invalid Time param: %s", __func__, msg->parm[4]);
        goto param_invalid;
    }
    /* 解析Time参数 */
    time_value = atoi(msg->parm[4]);
    if (time_value < 10 || time_value > 600)
    {
        LOG_INF("%s=>invalid Time param: %s", __func__, msg->parm[4]);
        goto param_invalid;
    }

    /* 所有参数验证通过,统一赋值 */
    gConfigParam.shockalarm_config.flag = FLAG_VALID;
    gConfigParam.shockalarm_config.shockalarm_sw = (uint8_t)sw_value;
    gConfigParam.shockalarm_config.shockalarm_level = (uint8_t)level_value;
    gConfigParam.shockalarm_config.shockalarm_type = (uint8_t)type_value;
    gConfigParam.shockalarm_config.shockalarm_time = time_value;

    /* 保存配置 */
    my_user_data_write(ZMS_ID_SHOCK_ALARM_CONFIG, &gConfigParam.shockalarm_config, sizeof(shock_alarm_config_t));

    /* 通知 G-Sensor 更新配置 */
    my_send_msg(MOD_GSENSOR, MOD_GSENSOR, MY_MSG_SHOCK_SW);

    LOG_INF("%s=>%s,%s,%s,%s,%s", __func__, msg->parm[0], msg->parm[1], msg->parm[2], msg->parm[3], msg->parm[4]);

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
    LOG_INF("SHOCKALARM: SW=%d, Level=%d, Type=%d, Silence=%d ",
           gConfigParam.shockalarm_config.shockalarm_sw,
           gConfigParam.shockalarm_config.shockalarm_level,
           gConfigParam.shockalarm_config.shockalarm_type,
           gConfigParam.shockalarm_config.shockalarm_time
        );

    //TODO 具体逻辑处理

    return BLE_DATA_TYPE_PACKET_MULTIPLE;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
    return BLE_DATA_TYPE_PACKET_MULTIPLE;
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
static int pwsave_cmd_handler(at_cmd_t* msg)
{
    uint16_t remaining;

    remaining = RESP_STRING_LENGTH_MAX;

    // 无参数即查询
    if (msg->parm_count == 0)
    {
        // 根据 pwsave_sw 的值选择 "ON" 或 "OFF"
        const char* state_str = gConfigParam.pwsave_config.pwsave_sw ? "ON" : "OFF";
        msg->resp_length = snprintf(msg->resp_msg, remaining, "%s:%s", msg->parm[0], state_str);
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
    }

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

            // 启动关机定时器，让蓝牙接收到回复，2秒后触发关机
            my_start_timer(MY_TIMER_SHUTDOWN, 2000, false, shutdown_timeout_timer);

            // 通知4G模块关机
            #if RETRANSMIT_CHECK_ENABLED
                lte_send_cmd_with_retry("POWOFF", "1");
            #else
                lte_send_command("POWOFF", "1");
            #endif

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
        msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
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
    gConfigParam.startr_config.flag = FLAG_VALID;
    /* 保存配置 */
    my_user_data_write(ZMS_ID_STARTR_CONFIG, &gConfigParam.startr_config, sizeof(startr_config_t));

    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
    LOG_INF("STARTR: SW=%d", gConfigParam.startr_config.startr_sw);

    //TODO 具体逻辑处理

    return BLE_DATA_TYPE_PACKET_MULTIPLE;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
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
        msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
    }

    no_count = string_check_is_number(1, msg->parm[1]);
    if (no_count == 0)
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
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
    LOG_INF("BT_CRFPWR: A=%d",gConfigParam.ble_tx_power.tx_power);

    return BLE_DATA_TYPE_PACKET_MULTIPLE;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
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
    uint32_t scan_interval_value;
    uint32_t scan_length_value;
    uint32_t updata_interval_value;

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

    /* 检查参数数量 */
    if (msg->parm_count != 4)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
    }

    no_count = string_check_is_number(0, msg->parm[1]);
    if (no_count == 0)
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
    if (no_count == 0)
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
    if (no_count == 0)
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
    if (no_count == 0)
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
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
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
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
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
        msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
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
        no_count = string_check_is_number(0, msg->parm[2]);
        if (no_count == 0)
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
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
    LOG_INF("TAG: SW=%d, Interval=%u", gConfigParam.tag_config.tag_sw, gConfigParam.tag_config.tag_interval);

    //更新非连接广播参数，里面会按配置打开或关闭广播，根据tag_sw的值
    my_ble_updata_adv_param(gConfigParam.tag_config.tag_interval);

    return BLE_DATA_TYPE_PACKET_MULTIPLE;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
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
        msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
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
    gConfigParam.adv_valid_value.flag = FLAG_VALID;
    gConfigParam.adv_valid_value.AppleValid = (uint8_t)sw_value;
    set_adv_valid_status(APPLE_ADV_TYPE, gConfigParam.adv_valid_value.AppleValid);
    my_no_con_start_adv(gConfigParam.tag_config.tag_sw);

    /* 保存配置 */
    my_user_data_write(ZMS_ID_ADV_VALID, &gConfigParam.adv_valid_value, sizeof(adv_valid_value_t));

    LOG_INF("%s=>%s,%s", __func__, msg->parm[0], msg->parm[1]);

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
    LOG_INF("JATAG: SW=%d, Interval=%u", gConfigParam.adv_valid_value.AppleValid, gConfigParam.tag_config.tag_interval);

    return BLE_DATA_TYPE_PACKET_MULTIPLE;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
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
        msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
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
    gConfigParam.adv_valid_value.flag = FLAG_VALID;
    gConfigParam.adv_valid_value.GoogleValid = (uint8_t)sw_value;
    set_adv_valid_status(GOOGLE_ADV_TYPE, gConfigParam.adv_valid_value.GoogleValid);
    my_no_con_start_adv(gConfigParam.tag_config.tag_sw);

    /* 保存配置 */
    my_user_data_write(ZMS_ID_ADV_VALID, &gConfigParam.adv_valid_value, sizeof(adv_valid_value_t));

    LOG_INF("%s=>%s,%s", __func__, msg->parm[0], msg->parm[1]);

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
    LOG_INF("JGTAG: SW=%d, Interval=%u", gConfigParam.adv_valid_value.GoogleValid, gConfigParam.tag_config.tag_interval);

    return BLE_DATA_TYPE_PACKET_MULTIPLE;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
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
    int display_value;

    remaining = RESP_STRING_LENGTH_MAX;

    // 无参数即查询
    if (msg->parm_count == 0)
    {
        const char* state_str = gConfigParam.led_config.led_display ? "ON" : "OFF";
        msg->resp_length = snprintf(msg->resp_msg, remaining, "%s:%s",
                                    msg->parm[0],
                                    state_str
        );
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
    }

    /* 检查参数数量 */
    if (msg->parm_count != 1)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
    }

    /* 解析A参数 */
    if (strcmp(msg->parm[1], "ON") == 0)
    {
        display_value = 1;
    }
    else if (strcmp(msg->parm[1], "OFF") == 0)
    {
        display_value = 0;
    }
    else
    {
        LOG_INF("%s=>invalid A param: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }

    /* 所有参数验证通过,统一赋值 */
    gConfigParam.led_config.flag = FLAG_VALID;
    gConfigParam.led_config.led_display = (uint8_t)display_value;

    // 只有在LTE就绪状态下才发送LED指令
    if (g_bLteReady == 1)
    {
        send_led_command();
    }

    /* 保存配置 */
    my_user_data_write(ZMS_ID_LED_CONFIG, &gConfigParam.led_config, sizeof(led_config_t));

    LOG_INF("%s=>%s,%s", __func__, msg->parm[0], msg->parm[1]);

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
    LOG_INF("LED: Display=%d", gConfigParam.led_config.led_display);

    return BLE_DATA_TYPE_PACKET_MULTIPLE;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
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
    uint16_t T1, T2;
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
    if (no_count == 0 || no_count > 6)
    {
        LOG_INF("%s=>invalid T1 param: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }

    no_count = string_check_is_number(0, msg->parm[2]);
    if (no_count == 0 || no_count > 6)
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
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
    LOG_INF("LTINT: T1=%d,T2=%d", gConfigParam.ltint_config.T1,gConfigParam.ltint_config.T2);

    return BLE_DATA_TYPE_PACKET_MULTIPLE;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
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
        msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
    }

    no_count = string_check_is_number(0, msg->parm[1]);
    if (no_count == 0)
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
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
    LOG_INF("BUZZER: Operator=%d", gConfigParam.buzzer_config.buzzer_operator);

    //TODO 具体逻辑处理
    my_set_buzzer_mode(operator_value);

    return BLE_DATA_TYPE_PACKET_MULTIPLE;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
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
            return BLE_DATA_TYPE_PACKET_MULTIPLE;
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
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_BTLOG_FAIL");
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
**           模式2: MODESET,2,[Sub Mode],[Static INT],[MOVING INT]#
**           模式3: MODESET,3#
**返 回 值:  BLE数据类型
*********************************************************************/
static int modeset_cmd_handler(at_cmd_t* msg)
{
    uint8_t no_count = 0;
    uint16_t remaining;
    device_work_mode_config_t param_work_mode_config;
    uint8_t gnss_sw;
    uint8_t sub_mode_val;
    uint32_t static_int_val;
    uint32_t moving_int_val;

    remaining = RESP_STRING_LENGTH_MAX;  // 计算响应消息缓冲区的大小

    if(msg->parm_count == 0)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        goto param_invalid;
    }

    no_count = string_check_is_number(0, msg->parm[1]);
    if (no_count == 0)
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
        // 切换到指定工作模式
        switch_work_mode(param_work_mode_config.current_mode);

        gConfigParam.device_workmode_config.flag = FLAG_VALID;
        /* 保存配置 */
        my_user_data_write(ZMS_ID_WORK_MODE_CONFIG, &gConfigParam.device_workmode_config, sizeof(device_work_mode_config_t));

        /* 生成成功响应 */
        msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
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
        if (no_count == 0)
        {
            LOG_INF("%s=>invalid Reporting Interval Sec param: %s", __func__, msg->parm[2]);
            goto param_invalid;
        }
        no_count = string_check_is_number(0, msg->parm[3]);
        if (no_count == 0)
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
        my_user_data_write(ZMS_ID_WORK_MODE_CONFIG, &gConfigParam.device_workmode_config, sizeof(device_work_mode_config_t));

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
        if (no_count == 0)
        {
            LOG_INF("%s=>invalid Reporting Interval Min param: %s", __func__, msg->parm[2]);
            goto param_invalid;
        }

        // 解析GNSS SW参数（ON/OFF字符串）
        if (strcmp(msg->parm[4], "ON") == 0)
        {
            gnss_sw = 1;
        }
        else if (strcmp(msg->parm[4], "OFF") == 0)
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
        my_user_data_write(ZMS_ID_WORK_MODE_CONFIG, &gConfigParam.device_workmode_config, sizeof(device_work_mode_config_t));

        if (gConfigParam.device_workmode_config.workmode_config.current_mode == MY_MODE_LONG_LIFE)
        {
            send_work_mode_command(param_work_mode_config.current_mode);
            // 重新开启LTE定时器
            my_send_msg(MOD_MAIN, MOD_MAIN, MY_MSG_RESET_LTE_TIMER);
        }

        LOG_INF("%s,%s,%s,%s,%s#", msg->parm[0], msg->parm[1], msg->parm[2], msg->parm[3], msg->parm[4]);
    }
    // 智能模式处理: MODESET,2,[Sub Mode],[Static INT],[MOVING INT]#
    else if (param_work_mode_config.current_mode == MY_MODE_SMART)
    {
        /* 检查参数数量 */
        if (msg->parm_count != 4)
        {
            LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
            goto param_invalid;
        }

        no_count = string_check_is_number(0, msg->parm[2]);
        if (no_count == 0)
        {
            LOG_INF("%s=>invalid Sub Mode param: %s", __func__, msg->parm[2]);
            goto param_invalid;
        }
        no_count = string_check_is_number(0, msg->parm[3]);
        if (no_count == 0)
        {
            LOG_INF("%s=>invalid Static INT param: %s", __func__, msg->parm[3]);
            goto param_invalid;
        }
        no_count = string_check_is_number(0, msg->parm[4]);
        if (no_count == 0)
        {
            LOG_INF("%s=>invalid Moving INT param: %s", __func__, msg->parm[4]);
            goto param_invalid;
        }

        // 解析智能模式参数
        sub_mode_val = atoi(msg->parm[2]);
        static_int_val = atoi(msg->parm[3]);
        moving_int_val = atoi(msg->parm[4]);

        // 设置智能模式参数（内部完成参数校验）
        if (set_intelligent_params(&gConfigParam.device_workmode_config.workmode_config,
            sub_mode_val, static_int_val, moving_int_val) < 0)
        {
            LOG_INF("%s=>set_intelligent_params failed", __func__);
            goto param_invalid;
        }

        gConfigParam.device_workmode_config.flag = FLAG_VALID;
        /* 保存配置 */
        my_user_data_write(ZMS_ID_WORK_MODE_CONFIG, &gConfigParam.device_workmode_config, sizeof(device_work_mode_config_t));

        if (gConfigParam.device_workmode_config.workmode_config.current_mode == MY_MODE_SMART)
        {
            send_work_mode_command(param_work_mode_config.current_mode);
            // 参数变更后，基于当前运动状态重新应用LTE唤醒策略
            smart_mode_apply_lte_policy();
        }

        LOG_INF("%s,%s,%s,%s,%s#", msg->parm[0], msg->parm[1], msg->parm[2], msg->parm[3], msg->parm[4]);
    }
    // 常在线模式处理: MODESET,3# （无附加参数）
    else if (param_work_mode_config.current_mode == MY_MODE_ALWAYS_ONLINE)
    {
        if (msg->parm_count != 1)
        {
            LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
            goto param_invalid;
        }

        // 常在线模式无附加参数，仅切换模式
        switch_work_mode(MY_MODE_ALWAYS_ONLINE);

        gConfigParam.device_workmode_config.flag = FLAG_VALID;
        /* 保存配置 */
        my_user_data_write(ZMS_ID_WORK_MODE_CONFIG, &gConfigParam.device_workmode_config, sizeof(device_work_mode_config_t));

        LOG_INF("MODESET,3 (ALWAYS_ONLINE)");
    }

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
    LOG_INF("MODESET: current_mode:%d", param_work_mode_config.current_mode);

    return BLE_DATA_TYPE_PACKET_MULTIPLE;

param_invalid:
    // 生成失败响应
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
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
                    gConfigParam.device_workmode_config.workmode_config.intelligent.static_interval,
                    gConfigParam.device_workmode_config.workmode_config.intelligent.moving_interval);
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
                    gConfigParam.device_workmode_config.workmode_config.intelligent.static_interval,
                    gConfigParam.device_workmode_config.workmode_config.intelligent.moving_interval,
                    MY_MODE_ALWAYS_ONLINE);

        if (ret > 0 && ret < remaining)  // 检查响应消息是否生成成功
        {
            msg->resp_length = ret;  // 设置响应消息的长度
            LOG_INF("MODEPARAM: %s", msg->resp_msg);  // 输出状态信息
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
    if (strcmp(msg->parm[1], "ADD") == 0)
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
        msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_ADD_OK", msg->parm[0]);
        return BLE_DATA_TYPE_PACKET_MULTIPLE;
    }
    else if (strcmp(msg->parm[1], "DEL") == 0)
    {
        if (msg->parm_count != 2)
        {
            goto param_invalid;
        }

        // BT_PARMAC,DEL,ALL#
        if (strcmp(msg->parm[2], "ALL") == 0)
        {
            my_tran_mac_del_all();
            LOG_INF("DEL ALL");
            msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_DEL_ALL_OK", msg->parm[0]);
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
                msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_DEL_OK", msg->parm[0]);
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
    else if (strcmp(msg->parm[1], "CHECK") == 0)
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
            msg->resp_length = snprintf(msg->resp_msg, remaining, "check not find any mac.");
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
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
    return BLE_DATA_TYPE_PACKET_MULTIPLE;
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
    int ret;                // snprintf 函数的返回值
    char motion[20];        // 运动状态
    char net_signal[10];    // 网络信号
    char gnss_signal[15];   // GNSS信号

    remaining = RESP_STRING_LENGTH_MAX;  // 计算响应消息缓冲区的大小

    /* 检查参数数量：应为0 */
    if (msg->parm_count == 0)  // 检查命令是否有参数
    {
        LOG_INF("%s=>%s", __func__, msg->parm[0]);  // 输出函数名和命令名

        switch (g_lte_net_signal_level)
        {
            case 0:
                memcpy(net_signal, "NA", sizeof("NA"));
                break;
            case 1:
            case 2:
                memcpy(net_signal, "Weak", sizeof("Weak"));
                break;
            case 3:
                memcpy(net_signal, "Normal", sizeof("Normal"));
                break;
            case 4:
                memcpy(net_signal, "Strong", sizeof("Strong"));
                break;
            default:
                memcpy(net_signal, "Unknown", sizeof("Unknown"));
                break;
        }

        switch (g_lte_gps_state)
        {
            case 0:
                memcpy(gnss_signal, "OFF", sizeof("OFF"));
                break;
            case 1:
                memcpy(gnss_signal, "Searching", sizeof("Searching"));
                break;
            case 2:
                memcpy(gnss_signal, "Fix", sizeof("Fix"));
                break;
            default:
                memcpy(gnss_signal, "Unknown", sizeof("Unknown"));
                break;
        }

        switch (g_gsensor_runtime_ctx.current_gsensor_state)
        {
            case STATE_STATIC:
                memcpy(motion, "Static", sizeof("Static"));
                break;

            case STATE_LAND_TRANSPORT:
                memcpy(motion, "Land Transport", sizeof("Land Transport"));
                break;

            case STATE_SEA_TRANSPORT:
                memcpy(motion, "Sea Transport", sizeof("Sea Transport"));
                break;

            default:
                memcpy(motion, "Unknown", sizeof("Unknown"));
                break;
        }

        // TODO: Network,GNSS采用的默认值，后续需要根据实际情况修改
        /* 生成响应消息，格式：[VERSION]%s*/
        ret = snprintf(msg->resp_msg, remaining, "Battery:%d%%(%s);Network:%s(%s);GNSS:%s(%s); \
            Tamper:%s;Motion:%s(%.2f KM/h)",
            get_show_percent(),
            g_charg_state == NO_CHARGING ? "Discharging" : "Charging",
            g_lte_net_flag == 0 ? "Disconnect" : "Connect",
            net_signal, gnss_signal, g_lte_gps_signal,
            get_light_tamper_state() ? "Remove" : "Noemal",
            motion, g_location_point.speed);

        if (ret > 0 && ret < remaining)  // 检查响应消息是否生成成功
        {
            msg->resp_length = ret;  // 设置响应消息的长度
            LOG_INF("STATUS: %s", msg->resp_msg);  // 输出状态信息
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
    return BLE_DATA_TYPE_PACKET_MULTIPLE;  // 返回 BLE 数据类型
}