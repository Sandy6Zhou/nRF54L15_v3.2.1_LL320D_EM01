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
*********************************************************************/

#include "my_comm.h"

LOG_MODULE_REGISTER(my_cmd_setting, LOG_LEVEL_INF);

/* 全局指令配置变量定义 */
DeviceCmdConfig g_device_cmd_config = {
    /* REMALM 指令默认配置 */
    .remalm_sw = 0,                    /* 默认关闭 */
    .remalm_mode = 0,                  /* 默认GPRS */

    /* LOCKPINCY 指令默认配置 */
    .lockpincy_report = 0,             /* 默认不上报 */
    .lockpincy_buzzer = 0,             /* 默认不报警 */

    /* LOCKERR 指令默认配置 */
    .lockerr_report = 1,               /* 默认GPRS */
    .lockerr_buzzer = 0,               /* 默认不报警 */

    /* PINSTAT 指令默认配置 */
    .pinstat_report = 0,               /* 默认不上报 */
    .pinstat_trigger = 0,              /* 默认都不触发 */

    /* LOCKSTAT 指令默认配置 */
    .lockstat_report = 0,              /* 默认GPRS */
    .lockstat_trigger = 0,             /* 默认都不触发 */

    /* MOTDET 指令默认配置 */
    .motdet_static_g = 10,             /* 默认10 mg */
    .motdet_land_g = 2000,             /* 默认2000 mg */
    .motdet_static_land_length = 50,   /* 默认50 s */
    .motdet_sea_transport_time = 10,   /* 默认10 s */
    .motdet_report_type = 0,           /* 默认GPRS */

    /* BATLEVEL 指令默认配置 */
    .batlevel_empty_trg = 2,           /* 默认状态变化触发 */
    .batlevel_empty_rpt = 0,           /* 默认GPRS */
    .batlevel_low_trg = 2,             /* 默认状态变化触发 */
    .batlevel_low_rpt = 0,             /* 默认GPRS */
    .batlevel_normal_trg = 1,          /* 默认在线触发 */
    .batlevel_normal_rpt = 0,          /* 默认GPRS */
    .batlevel_fair_trg = 1,            /* 默认在线触发 */
    .batlevel_fair_rpt = 0,            /* 默认GPRS */
    .batlevel_high_trg = 1,            /* 默认在线触发 */
    .batlevel_high_rpt = 0,            /* 默认GPRS */
    .batlevel_full_trg = 2,            /* 默认状态变化触发 */
    .batlevel_full_rpt = 0,            /* 默认GPRS */

    /* CHARGESTA 指令默认配置 */
    .chargesta_report = 0,             /* 默认GPRS */

    /* SHOCKALARM 指令默认配置 */
    .shockalarm_sw = 0,                /* 默认关闭 */
    .shockalarm_level = 3,             /* 默认中等敏感度 */
    .shockalarm_type = 0,              /* 默认GPRS */

    /* STARTR 指令默认配置 */
    .startr_sw = 0,                    /* 默认关闭 */

    /* PWRSAVE 指令默认配置 */
    .pwsave_sw = 0,                    /* 默认关闭 */

    /* BT_CRFPWR 指令默认配置 */
    .bt_crfpwr = 0,                    /* 默认0 dBm */

    /* BT_UPDATA 指令默认配置 */
    .bt_updata_mode = 0,               /* 默认不开启 */
    .bt_updata_scan_interval = 600,    /* 默认600秒 */
    .bt_updata_scan_length = 10,       /* 默认10秒 */
    .bt_updata_updata_interval = 14400,/* 默认14400秒 */

    /* TAG 指令默认配置 */
    .tag_sw = 0,                       /* 默认关闭 */
    .tag_interval = 2000,              /* 默认2000ms */

    /* LOCKCD 指令默认配置 */
    .lockcd_countdown = 3,             /* 默认3秒 */

    /* LED 指令默认配置 */
    .led_display = 0,                  /* 默认关闭 */

    /* BUZZER 直接控制指令默认配置 */
    .buzzer_operator = 0,             /* 默认停止 */

    /* NCFTRIG 指令默认配置 */
    .ncftrig_nfc_no = {0},           /* 默认空 */
    .ncftrig_command = {0},          /* 默认空 */

};

static int remalm_cmd_handler(at_cmd_struc* msg);
static int lockpincy_cmd_handler(at_cmd_struc* msg);
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
static int ncftrig_cmd_handler(at_cmd_struc* msg);

static const at_cmd_attr_t at_cmd_attr_table[] =
{
    {"REMALM",         remalm_cmd_handler},
    {"LOCKPINCYT",     lockpincy_cmd_handler},
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
    {"LOCKCD",         lockcd_cmd_handler},
    {"LED",            led_cmd_handler},
    {"BUZZER",         buzzer_cmd_handler},
    {"NCFTRIG",        ncftrig_cmd_handler},
};

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
**           sea_int     --  海运状态上报间隔（秒）
**           sleep_sw    --  休眠开关（0/1/2）
**出口参数:  无
**函数功能:  设置智能模式参数
**返 回 值:  0 表示成功，-1 表示失败（参数非法）
*********************************************************************/
int set_intelligent_params(DeviceWorkModeConfig *p_workmode, uint32_t static_int,
                     uint32_t land_int, uint32_t sea_int, uint8_t sleep_sw)
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

    if (str_data == NULL)
    {
        return -1;
    }

    len = strlen(str_data);
    for (i = 0, j = 0, p = str_data; i < len; i++, p++)
    {
        if (status == 0 && (*p == startChar || startChar == NULL))
        {
            status = 1;
            if (j >= limit)
            {
                return -2;
            }

            if (startChar == NULL)
            {
                tar_data[j++] = p;
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

        if (strchr(endChars, *p) != NULL)
        {
            *p = 0;
            break;
        }

        if (*p == splitChar)
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
    uint8_t  cmd_Len, par_len;
    uint16_t cmd_type = 0;
    uint8_t index;

    data_ptr = at_cmd_msg->rcv_msg;
    cmd_Len = strlen(data_ptr);
    data_ptr[cmd_Len] = '\r';
    data_ptr[cmd_Len+1] = '\n';

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
    // 匹配指令并执行对应处理函数
    for (index = 0; index < AT_CMD_TABLE_TOTAL; index++)
    {
        if (strcmp(at_cmd_attr_table[index].cmd_str, at_cmd_msg->parm[PARM_1]) == 0)
        {
            if (at_cmd_attr_table[index].cmd_func != NULL)
            {
                cmd_type = at_cmd_attr_table[index].cmd_func(at_cmd_msg);
            }
            break;
        }
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
    g_device_cmd_config.remalm_sw = (uint8_t)sw_value;
    g_device_cmd_config.remalm_mode = (uint8_t)m_value;

    LOG_INF("%s=>%s,%s,%s", __func__, msg->parm[0], msg->parm[1], msg->parm[2]);

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
    LOG_INF("REMALM: SW=%d, M=%d", g_device_cmd_config.remalm_sw, g_device_cmd_config.remalm_mode);

    //TODO 具体逻辑处理

    return BLE_DATA_TYPE_AT_CMD;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
    return BLE_DATA_TYPE_AT_CMD;
}

/********************************************************************
**函数名称:  lockpincy_cmd_handler
**入口参数:  msg      ---        AT指令结构体指针
**出口参数:  msg->resp_msg  ---  响应消息
**           msg->resp_length --- 响应长度
**函数功能:  处理LOCKPINCYT指令：设置锁销非法拔除事件上报与响应
**指令格式:  LOCKPINCYT,[Report],[Buzzer]#
**参数说明:  [Report] - 上报方式: 0-不上报, 1-GPRS, 2-GPRS+SMS, 3-GPRS+SMS+CALL
**           [Buzzer] - 蜂鸣器报警方式: 0-不报警, 1-报警30s, 2-持续报警
**返 回 值:  BLE数据类型
*********************************************************************/
static int lockpincy_cmd_handler(at_cmd_struc* msg)
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
    g_device_cmd_config.lockpincy_report = (uint8_t)report_value;
    g_device_cmd_config.lockpincy_buzzer = (uint8_t)buzzer_value;

    LOG_INF("%s=>%s,%s,%s", __func__, msg->parm[0], msg->parm[1], msg->parm[2]);

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
    LOG_INF("LOCKPINCYT: Report=%d, Buzzer=%d",
           g_device_cmd_config.lockpincy_report,
           g_device_cmd_config.lockpincy_buzzer);

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
    g_device_cmd_config.lockerr_report = (uint8_t)report_value;
    g_device_cmd_config.lockerr_buzzer = (uint8_t)buzzer_value;

    LOG_INF("%s=>%s,%s,%s", __func__, msg->parm[0], msg->parm[1], msg->parm[2]);

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
    LOG_INF("LOCKERR: Report=%d, Buzzer=%d",
           g_device_cmd_config.lockerr_report,
           g_device_cmd_config.lockerr_buzzer);

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
    g_device_cmd_config.pinstat_report = (uint8_t)report_value;
    g_device_cmd_config.pinstat_trigger = (uint8_t)trigger_value;

    LOG_INF("%s=>%s,%s,%s", __func__, msg->parm[0], msg->parm[1], msg->parm[2]);

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
    LOG_INF("PINSTAT: Report=%d, Trigger=%d",
           g_device_cmd_config.pinstat_report,
           g_device_cmd_config.pinstat_trigger);

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
    g_device_cmd_config.lockstat_report = (uint8_t)report_value;
    g_device_cmd_config.lockstat_trigger = (uint8_t)trigger_value;

    LOG_INF("%s=>%s,%s,%s", __func__, msg->parm[0], msg->parm[1], msg->parm[2]);

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
    LOG_INF("LOCKSTAT: Report=%d, Trigger=%d",
           g_device_cmd_config.lockstat_report,
           g_device_cmd_config.lockstat_trigger);

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
    g_device_cmd_config.motdet_static_g = (uint16_t)static_g_value;
    g_device_cmd_config.motdet_land_g = (uint16_t)land_g_value;
    g_device_cmd_config.motdet_static_land_length = (uint16_t)static_land_length_value;
    g_device_cmd_config.motdet_sea_transport_time = (uint16_t)sea_transport_time_value;
    g_device_cmd_config.motdet_report_type = (uint8_t)report_type_value;

    LOG_INF("%s=>%s,%s,%s,%s,%s,%s", __func__, msg->parm[0], msg->parm[1],
           msg->parm[2], msg->parm[3], msg->parm[4], msg->parm[5]);

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
    LOG_INF("MOTDET: StaticG=%d, LandG=%d, StaticLandLen=%d, SeaTime=%d, ReportType=%d",
           g_device_cmd_config.motdet_static_g,
           g_device_cmd_config.motdet_land_g,
           g_device_cmd_config.motdet_static_land_length,
           g_device_cmd_config.motdet_sea_transport_time,
           g_device_cmd_config.motdet_report_type);

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
**指令格式:  BATLEVEL,[Empty TRG],[Empty RPT],[LOW TRG],[LOW RPT],
**                  [Normal TRG],[Normal RPT],[Fair TRG],[Fair RPT],
**                  [High TRG],[High RPT],[Full TRG],[Full RPT]#
**参数说明:  共12个参数，每组对应一个电量状态的触发方式和上报方式
**           TRG参数: 0-不触发, 1-在线触发, 2-状态变化触发
**           RPT参数: 0-GPRS, 1-GPRS+SMS, 2-GPRS+SMS+CALL
**默认设置: BATLEVEL,2,0,2,0,1,0,1,0,1,0,2,0#
**返 回 值:  BLE数据类型
*********************************************************************/
static int batlevel_cmd_handler(at_cmd_struc* msg)
{
    uint16_t remaining;
    int param_values[12];
    int i;

    remaining = sizeof(msg->resp_msg);

    /* 检查参数数量 */
    if (msg->parm_count != 12)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
        return BLE_DATA_TYPE_AT_CMD;
    }

    /* 解析所有12个参数 */
    for (i = 0; i < 12; i++)
    {
        param_values[i] = atoi(msg->parm[i + 1]);
    }

    /* 验证Empty状态参数 (参数0-1) */
    if (param_values[0] < 0 || param_values[0] > 2)
    {
        LOG_INF("%s=>invalid Empty TRG param: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }

    if (param_values[1] < 0 || param_values[1] > 2)
    {
        LOG_INF("%s=>invalid Empty RPT param: %s", __func__, msg->parm[2]);
        goto param_invalid;
    }

    /* 验证Low状态参数 (参数2-3) */
    if (param_values[2] < 0 || param_values[2] > 2)
    {
        LOG_INF("%s=>invalid LOW TRG param: %s", __func__, msg->parm[3]);
        goto param_invalid;
    }

    if (param_values[3] < 0 || param_values[3] > 2)
    {
        LOG_INF("%s=>invalid LOW RPT param: %s", __func__, msg->parm[4]);
        goto param_invalid;
    }

    /* 验证Normal状态参数 (参数4-5) */
    if (param_values[4] < 0 || param_values[4] > 2)
    {
        LOG_INF("%s=>invalid Normal TRG param: %s", __func__, msg->parm[5]);
        goto param_invalid;
    }

    if (param_values[5] < 0 || param_values[5] > 2)
    {
        LOG_INF("%s=>invalid Normal RPT param: %s", __func__, msg->parm[6]);
        goto param_invalid;
    }

    /* 验证Fair状态参数 (参数6-7) */
    if (param_values[6] < 0 || param_values[6] > 2)
    {
        LOG_INF("%s=>invalid Fair TRG param: %s", __func__, msg->parm[7]);
        goto param_invalid;
    }

    if (param_values[7] < 0 || param_values[7] > 2)
    {
        LOG_INF("%s=>invalid Fair RPT param: %s", __func__, msg->parm[8]);
        goto param_invalid;
    }

    /* 验证High状态参数 (参数8-9) */
    if (param_values[8] < 0 || param_values[8] > 2)
    {
        LOG_INF("%s=>invalid High TRG param: %s", __func__, msg->parm[9]);
        goto param_invalid;
    }

    if (param_values[9] < 0 || param_values[9] > 2)
    {
        LOG_INF("%s=>invalid High RPT param: %s", __func__, msg->parm[10]);
        goto param_invalid;
    }

    /* 验证Full状态参数 (参数10-11) */
    if (param_values[10] < 0 || param_values[10] > 2)
    {
        LOG_INF("%s=>invalid Full TRG param: %s", __func__, msg->parm[11]);
        goto param_invalid;
    }

    if (param_values[11] < 0 || param_values[11] > 2)
    {
        LOG_INF("%s=>invalid Full RPT param: %s", __func__, msg->parm[12]);
        goto param_invalid;
    }

    /* 所有参数验证通过,统一赋值 */
    g_device_cmd_config.batlevel_empty_trg = (uint8_t)param_values[0];
    g_device_cmd_config.batlevel_empty_rpt = (uint8_t)param_values[1];
    g_device_cmd_config.batlevel_low_trg = (uint8_t)param_values[2];
    g_device_cmd_config.batlevel_low_rpt = (uint8_t)param_values[3];
    g_device_cmd_config.batlevel_normal_trg = (uint8_t)param_values[4];
    g_device_cmd_config.batlevel_normal_rpt = (uint8_t)param_values[5];
    g_device_cmd_config.batlevel_fair_trg = (uint8_t)param_values[6];
    g_device_cmd_config.batlevel_fair_rpt = (uint8_t)param_values[7];
    g_device_cmd_config.batlevel_high_trg = (uint8_t)param_values[8];
    g_device_cmd_config.batlevel_high_rpt = (uint8_t)param_values[9];
    g_device_cmd_config.batlevel_full_trg = (uint8_t)param_values[10];
    g_device_cmd_config.batlevel_full_rpt = (uint8_t)param_values[11];

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
    LOG_INF("BATLEVEL: Empty(TRG=%d,RPT=%d), Low(TRG=%d,RPT=%d), Normal(TRG=%d,RPT=%d), Fair(TRG=%d,RPT=%d), High(TRG=%d,RPT=%d), Full(TRG=%d,RPT=%d)",
           g_device_cmd_config.batlevel_empty_trg, g_device_cmd_config.batlevel_empty_rpt,
           g_device_cmd_config.batlevel_low_trg, g_device_cmd_config.batlevel_low_rpt,
           g_device_cmd_config.batlevel_normal_trg, g_device_cmd_config.batlevel_normal_rpt,
           g_device_cmd_config.batlevel_fair_trg, g_device_cmd_config.batlevel_fair_rpt,
           g_device_cmd_config.batlevel_high_trg, g_device_cmd_config.batlevel_high_rpt,
           g_device_cmd_config.batlevel_full_trg, g_device_cmd_config.batlevel_full_rpt);

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
**参数说明:  [RPT] - 状态变化时的上报方式: 0-GPRS(默认), 1-GPRS+SMS, 2-GPRS+SMS+CALL
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
    if (report_value < 0 || report_value > 2)
    {
        LOG_INF("%s=>invalid RPT param: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }
    g_device_cmd_config.chargesta_report = (uint8_t)report_value;

    LOG_INF("%s=>%s,%s", __func__, msg->parm[0], msg->parm[1]);

    /* 所有参数验证通过,生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
    LOG_INF("CHARGESTA: Report=%d", g_device_cmd_config.chargesta_report);

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
    g_device_cmd_config.shockalarm_sw = (uint8_t)sw_value;
    g_device_cmd_config.shockalarm_level = (uint8_t)level_value;
    g_device_cmd_config.shockalarm_type = (uint8_t)type_value;

    LOG_INF("%s=>%s,%s,%s,%s", __func__, msg->parm[0], msg->parm[1], msg->parm[2], msg->parm[3]);

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
    LOG_INF("SHOCKALARM: SW=%d, Level=%d, Type=%d",
           g_device_cmd_config.shockalarm_sw,
           g_device_cmd_config.shockalarm_level,
           g_device_cmd_config.shockalarm_type);

    //TODO 具体逻辑处理

    return BLE_DATA_TYPE_AT_CMD;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
    return BLE_DATA_TYPE_AT_CMD;
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
            g_device_cmd_config.pwsave_sw = 1;
            LOG_INF("%s=>%s,%s", __func__, msg->parm[0], msg->parm[1]);

            /* 根据指令说明，立即回复 "Poweroff OK" */
            msg->resp_length = snprintf(msg->resp_msg, remaining, "Poweroff OK");

            //TODO 具体逻辑处理
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
        if (g_device_cmd_config.startr_sw == 1)
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
        g_device_cmd_config.startr_sw = 1;
    }
    else if (strcmp(msg->parm[1], "OFF") == 0)
    {
        g_device_cmd_config.startr_sw = 0;
    }
    else
    {
        LOG_INF("%s=>invalid A param: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }

    /* 所有参数验证通过,生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
    LOG_INF("STARTR: SW=%d", g_device_cmd_config.startr_sw);

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

    int ret;

    remaining = sizeof(msg->resp_msg);

    /* 检查参数数量：应为0 */
    if (msg->parm_count == 0)
    {
        LOG_INF("%s=>%s", __func__, msg->parm[0]);

#if 1
        uint16_t battery_voltage_mv;
        int battery_temp_c;
        const char* charge_status;

        /* TODO 测试数据,后续需要更改为实际数据 */
        battery_voltage_mv = 3000;      /* 示例：3000mV */
        battery_temp_c = 37;            /* 示例：37℃ */
        charge_status = "CHARGE_IN";    /* 示例：正在充电 */
#endif
        /* 生成响应消息，格式：RETURN CBMT:CHARGNIG=XXX,VBAT=XXXX,VBATTEMP=XX.XX */
        ret = snprintf(msg->resp_msg, remaining, "RETURN_CBMT:CHARGNIG=%s,VBAT=%u,VBATTEMP=%d",
                      charge_status, battery_voltage_mv, battery_temp_c);

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
        && a_value != 5 && a_value != 7 && a_value != 12)
    {
        LOG_INF("%s=>invalid A param: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }
    g_device_cmd_config.bt_crfpwr = (int8_t)a_value;

    LOG_INF("%s=>%s,%s", __func__, msg->parm[0], msg->parm[1]);

    /* 所有参数验证通过,生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
    LOG_INF("BT_CRFPWR: A=%d", g_device_cmd_config.bt_crfpwr);

    //TODO 具体逻辑处理

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
    g_device_cmd_config.bt_updata_mode = (uint8_t)mode_value;
    g_device_cmd_config.bt_updata_scan_interval = scan_interval_value;
    g_device_cmd_config.bt_updata_scan_length = scan_length_value;
    g_device_cmd_config.bt_updata_updata_interval = updata_interval_value;

    LOG_INF("%s=>%s,%s,%s,%s,%s", __func__, msg->parm[0], msg->parm[1],
           msg->parm[2], msg->parm[3], msg->parm[4]);

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
    LOG_INF("BT_UPDATA: Mode=%d, ScanInterval=%u, ScanLength=%u, UpdataInterval=%u",
           g_device_cmd_config.bt_updata_mode,
           g_device_cmd_config.bt_updata_scan_interval,
           g_device_cmd_config.bt_updata_scan_length,
           g_device_cmd_config.bt_updata_updata_interval);

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
    g_device_cmd_config.tag_sw = (uint8_t)sw_value;
    if (msg->parm_count == 2)
    {
        g_device_cmd_config.tag_interval = (uint16_t)interval_value;
    }

    LOG_INF("%s=>%s,%s", __func__, msg->parm[0], msg->parm[1]);
    if (msg->parm_count == 2)
    {
        LOG_INF("%s=>%s", __func__, msg->parm[2]);
    }

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
    LOG_INF("TAG: SW=%d, Interval=%u", g_device_cmd_config.tag_sw, g_device_cmd_config.tag_interval);

    //TODO 具体逻辑处理

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
    g_device_cmd_config.lockcd_countdown = (uint16_t)countdown_value;

    LOG_INF("%s=>%s,%s", __func__, msg->parm[0], msg->parm[1]);

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
    LOG_INF("LOCKCD: Countdown=%u", g_device_cmd_config.lockcd_countdown);

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
    g_device_cmd_config.led_display = (uint8_t)display_value;

    LOG_INF("%s=>%s,%s", __func__, msg->parm[0], msg->parm[1]);

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
    LOG_INF("LED: Display=%d", g_device_cmd_config.led_display);

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
    g_device_cmd_config.buzzer_operator = (uint8_t)operator_value;

    LOG_INF("%s=>%s,%s", __func__, msg->parm[0], msg->parm[1]);

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
    LOG_INF("BUZZER: Operator=%d", g_device_cmd_config.buzzer_operator);

    //TODO 具体逻辑处理

    return BLE_DATA_TYPE_AT_CMD;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
    return BLE_DATA_TYPE_AT_CMD;
}

/********************************************************************
**函数名称:  ncftrig_cmd_handler
**入口参数:  msg      ---        AT指令结构体指针
**出口参数:  msg->resp_msg  ---  响应消息
**           msg->resp_length --- 响应长度
**函数功能:  处理NCFTRIG指令：将NFC卡号与需要执行的指令关联
**指令格式:  NCFTRIG,[NFC NO.],[Command]#
**参数说明:  NFC NO. - 联动的NFC卡号
**           Command - 需要执行的完整可执行指令
**返 回 值:  BLE数据类型
*********************************************************************/
static int ncftrig_cmd_handler(at_cmd_struc* msg)
{
    uint16_t remaining;
    int nfc_no_len;
    int command_len;

    remaining = sizeof(msg->resp_msg);

    /* 检查参数数量 */
    if (msg->parm_count != 2)
    {
        LOG_INF("%s=>%s, param count error: %d", __func__, msg->parm[0], msg->parm_count);
        msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
        return BLE_DATA_TYPE_AT_CMD;
    }

    /* 解析NFC NO.参数 */
    nfc_no_len = strlen(msg->parm[1]);
    if (nfc_no_len == 0 || nfc_no_len >= sizeof(g_device_cmd_config.ncftrig_nfc_no))
    {
        LOG_INF("%s=>invalid NFC NO. param: %s", __func__, msg->parm[1]);
        goto param_invalid;
    }

    /* 解析Command参数 */
    command_len = strlen(msg->parm[2]);
    if (command_len == 0 || command_len >= sizeof(g_device_cmd_config.ncftrig_command))
    {
        LOG_INF("%s=>invalid Command param: %s", __func__, msg->parm[2]);
        goto param_invalid;
    }

    /* 所有参数验证通过,统一赋值 */
    strcpy(g_device_cmd_config.ncftrig_nfc_no, msg->parm[1]);
    strcpy(g_device_cmd_config.ncftrig_command, msg->parm[2]);

    LOG_INF("%s=>%s,%s,%s", __func__, msg->parm[0], msg->parm[1], msg->parm[2]);

    /* 生成成功响应 */
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
    LOG_INF("NCFTRIG: NFC_NO=%s, Command=%s",
           g_device_cmd_config.ncftrig_nfc_no,
           g_device_cmd_config.ncftrig_command);

    //TODO 具体逻辑处理

    return BLE_DATA_TYPE_AT_CMD;

param_invalid:
    msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
    return BLE_DATA_TYPE_AT_CMD;
}