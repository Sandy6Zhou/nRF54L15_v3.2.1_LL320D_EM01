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

static int remalm_cmd_handler(at_cmd_struc* msg);

static const at_cmd_attr_t at_cmd_attr_table[] =
{
    {"REMALM",         remalm_cmd_handler},
    // TODO 后续增加其他指令
};

// REMALM,<SW>,<M>
static int remalm_cmd_handler(at_cmd_struc* msg)
{
    uint16_t remaining = sizeof(msg->resp_msg);
    uint8_t invalid = 0;

    if(msg->parm_count == 2)
    {
        if (strcmp(msg->parm[1], "ON") == 0)
        {
            //TODO 存储对应的状态信息
            //TODO 具体的逻辑处理
        }
        else if (strcmp(msg->parm[1], "OFF") == 0)
        {
            //TODO
        }
        else
        {
            invalid = 1; // 非法值标记
            LOG_INF("%s=>invalid SW param: %s", __func__, msg->parm[1]);
        }

#if 0
        //TODO 存储对应的状态信息
        //TODO 具体的逻辑处理
        int m_value = atoi(msg->parm[2]);
        if (m_value < 0 || m_value > 2) {
            LOG_INF("%s=>invalid M param: %s", __func__, msg->parm[2]);
            invalid = 1; // 非法值标记
        }
#endif
        LOG_INF("%s=>%s,%s,%s", __func__, msg->parm[0], msg->parm[1], msg->parm[2]);

        if (invalid)
        {
            msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
        }
        else
        {
            msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_OK", msg->parm[0]);
        }
    }
    else
    {
        LOG_INF("%s=>%s", __func__, msg->parm[0]);
        msg->resp_length = snprintf(msg->resp_msg, remaining, "RETURN_%s_FAIL", msg->parm[0]);
    }
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