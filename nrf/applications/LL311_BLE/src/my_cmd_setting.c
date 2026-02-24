#include "my_comm.h"

LOG_MODULE_REGISTER(my_cmd_setting, LOG_LEVEL_INF);

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