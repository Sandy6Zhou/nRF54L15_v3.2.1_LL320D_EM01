#include "my_comm.h"

LOG_MODULE_REGISTER(my_tool, LOG_LEVEL_INF);

// 一天的总秒数（24*60*60）
#define DAY_SECONDS 86400

/*********************************************************************
**函数名称:  my_set_system_time
**入口参数:  _sec  --  自 1970-01-01 00:00:00 UTC 起的秒数
**出口参数:  无
**函数功能:  设置系统时间（RTC实时时钟）
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
int my_set_system_time(time_t _sec)
{
    struct timespec ts;
    int ret;

    ts.tv_sec  = _sec;
    ts.tv_nsec = 0;

    ret = sys_clock_settime(SYS_CLOCK_REALTIME, &ts);
    if (ret < 0)
    {
        LOG_INF("sys_clock_settime failed, ret=%d", ret);
        return ret;
    }

    return 0;
}

/*********************************************************************
**函数名称:  my_get_system_time_sec
**入口参数:  无
**出口参数:  无
**函数功能:  获取当前系统时间（RTC实时时钟）
**返 回 值:  成功返回 >= 0 的 time_t（自1970-01-01 00:00:00 UTC起的秒数）
**           失败返回 (time_t)-1
*********************************************************************/
time_t my_get_system_time_sec(void)
{
    struct timespec ts;
    int ret;

    ret = sys_clock_gettime(SYS_CLOCK_REALTIME, &ts);
    if (ret < 0)
    {
        LOG_INF("clock_gettime failed, ret=%d", ret);
        return (time_t)-1;
    }

    return ts.tv_sec;
}

/*********************************************************************
**函数名称:  calculate_remaining_seconds
**入口参数:  start_time    --  起始上报时间，格式为HHMM（24小时制，如0001、2359）
**           interval_min   --  上报间隔，单位：分钟（必须大于0）
**           utc_timestamp  --  当前UTC时间戳，单位：秒（从1970-01-01 00:00:00 UTC起）
**出口参数:  无
**函数功能:  计算当前UTC时间到最近上报时间点的剩余秒数,支持跨天处理  
**返 回 值:  成功返回剩余秒数，失败返回-1（参数不合法）
*********************************************************************/
int calculate_remaining_seconds(const char *start_time, int interval_min, long long utc_timestamp)
{
    int ret_code;                /* 函数返回码（默认-1，失败） */
    int i;                       /* 循环遍历用临时变量 */
    int hour;                    /* 解析start_time得到的小时 */
    int minute;                  /* 解析start_time得到的分钟 */
    int base_seconds;            /* 起始时间转换为"当天的秒数"（0~86399） */
    int interval_seconds;        /* 上报间隔转换为秒数 */
    int current_seconds_of_day;  /* 当前UTC时间戳对应的"当天已过秒数" */
    int n;                       /* 从起始时间到当前时间的间隔数 */
    int next_point;              /* 下一个上报时间点的"当天秒数"（可能超过86400，代表跨天） */
    int remaining_seconds;       /* 最终要返回的剩余秒数 */

    /* 初始化返回码 */
    ret_code = -1;

    /* 1. 参数合法性校验：start_time非空且长度为4 */
    if (start_time == NULL || strlen(start_time) != 4)
    {
        LOG_INF("Error: start_time format is invalid, must be 4-digit string (HHMM)");
        return ret_code;
    }

    /* 2. 参数合法性校验：start_time全为数字 */
    for (i = 0; i < 4; i++)
    {
        if (!isdigit(start_time[i]))
        {
            LOG_INF("Error: start_time contains non-numeric characters");
            return ret_code;
        }
    }

    /* 3. 参数合法性校验：上报间隔为正整数 */
    if (interval_min <= 0)
    {
        LOG_INF("Error: reporting interval must be a positive integer (minutes)");
        return ret_code;
    }

    /* 4. 参数合法性校验：UTC时间戳非负 */
    if (utc_timestamp < 0)
    {
        LOG_INF("Error: UTC timestamp must be a non-negative integer (seconds)");
        return ret_code;
    }

    /* 5. 解析start_time为小时和分钟，并校验合法性 */
    hour = (start_time[0] - '0') * 10 + (start_time[1] - '0');
    minute = (start_time[2] - '0') * 10 + (start_time[3] - '0');
    if (hour < 0 || hour > 23 || minute < 0 || minute > 59)
    {
        LOG_INF("Error: hour (%d) or minute (%d) in start_time is out of range (hour:0-23, minute:0-59)", hour, minute);
        return ret_code;
    }

    /* 6. 转换起始时间为"当天的秒数"（如0001 → 1*60=60秒） */
    base_seconds = hour * 3600 + minute * 60;

    /* 7. 转换上报间隔为秒数（如600分钟 → 600*60=36000秒） */
    interval_seconds = interval_min * 60;

    /* 8. 计算当前UTC时间戳对应的"当天已过秒数"（取模86400） */
    current_seconds_of_day = (int)(utc_timestamp % DAY_SECONDS);

    /* 9. 计算最近的下一个上报时间点（修正跨天处理） */
    if (current_seconds_of_day < base_seconds)
    {
        /* 场景1：当前时间在起始时间之前 → 最近的就是起始时间 */
        remaining_seconds = base_seconds - current_seconds_of_day;
    } 
    else
    {
        /* 计算从起始时间到当前时间的间隔数（向下取整） */
        n = (current_seconds_of_day - base_seconds) / interval_seconds;
        /* 计算当天内的下一个上报点 */
        next_point = base_seconds + (n + 1) * interval_seconds;

        if (next_point < DAY_SECONDS)
        {
            /* 当天内有上报点 */
            remaining_seconds = (int)(next_point - current_seconds_of_day);
        } 
        else
        {
            /* 跨天：下一个上报点是次日的base_seconds */
            remaining_seconds = (int)((base_seconds + DAY_SECONDS) - current_seconds_of_day);
        }
    }
    LOG_INF("current_seconds_of_day:%d,base_seconds:%d,next_point:%d,remaining_seconds:%d",
        current_seconds_of_day,base_seconds,next_point,remaining_seconds);

    LOG_INF("interval_min:%d,interval_seconds:%d",
        interval_min,interval_seconds);

    /* 10. 函数执行成功，更新返回码为剩余秒数 */
    ret_code = remaining_seconds;

    return ret_code;
}

/*********************************************************************
**函数名称:  rand_0_to_120_seconds
**入口参数:  p_seconds  --  用于存储生成的随机秒数的指针
**出口参数:  p_seconds   --  输出0-120之间的随机秒数
**函数功能:  使用PSA加密库生成0-120秒范围内的随机数
**           用于上报时间随机偏移，防止多设备同时上传造成服务器压力
**返 回 值:  PSA_SUCCESS 表示成功，PSA_ERROR_INVALID_ARGUMENT 表示参数错误
*********************************************************************/
psa_status_t rand_0_to_120_seconds(uint32_t *p_seconds)
{
    psa_status_t status;
    uint8_t rnd[4];
    uint32_t val;

    if (p_seconds == NULL)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    /* 确保在系统初始化阶段已经调用过 psa_crypto_init() */
    status = psa_generate_random(rnd, sizeof(rnd));
    if (status != PSA_SUCCESS)
    {
        return status;
    }

    val = ((uint32_t)rnd[0] << 24) |
                   ((uint32_t)rnd[1] << 16) |
                   ((uint32_t)rnd[2] << 8)  |
                   ((uint32_t)rnd[3]);

    *p_seconds = val % 121U;   /* 0–120 */

    return PSA_SUCCESS;
}