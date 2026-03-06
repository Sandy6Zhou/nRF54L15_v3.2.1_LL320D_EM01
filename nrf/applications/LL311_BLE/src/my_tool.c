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

/************************************************************************
**函数名称:  my_generate_random
**入口参数:  out_val            ---       指向uint32_t类型变量的指针，用于存储生成的随机数
**出口参数:  psa_status_t       ---       PSA_SUCCESS表示随机数生成成功，其他值表示对应的错误码
**函数功能:  调用PSA加密API生成4字节随机数据，并将其拼接为一个32位无符号整数，存储到指定内存地址
*************************************************************************/
psa_status_t my_generate_random(uint32_t *out_val)
{
    uint8_t rnd[4];
    psa_status_t status;

    status = psa_generate_random(rnd, sizeof(rnd));
    if (status != PSA_SUCCESS) {
        return status;
    }

    *out_val = ((uint32_t)rnd[0] << 24) |
               ((uint32_t)rnd[1] << 16) |
               ((uint32_t)rnd[2] << 8)  |
               ((uint32_t)rnd[3]);

    return PSA_SUCCESS;
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
    uint32_t val;

    if (p_seconds == NULL)
    {
        return PSA_ERROR_INVALID_ARGUMENT;
    }

    status = my_generate_random(&val);
    if (status != PSA_SUCCESS)
    {
        return status;
    }

    *p_seconds = val % 121U;   /* 0–120 */

    return PSA_SUCCESS;
}

/************************************************************************
**函数名称:  my_get_str_at_pos
**入口参数:  szInput            ---       输入的原始字符串
**           iPos               ---       要提取的字段在分隔符分割后的位置（从0开始）
**           cSplit             ---       字段分隔符（如',' '|'等）
**           szOutBuf           ---       输出缓冲区，存放提取到的字符串
**           iBuffLen           ---       输出缓冲区长度（需预留'\0'空间）
**出口参数:  bool               ---       true=提取后还有后续字段，false=无后续字段
**函数功能:  按指定分隔符分割字符串，提取第iPos个位置的字段到输出缓冲区，防缓冲区溢出
*************************************************************************/
bool my_get_str_at_pos(char *szInput, uint16_t iPos, char cSplit, char *szOutBuf, uint16_t iBuffLen)
{
    uint16_t i = 0;
    char *c = szInput;
    char *p = szOutBuf;
    bool bHasMore = 0;

    if (szInput == NULL)
        return 0;

    while (*c != '\0')
    {
        if (i == iPos && *c != cSplit)
        {
            *p++ = *c;

            // 防止溢出，后面还要往这个 szBuf[iBuffLen-1] 放入结尾符
            if (p >= (szOutBuf + iBuffLen - 1))
                break;
        }
        else if (i > iPos)
        {
            bHasMore = 1;
            break;
        }

        if (*c == cSplit)
        {
            i++;
        }

        c++;
    }

    *p = '\0';

    return bHasMore;
}

/********************************************************************
**函数名称:  string_check_is_hex_str
**入口参数:  str: 输入的字符串
**出口参数:  无
**函数功能:  检测字符串是不是全是十六进制数据组成(0~9,a,b,c,d,e,f,A,B,C,D,E,F)
**返 回 值:  返回字符串的长度,0表示错误
*********************************************************************/
uint8_t string_check_is_hex_str(const char* str)
{
    uint8_t no_count = 0;

    if (str == NULL)
    {
        return 0;
    }

    while (*str)
    {
        if ((*str >= '0' && *str <= '9') ||
            (*str >= 'a' && *str <= 'f') ||
            (*str >= 'A' && *str <= 'F'))
        {
            no_count++;
            str++;
        }
        else
        {
            return 0;
        }
    }
    return no_count;
}

/********************************************************************
**函数名称:  hexstr_to_hex
**入口参数:  dest: 目标数组, dest_size: 目标数组长度, src: 原始字符串
**出口参数:  无
**函数功能:  将十六进制字符串转换成十六进制数组
**返 回 值:  返回十六进制数组长度,0表示错误
*********************************************************************/
uint8_t hexstr_to_hex(uint8_t *dest, uint8_t dest_size, const char *src)
{
    uint8_t offset = 0;
    uint8_t temp_data = 0;
    uint8_t hex_data = 0;
    uint8_t byte_count = 0;

    if (dest == NULL || src == NULL || dest_size == 0)
    {
        return 0;
    }

    while(*src)
    {
        if (*src >= '0' && *src <= '9')
        {
            temp_data = (uint8_t)(*src - '0');
        }
        else if (*src >= 'A' && *src <= 'F')
        {
            temp_data = (uint8_t)(*src - 'A' + 0x0A);
        }
        else if (*src >= 'a' && *src <= 'f')
        {
            temp_data = (uint8_t)(*src - 'a' + 0x0A);
        }
        else
        {
            return 0;
        }


        if (offset % 2)
        {
            hex_data |= (temp_data & 0x0F);
            dest[byte_count] = hex_data;
            byte_count++;

            // 检查缓冲区边界
            if (byte_count >= dest_size) {
                break;
            }
        }
        else
        {
            hex_data = ((temp_data << 4) & 0xf0);
        }

        offset++;
        src++;
    }

    return ((offset / 2) + (offset % 2));
}

/********************************************************************
**函数名称:  hex2ascii
**入口参数:  digit: 要转换的十六进制数字(0-15)
**出口参数:  无
**函数功能:  将十六进制数字转换成ASCII字符
**返 回 值:  返回对应的ASCII字符
*********************************************************************/
uint8_t hex2ascii(uint8_t digit)
{
    uint8_t val;

    if (digit <= 9) {
        val = digit - 0x0 + '0';
    } else {
        val = digit - 0xA + 'A';
    }

    return val;
}

/********************************************************************
**函数名称:  hex2hexstr
**入口参数:  hex: 十六进制数组, hex_len: 十六进制数组长度, str: 十六进制字符串, str_len: 十六进制字符串长度
**出口参数:  无
**函数功能:  将十六进制数组转换成十六进制字符串
**返 回 值:  无
*********************************************************************/
void hex2hexstr(uint8_t *hex, uint16_t hex_len, uint8_t *str, uint16_t str_len)
{
    uint16_t i = 0,j=0;
    uint8_t *buf = str;

    if (str_len < 2*hex_len) {
        LOG_INF("str_len is too small.");
        return;
    }

    while(hex_len--)
    {

        buf[j++] = hex2ascii((hex[i] >> 4) & 0x0f);
        buf[j++] = hex2ascii(hex[i] & 0x0f);
        i++;
    }
    buf[j] = 0;
}

/********************************************************************
**函数名称:  string_check_is_number
**入口参数:  flag: flag & 1 允许字符串中包含'+'或'-', flag & 2 允许字符串中包含'.', flag & 4 数字不允许大于7或小于1, str: 传入的字符串
**出口参数:  无
**函数功能:  检测字符串是不是全是数字组成
**返 回 值:  返回有效的字符数
*********************************************************************/
uint8_t string_check_is_number(uint8_t flag, const char* str)
{
    uint8_t no_count = 0;
    while (*str)
    {
        if ((flag & 1) && no_count == 0 && (*str == '+' || *str == '-'))
        {
            no_count++;
            str++;
        }
        else if ((flag & 2) && *str == '.')
        {
            flag &= ~2;
            no_count++;
            str++;
        }
        else if (*str >= '0' && *str <= '9')
        {
            no_count++;
            str++;
        }
        else
        {
            return 0;
        }
    }
    return no_count;
}

/********************************************************************
**函数名称:  HexChar2HexData
**入口参数:  c: 输入的字符
**出口参数:  无
**函数功能:  将十六进制字符转换成十六进制数据
**返 回 值:  返回对应的十六进制值,0xFF表示无效字符
*********************************************************************/
static uint8_t HexChar2HexData(char c)
{
    uint8_t hex = 0x00;

    if (c >= '0' && c <= '9')
        hex = (uint8_t)(c - '0');
    else if (c >= 'A' && c <= 'F')
        hex = (uint8_t)(c-'A'+0x0A);
    else if (c >= 'a' && c <= 'f')
        hex = (uint8_t)(c-'a'+0x0A);
    else
        hex = 0xFF;

    return hex;
}

/********************************************************************
**函数名称:  macstr_to_hex
**入口参数:  mac_str: MAC地址字符串, hex: 存储十六进制数据的指针
**出口参数:  无
**函数功能:  将MAC地址字符串转换为十六进制数据
**返 回 值:  转换成功返回true,失败返回false
*********************************************************************/
bool macstr_to_hex(const char *mac_str, uint8_t *hex)
{
    uint16_t len;
    char mystr[20];
    uint8_t *hex_p;
    uint16_t offset = 0;
    uint8_t hex_data_count = 0;
    uint8_t hex_H = 0x00;
    uint8_t hex_L = 0x00;

    if (mac_str == NULL)
        return false;

    len = strlen(mac_str);

    if (len == 12 || len == 17)
    {
        hex_p = hex;

        memset(mystr,0,sizeof mystr);
        strcpy(mystr, (const char*)mac_str);
        while(offset < len)
        {
            if (mystr[offset] == ':')
            {
                if (offset      == 2
                    || offset   == 5
                    || offset   == 8
                    || offset   == 11
                    || offset   == 14
                )
                {
                    offset++;
                }
                else
                    return false;
            }
            else
            {
                hex_H = 0x00;
                hex_L = 0x00;

                hex_H = HexChar2HexData(mystr[offset++]);
                hex_L = HexChar2HexData(mystr[offset++]);
                if (hex_H == 0xFF || hex_L == 0xFF)
                    return false;

                hex_data_count++;
                if (hex_data_count > 6)
                    return false;

                *hex_p++ = ((hex_H << 4) & 0xF0) + (hex_L & 0x0F);
             }
        }
        return true;
    }

    return false;
}

/********************************************************************
**函数名称:  char_array_reverse
**入口参数:  src: 输入的源数组, dest: 输出的逆序目标数组, src_len: src数组长度, dest_len: dest数组长度
**出口参数:  无
**函数功能:  字符数组整体逆序（无字符串终止符，纯字节操作）
**返 回 值:  0: 成功; -1: 入参无效（空指针/长度为0）; -2: src/dest长度不一致;
*********************************************************************/
int char_array_reverse(const uint8_t *src, uint32_t src_len, uint8_t *dest, uint32_t dest_len)
{
    // 入参合法性检查（空指针/长度为0）
    if (src == NULL || dest == NULL || src_len == 0 || dest_len == 0) {
        return -1; // 入参为空或长度为0
    }

    // src和dest长度必须完全一致
    if (src_len != dest_len) {
        return -2; // src/dest长度不一致，直接返回
    }

    // 纯字节操作，无字符串终止符
    for (uint32_t i = 0; i < src_len; i++) {
        dest[i] = src[src_len - 1 - i];
    }

    return 0;
}