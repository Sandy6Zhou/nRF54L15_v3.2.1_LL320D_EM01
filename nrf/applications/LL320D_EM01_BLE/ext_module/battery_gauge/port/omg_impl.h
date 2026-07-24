#ifndef _OMG_IMPL_H_
#define _OMG_IMPL_H_

#include "omg_battery.h"
#include "om70201wv_port.h"

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(battery_gauge_api);

/********************************************************************
**函数名称:  _omg_read_byte
**入口参数:  addr     ---        寄存器地址（输入）
            ret_data ---        数据缓冲区（输入）
**出口参数:  ret_data ---        读取到的字节数据（输出）
**函数功能:  为厂家驱动提供单字节寄存器读取接口
**返回值:    OMG_ERROR_NONE 表示成功，其他表示失败
*********************************************************************/
static int _omg_read_byte(const omg_uint8_t addr, omg_uint8_t *ret_data)
{
    int ret;

    ret = om70201wv_port_read(addr, ret_data, 1U);
    if (ret < 0)
    {
        return OMG_ERROR_IIC;
    }

    return OMG_ERROR_NONE;
}

/********************************************************************
**函数名称:  _omg_read_word
**入口参数:  addr     ---        寄存器高字节地址（输入）
            ret_data ---        数据缓冲区（输入）
**出口参数:  ret_data ---        读取到的双字节数据（输出）
**函数功能:  为厂家驱动提供大端双字节寄存器读取接口
**返回值:    OMG_ERROR_NONE 表示成功，其他表示失败
*********************************************************************/
static int _omg_read_word(const omg_uint8_t addr, omg_uint16_t *ret_data)
{
    int ret;
    omg_uint8_t data[2];

    ret = om70201wv_port_read(addr, data, sizeof(data));
    if (ret < 0)
    {
        return OMG_ERROR_IIC;
    }

    *ret_data = ((omg_uint16_t)data[0] << 8) | data[1];

    return OMG_ERROR_NONE;
}

/********************************************************************
**函数名称:  _omg_write_byte
**入口参数:  addr ---        寄存器地址（输入）
            data ---        待写入字节（输入）
**出口参数:  无
**函数功能:  为厂家驱动提供单字节寄存器写入接口
**返回值:    OMG_ERROR_NONE 表示成功，其他表示失败
*********************************************************************/
static int _omg_write_byte(const omg_uint8_t addr, const omg_uint8_t data)
{
    int ret;

    ret = om70201wv_port_write(addr, &data, 1U);
    if (ret < 0)
    {
        return OMG_ERROR_IIC;
    }

    return OMG_ERROR_NONE;
}

/********************************************************************
**函数名称:  _omg_write_word
**入口参数:  addr ---        寄存器高字节地址（输入）
            data ---        待写入双字节数据（输入）
**出口参数:  无
**函数功能:  为厂家驱动提供大端双字节寄存器写入接口
**返回值:    OMG_ERROR_NONE 表示成功，其他表示失败
*********************************************************************/
static int _omg_write_word(const omg_uint8_t addr, const omg_uint16_t data)
{
    int ret;
    omg_uint8_t write_data[2];

    write_data[0] = (omg_uint8_t)((data >> 8) & 0xFFU);
    write_data[1] = (omg_uint8_t)(data & 0xFFU);

    ret = om70201wv_port_write(addr, write_data, sizeof(write_data));
    if (ret < 0)
    {
        return OMG_ERROR_IIC;
    }

    return OMG_ERROR_NONE;
}

/********************************************************************
**函数名称:  _omg_delay_ms
**入口参数:  ms ---        延时时间，单位毫秒（输入）
**出口参数:  无
**函数功能:  为厂家驱动提供毫秒延时接口
**返回值:    无
*********************************************************************/
static void _omg_delay_ms(const omg_uint32_t ms)
{
    om70201wv_port_delay_ms(ms);
}

#if defined(OMG_ENABLE_SOC_COMPENSATION_RELAX_ANTI) && \
    (OMG_ENABLE_SOC_COMPENSATION_RELAX_ANTI == 1)
static omg_bool_t _omg_get_charge_status(void)
{
    return omg_false;
}
#endif

#if OMG_ENABLE_LOG_FEATURE
#define _omg_log_error(format, ...) LOG_ERR(format, ##__VA_ARGS__)
#define _omg_log_warn(format, ...) LOG_WRN(format, ##__VA_ARGS__)
#define _omg_log_info(format, ...) LOG_INF(format, ##__VA_ARGS__)
#define _omg_log_debug(format, ...) LOG_DBG(format, ##__VA_ARGS__)
#else
#define _omg_log_error(format, ...) do { } while (0)
#define _omg_log_warn(format, ...) do { } while (0)
#define _omg_log_info(format, ...) do { } while (0)
#define _omg_log_debug(format, ...) do { } while (0)
#endif

#endif
