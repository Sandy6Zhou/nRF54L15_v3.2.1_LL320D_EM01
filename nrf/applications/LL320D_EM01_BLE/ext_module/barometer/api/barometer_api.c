/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        barometer_api.c
**文件描述:        气压模块统一 API 接口实现文件
**当前版本:        V1.0
**作    者:       周森达 (zhousenda@jimiiot.com)
**完成日期:        2026.06.08
*********************************************************************
** 功能描述:       封装 SPA06-003 驱动，提供统一气压接口
*********************************************************************/

#include "barometer_api.h"
#include "../drivers/SPA06/inc/spa06_driver.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(barometer_api, LOG_LEVEL_INF);

/* 内部状态 */
static bool s_barometer_initialized = false;
static barometer_work_mode_t s_barometer_work_mode = BARO_MODE_STOP;

/********************************************************************
**函数名称:  barometer_convert_config_result
**入口参数:  ret      ---        底层驱动返回值
**出口参数:  无
**函数功能:  将底层配置接口返回值转换为 API 层错误码
**返 回 值:  BARO_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
static barometer_result_t barometer_convert_config_result(int ret)
{
    if (ret == 0)
    {
        return BARO_SUCCESS;
    }

    if (ret == -EINVAL)
    {
        return BARO_ERROR_PARAM;
    }

    if (ret == -ETIMEDOUT)
    {
        return BARO_ERROR_TIMEOUT;
    }

    return BARO_ERROR_COMM;
}

/********************************************************************
**函数名称:  barometer_convert_work_mode
**入口参数:  work_mode   ---        API 工作模式（输入）
**出口参数:  无
**函数功能:  将 API 工作模式转换为底层驱动工作模式
**返 回 值:  转换后的 SPA06 工作模式
*********************************************************************/
static spa06_work_mode_t barometer_convert_work_mode(barometer_work_mode_t work_mode)
{
    switch (work_mode)
    {
        case BARO_MODE_STOP:
            return SPA06_MODE_STOP;

        case BARO_MODE_SINGLE_TEMPERATURE:
            return SPA06_MODE_SINGLE_TEMPERATURE;

        case BARO_MODE_SINGLE_PRESSURE:
            return SPA06_MODE_SINGLE_PRESSURE;

        case BARO_MODE_SINGLE_BOTH:
            return SPA06_MODE_SINGLE_BOTH;

        case BARO_MODE_CONTINUOUS_TEMPERATURE:
            return SPA06_MODE_CONTINUOUS_TEMPERATURE;

        case BARO_MODE_CONTINUOUS_PRESSURE:
            return SPA06_MODE_CONTINUOUS_PRESSURE;

        case BARO_MODE_CONTINUOUS_BOTH:
            return SPA06_MODE_CONTINUOUS_BOTH;

        default:
            return SPA06_MODE_STOP;
    }
}

/********************************************************************
**函数名称:  barometer_init
**入口参数:  config   ---        采样配置参数（输入：采样率与过采样）
**出口参数:  无
**函数功能:  初始化气压模块，包括底层驱动、校准系数读取，并按传入配置
**           设定气压/温度采样率和过采样
**返 回 值:  BARO_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
barometer_result_t barometer_init(const struct barometer_config *config)
{
    int ret;
    spa06_config_t drv_config;

    if (config == NULL)
    {
        return BARO_ERROR_PARAM;
    }

    if (s_barometer_initialized)
    {
        LOG_WRN("Barometer already initialized");
        return BARO_SUCCESS;
    }

    drv_config.pressure_sample_rate_hz = config->pressure_sample_rate_hz;
    drv_config.pressure_oversampling = config->pressure_oversampling;
    drv_config.temperature_sample_rate_hz = config->temperature_sample_rate_hz;
    drv_config.temperature_oversampling = config->temperature_oversampling;

    ret = spa06_driver_init(&drv_config);
    if (ret == -ENODEV)
    {
        LOG_ERR("SPA06 driver init failed (chip ID): %d", ret);
        return BARO_ERROR_CHIP_ID;
    }
    else if (ret == -ETIMEDOUT)
    {
        LOG_ERR("SPA06 driver init failed (timeout): %d", ret);
        return BARO_ERROR_TIMEOUT;
    }
    else if (ret != 0)
    {
        LOG_ERR("SPA06 driver init failed: %d", ret);
        return BARO_ERROR_INIT;
    }

    s_barometer_initialized = true;
    s_barometer_work_mode = BARO_MODE_STOP;
    LOG_INF("Barometer API initialized");
    return BARO_SUCCESS;
}

/********************************************************************
**函数名称:  barometer_set_work_mode
**入口参数:  work_mode   ---        气压模块工作模式（输入）
**出口参数:  无
**函数功能:  设置气压模块工作模式
**返 回 值:  BARO_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
barometer_result_t barometer_set_work_mode(barometer_work_mode_t work_mode)
{
    int ret;
    spa06_work_mode_t drv_work_mode;

    if (!s_barometer_initialized)
    {
        return BARO_ERROR_INIT;
    }

    // 入参有效性校验：工作模式须落在合法枚举范围内（小于边界值 BARO_MODE_MAX）
    if (work_mode >= BARO_MODE_MAX)
    {
        return BARO_ERROR_PARAM;
    }

    drv_work_mode = barometer_convert_work_mode(work_mode);

    ret = spa06_driver_set_work_mode(drv_work_mode);
    if (ret < 0)
    {
        return barometer_convert_config_result(ret);
    }

    s_barometer_work_mode = work_mode;

    return BARO_SUCCESS;
}

/********************************************************************
**函数名称:  barometer_read
**入口参数:  data     ---        气压数据存储指针（输出）
**出口参数:  data     ---        包含气压和温度的结构体
**函数功能:  执行一次气压和温度测量
**返 回 值:  BARO_SUCCESS 表示成功，其他表示错误码
**注意事项:  根据对应模式只填充实际测量的字段
*********************************************************************/
barometer_result_t barometer_read(struct barometer_data *data)
{
    int ret;

    if (!s_barometer_initialized)
    {
        LOG_ERR("Barometer not initialized");
        return BARO_ERROR_INIT;
    }

    if (data == NULL)
    {
        return BARO_ERROR_PARAM;
    }

    if (s_barometer_work_mode == BARO_MODE_STOP)
    {
        return BARO_ERROR_PARAM;
    }

    ret = spa06_driver_read_measurement(&data->pressure_pa, &data->temperature);
    if (ret == -ETIMEDOUT)
    {
        return BARO_ERROR_TIMEOUT;
    }
    else if (ret < 0)
    {
        return BARO_ERROR_COMM;
    }

    return BARO_SUCCESS;
}

/********************************************************************
**函数名称:  barometer_get_chip_id
**入口参数:  id       ---        ID 存储指针（输出）
**出口参数:  id       ---        芯片 ID 值
**函数功能:  读取气压传感器芯片 ID
**返 回 值:  BARO_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
barometer_result_t barometer_get_chip_id(uint8_t *id)
{
    int ret;

    if (!s_barometer_initialized)
    {
        return BARO_ERROR_INIT;
    }

    if (id == NULL)
    {
        return BARO_ERROR_PARAM;
    }

    ret = spa06_driver_read_chip_id(id);
    if (ret < 0)
    {
        return BARO_ERROR_COMM;
    }

    return BARO_SUCCESS;
}
