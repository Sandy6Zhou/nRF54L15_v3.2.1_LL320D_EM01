/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        barometer_api.c
**文件描述:        气压模块统一 API 接口实现文件
**当前版本:        V1.0
**作    者:       周森达 (zhousenda@jimiiot.com)
**完成日期:        2026.06.08
*********************************************************************
** 功能描述:       封装底层气压驱动（SPA06/SPL16），提供统一气压接口
*********************************************************************/

#include "barometer_api.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(barometer_api, LOG_LEVEL_INF);

/* 底层驱动切换：通过编译期宏 BARO_USE_SPL16 选择 SPL16 或 SPA06 驱动 */
#ifdef BARO_USE_SPL16
#include "../drivers/SPL16/inc/spl16_driver.h"

#define BARO_DRV_CONFIG_T               spl16_config_t
#define BARO_DRV_WORK_MODE_T            spl16_work_mode_t
#define BARO_DRV_NAME                   "SPL16"

#define BARO_DRV_INIT(...)              spl16_driver_init(__VA_ARGS__)
#define BARO_DRV_SET_WORK_MODE(...)     spl16_driver_set_work_mode(__VA_ARGS__)
#define BARO_DRV_READ_MEASUREMENT(...)  spl16_driver_read_measurement(__VA_ARGS__)
#define BARO_DRV_READ_CHIP_ID(...)      spl16_driver_read_chip_id(__VA_ARGS__)

#define BARO_DRV_MODE_STOP                      SPL16_MODE_STOP
#define BARO_DRV_MODE_SINGLE_TEMPERATURE        SPL16_MODE_SINGLE_TEMPERATURE
#define BARO_DRV_MODE_SINGLE_PRESSURE           SPL16_MODE_SINGLE_PRESSURE
#define BARO_DRV_MODE_SINGLE_BOTH               SPL16_MODE_SINGLE_BOTH
#define BARO_DRV_MODE_CONTINUOUS_TEMPERATURE    SPL16_MODE_CONTINUOUS_TEMPERATURE
#define BARO_DRV_MODE_CONTINUOUS_PRESSURE       SPL16_MODE_CONTINUOUS_PRESSURE
#define BARO_DRV_MODE_CONTINUOUS_BOTH           SPL16_MODE_CONTINUOUS_BOTH
#else
#include "../drivers/SPA06/inc/spa06_driver.h"

#define BARO_DRV_CONFIG_T               spa06_config_t
#define BARO_DRV_WORK_MODE_T            spa06_work_mode_t
#define BARO_DRV_NAME                   "SPA06"

#define BARO_DRV_INIT(...)              spa06_driver_init(__VA_ARGS__)
#define BARO_DRV_SET_WORK_MODE(...)     spa06_driver_set_work_mode(__VA_ARGS__)
#define BARO_DRV_READ_MEASUREMENT(...)  spa06_driver_read_measurement(__VA_ARGS__)
#define BARO_DRV_READ_CHIP_ID(...)      spa06_driver_read_chip_id(__VA_ARGS__)

#define BARO_DRV_MODE_STOP                      SPA06_MODE_STOP
#define BARO_DRV_MODE_SINGLE_TEMPERATURE        SPA06_MODE_SINGLE_TEMPERATURE
#define BARO_DRV_MODE_SINGLE_PRESSURE           SPA06_MODE_SINGLE_PRESSURE
#define BARO_DRV_MODE_SINGLE_BOTH               SPA06_MODE_SINGLE_BOTH
#define BARO_DRV_MODE_CONTINUOUS_TEMPERATURE    SPA06_MODE_CONTINUOUS_TEMPERATURE
#define BARO_DRV_MODE_CONTINUOUS_PRESSURE       SPA06_MODE_CONTINUOUS_PRESSURE
#define BARO_DRV_MODE_CONTINUOUS_BOTH           SPA06_MODE_CONTINUOUS_BOTH
#endif

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
**返 回 值:  转换后的底层驱动工作模式
*********************************************************************/
static BARO_DRV_WORK_MODE_T barometer_convert_work_mode(barometer_work_mode_t work_mode)
{
    switch (work_mode)
    {
        case BARO_MODE_STOP:
            return BARO_DRV_MODE_STOP;

        case BARO_MODE_SINGLE_TEMPERATURE:
            return BARO_DRV_MODE_SINGLE_TEMPERATURE;

        case BARO_MODE_SINGLE_PRESSURE:
            return BARO_DRV_MODE_SINGLE_PRESSURE;

        case BARO_MODE_SINGLE_BOTH:
            return BARO_DRV_MODE_SINGLE_BOTH;

        case BARO_MODE_CONTINUOUS_TEMPERATURE:
            return BARO_DRV_MODE_CONTINUOUS_TEMPERATURE;

        case BARO_MODE_CONTINUOUS_PRESSURE:
            return BARO_DRV_MODE_CONTINUOUS_PRESSURE;

        case BARO_MODE_CONTINUOUS_BOTH:
            return BARO_DRV_MODE_CONTINUOUS_BOTH;

        default:
            return BARO_DRV_MODE_STOP;
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
    BARO_DRV_CONFIG_T drv_config;

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

    ret = BARO_DRV_INIT(&drv_config);
    if (ret == -ENODEV)
    {
        LOG_ERR("%s driver init failed (chip ID): %d", BARO_DRV_NAME, ret);
        return BARO_ERROR_CHIP_ID;
    }
    else if (ret == -ETIMEDOUT)
    {
        LOG_ERR("%s driver init failed (timeout): %d", BARO_DRV_NAME, ret);
        return BARO_ERROR_TIMEOUT;
    }
    else if (ret != 0)
    {
        LOG_ERR("%s driver init failed: %d", BARO_DRV_NAME, ret);
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
    BARO_DRV_WORK_MODE_T drv_work_mode;

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

    ret = BARO_DRV_SET_WORK_MODE(drv_work_mode);
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

    ret = BARO_DRV_READ_MEASUREMENT(&data->pressure_pa, &data->temperature);
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

    ret = BARO_DRV_READ_CHIP_ID(id);
    if (ret < 0)
    {
        return BARO_ERROR_COMM;
    }

    return BARO_SUCCESS;
}
