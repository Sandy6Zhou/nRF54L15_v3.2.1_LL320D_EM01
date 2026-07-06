/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        accelerometer_api.c
**文件描述:        加速度传感器模块统一 API 接口实现文件
**当前版本:        V1.0
**作    者:       周森达 (zhousenda@jimiiot.com)
**完成日期:        2026.07.06
*********************************************************************
** 功能描述:       封装 DA213 驱动，提供统一加速度传感器接口
*********************************************************************/

#include "accelerometer_api.h"
#include "../drivers/DA213/inc/da213_driver.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(accelerometer_api, LOG_LEVEL_INF);

/* 内部状态 */
static bool s_accel_initialized = false;
static accel_range_t s_current_range = ACCEL_RANGE_2G;

/********************************************************************
**函数名称:  accel_convert_result
**入口参数:  ret         ---        驱动层返回值（输入）
**出口参数:  无
**函数功能:  将驱动层 errno 返回值转换为 API 层结果枚举
**返回值:    对应的 accel_result_t 值
*********************************************************************/
static accel_result_t accel_convert_result(int ret)
{
    if (ret == 0)
    {
        return ACCEL_SUCCESS;
    }

    if (ret == -EINVAL)
    {
        return ACCEL_ERROR_PARAM;
    }

    if (ret == -ENODEV)
    {
        return ACCEL_ERROR_CHIP_ID;
    }

    return ACCEL_ERROR_COMM;
}

/********************************************************************
**函数名称:  accel_convert_range
**入口参数:  range       ---        API层量程枚举（输入）
**出口参数:  无
**函数功能:  将 API 层量程枚举转换为驱动层量程枚举
**返回值:    对应的 da213_range_t 值
*********************************************************************/
static da213_range_t accel_convert_range(accel_range_t range)
{
    switch (range)
    {
        case ACCEL_RANGE_2G:
            return DA213_RANGE_2G;

        case ACCEL_RANGE_4G:
            return DA213_RANGE_4G;

        case ACCEL_RANGE_8G:
            return DA213_RANGE_8G;

        case ACCEL_RANGE_16G:
            return DA213_RANGE_16G;

        default:
            return DA213_RANGE_2G;
    }
}

/********************************************************************
**函数名称:  accel_convert_odr
**入口参数:  odr         ---        API层ODR枚举（输入）
**出口参数:  无
**函数功能:  将 API 层 ODR 枚举转换为驱动层 ODR 枚举
**返回值:    对应的 da213_odr_t 值
*********************************************************************/
static da213_odr_t accel_convert_odr(accel_odr_t odr)
{
    switch (odr)
    {
        case ACCEL_ODR_1HZ:
            return DA213_ODR_1HZ;

        case ACCEL_ODR_1_95HZ:
            return DA213_ODR_1_95HZ;

        case ACCEL_ODR_3_9HZ:
            return DA213_ODR_3_9HZ;

        case ACCEL_ODR_7_81HZ:
            return DA213_ODR_7_81HZ;

        case ACCEL_ODR_15_63HZ:
            return DA213_ODR_15_63HZ;

        case ACCEL_ODR_31_25HZ:
            return DA213_ODR_31_25HZ;

        case ACCEL_ODR_62_5HZ:
            return DA213_ODR_62_5HZ;

        case ACCEL_ODR_125HZ:
            return DA213_ODR_125HZ;

        case ACCEL_ODR_250HZ:
            return DA213_ODR_250HZ;

        case ACCEL_ODR_500HZ:
            return DA213_ODR_500HZ;

        case ACCEL_ODR_1000HZ:
            return DA213_ODR_1000HZ;

        default:
            return DA213_ODR_125HZ;
    }
}

/********************************************************************
**函数名称:  accel_convert_power_mode
**入口参数:  mode        ---        API层电源模式枚举（输入）
**出口参数:  无
**函数功能:  将 API 层电源模式转换为驱动层电源模式
**返回值:    对应的 da213_power_mode_t 值
*********************************************************************/
static da213_power_mode_t accel_convert_power_mode(accel_power_mode_t mode)
{
    switch (mode)
    {
        case ACCEL_POWER_NORMAL:
            return DA213_MODE_NORMAL;

        case ACCEL_POWER_LOW_POWER:
            return DA213_MODE_LOW_POWER;

        case ACCEL_POWER_SUSPEND:
            return DA213_MODE_SUSPEND;

        default:
            return DA213_MODE_NORMAL;
    }
}

/********************************************************************
**函数名称:  accel_get_sensitivity
**入口参数:  无
**出口参数:  无
**函数功能:  根据当前量程获取灵敏度（LSB/g）
**返回值:    灵敏度值
*********************************************************************/
static int32_t accel_get_sensitivity(void)
{
    switch (s_current_range)
    {
        case ACCEL_RANGE_2G:
            return 4096;

        case ACCEL_RANGE_4G:
            return 2048;

        case ACCEL_RANGE_8G:
            return 1024;

        case ACCEL_RANGE_16G:
            return 512;

        default:
            return 4096;
    }
}

/********************************************************************
**函数名称:  accelerometer_init
**入口参数:  config      ---        初始化配置参数（输入）
**出口参数:  无
**函数功能:  初始化加速度传感器模块
**返回值:    ACCEL_SUCCESS 成功，其他值失败
*********************************************************************/
accel_result_t accelerometer_init(const struct accel_config *config)
{
    int ret;
    da213_config_t drv_config;

    if (config == NULL)
    {
        return ACCEL_ERROR_PARAM;
    }

    if (config->range >= ACCEL_RANGE_MAX || config->odr >= ACCEL_ODR_MAX ||
        config->power_mode >= ACCEL_POWER_MAX)
    {
        return ACCEL_ERROR_PARAM;
    }

    if (s_accel_initialized)
    {
        LOG_WRN("Accelerometer already initialized");
        return ACCEL_SUCCESS;
    }

    drv_config.range = accel_convert_range(config->range);
    drv_config.resolution = DA213_RESOLUTION_14BIT;
    drv_config.odr = accel_convert_odr(config->odr);
    drv_config.power_mode = accel_convert_power_mode(config->power_mode);
    drv_config.lp_bandwidth = DA213_LP_BW_62_5HZ;

    ret = da213_driver_init(&drv_config);
    if (ret == -ENODEV)
    {
        LOG_ERR("Accelerometer init failed (chip ID): %d", ret);
        return ACCEL_ERROR_CHIP_ID;
    }
    else if (ret != 0)
    {
        LOG_ERR("Accelerometer init failed: %d", ret);
        return ACCEL_ERROR_INIT;
    }

    /* DA213 的方向识别中断在非锁存模式下脉冲较短，统一配置临时锁存以提高捕获稳定性 */
    ret = da213_driver_set_int_latch(DA213_LATCH_25MS);
    if (ret != 0)
    {
        LOG_ERR("Accelerometer set int latch failed: %d", ret);
        return ACCEL_ERROR_INIT;
    }

    s_current_range = config->range;
    s_accel_initialized = true;
    LOG_INF("Accelerometer API initialized");

    return ACCEL_SUCCESS;
}

/********************************************************************
**函数名称:  accelerometer_set_power_mode
**入口参数:  mode        ---        电源模式（输入）
**出口参数:  无
**函数功能:  设置加速度传感器电源模式
**返回值:    ACCEL_SUCCESS 成功，其他值失败
*********************************************************************/
accel_result_t accelerometer_set_power_mode(accel_power_mode_t mode)
{
    int ret;

    if (!s_accel_initialized)
    {
        return ACCEL_ERROR_INIT;
    }

    if (mode >= ACCEL_POWER_MAX)
    {
        return ACCEL_ERROR_PARAM;
    }

    ret = da213_driver_set_power_mode(accel_convert_power_mode(mode));

    return accel_convert_result(ret);
}

/********************************************************************
**函数名称:  accelerometer_set_range
**入口参数:  range       ---        量程（输入）
**出口参数:  无
**函数功能:  设置加速度传感器量程
**返回值:    ACCEL_SUCCESS 成功，其他值失败
*********************************************************************/
accel_result_t accelerometer_set_range(accel_range_t range)
{
    int ret;

    if (!s_accel_initialized)
    {
        return ACCEL_ERROR_INIT;
    }

    if (range >= ACCEL_RANGE_MAX)
    {
        return ACCEL_ERROR_PARAM;
    }

    ret = da213_driver_set_range(accel_convert_range(range));
    if (ret == 0)
    {
        s_current_range = range;
    }

    return accel_convert_result(ret);
}

/********************************************************************
**函数名称:  accelerometer_set_odr
**入口参数:  odr         ---        输出数据率（输入）
**出口参数:  无
**函数功能:  设置加速度传感器输出数据率
**返回值:    ACCEL_SUCCESS 成功，其他值失败
*********************************************************************/
accel_result_t accelerometer_set_odr(accel_odr_t odr)
{
    int ret;

    if (!s_accel_initialized)
    {
        return ACCEL_ERROR_INIT;
    }

    if (odr >= ACCEL_ODR_MAX)
    {
        return ACCEL_ERROR_PARAM;
    }

    ret = da213_driver_set_odr(accel_convert_odr(odr));

    return accel_convert_result(ret);
}

/********************************************************************
**函数名称:  accelerometer_get_chip_id
**入口参数:  id          ---        芯片ID存储指针（输出）
**出口参数:  id          ---        存储读取到的芯片ID
**函数功能:  读取加速度传感器芯片 ID
**返回值:    ACCEL_SUCCESS 成功，其他值失败
*********************************************************************/
accel_result_t accelerometer_get_chip_id(uint8_t *id)
{
    int ret;

    if (!s_accel_initialized)
    {
        return ACCEL_ERROR_INIT;
    }

    if (id == NULL)
    {
        return ACCEL_ERROR_PARAM;
    }

    ret = da213_driver_read_chip_id(id);

    return accel_convert_result(ret);
}

/********************************************************************
**函数名称:  accelerometer_read
**入口参数:  data        ---        加速度数据存储指针（输出）
**出口参数:  data        ---        存储转换后的三轴加速度值（mg单位）
**函数功能:  读取三轴加速度数据并转换为 mg 单位
**返回值:    ACCEL_SUCCESS 成功，其他值失败
*********************************************************************/
accel_result_t accelerometer_read(struct accel_data *data)
{
    int ret;
    da213_raw_data_t raw;
    int32_t sensitivity;

    if (!s_accel_initialized)
    {
        return ACCEL_ERROR_INIT;
    }

    if (data == NULL)
    {
        return ACCEL_ERROR_PARAM;
    }

    ret = da213_driver_read_raw_data(&raw);
    if (ret < 0)
    {
        return ACCEL_ERROR_COMM;
    }

    /* 将原始数据转换为 mg：mg = raw * 1000 / sensitivity，raw 为当前量程与分辨率下的有效输出值 */
    sensitivity = accel_get_sensitivity();
    data->x_mg = ((int32_t)raw.x * 1000) / sensitivity;
    data->y_mg = ((int32_t)raw.y * 1000) / sensitivity;
    data->z_mg = ((int32_t)raw.z * 1000) / sensitivity;

    return ACCEL_SUCCESS;
}

/********************************************************************
**函数名称:  accelerometer_read_raw
**入口参数:  data        ---        原始数据存储指针（输出）
**出口参数:  data        ---        存储三轴原始加速度值
**函数功能:  读取三轴加速度原始数据
**返回值:    ACCEL_SUCCESS 成功，其他值失败
*********************************************************************/
accel_result_t accelerometer_read_raw(struct accel_raw_data *data)
{
    int ret;
    da213_raw_data_t raw;

    if (!s_accel_initialized)
    {
        return ACCEL_ERROR_INIT;
    }

    if (data == NULL)
    {
        return ACCEL_ERROR_PARAM;
    }

    ret = da213_driver_read_raw_data(&raw);
    if (ret < 0)
    {
        return ACCEL_ERROR_COMM;
    }

    data->x = raw.x;
    data->y = raw.y;
    data->z = raw.z;

    return ACCEL_SUCCESS;
}

/********************************************************************
**函数名称:  accelerometer_config_active_int
**入口参数:  config      ---        运动检测中断配置（输入）
**出口参数:  无
**函数功能:  配置运动检测（Active）中断
**返回值:    ACCEL_SUCCESS 成功，其他值失败
*********************************************************************/
accel_result_t accelerometer_config_active_int(const struct accel_active_int_config *config)
{
    int ret;
    da213_active_int_config_t drv_config;

    if (!s_accel_initialized)
    {
        return ACCEL_ERROR_INIT;
    }

    if (config == NULL)
    {
        return ACCEL_ERROR_PARAM;
    }

    drv_config.threshold = config->threshold;
    drv_config.duration = config->duration;
    drv_config.enable_x = config->enable_x;
    drv_config.enable_y = config->enable_y;
    drv_config.enable_z = config->enable_z;
    drv_config.int_pin = DA213_INT_PIN_1;

    ret = da213_driver_config_active_int(&drv_config);

    return accel_convert_result(ret);
}

/********************************************************************
**函数名称:  accelerometer_config_tap_int
**入口参数:  config      ---        敲击检测中断配置（输入）
**出口参数:  无
**函数功能:  配置敲击检测（Tap）中断
**返回值:    ACCEL_SUCCESS 成功，其他值失败
*********************************************************************/
accel_result_t accelerometer_config_tap_int(const struct accel_tap_int_config *config)
{
    int ret;
    da213_tap_int_config_t drv_config;

    if (!s_accel_initialized)
    {
        return ACCEL_ERROR_INIT;
    }

    if (config == NULL)
    {
        return ACCEL_ERROR_PARAM;
    }

    drv_config.threshold = config->threshold;
    drv_config.quiet = config->quiet;
    drv_config.shock = config->shock;
    drv_config.duration = config->duration;
    drv_config.enable_single = config->enable_single;
    drv_config.enable_double = config->enable_double;
    drv_config.int_pin = DA213_INT_PIN_1;

    ret = da213_driver_config_tap_int(&drv_config);

    return accel_convert_result(ret);
}

/********************************************************************
**函数名称:  accelerometer_config_freefall_int
**入口参数:  config      ---        自由落体中断配置（输入）
**出口参数:  无
**函数功能:  配置自由落体（Freefall）中断
**返回值:    ACCEL_SUCCESS 成功，其他值失败
*********************************************************************/
accel_result_t accelerometer_config_freefall_int(const struct accel_freefall_int_config *config)
{
    int ret;
    da213_freefall_int_config_t drv_config;

    if (!s_accel_initialized)
    {
        return ACCEL_ERROR_INIT;
    }

    if (config == NULL)
    {
        return ACCEL_ERROR_PARAM;
    }

    drv_config.threshold = config->threshold;
    drv_config.duration = config->duration;
    drv_config.hysteresis = config->hysteresis;
    drv_config.mode = config->sum_mode ? DA213_FREEFALL_SUM : DA213_FREEFALL_SINGLE;
    drv_config.int_pin = DA213_INT_PIN_1;

    ret = da213_driver_config_freefall_int(&drv_config);

    return accel_convert_result(ret);
}

/********************************************************************
**函数名称:  accelerometer_config_orient_int
**入口参数:  config      ---        方向识别中断配置（输入）
**出口参数:  无
**函数功能:  配置方向识别（Orient）中断
**返回值:    ACCEL_SUCCESS 成功，其他值失败
*********************************************************************/
accel_result_t accelerometer_config_orient_int(const struct accel_orient_int_config *config)
{
    int ret;
    da213_orient_int_config_t drv_config;

    if (!s_accel_initialized)
    {
        return ACCEL_ERROR_INIT;
    }

    if (config == NULL)
    {
        return ACCEL_ERROR_PARAM;
    }

    drv_config.mode = (da213_orient_mode_t)config->mode;
    drv_config.blocking = (da213_orient_block_t)config->blocking;
    drv_config.hysteresis = config->hysteresis;
    drv_config.z_blocking = config->z_blocking;
    drv_config.int_pin = DA213_INT_PIN_1;

    ret = da213_driver_config_orient_int(&drv_config);

    return accel_convert_result(ret);
}

/********************************************************************
**函数名称:  accelerometer_register_int_callback
**入口参数:  callback    ---        加速度传感器中断回调函数（输入）
**出口参数:  无
**函数功能:  注册 DA213 中断 GPIO 回调
**返回值:    ACCEL_SUCCESS 成功，其他值失败
*********************************************************************/
accel_result_t accelerometer_register_int_callback(accelerometer_int_callback_t callback)
{
    int ret;

    if (callback == NULL)
    {
        return ACCEL_ERROR_PARAM;
    }

    ret = da213_driver_register_int_callback(callback);

    return accel_convert_result(ret);
}

/********************************************************************
**函数名称:  accelerometer_set_int_latch
**入口参数:  latch_mode  ---        中断锁存模式（输入）
**出口参数:  无
**函数功能:  设置加速度传感器中断锁存模式
**返回值:    ACCEL_SUCCESS 成功，其他值失败
*********************************************************************/
accel_result_t accelerometer_set_int_latch(accel_int_latch_t latch_mode)
{
    int ret;

    if (!s_accel_initialized)
    {
        return ACCEL_ERROR_INIT;
    }

    if (latch_mode > ACCEL_INT_LATCH_HOLD2)
    {
        return ACCEL_ERROR_PARAM;
    }

    ret = da213_driver_set_int_latch((da213_int_latch_t)latch_mode);

    return accel_convert_result(ret);
}

/********************************************************************
**函数名称:  accelerometer_disable_int
**入口参数:  type        ---        要禁用的中断类型（输入）
**出口参数:  无
**函数功能:  禁用指定类型的中断
**返回值:    ACCEL_SUCCESS 成功，其他值失败
*********************************************************************/
accel_result_t accelerometer_disable_int(accel_int_type_t type)
{
    int ret;

    if (!s_accel_initialized)
    {
        return ACCEL_ERROR_INIT;
    }

    switch (type)
    {
        case ACCEL_INT_ACTIVE:
            ret = da213_driver_disable_active_int();
            break;

        case ACCEL_INT_SINGLE_TAP:
        case ACCEL_INT_DOUBLE_TAP:
            ret = da213_driver_disable_tap_int();
            break;

        case ACCEL_INT_FREEFALL:
            ret = da213_driver_disable_freefall_int();
            break;

        case ACCEL_INT_ORIENT:
            ret = da213_driver_disable_orient_int();
            break;

        case ACCEL_INT_NEWDATA:
            ret = da213_driver_enable_newdata_int(DA213_INT_PIN_1, false);
            break;

        default:
            return ACCEL_ERROR_PARAM;
    }

    return accel_convert_result(ret);
}

/********************************************************************
**函数名称:  accelerometer_read_int_status
**入口参数:  status      ---        中断状态存储指针（输出）
**出口参数:  status      ---        存储当前各中断触发标志状态
**函数功能:  读取并解析当前中断状态寄存器标志
**返回值:    ACCEL_SUCCESS 成功，其他值失败
*********************************************************************/
accel_result_t accelerometer_read_int_status(struct accel_int_status *status)
{
    int ret;
    uint8_t flag;

    if (!s_accel_initialized)
    {
        return ACCEL_ERROR_INIT;
    }

    if (status == NULL)
    {
        return ACCEL_ERROR_PARAM;
    }

    ret = da213_driver_read_motion_flag(&flag);
    if (ret < 0)
    {
        return ACCEL_ERROR_COMM;
    }

    status->active = (flag & DA213_MOTION_ACTIVE_INT) ? true : false;
    status->single_tap = (flag & DA213_MOTION_S_TAP_INT) ? true : false;
    status->double_tap = (flag & DA213_MOTION_D_TAP_INT) ? true : false;
    status->freefall = (flag & DA213_MOTION_FREEFALL_INT) ? true : false;
    status->orient = (flag & DA213_MOTION_ORIENT_INT) ? true : false;

    return ACCEL_SUCCESS;
}

/********************************************************************
**函数名称:  accelerometer_read_orient_status
**入口参数:  status      ---        方向状态存储指针（输出）
**出口参数:  status      ---        存储当前方向识别状态值
**函数功能:  读取并转换当前方向状态寄存器值
**返回值:    ACCEL_SUCCESS 成功，其他值失败
*********************************************************************/
accel_result_t accelerometer_read_orient_status(struct accel_orient_status *status)
{
    int ret;
    da213_orient_status_t drv_status;

    if (!s_accel_initialized)
    {
        return ACCEL_ERROR_INIT;
    }

    if (status == NULL)
    {
        return ACCEL_ERROR_PARAM;
    }

    ret = da213_driver_read_orient_status(&drv_status);
    if (ret < 0)
    {
        return ACCEL_ERROR_COMM;
    }

    status->orient_xy = drv_status.orient_xy;
    status->orient_z = drv_status.orient_z;

    return ACCEL_SUCCESS;
}

/********************************************************************
**函数名称:  accelerometer_reset_int
**入口参数:  无
**出口参数:  无
**函数功能:  复位所有锁存中断
**返回值:    ACCEL_SUCCESS 成功，其他值失败
*********************************************************************/
accel_result_t accelerometer_reset_int(void)
{
    int ret;

    if (!s_accel_initialized)
    {
        return ACCEL_ERROR_INIT;
    }

    ret = da213_driver_reset_int();

    return accel_convert_result(ret);
}
