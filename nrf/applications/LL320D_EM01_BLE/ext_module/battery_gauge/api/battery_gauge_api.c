/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        battery_gauge_api.c
**文件描述:        OM70201WV 电量计统一接口实现文件
**当前版本:        V1.0
**作    者:       周森达 (zhousenda@jimiiot.com)
**完成日期:        2026.07.21
*********************************************************************
** 功能描述:       封装 OM70201WV 厂家驱动和 Zephyr 端口层
**                 提供初始化、数据读取、工作模式和中断统一接口
*********************************************************************/

#include "battery_gauge_api.h"
#include "battery_gauge_reg.h"
#include "om70201wv_port.h"
#include "omg_battery.h"

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(battery_gauge_api, LOG_LEVEL_INF);

#define BATTERY_GAUGE_SOC_THRESHOLD_MAX 100U       // SOC 中断阈值最大百分比
#define BATTERY_GAUGE_TEMPERATURE_MIN_C (-40)      // 温度配置允许的最小摄氏度
#define BATTERY_GAUGE_TEMPERATURE_MAX_C 85         // 温度配置允许的最大摄氏度
#define BATTERY_GAUGE_TEMPERATURE_OFFSET_C 40      // 摄氏温度转寄存器值的零点偏移
#define BATTERY_GAUGE_TEMPERATURE_SCALE 2          // 温度寄存器每摄氏度对应的步进值
#define BATTERY_GAUGE_CYCLE_COUNT_MAX 1000U        // 厂家驱动支持的最大循环次数
#define BATTERY_GAUGE_CYCLE_COUNT_RAW_DIVISOR 32U  // 循环计数原始寄存器每次完整循环的步进值
#define BATTERY_GAUGE_CYCLE_COUNT_RAW_MAX \
    (BATTERY_GAUGE_CYCLE_COUNT_MAX * BATTERY_GAUGE_CYCLE_COUNT_RAW_DIVISOR)

static bool s_battery_gauge_initialized = false;

/********************************************************************
**函数名称:  battery_gauge_convert_vendor_result
**入口参数:  ret ---        厂家驱动返回值（输入）
**出口参数:  无
**函数功能:  将厂家驱动返回值转换为统一 API 错误码
**返回值:    转换后的 battery_gauge_result_t 错误码
*********************************************************************/
static battery_gauge_result_t battery_gauge_convert_vendor_result(int ret)
{
    if (ret == OMG_ERROR_NONE)
    {
        return BATTERY_GAUGE_SUCCESS;
    }

    if (ret == OMG_ERROR_IIC)
    {
        return BATTERY_GAUGE_ERROR_COMM;
    }

    if (ret == OMG_ERROR_CHIP_ID)
    {
        return BATTERY_GAUGE_ERROR_CHIP_ID;
    }

    if (ret == OMG_ERROR_NO_PROFILE)
    {
        return BATTERY_GAUGE_ERROR_INIT;
    }

    return BATTERY_GAUGE_ERROR_UNKNOWN;
}

/********************************************************************
**函数名称:  battery_gauge_check_initialized
**入口参数:  无
**出口参数:  无
**函数功能:  检查 OM70201WV API 是否已经初始化
**返回值:    BATTERY_GAUGE_SUCCESS 表示已初始化，其他表示未初始化
*********************************************************************/
static battery_gauge_result_t battery_gauge_check_initialized(void)
{
    if (s_battery_gauge_initialized != true)
    {
        return BATTERY_GAUGE_ERROR_INIT;
    }

    return BATTERY_GAUGE_SUCCESS;
}

/********************************************************************
**函数名称:  battery_gauge_convert_port_result
**入口参数:  ret ---        端口层返回值（输入）
**出口参数:  无
**函数功能:  将端口层返回值转换为统一 API 错误码
**返回值:    转换后的 battery_gauge_result_t 错误码
*********************************************************************/
static battery_gauge_result_t battery_gauge_convert_port_result(int ret)
{
    if (ret == 0)
    {
        return BATTERY_GAUGE_SUCCESS;
    }

    return BATTERY_GAUGE_ERROR_COMM;
}

/********************************************************************
**函数名称:  battery_gauge_read_cycle_count_raw_register
**入口参数:  cycle_count_raw ---        原始循环计数缓冲区（输入）
**出口参数:  cycle_count_raw ---        以 1/32 次为单位的原始计数（输出）
**函数功能:  读取 OM70201WV 循环计数寄存器原始值
**返回值:    0 表示成功，负值表示通信失败
*********************************************************************/
static int battery_gauge_read_cycle_count_raw_register(uint16_t *cycle_count_raw)
{
    int ret;
    uint8_t register_data[2];

    if (cycle_count_raw == NULL)
    {
        return -EINVAL;
    }

    ret = om70201wv_port_read(BATTERY_GAUGE_REG_CYCLE_COUNT_HIGH,
                              register_data,
                              sizeof(register_data));
    if (ret < 0)
    {
        return ret;
    }

    *cycle_count_raw = ((uint16_t)register_data[0] << 8) | register_data[1];

    return 0;
}

/********************************************************************
**函数名称:  battery_gauge_restore_cycle_count_raw_register
**入口参数:  cycle_count_raw ---        需要恢复的 1/32 次原始计数（输入）
**出口参数:  无
**函数功能:  写入 OM70201WV 循环计数并触发循环计数初始化
**返回值:    0 表示成功，负值表示通信或参数错误
*********************************************************************/
static int battery_gauge_restore_cycle_count_raw_register(uint16_t cycle_count_raw)
{
    int ret;
    uint8_t register_data[2];
    uint8_t config_value;

    if (cycle_count_raw > BATTERY_GAUGE_CYCLE_COUNT_RAW_MAX)
    {
        return -EINVAL;
    }

    register_data[0] = (uint8_t)(cycle_count_raw >> 8);
    register_data[1] = (uint8_t)(cycle_count_raw & 0xFFU);
    ret = om70201wv_port_write(BATTERY_GAUGE_REG_CYCLE_COUNT_HIGH,
                               register_data,
                               sizeof(register_data));
    if (ret < 0)
    {
        return ret;
    }

    config_value = BATTERY_GAUGE_CONFIG_ACTIVE_MODE_MASK |
                   BATTERY_GAUGE_CONFIG_CYCLE_COUNT_INIT_MASK;
    ret = om70201wv_port_write(BATTERY_GAUGE_REG_CONFIG,
                               &config_value,
                               sizeof(config_value));

    return ret;
}

/********************************************************************
**函数名称:  battery_gauge_temperature_to_register
**入口参数:  temperature_c ---        温度，单位摄氏度（输入）
**出口参数:  无
**函数功能:  将摄氏温度转换为 OM70201WV 温度阈值寄存器值
**返回值:    转换后的温度寄存器值
*********************************************************************/
static uint8_t battery_gauge_temperature_to_register(int8_t temperature_c)
{
    int16_t register_value;

    register_value = ((int16_t)temperature_c + BATTERY_GAUGE_TEMPERATURE_OFFSET_C) *
                     BATTERY_GAUGE_TEMPERATURE_SCALE;

    return (uint8_t)register_value;
}

/********************************************************************
**函数名称:  battery_gauge_init
**入口参数:  cycle_count_raw ---        需要恢复的原始循环计数，NULL 表示不恢复（输入）
**出口参数:  无
**函数功能:  初始化 OM70201WV 电量计并按需恢复循环计数
**返回值:    BATTERY_GAUGE_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
battery_gauge_result_t battery_gauge_init(const uint16_t *cycle_count_raw)
{
    int ret;
    battery_gauge_result_t result;

    if (s_battery_gauge_initialized == true)
    {
        return BATTERY_GAUGE_SUCCESS;
    }

    ret = om70201wv_port_init();
    if (ret < 0)
    {
        LOG_ERR("OM70201WV port init failed: %d", ret);
        return BATTERY_GAUGE_ERROR_INIT;
    }

    if ((cycle_count_raw != NULL) &&
        (*cycle_count_raw > BATTERY_GAUGE_CYCLE_COUNT_RAW_MAX))
    {
        return BATTERY_GAUGE_ERROR_PARAM;
    }

    ret = omg_init();
    result = battery_gauge_convert_vendor_result(ret);
    if (result != BATTERY_GAUGE_SUCCESS)
    {
        LOG_ERR("OM70201WV vendor init failed: %d", ret);
        return result;
    }

    s_battery_gauge_initialized = true;

    if (cycle_count_raw != NULL)
    {
        ret = battery_gauge_restore_cycle_count_raw_register(
            *cycle_count_raw);
        if (ret < 0)
        {
            s_battery_gauge_initialized = false;
            return battery_gauge_convert_port_result(ret);
        }

        ret = omg_set_soh_by_cycle_cnt(
            *cycle_count_raw / BATTERY_GAUGE_CYCLE_COUNT_RAW_DIVISOR);
        if (ret < 0)
        {
            s_battery_gauge_initialized = false;
            return battery_gauge_convert_vendor_result(ret);
        }
    }

    return BATTERY_GAUGE_SUCCESS;
}

/********************************************************************
**函数名称:  battery_gauge_get_chip_id
**入口参数:  chip_id ---        芯片 ID 缓冲区（输入）
**出口参数:  chip_id ---        OM70201WV 芯片 ID（输出）
**函数功能:  读取并校验 OM70201WV 芯片 ID
**返回值:    BATTERY_GAUGE_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
battery_gauge_result_t battery_gauge_get_chip_id(uint8_t *chip_id)
{
    int ret;
    battery_gauge_result_t result;

    if (chip_id == NULL)
    {
        return BATTERY_GAUGE_ERROR_PARAM;
    }

    result = battery_gauge_check_initialized();
    if (result != BATTERY_GAUGE_SUCCESS)
    {
        return result;
    }

    ret = omg_get_id(chip_id);

    return battery_gauge_convert_vendor_result(ret);
}

/********************************************************************
**函数名称:  battery_gauge_read
**入口参数:  data ---        电量计数据缓冲区（输入）
**出口参数:  data ---        电压、电流、温度、SOC、SOH 和循环次数（输出）
**函数功能:  按厂家推荐顺序读取 OM70201WV 全部常用数据
**返回值:    BATTERY_GAUGE_SUCCESS 表示成功，其他表示错误码
**注意事项:  current_ma 保持芯片原始符号，实机确认方向后由 API 层统一约定
*********************************************************************/
battery_gauge_result_t battery_gauge_read(battery_gauge_data_t *data)
{
    int ret;
    battery_gauge_result_t result;

    if (data == NULL)
    {
        return BATTERY_GAUGE_ERROR_PARAM;
    }

    result = battery_gauge_get_voltage(&data->voltage_mv);
    if (result != BATTERY_GAUGE_SUCCESS)
    {
        return result;
    }

    result = battery_gauge_get_current(&data->current_ma);
    if (result != BATTERY_GAUGE_SUCCESS)
    {
        return result;
    }

    result = battery_gauge_get_temperature(&data->temperature_c);
    if (result != BATTERY_GAUGE_SUCCESS)
    {
        return result;
    }

    result = battery_gauge_get_soc(&data->soc_percent);
    if (result != BATTERY_GAUGE_SUCCESS)
    {
        return result;
    }

    result = battery_gauge_get_cycle_count(&data->cycle_count);
    if (result != BATTERY_GAUGE_SUCCESS)
    {
        return result;
    }

    ret = omg_get_soh(&data->soh_percent);

    return battery_gauge_convert_vendor_result(ret);
}

/********************************************************************
**函数名称:  battery_gauge_get_voltage
**入口参数:  voltage_mv ---        电压缓冲区（输入）
**出口参数:  voltage_mv ---        电池电压，单位 mV（输出）
**函数功能:  读取 OM70201WV 电池电压
**返回值:    BATTERY_GAUGE_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
battery_gauge_result_t battery_gauge_get_voltage(uint16_t *voltage_mv)
{
    int ret;
    battery_gauge_result_t result;

    if (voltage_mv == NULL)
    {
        return BATTERY_GAUGE_ERROR_PARAM;
    }

    result = battery_gauge_check_initialized();
    if (result != BATTERY_GAUGE_SUCCESS)
    {
        return result;
    }

    ret = omg_get_vol(voltage_mv);

    return battery_gauge_convert_vendor_result(ret);
}

/********************************************************************
**函数名称:  battery_gauge_get_current
**入口参数:  current_ma ---        电流缓冲区（输入）
**出口参数:  current_ma ---        电池电流，单位 mA（输出）
**函数功能:  读取 OM70201WV 有符号电池电流
**返回值:    BATTERY_GAUGE_SUCCESS 表示成功，其他表示错误码
**注意事项:  当前硬件 CSP 接 GND，采用低边电流采样
*********************************************************************/
battery_gauge_result_t battery_gauge_get_current(int16_t *current_ma)
{
    int ret;
    battery_gauge_result_t result;

    if (current_ma == NULL)
    {
        return BATTERY_GAUGE_ERROR_PARAM;
    }

    result = battery_gauge_check_initialized();
    if (result != BATTERY_GAUGE_SUCCESS)
    {
        return result;
    }

    ret = omg_get_cur(current_ma);

    return battery_gauge_convert_vendor_result(ret);
}

/********************************************************************
**函数名称:  battery_gauge_get_temperature
**入口参数:  temperature_c ---        温度缓冲区（输入）
**出口参数:  temperature_c ---        电池温度，单位摄氏度（输出）
**函数功能:  读取 OM70201WV 电池温度
**返回值:    BATTERY_GAUGE_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
battery_gauge_result_t battery_gauge_get_temperature(int8_t *temperature_c)
{
    int ret;
    battery_gauge_result_t result;

    if (temperature_c == NULL)
    {
        return BATTERY_GAUGE_ERROR_PARAM;
    }

    result = battery_gauge_check_initialized();
    if (result != BATTERY_GAUGE_SUCCESS)
    {
        return result;
    }

    ret = omg_get_temp(temperature_c);

    return battery_gauge_convert_vendor_result(ret);
}

/********************************************************************
**函数名称:  battery_gauge_get_soc
**入口参数:  soc_percent ---        SOC 缓冲区（输入）
**出口参数:  soc_percent ---        电池剩余电量百分比（输出）
**函数功能:  读取 OM70201WV SOC
**返回值:    BATTERY_GAUGE_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
battery_gauge_result_t battery_gauge_get_soc(uint8_t *soc_percent)
{
    int ret;
    battery_gauge_result_t result;

    if (soc_percent == NULL)
    {
        return BATTERY_GAUGE_ERROR_PARAM;
    }

    result = battery_gauge_check_initialized();
    if (result != BATTERY_GAUGE_SUCCESS)
    {
        return result;
    }

    ret = omg_get_soc(soc_percent);

    return battery_gauge_convert_vendor_result(ret);
}

/********************************************************************
**函数名称:  battery_gauge_get_soh
**入口参数:  soh_percent ---        SOH 缓冲区（输入）
**出口参数:  soh_percent ---        电池健康度百分比（输出）
**函数功能:  读取 OM70201WV SOH
**返回值:    BATTERY_GAUGE_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
battery_gauge_result_t battery_gauge_get_soh(uint8_t *soh_percent)
{
    int ret;
    battery_gauge_result_t result;

    if (soh_percent == NULL)
    {
        return BATTERY_GAUGE_ERROR_PARAM;
    }

    result = battery_gauge_check_initialized();
    if (result != BATTERY_GAUGE_SUCCESS)
    {
        return result;
    }

    ret = omg_get_soh(soh_percent);

    return battery_gauge_convert_vendor_result(ret);
}

/********************************************************************
**函数名称:  battery_gauge_get_cycle_count
**入口参数:  cycle_count ---        循环次数缓冲区（输入）
**出口参数:  cycle_count ---        电池循环次数（输出）
**函数功能:  读取 OM70201WV 电池循环次数
**返回值:    BATTERY_GAUGE_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
battery_gauge_result_t battery_gauge_get_cycle_count(uint16_t *cycle_count)
{
    int ret;
    uint16_t cycle_count_raw;
    battery_gauge_result_t result;

    if (cycle_count == NULL)
    {
        return BATTERY_GAUGE_ERROR_PARAM;
    }

    result = battery_gauge_check_initialized();
    if (result != BATTERY_GAUGE_SUCCESS)
    {
        return result;
    }

    cycle_count_raw = 0U;
    ret = battery_gauge_read_cycle_count_raw_register(&cycle_count_raw);
    result = battery_gauge_convert_port_result(ret);
    if (result != BATTERY_GAUGE_SUCCESS)
    {
        return result;
    }

    *cycle_count = cycle_count_raw / BATTERY_GAUGE_CYCLE_COUNT_RAW_DIVISOR;

    ret = omg_set_soh_by_cycle_cnt(*cycle_count);
    result = battery_gauge_convert_vendor_result(ret);
    if (result != BATTERY_GAUGE_SUCCESS)
    {
        return result;
    }

    return BATTERY_GAUGE_SUCCESS;
}

/********************************************************************
**函数名称:  battery_gauge_get_cycle_count_raw
**入口参数:  cycle_count_raw ---        原始循环计数缓冲区（输入）
**出口参数:  cycle_count_raw ---        以 1/32 次为单位的原始循环计数（输出）
**函数功能:  读取 OM70201WV 循环次数原始寄存器值
**返回值:    BATTERY_GAUGE_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
battery_gauge_result_t battery_gauge_get_cycle_count_raw(uint16_t *cycle_count_raw)
{
    int ret;
    battery_gauge_result_t result;

    if (cycle_count_raw == NULL)
    {
        return BATTERY_GAUGE_ERROR_PARAM;
    }

    result = battery_gauge_check_initialized();
    if (result != BATTERY_GAUGE_SUCCESS)
    {
        return result;
    }

    ret = battery_gauge_read_cycle_count_raw_register(cycle_count_raw);

    return battery_gauge_convert_port_result(ret);
}

/********************************************************************
**函数名称:  battery_gauge_set_cycle_count
**入口参数:  cycle_count ---        需要设置的电池循环次数（输入）
**出口参数:  无
**函数功能:  设置 OM70201WV 电池循环次数并更新 SOH
**返回值:    BATTERY_GAUGE_SUCCESS 表示成功，其他表示错误码
**注意事项:  仅用于生产初始化、数据恢复和测试验证
*********************************************************************/
battery_gauge_result_t battery_gauge_set_cycle_count(uint16_t cycle_count)
{
    int ret;
    uint16_t cycle_count_raw;
    battery_gauge_result_t result;

    result = battery_gauge_check_initialized();
    if (result != BATTERY_GAUGE_SUCCESS)
    {
        return result;
    }

    if (cycle_count > BATTERY_GAUGE_CYCLE_COUNT_MAX)
    {
        return BATTERY_GAUGE_ERROR_PARAM;
    }

    cycle_count_raw = cycle_count * BATTERY_GAUGE_CYCLE_COUNT_RAW_DIVISOR;
    ret = battery_gauge_restore_cycle_count_raw_register(cycle_count_raw);
    result = battery_gauge_convert_port_result(ret);
    if (result != BATTERY_GAUGE_SUCCESS)
    {
        return result;
    }

    ret = omg_set_soh_by_cycle_cnt(cycle_count);
    result = battery_gauge_convert_vendor_result(ret);
    if (result != BATTERY_GAUGE_SUCCESS)
    {
        return result;
    }

    return BATTERY_GAUGE_SUCCESS;
}

/********************************************************************
**函数名称:  battery_gauge_set_work_mode
**入口参数:  work_mode ---        电量计工作模式（输入）
**出口参数:  无
**函数功能:  设置 OM70201WV 进入睡眠模式或正常工作模式
**返回值:    BATTERY_GAUGE_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
battery_gauge_result_t battery_gauge_set_work_mode(
    battery_gauge_work_mode_t work_mode)
{
    int ret;
    battery_gauge_result_t result;

    result = battery_gauge_check_initialized();
    if (result != BATTERY_GAUGE_SUCCESS)
    {
        return result;
    }

    if (work_mode >= BATTERY_GAUGE_WORK_MODE_MAX)
    {
        return BATTERY_GAUGE_ERROR_PARAM;
    }

    switch (work_mode)
    {
        case BATTERY_GAUGE_WORK_MODE_SLEEP:
            ret = omg_sleep();
            break;

        case BATTERY_GAUGE_WORK_MODE_NORMAL:
            ret = omg_active();
            break;

        default:
            return BATTERY_GAUGE_ERROR_PARAM;
    }

    return battery_gauge_convert_vendor_result(ret);
}

/********************************************************************
**函数名称:  battery_gauge_interrupt_config
**入口参数:  config ---        中断使能和阈值配置（输入）
**出口参数:  无
**函数功能:  配置 OM70201WV SOC、高温和低温中断
**返回值:    BATTERY_GAUGE_SUCCESS 表示成功，其他表示错误码
**注意事项:  SOC 阈值为 100 时，SOC 每变化 1% 产生一次中断
*********************************************************************/
battery_gauge_result_t battery_gauge_interrupt_config(
    const battery_gauge_interrupt_config_t *config)
{
    int ret;
    uint8_t interrupt_config;
    uint8_t register_value;
    battery_gauge_result_t result;

    if (config == NULL)
    {
        return BATTERY_GAUGE_ERROR_PARAM;
    }

    result = battery_gauge_check_initialized();
    if (result != BATTERY_GAUGE_SUCCESS)
    {
        return result;
    }

    if ((config->soc_threshold_percent > BATTERY_GAUGE_SOC_THRESHOLD_MAX) ||
        (config->high_temperature_c < BATTERY_GAUGE_TEMPERATURE_MIN_C) ||
        (config->high_temperature_c > BATTERY_GAUGE_TEMPERATURE_MAX_C) ||
        (config->low_temperature_c < BATTERY_GAUGE_TEMPERATURE_MIN_C) ||
        (config->low_temperature_c > BATTERY_GAUGE_TEMPERATURE_MAX_C) ||
        (config->low_temperature_c > config->high_temperature_c))
    {
        return BATTERY_GAUGE_ERROR_PARAM;
    }

    register_value = config->soc_threshold_percent;
    ret = om70201wv_port_write(BATTERY_GAUGE_REG_SOC_ALERT, &register_value, 1U);
    if (ret < 0)
    {
        return battery_gauge_convert_port_result(ret);
    }

    register_value = battery_gauge_temperature_to_register(config->high_temperature_c);
    ret = om70201wv_port_write(BATTERY_GAUGE_REG_TEMP_MAX, &register_value, 1U);
    if (ret < 0)
    {
        return battery_gauge_convert_port_result(ret);
    }

    register_value = battery_gauge_temperature_to_register(config->low_temperature_c);
    ret = om70201wv_port_write(BATTERY_GAUGE_REG_TEMP_MIN, &register_value, 1U);
    if (ret < 0)
    {
        return battery_gauge_convert_port_result(ret);
    }

    ret = om70201wv_port_read(BATTERY_GAUGE_REG_INTERRUPT_CONFIG, &interrupt_config, 1U);
    if (ret < 0)
    {
        return battery_gauge_convert_port_result(ret);
    }

    interrupt_config &= BATTERY_GAUGE_INTERRUPT_TEMP_SELECT_MASK;
    if (config->soc_enable == true)
    {
        interrupt_config |= BATTERY_GAUGE_INTERRUPT_SOC_ENABLE_MASK;
    }

    if (config->high_temperature_enable == true)
    {
        interrupt_config |= BATTERY_GAUGE_INTERRUPT_HIGH_TEMP_ENABLE_MASK;
    }

    if (config->low_temperature_enable == true)
    {
        interrupt_config |= BATTERY_GAUGE_INTERRUPT_LOW_TEMP_ENABLE_MASK;
    }

    ret = om70201wv_port_write(BATTERY_GAUGE_REG_INTERRUPT_CONFIG, &interrupt_config, 1U);

    return battery_gauge_convert_port_result(ret);
}

/********************************************************************
**函数名称:  battery_gauge_interrupt_get_status
**入口参数:  status ---        中断状态缓冲区（输入）
**出口参数:  status ---        中断状态位组合（输出）
**函数功能:  读取 OM70201WV SOC、高温和低温中断状态
**返回值:    BATTERY_GAUGE_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
battery_gauge_result_t battery_gauge_interrupt_get_status(
    battery_gauge_interrupt_status_t *status)
{
    int ret;
    uint8_t interrupt_config;
    battery_gauge_result_t result;

    if (status == NULL)
    {
        return BATTERY_GAUGE_ERROR_PARAM;
    }

    result = battery_gauge_check_initialized();
    if (result != BATTERY_GAUGE_SUCCESS)
    {
        return result;
    }

    ret = om70201wv_port_read(BATTERY_GAUGE_REG_INTERRUPT_CONFIG, &interrupt_config, 1U);
    if (ret < 0)
    {
        return battery_gauge_convert_port_result(ret);
    }

    *status = (battery_gauge_interrupt_status_t)(interrupt_config &
                                                 BATTERY_GAUGE_INTERRUPT_STATUS_MASK);

    return BATTERY_GAUGE_SUCCESS;
}

/********************************************************************
**函数名称:  battery_gauge_register_interrupt_callback
**入口参数:  callback ---        中断回调函数（输入）
**出口参数:  无
**函数功能:  注册 OM70201WV INTN GPIO 中断回调
**返回值:    BATTERY_GAUGE_SUCCESS 表示成功，其他表示错误码
**注意事项:  回调在 GPIO 中断上下文执行，禁止直接调用电量计读写接口
*********************************************************************/
battery_gauge_result_t battery_gauge_register_interrupt_callback(
    battery_gauge_interrupt_callback_t callback)
{
    int ret;
    battery_gauge_result_t result;

    if (callback == NULL)
    {
        return BATTERY_GAUGE_ERROR_PARAM;
    }

    result = battery_gauge_check_initialized();
    if (result != BATTERY_GAUGE_SUCCESS)
    {
        return result;
    }

    ret = om70201wv_port_register_interrupt_callback(callback);

    return battery_gauge_convert_port_result(ret);
}
