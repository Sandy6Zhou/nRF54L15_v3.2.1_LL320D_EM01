/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        spa06_driver.c
**文件描述:        SPA06-003 气压传感器驱动实现文件
**当前版本:        V1.0
**作    者:       周森达 (zhousenda@jimiiot.com)
**完成日期:        2026.06.08
*********************************************************************
** 功能描述:       SPA06-003 气压传感器驱动实现文件，包含 I2C 寄存器读写、
**                采样率与过采样配置、校准系数读取解析与补偿计算、单次/连续
**                测量流程、测量就绪轮询与超时处理，以及工作模式控制
*********************************************************************/

#include "../inc/spa06_driver.h"

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(spa06_driver, LOG_LEVEL_INF);

/* I2C 设备配置 */
#define BARO_I2C_NODE DT_ALIAS(baro_i2c)
static const struct device *s_i2c_dev = DEVICE_DT_GET(BARO_I2C_NODE);

/* 测量就绪轮询步进周期（ms），以及在测量时间基础上额外预留的余量（ms） */
#define SPA06_MEAS_POLL_STEP_MS             5
#define SPA06_MEAS_TIMEOUT_MARGIN_MS        30

/* 连续气压模式温度刷新间隔（每读取 N 次气压重测一次温度，补偿温漂误差） */
#define SPA06_PRS_TEMP_REFRESH_INTERVAL     16

/* 内部状态 */
static struct spa06_coef s_coef;                                // 校准系数集合，初始化时从芯片寄存器读取并解析
static int32_t s_kp;                                            // 气压换算比例因子（由气压过采样档位决定），补偿计算时用于归一化原始气压值
static int32_t s_kt;                                            // 温度换算比例因子（由温度过采样档位决定），补偿计算时用于归一化原始温度值
static spa06_work_mode_t s_work_mode = SPA06_MODE_STOP;         // 当前工作模式，默认停止，决定读取测量时走单次/连续/温补分支
static int32_t s_last_raw_temperature;                          // 最近一次有效的原始温度值，连续气压模式下用于复用温度做气压补偿
static bool s_temperature_valid = false;                        // 原始温度缓存是否有效标志，false 时需先重新测量温度再做气压补偿

/* 气压/温度单次测量就绪等待超时（ms），按过采样档位在配置时动态计算 */
static uint16_t s_pressure_timeout_ms;
static uint16_t s_temperature_timeout_ms;

/* 连续气压模式温度刷新计数器，达到刷新间隔后重测一次温度 */
static uint8_t s_pressure_temp_refresh_cnt;

/********************************************************************
**函数名称:  spa06_reg_read
**入口参数:  reg      ---        寄存器地址
**           buf      ---        数据存储缓冲区（输出）
**           len      ---        读取长度
**出口参数:  buf      ---        读取到的寄存器数据
**函数功能:  通过 I2C 读取指定寄存器
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
static int spa06_reg_read(uint8_t reg, uint8_t *buf, uint8_t len)
{
    return i2c_write_read(s_i2c_dev, SPA06_I2C_ADDR, &reg, 1, buf, len);
}

/********************************************************************
**函数名称:  spa06_reg_write
**入口参数:  reg      ---        寄存器地址
**           val      ---        写入值
**出口参数:  无
**函数功能:  通过 I2C 写入指定寄存器
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
static int spa06_reg_write(uint8_t reg, uint8_t val)
{
    uint8_t tx_buf[2];

    tx_buf[0] = reg;
    tx_buf[1] = val;

    return i2c_write(s_i2c_dev, tx_buf, 2, SPA06_I2C_ADDR);
}

/********************************************************************
**函数名称:  spa06_twos_complement_24
**入口参数:  raw      ---        24-bit 无符号原始值
**出口参数:  无
**函数功能:  将 24-bit 无符号值转换为有符号 int32_t
**返 回 值:  符号扩展后的 32-bit 有符号值
*********************************************************************/
static int32_t spa06_twos_complement_24(uint32_t raw)
{
    if (raw & 0x800000)
    {
        return (int32_t)(raw | 0xFF000000);
    }
    return (int32_t)raw;
}

/********************************************************************
**函数名称:  spa06_get_sample_rate_cfg
**入口参数:  sample_rate_hz   ---        采样率，单位 Hz
**           rate_cfg         ---        采样率配置值存储指针（输出）
**出口参数:  rate_cfg         ---        采样率对应寄存器位[7:4]
**函数功能:  将采样率转换为 SPA06 寄存器配置值
**返 回 值:  0 表示成功，负值表示参数不支持
*********************************************************************/
static int spa06_get_sample_rate_cfg(uint8_t sample_rate_hz, uint8_t *rate_cfg)
{
    switch (sample_rate_hz)
    {
        case 1:
            *rate_cfg = 0U << 4;
            break;

        case 2:
            *rate_cfg = 1U << 4;
            break;

        case 4:
            *rate_cfg = 2U << 4;
            break;

        case 8:
            *rate_cfg = 3U << 4;
            break;

        case 16:
            *rate_cfg = 4U << 4;
            break;

        case 32:
            *rate_cfg = 5U << 4;
            break;

        case 64:
            *rate_cfg = 6U << 4;
            break;

        case 128:
            *rate_cfg = 7U << 4;
            break;

        default:
            return -EINVAL;
    }

    return 0;
}

/********************************************************************
**函数名称:  spa06_get_oversampling_cfg
**入口参数:  oversampling   ---        过采样倍率
**           osr_cfg        ---        过采样配置值存储指针（输出）
**           scale_factor   ---        补偿比例因子存储指针（输出）
**           shift_enable   ---        是否使能 shift 位存储指针（输出）
**出口参数:  osr_cfg        ---        过采样对应寄存器位[3:0]
**           scale_factor   ---        对应过采样倍率的比例因子
**           shift_enable   ---        大于 8x 时为 true
**函数功能:  将过采样倍率转换为 SPA06 配置值和比例因子
**返 回 值:  0 表示成功，负值表示参数不支持
*********************************************************************/
static int spa06_get_oversampling_cfg(uint8_t oversampling, uint8_t *osr_cfg,
                                      int32_t *scale_factor, bool *shift_enable)
{
    switch (oversampling)
    {
        case 1:
            *osr_cfg = 0;
            *scale_factor = 524288;
            *shift_enable = false;
            break;

        case 2:
            *osr_cfg = 1;
            *scale_factor = 1572864;
            *shift_enable = false;
            break;

        case 4:
            *osr_cfg = 2;
            *scale_factor = 3670016;
            *shift_enable = false;
            break;

        case 8:
            *osr_cfg = 3;
            *scale_factor = 7864320;
            *shift_enable = false;
            break;

        case 16:
            *osr_cfg = 4;
            *scale_factor = 253952;
            *shift_enable = true;
            break;

        case 32:
            *osr_cfg = 5;
            *scale_factor = 516096;
            *shift_enable = true;
            break;

        case 64:
            *osr_cfg = 6;
            *scale_factor = 1040384;
            *shift_enable = true;
            break;

        case 128:
            *osr_cfg = 7;
            *scale_factor = 2088960;
            *shift_enable = true;
            break;

        default:
            return -EINVAL;
    }

    return 0;
}

/********************************************************************
**函数名称:  spa06_calc_measurement_timeout
**入口参数:  oversampling   ---        过采样倍率（输入）
**出口参数:  无
**函数功能:  根据过采样倍率查表得到单次测量时间（手册 Table 8，向上取整），
**           叠加固定余量后作为就绪轮询超时
**返 回 值:  对应的就绪等待超时，单位 ms（不支持的倍率返回最大档超时）
*********************************************************************/
static uint16_t spa06_calc_measurement_timeout(uint8_t oversampling)
{
    uint16_t meas_time;     // 单次测量时间，单位 ms（手册值向上取整）

    switch (oversampling)
    {
        case 1:
            meas_time = 4U;         // 3.6ms
            break;

        case 2:
            meas_time = 6U;         // 5.2ms
            break;

        case 4:
            meas_time = 9U;         // 8.4ms
            break;

        case 8:
            meas_time = 15U;        // 14.8ms
            break;

        case 16:
            meas_time = 28U;        // 27.6ms
            break;

        case 32:
            meas_time = 54U;        // 53.2ms
            break;

        case 64:
            meas_time = 105U;       // 104.4ms
            break;

        case 128:
            meas_time = 207U;       // 206.8ms
            break;

        default:
            meas_time = 207U;       // 未知倍率按最大档处理，避免超时过短
            break;
    }

    return meas_time + SPA06_MEAS_TIMEOUT_MARGIN_MS;
}

/********************************************************************
**函数名称:  spa06_update_shift_cfg
**入口参数:  shift_mask      ---        shift 控制位掩码
**           shift_enable    ---        true 使能，false 关闭
**出口参数:  无
**函数功能:  更新 CFG_REG 中对应的 shift 控制位
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
static int spa06_update_shift_cfg(uint8_t shift_mask, bool shift_enable)
{
    int ret;
    uint8_t cfg_reg;

    ret = spa06_reg_read(SPA06_REG_CFG_REG, &cfg_reg, 1);
    if (ret < 0)
    {
        return ret;
    }

    if (shift_enable == true)
    {
        cfg_reg |= shift_mask;
    }
    else
    {
        cfg_reg &= (uint8_t)(~shift_mask);
    }

    ret = spa06_reg_write(SPA06_REG_CFG_REG, cfg_reg);

    return ret;
}

/********************************************************************
**函数名称:  spa06_wait_measurement_ready
**入口参数:  ready_mask      ---        数据就绪标志位掩码（输入）
**           timeout_ms      ---        最大等待时间，单位 ms（输入）
**出口参数:  无
**函数功能:  在指定超时内轮询等待指定测量数据就绪
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
static int spa06_wait_measurement_ready(uint8_t ready_mask, uint16_t timeout_ms)
{
    int ret;
    int timeout;
    uint8_t status;

    timeout = (int)timeout_ms / SPA06_MEAS_POLL_STEP_MS;
    while (timeout > 0)
    {
        ret = spa06_reg_read(SPA06_REG_MEAS_CFG, &status, 1);
        if (ret < 0)
        {
            return ret;
        }

        if ((status & ready_mask) == ready_mask)   // 掩码对应位全部就绪才返回成功
        {
            return 0;
        }

        k_sleep(K_MSEC(SPA06_MEAS_POLL_STEP_MS));
        timeout--;
    }

    return -ETIMEDOUT;
}

/********************************************************************
**函数名称:  spa06_read_raw_temperature
**入口参数:  raw_temperature ---        温度原始值存储指针（输出）
**出口参数:  raw_temperature ---        24-bit 符号扩展后的温度原始值
**函数功能:  读取温度原始寄存器数据
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
static int spa06_read_raw_temperature(int32_t *raw_temperature)
{
    int ret;
    uint8_t buf[3];
    uint32_t raw;

    ret = spa06_reg_read(SPA06_REG_TMP_B2, buf, sizeof(buf));
    if (ret < 0)
    {
        return ret;
    }

    raw = (uint32_t)buf[0] << 16 | (uint32_t)buf[1] << 8 | (uint32_t)buf[2];
    *raw_temperature = spa06_twos_complement_24(raw);

    return 0;
}

/********************************************************************
**函数名称:  spa06_read_raw_pressure
**入口参数:  raw_pressure    ---        气压原始值存储指针（输出）
**出口参数:  raw_pressure    ---        24-bit 符号扩展后的气压原始值
**函数功能:  读取气压原始寄存器数据
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
static int spa06_read_raw_pressure(int32_t *raw_pressure)
{
    int ret;
    uint8_t buf[3];
    uint32_t raw;

    ret = spa06_reg_read(SPA06_REG_PSR_B2, buf, sizeof(buf));
    if (ret < 0)
    {
        return ret;
    }

    raw = (uint32_t)buf[0] << 16 | (uint32_t)buf[1] << 8 | (uint32_t)buf[2];
    *raw_pressure = spa06_twos_complement_24(raw);

    return 0;
}

/********************************************************************
**函数名称:  spa06_measure_single_temperature
**入口参数:  raw_temperature ---        温度原始值存储指针（输出）
**出口参数:  raw_temperature ---        单次测量得到的温度原始值
**函数功能:  执行一次单次温度测量并读取原始数据
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
static int spa06_measure_single_temperature(int32_t *raw_temperature)
{
    int ret;

    ret = spa06_reg_write(SPA06_REG_MEAS_CFG, SPA06_MEAS_TMP_SINGLE);
    if (ret < 0)
    {
        return ret;
    }

    ret = spa06_wait_measurement_ready(SPA06_MEAS_TMP_RDY, s_temperature_timeout_ms);
    if (ret < 0)
    {
        LOG_ERR("SPA06 temperature measurement timeout");
        return ret;
    }

    ret = spa06_read_raw_temperature(raw_temperature);
    if (ret < 0)
    {
        return ret;
    }

    s_last_raw_temperature = *raw_temperature;
    s_temperature_valid = true;

    return 0;
}

/********************************************************************
**函数名称:  spa06_measure_single_pressure
**入口参数:  raw_pressure    ---        气压原始值存储指针（输出）
**出口参数:  raw_pressure    ---        单次测量得到的气压原始值
**函数功能:  执行一次单次气压测量并读取原始数据
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
static int spa06_measure_single_pressure(int32_t *raw_pressure)
{
    int ret;

    ret = spa06_reg_write(SPA06_REG_MEAS_CFG, SPA06_MEAS_PRS_SINGLE);
    if (ret < 0)
    {
        return ret;
    }

    ret = spa06_wait_measurement_ready(SPA06_MEAS_PRS_RDY, s_pressure_timeout_ms);
    if (ret < 0)
    {
        LOG_ERR("SPA06 pressure measurement timeout");
        return ret;
    }

    ret = spa06_read_raw_pressure(raw_pressure);
    if (ret < 0)
    {
        return ret;
    }

    return 0;
}

/********************************************************************
**函数名称:  spa06_read_continuous_data
**入口参数:  read_pressure   ---        是否读取气压（输入）
**           read_temperature ---       是否读取温度（输入）
**           raw_pressure    ---        气压原始值存储指针（输出）
**           raw_temperature ---        温度原始值存储指针（输出）
**出口参数:  raw_pressure    ---        读取到的气压原始值
**           raw_temperature ---        读取到的温度原始值
**函数功能:  读取连续模式下的原始测量数据
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
static int spa06_read_continuous_data(bool read_pressure, bool read_temperature,
                                      int32_t *raw_pressure, int32_t *raw_temperature)
{
    int ret;
    uint8_t ready_mask;
    uint16_t timeout_ms;

    ready_mask = 0;
    timeout_ms = 0;
    if (read_pressure == true)
    {
        ready_mask |= SPA06_MEAS_PRS_RDY;
        timeout_ms += s_pressure_timeout_ms;
    }

    if (read_temperature == true)
    {
        ready_mask |= SPA06_MEAS_TMP_RDY;
        timeout_ms += s_temperature_timeout_ms;
    }

    ret = spa06_wait_measurement_ready(ready_mask, timeout_ms);
    if (ret < 0)
    {
        LOG_ERR("SPA06 continuous measurement timeout");
        return ret;
    }

    if (read_temperature == true)
    {
        ret = spa06_read_raw_temperature(raw_temperature);
        if (ret < 0)
        {
            return ret;
        }

        s_last_raw_temperature = *raw_temperature;
        s_temperature_valid = true;
    }

    if (read_pressure == true)
    {
        ret = spa06_read_raw_pressure(raw_pressure);
        if (ret < 0)
        {
            return ret;
        }
    }

    return 0;
}

/********************************************************************
**函数名称:  spa06_config_pressure
**入口参数:  sample_rate_hz   ---        气压采样率，单位 Hz
**           oversampling     ---        气压过采样倍率
**出口参数:  无
**函数功能:  配置 SPA06 气压采样率和过采样参数
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
static int spa06_config_pressure(uint8_t sample_rate_hz, uint8_t oversampling)
{
    int ret;
    uint8_t rate_cfg;
    uint8_t osr_cfg;
    uint8_t reg_value;
    int32_t scale_factor;
    bool shift_enable;

    ret = spa06_get_sample_rate_cfg(sample_rate_hz, &rate_cfg);
    if (ret < 0)
    {
        return ret;
    }

    ret = spa06_get_oversampling_cfg(oversampling, &osr_cfg, &scale_factor, &shift_enable);
    if (ret < 0)
    {
        return ret;
    }

    reg_value = rate_cfg | osr_cfg;

    ret = spa06_reg_write(SPA06_REG_PRS_CFG, reg_value);
    if (ret < 0)
    {
        return ret;
    }

    ret = spa06_update_shift_cfg(SPA06_CFG_PRS_SHIFT, shift_enable);
    if (ret < 0)
    {
        return ret;
    }

    s_kp = scale_factor;

    /* 按过采样档位记录气压就绪等待超时 */
    s_pressure_timeout_ms = spa06_calc_measurement_timeout(oversampling);

    return 0;
}

/********************************************************************
**函数名称:  spa06_config_temperature
**入口参数:  sample_rate_hz   ---        温度采样率，单位 Hz
**           oversampling     ---        温度过采样倍率
**出口参数:  无
**函数功能:  配置 SPA06 温度采样率和过采样参数
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
static int spa06_config_temperature(uint8_t sample_rate_hz, uint8_t oversampling)
{
    int ret;
    uint8_t rate_cfg;
    uint8_t osr_cfg;
    uint8_t reg_value;
    int32_t scale_factor;
    bool shift_enable;

    ret = spa06_get_sample_rate_cfg(sample_rate_hz, &rate_cfg);
    if (ret < 0)
    {
        return ret;
    }

    ret = spa06_get_oversampling_cfg(oversampling, &osr_cfg, &scale_factor, &shift_enable);
    if (ret < 0)
    {
        return ret;
    }

    reg_value = rate_cfg | osr_cfg;

    ret = spa06_reg_write(SPA06_REG_TMP_CFG, reg_value);
    if (ret < 0)
    {
        return ret;
    }

    ret = spa06_update_shift_cfg(SPA06_CFG_TMP_SHIFT, shift_enable);
    if (ret < 0)
    {
        return ret;
    }

    s_kt = scale_factor;

    /* 按过采样档位记录温度就绪等待超时 */
    s_temperature_timeout_ms = spa06_calc_measurement_timeout(oversampling);

    return 0;
}

/********************************************************************
**函数名称:  spa06_read_coefficients
**入口参数:  无
**出口参数:  无
**函数功能:  读取并解析 SPA06-003 校准系数（寄存器 0x10~0x24）
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
static int spa06_read_coefficients(void)
{
    int ret;
    uint8_t buf[21];

    ret = spa06_reg_read(SPA06_REG_COEF_START, buf, sizeof(buf));
    if (ret < 0)
    {
        LOG_ERR("Read coefficients failed: %d", ret);
        return ret;
    }

    // c0: 12-bit signed, buf[0]<<4 | buf[1]>>4
    s_coef.c0 = (int16_t)((uint16_t)buf[0] << 4 | (uint16_t)buf[1] >> 4);
    if (s_coef.c0 & 0x0800)
    {
        s_coef.c0 |= 0xF000;
    }

    // c1: 12-bit signed, (buf[1]&0x0F)<<8 | buf[2]
    s_coef.c1 = (int16_t)(((uint16_t)buf[1] & 0x0F) << 8 | (uint16_t)buf[2]);
    if (s_coef.c1 & 0x0800)
    {
        s_coef.c1 |= 0xF000;
    }

    // c00: 20-bit signed, buf[3]<<12 | buf[4]<<4 | buf[5]>>4
    s_coef.c00 = (int32_t)((uint32_t)buf[3] << 12 | (uint32_t)buf[4] << 4 | (uint32_t)buf[5] >> 4);
    if (s_coef.c00 & 0x080000)
    {
        s_coef.c00 |= 0xFFF00000;
    }

    // c10: 20-bit signed, (buf[5]&0x0F)<<16 | buf[6]<<8 | buf[7]
    s_coef.c10 = (int32_t)(((uint32_t)buf[5] & 0x0F) << 16 | (uint32_t)buf[6] << 8 | (uint32_t)buf[7]);
    if (s_coef.c10 & 0x080000)
    {
        s_coef.c10 |= 0xFFF00000;
    }

    // c01: 16-bit signed
    s_coef.c01 = (int16_t)((uint16_t)buf[8] << 8 | (uint16_t)buf[9]);

    // c11: 16-bit signed
    s_coef.c11 = (int16_t)((uint16_t)buf[10] << 8 | (uint16_t)buf[11]);

    // c20: 16-bit signed
    s_coef.c20 = (int16_t)((uint16_t)buf[12] << 8 | (uint16_t)buf[13]);

    // c21: 16-bit signed
    s_coef.c21 = (int16_t)((uint16_t)buf[14] << 8 | (uint16_t)buf[15]);

    // c30: 16-bit signed
    s_coef.c30 = (int16_t)((uint16_t)buf[16] << 8 | (uint16_t)buf[17]);

    // c31: 12-bit signed, buf[18]<<4 | buf[19]>>4（SPA06-003 特有）
    s_coef.c31 = (int16_t)((uint16_t)buf[18] << 4 | (uint16_t)buf[19] >> 4);
    if (s_coef.c31 & 0x0800)
    {
        s_coef.c31 |= 0xF000;
    }

    // c40: 12-bit signed, (buf[19]&0x0F)<<8 | buf[20]（SPA06-003 特有）
    s_coef.c40 = (int16_t)(((uint16_t)buf[19] & 0x0F) << 8 | (uint16_t)buf[20]);
    if (s_coef.c40 & 0x0800)
    {
        s_coef.c40 |= 0xF000;
    }

    LOG_DBG("Coef: c0=%d c1=%d c00=%d c10=%d", s_coef.c0, s_coef.c1, s_coef.c00, s_coef.c10);
    return 0;
}

/********************************************************************
**函数名称:  spa06_compensate
**入口参数:  raw_prs       ---        气压原始值（24-bit 符号扩展）
**           raw_tmp       ---        温度原始值（24-bit 符号扩展，参与气压温度补偿）
**           pressure_pa   ---        气压结果存储指针（输出）
**出口参数:  pressure_pa   ---        补偿后气压值，单位 Pa
**函数功能:  使用校准系数对原始气压数据进行补偿计算（气压补偿需用温度
**           原始值修正温度交叉敏感性，但本函数不输出温度）
**返 回 值:  0 表示成功
*********************************************************************/
static int spa06_compensate(int32_t raw_prs, int32_t raw_tmp,
                            int32_t *pressure_pa)
{
    float fTsc;
    float fPsc;
    float qua2;
    float qua3;
    float fP;

    // 缩放原始值
    fTsc = (float)raw_tmp / (float)s_kt;
    fPsc = (float)raw_prs / (float)s_kp;

    // 气压补偿（4阶多项式，SPA06-003），fTsc 用于修正温度交叉敏感性
    qua2 = (float)s_coef.c10 + fPsc * ((float)s_coef.c20 + fPsc * ((float)s_coef.c30 + fPsc * (float)s_coef.c40));
    qua3 = fTsc * fPsc * ((float)s_coef.c11 + fPsc * ((float)s_coef.c21 + fPsc * (float)s_coef.c31));
    fP = (float)s_coef.c00 + fPsc * qua2 + fTsc * (float)s_coef.c01 + qua3;
    *pressure_pa = (int32_t)(fP + 0.5f);

    return 0;
}

/********************************************************************
**函数名称:  spa06_compensate_temperature_only
**入口参数:  raw_tmp       ---        温度原始值（输入）
**           temperature   ---        温度结果存储指针（输出）
**出口参数:  temperature   ---        补偿后温度值，单位 0.01°C
**函数功能:  使用校准系数对温度原始数据进行补偿计算
**返 回 值:  0 表示成功
*********************************************************************/
static int spa06_compensate_temperature_only(int32_t raw_tmp, int16_t *temperature)
{
    float fTsc;
    float fT;

    fTsc = (float)raw_tmp / (float)s_kt;
    fT = (float)s_coef.c0 * 0.5f + (float)s_coef.c1 * fTsc;
    *temperature = (int16_t)(fT * 100.0f);

    return 0;
}

/********************************************************************
**函数名称:  spa06_driver_init
**入口参数:  config   ---        采样配置参数（输入：采样率与过采样）
**出口参数:  无
**函数功能:  初始化 SPA06-003 驱动，包括 I2C 总线、校准系数读取，并按
**           传入配置设定气压/温度采样率和过采样
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
int spa06_driver_init(const spa06_config_t *config)
{
    int ret;
    uint8_t chip_id;
    uint8_t meas_cfg;
    int timeout;

    if (config == NULL)
    {
        return -EINVAL;
    }

    /* 检查 I2C 设备 */
    if (!device_is_ready(s_i2c_dev))
    {
        LOG_ERR("SPA06 I2C device not ready");
        return -ENODEV;
    }

    /* 等待传感器上电稳定 */
    k_sleep(K_MSEC(5));

    /* 软复位 */
    ret = spa06_reg_write(SPA06_REG_RESET, SPA06_RESET_CMD);
    if (ret < 0)
    {
        LOG_ERR("SPA06 soft reset failed: %d", ret);
        return ret;
    }
    /* 等待复位稳定 */
    k_sleep(K_MSEC(5));

    /* 轮询等待传感器就绪（COEF_RDY + SENSOR_RDY） */
    timeout = 10;
    while (timeout > 0)
    {
        ret = spa06_reg_read(SPA06_REG_MEAS_CFG, &meas_cfg, 1);
        if (ret < 0)
        {
            LOG_ERR("SPA06 read MEAS_CFG failed: %d", ret);
            return ret;
        }
        if ((meas_cfg & (SPA06_MEAS_COEF_RDY | SPA06_MEAS_SENSOR_RDY)) ==
            (SPA06_MEAS_COEF_RDY | SPA06_MEAS_SENSOR_RDY))
        {
            break;
        }
        k_sleep(K_MSEC(5));
        timeout--;
    }
    if (timeout <= 0)
    {
        LOG_ERR("SPA06 sensor not ready, MEAS_CFG=0x%02X", meas_cfg);
        return -ETIMEDOUT;
    }

    /* 读取并验证 Chip ID */
    ret = spa06_reg_read(SPA06_REG_CHIP_ID, &chip_id, 1);
    if (ret < 0)
    {
        LOG_ERR("SPA06 read chip ID failed: %d", ret);
        return ret;
    }
    if (chip_id != SPA06_CHIP_ID_VALUE)
    {
        LOG_ERR("SPA06 chip ID mismatch: 0x%02X (expected 0x%02X)", chip_id, SPA06_CHIP_ID_VALUE);
        return -ENODEV;
    }
    LOG_INF("SPA06 chip ID: 0x%02X", chip_id);

    /* 读取校准系数 */
    ret = spa06_read_coefficients();
    if (ret < 0)
    {
        return ret;
    }

    /* 按传入配置设定气压采样率和过采样 */
    ret = spa06_config_pressure(config->pressure_sample_rate_hz,
                                config->pressure_oversampling);
    if (ret < 0)
    {
        return ret;
    }

    /* 按传入配置设定温度采样率和过采样 */
    ret = spa06_config_temperature(config->temperature_sample_rate_hz,
                                   config->temperature_oversampling);
    if (ret < 0)
    {
        return ret;
    }

    s_temperature_valid = false;
    s_work_mode = SPA06_MODE_STOP;
    s_pressure_temp_refresh_cnt = 0;

    /* 进入待机模式 */
    spa06_reg_write(SPA06_REG_MEAS_CFG, SPA06_MEAS_STANDBY);

    LOG_INF("SPA06 driver initialized");
    return 0;
}

/********************************************************************
**函数名称:  spa06_driver_read_chip_id
**入口参数:  id       ---        ID 存储指针（输出）
**出口参数:  id       ---        芯片 ID 值
**函数功能:  读取 SPA06-003 芯片 ID
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
int spa06_driver_read_chip_id(uint8_t *id)
{
    return spa06_reg_read(SPA06_REG_CHIP_ID, id, 1);
}

/********************************************************************
**函数名称:  spa06_driver_set_work_mode
**入口参数:  work_mode      ---        SPA06 工作模式（输入）
**出口参数:  无
**函数功能:  设置 SPA06 工作模式并更新测量控制寄存器
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
int spa06_driver_set_work_mode(spa06_work_mode_t work_mode)
{
    int ret;
    uint8_t meas_cfg;

    switch (work_mode)
    {
        case SPA06_MODE_STOP:
            meas_cfg = SPA06_MEAS_STANDBY;
            break;

        case SPA06_MODE_SINGLE_TEMPERATURE:
            meas_cfg = SPA06_MEAS_STANDBY;
            break;

        case SPA06_MODE_SINGLE_PRESSURE:
            meas_cfg = SPA06_MEAS_STANDBY;
            break;

        case SPA06_MODE_SINGLE_BOTH:
            meas_cfg = SPA06_MEAS_STANDBY;
            break;

        case SPA06_MODE_CONTINUOUS_TEMPERATURE:
            meas_cfg = SPA06_MEAS_TMP_CONTINUOUS;
            break;

        case SPA06_MODE_CONTINUOUS_PRESSURE:
            meas_cfg = SPA06_MEAS_PRS_CONTINUOUS;
            break;

        case SPA06_MODE_CONTINUOUS_BOTH:
            meas_cfg = SPA06_MEAS_BOTH_CONTINUOUS;
            break;

        default:
            return -EINVAL;
    }

    ret = spa06_reg_write(SPA06_REG_MEAS_CFG, meas_cfg);
    if (ret < 0)
    {
        return ret;
    }

    /* 切换工作模式时复位温度缓存与刷新计数器，保证新会话首读重测温度 */
    s_temperature_valid = false;
    s_pressure_temp_refresh_cnt = 0;
    s_work_mode = work_mode;

    return 0;
}

/********************************************************************
**函数名称:  spa06_driver_read_measurement
**入口参数:  pressure_pa   ---        气压存储指针（输出）
**           temperature   ---        温度存储指针（输出）
**出口参数:  pressure_pa   ---        气压值，单位 Pa
**           temperature   ---        温度值，单位 0.01°C
**函数功能:  根据当前工作模式读取一次测量数据
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
int spa06_driver_read_measurement(int32_t *pressure_pa, int16_t *temperature)
{
    int ret;
    int32_t raw_tmp;
    int32_t raw_prs;

    if ((pressure_pa == NULL) || (temperature == NULL))
    {
        return -EINVAL;
    }

    raw_tmp = 0;
    raw_prs = 0;
    *pressure_pa = 0;
    *temperature = 0;

    switch (s_work_mode)
    {
        case SPA06_MODE_STOP:
            return -EINVAL;

        case SPA06_MODE_SINGLE_TEMPERATURE:
            ret = spa06_measure_single_temperature(&raw_tmp);
            if (ret < 0)
            {
                return ret;
            }

            return spa06_compensate_temperature_only(raw_tmp, temperature);

        case SPA06_MODE_SINGLE_PRESSURE:
            // 首次进入或刷新间隔到达时重测温度，补偿温漂误差
            if ((s_temperature_valid == false) ||
                (s_pressure_temp_refresh_cnt >= SPA06_PRS_TEMP_REFRESH_INTERVAL))
            {
                ret = spa06_measure_single_temperature(&raw_tmp);
                if (ret < 0)
                {
                    return ret;
                }

                s_pressure_temp_refresh_cnt = 0;
            }
            else
            {
                raw_tmp = s_last_raw_temperature;
            }

            ret = spa06_measure_single_pressure(&raw_prs);
            if (ret < 0)
            {
                return ret;
            }

            s_pressure_temp_refresh_cnt++;

            return spa06_compensate(raw_prs, raw_tmp, pressure_pa);

        case SPA06_MODE_SINGLE_BOTH:
            ret = spa06_measure_single_temperature(&raw_tmp);
            if (ret < 0)
            {
                return ret;
            }

            ret = spa06_measure_single_pressure(&raw_prs);
            if (ret < 0)
            {
                return ret;
            }

            spa06_compensate_temperature_only(raw_tmp, temperature);
            return spa06_compensate(raw_prs, raw_tmp, pressure_pa);

        case SPA06_MODE_CONTINUOUS_TEMPERATURE:
            ret = spa06_read_continuous_data(false, true, &raw_prs, &raw_tmp);
            if (ret < 0)
            {
                return ret;
            }

            return spa06_compensate_temperature_only(raw_tmp, temperature);

        case SPA06_MODE_CONTINUOUS_PRESSURE:
            // 首次进入或刷新间隔到达时，重测单次温度刷新缓存，补偿温漂误差
            if ((s_temperature_valid == false) ||
                (s_pressure_temp_refresh_cnt >= SPA06_PRS_TEMP_REFRESH_INTERVAL))
            {
                ret = spa06_measure_single_temperature(&raw_tmp);
                if (ret < 0)
                {
                    return ret;
                }

                // 单次温度测量会将 MEAS_CFG 切为待机，需恢复连续气压模式
                ret = spa06_reg_write(SPA06_REG_MEAS_CFG, SPA06_MEAS_PRS_CONTINUOUS);
                if (ret < 0)
                {
                    return ret;
                }

                s_pressure_temp_refresh_cnt = 0;
            }
            else
            {
                raw_tmp = s_last_raw_temperature;
            }

            ret = spa06_read_continuous_data(true, false, &raw_prs, &raw_tmp);
            if (ret < 0)
            {
                return ret;
            }

            s_pressure_temp_refresh_cnt++;

            return spa06_compensate(raw_prs, raw_tmp, pressure_pa);

        case SPA06_MODE_CONTINUOUS_BOTH:
            ret = spa06_read_continuous_data(true, true, &raw_prs, &raw_tmp);
            if (ret < 0)
            {
                return ret;
            }

            spa06_compensate_temperature_only(raw_tmp, temperature);
            return spa06_compensate(raw_prs, raw_tmp, pressure_pa);

        default:
            return -EINVAL;
    }
}
