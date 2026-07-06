/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        da213_driver.c
**文件描述:        DA213 三轴加速度传感器驱动实现文件
**当前版本:        V1.0
**作    者:       周森达 (zhousenda@jimiiot.com)
**完成日期:        2026.07.06
*********************************************************************
** 功能描述:       DA213 三轴加速度传感器驱动实现文件，包含 I2C 寄存器读写、
**                采样率与量程配置、电源模式控制、三轴数据读取、运动/敲击/
**                自由落体/方向识别中断配置及偏移补偿等
*********************************************************************/

#include "../inc/da213_driver.h"

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(da213_driver, LOG_LEVEL_INF);

/* I2C 设备绑定，使用 devicetree 别名 gsensor-i2c */
#define GSENSOR_I2C_NODE DT_ALIAS(gsensor_i2c)
#define GSENSOR_INT_NODE DT_ALIAS(gsensor_int)
static const struct device *s_i2c_dev = DEVICE_DT_GET(GSENSOR_I2C_NODE);
static const struct gpio_dt_spec s_int_gpio = GPIO_DT_SPEC_GET(GSENSOR_INT_NODE, gpios);

/* 内部状态 */
static da213_range_t s_current_range = DA213_RANGE_2G;
static da213_resolution_t s_current_resolution = DA213_RESOLUTION_14BIT;
static struct gpio_callback s_int_gpio_cb;
static da213_int_callback_t s_int_callback = NULL;
static bool s_initialized = false;
static bool s_int_callback_added = false;

/********************************************************************
**函数名称:  da213_reg_read
**入口参数:  reg_addr    ---        寄存器地址（输入）
**          data        ---        数据缓冲区指针（输出）
**          len         ---        读取长度（输入）
**出口参数:  data        ---        存储读取到的数据
**函数功能:  通过 I2C 读取 DA213 指定寄存器的数据
**返回值:    0 成功，负值 errno 失败
*********************************************************************/
static int da213_reg_read(uint8_t reg_addr, uint8_t *data, uint8_t len)
{
    int ret;

    ret = i2c_burst_read(s_i2c_dev, DA213_I2C_ADDR, reg_addr, data, len);
    if (ret < 0)
    {
        LOG_ERR("I2C read failed, reg=0x%02X, ret=%d", reg_addr, ret);
    }

    return ret;
}

/********************************************************************
**函数名称:  da213_reg_write
**入口参数:  reg_addr    ---        寄存器地址（输入）
**          data        ---        要写入的数据（输入）
**出口参数:  无
**函数功能:  通过 I2C 向 DA213 指定寄存器写入单字节数据
**返回值:    0 成功，负值 errno 失败
*********************************************************************/
static int da213_reg_write(uint8_t reg_addr, uint8_t data)
{
    int ret;

    ret = i2c_reg_write_byte(s_i2c_dev, DA213_I2C_ADDR, reg_addr, data);
    if (ret < 0)
    {
        LOG_ERR("I2C write failed, reg=0x%02X, ret=%d", reg_addr, ret);
    }

    return ret;
}

/********************************************************************
**函数名称:  da213_reg_mask_write
**入口参数:  reg_addr    ---        寄存器地址（输入）
**          mask        ---        位掩码（输入）
**          data        ---        要写入的数据（输入）
**出口参数:  无
**函数功能:  读-修改-写操作，仅修改寄存器中 mask 指定的位
**返回值:    0 成功，负值 errno 失败
*********************************************************************/
static int da213_reg_mask_write(uint8_t reg_addr, uint8_t mask, uint8_t data)
{
    int ret;
    uint8_t tmp;

    ret = da213_reg_read(reg_addr, &tmp, 1);
    if (ret < 0)
    {
        return ret;
    }

    tmp &= ~mask;
    tmp |= (data & mask);
    return da213_reg_write(reg_addr, tmp);
}

/********************************************************************
**函数名称:  da213_driver_int_isr
**入口参数:  dev         ---        GPIO设备指针（输入）
**          cb          ---        GPIO回调结构体（输入）
**          pins        ---        中断引脚位图（输入）
**出口参数:  无
**函数功能:  DA213 中断 GPIO ISR 回调转发函数
**返回值:    无
*********************************************************************/
static void da213_driver_int_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    if (s_int_callback != NULL)
    {
        s_int_callback();
    }
}

/********************************************************************
**函数名称:  da213_driver_init
**入口参数:  config      ---        初始化配置参数指针（输入）
**出口参数:  无
**函数功能:  初始化 DA213 传感器，校验芯片ID，执行软复位，进入工程模式，
**          并根据配置参数设置量程、分辨率、ODR和电源模式
**返回值:    0 成功，-ENODEV 芯片ID不匹配，其他负值 errno 失败
*********************************************************************/
int da213_driver_init(const da213_config_t *config)
{
    int ret;
    uint8_t chip_id = 0;
    int retry;

    if (config == NULL)
    {
        return -EINVAL;
    }

    if (!device_is_ready(s_i2c_dev))
    {
        LOG_ERR("I2C device not ready");
        return -ENODEV;
    }

    /* 校验芯片 ID，重试3次 */
    for (retry = 0; retry < 3; retry++)
    {
        ret = da213_reg_read(DA213_REG_CHIP_ID, &chip_id, 1);
        if (ret == 0 && chip_id == DA213_CHIP_ID_VALUE)
        {
            break;
        }
        k_msleep(5);
    }

    if (chip_id != DA213_CHIP_ID_VALUE)
    {
        LOG_ERR("DA213 chip ID mismatch: 0x%02X (expected 0x%02X)", chip_id, DA213_CHIP_ID_VALUE);
        return -ENODEV;
    }

    LOG_INF("DA213 chip ID: 0x%02X", chip_id);

    /* 配置量程和分辨率 */
    ret = da213_driver_set_range(config->range);
    if (ret < 0)
    {
        return ret;
    }

    ret = da213_driver_set_resolution(config->resolution);
    if (ret < 0)
    {
        return ret;
    }

    /* 先配置电源模式，低功耗模式下再单独配置 MODE_BW 中的带宽位 */
    ret = da213_driver_set_power_mode(config->power_mode);
    if (ret < 0)
    {
        return ret;
    }

    /* 若为低功耗模式，配置低功耗带宽 */
    if (config->power_mode == DA213_MODE_LOW_POWER)
    {
        ret = da213_driver_set_lp_bandwidth(config->lp_bandwidth);
        if (ret < 0)
        {
            return ret;
        }
    }

    /* 配置 ODR */
    ret = da213_driver_set_odr(config->odr);
    if (ret < 0)
    {
        return ret;
    }

    s_initialized = true;
    LOG_INF("DA213 driver initialized");

    return 0;
}

/********************************************************************
**函数名称:  da213_driver_read_chip_id
**入口参数:  id          ---        芯片ID存储指针（输出）
**出口参数:  id          ---        存储读取到的芯片ID值
**函数功能:  读取 DA213 芯片 ID 寄存器
**返回值:    0 成功，负值 errno 失败
*********************************************************************/
int da213_driver_read_chip_id(uint8_t *id)
{
    if (id == NULL)
    {
        return -EINVAL;
    }

    return da213_reg_read(DA213_REG_CHIP_ID, id, 1);
}

/********************************************************************
**函数名称:  da213_driver_set_power_mode
**入口参数:  mode        ---        电源模式（输入）
**出口参数:  无
**函数功能:  设置 DA213 电源模式（正常/低功耗/挂起）
**返回值:    0 成功，负值 errno 失败
*********************************************************************/
int da213_driver_set_power_mode(da213_power_mode_t mode)
{
    uint8_t reg_val;

    switch (mode)
    {
        case DA213_MODE_NORMAL:
            reg_val = (0x00 << 6) | (0x07 << 1);    // pwr_mode=00, low_power_bw=0111(62.5Hz)
            break;

        case DA213_MODE_LOW_POWER:
            reg_val = (0x01 << 6) | (0x07 << 1);    // pwr_mode=01, low_power_bw=0111
            break;

        case DA213_MODE_SUSPEND:
            reg_val = 0x80;     // pwr_mode=10
            break;

        default:
            return -EINVAL;
    }

    return da213_reg_write(DA213_REG_MODE_BW, reg_val);
}

/********************************************************************
**函数名称:  da213_driver_set_range
**入口参数:  range       ---        量程设置（输入）
**出口参数:  无
**函数功能:  设置 DA213 加速度量程（±2g/±4g/±8g/±16g）
**返回值:    0 成功，负值 errno 失败
*********************************************************************/
int da213_driver_set_range(da213_range_t range)
{
    int ret;

    if (range > DA213_RANGE_16G)
    {
        return -EINVAL;
    }

    ret = da213_reg_mask_write(DA213_REG_RESOLUTION_RANGE, 0x03, (uint8_t)range);
    if (ret == 0)
    {
        s_current_range = range;
    }

    return ret;
}

/********************************************************************
**函数名称:  da213_driver_set_resolution
**入口参数:  resolution  ---        分辨率设置（输入）
**出口参数:  无
**函数功能:  设置 DA213 数据分辨率（14/12/10/8位）
**返回值:    0 成功，负值 errno 失败
*********************************************************************/
int da213_driver_set_resolution(da213_resolution_t resolution)
{
    int ret;

    if (resolution > DA213_RESOLUTION_8BIT)
    {
        return -EINVAL;
    }

    ret = da213_reg_mask_write(DA213_REG_RESOLUTION_RANGE, 0x0C, (uint8_t)(resolution << 2));
    if (ret == 0)
    {
        s_current_resolution = resolution;
    }

    return ret;
}

/********************************************************************
**函数名称:  da213_driver_set_odr
**入口参数:  odr         ---        输出数据率（输入）
**出口参数:  无
**函数功能:  设置 DA213 输出数据率
**返回值:    0 成功，负值 errno 失败
*********************************************************************/
int da213_driver_set_odr(da213_odr_t odr)
{
    if (odr > DA213_ODR_1000HZ)
    {
        return -EINVAL;
    }

    return da213_reg_mask_write(DA213_REG_ODR_AXIS, 0x0F, (uint8_t)odr);
}

/********************************************************************
**函数名称:  da213_driver_set_lp_bandwidth
**入口参数:  bandwidth   ---        低功耗带宽（输入）
**出口参数:  无
**函数功能:  设置 DA213 低功耗模式下的带宽
**返回值:    0 成功，负值 errno 失败
*********************************************************************/
int da213_driver_set_lp_bandwidth(da213_lp_bw_t bandwidth)
{
    return da213_reg_mask_write(DA213_REG_MODE_BW, 0x1E, (uint8_t)(bandwidth << 1));
}

/********************************************************************
**函数名称:  da213_driver_read_raw_data
**入口参数:  data        ---        三轴数据存储指针（输出）
**出口参数:  data        ---        存储读取到的三轴加速度原始数据
**函数功能:  读取 DA213 三轴加速度原始数据（14位左对齐，右移2位得到有效值）
**返回值:    0 成功，负值 errno 失败
*********************************************************************/
int da213_driver_read_raw_data(da213_raw_data_t *data)
{
    int ret;
    uint8_t buf[6];
    uint8_t shift;

    if (data == NULL)
    {
        return -EINVAL;
    }

    ret = da213_reg_read(DA213_REG_ACC_X_LSB, buf, 6);
    if (ret < 0)
    {
        return ret;
    }

    /* 根据分辨率确定右移位数，原始数据为左对齐格式，14bit时组合后的16位数据低2位无效 */
    switch (s_current_resolution)
    {
        case DA213_RESOLUTION_14BIT:
            shift = 2;
            break;

        case DA213_RESOLUTION_12BIT:
            shift = 4;
            break;

        case DA213_RESOLUTION_10BIT:
            shift = 6;
            break;

        case DA213_RESOLUTION_8BIT:
            shift = 8;
            break;

        default:
            shift = 2;
            break;
    }

    data->x = ((int16_t)(buf[1] << 8 | buf[0])) >> shift;
    data->y = ((int16_t)(buf[3] << 8 | buf[2])) >> shift;
    data->z = ((int16_t)(buf[5] << 8 | buf[4])) >> shift;

    return 0;
}

/********************************************************************
**函数名称:  da213_driver_set_axis_enable
**入口参数:  enable_x    ---        X轴使能（输入）
**          enable_y    ---        Y轴使能（输入）
**          enable_z    ---        Z轴使能（输入）
**出口参数:  无
**函数功能:  使能或禁用 DA213 各轴数据输出
**返回值:    0 成功，负值 errno 失败
*********************************************************************/
int da213_driver_set_axis_enable(bool enable_x, bool enable_y, bool enable_z)
{
    uint8_t val = 0;

    /* 寄存器中 1 表示禁用 */
    if (!enable_x)
    {
        val |= (1 << 7);
    }
    if (!enable_y)
    {
        val |= (1 << 6);
    }
    if (!enable_z)
    {
        val |= (1 << 5);
    }

    return da213_reg_mask_write(DA213_REG_ODR_AXIS, 0xE0, val);
}

/********************************************************************
**函数名称:  da213_driver_set_axis_polarity
**入口参数:  invert_x    ---        X轴极性反转（输入）
**          invert_y    ---        Y轴极性反转（输入）
**          invert_z    ---        Z轴极性反转（输入）
**出口参数:  无
**函数功能:  设置 DA213 各轴极性（是否取反）
**返回值:    0 成功，负值 errno 失败
*********************************************************************/
int da213_driver_set_axis_polarity(bool invert_x, bool invert_y, bool invert_z)
{
    uint8_t val = 0;

    if (invert_x)
    {
        val |= (1 << 3);
    }
    if (invert_y)
    {
        val |= (1 << 2);
    }
    if (invert_z)
    {
        val |= (1 << 1);
    }

    return da213_reg_mask_write(DA213_REG_SWAP_POLARITY, 0x0E, val);
}

/********************************************************************
**函数名称:  da213_driver_set_xy_swap
**入口参数:  swap        ---        是否交换XY轴数据（输入）
**出口参数:  无
**函数功能:  设置 DA213 X/Y 轴数据交换
**返回值:    0 成功，负值 errno 失败
*********************************************************************/
int da213_driver_set_xy_swap(bool swap)
{
    return da213_reg_mask_write(DA213_REG_SWAP_POLARITY, 0x01, swap ? 0x01 : 0x00);
}

/********************************************************************
**函数名称:  da213_driver_set_int_pin_config
**入口参数:  pin         ---        中断引脚选择（输入）
**          config      ---        中断引脚配置（输入）
**出口参数:  无
**函数功能:  配置 DA213 中断引脚的输出类型和有效电平
**返回值:    0 成功，负值 errno 失败
*********************************************************************/
int da213_driver_set_int_pin_config(da213_int_pin_t pin, const da213_int_pin_config_t *config)
{
    uint8_t mask;
    uint8_t val;

    if (config == NULL)
    {
        return -EINVAL;
    }

    if (pin == DA213_INT_PIN_1)
    {
        mask = 0x03;
        val = ((uint8_t)config->output_type << 1) | (uint8_t)config->active_level;
    }
    else
    {
        mask = 0x0C;
        val = ((uint8_t)config->output_type << 3) | ((uint8_t)config->active_level << 2);
    }

    return da213_reg_mask_write(DA213_REG_INT_CONFIG, mask, val);
}

/********************************************************************
**函数名称:  da213_driver_set_int_latch
**入口参数:  latch_mode  ---        中断锁存模式（输入）
**出口参数:  无
**函数功能:  设置 DA213 中断锁存模式
**返回值:    0 成功，负值 errno 失败
*********************************************************************/
int da213_driver_set_int_latch(da213_int_latch_t latch_mode)
{
    return da213_reg_mask_write(DA213_REG_INT_LATCH, 0x0F, (uint8_t)latch_mode);
}

/********************************************************************
**函数名称:  da213_driver_register_int_callback
**入口参数:  callback     ---        DA213 中断回调函数（输入）
**出口参数:  无
**函数功能:  注册 DA213 中断 GPIO 回调
**返回值:    0 成功，负值 errno 失败
*********************************************************************/
int da213_driver_register_int_callback(da213_int_callback_t callback)
{
    int ret;

    if (callback == NULL)
    {
        return -EINVAL;
    }

    if (device_is_ready(s_int_gpio.port) != true)
    {
        return -ENODEV;
    }

    s_int_callback = callback;

    ret = gpio_pin_configure_dt(&s_int_gpio, GPIO_INPUT);
    if (ret < 0)
    {
        return ret;
    }

    ret = gpio_pin_interrupt_configure_dt(&s_int_gpio, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret < 0)
    {
        return ret;
    }

    if (s_int_callback_added != true)
    {
        gpio_init_callback(&s_int_gpio_cb, da213_driver_int_isr, BIT(s_int_gpio.pin));
        ret = gpio_add_callback(s_int_gpio.port, &s_int_gpio_cb);
        if (ret < 0)
        {
            return ret;
        }

        s_int_callback_added = true;
    }

    return 0;
}

/********************************************************************
**函数名称:  da213_driver_reset_int
**入口参数:  无
**出口参数:  无
**函数功能:  复位 DA213 所有锁存中断
**返回值:    0 成功，负值 errno 失败
*********************************************************************/
int da213_driver_reset_int(void)
{
    return da213_reg_mask_write(DA213_REG_INT_LATCH, 0x80, 0x80);
}

/********************************************************************
**函数名称:  da213_driver_enable_newdata_int
**入口参数:  pin         ---        中断引脚选择（输入）
**          enable      ---        使能/禁用（输入）
**出口参数:  无
**函数功能:  使能或禁用新数据就绪中断并映射到指定引脚
**返回值:    0 成功，负值 errno 失败
*********************************************************************/
int da213_driver_enable_newdata_int(da213_int_pin_t pin, bool enable)
{
    int ret;

    /* 使能/禁用 new data 中断 */
    ret = da213_reg_mask_write(DA213_REG_INT_SET2, 0x10, enable ? 0x10 : 0x00);
    if (ret < 0)
    {
        return ret;
    }

    /* 映射到对应引脚 */
    if (pin == DA213_INT_PIN_1)
    {
        ret = da213_reg_mask_write(DA213_REG_INT_MAP2, 0x01, enable ? 0x01 : 0x00);
    }
    else
    {
        ret = da213_reg_mask_write(DA213_REG_INT_MAP2, 0x80, enable ? 0x80 : 0x00);
    }

    return ret;
}

/********************************************************************
**函数名称:  da213_driver_config_active_int
**入口参数:  config      ---        运动检测中断配置（输入）
**出口参数:  无
**函数功能:  配置 DA213 Active（运动检测）中断参数并使能
**返回值:    0 成功，负值 errno 失败
*********************************************************************/
int da213_driver_config_active_int(const da213_active_int_config_t *config)
{
    int ret;
    uint8_t int_set1_val;

    if (config == NULL)
    {
        return -EINVAL;
    }

    /* 设置持续时间 */
    ret = da213_reg_write(DA213_REG_ACTIVE_DUR, config->duration & 0x03);
    if (ret < 0)
    {
        return ret;
    }

    /* 设置阈值 */
    ret = da213_reg_write(DA213_REG_ACTIVE_THS, config->threshold);
    if (ret < 0)
    {
        return ret;
    }

    /* 使能各轴 */
    int_set1_val = 0;
    if (config->enable_x)
    {
        int_set1_val |= 0x01;
    }
    if (config->enable_y)
    {
        int_set1_val |= 0x02;
    }
    if (config->enable_z)
    {
        int_set1_val |= 0x04;
    }
    ret = da213_reg_mask_write(DA213_REG_INT_SET1, 0x07, int_set1_val);
    if (ret < 0)
    {
        return ret;
    }

    /* 映射到中断引脚 */
    if (config->int_pin == DA213_INT_PIN_1)
    {
        ret = da213_reg_mask_write(DA213_REG_INT_MAP1, 0x04, 0x04);
    }
    else
    {
        ret = da213_reg_mask_write(DA213_REG_INT_MAP3, 0x04, 0x04);
    }

    return ret;
}

/********************************************************************
**函数名称:  da213_driver_disable_active_int
**入口参数:  无
**出口参数:  无
**函数功能:  禁用 DA213 Active（运动检测）中断
**返回值:    0 成功，负值 errno 失败
*********************************************************************/
int da213_driver_disable_active_int(void)
{
    int ret;

    /* 禁用各轴 active 中断使能 */
    ret = da213_reg_mask_write(DA213_REG_INT_SET1, 0x07, 0x00);
    if (ret < 0)
    {
        return ret;
    }

    /* 清除中断映射 */
    ret = da213_reg_mask_write(DA213_REG_INT_MAP1, 0x04, 0x00);
    if (ret < 0)
    {
        return ret;
    }

    ret = da213_reg_mask_write(DA213_REG_INT_MAP3, 0x04, 0x00);

    return ret;
}

/********************************************************************
**函数名称:  da213_driver_config_tap_int
**入口参数:  config      ---        敲击检测中断配置（输入）
**出口参数:  无
**函数功能:  配置 DA213 Tap（敲击检测）中断参数并使能
**返回值:    0 成功，负值 errno 失败
*********************************************************************/
int da213_driver_config_tap_int(const da213_tap_int_config_t *config)
{
    int ret;
    uint8_t tap_dur_val;
    uint8_t int_set1_val;

    if (config == NULL)
    {
        return -EINVAL;
    }

    /* 配置 TAP_DUR 寄存器：tap_quiet[7] | tap_shock[6] | tap_dur[2:0] */
    tap_dur_val = ((config->quiet & 0x01) << 7) |
                  ((config->shock & 0x01) << 6) |
                  (config->duration & 0x07);
    ret = da213_reg_write(DA213_REG_TAP_DUR, tap_dur_val);
    if (ret < 0)
    {
        return ret;
    }

    /* 配置 TAP_THS 寄存器 */
    ret = da213_reg_mask_write(DA213_REG_TAP_THS, 0x1F, config->threshold & 0x1F);
    if (ret < 0)
    {
        return ret;
    }

    /* 使能单击/双击中断 */
    int_set1_val = 0;
    if (config->enable_single)
    {
        int_set1_val |= 0x20;
    }
    if (config->enable_double)
    {
        int_set1_val |= 0x10;
    }
    ret = da213_reg_mask_write(DA213_REG_INT_SET1, 0x30, int_set1_val);
    if (ret < 0)
    {
        return ret;
    }

    /* 映射到中断引脚 */
    if (config->int_pin == DA213_INT_PIN_1)
    {
        ret = da213_reg_mask_write(DA213_REG_INT_MAP1, 0x30,
                                   (config->enable_single ? 0x20 : 0x00) |
                                   (config->enable_double ? 0x10 : 0x00));
    }
    else
    {
        ret = da213_reg_mask_write(DA213_REG_INT_MAP3, 0x30,
                                   (config->enable_single ? 0x20 : 0x00) |
                                   (config->enable_double ? 0x10 : 0x00));
    }

    return ret;
}

/********************************************************************
**函数名称:  da213_driver_disable_tap_int
**入口参数:  无
**出口参数:  无
**函数功能:  禁用 DA213 Tap（敲击检测）中断
**返回值:    0 成功，负值 errno 失败
*********************************************************************/
int da213_driver_disable_tap_int(void)
{
    int ret;

    ret = da213_reg_mask_write(DA213_REG_INT_SET1, 0x30, 0x00);
    if (ret < 0)
    {
        return ret;
    }

    ret = da213_reg_mask_write(DA213_REG_INT_MAP1, 0x30, 0x00);
    if (ret < 0)
    {
        return ret;
    }

    ret = da213_reg_mask_write(DA213_REG_INT_MAP3, 0x30, 0x00);

    return ret;
}

/********************************************************************
**函数名称:  da213_driver_config_freefall_int
**入口参数:  config      ---        自由落体中断配置（输入）
**出口参数:  无
**函数功能:  配置 DA213 Freefall（自由落体）中断参数并使能
**返回值:    0 成功，负值 errno 失败
*********************************************************************/
int da213_driver_config_freefall_int(const da213_freefall_int_config_t *config)
{
    int ret;
    uint8_t hyst_val;

    if (config == NULL)
    {
        return -EINVAL;
    }

    /* 设置持续时间 */
    ret = da213_reg_write(DA213_REG_FREEFALL_DUR, config->duration);
    if (ret < 0)
    {
        return ret;
    }

    /* 设置阈值 */
    ret = da213_reg_write(DA213_REG_FREEFALL_THS, config->threshold);
    if (ret < 0)
    {
        return ret;
    }

    /* 设置迟滞和模式 */
    hyst_val = ((uint8_t)config->mode << 2) | (config->hysteresis & 0x03);
    ret = da213_reg_mask_write(DA213_REG_FREEFALL_HYST, 0x07, hyst_val);
    if (ret < 0)
    {
        return ret;
    }

    /* 使能 freefall 中断 */
    ret = da213_reg_mask_write(DA213_REG_INT_SET2, 0x08, 0x08);
    if (ret < 0)
    {
        return ret;
    }

    /* 映射到中断引脚 */
    if (config->int_pin == DA213_INT_PIN_1)
    {
        ret = da213_reg_mask_write(DA213_REG_INT_MAP1, 0x01, 0x01);
    }
    else
    {
        ret = da213_reg_mask_write(DA213_REG_INT_MAP3, 0x01, 0x01);
    }

    return ret;
}

/********************************************************************
**函数名称:  da213_driver_disable_freefall_int
**入口参数:  无
**出口参数:  无
**函数功能:  禁用 DA213 Freefall（自由落体）中断
**返回值:    0 成功，负值 errno 失败
*********************************************************************/
int da213_driver_disable_freefall_int(void)
{
    int ret;

    ret = da213_reg_mask_write(DA213_REG_INT_SET2, 0x08, 0x00);
    if (ret < 0)
    {
        return ret;
    }

    ret = da213_reg_mask_write(DA213_REG_INT_MAP1, 0x01, 0x00);
    if (ret < 0)
    {
        return ret;
    }

    ret = da213_reg_mask_write(DA213_REG_INT_MAP3, 0x01, 0x00);

    return ret;
}

/********************************************************************
**函数名称:  da213_driver_config_orient_int
**入口参数:  config      ---        方向识别中断配置（输入）
**出口参数:  无
**函数功能:  配置 DA213 Orient（方向识别）中断参数并使能
**返回值:    0 成功，负值 errno 失败
*********************************************************************/
int da213_driver_config_orient_int(const da213_orient_int_config_t *config)
{
    int ret;
    uint8_t orient_hyst_val;

    if (config == NULL)
    {
        return -EINVAL;
    }

    /* 配置 ORIENT_HYST 寄存器：orient_hyst[6:4] | orient_block[3:2] | orient_mode[1:0] */
    orient_hyst_val = ((config->hysteresis & 0x07) << 4) |
                      ((uint8_t)config->blocking << 2) |
                      (uint8_t)config->mode;
    ret = da213_reg_write(DA213_REG_ORIENT_HYST, orient_hyst_val);
    if (ret < 0)
    {
        return ret;
    }

    /* 配置 Z_BLOCK 寄存器 */
    ret = da213_reg_mask_write(DA213_REG_Z_BLOCK, 0x0F, config->z_blocking & 0x0F);
    if (ret < 0)
    {
        return ret;
    }

    /* 使能 orient 中断 */
    ret = da213_reg_mask_write(DA213_REG_INT_SET1, 0x40, 0x40);
    if (ret < 0)
    {
        return ret;
    }

    /* 映射到中断引脚 */
    if (config->int_pin == DA213_INT_PIN_1)
    {
        ret = da213_reg_mask_write(DA213_REG_INT_MAP1, 0x40, 0x40);
    }
    else
    {
        ret = da213_reg_mask_write(DA213_REG_INT_MAP3, 0x40, 0x40);
    }

    return ret;
}

/********************************************************************
**函数名称:  da213_driver_disable_orient_int
**入口参数:  无
**出口参数:  无
**函数功能:  禁用 DA213 Orient（方向识别）中断
**返回值:    0 成功，负值 errno 失败
*********************************************************************/
int da213_driver_disable_orient_int(void)
{
    int ret;

    ret = da213_reg_mask_write(DA213_REG_INT_SET1, 0x40, 0x00);
    if (ret < 0)
    {
        return ret;
    }

    ret = da213_reg_mask_write(DA213_REG_INT_MAP1, 0x40, 0x00);
    if (ret < 0)
    {
        return ret;
    }

    ret = da213_reg_mask_write(DA213_REG_INT_MAP3, 0x40, 0x00);

    return ret;
}

/********************************************************************
**函数名称:  da213_driver_read_motion_flag
**入口参数:  flag        ---        运动标志存储指针（输出）
**出口参数:  flag        ---        存储运动标志寄存器值
**函数功能:  读取 DA213 运动标志寄存器，判断哪些中断被触发
**返回值:    0 成功，负值 errno 失败
*********************************************************************/
int da213_driver_read_motion_flag(uint8_t *flag)
{
    if (flag == NULL)
    {
        return -EINVAL;
    }

    return da213_reg_read(DA213_REG_MOTION_FLAG, flag, 1);
}

/********************************************************************
**函数名称:  da213_driver_read_newdata_flag
**入口参数:  new_data    ---        新数据标志存储指针（输出）
**出口参数:  new_data    ---        true表示有新数据
**函数功能:  读取 DA213 新数据就绪标志
**返回值:    0 成功，负值 errno 失败
*********************************************************************/
int da213_driver_read_newdata_flag(bool *new_data)
{
    int ret;
    uint8_t reg_val;

    if (new_data == NULL)
    {
        return -EINVAL;
    }

    ret = da213_reg_read(DA213_REG_NEWDATA_FLAG, &reg_val, 1);
    if (ret < 0)
    {
        return ret;
    }

    *new_data = (reg_val & 0x01) ? true : false;

    return 0;
}

/********************************************************************
**函数名称:  da213_driver_read_tap_active_status
**入口参数:  status      ---        Tap/Active 状态存储指针（输出）
**出口参数:  status      ---        存储 Tap 和 Active 触发状态详情
**函数功能:  读取 DA213 TAP_ACTIVE_STATUS 寄存器，获取触发轴和方向信息
**返回值:    0 成功，负值 errno 失败
*********************************************************************/
int da213_driver_read_tap_active_status(da213_tap_active_status_t *status)
{
    int ret;
    uint8_t reg_val;

    if (status == NULL)
    {
        return -EINVAL;
    }

    ret = da213_reg_read(DA213_REG_TAP_ACTIVE_STATUS, &reg_val, 1);
    if (ret < 0)
    {
        return ret;
    }

    status->tap_sign_negative = (reg_val & DA213_TAP_SIGN_NEGATIVE) ? true : false;
    status->tap_first_x = (reg_val & DA213_TAP_FIRST_X) ? true : false;
    status->tap_first_y = (reg_val & DA213_TAP_FIRST_Y) ? true : false;
    status->tap_first_z = (reg_val & DA213_TAP_FIRST_Z) ? true : false;
    status->active_sign_negative = (reg_val & DA213_ACTIVE_SIGN_NEGATIVE) ? true : false;
    status->active_first_x = (reg_val & DA213_ACTIVE_FIRST_X) ? true : false;
    status->active_first_y = (reg_val & DA213_ACTIVE_FIRST_Y) ? true : false;
    status->active_first_z = (reg_val & DA213_ACTIVE_FIRST_Z) ? true : false;

    return 0;
}

/********************************************************************
**函数名称:  da213_driver_read_orient_status
**入口参数:  status      ---        方向状态存储指针（输出）
**出口参数:  status      ---        存储方向识别状态
**函数功能:  读取 DA213 ORIENT_STATUS 寄存器，获取设备当前朝向
**返回值:    0 成功，负值 errno 失败
*********************************************************************/
int da213_driver_read_orient_status(da213_orient_status_t *status)
{
    int ret;
    uint8_t reg_val;

    if (status == NULL)
    {
        return -EINVAL;
    }

    ret = da213_reg_read(DA213_REG_ORIENT_STATUS, &reg_val, 1);
    if (ret < 0)
    {
        return ret;
    }

    status->orient_z = (reg_val >> 6) & 0x01;
    status->orient_xy = (reg_val >> 4) & 0x03;

    return 0;
}

/********************************************************************
**函数名称:  da213_driver_set_offset
**入口参数:  offset_x    ---        X轴偏移补偿值（输入）
**          offset_y    ---        Y轴偏移补偿值（输入）
**          offset_z    ---        Z轴偏移补偿值（输入）
**出口参数:  无
**函数功能:  设置 DA213 自定义偏移补偿值，LSB=3.9mg
**返回值:    0 成功，负值 errno 失败
*********************************************************************/
int da213_driver_set_offset(int8_t offset_x, int8_t offset_y, int8_t offset_z)
{
    int ret;

    ret = da213_reg_write(DA213_REG_CUSTOM_OFFSET_X, (uint8_t)offset_x);
    if (ret < 0)
    {
        return ret;
    }

    ret = da213_reg_write(DA213_REG_CUSTOM_OFFSET_Y, (uint8_t)offset_y);
    if (ret < 0)
    {
        return ret;
    }

    ret = da213_reg_write(DA213_REG_CUSTOM_OFFSET_Z, (uint8_t)offset_z);

    return ret;
}

/********************************************************************
**函数名称:  da213_driver_get_offset
**入口参数:  offset_x    ---        X轴偏移存储指针（输出）
**          offset_y    ---        Y轴偏移存储指针（输出）
**          offset_z    ---        Z轴偏移存储指针（输出）
**出口参数:  offset_x/y/z ---      存储当前偏移补偿值
**函数功能:  读取 DA213 当前自定义偏移补偿值
**返回值:    0 成功，负值 errno 失败
*********************************************************************/
int da213_driver_get_offset(int8_t *offset_x, int8_t *offset_y, int8_t *offset_z)
{
    int ret;

    if (offset_x == NULL || offset_y == NULL || offset_z == NULL)
    {
        return -EINVAL;
    }

    ret = da213_reg_read(DA213_REG_CUSTOM_OFFSET_X, (uint8_t *)offset_x, 1);
    if (ret < 0)
    {
        return ret;
    }

    ret = da213_reg_read(DA213_REG_CUSTOM_OFFSET_Y, (uint8_t *)offset_y, 1);
    if (ret < 0)
    {
        return ret;
    }

    ret = da213_reg_read(DA213_REG_CUSTOM_OFFSET_Z, (uint8_t *)offset_z, 1);

    return ret;
}
