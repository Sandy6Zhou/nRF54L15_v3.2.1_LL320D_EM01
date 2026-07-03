#include "bmi325_port.h"

#include <errno.h>
#include <stdbool.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>

#define IMU_I2C_NODE DT_ALIAS(gsensor_i2c)
#define IMU_INT_NODE DT_ALIAS(gsensor_int)      /* 当前硬件仅接出 1 路 IMU 中断 GPIO，INT2 预留 */

static const struct device *s_i2c_dev = DEVICE_DT_GET(IMU_I2C_NODE);
static const struct gpio_dt_spec s_int_gpio = GPIO_DT_SPEC_GET(IMU_INT_NODE, gpios);
static struct bmi3_dev s_bmi3_dev;
static struct gpio_callback s_int_gpio_cb;
static uint8_t s_bmi325_i2c_addr = BMI3_ADDR_I2C_PRIM;
static bool s_port_inited = false;
static bool s_int_callback_added = false;
static void (*s_int_callback)(void) = NULL;
/* 当前项目约定所有 IMU API 均在同一线程串行调用，此处不额外增加锁保护。 */

/********************************************************************
**函数名称:  bmi325_port_i2c_read
**入口参数:  reg_addr ---        寄存器地址（输入）
**           reg_data ---        读数据缓冲区（输出）
**           length   ---        读取长度（输入）
**           intf_ptr ---        接口上下文（输入）
**出口参数:  reg_data ---        读取到的寄存器数据
**函数功能:  提供给原厂 BMI325 驱动的 I2C 读回调
**返回值:    0 表示成功，非 0 表示失败
*********************************************************************/
static BMI3_INTF_RET_TYPE bmi325_port_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    const struct device *i2c_dev;

    i2c_dev = (const struct device *)intf_ptr;
    if ((i2c_dev == NULL) || (reg_data == NULL) || (length == 0U))
    {
        return (BMI3_INTF_RET_TYPE)-EINVAL;
    }

    return (BMI3_INTF_RET_TYPE)i2c_write_read(i2c_dev, s_bmi325_i2c_addr, &reg_addr, 1, reg_data, length);
}

/********************************************************************
**函数名称:  bmi325_port_i2c_write
**入口参数:  reg_addr ---        寄存器地址（输入）
**           reg_data ---        写数据缓冲区（输入）
**           length   ---        写入长度（输入）
**           intf_ptr ---        接口上下文（输入）
**出口参数:  无
**函数功能:  提供给原厂 BMI325 驱动的 I2C 写回调
**返回值:    0 表示成功，非 0 表示失败
*********************************************************************/
static BMI3_INTF_RET_TYPE bmi325_port_i2c_write(uint8_t reg_addr,
                                                const uint8_t *reg_data,
                                                uint32_t length,
                                                void *intf_ptr)
{
    const struct device *i2c_dev;
    uint8_t tx_buf[64];

    i2c_dev = (const struct device *)intf_ptr;
    if ((i2c_dev == NULL) || (reg_data == NULL) || (length == 0U) || (length > (sizeof(tx_buf) - 1U)))
    {
        return (BMI3_INTF_RET_TYPE)-EINVAL;
    }

    tx_buf[0] = reg_addr;
    memcpy(&tx_buf[1], reg_data, length);

    return (BMI3_INTF_RET_TYPE)i2c_write(i2c_dev, tx_buf, length + 1U, s_bmi325_i2c_addr);
}

/********************************************************************
**函数名称:  bmi325_port_delay_us
**入口参数:  period_us ---       延时时间，单位微秒（输入）
**           intf_ptr  ---       接口上下文（输入）
**出口参数:  无
**函数功能:  提供给原厂 BMI325 驱动的延时回调
**返回值:    无
*********************************************************************/
static void bmi325_port_delay_us(uint32_t period_us, void *intf_ptr)
{
    ARG_UNUSED(intf_ptr);

    if (period_us > 0U)
    {
        k_usleep(period_us);
    }
}

/********************************************************************
**函数名称:  bmi325_port_int_isr
**入口参数:  dev      ---        GPIO 设备指针（输入）
**           cb       ---        GPIO 回调结构体（输入）
**           pins     ---        中断引脚位掩码（输入）
**出口参数:  无
**函数功能:  BMI325 INT GPIO 中断服务函数
**返回值:    无
*********************************************************************/
static void bmi325_port_int_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
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
**函数名称:  bmi325_port_init_dev
**入口参数:  无
**出口参数:  无
**函数功能:  初始化 BMI325 Zephyr 端口层设备上下文
**返回值:    0 表示成功，负值表示失败
*********************************************************************/
int bmi325_port_init_dev(void)
{
    if (device_is_ready(s_i2c_dev) != true)
    {
        return -ENODEV;
    }

    memset(&s_bmi3_dev, 0, sizeof(s_bmi3_dev));
    s_bmi3_dev.intf = BMI3_I2C_INTF;
    s_bmi3_dev.intf_ptr = (void *)s_i2c_dev;
    s_bmi3_dev.read = bmi325_port_i2c_read;
    s_bmi3_dev.write = bmi325_port_i2c_write;
    s_bmi3_dev.delay_us = bmi325_port_delay_us;
    s_bmi3_dev.read_write_len = 32U;
    s_port_inited = true;

    return 0;
}

/********************************************************************
**函数名称:  bmi325_port_get_dev
**入口参数:  无
**出口参数:  无
**函数功能:  获取 BMI325 原厂驱动设备上下文指针
**返回值:    成功返回设备上下文指针，失败返回 NULL
*********************************************************************/
struct bmi3_dev *bmi325_port_get_dev(void)
{
    if (s_port_inited != true)
    {
        return NULL;
    }

    return &s_bmi3_dev;
}

/********************************************************************
**函数名称:  bmi325_port_register_int_callback
**入口参数:  callback ---        中断回调函数（输入）
**出口参数:  无
**函数功能:  注册 BMI325 INT GPIO 中断回调
**返回值:    0 表示成功，负值表示失败
*********************************************************************/
int bmi325_port_register_int_callback(void (*callback)(void))
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
        gpio_init_callback(&s_int_gpio_cb, bmi325_port_int_isr, BIT(s_int_gpio.pin));
        ret = gpio_add_callback(s_int_gpio.port, &s_int_gpio_cb);
        if (ret < 0)
        {
            return ret;
        }

        s_int_callback_added = true;
    }

    return 0;
}
