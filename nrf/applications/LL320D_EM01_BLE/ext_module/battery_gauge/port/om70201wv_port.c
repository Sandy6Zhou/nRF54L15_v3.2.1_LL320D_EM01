/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        om70201wv_port.c
**文件描述:        OM70201WV Zephyr 平台适配实现文件
**当前版本:        V1.0
**作    者:       周森达 (zhousenda@jimiiot.com)
**完成日期:        2026.07.21
*********************************************************************
** 功能描述:       适配 Zephyr I2C、INTN GPIO 和毫秒延时接口
**                 为厂家驱动提供寄存器访问和中断回调能力
*********************************************************************/

#include "om70201wv_port.h"

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>

#define OM70201WV_I2C_NODE DT_ALIAS(battery_gauge_i2c)
#define OM70201WV_INTERRUPT_NODE DT_ALIAS(battery_gauge_int)
#define OM70201WV_I2C_ADDRESS 0x38U
#define OM70201WV_WRITE_BUFFER_SIZE 33U
static const struct device *s_i2c_device = DEVICE_DT_GET(OM70201WV_I2C_NODE);
static const struct gpio_dt_spec s_interrupt_gpio = GPIO_DT_SPEC_GET(OM70201WV_INTERRUPT_NODE, gpios);
static struct gpio_callback s_interrupt_gpio_callback;
static void (*s_interrupt_callback)(void) = NULL;
static bool s_port_initialized = false;

/********************************************************************
**函数名称:  om70201wv_port_interrupt_isr
**入口参数:  device  ---        GPIO 设备指针（输入）
            callback ---        GPIO 回调结构体（输入）
            pins     ---        触发引脚掩码（输入）
**出口参数:  无
**函数功能:  处理 OM70201WV INTN 下降沿中断并通知上层
**返回值:    无
*********************************************************************/
static void om70201wv_port_interrupt_isr(const struct device *device,
                                         struct gpio_callback *callback,
                                         uint32_t pins)
{
    ARG_UNUSED(device);
    ARG_UNUSED(callback);
    ARG_UNUSED(pins);

    if (s_interrupt_callback != NULL)
    {
        s_interrupt_callback();
    }
}

/********************************************************************
**函数名称:  om70201wv_port_init
**入口参数:  无
**出口参数:  无
**函数功能:  初始化 OM70201WV 的 I2C 和中断 GPIO 端口
**返回值:    0 表示成功，负值表示失败
*********************************************************************/
int om70201wv_port_init(void)
{
    int ret;

    if (s_port_initialized == true)
    {
        return 0;
    }

    if (device_is_ready(s_i2c_device) != true)
    {
        return -ENODEV;
    }

    if (device_is_ready(s_interrupt_gpio.port) != true)
    {
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&s_interrupt_gpio, GPIO_INPUT);
    if (ret < 0)
    {
        return ret;
    }

    ret = gpio_pin_interrupt_configure_dt(&s_interrupt_gpio, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret < 0)
    {
        return ret;
    }

    gpio_init_callback(&s_interrupt_gpio_callback,
                       om70201wv_port_interrupt_isr,
                       BIT(s_interrupt_gpio.pin));
    ret = gpio_add_callback(s_interrupt_gpio.port, &s_interrupt_gpio_callback);
    if (ret < 0)
    {
        return ret;
    }

    s_port_initialized = true;

    return 0;
}

/********************************************************************
**函数名称:  om70201wv_port_read
**入口参数:  reg_addr ---        寄存器地址（输入）
            data     ---        数据缓冲区（输入）
            length   ---        读取长度（输入）
**出口参数:  data     ---        读取到的寄存器数据（输出）
**函数功能:  从 OM70201WV 连续读取寄存器数据
**返回值:    0 表示成功，负值表示失败
*********************************************************************/
int om70201wv_port_read(uint8_t reg_addr, uint8_t *data, uint16_t length)
{
    if ((s_port_initialized != true) || (data == NULL) || (length == 0U))
    {
        return -EINVAL;
    }

    return i2c_write_read(s_i2c_device,
                          OM70201WV_I2C_ADDRESS,
                          &reg_addr,
                          sizeof(reg_addr),
                          data,
                          length);
}

/********************************************************************
**函数名称:  om70201wv_port_write
**入口参数:  reg_addr ---        寄存器地址（输入）
            data     ---        待写入数据（输入）
            length   ---        写入长度（输入）
**出口参数:  无
**函数功能:  向 OM70201WV 连续写入寄存器数据
**返回值:    0 表示成功，负值表示失败
*********************************************************************/
int om70201wv_port_write(uint8_t reg_addr, const uint8_t *data, uint16_t length)
{
    uint8_t write_buffer[OM70201WV_WRITE_BUFFER_SIZE];

    if ((s_port_initialized != true) || (data == NULL) || (length == 0U))
    {
        return -EINVAL;
    }

    if (length > (sizeof(write_buffer) - 1U))
    {
        return -EMSGSIZE;
    }

    write_buffer[0] = reg_addr;
    memcpy(&write_buffer[1], data, length);

    return i2c_write(s_i2c_device,
                     write_buffer,
                     (uint32_t)length + 1U,
                     OM70201WV_I2C_ADDRESS);
}

/********************************************************************
**函数名称:  om70201wv_port_delay_ms
**入口参数:  delay_ms ---        延时时间，单位毫秒（输入）
**出口参数:  无
**函数功能:  提供 OM70201WV 驱动所需的毫秒延时
**返回值:    无
*********************************************************************/
void om70201wv_port_delay_ms(uint32_t delay_ms)
{
    if (delay_ms > 0U)
    {
        k_msleep(delay_ms);
    }
}

/********************************************************************
**函数名称:  om70201wv_port_register_interrupt_callback
**入口参数:  callback ---        中断回调函数（输入）
**出口参数:  无
**函数功能:  注册 OM70201WV INTN 中断回调函数
**返回值:    0 表示成功，负值表示失败
**注意事项:  回调在 GPIO 中断上下文执行，禁止在回调中直接访问 I2C
*********************************************************************/
int om70201wv_port_register_interrupt_callback(void (*callback)(void))
{
    if ((s_port_initialized != true) || (callback == NULL))
    {
        return -EINVAL;
    }

    s_interrupt_callback = callback;

    return 0;
}
