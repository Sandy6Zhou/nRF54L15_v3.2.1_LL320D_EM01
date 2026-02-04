/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        my_motor.c
**文件描述:        电机控制模块实现文件
**当前版本:        V1.0
**作    者:        周森达 (zhousenda@jimiiot.com)
**完成日期:        2026.02.04
*********************************************************************
** 功能描述:        1. 电机正反转、对应电源使能及检测中断
*********************************************************************/

#include "my_comm.h"

LOG_MODULE_REGISTER(my_motor, LOG_LEVEL_INF);

static const struct gpio_dt_spec motor_a = GPIO_DT_SPEC_GET(DT_ALIAS(motor_ctrl_a), gpios);
static const struct gpio_dt_spec motor_b = GPIO_DT_SPEC_GET(DT_ALIAS(motor_ctrl_b), gpios);
static const struct gpio_dt_spec motor_pwr_en = GPIO_DT_SPEC_GET(DT_ALIAS(motor_pwr_ctrl), gpios);
static const struct gpio_dt_spec motor_pos_a = GPIO_DT_SPEC_GET(DT_ALIAS(motor_posdet_a), gpios);
static const struct gpio_dt_spec motor_pos_b = GPIO_DT_SPEC_GET(DT_ALIAS(motor_posdet_b), gpios);

static struct gpio_callback motor_pos_cb;

/* 电源控制：1 = 打开，0 = 关闭 */
void motor_power_set(bool on)
{
    LOG_INF("%s:%d", __func__, on);
    gpio_pin_set_dt(&motor_pwr_en, on ? 1 : 0);
}

void motor_forward(void)
{
    LOG_INF("%s:run", __func__);
    /* A 高，B 低 -> 正转（具体取决于你的驱动电路） */
    gpio_pin_set_dt(&motor_a, 1);
    gpio_pin_set_dt(&motor_b, 0);
}

void motor_reverse(void)
{
    LOG_INF("%s:run", __func__);
    /* A 低，B 高 -> 反转 */
    gpio_pin_set_dt(&motor_a, 0);
    gpio_pin_set_dt(&motor_b, 1);
}

void motor_stop(void)
{
    LOG_INF("%s:run", __func__);
    /* 关掉方向输出 */
    gpio_pin_set_dt(&motor_a, 0);
    gpio_pin_set_dt(&motor_b, 0);
}

static void motor_pos_isr(const struct device *dev,
                        struct gpio_callback *cb,
                        uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);

    int level;

    if (pins & BIT(motor_pos_a.pin))
    {
        level = gpio_pin_get_dt(&motor_pos_a);
        //TODO 开锁
        // LOG_INF("open lock:%d", level);
        //TODO 发消息给ctrl去再次获取该引脚电平状态，以及消抖处理
    }

    if (pins & BIT(motor_pos_b.pin))
    {
        level = gpio_pin_get_dt(&motor_pos_b);
        //TODO 关锁
        // LOG_INF("close lock:%d", level);
        //TODO 发消息给ctrl去再次获取该引脚电平状态，以及消抖处理
    }
}

int motor_gpio_init(void)
{
    int ret;

    /* 检查所有端口是否就绪 */
    if (!device_is_ready(motor_a.port) ||
        !device_is_ready(motor_b.port) ||
        !device_is_ready(motor_pwr_en.port) ||
        !device_is_ready(motor_pos_a.port) ||
        !device_is_ready(motor_pos_b.port))
    {
        return -ENODEV;
    }

    /* 输出引脚配置：P2.0 / P2.1 / P2.5 */
    ret = gpio_pin_configure_dt(&motor_a, GPIO_OUTPUT_INACTIVE);
    if (ret) return ret;

    ret = gpio_pin_configure_dt(&motor_b, GPIO_OUTPUT_INACTIVE);
    if (ret) return ret;

    /* 电源默认关闭 */
    ret = gpio_pin_configure_dt(&motor_pwr_en, GPIO_OUTPUT_INACTIVE);
    if (ret) return ret;

    /* 输入引脚配置：P0.2 / P0.3，默认上拉，高电平，下降沿触发 */
    ret = gpio_pin_configure_dt(&motor_pos_a, GPIO_INPUT);
    if (ret) return ret;

    ret = gpio_pin_configure_dt(&motor_pos_b, GPIO_INPUT);
    if (ret) return ret;

    /* 配置中断为下降沿触发 */
    ret = gpio_pin_interrupt_configure_dt(&motor_pos_a, GPIO_INT_EDGE_FALLING);
    if (ret) return ret;

    ret = gpio_pin_interrupt_configure_dt(&motor_pos_b, GPIO_INT_EDGE_FALLING);
    if (ret) return ret;

    /* 注册回调 */
    gpio_init_callback(&motor_pos_cb, motor_pos_isr, BIT(motor_pos_a.pin) | BIT(motor_pos_b.pin));
    gpio_add_callback(motor_pos_a.port, &motor_pos_cb);

    return 0;
}