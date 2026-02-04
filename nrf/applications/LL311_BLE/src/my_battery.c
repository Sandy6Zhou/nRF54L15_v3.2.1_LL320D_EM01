/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        my_battery.c
**文件描述:        电池管理模块实现文件
**当前版本:        V1.0
**作    者:        周森达 (zhousenda@jimiiot.com)
**完成日期:        2026.02.04
*********************************************************************
** 功能描述:        1. 电池电压、NTC采集以及电源开关使能
**                 2. 电池状态、充电状态检测
*********************************************************************/

#include "my_comm.h"

LOG_MODULE_REGISTER(my_battery, LOG_LEVEL_INF);

/* zephyr,user 里有两个 io-channels：index 0 = 通道0(电池)，index 1 = 通道1(NTC) */
#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)
/* 通道 0：电池电压 */
static const struct adc_dt_spec batt_adc = ADC_DT_SPEC_GET_BY_IDX(ZEPHYR_USER_NODE, 0);
/* 通道 1：NTC 温度 */
static const struct adc_dt_spec ntc_adc = ADC_DT_SPEC_GET_BY_IDX(ZEPHYR_USER_NODE, 1);

static const struct gpio_dt_spec batt_pwr_en = GPIO_DT_SPEC_GET(DT_ALIAS(batt_pwr_ctrl), gpios);
static const struct gpio_dt_spec charge_state = GPIO_DT_SPEC_GET(DT_ALIAS(batt_state), gpios);
static const struct gpio_dt_spec charge_det = GPIO_DT_SPEC_GET(DT_ALIAS(charge_detect), gpios);

static struct gpio_callback batt_gpio_cb;
// ADC原始采样值
static int16_t batt_raw;
static int16_t ntc_raw;

static struct adc_sequence batt_seq = {
    .buffer = &batt_raw,
    .buffer_size = sizeof(batt_raw),
};

static struct adc_sequence ntc_seq = {
    .buffer = &ntc_raw,
    .buffer_size = sizeof(ntc_raw),
};

/* 读取电池电压原始值，并转换为 mV（不含分压系数） */
int batt_read_mv(int32_t *mv)
{
    int err;
    int32_t val_mv;

    err = adc_read(batt_adc.dev, &batt_seq);
    if (err < 0) return err;

    val_mv = batt_raw;
    // TODO 后续需取均值，以及开关ADC处理
    err = adc_raw_to_millivolts_dt(&batt_adc, &val_mv);
    if (err < 0) return err;

    *mv = val_mv;
    return 0;
}

/* 读取 NTC 原始 ADC 值（后续可在应用层根据电阻/温度表换算） */
int ntc_read_raw(int16_t *raw)
{
    int err;

    err = adc_read(ntc_adc.dev, &ntc_seq);
    if (err < 0) return err;

    *raw = ntc_raw;
    return 0;
}

int batt_adc_init(void)
{
    int err;

    /* 检查 ADC 是否就绪 */
    if (!adc_is_ready_dt(&batt_adc))
    {
        return -ENODEV;
    }

    /* 配置两个通道（配置来自 devicetree） */
    err = adc_channel_setup_dt(&batt_adc);
    if (err < 0) return err;

    err = adc_channel_setup_dt(&ntc_adc);
    if (err < 0) return err;

    /* 初始化 sequence（会填充通道掩码等） */
    err = adc_sequence_init_dt(&batt_adc, &batt_seq);
    if (err < 0) return err;

    err = adc_sequence_init_dt(&ntc_adc, &ntc_seq);
    if (err < 0) return err;

    return 0;
}

/* 电池使能：true=打开，false=关闭 */
void batt_enable(bool on)
{
    LOG_INF("%s:%d", __func__, on);
    gpio_pin_set_dt(&batt_pwr_en, on ? 1 : 0);
}

/* 共用 ISR：根据 pins 掩码区分是 P1.7 还是 P1.8 触发 */
static void batt_gpio_isr(const struct device *dev,
                        struct gpio_callback *cb, 
                        uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    int level;

    if (pins & BIT(charge_state.pin))
    {
        /* P1.7 触发：电池状态变化（充电中 / 充满） */
        level = gpio_pin_get_dt(&charge_state);
        // LOG_INF("charge_state:%d", level);
        //TODO 发消息给ctrl去再次获取该引脚电平状态，以及消抖处理
    }

    if (pins & BIT(charge_det.pin))
    {
        /* P1.8 触发：充电检测变化（开始充电 / 停止充电） */
        level = gpio_pin_get_dt(&charge_det);
        // LOG_INF("charge_det:%d", level);
        //TODO 发消息给ctrl去再次获取该引脚电平状态，以及消抖处理
    }
}

int batt_gpio_init(void)
{
    int ret;

    if (!device_is_ready(batt_pwr_en.port) ||
        !device_is_ready(charge_state.port) ||
        !device_is_ready(charge_det.port))
    {
        return -ENODEV;
    }

    /* 使能口：输出，默认关闭 */
    ret = gpio_pin_configure_dt(&batt_pwr_en, GPIO_OUTPUT_INACTIVE);
    if (ret) return ret;

    /* 状态/充电检测：输入（上拉在 DTS 里已经配置） */
    ret = gpio_pin_configure_dt(&charge_state, GPIO_INPUT);
    if (ret) return ret;

    ret = gpio_pin_configure_dt(&charge_det, GPIO_INPUT);
    if (ret) return ret;

    /* 配置中断：
     * 根据硬件逻辑选择：
     *  - GPIO_INT_EDGE_FALLING：只在拉低时触发
     *  - GPIO_INT_EDGE_RISING：只在拉高时触发
     *  - GPIO_INT_EDGE_BOTH：高/低变化都触发
     */
    ret = gpio_pin_interrupt_configure_dt(&charge_state, GPIO_INT_EDGE_BOTH);
    if (ret) return ret;

    ret = gpio_pin_interrupt_configure_dt(&charge_det, GPIO_INT_EDGE_BOTH);
    if (ret) return ret;

    /* 一个回调处理两个引脚 */
    gpio_init_callback(&batt_gpio_cb, batt_gpio_isr,
                       BIT(charge_state.pin) | BIT(charge_det.pin));
    gpio_add_callback(charge_state.port, &batt_gpio_cb);

    return 0;
}