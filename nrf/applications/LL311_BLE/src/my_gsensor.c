/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        my_gsensor.c
**文件描述:        G-Sensor 管理模块实现文件 (DA215S)
**当前版本:        V1.0
**作    者:        Harrison Wu (wuyujiao@jimiiot.com)
**完成日期:        2026.01.15
*********************************************************************
** 功能描述:        1. 实现 DA215S 的 I2C 初始化与数据读取
**                 2. 实现电源控制逻辑 (P2.10)
**                 3. 封装通用的读取接口以兼容未来扩展
*********************************************************************/

#include "my_comm.h"

/* 注册 G-Sensor 模块日志 */
LOG_MODULE_REGISTER(my_gsensor, LOG_LEVEL_INF);

/* 从设备树获取硬件配置 */
#define I2C_NODE DT_ALIAS(gsensor_i2c)
static const struct device *i2c_dev = DEVICE_DT_GET(I2C_NODE);

#define GSENSOR_PWR_NODE DT_ALIAS(gsensor_pwr_ctrl)
static const struct gpio_dt_spec gsensor_pwr_gpio = GPIO_DT_SPEC_GET(GSENSOR_PWR_NODE, gpios);

static const struct gpio_dt_spec gsen_int = GPIO_DT_SPEC_GET(DT_ALIAS(gsensor_int), gpios);
static struct gpio_callback gsensor_gpio_cb;
/* DA215S I2C 从机地址 (通常为 0x27) */
#define DA215S_I2C_ADDR 0x27

/* DA215S 寄存器定义 */
#define DA215S_REG_CHIP_ID    0x01
#define DA215S_REG_X_LSB      0x02
#define DA215S_REG_X_MSB      0x03
#define DA215S_REG_Y_LSB      0x04
#define DA215S_REG_Y_MSB      0x05
#define DA215S_REG_Z_LSB      0x06
#define DA215S_REG_Z_MSB      0x07
#define DA215S_REG_RESOLUTION 0x0F

/* 当前识别到的传感器类型 */
static enum gsensor_type current_sensor = GSENSOR_TYPE_UNKNOWN;

/* 消息队列定义 */
K_MSGQ_DEFINE(my_gsensor_msgq, sizeof(MSG_S), 10, 4);

/* 线程数据与栈定义 */
K_THREAD_STACK_DEFINE(my_gsensor_task_stack, MY_GSENSOR_TASK_STACK_SIZE);
static struct k_thread my_gsensor_task_data;

/********************************************************************
**函数名称:  my_gsensor_task
**入口参数:  无
**出口参数:  无
**函数功能:  G-Sensor 模块主线程，处理来自消息队列的任务
**返 回 值:  无
*********************************************************************/
static void my_gsensor_task(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    MSG_S msg;

    LOG_INF("G-Sensor thread started");

    for (;;)
    {
        my_recv_msg(&my_gsensor_msgq, (void *)&msg, sizeof(MSG_S), K_FOREVER);

        switch (msg.msgID)
        {
            /* TODO: 添加 G-Sensor 相关的消息处理逻辑 */
            default:
                break;
        }
    }
}

/********************************************************************
**函数名称:  da215s_check_id
**入口参数:  无
**出口参数:  无
**函数功能:  通过 I2C 读取芯片 ID 以确认 DA215S 是否在线
**返 回 值:  true 表示识别成功，false 表示失败
*********************************************************************/
static bool da215s_check_id(void)
{
    uint8_t chip_id = 0;
    int err;

    err = i2c_reg_read_byte(i2c_dev, DA215S_I2C_ADDR, DA215S_REG_CHIP_ID, &chip_id);
    if (err)
    {
        LOG_ERR("I2C read DA215S ID failed (err %d)", err);
        return false;
    }

    LOG_INF("DA215S Chip ID: 0x%02X", chip_id);

    /* DA215S 的默认 ID 通常为 0x13 */
    if (chip_id == 0x13)
    {
        return true;
    }

    return false;
}

int my_gsensor_pwr_on(bool on)
{
    LOG_INF("GSENSOR Power Control: %s", on ? "Power ON" : "Power OFF");
    return gpio_pin_set_dt(&gsensor_pwr_gpio, on ? 1 : 0);
}

int my_gsensor_read_data(struct gsensor_data *data)
{
    uint8_t raw_data[6];
    int err;

    if (current_sensor == GSENSOR_TYPE_DA215S)
    {
        /* DA215S 支持自动增量读取，一次读取 6 个字节 (X_L, X_M, Y_L, Y_M, Z_L, Z_M) */
        err = i2c_burst_read(i2c_dev, DA215S_I2C_ADDR, DA215S_REG_X_LSB, raw_data, 6);
        if (err)
        {
            LOG_ERR("Failed to read DA215S data (err %d)", err);
            return err;
        }

        /* 组合三轴数据 (DA215S 是 12位 或 14位，此处按 16位 补齐解析) */
        data->x = (int16_t)((raw_data[1] << 8) | raw_data[0]);
        data->y = (int16_t)((raw_data[3] << 8) | raw_data[2]);
        data->z = (int16_t)((raw_data[5] << 8) | raw_data[4]);

        /* 根据 DA215S 的数据对齐方式，可能需要右移 4 位 (如果是 12 位模式) */
        // data->x >>= 4; ...

        return 0;
    }

    return -ENOTSUP;
}

static void gsensor_gpio_isr(const struct device *dev,
                          struct gpio_callback *cb,
                          uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    int level;

    if (pins & BIT(gsen_int.pin))
    {
        level = gpio_pin_get_dt(&gsen_int);
        // LOG_INF("gsen_int:%d", level);
        //TODO 发消息给ctrl去再次获取该引脚电平状态，以及消抖处理
    }
}

int my_gsensor_init(k_tid_t *tid)
{
    int err;

    /* 1. 检查硬件接口是否就绪 */
    if (!device_is_ready(i2c_dev) ||
        !device_is_ready(gsen_int.port) ||
        !device_is_ready(gsensor_pwr_gpio.port))
    {
        LOG_ERR("I2C device not ready");
        return -ENODEV;
    }

    if (!gpio_is_ready_dt(&gsensor_pwr_gpio))
    {
        LOG_ERR("GSENSOR Power GPIO not ready");
        return -ENODEV;
    }

    /* 2. 配置电源引脚并开启电源 */
    err = gpio_pin_configure_dt(&gsensor_pwr_gpio, GPIO_OUTPUT_ACTIVE);
    if (err)
    {
        LOG_ERR("Failed to configure GSENSOR Power GPIO (err %d)", err);
        return err;
    }
    
    /* 2.1. 配置中断引脚 */
    err = gpio_pin_configure_dt(&gsen_int, GPIO_INPUT);
    if (err)
    {
        LOG_ERR("Failed to configure GSENSOR interrupt GPIO input mode (err %d)", err);
        return err;
    }
    err = gpio_pin_interrupt_configure_dt(&gsen_int, GPIO_INT_EDGE_RISING);
    if (err)
    {
        LOG_ERR("Failed to configure GSENSOR Interrupt GPIO (err %d)", err);
        return err;
    }

    gpio_init_callback(&gsensor_gpio_cb, gsensor_gpio_isr, BIT(gsen_int.pin));
    gpio_add_callback(gsen_int.port, &gsensor_gpio_cb);

    /* 等待芯片上电稳定 */
    k_sleep(K_MSEC(50));

    /* 3. 识别传感器型号 */
    if (da215s_check_id())
    {
        current_sensor = GSENSOR_TYPE_DA215S;
        LOG_INF("GSENSOR identified as DA215S");

        /* 可以在这里进行 DA215S 的具体配置，如量程、采样率等 */
        // i2c_reg_write_byte(i2c_dev, DA215S_I2C_ADDR, ...);
    }
    else
    {
        LOG_WRN("Unknown GSENSOR type or device not responding");
        return -ENODEV;
    }

    /* 4. 初始化消息队列 */
    my_init_msg_handler(MOD_GSENSOR, &my_gsensor_msgq);

    /* 5. 启动 G-Sensor 线程 */
    *tid = k_thread_create(&my_gsensor_task_data, my_gsensor_task_stack,
                           K_THREAD_STACK_SIZEOF(my_gsensor_task_stack),
                           my_gsensor_task, NULL, NULL, NULL,
                           MY_GSENSOR_TASK_PRIORITY, 0, K_NO_WAIT);

    /* 6. 设置线程名称 */
    k_thread_name_set(*tid, "MY_GSENSOR");

    LOG_INF("G-Sensor module initialized");

    return 0;
}
