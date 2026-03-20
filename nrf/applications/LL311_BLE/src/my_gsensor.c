/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        my_gsensor.c
**文件描述:        G-Sensor 管理模块实现文件 (LSM6DSV16X)
**当前版本:        V1.0
**作    者:        Harrison Wu (wuyujiao@jimiiot.com)
**完成日期:        2026.01.15
*********************************************************************
** 功能描述:        1. 实现 LSM6DSV16X 的 I2C 初始化与数据读取
**                 2. 实现电源控制逻辑 (P2.6)
**                 3. 使用 ST 官方 STdC 驱动库
*********************************************************************/

/* 必须在包含 my_comm.h 之前定义 BLE_LOG_MODULE_ID，避免与 my_ble_log.h 中的默认定义冲突 */
#define BLE_LOG_MODULE_ID BLE_LOG_MOD_SENSOR

#include "my_comm.h"
#include "lsm6dsv16x_reg.h"

// 传感器相关宏定义（基于LSM6DSVD文档特性）
#define LSM6DSVD_ACC_FS       2             // 加速度满量程 ±2g
#define LSM6DSVD_ACC_SENSITIVITY 0.061f     // 灵敏度 0.061 mg/LSB（±2g时，文档Table 3）
#define SAMPLE_RATE_HZ        120           // 采样率120Hz（平衡响应速度与功耗）
#define WINDOW_SIZE           50            // 滑动窗口大小（约0.42秒数据，120Hz×50≈0.417s）
#define STATIC_VAR_THRESHOLD  0.0004f       // 静止方差阈值（g²）
#define LAND_VAR_THRESHOLD    0.01f         // 陆运方差阈值（g²）
// 海运：方差 > LAND_VAR_THRESHOLD

// 全局缓存：滑动窗口数据（合加速度，单位g）
static float g_acc_magnitude_window[WINDOW_SIZE] = {0};
// 全局缓存：当前窗口数据索引
static uint8_t g_window_index = 0;
/* 全局变量gsensor状态定义 */
gsensor_state_t g_current_gsensor_state = STATE_STATIC;  // 默认静止状态
/* 定义并初始化互斥锁 */
K_MUTEX_DEFINE(gsensor_mutex);
/* GSENSOR电源状态 */
static bool g_gsensor_power_state = false;  // false=关闭, true=开启

/* 注册 G-Sensor 模块日志 */
LOG_MODULE_REGISTER(my_gsensor, LOG_LEVEL_INF);

/* 从设备树获取硬件配置 */
#define I2C_NODE DT_ALIAS(gsensor_i2c)
static const struct device *i2c_dev = DEVICE_DT_GET(I2C_NODE);

#define GSENSOR_PWR_NODE DT_ALIAS(gsensor_pwr_ctrl)
static const struct gpio_dt_spec gsensor_pwr_gpio = GPIO_DT_SPEC_GET(GSENSOR_PWR_NODE, gpios);

static const struct gpio_dt_spec gsen_int = GPIO_DT_SPEC_GET(DT_ALIAS(gsensor_int), gpios);
static struct gpio_callback gsensor_gpio_cb;

/* LSM6DSV16X I2C 从机地址 */
#define MY_LSM6DSV16X_I2C_ADDR 0x6A

/* LSM6DSV16X 芯片 ID */
#define MY_LSM6DSV16X_ID 0x71

/* STdC 接口实现 */
static int32_t lsm6dsv16x_write(void *handle, uint8_t reg, const uint8_t *data, uint16_t len)
{
    const struct device *dev = (const struct device *)handle;
    return i2c_burst_write(dev, MY_LSM6DSV16X_I2C_ADDR, reg, data, len);
}

static int32_t lsm6dsv16x_read(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
    const struct device *dev = (const struct device *)handle;
    return i2c_burst_read(dev, MY_LSM6DSV16X_I2C_ADDR, reg, data, len);
}

static stmdev_ctx_t lsm_ctx = {
    .write_reg = lsm6dsv16x_write,
    .read_reg = lsm6dsv16x_read,
    .mdelay = k_msleep,
    .handle = (void *)DEVICE_DT_GET(I2C_NODE),
};

/* 当前识别到的传感器类型 (仅支持 LSM6DSV16X) */
static bool sensor_ready = false;

/* 消息队列定义 */
K_MSGQ_DEFINE(my_gsensor_msgq, sizeof(MSG_S), 10, 4);

/* 线程数据与栈定义 */
K_THREAD_STACK_DEFINE(my_gsensor_task_stack, MY_GSENSOR_TASK_STACK_SIZE);
static struct k_thread my_gsensor_task_data;

/********************************************************************
**函数名称:  my_gsensor_get_state
**入口参数:  无
**出口参数:  无
**函数功能:  获取当前GSENSOR状态
**返 回 值:  当前GSENSOR状态（静止/陆运/海运）
*********************************************************************/
gsensor_state_t my_gsensor_get_state(void)
{
    return g_current_gsensor_state;
}

/********************************************************************
**函数名称:  gsensor_calc_magnitude_variance
**入口参数:  无
**出口参数:  无
**函数功能:  计算滑动窗口内合加速度的方差
**返 回 值:  方差值（g²）
*********************************************************************/
static float gsensor_calc_magnitude_variance(void)
{
    float sum = 0.0f, mean = 0.0f, variance = 0.0f;
    uint8_t i;
    float diff;

    // 1. 计算窗口内合加速度的均值
    for (i = 0; i < WINDOW_SIZE; i++)
    {
        sum += g_acc_magnitude_window[i];
    }
    mean = sum / WINDOW_SIZE;

    // 2. 计算方差（无偏方差：除以WINDOW_SIZE-1）
    for (i = 0; i < WINDOW_SIZE; i++)
    {
        diff = g_acc_magnitude_window[i] - mean;
        variance += diff * diff;
    }

    // 防止除零
    if (WINDOW_SIZE > 1)
    {
        variance /= (WINDOW_SIZE - 1);  // 无偏方差，避免窗口数据量小时误差过大
    }

    return variance;
}

/********************************************************************
**函数名称:  analyze_gsensor_state
**入口参数:  data - GSENSOR原始数据指针
**出口参数:  无
**函数功能:  分析GSENSOR数据，更新滑动窗口并判断当前运动状态
**返 回 值:  -1:表示数据不足, 0:表示数据可用
*********************************************************************/
static int analyze_gsensor_state(const struct gsensor_data *data)
{
    float ax, ay, az, magnitude;
    float var;
    static uint32_t g_data_count = 0;  // 记录写入的数据量

    // 1. 原始数据转换为实际加速度（单位：g）
    // 转换公式：实际加速度(g) = 原始值(LSB) × 灵敏度(mg/LSB) × 1e-3 (mg→g)
    ax = data->x * LSM6DSVD_ACC_SENSITIVITY * 1e-3f;
    ay = data->y * LSM6DSVD_ACC_SENSITIVITY * 1e-3f;
    az = data->z * LSM6DSVD_ACC_SENSITIVITY * 1e-3f;

    // 2. 计算合加速度（消除安装方向影响）
    magnitude = sqrtf(ax*ax + ay*ay + az*az);

    // 3. 更新滑动窗口（环形缓存）
    g_acc_magnitude_window[g_window_index++] = magnitude;
    if (g_window_index >= WINDOW_SIZE)
    {
        g_window_index = 0;  // 窗口满了，覆盖最旧数据
    }

    g_data_count++;

    /* 无限等待直到拿到互斥锁 */
    k_mutex_lock(&gsensor_mutex, K_FOREVER);
    // 4. 计算方差（窗口数据满了才计算，避免采集数据不足导致误差）
    if (g_data_count < WINDOW_SIZE)
    {
        g_current_gsensor_state = STATE_UNKNOWN;
        return -1;
    }
    else
    {
        g_data_count = 0;
    }

    // 5. 计算方差
    var = gsensor_calc_magnitude_variance();

    // 6. 根据方差判断状态（基于之前校准的阈值）
    if (var < STATIC_VAR_THRESHOLD)
    {
        g_current_gsensor_state = STATE_STATIC;
    } 
    else if (var < LAND_VAR_THRESHOLD)
    {
        g_current_gsensor_state = STATE_LAND_TRANSPORT;
    } 
    else
    {
        g_current_gsensor_state = STATE_SEA_TRANSPORT;
    }

    /* 退出临界区，释放互斥锁 */
    k_mutex_unlock(&gsensor_mutex);

    return 0;
}

/********************************************************************
**函数名称:  get_motion_status
**入口参数:  无
**出口参数:  无
**函数功能:  获取当前运动状态，根据状态设置LTE电源定时器间隔
**返 回 值:  无
*********************************************************************/
void get_motion_status(void)
{
    struct gsensor_data sensor_data;
    uint32_t timer_interval = 0;
    DeviceWorkModeConfig *p_workmode;
    int ret;

    p_workmode = get_workmode_config_ptr();

#if 0 // TODO 需要测试这种处理算法是否可行
LOOP:
    if (my_gsensor_read_data(&sensor_data) == 0)
    {
        /* 分析GSENSOR状态 */
        ret = analyze_gsensor_state(&sensor_data);
        if (ret == -1)
        {
            // 连续读取50次数据
            goto LOOP;
        }
    }
    else
    {
        MY_LOG_INF("read gsensor data fail");
        return ;
    }
#endif

    /* 根据当前GSENSOR状态确定定时器间隔 */
    switch (my_gsensor_get_state())
    {
        case STATE_STATIC:
            timer_interval = p_workmode->intelligent.stop_status_interval_sec;  // 静止状态：默认86400秒（24小时）
            MY_LOG_INF("Smart mode: STATIC state, interval = %d", timer_interval);
            break;

        case STATE_LAND_TRANSPORT:
            timer_interval = p_workmode->intelligent.land_status_interval_sec;  // 陆运状态：默认15秒
            MY_LOG_INF("Smart mode: LAND TRANSPORT state, interval = %d", timer_interval);
            break;

        case STATE_SEA_TRANSPORT:
            timer_interval = p_workmode->intelligent.sea_status_interval_sec;  // 海运状态：默认14400秒（4小时）
            MY_LOG_INF("Smart mode: SEA TRANSPORT state, interval = %d", timer_interval);
            break;

        default:
            timer_interval = STATIC_INTERVAL;  // 默认使用静止状态间隔
            break;
    }

    /* 等获取到运动状态再开启LTE */
    my_send_msg(MOD_GSENSOR, MOD_LTE, MY_MSG_LTE_PWRON);

    /* sleep_switch为0、1的情况下,4G不会断电,所以无需开启唤醒4G定时器 */
    if (p_workmode->intelligent.sleep_switch == 0 ||
        p_workmode->intelligent.sleep_switch == 1)
    {
        my_stop_timer(MY_TIMER_LTE_POWER);
        return ;
    } 
    else if (p_workmode->intelligent.sleep_switch == 2)
    {
        /* 当设置的定时间隔>600时,4G是会整个断电的,所以需要启动定时器，使用动态确定的间隔 */
        if (timer_interval > 600)
        {
            my_start_timer(MY_TIMER_LTE_POWER, timer_interval * 1000, true, awaken_lte_timer_callback);
        } 
        else
        {
            my_stop_timer(MY_TIMER_LTE_POWER);
        }
    }

    //TODO 后续运动状态改变,需要将运动状态传给4G
}


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

    MY_LOG_INF("G-Sensor thread started");

    for (;;)
    {
        my_recv_msg(&my_gsensor_msgq, (void *)&msg, sizeof(MSG_S), K_FOREVER);
        
        switch (msg.msgID)
        {
            case MY_MSG_GSENSOR_READ:
            {
                /* 读取六轴数据 */
                if (sensor_ready)
                {
                    int16_t acc_raw[3], gyro_raw[3];
                    
                    /* 读取加速度计数据 */
                    if (lsm6dsv16x_acceleration_raw_get(&lsm_ctx, acc_raw) == 0)
                    {
                        MY_LOG_INF("ACC: X=%6d, Y=%6d, Z=%6d", acc_raw[0], acc_raw[1], acc_raw[2]);
                    }
                    else
                    {
                        MY_LOG_ERR("Failed to read accelerometer");
                    }
                    
                    /* 读取陀螺仪数据 */
                    if (lsm6dsv16x_angular_rate_raw_get(&lsm_ctx, gyro_raw) == 0)
                    {
                        MY_LOG_INF("GYR: X=%6d, Y=%6d, Z=%6d", gyro_raw[0], gyro_raw[1], gyro_raw[2]);
                    }
                    else
                    {
                        MY_LOG_ERR("Failed to read gyroscope");
                    }
                }
                else
                {
                    MY_LOG_WRN("Sensor not ready");
                }
                break;
            }

            case MY_MSG_GSENSOR_PWROFF:
                my_gsensor_pwr_on(false);
                break;

            case MY_MSG_GSENSOR_PWRON:
                my_gsensor_pwr_on(true);
                my_send_msg(MOD_GSENSOR, MOD_GSENSOR, MY_MSG_GSENSOR_INIT);
                break;

            case MY_MSG_GSENSOR_INIT:
                my_lsm6dsv16x_init();
                my_send_msg(MOD_GSENSOR, MOD_GSENSOR, MY_MSG_GSENSOR_GET_MOTION_STATUS);
                break;

            case MY_MSG_GSENSOR_GET_MOTION_STATUS:
                get_motion_status();
                break;

            default:
                break;
        }
    }
}

/********************************************************************
**函数名称:  lsm6dsv16x_check_id
**入口参数:  无
**出口参数:  无
**函数功能:  检查 LSM6DSV16X 是否在线
**返 回 值:  true 表示识别成功
*********************************************************************/
static bool lsm6dsv16x_check_id(void)
{
    uint8_t chip_id = 0;

    if (lsm6dsv16x_device_id_get(&lsm_ctx, &chip_id) == 0)
    {
        if (chip_id == MY_LSM6DSV16X_ID)
        {
            MY_LOG_INF("LSM6DSV16X identified ID: 0x%02X", chip_id);
            return true;
        }
    }

    return false;
}

int my_gsensor_pwr_on(bool on)
{
    int err;

    /* 检查当前电源状态，避免重复操作 */
    if (g_gsensor_power_state == on)
    {
        /* 状态相同，无需操作 */
        MY_LOG_INF("GSENSOR Power: already %s", on ? "ON" : "OFF");
        return 0;
    }

    /* 执行电源控制操作 */
    err = gpio_pin_set_dt(&gsensor_pwr_gpio, on ? 1 : 0);
    if (err == 0)
    {
        /* 操作成功，更新状态 */
        g_gsensor_power_state = on;
        MY_LOG_INF("GSENSOR Power Control: %s", on ? "Power ON" : "Power OFF");
    }
    else
    {
        MY_LOG_ERR("GSENSOR Power Control failed (err %d)", err);
    }

    return err;
}

int my_gsensor_read_data(struct gsensor_data *data)
{
    int16_t data_raw[3];
    
    if (!sensor_ready)
    {
        return -ENODEV;
    }

    if (lsm6dsv16x_acceleration_raw_get(&lsm_ctx, data_raw) == 0)
    {
        data->x = data_raw[0];
        data->y = data_raw[1];
        data->z = data_raw[2];
        return 0;
    }
    
    return -EIO;
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
        // MY_LOG_INF("gsen_int:%d", level);
        //TODO 发消息给ctrl去再次获取该引脚电平状态，以及消抖处理
    }
}

int my_lsm6dsv16x_init(void)
{
    /* 等待芯片上电稳定 */
    k_sleep(K_MSEC(50));

    /* 3. 识别 LSM6DSV16X 传感器 */
    if (lsm6dsv16x_check_id())
    {
        sensor_ready = true;
        MY_LOG_INF("GSENSOR identified as LSM6DSV16X");

        /* 初始化 LSM6DSV16X */
        lsm6dsv16x_reset_set(&lsm_ctx, LSM6DSV16X_GLOBAL_RST);
        k_sleep(K_MSEC(30));
        lsm6dsv16x_block_data_update_set(&lsm_ctx, PROPERTY_ENABLE);
        
        /* 配置加速度计：120Hz, 2g */
        lsm6dsv16x_xl_data_rate_set(&lsm_ctx, LSM6DSV16X_ODR_AT_120Hz);
        lsm6dsv16x_xl_full_scale_set(&lsm_ctx, LSM6DSV16X_2g);
        
        /* 配置陀螺仪：120Hz, 250dps */
        lsm6dsv16x_gy_data_rate_set(&lsm_ctx, LSM6DSV16X_ODR_AT_120Hz);
        lsm6dsv16x_gy_full_scale_set(&lsm_ctx, LSM6DSV16X_250dps);
    }
    else
    {
        MY_LOG_ERR("LSM6DSV16X not detected");
        return -ENODEV;
    }
    return 0;
}

int my_gsensor_init(k_tid_t *tid)
{
    int err;

    /* 1. 检查硬件接口是否就绪 */
    if (!device_is_ready(i2c_dev) ||
        !device_is_ready(gsen_int.port) ||
        !device_is_ready(gsensor_pwr_gpio.port))
    {
        MY_LOG_ERR("I2C device not ready");
        return -ENODEV;
    }

    if (!gpio_is_ready_dt(&gsensor_pwr_gpio))
    {
        MY_LOG_ERR("GSENSOR Power GPIO not ready");
        return -ENODEV;
    }

    /* 2. 配置电源引脚并开启电源 */
    err = gpio_pin_configure_dt(&gsensor_pwr_gpio, GPIO_OUTPUT_ACTIVE);
    if (err)
    {
        MY_LOG_ERR("Failed to configure GSENSOR Power GPIO (err %d)", err);
        return err;
    }
    
    /* 2.1. 配置中断引脚 */
    err = gpio_pin_configure_dt(&gsen_int, GPIO_INPUT);
    if (err)
    {
        MY_LOG_ERR("Failed to configure GSENSOR interrupt GPIO input mode (err %d)", err);
        return err;
    }
    err = gpio_pin_interrupt_configure_dt(&gsen_int, GPIO_INT_EDGE_RISING);
    if (err)
    {
        MY_LOG_ERR("Failed to configure GSENSOR Interrupt GPIO (err %d)", err);
        return err;
    }

    gpio_init_callback(&gsensor_gpio_cb, gsensor_gpio_isr, BIT(gsen_int.pin));
    gpio_add_callback(gsen_int.port, &gsensor_gpio_cb);

    /* 3. 初始化 LSM6DSV16X 传感器 */
    err = my_lsm6dsv16x_init();
    if (err != 0)
    {
        return err;
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

    MY_LOG_INF("G-Sensor module initialized");

    return 0;
}
