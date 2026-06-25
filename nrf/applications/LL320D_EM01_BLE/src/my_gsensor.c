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

#define GET_IMU_DATA 0 // 打开收集G_Sensor数据

/* GSENSOR 中断消抖控制结构体 */
typedef struct
{
    struct k_timer timer;
    bool debouncing;
    int64_t wakeup_mode_enter_ms;
} gsensor_int_ctrl_t;

/* GSENSOR 运行时上下文全局实例 */
gsensor_runtime_ctx_t g_gsensor_runtime_ctx =
{
    .imu_readings = {0},
    .window_index = 0,
    .sensor_ready = false,
    .sample_count = 0,
    .window_ready = false,
    .last_gsensor_state = STATE_UNKNOWN,
    .current_gsensor_state = STATE_STATIC,
};

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} sensor_data_t;

state_machine_t sm_batch;                          // 状态机实例
bayesian_classifier_t bayes_clf;                  /* 贝叶斯分类器实例 */

/* 注册 G-Sensor 模块日志 */
LOG_MODULE_REGISTER(my_gsensor, LOG_LEVEL_INF);

/* 从设备树获取硬件配置 */
#define I2C_NODE DT_ALIAS(gsensor_i2c)
static const struct device *i2c_dev = DEVICE_DT_GET(I2C_NODE);

#define GSENSOR_PWR_NODE DT_ALIAS(gsensor_pwr_ctrl)
static const struct gpio_dt_spec gsensor_pwr_gpio = GPIO_DT_SPEC_GET(GSENSOR_PWR_NODE, gpios);

static const struct gpio_dt_spec gsen_int = GPIO_DT_SPEC_GET(DT_ALIAS(gsensor_int), gpios);
static struct gpio_callback s_gsensor_gpio_cb;
static gsensor_int_ctrl_t s_gsensor_int_ctrl;

/* LSM6DSV16X I2C 从机地址 */
#define MY_LSM6DSV16X_I2C_ADDR 0x6A

// FIFO 水印阈值（1 LSb = TAG (1 Byte) + 1 sensor (6 Bytes) written in FIFO）
// 实际数据为（FIFO_WATERMARK/2）个陀螺仪+加速度计数据
// 30hz采样率,FIFO满大概2.8秒
#define FIFO_WATERMARK         168

/********************************************************************
**函数名称:  gsensor_reg_check
**入口参数:  ret      ---        I2C 操作返回值（输入）
**出口参数:  无
**函数功能:  检查 I2C 寄存器操作返回值，非 0 时直接返回错误码
**返 回 值:  0 表示成功，非 0 表示失败
*********************************************************************/
#define GSENSOR_REG_CHECK(ret) do { if ((ret) != 0) return (ret); } while (0)

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

/* 消息队列定义 */
K_MSGQ_DEFINE(my_gsensor_msgq, sizeof(msg_t), 10, 4);

/* 线程数据与栈定义 */
K_THREAD_STACK_DEFINE(my_gsensor_task_stack, MY_GSENSOR_TASK_STACK_SIZE);
static struct k_thread s_my_gsensor_task_data;

/* 电源管理回调函数前置声明 */
static int gsensor_pm_init(void);
static int gsensor_pm_suspend(void);
static int gsensor_pm_resume(void);

/* GSENSOR 电源管理操作回调结构体 */
static const pm_device_ops_t gsensor_pm_ops =
{
    .init = gsensor_pm_init,
    .suspend = gsensor_pm_suspend,
    .resume = gsensor_pm_resume,
};

static uint8_t s_chip_id = 0; // 存储识别到的芯片ID

static lsm6dsv16x_pin_int_route_t s_int1_route = { 0 };     // 存储INT1引脚路由配置
static bool s_shock_alarm_flag = false;                     // 敲击报警延时标志位

/* 子模式规则表：[sub_mode][状态] -> {cell_always_on, interval_unit_is_sec}
** 状态索引: 0=静止, 1=运动 */
typedef struct
{
    bool cell_always_on;        // true=Cell常开不断电, false=Cell断电需定时唤醒
    bool interval_unit_is_sec;  // true=间隔单位为秒, false=间隔单位为分钟
} smart_sub_mode_cfg_t;

static const smart_sub_mode_cfg_t s_smart_sub_mode_table[6][2] =
{
    /* sub0: 静止Sleep/min, 运动Sleep/min */
    {{false, false}, {false, false}},
    /* sub1: 静止Sleep/min, 运动Cell常开/min */
    {{false, false}, {true,  false}},
    /* sub2: 静止Sleep/min, 运动Cell+GNSS常开/sec */
    {{false, false}, {true,  true}},
    /* sub3: 静止Cell常开/min, 运动Cell常开/min */
    {{true,  false}, {true,  false}},
    /* sub4: 静止Cell常开/min, 运动Cell+GNSS常开/sec */
    {{true,  false}, {true,  true}},
    /* sub5: 静止Cell+GNSS常开/sec, 运动Cell+GNSS常开/sec */
    {{true,  true},  {true,  true}},
};

/********************************************************************
**函数名称:  gsensor_is_smart_mode
**入口参数:  无
**出口参数:  无
**函数功能:  判断当前工作模式是否为智能模式
**返 回 值:  true 表示智能模式，false 表示其他模式
*********************************************************************/
static bool gsensor_is_smart_mode(void)
{
    return (gConfigParam.device_workmode_config.workmode_config.current_mode == MY_MODE_SMART);
}

/********************************************************************
**函数名称:  gsensor_sample_timer_cb
**入口参数:  param     ---        定时器参数
**出口参数:  无
**函数功能:  GSENSOR 周期采样定时器回调，仅向线程发送读取消息
**返 回 值:  无
*********************************************************************/
static void gsensor_sample_timer_cb(void *param)
{
    ARG_UNUSED(param);

    my_send_msg(MOD_GSENSOR, MOD_GSENSOR, MY_MSG_GSENSOR_SAMPLE);
}

/********************************************************************
**函数名称:  gsensor_int_timer_handler
**入口参数:  timer    ---        定时器句柄（输入）
**出口参数:  无
**函数功能:  GSENSOR INT1 消抖定时器回调，确认中断电平稳定后再发送唤醒消息
**返 回 值:  无
*********************************************************************/
static void gsensor_int_timer_handler(struct k_timer *timer)
{
    ARG_UNUSED(timer);
    int level;

    level = gpio_pin_get_dt(&gsen_int);
    if (level == 1)
    {
        my_send_msg(MOD_GSENSOR, MOD_GSENSOR, MY_MSG_GSENSOR_INT);
    }

    s_gsensor_int_ctrl.debouncing = false;
}

/********************************************************************
**函数名称:  gsensor_int_edge_handler
**入口参数:  无
**出口参数:  无
**函数功能:  GSENSOR INT1 边沿中断处理，仅启动消抖定时器
**返 回 值:  无
*********************************************************************/
static void gsensor_int_edge_handler(void)
{
    if (s_gsensor_int_ctrl.debouncing != true)
    {
        s_gsensor_int_ctrl.debouncing = true;
        k_timer_start(&s_gsensor_int_ctrl.timer, K_MSEC(GSENSOR_INT_DEBOUNCE_MS), K_NO_WAIT);
    }
}

/********************************************************************
**函数名称:  gsensor_reset_sample_window
**入口参数:  无
**出口参数:  无
**函数功能:  重置 GSENSOR 滑动窗口与各个状态计数，为重新采集运动数据做准备
**返 回 值:  无
**注意事项:  恢复唤醒待机模式前必须调用此函数清空旧数据，避免历史窗口污染新状态判定
*********************************************************************/
static void gsensor_reset_sample_window(void)
{
    // 清零IMU滑动窗口（环形缓存）
    memset(g_gsensor_runtime_ctx.imu_readings, 0, sizeof(g_gsensor_runtime_ctx.imu_readings));
    // 重置窗口写索引（从头开始填充）
    g_gsensor_runtime_ctx.window_index = 0;
    // 重置采样计数（窗口数据量归零）
    g_gsensor_runtime_ctx.sample_count = 0;
    // 标记窗口未满（需重新填满500个数据点）
    g_gsensor_runtime_ctx.window_ready = false;
}

/********************************************************************
**函数名称:  gsensor_apply_shock_config
**入口参数:  无
**出口参数:  无
**函数功能:  进行G-Sensor 敲击报警配置，包括时间窗口、阈值、优先级等
**返 回 值:  无
*********************************************************************/
static int gsensor_apply_shock_config(void)
{
    int ret;
    uint8_t value;
    lsm6dsv16x_tap_dur_t tap_dur = { 0 };
    lsm6dsv16x_interrupt_mode_t int_mode = { 0 };
    lsm6dsv16x_tap_cfg0_t tap_cfg0 = { 0 };

    // 根据存储的配置是否打开中断功能
    if (gConfigParam.shockalarm_config.shockalarm_sw)
    {
        //配置敲击时间窗口
        // TAP_DUR (0x5A) 寄存器
        tap_dur.shock = 0x00;      // SHOCK[1:0]: 冲击时间窗口（00=4/ODR, 01=8/ODR, 10=12/ODR, 11=16/ODR）
        tap_dur.quiet = 0x02;      // QUIET[1:0]: 静音时间（00=2/ODR, 01=4/ODR, 10=6/ODR, 11=8/ODR）
        tap_dur.dur = 0x03;        // DUR[3:0]: 双击最大时间间隔（0000=16/ODR, 其他=n*32/ODR）
        ret = lsm6dsv16x_write_reg(&lsm_ctx, LSM6DSV16X_TAP_DUR, (uint8_t *)&tap_dur, 1);
        GSENSOR_REG_CHECK(ret);

        // 配置 Z 轴敲击阈值 + 轴优先级
        // TAP_CFG1 (0x57) 寄存器
        // bit4-0: TAP_TS_Z[4:0] - Z轴敲击阈值（0-31）
        value = gConfigParam.shockalarm_config.shockalarm_level * SHOCKALARM_TAP_STEP_THRESHOLD + 9;// 敲击步进阈值,9以下阈值过小，摇晃设备容易误判，从9开始阈值增加
        ret = lsm6dsv16x_write_reg(&lsm_ctx, LSM6DSV16X_TAP_CFG1, &value, 1);
        GSENSOR_REG_CHECK(ret);

        // 配置 Y 轴敲击阈值
        // TAP_CFG2 (0x58) 寄存器
        // bit7-5: 保留（必须为0）
        // bit4-0: TAP_TS_Y[4:0] - Y轴敲击阈值（0-31）
        ret = lsm6dsv16x_write_reg(&lsm_ctx, LSM6DSV16X_TAP_CFG2, &value, 1);
        GSENSOR_REG_CHECK(ret);

        // 配置 X 轴敲击阈值
        // TAP_THS_6D (0x59) 寄存器
        // bit4-0: TAP_TS_X[4:0] - X轴敲击阈值（0-31）
        ret = lsm6dsv16x_write_reg(&lsm_ctx, LSM6DSV16X_TAP_THS_6D, &value, 1);
        GSENSOR_REG_CHECK(ret);

        // 使能xyz轴的敲击检测
        tap_cfg0.tap_x_en = 1;
        tap_cfg0.tap_y_en = 1;
        tap_cfg0.tap_z_en = 1;
        ret = lsm6dsv16x_write_reg(&lsm_ctx, LSM6DSV16X_TAP_CFG0, (uint8_t *)&tap_cfg0, 1);
        GSENSOR_REG_CHECK(ret);

        int_mode.enable = 1;
        int_mode.lir = 1;
        ret = lsm6dsv16x_interrupt_enable_set(&lsm_ctx, int_mode);
        GSENSOR_REG_CHECK(ret);

        // 配置INT1引脚路由到wakeup事件（运动超阈值时拉高INT1唤醒主控）
        // s_int1_route.freefall = 1;
        s_int1_route.single_tap = 1;
        ret = lsm6dsv16x_pin_int1_route_set(&lsm_ctx, &s_int1_route);
        GSENSOR_REG_CHECK(ret);
    }
    else
    {
        // s_int1_route.freefall = 0;
        s_int1_route.single_tap = 0;
        ret = lsm6dsv16x_pin_int1_route_set(&lsm_ctx, &s_int1_route);
        GSENSOR_REG_CHECK(ret);
    }

    return 0;
}

/********************************************************************
**函数名称:  gsensor_apply_low_power_mode_config
**入口参数:  无
**出口参数:  无
**函数功能:  配置 G-Sensor 为低功耗模式
**返 回 值:  0 表示成功，负值表示错误码
*********************************************************************/
static int gsensor_apply_low_power_mode_config(void)
{
    int ret;

    // 敲击报警关闭时，断开G-Sensor电源
    if (gConfigParam.shockalarm_config.shockalarm_sw == 0)
    {
        g_gsensor_runtime_ctx.sensor_ready = false;
        my_gsensor_pwr_on(false);

        return 0;
    }

    // 关闭陀螺仪以降低功耗（唤醒待机模式仅需加速度计检测运动）
    ret = lsm6dsv16x_gy_data_rate_set(&lsm_ctx, LSM6DSV16X_ODR_OFF);
    GSENSOR_REG_CHECK(ret);

    // 配置加速度计数据速率为15Hz（低功耗待机，检测运动事件即可）
    ret = lsm6dsv16x_xl_data_rate_set(&lsm_ctx, LSM6DSV16X_ODR_AT_15Hz);
    GSENSOR_REG_CHECK(ret);

    // 设置加速度计满量程为±2g（与正常运行模式保持一致）
    ret = lsm6dsv16x_xl_full_scale_set(&lsm_ctx, LSM6DSV16X_2g);
    GSENSOR_REG_CHECK(ret);

    // 进入LPM2低功耗休眠模式
    ret = lsm6dsv16x_xl_mode_set(&lsm_ctx, LSM6DSV16X_XL_LOW_POWER_2_AVG_MD);
    GSENSOR_REG_CHECK(ret);


    return 0;
}

/********************************************************************
**函数名称:  gsensor_apply_run_mode_config
**入口参数:  无
**出口参数:  无
**函数功能:  配置 G-Sensor 为正常运行采样模式
**返 回 值:  0 表示成功，负值表示错误码
*********************************************************************/
static int gsensor_apply_run_mode_config(void)
{
    lsm6dsv16x_interrupt_mode_t int_mode = { 0 };
    int ret;

    // 使能数据块更新，确保读取的数据一致
    ret = lsm6dsv16x_block_data_update_set(&lsm_ctx, PROPERTY_ENABLE);
    GSENSOR_REG_CHECK(ret);

    // 配置加速度计数据速率为60Hz（响应速度与功耗的平衡点）
    ret = lsm6dsv16x_xl_data_rate_set(&lsm_ctx, LSM6DSV16X_ODR_AT_60Hz);
    GSENSOR_REG_CHECK(ret);

    // 设置加速度计满量程为±2g（灵敏度0.061mg/LSB，适用日常运动检测）
    ret = lsm6dsv16x_xl_full_scale_set(&lsm_ctx, LSM6DSV16X_2g);
    GSENSOR_REG_CHECK(ret);

    // 进入正常运行模式
    ret = lsm6dsv16x_xl_mode_set(&lsm_ctx, LSM6DSV16X_XL_HIGH_PERFORMANCE_MD);
    GSENSOR_REG_CHECK(ret);

    // 配置陀螺仪数据速率为60Hz（与加速度计同步，便于运动状态融合分析）
    ret = lsm6dsv16x_gy_data_rate_set(&lsm_ctx, LSM6DSV16X_ODR_AT_60Hz);
    GSENSOR_REG_CHECK(ret);

    // 设置陀螺仪满量程为125dps（分辨率比250dps高一倍，噪声占比更小，更适合静止/海运检测）
    ret = lsm6dsv16x_gy_full_scale_set(&lsm_ctx, LSM6DSV16X_125dps);
    GSENSOR_REG_CHECK(ret);

    // 等待陀螺仪启动稳定
    k_sleep(K_MSEC(50));

    // 配置 FIFO 批量模式，将数据批量写入FIFO，加速度计写入30Hz
    ret = lsm6dsv16x_fifo_xl_batch_set(&lsm_ctx, LSM6DSV16X_XL_BATCHED_AT_30Hz);
    GSENSOR_REG_CHECK(ret);

    // 配置 FIFO 批量模式，将数据批量写入FIFO，陀螺仪写入30Hz
    ret = lsm6dsv16x_fifo_gy_batch_set(&lsm_ctx, LSM6DSV16X_GY_BATCHED_AT_30Hz);
    GSENSOR_REG_CHECK(ret);

    // 配置 FIFO 水印阈值
    ret = lsm6dsv16x_fifo_watermark_set(&lsm_ctx, FIFO_WATERMARK);
    GSENSOR_REG_CHECK(ret);

    // 配置 FIFO 模式为水印模式
    ret = lsm6dsv16x_fifo_mode_set(&lsm_ctx, LSM6DSV16X_FIFO_MODE);
    GSENSOR_REG_CHECK(ret);

    // 打开事件中断总使能，并使用锁存模式保持INT1电平，软件手动清除，防止中断丢失
    int_mode.enable = 1;
    int_mode.lir = 1;
    ret = lsm6dsv16x_interrupt_enable_set(&lsm_ctx, int_mode);
    GSENSOR_REG_CHECK(ret);

    // 配置INT1引脚路由到FIFO水印事件
    s_int1_route.fifo_th = 1;
    ret = lsm6dsv16x_pin_int1_route_set(&lsm_ctx, &s_int1_route);
    GSENSOR_REG_CHECK(ret);

    return 0;
}

/********************************************************************
**函数名称:  my_gsensor_get_state
**入口参数:  无
**出口参数:  无
**函数功能:  获取当前GSENSOR状态
**返 回 值:  当前GSENSOR状态（静止/陆运/海运）
*********************************************************************/
gsensor_state_t my_gsensor_get_state(void)
{
    return g_gsensor_runtime_ctx.current_gsensor_state;
}

/********************************************************************
**函数名称:  lsm6dsvd_get_fifo_diff
**入口参数:  无
**出口参数:  无
**函数功能:  获取当前FIFO数据数量
**返 回 值:  FIFO数据数量
*********************************************************************/
uint16_t lsm6dsvd_get_fifo_diff(void)
{
    lsm6dsv16x_fifo_status_t status = { 0 };
    lsm6dsv16x_fifo_status_get(&lsm_ctx, &status);

    return (uint16_t)status.fifo_level;
}

/********************************************************************
**函数名称:  lsm6dsvd_read_fifo_entry
**入口参数:  无
**出口参数:  tag      ---        FIFO标签指针（输出）
**          data     ---        FIFO原始数据指针（输出）
**函数功能:  从FIFO读取一个原始数据条目
**返 回 值:  true:表示成功读取, false:表示FIFO为空
*********************************************************************/
bool lsm6dsvd_read_fifo_entry(uint8_t *tag, sensor_data_t *data)
{
    lsm6dsv16x_fifo_out_raw_t fifo_out_raw = { 0 };

    lsm6dsv16x_fifo_out_raw_get(&lsm_ctx, &fifo_out_raw);

    if (fifo_out_raw.tag == LSM6DSV16X_FIFO_EMPTY)
    {
        return false;
    }

    *tag = fifo_out_raw.tag;

    data->x = (int16_t)(fifo_out_raw.data[0] | (fifo_out_raw.data[1] << 8));
    data->y = (int16_t)(fifo_out_raw.data[2] | (fifo_out_raw.data[3] << 8));
    data->z = (int16_t)(fifo_out_raw.data[4] | (fifo_out_raw.data[5] << 8));

    return true;
}

/********************************************************************
**函数名称:  analyze_gsensor_state
**入口参数:  data      ---        加速度原始数据指针（输入）
            gyro_raw  ---        陀螺仪原始数据指针（输入）
**出口参数:  无
**函数功能:  分析GSENSOR数据，采用贝叶斯分类器判断运动状态,并更新状态机状态
**返 回 值:  -1:表示数据不足, 0:表示数据可用
*********************************************************************/
static int analyze_gsensor_state(const gsensor_data_t *data)
{
    static gsensor_state_t last_state = STATE_UNKNOWN;
    float ax, ay, az;
    float gx, gy, gz;
    float raw_prob[NUM_MODES] = {0};      /* 构造状态机输入概率 */
    float rem = 0.0f;                     /* 剩余概率 */
    int m = 0;                            /* 模式索引 */
    int ret = 0;
    classification_result_t br = {0};

    // 1. 原始数据转换为实际加速度（单位：m/s^2）
    ax = data->acc_raw_x * LSM6DSVD_ACC_SENSITIVITY * 1e-3f * 10.0f;
    ay = data->acc_raw_y * LSM6DSVD_ACC_SENSITIVITY * 1e-3f * 10.0f;
    az = data->acc_raw_z * LSM6DSVD_ACC_SENSITIVITY * 1e-3f * 10.0f;

    // 3.角速度减去零偏
    gx = data->gyro_raw_x * LSM6DSVD_GYRO_SENSITIVITY * 1e-3f;
    gy = data->gyro_raw_y * LSM6DSVD_GYRO_SENSITIVITY * 1e-3f;
    gz = data->gyro_raw_z * LSM6DSVD_GYRO_SENSITIVITY * 1e-3f;

    #if GET_IMU_DATA
    LOG_INF("%f, %f, %f, %f, %f, %f", ax, ay, az, gx, gy, gz);
    #endif
    // 4. 更新IMU数据滑动窗口
    g_gsensor_runtime_ctx.imu_readings[g_gsensor_runtime_ctx.window_index].acc_x = ax;
    g_gsensor_runtime_ctx.imu_readings[g_gsensor_runtime_ctx.window_index].acc_y = ay;
    g_gsensor_runtime_ctx.imu_readings[g_gsensor_runtime_ctx.window_index].acc_z = az;
    g_gsensor_runtime_ctx.imu_readings[g_gsensor_runtime_ctx.window_index].gyro_x = gx;
    g_gsensor_runtime_ctx.imu_readings[g_gsensor_runtime_ctx.window_index].gyro_y = gy;
    g_gsensor_runtime_ctx.imu_readings[g_gsensor_runtime_ctx.window_index].gyro_z = gz;

    // 5. 推进环形索引
    g_gsensor_runtime_ctx.window_index++;
    if (g_gsensor_runtime_ctx.window_index >= WINDOW_SIZE)
    {
        g_gsensor_runtime_ctx.window_index = 0;
    }

    g_gsensor_runtime_ctx.sample_count++;

    // 6. 窗口未满，数据不足
    if (g_gsensor_runtime_ctx.sample_count < WINDOW_SIZE)
    {
        g_gsensor_runtime_ctx.window_ready = false;
        return -1;
    }

    ret = lsm6dsv16x_fifo_mode_set(&lsm_ctx, LSM6DSV16X_BYPASS_MODE);
    GSENSOR_REG_CHECK(ret);
    g_gsensor_runtime_ctx.window_ready = true;

    bayes_set_dynamic_prior(&bayes_clf, last_state); /* 设置动态先验 */
    br = classify_bayesian(&bayes_clf, g_gsensor_runtime_ctx.imu_readings, WINDOW_SIZE); /* 贝叶斯分类 */
    last_state = br.mode;
    LOG_INF("br.mode = %d, br.confidence = %f", br.mode, br.confidence);

    raw_prob[mode_to_best(br.mode)] = br.confidence;     /* 主模式概率 = 置信度 */
    rem = 1.0f - br.confidence;      /* 剩余概率 */
    for (m = 0; m < NUM_MODES; m++)
    {  /* 分配剩余概率 */
        if (m != mode_to_best(br.mode))
        {
            raw_prob[m] = rem / 2.0f; /* 非主模式均分剩余 */
        }
    }
    g_gsensor_runtime_ctx.last_gsensor_state = g_gsensor_runtime_ctx.current_gsensor_state;
    g_gsensor_runtime_ctx.current_gsensor_state = sm_update(&sm_batch, raw_prob); /* 状态机更新 */
    LOG_INF("g_gsensor_runtime_ctx.current_gsensor_state = %d", g_gsensor_runtime_ctx.current_gsensor_state);

    // 启动 GSENSOR 周期采样定时器
    my_start_timer(MY_TIMER_GSENSOR_SAMPLE, gConfigParam.motdet_config.motdet_detection_interval * 1000, false, gsensor_sample_timer_cb);

    return 0;
}

/********************************************************************
**函数名称:  get_motion_status
**入口参数:  无
**出口参数:  无
**函数功能:  获取当前运动状态，根据子模式规则表决定LTE电源策略和定时器间隔
**返 回 值:  无
**注意事项:  子模式0~4静止间隔单位为分钟，子模式5为秒；
**           子模式0~1运动间隔为分钟，子模式2~5为秒；
**           Cell常开时不启动唤醒定时器，Cell断电时按间隔启动定时器唤醒4G
*********************************************************************/
/********************************************************************
**函数名称:  smart_mode_apply_lte_policy
**入口参数:  无
**出口参数:  无
**函数功能:  根据当前运动状态和智能模式子模式参数，重新应用LTE唤醒定时器策略
**返 回 值:  无
**注意事项:  不发送状态切换告警，仅设置LTE电源策略和定时器间隔
**           适用于参数更新但运动状态未变的场景
*********************************************************************/
void smart_mode_apply_lte_policy(void)
{
    uint32_t raw_interval = 0;
    uint32_t timer_interval_sec = 0;
    uint8_t sub_mode;
    bool is_moving;
    bool cell_always_on;
    bool unit_is_sec;
    gsensor_state_t state;

    sub_mode = gConfigParam.device_workmode_config.workmode_config.intelligent.sub_mode;
    state = my_gsensor_get_state();

    // 判断运动/静止（陆运和海运统一视为运动状态）
    if (state == STATE_STATIC)
    {
        is_moving = false;
        raw_interval = gConfigParam.device_workmode_config.workmode_config.intelligent.static_interval;
        MY_LOG_INF("Smart policy: STATIC, raw_interval = %d", raw_interval);
    }
    else
    {
        is_moving = true;
        raw_interval = gConfigParam.device_workmode_config.workmode_config.intelligent.moving_interval;
        MY_LOG_INF("Smart policy: MOVING, raw_interval = %d", raw_interval);
    }

    // 静止间隔为0表示静止不周期上报，不启动定时器
    if (!is_moving && raw_interval == 0)
    {
        my_stop_timer(MY_TIMER_LTE_POWER);
        MY_LOG_INF("Smart policy: STATIC interval=0, no periodic reporting");
        return;
    }

    // 查子模式规则表
    cell_always_on = s_smart_sub_mode_table[sub_mode][is_moving ? 1 : 0].cell_always_on;
    unit_is_sec = s_smart_sub_mode_table[sub_mode][is_moving ? 1 : 0].interval_unit_is_sec;
    MY_LOG_INF("Smart policy: sub_mode=%d, moving=%d, raw_interval=%d, unit=%s, cell_always_on=%d",
        sub_mode, is_moving, raw_interval, unit_is_sec ? "sec" : "min", cell_always_on);

    // 将原始间隔值转换为秒
    if (unit_is_sec)
    {
        timer_interval_sec = raw_interval;
    }
    else
    {
        timer_interval_sec = raw_interval * 60;  // 分钟转秒
    }

    /* 开启LTE */
    my_send_msg(MOD_GSENSOR, MOD_LTE, MY_MSG_LTE_PWRON);

    /* 根据子模式决定4G电源策略 */
    if (cell_always_on)
    {
        // Cell常开，不需要唤醒定时器
        my_stop_timer(MY_TIMER_LTE_POWER);
        MY_LOG_INF("Smart policy: sub%d, Cell always ON, no timer needed", sub_mode);
    }
    else
    {
        // Cell休眠，启动周期唤醒定时器
        my_start_timer(MY_TIMER_LTE_POWER, timer_interval_sec * 1000, true, awaken_lte_timer_callback);
        MY_LOG_INF("Smart policy: sub%d, Cell sleep, timer = %d sec", sub_mode, timer_interval_sec);
    }
}

/********************************************************************
**函数名称:  get_motion_status
**入口参数:  无
**出口参数:  无
**函数功能:  获取当前运动状态，发送状态切换告警并应用LTE唤醒策略
**返 回 值:  无
**注意事项:  仅在运动状态真正切换时调用，包含告警发送
*********************************************************************/
void get_motion_status(void)
{
    gsensor_state_t state;

    state = my_gsensor_get_state();

    // 发送状态切换告警
    if (state == STATE_STATIC)
    {
        send_alarm_message_to_lte(ALARM_STILL, NULL);
    }
    else
    {
        send_alarm_message_to_lte(ALARM_LAND, NULL);
    }

    // 应用LTE唤醒策略
    smart_mode_apply_lte_policy();
}

/********************************************************************
**函数名称:  gsensor_pm_init
**入口参数:  无
**出口参数:  无
**函数功能:  G-Sensor 电源管理初始化回调（首次启动时调用），配置电源控制 GPIO
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
static int gsensor_pm_init(void)
{
    int err;  // 错误码变量

    /* 配置 G-Sensor power 引脚，默认拉低*/
    err = my_gsensor_pwr_on(false);
    if (err)
    {
        MY_LOG_ERR("Failed to configure power GPIO: %d", err);
        return err;  // 返回配置错误
    }
    g_gsensor_runtime_ctx.sensor_ready = false;
    MY_LOG_INF("G-Sensor pwr initialized in low power mode");
    return 0;  // 初始化成功
}

/********************************************************************
**函数名称:  gsensor_pm_suspend
**入口参数:  无
**出口参数:  无
**函数功能:  挂起 G-Sensor，停止采样定时器并依据当前工作模式进入对应低功耗态
**返 回 值:  0 表示成功，负值表示错误码
**详细说明:
**           1. 智能模式：保持传感器供电，配置为唤醒待机模式（15Hz加速度计 + 唤醒中断），
**              并记录保护时间戳以过滤配置期间的假唤醒中断
**           2. 非智能模式：直接切断传感器电源，标记传感器未就绪
*********************************************************************/
static int gsensor_pm_suspend(void)
{
    int ret = 0;

    if (gsensor_is_smart_mode() == true)
    {
        if (g_gsensor_runtime_ctx.window_ready == true)
        {
            // 配置为低功耗模式
            gsensor_apply_low_power_mode_config();
        }
        MY_LOG_INF("GSENSOR successfully entered sleep mode");

        return 0;
    }

    ret = lsm6dsv16x_fifo_mode_set(&lsm_ctx, LSM6DSV16X_BYPASS_MODE);
    GSENSOR_REG_CHECK(ret);

    ret = gsensor_apply_low_power_mode_config();
    GSENSOR_REG_CHECK(ret);

    MY_LOG_INF("G-Sensor power suspended");
    return 0;
}

/********************************************************************
**函数名称:  gsensor_pm_resume
**入口参数:  无
**出口参数:  无
**函数功能:  恢复 G-Sensor 从低功耗状态到工作状态
**返 回 值:  0 表示成功，负值表示失败
**
**详细说明:  该函数在 G-Sensor 需要恢复工作时被调用，主要完成以下工作：
**           1. 打开 G-Sensor 电源
**           2. 尝试初始化 LSM6DSV16X 传感器，最多尝试 3 次
**           3. 如果初始化失败，关闭电源并返回错误
**           4. 如果初始化成功，获取运动状态
**           5. 记录恢复状态和重试次数
**
**重试机制:  为提高可靠性，函数实现了最多 3 次的初始化重试，每次重试前等待 10ms
**           确保 I2C 总线稳定，避免因总线不稳定导致的初始化失败
*********************************************************************/
static int gsensor_pm_resume(void)
{
    int result;         // 初始化结果
    int retry_count = 0; // 重试计数

    // 传感器未断电，只需切回运行模式，无需完整初始化
    if (g_gsensor_runtime_ctx.sensor_ready == true)
    {
        if (gsensor_is_smart_mode() == true && g_gsensor_runtime_ctx.window_ready == true)
        {
            // 清空窗口重新开始 burst 采集，确保每次判定都基于最新连续数据
            gsensor_reset_sample_window();

            // 配置为正常运行模式
            result = gsensor_apply_run_mode_config();
            if (result != 0)
            {
                MY_LOG_ERR("Failed to apply run mode config: %d", result);
                return result;
            }
        }
        MY_LOG_INF("gsensor resume wake up success for sleep");
        return 0;
    }

    // 打开 G-Sensor 电源
    my_gsensor_pwr_on(true);

    // 清空窗口重新开始 burst 采集，确保每次判定都基于最新连续数据
    gsensor_reset_sample_window();
    /* 传感器已断电，需要完整初始化
    * 尝试初始化 LSM6DSV16X 传感器，最多尝试 3 次
    */
    do
    {
        result = my_lsm6dsv16x_init();
        if (result == 0)
        {
            break; // 初始化成功，退出重试循环
        }

        retry_count++;
        MY_LOG_WRN("G-Sensor init attempt %d failed: %d", retry_count, result);
        k_msleep(10); /* 等待 I2C 总线稳定 */
    } while (retry_count < 3);

    // 初始化失败处理
    if (result != 0)
    {
        MY_LOG_ERR("Failed to reinitialize G-Sensor API after %d attempts: %d", retry_count, result);
        /* 恢复失败，重新进入低功耗 */
        my_gsensor_pwr_on(false);
        g_gsensor_runtime_ctx.sensor_ready = false;
        return -EIO; // 返回 I/O 错误
    }


    MY_LOG_INF("gsensor resume wake up success");
    return 0;
}


/********************************************************************
**函数名称:  gsensor_handle_read_msg
**入口参数:  无
**出口参数:  无
**函数功能:  处理 GSENSOR 数据读取消息，执行加速度/陀螺仪数据采集与运动状态分析
**返 回 值:  无
**详细说明:  1. 检查传感器就绪状态和工作模式
**           2. 若窗口已就绪则重置窗口，确保基于最新数据判定
**           3. 读取加速度计和陀螺仪数据
**           4. 调用 analyze_gsensor_state 进行运动状态判决
**           5. 根据状态决定是否需要 burst 采样或进入休眠
********************************************************************/
static void gsensor_handle_read_msg(void)
{
    uint16_t i = 0;
    uint16_t entries = 0;
    uint8_t tag;
    sensor_data_t data;
    gsensor_data_t sensor_data;

    // 传感器未就绪
    if (g_gsensor_runtime_ctx.sensor_ready != true)
    {
        MY_LOG_WRN("Sensor not ready");
        return;
    }


    // GSENSOR 仅在智能模式下生效，非智能模式直接返回
    if (gsensor_is_smart_mode() != true)
    {
        return;
    }

    entries = lsm6dsvd_get_fifo_diff();
    LOG_INF("lsm6dsvd_get_fifo_diff: %d", entries);

    for (i = 0; i < entries; i++)
    {
        if (!lsm6dsvd_read_fifo_entry(&tag, &data))
        {
            break;
        }

        if (tag == LSM6DSV16X_GY_NC_TAG)
        { // Gyroscope
           sensor_data.gyro_raw_x = data.x;
           sensor_data.gyro_raw_y = data.y;
           sensor_data.gyro_raw_z = data.z;
        }
        else if (tag == LSM6DSV16X_XL_NC_TAG)
        { // Accelerometer
            sensor_data.acc_raw_x = data.x;
            sensor_data.acc_raw_y = data.y;
            sensor_data.acc_raw_z = data.z;
        }

        if (i % 2 == 1)
        {
            analyze_gsensor_state(&sensor_data);
        }

        if (g_gsensor_runtime_ctx.window_ready == true)
        {
            // 如果窗口已满，说明已完成一次状态判定
            break;
        }
    }

    if (g_gsensor_runtime_ctx.last_gsensor_state != g_gsensor_runtime_ctx.current_gsensor_state)
    {
        g_gsensor_runtime_ctx.last_gsensor_state = g_gsensor_runtime_ctx.current_gsensor_state;
        get_motion_status();
    }

    my_pm_device_suspend(MY_PM_DEV_GSENSOR);
}

/********************************************************************
**函数名称:  gsensor_int_handler
**入口参数:  无
**出口参数:  无
**函数功能:  处理 G-Sensor 中断，包括 FIFO 数据和敲击检测
**返 回 值:  无
*********************************************************************/
void gsensor_int_handler(void)
{
    lsm6dsv16x_tap_src_t tap_src = {0};
    lsm6dsv16x_fifo_status_t fifo_status = {0};

    // 读取敲击检测寄存器，检查是否有敲击事件发生
    lsm6dsv16x_read_reg(&lsm_ctx, LSM6DSV16X_TAP_SRC, (uint8_t *)&tap_src, 1);
    // 读取 FIFO 状态寄存器，检查是否有FIFO中断发生
    lsm6dsv16x_fifo_status_get(&lsm_ctx, &fifo_status);

    if (fifo_status.fifo_th)
    {
        my_send_msg(MOD_GSENSOR, MOD_GSENSOR, MY_MSG_GSENSOR_FIFO_INT);
    }
    // 检查唤醒中断标志位
    if (tap_src.tap_ia)
    {
        my_send_msg(MOD_GSENSOR, MOD_GSENSOR, MY_MSG_GSENSOR_SHOCK_INT);
    }
}

/********************************************************************
**函数名称:  shock_alarm_timer_cb
**入口参数:  无
**出口参数:  无
**函数功能:  敲击报警定时器回调函数，用于处理敲击报警超时
**返 回 值:  无
*********************************************************************/
static void shock_alarm_timer_cb(void *param)
{
    // 敲击报警超时，清除报警标志位
    s_shock_alarm_flag = false;
}

/********************************************************************
**函数名称:  my_gsensor_shock_handler
**入口参数:  无
**出口参数:  无
**函数功能:  处理 G-Sensor 敲击报警，包括发送报警消息和启动定时器,定时器启动期间不发送报警
**返 回 值:  无
*********************************************************************/
void my_gsensor_shock_handler(void)
{
    // 敲击报警未触发时，发送报警消息并启动定时器
    if (s_shock_alarm_flag == false)
    {
        s_shock_alarm_flag = true;
        send_alarm_message_to_lte(ALARM_IMPACT, NULL);
        my_start_timer(MY_TIMER_GSENSOR_SHOCK_ALARM, gConfigParam.shockalarm_config.shockalarm_time * 1000, false, shock_alarm_timer_cb);
    }

    LOG_INF("G-Sensor shock alarm");
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

    msg_t msg;
    int ret;

    /* 注册 G-Sensor 到电源管理模块 */
    ret = my_pm_device_register(MY_PM_DEV_GSENSOR, &gsensor_pm_ops);
    if (ret < 0)
    {
        MY_LOG_ERR("G-Sensor PM registration failed");
        /* 注册失败，线程继续运行但无法使用 PM 功能 */
    }
    else
    {
        MY_LOG_INF("G-Sensor PM registered successfully");
    }

    MY_LOG_INF("G-Sensor thread started");

    LOG_INF("Training complete!\n");

    for (;;)
    {
        my_recv_msg(&my_gsensor_msgq, (void *)&msg, sizeof(msg_t), K_FOREVER);

        switch (msg.msgID)
        {
            case MY_MSG_GSENSOR_FIFO_INT:
                if (my_pm_device_get_state(MY_PM_DEV_GSENSOR) == MY_PM_STATE_SUSPENDED)
                {
                    ret = my_pm_device_resume(MY_PM_DEV_GSENSOR);
                    if (ret == 0)
                    {
                        gsensor_handle_read_msg();
                    }
                }
                break;

            case MY_MSG_GSENSOR_LOW_POWER:
                // 智能模式静止挂起后，设备状态可能已经是 SUSPENDED，但传感器仍处于带电唤醒待机态
                if (my_pm_device_get_state(MY_PM_DEV_GSENSOR) == MY_PM_STATE_SUSPENDED)
                {
                    // 停止采样定时器
                    my_stop_timer(MY_TIMER_GSENSOR_SAMPLE);
                }
                else
                {
                    // 通过电源管理模块挂起 G-Sensor 设备（会自动挂起 I2C21 总线），会调用 gsensor_pm_suspend
                    my_pm_device_suspend(MY_PM_DEV_GSENSOR);
                }
                sm_init(&sm_batch);
                break;

            case MY_MSG_GSENSOR_INT:
                gsensor_int_handler();
                break;

            case MY_MSG_GSENSOR_SHOCK_INT:
                my_gsensor_shock_handler();
                break;

            case MY_MSG_GSENSOR_SAMPLE:
                // 释放总线，初始化 FIFO 模式
                my_pm_device_resume(MY_PM_DEV_GSENSOR);

                my_pm_device_suspend(MY_PM_DEV_GSENSOR);
                break;

            case MY_MSG_GPS_SPEED_UPDATE:
                // 为智能模式时，速度大于20km/h，且当前状态为静止，认为是状态误判
                if (gsensor_is_smart_mode() && g_location_point.speed > 20 && g_gsensor_runtime_ctx.current_gsensor_state == STATE_STATIC)
                {
                    // 网络状态为0，认为是海运输状态
                    if (g_lte_net_flag == 0)
                    {
                        sm_batch.candidate_count = 0; // 重置候选状态计数
                        sm_batch.candidate_mode = STATE_SEA_TRANSPORT;// 候选状态为海运输状态
                        sm_batch.current_mode = STATE_SEA_TRANSPORT;// 当前状态为海运输状态

                        g_gsensor_runtime_ctx.current_gsensor_state = STATE_SEA_TRANSPORT;

                        get_motion_status();  // 更新当前状态

                        LOG_INF("sea transport");
                    }
                }
                break;

            case MY_MSG_SHOCK_SW:
                // 处理撞击检测开关消息
                ret = my_pm_device_resume(MY_PM_DEV_GSENSOR);
                if (ret == 0)
                {
                    gsensor_apply_shock_config();
                }
                my_pm_device_suspend(MY_PM_DEV_GSENSOR);

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
bool lsm6dsv16x_check_id(void)
{
    if (lsm6dsv16x_device_id_get(&lsm_ctx, &s_chip_id) == 0)
    {
        if (s_chip_id == MY_LSM6DSV16X_ID)
        {
            MY_LOG_INF("LSM6DSV16X identified ID: 0x%02X", s_chip_id);
            return true;
        }
    }

    return false;
}

/********************************************************************
**函数名称:  get_chip_id
**入口参数:  无
**出口参数:  无
**函数功能:  获取 LSM6DSV16X 芯片ID
**返 回 值:  LSM6DSV16X 芯片ID
*********************************************************************/
uint8_t get_chip_id(void)
{
    return s_chip_id;
}

int my_gsensor_pwr_on(bool on)
{
    int err;
    static bool s_gsensor_power_state = false;  // false=关闭, true=开启

    /* 检查当前电源状态，避免重复操作 */
    if (s_gsensor_power_state == on)
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
        s_gsensor_power_state = on;
        MY_LOG_INF("GSENSOR Power Control: %s", on ? "Power ON" : "Power OFF");
    }
    else
    {
        MY_LOG_ERR("GSENSOR Power Control failed (err %d)", err);
    }

    return err;
}

static void gsensor_gpio_isr(const struct device *dev,
                          struct gpio_callback *cb,
                          uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);

    if (pins & BIT(gsen_int.pin))
    {
        gsensor_int_edge_handler();
    }
}

int my_lsm6dsv16x_init(void)
{
    // 1.等待芯片上电稳定
    k_sleep(K_MSEC(50));

    // 2.识别 LSM6DSV16X 传感器
    if (lsm6dsv16x_check_id())
    {
        g_gsensor_runtime_ctx.sensor_ready = true;
        MY_LOG_INF("GSENSOR identified as LSM6DSV16X");

        // 3.初始化 LSM6DSV16X
        lsm6dsv16x_reset_set(&lsm_ctx, LSM6DSV16X_GLOBAL_RST);
        // 4.确保寄存器稳定后，才能设置寄存器
        k_sleep(K_MSEC(20));
        lsm6dsv16x_fifo_mode_set(&lsm_ctx, LSM6DSV16X_BYPASS_MODE);
        gsensor_apply_shock_config();
        if (gsensor_is_smart_mode() == true ? gsensor_apply_run_mode_config() : gsensor_apply_low_power_mode_config())
        {
            MY_LOG_ERR("Failed to configure GSENSOR run mode");
            g_gsensor_runtime_ctx.sensor_ready = false;
            return -EIO;
        }

        // 5.确保陀螺仪启动稳定时间
        k_sleep(K_MSEC(50));
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
    // 同步更新gsensor电源状态
    my_gsensor_pwr_on(true);

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

    gpio_init_callback(&s_gsensor_gpio_cb, gsensor_gpio_isr, BIT(gsen_int.pin));
    gpio_add_callback(gsen_int.port, &s_gsensor_gpio_cb);

    k_timer_init(&s_gsensor_int_ctrl.timer, gsensor_int_timer_handler, NULL);
    s_gsensor_int_ctrl.debouncing = false;
    s_gsensor_int_ctrl.wakeup_mode_enter_ms = 0;

    /* 3. 初始化 LSM6DSV16X 传感器 */
    err = my_lsm6dsv16x_init();
    if (err != 0)
    {
        my_gsensor_pwr_on(false);
        return err;
    }

    bayes_init(&bayes_clf);                        /* 初始化分类器, 加载模型参数 */
    sm_init(&sm_batch);                            /* 初始化状态机 */

    /* 4. 初始化消息队列 */
    my_init_msg_handler(MOD_GSENSOR, &my_gsensor_msgq);

    /* 5. 启动 G-Sensor 线程 */
    *tid = k_thread_create(&s_my_gsensor_task_data, my_gsensor_task_stack,
                           K_THREAD_STACK_SIZEOF(my_gsensor_task_stack),
                           my_gsensor_task, NULL, NULL, NULL,
                           MY_GSENSOR_TASK_PRIORITY, 0, K_NO_WAIT);

    /* 6. 设置线程名称 */
    k_thread_name_set(*tid, "MY_GSENSOR");

    MY_LOG_INF("G-Sensor module initialized");

    return 0;
}
