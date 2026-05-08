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

StateMachine sm_batch;                          // 状态机实例
BayesianClassifier bayes_clf;                  /* 贝叶斯分类器实例 */

/* 注册 G-Sensor 模块日志 */
LOG_MODULE_REGISTER(my_gsensor, LOG_LEVEL_INF);

/* 从设备树获取硬件配置 */
#define I2C_NODE DT_ALIAS(gsensor_i2c)
static const struct device *i2c_dev = DEVICE_DT_GET(I2C_NODE);

#define GSENSOR_PWR_NODE DT_ALIAS(gsensor_pwr_ctrl)
static const struct gpio_dt_spec gsensor_pwr_gpio = GPIO_DT_SPEC_GET(GSENSOR_PWR_NODE, gpios);

static const struct gpio_dt_spec gsen_int = GPIO_DT_SPEC_GET(DT_ALIAS(gsensor_int), gpios);
static struct gpio_callback gsensor_gpio_cb;
static gsensor_int_ctrl_t g_gsensor_int_ctrl;

/* LSM6DSV16X I2C 从机地址 */
#define MY_LSM6DSV16X_I2C_ADDR 0x6A

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
K_MSGQ_DEFINE(my_gsensor_msgq, sizeof(MSG_S), 10, 4);

/* 线程数据与栈定义 */
K_THREAD_STACK_DEFINE(my_gsensor_task_stack, MY_GSENSOR_TASK_STACK_SIZE);
static struct k_thread my_gsensor_task_data;

/* 电源管理回调函数前置声明 */
static int gsensor_pm_init(void);
static int gsensor_pm_suspend(void);
static int gsensor_pm_resume(void);

/* GSENSOR 电源管理操作回调结构体 */
static const struct PM_DEVICE_OPS gsensor_pm_ops =
{
    .init = gsensor_pm_init,
    .suspend = gsensor_pm_suspend,
    .resume = gsensor_pm_resume,
};

static uint8_t s_chip_id = 0; // 存储识别到的芯片ID
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

    my_send_msg(MOD_GSENSOR, MOD_GSENSOR, MY_MSG_GSENSOR_READ);
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
        my_send_msg(MOD_GSENSOR, MOD_GSENSOR, MY_MSG_GSENSOR_WAKEUP_INT);
    }

    g_gsensor_int_ctrl.debouncing = false;
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
    if (g_gsensor_int_ctrl.debouncing != true)
    {
        g_gsensor_int_ctrl.debouncing = true;
        k_timer_start(&g_gsensor_int_ctrl.timer, K_MSEC(GSENSOR_INT_DEBOUNCE_MS), K_NO_WAIT);
    }
}

/********************************************************************
**函数名称:  gsensor_start_sample_timer
**入口参数:  无
**出口参数:  无
**函数功能:  启动 GSENSOR 周期采样定时器，仅在智能模式下生效
**返 回 值:  无
*********************************************************************/
static void gsensor_start_sample_timer(void)
{
    if (gsensor_is_smart_mode() != true)
    {
        return;
    }

    my_start_timer(MY_TIMER_GSENSOR_SAMPLE,
                   GSENSOR_SAMPLE_INTERVAL_MS,
                   true,
                   gsensor_sample_timer_cb);
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
**函数名称:  gsensor_reset_state
**入口参数:  无
**出口参数:  无
**函数功能:  重置 G-Sensor 运动状态机，清除滞后计数与状态缓存
**返 回 值:  无
**注意事项:  在传感器恢复运行或模式切换时调用，防止历史状态残留导致误判
*********************************************************************/
static void gsensor_reset_state(void)
{
    sm_batch.candidate_count = 0;           // 重置状态切换候选计数

    sm_batch.current_mode = STATE_UNKNOWN; // 重置当前状态

    sm_batch.candidate_mode = STATE_UNKNOWN; // 重置状态切换候选
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
    lsm6dsv16x_pin_int_route_t int1_route = { 0 };
    lsm6dsv16x_interrupt_mode_t int_mode = { 0 };
    int ret;

    // 使能数据块更新，确保读取的数据一致
    ret = lsm6dsv16x_block_data_update_set(&lsm_ctx, PROPERTY_ENABLE);
    GSENSOR_REG_CHECK(ret);

    // 配置加速度计数据速率为120Hz（响应速度与功耗的平衡点）
    ret = lsm6dsv16x_xl_data_rate_set(&lsm_ctx, LSM6DSV16X_ODR_AT_120Hz);
    GSENSOR_REG_CHECK(ret);

    // 设置加速度计满量程为±2g（灵敏度0.061mg/LSB，适用日常运动检测）
    ret = lsm6dsv16x_xl_full_scale_set(&lsm_ctx, LSM6DSV16X_2g);
    GSENSOR_REG_CHECK(ret);

    // 配置陀螺仪数据速率为120Hz（与加速度计同步，便于运动状态融合分析）
    ret = lsm6dsv16x_gy_data_rate_set(&lsm_ctx, LSM6DSV16X_ODR_AT_120Hz);
    GSENSOR_REG_CHECK(ret);

    // 设置陀螺仪满量程为125dps（分辨率比250dps高一倍，噪声占比更小，更适合静止/海运检测）
    ret = lsm6dsv16x_gy_full_scale_set(&lsm_ctx, LSM6DSV16X_125dps);
    GSENSOR_REG_CHECK(ret);

    // 清零中断路由配置（本模式下不使用硬件中断，由软件轮询采集数据）
    ret = lsm6dsv16x_pin_int1_route_set(&lsm_ctx, &int1_route);
    GSENSOR_REG_CHECK(ret);

    // 正常运行模式关闭全局事件中断，避免残留锁存状态影响后续休眠唤醒
    int_mode.enable = 0;
    int_mode.lir = 0;
    ret = lsm6dsv16x_interrupt_enable_set(&lsm_ctx, int_mode);
    GSENSOR_REG_CHECK(ret);

    // 禁用活动/睡眠模式，恢复正常运行状态
    ret = lsm6dsv16x_act_mode_set(&lsm_ctx, LSM6DSV16X_XL_AND_GY_NOT_AFFECTED);
    GSENSOR_REG_CHECK(ret);

    return 0;
}

/********************************************************************
**函数名称:  gsensor_apply_wakeup_mode_config
**入口参数:  无
**出口参数:  无
**函数功能:  配置 G-Sensor 为低功耗唤醒待机模式
**返 回 值:  0 表示成功，负值表示错误码
*********************************************************************/
static int gsensor_apply_wakeup_mode_config(void)
{
    int ret;
    lsm6dsv16x_wake_up_ths_t wake_up_ths = { 0 };
    lsm6dsv16x_wake_up_dur_t wake_up_dur = { 0 };
    lsm6dsv16x_pin_int_route_t int1_route = { 0 };
    lsm6dsv16x_interrupt_mode_t int_mode = { 0 };

    // 关闭陀螺仪以降低功耗（唤醒待机模式仅需加速度计检测运动）
    ret = lsm6dsv16x_gy_data_rate_set(&lsm_ctx, LSM6DSV16X_ODR_OFF);
    GSENSOR_REG_CHECK(ret);

    // 配置加速度计数据速率为15Hz（低功耗待机，检测运动事件即可）
    ret = lsm6dsv16x_xl_data_rate_set(&lsm_ctx, LSM6DSV16X_ODR_AT_15Hz);
    GSENSOR_REG_CHECK(ret);

    // 设置加速度计满量程为±2g（与正常运行模式保持一致）
    ret = lsm6dsv16x_xl_full_scale_set(&lsm_ctx, LSM6DSV16X_2g);
    GSENSOR_REG_CHECK(ret);

    // 配置唤醒阈值（值2≈244mg，超过此加速度触发唤醒中断）
    wake_up_ths.wk_ths = GSENSOR_WAKEUP_THRESHOLD;
    ret = lsm6dsv16x_write_reg(&lsm_ctx, LSM6DSV16X_WAKE_UP_THS, (uint8_t *)&wake_up_ths, 1);
    GSENSOR_REG_CHECK(ret);

    // 打开事件中断总使能，并使用锁存模式保持INT1电平，软件手动清除，防止中断丢失
    int_mode.enable = 1;
    int_mode.lir = 1;
    ret = lsm6dsv16x_interrupt_enable_set(&lsm_ctx, int_mode);
    GSENSOR_REG_CHECK(ret);

    // 配置INT1引脚路由到wakeup事件（运动超阈值时拉高INT1唤醒主控）
    int1_route.wakeup = 1;
    ret = lsm6dsv16x_pin_int1_route_set(&lsm_ctx, &int1_route);
    GSENSOR_REG_CHECK(ret);

    wake_up_dur.sleep_dur = 0;                      // 闲置多久后进入睡眠（填 0 = 立即睡眠）
    wake_up_dur.wake_dur = GSENSOR_WAKEUP_DURATION; // 配置唤醒持续时间（0=1个ODR周期即触发，响应最灵敏）
    ret = lsm6dsv16x_write_reg(&lsm_ctx, LSM6DSV16X_WAKE_UP_DUR, (uint8_t *)&wake_up_dur, 1);
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
**函数名称:  gsensor_calc_window_variance
**入口参数:  window    ---        滑动窗口数据指针（输入）
            size      ---        窗口大小（输入）
**出口参数:  无
**函数功能:  计算滑动窗口内数据的无偏方差
**返 回 值:  方差值
*********************************************************************/
static float gsensor_calc_window_variance(const float *window, uint8_t size)
{
    float sum = 0.0f, mean = 0.0f, variance = 0.0f;
    uint8_t i;
    float diff;

    // 1. 计算窗口内数据均值
    for (i = 0; i < size; i++)
    {
        sum += window[i];
    }
    mean = sum / size;

    // 2. 计算方差（无偏方差：除以size-1）
    for (i = 0; i < size; i++)
    {
        diff = window[i] - mean;
        variance += diff * diff;
    }

    // 防止除零
    if (size > 1)
    {
        variance /= (size - 1);  // 无偏方差，避免窗口数据量小时误差过大
    }

    return variance;
}

/********************************************************************
**函数名称:  gsensor_calc_window_mean
**入口参数:  window    ---        滑动窗口数据指针（输入）
            size      ---        窗口大小（输入）
**出口参数:  无
**函数功能:  计算滑动窗口内数据的算术均值
**返 回 值:  均值
*********************************************************************/
static float gsensor_calc_window_mean(const float *window, uint8_t size)
{
    float sum = 0.0f;
    uint8_t i;

    for (i = 0; i < size; i++)
    {
        sum += window[i];
    }

    return sum / size;
}

/********************************************************************
**函数名称:  analyze_gsensor_state
**入口参数:  data      ---        加速度原始数据指针（输入）
            gyro_raw  ---        陀螺仪原始数据指针（输入）
**出口参数:  无
**函数功能:  分析GSENSOR数据，采用贝叶斯分类器判断运动状态,并更新状态机状态
**返 回 值:  -1:表示数据不足, 0:表示数据可用
*********************************************************************/
static int analyze_gsensor_state(const struct gsensor_data *data)
{
    static gsensor_state_t last_state = STATE_UNKNOWN;
    float ax, ay, az;
    float gx, gy, gz;
    float acc_var, gyro_mean = 0.0f, gyro_var = 0.0f;
    float raw_prob[NUM_MODES] = {0};      /* 构造状态机输入概率 */
    float rem = 0.0f;                     /* 剩余概率 */
    int m = 0;                            /* 模式索引 */
    ClassificationResult br = {0};

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
    uint32_t timer_interval = 0;

    /* 根据当前GSENSOR状态确定定时器间隔 */
    switch (my_gsensor_get_state())
    {
        case STATE_STATIC:
            timer_interval = gConfigParam.device_workmode_config.workmode_config.intelligent.stop_status_interval_sec;  // 静止状态：默认86400秒（24小时）
            MY_LOG_INF("Smart mode: STATIC state, interval = %d", timer_interval);
            break;

        case STATE_LAND_TRANSPORT:
            timer_interval = gConfigParam.device_workmode_config.workmode_config.intelligent.land_status_interval_sec;  // 陆运状态：默认15秒
            MY_LOG_INF("Smart mode: LAND TRANSPORT state, interval = %d", timer_interval);
            break;

        case STATE_SEA_TRANSPORT:
            timer_interval = gConfigParam.device_workmode_config.workmode_config.intelligent.sea_status_interval_sec;  // 海运状态：默认14400秒（4小时）
            MY_LOG_INF("Smart mode: SEA TRANSPORT state, interval = %d", timer_interval);
            break;

        default:
            timer_interval = STATIC_INTERVAL;  // 默认使用静止状态间隔
            break;
    }

    /* 等获取到运动状态再开启LTE */
    my_send_msg(MOD_GSENSOR, MOD_LTE, MY_MSG_LTE_PWRON);

    /* sleep_switch为0、1的情况下,4G不会断电,所以无需开启唤醒4G定时器 */
    if (gConfigParam.device_workmode_config.workmode_config.intelligent.sleep_switch == 0 ||
        gConfigParam.device_workmode_config.workmode_config.intelligent.sleep_switch == 1)
    {
        my_stop_timer(MY_TIMER_LTE_POWER);
        return ;
    }
    else if (gConfigParam.device_workmode_config.workmode_config.intelligent.sleep_switch == 2)
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

    my_stop_timer(MY_TIMER_GSENSOR_SAMPLE);

    if (gsensor_is_smart_mode() == true)
    {
        /* 更新保护时间戳，再配置传感器寄存器。
         * 防止配置过程中产生的假唤醒使使设备唤醒
         */
        g_gsensor_int_ctrl.wakeup_mode_enter_ms = k_uptime_get();
        ret = gsensor_apply_wakeup_mode_config();
        if (ret != 0)
        {
            MY_LOG_ERR("Failed to set G-Sensor wakeup mode: %d", ret);
            return ret;
        }

        MY_LOG_INF("GSENSOR successfully entered sleep mode");

        return 0;
    }

    ret = my_gsensor_pwr_on(false);
    if (ret != 0)
    {
        MY_LOG_ERR("Failed to suspend G-Sensor power: %d", ret);
        return ret;
    }

    g_gsensor_runtime_ctx.sensor_ready = false;
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
    lsm6dsv16x_all_sources_t all_sources;
    int16_t acc_raw[3];

    // 打开 G-Sensor 电源
    my_gsensor_pwr_on(true);

    // 智能模式下传感器未断电，只需切回运行模式，无需完整初始化
    if (gsensor_is_smart_mode() == true && g_gsensor_runtime_ctx.sensor_ready == true)
    {
        result = gsensor_apply_run_mode_config();
        if (result != 0)
        {
            MY_LOG_ERR("Failed to apply run mode config: %d", result);
            return result;
        }

        // 清楚状态及窗口
        gsensor_reset_state();
        gsensor_reset_sample_window();
        // 开启采样
        gsensor_start_sample_timer();

        memset(&all_sources, 0, sizeof(all_sources));
        // 先读取并清除中断锁存，避免 INT1 持续高电平导致后续无法触发中断
        lsm6dsv16x_all_sources_get(&lsm_ctx, &all_sources);

        // 读取唤醒时的加速度值，用于分析误触发原因
        if (lsm6dsv16x_acceleration_raw_get(&lsm_ctx, acc_raw) == 0)
        {
            MY_LOG_INF("wakeup acc raw=[%6d,%6d,%6d] ths=%d",
                        acc_raw[0], acc_raw[1], acc_raw[2], GSENSOR_WAKEUP_THRESHOLD);
        }

        MY_LOG_INF("gsensor resume wake up success for sleep");
        return 0;
    }

    /* 非智能模式：传感器已断电，需要完整初始化
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

    // 清楚状态及窗口
    gsensor_reset_state();
    gsensor_reset_sample_window();
    // 开启采样
    gsensor_start_sample_timer();

    MY_LOG_INF("gsensor resume wake up success");
    return 0;
}

/********************************************************************
**函数名称:  gsensor_resume_and_read
**入口参数:  无
**出口参数:  无
**函数功能:  恢复 G-Sensor 并触发数据读取
**返 回 值:  无
**详细说明:  统一封装电源恢复与数据读取消息发送，避免多处重复代码
********************************************************************/
static void gsensor_resume_and_read(void)
{
    int ret;

    ret = my_pm_device_resume(MY_PM_DEV_GSENSOR);
    if (ret == 0)
    {
        my_send_msg(MOD_GSENSOR, MOD_GSENSOR, MY_MSG_GSENSOR_READ);
    }
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
    int16_t acc_raw[3];
    int16_t gyro_raw[3];
    struct gsensor_data sensor_data;
    int state_ret = 0;
    bool is_read_ok = false;
    bool need_burst_sample = false;
    bool need_suspend = false;

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

    /* 如果窗口已满，说明之前已完成一次状态判定，窗口数据已过期。
     * 清空窗口重新开始 burst 采集，确保每次判定都基于最新连续数据
     */
    if (g_gsensor_runtime_ctx.window_ready == true)
    {
        gsensor_reset_sample_window();
    }

    // 读取加速度计和陀螺仪数据
    if (lsm6dsv16x_acceleration_raw_get(&lsm_ctx, acc_raw) == 0 && lsm6dsv16x_angular_rate_raw_get(&lsm_ctx, gyro_raw) == 0)
    {
        is_read_ok = true;
        sensor_data.acc_raw_x = acc_raw[0];
        sensor_data.acc_raw_y = acc_raw[1];
        sensor_data.acc_raw_z = acc_raw[2];
        sensor_data.gyro_raw_x = gyro_raw[0];
        sensor_data.gyro_raw_y = gyro_raw[1];
        sensor_data.gyro_raw_z = gyro_raw[2];
    }
    else
    {
        MY_LOG_ERR("Failed to read accelerometer and gyroscope");
    }

    if (is_read_ok != true)
    {
        my_start_timer(MY_TIMER_GSENSOR_BURST,
                       GSENSOR_BURST_SAMPLE_INTERVAL_MS,
                       false,
                       gsensor_sample_timer_cb);
        return;
    }

    state_ret = analyze_gsensor_state(&sensor_data);

    if (state_ret == -1)
    {
        need_burst_sample = true;
    }
    else if (state_ret == 0)
    {
        if (g_gsensor_runtime_ctx.last_gsensor_state != g_gsensor_runtime_ctx.current_gsensor_state)
        {
            g_gsensor_runtime_ctx.last_gsensor_state = g_gsensor_runtime_ctx.current_gsensor_state;
            get_motion_status();
        }

        // 统一滞后机制已确保状态稳定，确认静止后直接进入唤醒待机
        if (g_gsensor_runtime_ctx.current_gsensor_state == STATE_STATIC)
        {
            need_suspend = true;
        }
    }

    if (need_burst_sample == true)
    {
        my_start_timer(MY_TIMER_GSENSOR_BURST,
                       GSENSOR_BURST_SAMPLE_INTERVAL_MS,
                       false,
                       gsensor_sample_timer_cb);
    }

    if (need_suspend == true)
    {
        my_pm_device_suspend(MY_PM_DEV_GSENSOR);
    }
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
    int ret;
    int64_t elapsed_ms;

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
        my_recv_msg(&my_gsensor_msgq, (void *)&msg, sizeof(MSG_S), K_FOREVER);

        switch (msg.msgID)
        {
            case MY_MSG_GSENSOR_READ:
                gsensor_handle_read_msg();
                break;

            case MY_MSG_GSENSOR_PWROFF:
                // 智能模式静止挂起后，设备状态可能已经是 SUSPENDED，但传感器仍处于带电唤醒待机态
                if (my_pm_device_get_state(MY_PM_DEV_GSENSOR) == MY_PM_STATE_SUSPENDED)
                {
                    g_gsensor_runtime_ctx.sensor_ready = false;
                    my_gsensor_pwr_on(false);
                }
                else
                {
                    // 通过电源管理模块挂起 G-Sensor 设备（会自动挂起 I2C21 总线），会调用 gsensor_pm_suspend
                    my_pm_device_suspend(MY_PM_DEV_GSENSOR);
                }
                break;

            case MY_MSG_GSENSOR_PWRON:
                // 通过电源管理模块恢复 G-Sensor 设备（会自动恢复 I2C21 总线），会调用 gsensor_pm_resume
                gsensor_resume_and_read();
                break;

            case MY_MSG_GSENSOR_WAKEUP_INT:
                // 检查保护时间，过滤配置期间产生的假唤醒
                elapsed_ms = k_uptime_get() - g_gsensor_int_ctrl.wakeup_mode_enter_ms;
                if (elapsed_ms < GSENSOR_WAKEUP_GUARD_MS)
                {
                    MY_LOG_INF("wakeup guard ignore elapsed=%lldms < %dms",
                               elapsed_ms, GSENSOR_WAKEUP_GUARD_MS);
                    break;
                }

                if (my_pm_device_get_state(MY_PM_DEV_GSENSOR) == MY_PM_STATE_SUSPENDED)
                {
                    gsensor_resume_and_read();
                }
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

int my_gsensor_read_data(struct gsensor_data *data)
{
    int16_t data_raw[3];

    if (!g_gsensor_runtime_ctx.sensor_ready)
    {
        return -ENODEV;
    }

    if (lsm6dsv16x_acceleration_raw_get(&lsm_ctx, data_raw) == 0)
    {
        data->acc_raw_x = data_raw[0];
        data->acc_raw_y = data_raw[1];
        data->acc_raw_z = data_raw[2];
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
        if (gsensor_apply_run_mode_config() != 0)
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

    k_timer_init(&g_gsensor_int_ctrl.timer, gsensor_int_timer_handler, NULL);
    g_gsensor_int_ctrl.debouncing = false;
    g_gsensor_int_ctrl.wakeup_mode_enter_ms = 0;

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
    *tid = k_thread_create(&my_gsensor_task_data, my_gsensor_task_stack,
                           K_THREAD_STACK_SIZEOF(my_gsensor_task_stack),
                           my_gsensor_task, NULL, NULL, NULL,
                           MY_GSENSOR_TASK_PRIORITY, 0, K_NO_WAIT);

    /* 6. 设置线程名称 */
    k_thread_name_set(*tid, "MY_GSENSOR");

    MY_LOG_INF("G-Sensor module initialized");

    return 0;
}
