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
#include "imu_api.h"

/* GSENSOR 运行时上下文全局实例 */
gsensor_runtime_ctx_t g_gsensor_runtime_ctx =
{
    .sensor_ready = false,
    .last_gsensor_state = STATE_STATIC,
    .current_gsensor_state = STATE_STATIC,
};

/* 注册 G-Sensor 模块日志 */
LOG_MODULE_REGISTER(my_gsensor, LOG_LEVEL_INF);

/* 从设备树获取硬件配置 */
#define GSENSOR_PWR_NODE DT_ALIAS(gsensor_pwr_ctrl)
static const struct gpio_dt_spec gsensor_pwr_gpio = GPIO_DT_SPEC_GET(GSENSOR_PWR_NODE, gpios);

/********************************************************************
**函数名称:  gsensor_reg_check
**入口参数:  ret      ---        I2C 操作返回值（输入）
**出口参数:  无
**函数功能:  检查 I2C 寄存器操作返回值，非 0 时直接返回错误码
**返 回 值:  0 表示成功，非 0 表示失败
*********************************************************************/
#define GSENSOR_REG_CHECK(ret) do { if ((ret) != 0) return (ret); } while (0)

/* LSM6DSV16X 芯片 ID */
#define MY_BMI325_ID 0x45
#define TIMESTAMP_QUEUE_SIZE  500   // 队列大小（预留足够空间）

#define MY_IMU_ODR IMU_ODR_100HZ // IMU采样率100Hz

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

typedef struct {
    uint32_t timestamps[TIMESTAMP_QUEUE_SIZE];  // 时间戳队列（环形）
    uint32_t head;                              // 队列头（写入位置）
    uint32_t tail;                              // 队列尾（读取位置）
} sliding_window_t;

// 全局滑动窗口实例
static sliding_window_t g_int_window =
{
    .head = 0,
    .tail = 0,
};

static attitude_ctx_t s_attitude_ctx = {0}; // 存储当前姿态信息

/********************************************************************
**函数名称:  my_gsensor_save_imu_bias
**入口参数:  无
**出口参数:  无
**函数功能:  保存当前 IMU 偏置值到用户数据存储
**返 回 值:  无
*********************************************************************/
void my_gsensor_save_imu_bias(void)
{
    if (g_gsensor_runtime_ctx.sensor_ready == false)
    {
        return;
    }

    gConfigParam.imu_zero_bias_config.gyro_bias_x = s_attitude_ctx.gyro_bias_x;
    gConfigParam.imu_zero_bias_config.gyro_bias_y = s_attitude_ctx.gyro_bias_y;
    gConfigParam.imu_zero_bias_config.gyro_bias_z = s_attitude_ctx.gyro_bias_z;
    my_user_data_write(ZMS_ID_IMU_ZERO_BIAS_CONFIG, &gConfigParam.imu_zero_bias_config, sizeof(imu_zero_bias_config_t));
}

/********************************************************************
**函数名称:  gsensor_is_required_mode
**入口参数:  无
**出口参数:  无
**函数功能:  判断当前工作模式是否为G-Sensor需要开启的模式
**返 回 值:  true 表示需要开启的模式，false 表示其他模式
*********************************************************************/
static bool gsensor_is_required_mode(void)
{
    if (gConfigParam.device_workmode_config.workmode_config.current_mode == MY_MODE_SMART ||
        gConfigParam.device_workmode_config.workmode_config.current_mode == MY_MODE_CONTINUOUS ||
        gConfigParam.device_workmode_config.workmode_config.current_mode == MY_MODE_ALWAYS_ONLINE)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/********************************************************************
**函数名称:  imu_int_dither_timer_cb
**入口参数:  param      ---        无（用于符合回调函数签名）
**出口参数:  无
**函数功能:  IMU 中断去抖定时器回调函数，用于触发 INT1 引脚的中断事件
**返值:    无
*********************************************************************/
void imu_int_dither_timer_cb(void *param)
{
    my_send_msg(MOD_GSENSOR, MOD_GSENSOR, MY_MSG_GSENSOR_INT);
}

/********************************************************************
**函数名称:  gsensor_state_check_timer_cb
**入口参数:  param      ---        无（用于符合回调函数签名）
**出口参数:  无
**函数功能:  G-Sensor 状态检查定时器回调函数，用于检查运动状态
**返值:    无
*********************************************************************/
void gsensor_state_check_timer_cb(void *param)
{
    my_send_msg(MOD_GSENSOR, MOD_GSENSOR, MY_MSG_MOTION_CHECK);
}

/********************************************************************
**函数名称:  imu_int_callback
**入口参数:  无
**出口参数:  无
**函数功能:  IMU 中断回调函数，用于处理 INT1 引脚的中断事件
**返值:    无
*********************************************************************/
void imu_int_callback(void)
{
#if GSENSOR_DUTY_PROJECT
    my_start_timer(MY_TIMER_IMU_INT_DITHER, 1000, false, imu_int_dither_timer_cb);
#else
    my_send_msg(MOD_GSENSOR, MOD_GSENSOR, MY_MSG_GSENSOR_INT);
#endif
}

/********************************************************************
**函数名称:  gsensor_algorithm_timer_cb
**入口参数:  param      ---        无（用于符合回调函数签名）
**出口参数:  无
**函数功能:  G-Sensor 算法定时器回调函数，用于更新姿态解算
**返值:    无
*********************************************************************/
void gsensor_algorithm_timer_cb(void *param)
{
    my_send_msg(MOD_GSENSOR, MOD_GSENSOR, MY_MSG_READ_GSENSOR_DATA);
}


/********************************************************************
**函数名称:  imu_odr_to_hz
**入口参数:  odr      ---        IMU 采样率
**出口参数:  无
**函数功能:  读取当前配置的 IMU 采样率
**返 回 值:  IMU 采样率（Hz）
*********************************************************************/
uint16_t imu_odr_to_hz(imu_odr_t odr)
{
    switch (odr)
    {
        case IMU_ODR_25HZ:
            return 25;

        case IMU_ODR_50HZ:
            return 50;

        case IMU_ODR_100HZ:
            return 100;

        case IMU_ODR_200HZ:
            return 200;

        case IMU_ODR_400HZ:
            return 400;

        case IMU_ODR_800HZ:
            return 800;

        case IMU_ODR_1600HZ:
            return 1600;

        default:
            return 1;
    }
}

/********************************************************************
**函数名称:  gsensor_motion_int_config
**入口参数:  无
**出口参数:  无
**函数功能:  配置 G-Sensor 为运动中断模式
**返 回 值:  0 表示成功，负值表示错误码
*********************************************************************/
static int gsensor_motion_int_config(void)
{
    int ret;
    struct imu_config imu_cfg = { 0 };
    struct imu_int_config imu_int_cfg = { 0 };
    struct imu_any_motion_config imu_motion_cfg = { 0 };

    imu_cfg.acc_odr = MY_IMU_ODR;
    imu_cfg.acc_range = IMU_ACC_RANGE_2G;
    imu_cfg.gyr_odr = MY_IMU_ODR;
    imu_cfg.gyr_range = IMU_GYR_RANGE_250DPS;
    imu_cfg.power_mode = IMU_POWER_LOW_POWER;
    ret = imu_set_config(&imu_cfg);
    GSENSOR_REG_CHECK(ret);

    imu_int_cfg.active_high = true;
    imu_int_cfg.open_drain = false;
    imu_int_cfg.latch = true;
    ret = imu_int_pin_config(IMU_INT_PIN1, &imu_int_cfg);
    GSENSOR_REG_CHECK(ret);

    ret = imu_int_map(IMU_INT_SRC_ANY_MOTION, IMU_INT_PIN1);
    GSENSOR_REG_CHECK(ret);

    imu_motion_cfg.slope_thres = 20; // 灵敏度
    imu_motion_cfg.duration = 9;     //  9*20 = 180ms，防止短促震动误触发
    imu_motion_cfg.hysteresis = 5;   //  滞回消抖，防止阈值附近抖动
    imu_motion_cfg.wait_time = 6;    //  6*20 = 120ms，防止中断过于频繁
    imu_motion_cfg.acc_ref_up = 1;   // Always模式（参考值立即更新，适合检测新运动）

    ret = imu_set_any_motion_config(&imu_motion_cfg);
    GSENSOR_REG_CHECK(ret);

    ret = imu_feature_enable(IMU_FEATURE_ANY_MOTION, true);
    GSENSOR_REG_CHECK(ret);

    ret = imu_register_int_callback(imu_int_callback);
    GSENSOR_REG_CHECK(ret);

    // 等待陀螺仪稳定，确保数据采集准确,不然会导致姿态解算错误，上电不稳定偏航角会直接漂移16度
    k_sleep(K_MSEC(50));

    // 启动状态检测定时器
    my_start_timer(MY_TIMER_GSENSOR_STATE_CHECK, 2000, true, gsensor_state_check_timer_cb);
    // 启动算法定时器
    my_start_timer(MY_TIMER_GSENSOR_ALGORITHM, 1000/imu_odr_to_hz(MY_IMU_ODR), true, gsensor_algorithm_timer_cb);

    return 0;
}

/********************************************************************
**函数名称:  my_gsensor_get_state
**入口参数:  无
**出口参数:  无
**函数功能:  获取当前GSENSOR状态
**返 回 值:  当前GSENSOR状态（静止/运动）
*********************************************************************/
gsensor_state_t my_gsensor_get_state(void)
{
    return g_gsensor_runtime_ctx.current_gsensor_state;
}

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

    if (gConfigParam.motdetalm_config.motdetalm_sw)
    {
        // 发送状态切换告警
        if (state == STATE_STATIC)
        {
            send_alarm_message_to_lte(ALARM_MOVE_START, NULL);
        }
        else
        {
            send_alarm_message_to_lte(ALARM_MOVE_STOP, NULL);
        }
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

#ifdef GSENSOR_DUTY_PROJECT
    my_stop_timer(MY_TIMER_IMU_INT_DITHER);
#endif
    g_gsensor_runtime_ctx.sensor_ready = false;
    my_stop_timer(MY_TIMER_GSENSOR_STATE_CHECK);
    my_stop_timer(MY_TIMER_GSENSOR_ALGORITHM);
    // 其它模式关闭 G-Sensor 电源
    my_gsensor_pwr_on(false);

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

    // 打开 G-Sensor 电源
    my_gsensor_pwr_on(true);

    /* 传感器已断电，需要完整初始化
    * 尝试初始化 LSM6DSV16X 传感器，最多尝试 3 次
    */
    do
    {
        result = my_bmi325_init();
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
**函数名称:  sliding_window_add_int
**入口参数:  window --- 滑动窗口实例（输入）
**出口参数:  无
**函数功能:  添加中断时间差到滑动窗口
**返值:    无
*********************************************************************/
static void sliding_window_add_int(sliding_window_t *window)
{
    uint32_t current_time = k_uptime_get_32();
    uint32_t next_head = (window->head + 1) % TIMESTAMP_QUEUE_SIZE;

    // 关键检查：如果队列即将满（head 即将追上 tail），强制移动 tail
    if (next_head == window->tail)
    {
        // 队列已满，强制丢弃最旧的数据
        window->tail = (window->tail + 1) % TIMESTAMP_QUEUE_SIZE;
        MY_LOG_INF("Sliding window full, discarding oldest timestamp");
    }

    //  写入时间戳到队列
    window->timestamps[window->head] = current_time;

    //  环形队列：head循环递增
    window->head = (window->head + 1) % TIMESTAMP_QUEUE_SIZE;
}

/********************************************************************
**函数名称:  sliding_window_update_tail
**入口参数:  window --- 滑动窗口实例（输入）
**出口参数:  无
**函数功能:  更新 tail 指针，丢弃过期的时间戳
**返值:    无
*********************************************************************/
static void sliding_window_update_tail(sliding_window_t *window)
{
    uint32_t timestamp = 0;
    uint32_t current_time = k_uptime_get_32();
    uint32_t window_size_ms = gConfigParam.motdet_config.motdet_duration * 1000;  // 转换为毫秒

    //  从 tail 开始遍历，丢弃过期的时间戳
    while (window->tail != window->head)
    {
        timestamp = window->timestamps[window->tail];

        // 如果时间戳在窗口内（未过期），停止遍历
        if ((current_time - timestamp) <= window_size_ms)
        {
            break;  // 找到第一个有效的时间戳，停止
        }

        // 时间戳已过期，tail 前移
        window->tail = (window->tail + 1) % TIMESTAMP_QUEUE_SIZE;
    }
}

/********************************************************************
**函数名称:  sliding_window_get_count
**入口参数:  window --- 滑动窗口实例（输入）
**出口参数:  无
**函数功能:  获取窗口内的有效中断次数（从队尾开始遍历，提前终止）
**返值:    最近窗口大小内的中断次数
*********************************************************************/
static uint16_t sliding_window_get_count(sliding_window_t *window)
{
    uint16_t i = 0;
    uint16_t count = 0;

    sliding_window_update_tail(window);

    if (window->tail == window->head)
    {
        return 0;  // 队列为空
    }

    i = window->tail;

    while (i != window->head)
    {
        count++;
        i = (i + 1) % TIMESTAMP_QUEUE_SIZE;
        // 安全检查：防止无限循环（理论上不应该发生）
        if (count >= TIMESTAMP_QUEUE_SIZE)
        {
            MY_LOG_ERR("Sliding window count overflow!");
            break;
        }
    }

    return count;
}

/********************************************************************
**函数名称:  sliding_window_judge_state
**入口参数:  window --- 滑动窗口实例（输入）
**出口参数:  无
**函数功能:  判断运动状态
**返值:    STATE_MOVING（运动）或 STATE_STATIC（静止）
*********************************************************************/
static gsensor_state_t sliding_window_judge_state(sliding_window_t *window)
{
    uint16_t int_count = sliding_window_get_count(window);

    // 判断运动状态
    if (int_count >= gConfigParam.motdet_config.motdet_vibration)
    {
        return STATE_MOTION;  // 窗口内中断次数 ≥ 阈值 → 运动状态
    }
    else
    {
        return STATE_STATIC;  // 窗口内中断次数 < 阈值 → 静止状态
    }
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
     uint16_t int_status = 0;

    imu_read_int_status(IMU_INT_PIN1, &int_status);  // 读取后清除中断标志

    if (int_status & IMU_INT_STATUS_ANY_MOTION)
    {
        sliding_window_add_int(&g_int_window);
    }

}

/********************************************************************
**函数名称:  my_gsensor_overturn_check
**入口参数:  euler_angle --- 欧拉角指针（输入）
**出口参数:  无
**函数功能:  检查是否发生翻转
**返 回 值:  无
*********************************************************************/
void my_gsensor_overturn_check(euler_angle_t *euler_angle)
{
    static int time_count = 0;
    static bool s_is_over_turn = false;

    if (gConfigParam.imu_alm_config.imu_alm_sw == 0)
    {
        s_is_over_turn = false;
        time_count = 0;
        return;
    }
    if ((s_is_over_turn == false) &&
        ((fabsf(euler_angle->roll) > gConfigParam.imu_alm_config.imu_roll_threshold && gConfigParam.imu_alm_config.imu_roll_threshold != 255) ||
        (fabsf(euler_angle->pitch) > gConfigParam.imu_alm_config.imu_pitch_threshold && gConfigParam.imu_alm_config.imu_pitch_threshold != 255) ||
        (fabsf(euler_angle->yaw) > gConfigParam.imu_alm_config.imu_yaw_threshold && gConfigParam.imu_alm_config.imu_yaw_threshold != 255)))
    {
        time_count++;
        if (time_count > gConfigParam.imu_alm_config.imu_duration_time * imu_odr_to_hz(MY_IMU_ODR))
        {
            s_is_over_turn = true;
            time_count = 0;
            send_alarm_message_to_lte(ALARM_FILP, NULL);
            LOG_INF("IMU Over Turn Alarm");
        }
    }
    else if ((s_is_over_turn == true) &&
            ((fabsf(euler_angle->roll) < gConfigParam.imu_alm_config.imu_roll_threshold || gConfigParam.imu_alm_config.imu_roll_threshold == 255) &&
            (fabsf(euler_angle->pitch) < gConfigParam.imu_alm_config.imu_pitch_threshold || gConfigParam.imu_alm_config.imu_pitch_threshold == 255) &&
            (fabsf(euler_angle->yaw) < gConfigParam.imu_alm_config.imu_yaw_threshold || gConfigParam.imu_alm_config.imu_yaw_threshold == 255)))
    {
        time_count++;
        if (time_count > gConfigParam.imu_alm_config.recover_time * imu_odr_to_hz(MY_IMU_ODR))
        {
            s_is_over_turn = false;
            time_count = 0;
            send_alarm_message_to_lte(ALARM_FILP_BACK, NULL);
            LOG_INF("IMU Reversal Recovery Alarm");
        }
    }
    else
    {
        time_count = 0;
    }
}

/********************************************************************
**函数名称:  my_gsensor_read_data
**入口参数:  无
**出口参数:  无
**函数功能:  读取 G-Sensor 原始数据和处理后数据
**返 回 值:  无
*********************************************************************/
void my_gsensor_read_data(void)
{
    int ret = 0;
    euler_angle_t euler_angle = {0};

    ret =  attitude_read_imu_and_update(&s_attitude_ctx, &euler_angle, imu_odr_to_hz(MY_IMU_ODR));
    if (ret < 0)
    {
        return;
    }
    // 打印欧拉角（可选）, 单位：度
    // 注意：欧拉角的顺序是 roll, pitch, yaw
    // 要查看角度时打开日志打印注释
    // LOG_INF("euler_angle: %f, %f, %f", euler_angle.roll, euler_angle.pitch, euler_angle.yaw);
    my_gsensor_overturn_check(&euler_angle);
}

/********************************************************************
**函数名称:  my_gsensor_motion_state_check
**入口参数:  无
**出口参数:  无
**函数功能:  检查 G-Sensor 运动状态
**返 回 值:  无
*********************************************************************/
void my_gsensor_motion_state_check()
{
    uint32_t int_count = 0;

    if (g_gsensor_runtime_ctx.sensor_ready == false)
    {
        return;  // 设备未就绪，不检测
    }

    int_count = sliding_window_get_count(&g_int_window);

    // 判断运动状态
    g_gsensor_runtime_ctx.current_gsensor_state = sliding_window_judge_state(&g_int_window);

    // 如果状态变化，更新并打印日志
    if (g_gsensor_runtime_ctx.last_gsensor_state != g_gsensor_runtime_ctx.current_gsensor_state)
    {
        g_gsensor_runtime_ctx.last_gsensor_state = g_gsensor_runtime_ctx.current_gsensor_state;
        get_motion_status();
        LOG_INF("current_gsensor_state: %d", g_gsensor_runtime_ctx.current_gsensor_state);
    }
    // LOG_INF("current_gsensor_state: %d, int_count: %d", g_gsensor_runtime_ctx.current_gsensor_state, int_count);
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

    if (gsensor_is_required_mode() == true)
    {
        my_pm_device_resume(MY_PM_DEV_GSENSOR);
    }

    MY_LOG_INF("G-Sensor thread started");

    LOG_INF("Training complete!\n");

    for (;;)
    {
        my_recv_msg(&my_gsensor_msgq, (void *)&msg, sizeof(msg_t), K_FOREVER);

        switch (msg.msgID)
        {
            case MY_MSG_GSENSOR_HIGH_POWER:
                // 保持传感器供电
                my_pm_device_resume(MY_PM_DEV_GSENSOR);
                break;

            case MY_MSG_GSENSOR_LOW_POWER:
                    my_pm_device_suspend(MY_PM_DEV_GSENSOR);
                break;

            case MY_MSG_GSENSOR_INT:
                gsensor_int_handler();
                break;

            case MY_MSG_READ_GSENSOR_DATA:
                my_gsensor_read_data();
                break;

            case MY_MSG_MOTION_CHECK:
                my_gsensor_motion_state_check();
                break;

            default:
                break;
        }
    }
}

/********************************************************************
**函数名称:  bmi325_check_id
**入口参数:  无
**出口参数:  无
**函数功能:  检查 BMI325 是否在线
**返 回 值:  true 表示识别成功
*********************************************************************/
bool bmi325_check_id(void)
{
    if (imu_get_chip_id(&s_chip_id) == IMU_SUCCESS)
    {
        if (s_chip_id == MY_BMI325_ID)
        {
            MY_LOG_INF("BMI325 identified ID: 0x%02X", s_chip_id);
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

int my_bmi325_init(void)
{
    uint8_t ret = 0;
    // 1.等待芯片上电稳定
    k_sleep(K_MSEC(50));

    ret = imu_init(NULL);
    if (ret != IMU_SUCCESS)
    {
        MY_LOG_ERR("Failed to initialize BMI325 (err %d)", ret);
        return -ENODEV;
    }

    // 2.识别 BMI325 传感器
    if (bmi325_check_id())
    {
        if (gsensor_is_required_mode() == true)
        {
            if (gsensor_motion_int_config() != 0)
            {
                MY_LOG_ERR("Failed to configure G-Sensor motion interrupt");
                return -ENODEV;
            }
        }
        g_gsensor_runtime_ctx.sensor_ready = true;
        MY_LOG_INF("BMI325 initialized");
    }
    else
    {
        MY_LOG_ERR("BMI325 not detected");
        return -ENODEV;
    }
    return 0;
}

int my_gsensor_init(k_tid_t *tid)
{
    int err;

    /* 1. 检查硬件接口是否就绪 */
    if (!device_is_ready(gsensor_pwr_gpio.port))
    {
        MY_LOG_ERR("GSENSOR Power GPIO device not ready");
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

    /* 3. 初始化消息队列 */
    my_init_msg_handler(MOD_GSENSOR, &my_gsensor_msgq);

    /* 4. 启动 G-Sensor 线程 */
    *tid = k_thread_create(&s_my_gsensor_task_data, my_gsensor_task_stack,
                           K_THREAD_STACK_SIZEOF(my_gsensor_task_stack),
                           my_gsensor_task, NULL, NULL, NULL,
                           MY_GSENSOR_TASK_PRIORITY, 0, K_NO_WAIT);

    /* 5. 设置线程名称 */
    k_thread_name_set(*tid, "MY_GSENSOR");

    MY_LOG_INF("G-Sensor module initialized");

    return 0;
}
