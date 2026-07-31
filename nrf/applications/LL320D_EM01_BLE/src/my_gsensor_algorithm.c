/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        my_gsenser_algorithm.c
**文件描述:        G-Sensor相关算法实现文件
**当前版本:        V1.0
**作    者:        曹阳 (caoyang@jimiiot.com)
**完成日期:        2026.04.30
*********************************************************************
** 功能描述:        1. 6轴G-Sensor相关算法
*********************************************************************/

/* 必须在包含 my_comm.h 之前定义 BLE_LOG_MODULE_ID，避免与 my_ble_log.h 中的默认定义冲突 */
#define BLE_LOG_MODULE_ID BLE_LOG_MOD_SENSOR

#include "imu_api.h"
#include "my_comm.h"
#include "math.h"

/* 注册 G-Sensor 算法 模块日志 */
LOG_MODULE_REGISTER(my_gsensor_algorithm, LOG_LEVEL_INF);

/* ============================================================
 *  常量定义
 * ============================================================ */

/* Mahony 滤波器默认增益 (针对运输设备场景调优)
 * 运输设备振动大、加减速频繁:
 *   kp 适当降低, 减少加速度计高频噪声耦合到姿态
 *   ki 保持较小, 仅缓慢消除陀螺仪零偏 */
#define MAHONY_DEFAULT_KP 0.8f   /* 比例增益: 运输场景降低, 减少振动干扰 */
#define MAHONY_DEFAULT_KI 0.002f /* 积分增益: 缓慢修正陀螺仪零偏 */

/* 加速度模值范围: 用于判断加速度数据是否可靠 (单位: m/s^2)
 * 运输设备存在持续振动和加减速, 收窄窗口以排除线性加速度干扰
 * 仅当加速度模值接近重力加速度 (9.8 m/s^2) 时才用于修正陀螺仪 */
#define GRAVITY_NOMINAL 9.80665f
#define GRAVITY_MIN (GRAVITY_NOMINAL * 0.85f) /* 8.34 m/s^2, 收窄下限 */
#define GRAVITY_MAX (GRAVITY_NOMINAL * 1.15f) /* 11.28 m/s^2, 收窄上限 */

/* 加速度低通滤波系数 (运输场景消振)
 * alpha 越小滤波越强, 响应越慢; 越大滤波越弱, 响应越快
 * 0.2 适合运输场景, 有效抑制发动机/路面振动 */
#define ACC_LOWPASS_ALPHA 0.2f

/* 角度转换 */
#define RAD_TO_DEG (180.0f / 3.14159265358979f)

/* 在线零偏估计参数
 * 当加速度接近重力且陀螺仪角速度很小时, 认为传感器近似静止,
 * 此时陀螺仪输出即为零偏, 用低通滤波逐步追踪
 *
 * GYRO_BIAS_ALPHA: 零偏追踪速率, 越小追踪越慢但更稳定
 *   0.001 对应时间常数约 1000 个采样周期 (100Hz 下约 10 秒)
 *   运输场景需要更慢的追踪, 避免将真实旋转误判为零偏
 *
 * GYRO_STATIONARY_THRESH: 判定静止的角速度阈值 (rad/s)
 *   约 0.5°/s, 低于此值视为无旋转, 可用于零偏估计
 *
 * GYRO_BIAS_MAX: 零偏上限 (rad/s), 约 3°/s
 *   防止异常数据导致零偏估计发散 */
#define GYRO_BIAS_ALPHA 0.001f
#define GYRO_STATIONARY_THRESH 0.00873f /* 0.5°/s -> rad/s */
#define GYRO_BIAS_MAX 0.05236f          /* 3°/s -> rad/s */

/* ============================================================
 *  内联辅助函数
 * ============================================================ */

/* 快速平方根倒数 (Quake III 算法) */
static inline float inv_sqrtf(float x)
{
    float half_x = 0.5f * x;
    float y = x;
    int32_t i = *(int32_t *)&y;

    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (half_x * y * y)); /* 牛顿迭代 1 次 */
    y = y * (1.5f - (half_x * y * y)); /* 牛顿迭代 2 次, 提高精度 */

    return y;
}

/********************************************************************
**函数名称:  attitude_init
**入口参数:  ctx      ---        姿态解算上下文指针
**出口参数:  无
**函数功能:  初始化姿态解算上下文，四元数设为单位四元数
**返 回 值:  0 表示成功，负值表示参数错误
*********************************************************************/
int attitude_init(attitude_ctx_t *ctx)
{
    struct imu_data init_data;

    if (ctx == NULL)
    {
        return -EINVAL;
    }

    /* 单位四元数: q0=1, q1=q2=q3=0 表示无旋转 */
    ctx->quat.q0 = 1.0f;
    ctx->quat.q1 = 0.0f;
    ctx->quat.q2 = 0.0f;
    ctx->quat.q3 = 0.0f;

    ctx->euler.roll = 0.0f;
    ctx->euler.pitch = 0.0f;
    ctx->euler.yaw = 0.0f;

    ctx->initialized = true;
    ctx->kp = MAHONY_DEFAULT_KP;
    ctx->ki = MAHONY_DEFAULT_KI;

    ctx->integral_fb_x = 0.0f;
    ctx->integral_fb_y = 0.0f;
    ctx->integral_fb_z = 0.0f;

    ctx->acc_lpf_x = 0.0f;
    ctx->acc_lpf_y = 0.0f;
    ctx->acc_lpf_z = 0.0f;

    /* 陀螺仪零偏初始值: 从配置中获取, 在线逐步追踪 */
    ctx->gyro_bias_x = gConfigParam.imu_zero_bias_config.gyro_bias_x;
    ctx->gyro_bias_y = gConfigParam.imu_zero_bias_config.gyro_bias_y;
    ctx->gyro_bias_z = gConfigParam.imu_zero_bias_config.gyro_bias_z;

    MY_LOG_INF("Attitude algorithm initialized (Mahony filter for transport, kp=%.2f, ki=%.4f)",
               ctx->kp, ctx->ki);

    return 0;
}

/********************************************************************
**函数名称:  attitude_update
**入口参数:  ctx      ---        姿态解算上下文指针
**           reading  ---        IMU 6轴数据 (acc: m/s^2, gyro: rad/s)
**           dt       ---        采样间隔 (秒)
**出口参数:  无
**函数功能:  基于 Mahony 互补滤波更新姿态
**返 回 值:  0 表示成功，负值表示参数错误
**注意事项:  Mahony 滤波器利用重力方向作为参考修正陀螺仪漂移:
**           1. 归一化加速度计数据
**           2. 根据当前姿态四元数计算重力在传感器坐标系的投影
**           3. 加速度计测量值与重力投影的叉积即为修正误差
**           4. 用 PI 控制器将误差反馈到陀螺仪角速度
**           5. 一阶龙格-库塔法积分四元数
*********************************************************************/
int attitude_update(attitude_ctx_t *ctx, const imu_reading_t *reading, float dt)
{
    float norm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float gx, gy, gz;
    float qa, qb, qc;
    float acc_norm;

    if (ctx == NULL || reading == NULL || dt <= 0.0f)
    {
        return -EINVAL;
    }

    if (!ctx->initialized)
    {
        MY_LOG_WRN("Attitude context not initialized, auto-initializing");
        attitude_init(ctx);
    }

    gx = reading->gyro_x;
    gy = reading->gyro_y;
    gz = reading->gyro_z;

    /* 陀螺仪零偏补偿: 扣除在线估计的零偏值 */
    gx -= ctx->gyro_bias_x;
    gy -= ctx->gyro_bias_y;
    gz -= ctx->gyro_bias_z;

    /* 保存零偏补偿后的陀螺仪角速度 (未叠加 PI 反馈)
     * 用于后续在线零偏估计的静止判定, 避免被 PI 反馈干扰 */
    float gx_compensated = gx;
    float gy_compensated = gy;
    float gz_compensated = gz;

    /* 加速度低通滤波 (一阶 IIR), 抑制运输场景的发动机/路面高频振动
     * lpf_out = alpha * raw + (1 - alpha) * lpf_out_prev */
    ctx->acc_lpf_x = ACC_LOWPASS_ALPHA * reading->acc_x + (1.0f - ACC_LOWPASS_ALPHA) * ctx->acc_lpf_x;
    ctx->acc_lpf_y = ACC_LOWPASS_ALPHA * reading->acc_y + (1.0f - ACC_LOWPASS_ALPHA) * ctx->acc_lpf_y;
    ctx->acc_lpf_z = ACC_LOWPASS_ALPHA * reading->acc_z + (1.0f - ACC_LOWPASS_ALPHA) * ctx->acc_lpf_z;

    /* 计算滤波后加速度模值，判断数据可靠性 */
    acc_norm = sqrtf(ctx->acc_lpf_x * ctx->acc_lpf_x +
                     ctx->acc_lpf_y * ctx->acc_lpf_y +
                     ctx->acc_lpf_z * ctx->acc_lpf_z);

    /* 仅当加速度接近重力加速度时才进行修正 (排除自由落体/大加速度场景) */
    if (acc_norm > GRAVITY_MIN && acc_norm < GRAVITY_MAX)
    {
        /* 归一化滤波后的加速度 */
        norm = inv_sqrtf(ctx->acc_lpf_x * ctx->acc_lpf_x +
                         ctx->acc_lpf_y * ctx->acc_lpf_y +
                         ctx->acc_lpf_z * ctx->acc_lpf_z);
        float ax = ctx->acc_lpf_x * norm;
        float ay = ctx->acc_lpf_y * norm;
        float az = ctx->acc_lpf_z * norm;

        /* 预计算四元数乘积项 */
        q0q0 = ctx->quat.q0 * ctx->quat.q0;
        q0q1 = ctx->quat.q0 * ctx->quat.q1;
        q0q2 = ctx->quat.q0 * ctx->quat.q2;
        q0q3 = ctx->quat.q0 * ctx->quat.q3;
        q1q1 = ctx->quat.q1 * ctx->quat.q1;
        q1q2 = ctx->quat.q1 * ctx->quat.q2;
        q1q3 = ctx->quat.q1 * ctx->quat.q3;
        q2q2 = ctx->quat.q2 * ctx->quat.q2;
        q2q3 = ctx->quat.q2 * ctx->quat.q3;
        q3q3 = ctx->quat.q3 * ctx->quat.q3;

        /* 重力方向在传感器坐标系的投影 (当前四元数姿态下的期望重力方向)
         * 参考系重力 = [0, 0, 1], 旋转到传感器坐标系 */
        halfvx = 2.0f * (q1q3 - q0q2);
        halfvy = 2.0f * (q0q1 + q2q3);
        halfvz = q0q0 - q1q1 - q2q2 + q3q3;

        /* 叉积: 修正误差 = 实际加速度 × 期望重力方向 */
        halfex = ay * halfvz - az * halfvy;
        halfey = az * halfvx - ax * halfvz;

        /* 积分反馈 (PI 控制器的 I 项) */
        ctx->integral_fb_x += ctx->ki * halfex * dt;
        ctx->integral_fb_y += ctx->ki * halfey * dt;

        /* 比例 + 积分反馈叠加到陀螺仪角速度 */
        gx += ctx->kp * halfex + ctx->integral_fb_x;
        gy += ctx->kp * halfey + ctx->integral_fb_y;
    }

    /* 一阶龙格-库塔法积分四元数
     * q_dot = 0.5 * q ⊗ [0, gx, gy, gz] */
    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;

    qa = ctx->quat.q0;
    qb = ctx->quat.q1;
    qc = ctx->quat.q2;

    ctx->quat.q0 += (-qb * gx - qc * gy - ctx->quat.q3 * gz);
    ctx->quat.q1 += (qa * gx + qc * gz - ctx->quat.q3 * gy);
    ctx->quat.q2 += (qa * gy - qb * gz + ctx->quat.q3 * gx);
    ctx->quat.q3 += (qa * gz + qb * gy - qc * gx);

    /* 归一化四元数 */
    norm = inv_sqrtf(ctx->quat.q0 * ctx->quat.q0 +
                     ctx->quat.q1 * ctx->quat.q1 +
                     ctx->quat.q2 * ctx->quat.q2 +
                     ctx->quat.q3 * ctx->quat.q3);
    ctx->quat.q0 *= norm;
    ctx->quat.q1 *= norm;
    ctx->quat.q2 *= norm;
    ctx->quat.q3 *= norm;

    /* 在线零偏估计: 当传感器近似静止时, 陀螺仪输出即为零偏
     * 判定条件: 加速度接近重力 (acc_norm 已在上方计算) AND 陀螺仪角速度很小
     * 使用零偏补偿后的角速度 (未叠加 PI 反馈) 判定静止, 避免被 PI 反馈干扰
     * 使用低通滤波逐步追踪零偏, 不会将真实旋转误判为零偏 */
    {
        float gyro_norm = sqrtf(gx_compensated * gx_compensated +
                                gy_compensated * gy_compensated +
                                gz_compensated * gz_compensated);

        if (acc_norm > GRAVITY_MIN && acc_norm < GRAVITY_MAX &&
            gyro_norm < GYRO_STATIONARY_THRESH)
        {
            /* 静止时补偿后仍有残差, 说明零偏估计需要微调 */
            ctx->gyro_bias_x += GYRO_BIAS_ALPHA * gx_compensated;
            ctx->gyro_bias_y += GYRO_BIAS_ALPHA * gy_compensated;
            ctx->gyro_bias_z += GYRO_BIAS_ALPHA * gz_compensated;

            /* 限幅: 防止零偏估计发散 */
            if (fabsf(ctx->gyro_bias_x) > GYRO_BIAS_MAX)
                ctx->gyro_bias_x = copysignf(GYRO_BIAS_MAX, ctx->gyro_bias_x);
            if (fabsf(ctx->gyro_bias_y) > GYRO_BIAS_MAX)
                ctx->gyro_bias_y = copysignf(GYRO_BIAS_MAX, ctx->gyro_bias_y);
            if (fabsf(ctx->gyro_bias_z) > GYRO_BIAS_MAX)
                ctx->gyro_bias_z = copysignf(GYRO_BIAS_MAX, ctx->gyro_bias_z);
        }
    }

    /* 从四元数提取欧拉角 (Z-Y-X 旋转顺序) */
    attitude_get_euler(ctx, &ctx->euler);

    return 0;
}

/********************************************************************
**函数名称:  attitude_get_euler
**入口参数:  ctx      ---        姿态解算上下文指针
**出口参数:  euler    ---        输出欧拉角 (度)
**函数功能:  从当前四元数提取欧拉角 (Z-Y-X 旋转顺序)
**返 回 值:  0 表示成功，负值表示参数错误
**注意事项:  roll ∈ [-180, 180], pitch ∈ [-90, 90], yaw ∈ [-180, 180]
**           无磁力计，yaw 仅有陀螺仪积分，会持续漂移
*********************************************************************/
int attitude_get_euler(const attitude_ctx_t *ctx, euler_angle_t *euler)
{
    float sinr_cosp, cosr_cosp, sinp, siny_cosp, cosy_cosp;

    if (ctx == NULL || euler == NULL)
    {
        return -EINVAL;
    }

    /* Roll (X轴旋转) */
    sinr_cosp = 2.0f * (ctx->quat.q0 * ctx->quat.q1 + ctx->quat.q2 * ctx->quat.q3);
    cosr_cosp = 1.0f - 2.0f * (ctx->quat.q1 * ctx->quat.q1 + ctx->quat.q2 * ctx->quat.q2);
    euler->roll = atan2f(sinr_cosp, cosr_cosp) * RAD_TO_DEG;

    /* Pitch (Y轴旋转), 处理万向锁 */
    sinp = 2.0f * (ctx->quat.q0 * ctx->quat.q2 - ctx->quat.q3 * ctx->quat.q1);
    if (fabsf(sinp) >= 1.0f)
    {
        euler->pitch = copysignf(90.0f, sinp); /* 万向锁: ±90° */
    }
    else
    {
        euler->pitch = asinf(sinp) * RAD_TO_DEG;
    }

    /* Yaw (Z轴旋转) */
    siny_cosp = 2.0f * (ctx->quat.q0 * ctx->quat.q3 + ctx->quat.q1 * ctx->quat.q2);
    cosy_cosp = 1.0f - 2.0f * (ctx->quat.q2 * ctx->quat.q2 + ctx->quat.q3 * ctx->quat.q3);
    euler->yaw = atan2f(siny_cosp, cosy_cosp) * RAD_TO_DEG;

    return 0;
}

/********************************************************************
**函数名称:  attitude_get_quaternion
**入口参数:  ctx      ---        姿态解算上下文指针
**出口参数:  quat     ---        输出四元数
**函数功能:  获取当前姿态四元数
**返 回 值:  0 表示成功，负值表示参数错误
*********************************************************************/
int attitude_get_quaternion(const attitude_ctx_t *ctx, quaternion_t *quat)
{
    if (ctx == NULL || quat == NULL)
    {
        return -EINVAL;
    }

    *quat = ctx->quat;
    return 0;
}

/********************************************************************
**函数名称:  attitude_read_imu_and_update
**入口参数:  ctx      ---        姿态解算上下文指针
**入口参数:  odr      ---        IMU 采样率
**出口参数:  euler    ---        输出当前欧拉角 (度)，可为 NULL
**函数功能:  读取 BMI325 IMU 数据并更新姿态 (便捷接口)
**返 回 值:  0 表示成功，负值表示失败
**注意事项:  内部调用 imu_read() 获取数据，自动将单位从
**           mg -> m/s^2, mdps -> rad/s 进行换算
*********************************************************************/
int attitude_read_imu_and_update(attitude_ctx_t *ctx, euler_angle_t *euler, uint16_t odr)
{
    struct imu_data data;
    imu_reading_t reading;
    imu_result_t ret;
    float dt;

    if (ctx == NULL)
    {
        return -EINVAL;
    }

    /* 读取 IMU 换算数据 */
    ret = imu_read(&data);
    if (ret != IMU_SUCCESS)
    {
        MY_LOG_ERR("imu_read failed: %d", ret);
        return -EIO;
    }

    /* 单位换算: mg -> m/s^2 */
    reading.acc_x = (float)data.acc_x * GRAVITY_NOMINAL / 1000.0f;
    reading.acc_y = (float)data.acc_y * GRAVITY_NOMINAL / 1000.0f;
    reading.acc_z = (float)data.acc_z * GRAVITY_NOMINAL / 1000.0f;

    /* 单位换算: mdps -> rad/s */
    reading.gyro_x = (float)data.gyr_x * (3.14159265358979f / 180000.0f);
    reading.gyro_y = (float)data.gyr_y * (3.14159265358979f / 180000.0f);
    reading.gyro_z = (float)data.gyr_z * (3.14159265358979f / 180000.0f);

    /* 根据 IMU 配置的 ODR 计算采样间隔
     * 当前默认 100Hz ODR -> dt = 0.01s */
    dt = 1.0f / (float)odr;

    /* 更新姿态 */
    ret = attitude_update(ctx, &reading, dt);
    if (ret != 0)
    {
        return ret;
    }

    /* 输出欧拉角 */
    if (euler != NULL)
    {
        *euler = ctx->euler;
    }

    return 0;
}
