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

#include "my_comm.h"

/* 注册 G-Sensor 算法 模块日志 */
LOG_MODULE_REGISTER(my_gsensor_algorithm, LOG_LEVEL_INF);

/*
 *逻辑处理：
 *1. 设备采集6轴G-Sensor数据, 一次采集250个点, 频率25Hz, 窗口时长10秒
 *2. 数据预处理: 窗口均值分离重力向量, 得到线性加速度
 *3. 特征提取: 提取8维特征向量 - 时域3个 + 频域3个 + 周期性2个
 *4. 贝叶斯分类: 利用预训练的mean/std模型, 计算三类后验概率
 *5. 状态机防护: 连续3次确认才切换, 禁止Still与Sea直接互跳
 *6. 输出最终稳定的运输模式: 静止/陆运/海运
 */

/* ============================================================
 *  状态机参数宏定义
 * ============================================================ */
#define SMOOTH_ALPHA 0.3   // EMA平滑系数: 越大响应越快, 越小越平滑
#define TRANSITION_COUNT 3 // 状态切换需要连续确认的次数

#define MAX_FFT_SIZE 256
static float s_re_buf[MAX_FFT_SIZE] = {0}; // 实部缓冲区
static float s_im_buf[MAX_FFT_SIZE] = {0}; // 虚部缓冲区

static float s_acc_mag[MAX_FFT_SIZE] = {0};           // 加速度幅值缓存
static float s_acc_lin_x[MAX_FFT_SIZE] = {0};         // 加速度线性分量缓存
static float s_acc_lin_y[MAX_FFT_SIZE] = {0};         // 加速度线性分量缓存
static float s_acc_lin_z[MAX_FFT_SIZE] = {0};         // 加速度线性分量缓存
static float s_gyro_mag[MAX_FFT_SIZE] = {0};          // 角速度幅值缓存
static float s_acc_detrend[MAX_FFT_SIZE] = {0};       // 去均值后加速度缓存
static float s_gyro_detrend[MAX_FFT_SIZE] = {0};      // 去均值后角速度缓存
static float s_fft_sig[MAX_FFT_SIZE] = {0};           // 加速度FFT输入缓存
static float s_g_fft_sig[MAX_FFT_SIZE] = {0};         // 角速度FFT输入缓存
static float s_fft_mag[MAX_FFT_SIZE / 2 + 1] = {0};   // 加速度FFT频谱幅值缓存
static float s_g_fft_mag[MAX_FFT_SIZE / 2 + 1] = {0}; // 角速度FFT频谱幅值缓存

// 模型参数
static const float s_model_mean[NUM_MODES][FEATURE_DIM] = {
    { 0.008365f, 0.014033f, 0.033619f, 3.083147f, 0.023120f, 0.928201f, 0.000314f, 0.000315f },  /* STILL */
    { 1.806281f, 2.253712f, 24.116830f, 2.957589f, 0.005272f, 0.547244f, 0.001347f, 0.001314f },  /* LAND */
    { 0.173224f, 0.862367f, 17.205777f, 4.192243f, 0.049493f, 0.740867f, 0.000852f, 0.001869f },  /* SEA */
};

static const float s_model_std[NUM_MODES][FEATURE_DIM] = {
    { 0.002393f, 0.004556f, 0.001000f, 1.393205f, 0.010910f, 0.027894f, 0.001000f, 0.001000f },  /* STILL */
    { 0.277943f, 0.279381f, 3.642073f, 0.130125f, 0.003516f, 0.199636f, 0.001000f, 0.001000f },  /* LAND */
    { 0.028869f, 0.074845f, 2.709519f, 0.538015f, 0.016170f, 0.052227f, 0.001000f, 0.001000f },  /* SEA */
};

/********************************************************************
**函数名称:  vec3_mag
**入口参数:  x, y, z  ---        三个分量
**出口参数:  sqrtf(x^2 + y^2 + z^2)
**函数功能:  计算三维向量的幅值(欧几里得范数)
**返 回 值:  无
*********************************************************************/
static float vec3_mag(float x, float y, float z)
{
    return sqrtf(x * x + y * y + z * z); // 向量模长公式
}

/********************************************************************
**函数名称:  arr_mean
**入口参数:  data, n  ---        数组指针, 数组长度
**出口参数:  无
**函数功能:  计算数组的算术平均值
**返 回 值:  无
*********************************************************************/
static float arr_mean(const float *data, int n)
{
    float s = 0; // 累加器初始化
    int i = 0;

    if (n <= 0)
    {
        return 0; // 数组长度为0时, 返回0
    }

    for (i = 0; i < n; i++)
    {
        s += data[i]; /* 逐元素累加 */
    }
    return s / n; // 返回平均值
}

/********************************************************************
**函数名称:  arr_stddev
**入口参数:  data, n  ---        数组指针, 数组长度
**出口参数:  无
**函数功能:  计算数组的标准差(总体标准差)
**返 回 值:  无
*********************************************************************/
static float arr_stddev(const float *data, int n)
{
    float m = 0; // 均值
    float s = 0; // 方差累加器
    int i = 0;

    if (n <= 0)
    {
        return 0; // 数组长度为0时, 返回0
    }

    m = arr_mean(data, n); // 先求均值
    for (i = 0; i < n; i++)
    {
        s += (data[i] - m) * (data[i] - m); // 累加平方差
    }
    return sqrtf(s / n); // 返回标准差
}

/********************************************************************
**函数名称:  rfft_magnitude
**入口参数:  signal, n, mag  ---        信号数组指针, 长度(必须为2的幂), 频谱幅值数组指针
**出口参数:  无
**函数功能:  实数FFT, 计算信号频谱幅值)
**返 回 值:  无
**实现:  基2蝶形运算FFT (Cooley-Tukey算法)
*********************************************************************/
static void rfft_magnitude(const float *signal, int n, float *mag)
{
    int bits = 0;    // 比特计数器
    int tmp = 0;     // 临时变量
    int i = 0;       // 循环变量
    int j = 0;       // 循环变量
    int rev = 0;     // 反转后的索引
    int b = 0;       // 比特计数器
    int half = 0;    // 半长度计数器
    float tr = 0;    // 临时实部变量
    float ti = 0;    // 临时虚部变量
    int len = 0;     // 蝶形运算长度
    float angle = 0; // 旋转因子角度
    float wre = 0;   // 旋转因子实部
    float wim = 0;   // 旋转因子虚部
    float cre = 0;   // 当前旋转因子实部
    float cim = 0;   // 当前旋转因子虚部
    float ure = 0;   // 上半部分实部
    float uim = 0;   // 上半部分虚部
    float vre = 0;   // 下半部分*旋转因子 实部
    float vim = 0;   // 下半部分虚部
    float ncre = 0;  // 下半部分*旋转因子 实部
    float ncim = 0;  // 下半部分*旋转因子 虚部

    // 检查输入是否为2的幂
    if (n <= 0 || n > MAX_FFT_SIZE || (n & (n - 1)) != 0)
    {
        LOG_INF("rfft_magnitude: n=%d is not a power of 2 or out of range", n);
        return; // 无效参数
    }

    memcpy(s_re_buf, signal, n * sizeof(float)); // 复制输入信号到实部缓冲区
    memset(s_im_buf, 0, n * sizeof(float));      // 初始化虚部缓冲区为0

    /* --- 步骤1: 计算FFT所需的比特数 --- */
    bits = 0; // 比特计数器
    for (tmp = n; tmp > 1; tmp >>= 1)
    {
        bits++; // n=2^bits, 计算bits
    }

    /* --- 步骤2: 比特反转排列(蝶形运算需要) --- */
    for (i = 0; i < n; i++) // 遍历每个索引
    {
        rev = 0; // 初始化反转位置
        for (b = 0; b < bits; b++)
        { // 逐比特处理
            if (i & (1 << b))
                rev |= (1 << (bits - 1 - b)); // 如果第b位为1, 设置反转位置
        }
        if (i < rev)
        { // 只交换一次(避免重复交换)
            tr = s_re_buf[i];
            s_re_buf[i] = s_re_buf[rev];
            s_re_buf[rev] = tr; // 交换实部
            ti = s_im_buf[i];
            s_im_buf[i] = s_im_buf[rev];
            s_im_buf[rev] = ti; // 交换虚部
        }
    }

    /* --- 步骤3: 蝶形运算(逐级合并) --- */
    for (len = 2; len <= n; len <<= 1)
    {                            // len=当前蝶形运算的长度: 2,4,8,...,n
        angle = -2.0 * PI / len; // 旋转因子角度: -2*pi/len
        wre = cos(angle);        // 旋转因子实部
        wim = sin(angle);        // 旋转因子虚部
        for (i = 0; i < n; i += len)
        { // 处理每个蝶形组
            cre = 1.0;
            cim = 0.0; // 当前旋转因子, 初始为1+0j
            for (j = 0; j < len / 2; j++)
            {                                                                            // 组内蝶形运算
                ure = s_re_buf[i + j];                                                   // 上半部分实部
                uim = s_im_buf[i + j];                                                   // 上半部分虚部
                vre = cre * s_re_buf[i + j + len / 2] - cim * s_im_buf[i + j + len / 2]; // 下半部分*旋转因子 实部
                vim = cre * s_im_buf[i + j + len / 2] + cim * s_re_buf[i + j + len / 2]; // 下半部分*旋转因子 虚部
                s_re_buf[i + j] = ure + vre;                                             // 合并: 上=上+下*W
                s_im_buf[i + j] = uim + vim;                                             // 合并: 上=上+下*W (虚部)
                s_re_buf[i + j + len / 2] = ure - vre;                                   // 合并: 下=上-下*W
                s_im_buf[i + j + len / 2] = uim - vim;                                   // 合并: 下=上-下*W (虚部)
                ncre = cre * wre - cim * wim;                                            // 旋转因子递推: W^k = W^(k-1) * W (实部)
                ncim = cre * wim + cim * wre;                                            // 旋转因子递推: W^k = W^(k-1) * W (虚部)
                cre = ncre;                                                              // 更新旋转因子实部
                cim = ncim;                                                              // 更新旋转因子虚部
            }
        }
    }

    /* --- 步骤4: 计算幅值(仅正频率部分) --- */
    half = n / 2; // 正频率点数 = N/2
    for (i = 0; i <= half; i++)
    {                                                                              // 遍历0~N/2频率点
        mag[i] = sqrtf(s_re_buf[i] * s_re_buf[i] + s_im_buf[i] * s_im_buf[i]) / n; // 幅值 = |X[k]| / N (归一化)
    }
}

/********************************************************************
**函数名称:  autocorr_max
**入口参数:  signal, n, max_lag  ---        信号数组指针, 长度, 最大滞后值
**出口参数:  无
**函数功能:  归一化自相关最大值 (0~1, 越大越周期性)
**原理:  R(lag) = sum((x[i]-mean)*(x[i+lag]-mean)) / (var*(n-lag))
    海运摇摆周期6-20s, 自相关峰值高; 陆运/噪声无规律, 峰值低
*********************************************************************/
static float autocorr_max(const float *signal, int n, int max_lag)
{
    int i = 0;        // 循环索引
    int best_lag = 0; // 最佳滞后值
    int lag = 0;      // 滞后值索引
    float mean = 0;   // 信号均值
    float var = 0;    // 方差累加器
    float best = 0;   // 最大自相关值
    float sum = 0;    // 自相关累加器

    mean = arr_mean(signal, n); // 计算信号均值
    var = 0;                    // 方差累加器
    for (i = 0; i < n; i++)
    {
        var += (signal[i] - mean) * (signal[i] - mean); // 累加平方偏差
    }
    if (var < 1e-12)
    {
        return 0.0; // 信号方差太小(接近常数), 无周期性
    }

    best = 0;     // 最大自相关值
    best_lag = 0; // 对应的滞后值
    for (lag = 2; lag <= max_lag && lag < n; lag++)
    {            // 遍历所有滞后值, 从2开始避免lag=0/1
        sum = 0; // 自相关累加器
        for (i = 0; i < n - lag; i++)
        {                                                         // 计算滞后lag的自相关
            sum += (signal[i] - mean) * (signal[i + lag] - mean); // 互乘累加
        }
        sum /= (var * (n - lag)); // 归一化: 除以方差和有效点数
        if (sum > best)
        {                   // 找最大值
            best = sum;     // 更新最大自相关值
            best_lag = lag; // 更新最佳滞后
        }
    }
    return best; // 返回最大归一化自相关值
}

/********************************************************************
**函数名称:  extract_features
**入口参数:  readings, n  ---        IMU数据数组指针, 数据长度
**出口参数:  无
**函数功能:  特征提取流程:
    1. 计算重力方向(低通滤波近似)
    2. 分离线性加速度和重力分量
    3. 时域特征: 均值/标准差/峰峰值/RMS
    4. 频域特征: FFT主频率/频带能量比
    5. 周期性特征: 自相关峰值
*********************************************************************/
void extract_features(const IMUReading *readings, int n, Features *feat)
{
    int i = 0;       // 循环索引
    int fft_n = 0;   /* FFT长度 */
    int max_bin = 0; /* 5Hz对应的FFT bin索引 */
    int dom_bin = 0; /* 主频率bin */
    int g_fft_n = 0; /* 角速度FFT长度 */
    int max_lag = 0; // 最大滞后=5秒对应的采样点数
    float gx = 0;
    float gy = 0;
    float gz = 0;        /* 重力分量累加器 */
    float g_mag = 0;     /* 重力向量幅值 */
    float lin_rms = 0;   /* 线性加速度RMS累加器 */
    float gyro_rms = 0;  /* 角速度RMS累加器 */
    float lm = 0;        /* 线性加速度幅值 */
    float lin_x_std = 0; /* X轴线性加速度标准差 */
    float lin_y_std = 0; /* Y轴线性加速度标准差 */
    float lin_z_std = 0; /* Z轴线性加速度标准差 */
    float acc_m = 0;     /* 加速度幅值均值 */
    float freq_res = 0;  /* FFT频率分辨率 */
    float dom_val = 0;   /* 主频率幅值 */
    float total_e = 0;
    float low_e = 0;
    float mid_e = 0;
    float high_e = 0; /* 能量累加器 */
    float e = 0;      /* 单频能量 */
    float f = 0;      /* 当前频率(Hz) */
    float gm = 0;     /* 角速度幅值均值 */
    float g_total_e = 0;
    float g_low_e = 0; /* 角速度能量累加器 */

    if (n <= 0 || n > MAX_FFT_SIZE)
    {
        LOG_ERR("extract_features: n=%d is out of range", n);
        return; // 无效参数
    }

    /* --- 步骤1: 估计重力方向(窗口内加速度均值) --- */
    for (i = 0; i < n; i++)
    {                            /* 遍历所有采样点 */
        gx += readings[i].acc_x; /* 累加X轴加速度 */
        gy += readings[i].acc_y; /* 累加Y轴加速度 */
        gz += readings[i].acc_z; /* 累加Z轴加速度 */
    }
    gx /= n;                      /* X轴加速度均值(重力X分量) */
    gy /= n;                      /* Y轴加速度均值(重力Y分量) */
    gz /= n;                      /* Z轴加速度均值(重力Z分量) */
    g_mag = vec3_mag(gx, gy, gz); /* 重力向量幅值(应接近9.81) */

    /* --- 步骤2: 计算幅值和线性加速度 --- */
    for (i = 0; i < n; i++)
    {                                                                                         /* 遍历所有采样点 */
        s_acc_mag[i] = vec3_mag(readings[i].acc_x, readings[i].acc_y, readings[i].acc_z);     /* 加速度幅值 */
        s_acc_lin_x[i] = readings[i].acc_x - gx;                                              /* X轴线性加速度 = 原始 - 重力 */
        s_acc_lin_y[i] = readings[i].acc_y - gy;                                              /* Y轴线性加速度 */
        s_acc_lin_z[i] = readings[i].acc_z - gz;                                              /* Z轴线性加速度 */
        s_gyro_mag[i] = vec3_mag(readings[i].gyro_x, readings[i].gyro_y, readings[i].gyro_z); /* 角速度幅值 */
    }

    /* --- 步骤3: 时域特征 - 加速度 --- */
    feat->acc_mean = arr_mean(s_acc_mag, n);  /* 加速度幅值均值 */
    feat->acc_std = arr_stddev(s_acc_mag, n); /* 加速度幅值标准差(振动强度指标) */
    feat->acc_max = s_acc_mag[0];             /* 初始化最大值 */
    feat->acc_min = s_acc_mag[0];             /* 初始化最小值 */
    for (i = 1; i < n; i++)
    { /* 遍历找极值 */
        if (s_acc_mag[i] > feat->acc_max)
        {
            feat->acc_max = s_acc_mag[i]; /* 更新最大值 */
        }
        if (s_acc_mag[i] < feat->acc_min)
        {
            feat->acc_min = s_acc_mag[i]; /* 更新最小值 */
        }
    }
    feat->acc_peak_to_peak = feat->acc_max - feat->acc_min; /* 峰峰值 = 最大 - 最小 */
    feat->gravity_mag = g_mag;                              /* 重力幅值 */

    /* --- 步骤4: 时域特征 - 线性加速度RMS --- */
    for (i = 0; i < n; i++)
    {                                                                  /* 遍历所有点 */
        lm = vec3_mag(s_acc_lin_x[i], s_acc_lin_y[i], s_acc_lin_z[i]); /* 线性加速度幅值 */
        lin_rms += lm * lm;                                            /* 累加幅值平方 */
    }
    feat->acc_linear_rms = sqrt(lin_rms / n); /* RMS = sqrt(均值(平方)) */

    /* --- 步骤5: 时域特征 - 线性加速度三轴合成标准差 --- */
    lin_x_std = arr_stddev(s_acc_lin_x, n);                                                             /* X轴线性加速度标准差 */
    lin_y_std = arr_stddev(s_acc_lin_y, n);                                                             /* Y轴线性加速度标准差 */
    lin_z_std = arr_stddev(s_acc_lin_z, n);                                                             /* Z轴线性加速度标准差 */
    feat->acc_linear_std = sqrt(lin_x_std * lin_x_std + lin_y_std * lin_y_std + lin_z_std * lin_z_std); /* 三轴合成 */

    /* --- 步骤6: 时域特征 - 角速度 --- */
    feat->gyro_mean = arr_mean(s_gyro_mag, n);  /* 角速度幅值均值 */
    feat->gyro_std = arr_stddev(s_gyro_mag, n); /* 角速度幅值标准差(旋转强度指标) */
    feat->gyro_max = s_gyro_mag[0];             /* 初始化角速度最大值 */
    for (i = 1; i < n; i++)
    { /* 遍历找最大值 */
        if (s_gyro_mag[i] > feat->gyro_max)
            feat->gyro_max = s_gyro_mag[i]; /* 更新最大值 */
    }
    for (i = 0; i < n; i++)
    {
        gyro_rms += s_gyro_mag[i] * s_gyro_mag[i]; /* 累加幅值平方 */
    }
    feat->gyro_rms = sqrt(gyro_rms / n);           /* 角速度RMS */

    /* --- 步骤7: 频域特征 - 加速度FFT --- */
    acc_m = arr_mean(s_acc_mag, n); /* 加速度幅值均值 */
    for (i = 0; i < n; i++)
        s_acc_detrend[i] = s_acc_mag[i] - acc_m; /* 去均值(消除直流分量) */

    fft_n = 1; /* FFT长度, 初始化为1 */
    while (fft_n < n)
        fft_n <<= 1;                                     /* 找到>=n的最小2的幂(FFT要求) */
    memcpy(s_fft_sig, s_acc_detrend, n * sizeof(float)); /* 复制去均值数据 */

    rfft_magnitude(s_fft_sig, fft_n, s_fft_mag); /* 执行FFT, 获取频谱 */

    /* --- 步骤8: 主频率检测(0~5Hz范围内) --- */
    freq_res = IMU_SAMPLE_RATE / fft_n; /* 频率分辨率 = 采样率/FFT长度 */
    max_bin = (int)(5.0 / freq_res);    /* 5Hz对应的FFT bin索引 */
    if (max_bin < 1)
    {
        max_bin = 1; /* 至少检查bin=1 */
    }
    dom_bin = 1;            /* 主频率bin, 从1开始(跳过直流) */
    dom_val = s_fft_mag[1]; /* 主频率幅值, 初始化为bin=1 */
    for (i = 2; i <= max_bin; i++)
    { /* 遍历2~5Hz的频率点 */
        if (s_fft_mag[i] > dom_val)
        {                           /* 找最大幅值 */
            dom_val = s_fft_mag[i]; /* 更新最大幅值 */
            dom_bin = i;            /* 更新主频率bin */
        }
    }
    feat->acc_dominant_freq = dom_bin * freq_res; /* 主频率(Hz) = bin索引 * 频率分辨率 */

    /* --- 步骤9: 频带能量比(低/中/高频) --- */
    for (i = 1; i <= fft_n / 2; i++)
    {                                    /* 遍历正频率部分(跳过直流) */
        e = s_fft_mag[i] * s_fft_mag[i]; /* 单频能量 = 幅值^2 */
        total_e += e;                    /* 累加总能量 */
        f = i * freq_res;                /* 当前频率(Hz) */
        if (f < 1.0)
            low_e += e; /* 低频: <1Hz (海运摇摆) */
        else if (f < 3.0)
            mid_e += e; /* 中频: 1-3Hz (陆运低频) */
        else
            high_e += e; /* 高频: >3Hz (陆运发动机) */
    }
    if (total_e > 1e-12)
    {                                                 /* 避免除以0 */
        feat->acc_low_freq_ratio = low_e / total_e;   /* 低频能量占比 */
        feat->acc_mid_freq_ratio = mid_e / total_e;   /* 中频能量占比 */
        feat->acc_high_freq_ratio = high_e / total_e; /* 高频能量占比 */
    }
    else
    {                                  /* 信号能量极小 */
        feat->acc_low_freq_ratio = 0;  /* 低频比置0 */
        feat->acc_mid_freq_ratio = 0;  /* 中频比置0 */
        feat->acc_high_freq_ratio = 0; /* 高频比置0 */
    }

    /* --- 步骤10: 频域特征 - 角速度FFT --- */
    gm = arr_mean(s_gyro_mag, n); /* 角速度幅值均值 */
    for (i = 0; i < n; i++)
        s_gyro_detrend[i] = s_gyro_mag[i] - gm; /* 去均值 */

    g_fft_n = 1; /* 角速度FFT长度 */
    while (g_fft_n < n)
        g_fft_n <<= 1;                                      /* 找到>=n的最小2的幂 */
    memcpy(s_g_fft_sig, s_gyro_detrend, n * sizeof(float)); /* 复制去均值角速度 */

    rfft_magnitude(s_g_fft_sig, g_fft_n, s_g_fft_mag); /* 执行角速度FFT */

    for (i = 1; i <= g_fft_n / 2; i++)
    {                                        /* 遍历正频率 */
        e = s_g_fft_mag[i] * s_g_fft_mag[i]; /* 单频能量 */
        g_total_e += e;                      /* 累加总能量 */
        if (i * freq_res < 1.0)
            g_low_e += e; /* 累加低频能量(<1Hz) */
    }
    feat->gyro_low_freq_ratio = (g_total_e > 1e-12) ? g_low_e / g_total_e : 0; /* 角速度低频能量比 */

    /* --- 步骤11: 周期性特征 - 自相关 --- */
    max_lag = (int)(5.0 * IMU_SAMPLE_RATE); /* 最大滞后=5秒对应的采样点数 */
    if (max_lag > n / 2)
    {
        max_lag = n / 2; /* 不超过半窗口长度 */
    }
    feat->acc_periodicity = autocorr_max(s_acc_detrend, n, max_lag);   /* 加速度周期性 */
    feat->gyro_periodicity = autocorr_max(s_gyro_detrend, n, max_lag); /* 角速度周期性 */
}

/********************************************************************
**函数名称:  features_to_vector
**入口参数:  f, vec  ---        特征结构体指针, 8维特征向量
**出口参数:  无
**函数功能:  只选取对分类最有区分度的8个特征:
    [0] acc_std:          加速度标准差(陆运>海运>静止)
    [1] acc_linear_rms:   线性加速度RMS(陆运最大)
    [2] gyro_std:         角速度标准差(海运>陆运>静止)
    [3] acc_dominant_freq: 主频率(海运<1Hz, 陆运1-3Hz)
    [4] acc_low_freq_ratio: 低频能量比(海运最高)
    [5] acc_high_freq_ratio: 高频能量比(陆运最高)
    [6] acc_periodicity:  加速度周期性(海运最高)
    [7] gyro_periodicity: 角速度周期性(海运最高)
*********************************************************************/
static void features_to_vector(const Features *f, float vec[FEATURE_DIM])
{
    vec[0] = f->acc_std;             /* 加速度标准差 */
    vec[1] = f->acc_linear_rms;      /* 线性加速度RMS */
    vec[2] = f->gyro_std;            /* 角速度标准差 */
    vec[3] = f->acc_dominant_freq;   /* 加速度主频率 */
    vec[4] = f->acc_low_freq_ratio;  /* 低频能量比 */
    vec[5] = f->acc_high_freq_ratio; /* 高频能量比 */
    vec[6] = f->acc_periodicity;     /* 加速度周期性 */
    vec[7] = f->gyro_periodicity;    /* 角速度周期性 */
}

/********************************************************************
**函数名称:  transition_allowed
**入口参数:  无
**出口参数:  无
**函数功能:  状态转移矩阵: 定义哪些状态转换是合法的
**  transition_allowed[当前状态][目标状态] = 1允许 / 0禁止
**  业务约束:
**    Still <-> Land: 允许 (装车/卸货)
**    Land  <-> Sea:  允许 (装船/卸船)
**    Still <-> Sea:  禁止 (必须经过陆运中转)
*********************************************************************/
static const int transition_allowed[NUM_MODES][NUM_MODES] =
{
        {1, 1, 0}, /* Still -> Still/Land/Sea */
        {1, 1, 1}, /* Land  -> Still/Land/Sea */
        {0, 1, 1}, /* Sea   -> Still/Land/Sea */
};

/********************************************************************
**函数名称:  sm_init
**入口参数:  sm  ---        状态机指针
**出口参数:  无
**函数功能:  初始化状态机
**  默认状态为静止, 概率全部分配给静止
*********************************************************************/
void sm_init(StateMachine *sm)
{
    sm->current_mode = STATE_UNKNOWN;     /* 初始状态: 未知 */
    sm->candidate_mode = STATE_UNKNOWN;   /* 候选模式: 未知 */
    sm->candidate_count = 0;           /* 候选计数: 0 */
    sm->smooth_prob[0] = 1.0 / 3; /* 静止概率: 1/3 */
    sm->smooth_prob[1] = 1.0 / 3;  /* 陆运概率: 1/3 */
    sm->smooth_prob[2] = 1.0 / 3;   /* 海运概率: 1/3 */
    sm->initialized = 0;               /* 尚未初始化(等待首次输入) */
}

/********************************************************************
**函数名称:  best_to_mode
**入口参数:  best  ---        最大概率模式索引(0=静止, 1=陆运, 2=海运)
**出口参数:  无
**函数功能:  将最大概率模式索引转换为运输模式枚举
*********************************************************************/
gsensor_state_t best_to_mode(int best)
{
    switch (best)
    {
        case 0:
            return STATE_STATIC;
        break;

        case 1:
            return STATE_LAND_TRANSPORT;
        break;

        case 2:
            return STATE_SEA_TRANSPORT;
        break;

        default:
            return STATE_UNKNOWN;
    }
}

/********************************************************************
**函数名称:  mode_to_best
**入口参数:  mode  ---        运输模式枚举
**出口参数:  无
**函数功能:  将运输模式枚举转换为最大概率模式索引
*********************************************************************/
int mode_to_best(gsensor_state_t mode)
{
    switch (mode)
    {
        case STATE_STATIC:
            return 0;
        break;

        case STATE_LAND_TRANSPORT:
            return 1;
        break;

        case STATE_SEA_TRANSPORT:
            return 2;
        break;

        default:
            return -1;
    }
}


/********************************************************************
**函数名称:  sm_update
**入口参数:  sm  ---        状态机指针
**出口参数:  无
**函数功能:  状态机更新(核心逻辑)
**  输入: sm=原始分类概率[NUM_MODES]
**  输出: 平滑后的运输模式
**  三重保护机制:
**    1. EMA平滑: 消除单次分类的概率抖动
**    2. 非法转移拦截: Still不能直接跳到Sea
**    3. N次确认: 连续N次检测到新模式才切换
*********************************************************************/
gsensor_state_t sm_update(StateMachine *sm, const float raw_prob[NUM_MODES])
{
    int m = 0;    /* 模式索引 */
    int best = 0; /* 最大概率模式索引 */
    gsensor_state_t new_mode = STATE_UNKNOWN; /* 平滑后的新模式 */

    /* --- 首次调用: 直接初始化 --- */
    if (!sm->initialized)
    { /* 如果尚未初始化 */
        for (m = 0; m < NUM_MODES; m++)
        {
            sm->smooth_prob[m] = raw_prob[m]; /* 直接使用原始概率 */
        }
        sm->initialized = 1; /* 标记已初始化 */
        best = 0;            /* 最大概率模式索引 */
        for (m = 1; m < NUM_MODES; m++)
        { /* 遍历找最大 */
            if (raw_prob[m] > raw_prob[best])
            {
                best = m; /* 更新最佳 */
            }
        }
        sm->current_mode = best_to_mode(best); /* 设置当前模式 */
        sm->candidate_mode = sm->current_mode;  /* 候选模式同当前 */
        return sm->current_mode;                /* 返回初始模式 */
    }

    /* --- 保护机制1: EMA概率平滑 --- */
    for (m = 0; m < NUM_MODES; m++)
    { /* 遍历每种模式 */
        sm->smooth_prob[m] = SMOOTH_ALPHA * raw_prob[m] + (1.0 - SMOOTH_ALPHA) * sm->smooth_prob[m];
        /* 新概率 = alpha*原始 + (1-alpha)*历史, alpha=0.3表示70%权重给历史 */
    }

    /* --- 找到平滑后概率最大的模式 --- */
    best = 0; /* 最大概率模式索引 */
    for (m = 1; m < NUM_MODES; m++)
    { /* 遍历 */
        if (sm->smooth_prob[m] > sm->smooth_prob[best])
            best = m; /* 更新最佳 */
    }
    new_mode = best_to_mode(best); /* 平滑后的新模式 */

    /* --- 状态切换逻辑 --- */
    if (new_mode != sm->current_mode && new_mode != STATE_UNKNOWN)
    { /* 如果检测到模式变化 */
        /* --- 保护机制2: 非法转移拦截 --- */
        if (!transition_allowed[mode_to_best(sm->current_mode)][mode_to_best(new_mode)])
        {
            return sm->current_mode; /* 禁止的转移, 忽略, 保持当前 */
        }
        /* --- 保护机制3: N次确认切换 --- */
        if (new_mode == sm->candidate_mode)
        {                          /* 如果与候选模式一致 */
            sm->candidate_count++; /* 候选计数+1 */
            if (sm->candidate_count >= TRANSITION_COUNT)
            {                                /* 达到确认次数 */
                sm->current_mode = new_mode; /* 执行状态切换 */
                sm->candidate_count = 0;     /* 重置计数器 */
            }
        }
        else
        {                                  /* 新的候选模式 */
            sm->candidate_mode = new_mode; /* 更新候选模式 */
            sm->candidate_count = 1;       /* 计数器从1开始 */
        }
    }
    else
    {                                          /* 模式未变化 */
        sm->candidate_count = 0;               /* 重置候选计数 */
        sm->candidate_mode = sm->current_mode; /* 候选模式同当前 */
    }

    return sm->current_mode; /* 返回当前(可能已切换)模式 */
}

/********************************************************************
**函数名称:  bayes_init
**入口参数:  bclf  ---        贝叶斯分类器指针
**出口参数:  无
**函数功能:  初始化贝叶斯分类器
**  先验概率设为均匀分布(1/3)
*********************************************************************/
void bayes_init(BayesianClassifier *bclf)
{
    int m = 0;          // 模式索引

    for (m = 0; m < NUM_MODES; m++)
    {
        memcpy(bclf->models[m].mean, s_model_mean[m], FEATURE_DIM * sizeof(float));
        memcpy(bclf->models[m].std,  s_model_std[m],  FEATURE_DIM * sizeof(float));
    }
    bclf->prior[0] = 1.0 / NUM_MODES; /* 静止先验 = 1/3 */
    bclf->prior[1] = 1.0 / NUM_MODES;  /* 陆运先验 = 1/3 */
    bclf->prior[2] = 1.0 / NUM_MODES;   /* 海运先验 = 1/3 */
    bclf->trained = 1;                            /* 标记已训练完成 */
}

/********************************************************************
**函数名称:  bayes_set_dynamic_prior
**入口参数:  bclf  ---        贝叶斯分类器指针
**入口参数:  prev_mode  ---        上一时刻的运输模式枚举
**函数功能:  根据上一时刻的判定结果, 调整当前时刻的先验
**  核心思想: 运输状态具有惯性, 不太可能突然跳变
**    - 如果上一时刻是Still: 大概率继续Still, 可能转Land, 不可能转Sea
**    - 如果上一时刻是Land:  大概率继续Land, 可能转Still或Sea
**    - 如果上一时刻是Sea:   大概率继续Sea, 可能转Land, 不可能转Still
*********************************************************************/
void bayes_set_dynamic_prior(BayesianClassifier *bclf, gsensor_state_t prev_mode)
{
    switch (prev_mode)
    {                                       /* 根据上一时刻模式 */
        case STATE_STATIC:                    /* 上一时刻: 静止 */
            bclf->prior[0] = 0.60; /* 静止持续概率60% */
            bclf->prior[1] = 0.35;  /* 可能开始陆运35% */
            bclf->prior[2] = 0.05;   /* 不可能直接转海运5% */
            break;

        case STATE_LAND_TRANSPORT:                     /* 上一时刻: 陆运 */
            bclf->prior[0] = 0.20; /* 可能停车20% */
            bclf->prior[1] = 0.50;  /* 陆运持续概率50% */
            bclf->prior[2] = 0.30;   /* 可能装船30% */
            break;

        case STATE_SEA_TRANSPORT:                      /* 上一时刻: 海运 */
            bclf->prior[0] = 0.05; /* 不可能直接转静止5% */
            bclf->prior[1] = 0.35;  /* 可能卸船35% */
            bclf->prior[2] = 0.60;   /* 海运持续概率60% */
            break;

        default:
            bclf->prior[0] = 1.0 / 3; /* 静止先验 = 1/3 */
            bclf->prior[1] = 1.0 / 3;  /* 陆运先验 = 1/3 */
            bclf->prior[2] = 1.0 / 3;   /* 海运先验 = 1/3 */
            break;
    }
}

/********************************************************************
**函数名称:  classify_bayesian
**入口参数:  bclf  ---        贝叶斯分类器指针
**入口参数:  readings  ---        IMU数据指针
**入口参数:  n  ---        数据长度
**函数功能:  贝叶斯分类器核心推理函数
**出口参数: ClassificationResult(模式+置信度+特征)
**贝叶斯定理: P(mode|features) = P(features|mode) * P(mode) / P(features)
**    P(features|mode) = 似然 = prod_d Gaussian(x_d; mean_d, std_d)
**    P(mode) = 先验(动态调整)
**    对数空间计算避免下溢: log_posterior = log_lik + log_prior
*********************************************************************/
ClassificationResult classify_bayesian(BayesianClassifier *bclf,
                                       const IMUReading *readings, int n)
{
    int m = 0;                            /* 模式索引 */
    int d = 0;                            /* 特征维度索引 */
    Features feat;                        /* 特征结构体 */
    ClassificationResult result = {STATE_UNKNOWN, 0.0, {0}};        /* 返回结果 */
    float vec[FEATURE_DIM];              /* 8维特征向量 */
    float log_posterior[NUM_MODES]; /* 各模式对数后验 */
    float log_lik = 0; /* 对数似然累加器 */
    float diff = 0; /* 标准化偏差 */
    float log_prior = 0; /* 对数先验 */
    float max_lp = 0; /* 最大对数后验 */
    float exp_sum = 0;        /* exp值总和 */
    float exp_val[NUM_MODES]; /* 各模式exp值 */
    int best = 0;                            /* 最佳模式索引 */
    float best_prob = 0.0; /* 归一化后验概率 */
    float prob = 0.0; /*归一化概率 */

    /* --- 检查分类器是否已训练 --- */
    if (!bclf->trained)
    {                                                    /* 未训练 */
        return result;
    }

    /* --- 步骤1: 提取特征并转为向量 --- */
    extract_features(readings, n, &feat); /* 提取全部特征 */
    features_to_vector(&feat, vec);       /* 转为向量 */

    /* --- 步骤2: 计算对数后验 = 对数似然 + 对数先验 --- */
    for (m = 0; m < NUM_MODES; m++)
    {                       /* 遍历每种模式 */
        log_lik = 0; /* 对数似然累加器 */
        for (d = 0; d < FEATURE_DIM; d++)
        {                                                                              /* 遍历每个特征维度 */
            diff = (vec[d] - bclf->models[m].mean[d]) / bclf->models[m].std[d]; /* 标准化偏差(Mahalanobis) */
            log_lik += -0.5 * diff * diff - log(bclf->models[m].std[d]);               /* 高斯对数似然: -0.5*z^2 - log(sigma) */
        }
        log_prior = log(bclf->prior[m] + 1e-12); /* 对数先验, +1e-12避免log(0) */
        log_posterior[m] = log_lik + log_prior;         /* 对数后验 = 对数似然 + 对数先验 */
    }

    /* --- 步骤3: LogSumExp技巧计算归一化概率 --- */
    max_lp = log_posterior[0]; /* 找最大对数后验(数值稳定) */
    for (m = 1; m < NUM_MODES; m++)
    { /* 遍历 */
        if (log_posterior[m] > max_lp)
            max_lp = log_posterior[m]; /* 更新最大值 */
    }

    for (m = 0; m < NUM_MODES; m++)
    {                                                /* 遍历 */
        exp_val[m] = exp(log_posterior[m] - max_lp); /* exp(log_p - max), 减去最大值避免溢出 */
        exp_sum += exp_val[m];                       /* 累加exp值 */
    }

    /* --- 步骤4: 找到后验概率最大的模式 --- */
    best_prob = exp_val[0] / exp_sum; /* 归一化后验概率 */
    for (m = 1; m < NUM_MODES; m++)
    {                                       /* 遍历 */
        prob = exp_val[m] / exp_sum; /* 归一化概率 */
        if (prob > best_prob)
        {                     /* 找最大 */
            best_prob = prob; /* 更新最大概率 */
            best = m;         /* 更新最佳模式 */
        }
    }

    /* --- 构造返回结果 --- */
    result.mode = best_to_mode(best); /* 判定模式 */
    result.confidence = best_prob;     /* 置信度(后验概率) */
    result.features = feat;            /* 保存特征(调试用) */
    return result;                     /* 返回分类结果 */
}
