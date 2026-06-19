/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        my_gsenser_algorithm.h
**文件描述:        G-Sensor相关算法实现头文件
**当前版本:        V1.0
**作    者:        曹阳 (caoyang@jimiiot.com)
**完成日期:        2026.04.30
*********************************************************************/
#ifndef _MY_GSENSOR_ALGORITHM_H_
#define _MY_GSENSOR_ALGORITHM_H_

#include "my_gsensor.h"

#define IMU_SAMPLE_RATE  30.0                   /* IMU采样率: 30Hz, 即每秒30个采样点, extract_features函数需要用它做除法转换float,不然除数为0，所以后面要加.0 */
#define PI 3.14159265358979323846                /* 圆周率常量 */
#define NUM_MODES 3                              /* 运输模式数量: 静止/陆运/海运 */
#define FEATURE_DIM 8                            /* 特征向量维度: 8个关键特征 */

/* ============================================================
 *  特征结构体: 从IMU数据中提取的全部特征
 * ============================================================ */
typedef struct {
    float acc_mean;                              /* 加速度幅值均值 */
    float acc_std;                               /* 加速度幅值标准差 */
    float acc_max;                               /* 加速度幅值最大值 */
    float acc_min;                               /* 加速度幅值最小值 */
    float acc_peak_to_peak;                      /* 加速度峰峰值(最大-最小) */
    float acc_linear_std;                        /* 线性加速度(去重力)标准差 */
    float acc_linear_rms;                        /* 线性加速度(去重力)均方根 */
    float gravity_mag;                           /* 重力向量幅值 */
    float gyro_mean;                             /* 角速度幅值均值 */
    float gyro_std;                              /* 角速度幅值标准差 */
    float gyro_max;                              /* 角速度幅值最大值 */
    float gyro_rms;                              /* 角速度幅值均方根 */
    float acc_dominant_freq;                     /* 加速度主频率(Hz) */
    float acc_low_freq_ratio;                    /* 加速度低频能量比(<1Hz) */
    float acc_mid_freq_ratio;                    /* 加速度中频能量比(1-3Hz) */
    float acc_high_freq_ratio;                   /* 加速度高频能量比(>3Hz) */
    float gyro_low_freq_ratio;                   /* 角速度低频能量比(<1Hz) */
    float acc_periodicity;                       /* 加速度周期性(自相关峰值) */
    float gyro_periodicity;                      /* 角速度周期性(自相关峰值) */
} features_t;

/* ============================================================
 *  分类结果结构体: 分类器输出
 * ============================================================ */
typedef struct {
    gsensor_state_t mode;                           /* 判定的运输模式 */
    float confidence;                            /* 置信度(0~1) */
    features_t features;                            /* 本次提取的特征(调试用) */
} classification_result_t;

/* ============================================================
 *  状态机结构体: 时序平滑 + 转移约束
 * ============================================================ */
typedef struct {
    gsensor_state_t current_mode;                   /* 当前确认的运输模式 */
    gsensor_state_t candidate_mode;                 /* 候选切换模式(等待确认) */
    int candidate_count;                          /* 候选模式连续出现的次数 */
    float smooth_prob[NUM_MODES];                /* EMA平滑后的各模式概率 */
    int initialized;                              /* 状态机是否已初始化(0=未, 1=已) */
} state_machine_t;

/* ============================================================
 *  单类模型结构体: 存储一个运输模式的统计参数
 * ============================================================ */
typedef struct {
    float mean[FEATURE_DIM];                     /* 各特征维度的均值 */
    float std[FEATURE_DIM];                      /* 各特征维度的标准差 */
} class_model_t;

/* ============================================================
 *  贝叶斯分类器结构体: 含模型参数和先验概率
 * ============================================================ */
typedef struct {
    class_model_t models[NUM_MODES];                 /* 三个运输模式的统计模型 */
    float prior[NUM_MODES];                      /* 三个运输模式的先验概率 */
    int trained;                                  /* 是否已完成训练(0=未, 1=已) */
} bayesian_classifier_t;

extern state_machine_t sm_batch;                          // 状态机实例

/********************************************************************
**函数名称:  bayes_init
**入口参数:  bclf  ---        贝叶斯分类器指针
**出口参数:  无
**函数功能:  初始化贝叶斯分类器
**  先验概率设为均匀分布(1/3)
*********************************************************************/
void bayes_init(bayesian_classifier_t *bclf);

/********************************************************************
**函数名称:  classify_bayesian
**入口参数:  bclf  ---        贝叶斯分类器指针
**入口参数:  readings  ---        IMU数据指针
**入口参数:  n  ---        数据长度
**函数功能:  贝叶斯分类器核心推理函数
**出口参数: classification_result_t(模式+置信度+特征)
**贝叶斯定理: P(mode|features) = P(features|mode) * P(mode) / P(features)
**    P(features|mode) = 似然 = prod_d Gaussian(x_d; mean_d, std_d)
**    P(mode) = 先验(动态调整)
**    对数空间计算避免下溢: log_posterior = log_lik + log_prior
*********************************************************************/
classification_result_t classify_bayesian(bayesian_classifier_t *bclf,
                                       const imu_reading_t *readings, int n);

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
void bayes_set_dynamic_prior(bayesian_classifier_t *bclf, gsensor_state_t prev_mode);

/********************************************************************
**函数名称:  mode_to_best
**入口参数:  mode  ---        运输模式枚举
**出口参数:  无
**函数功能:  将运输模式枚举转换为最大概率模式索引
*********************************************************************/
int mode_to_best(gsensor_state_t mode);

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
gsensor_state_t sm_update(state_machine_t *sm, const float raw_prob[NUM_MODES]);

/********************************************************************
**函数名称:  sm_init
**入口参数:  sm  ---        状态机指针
**出口参数:  无
**函数功能:  初始化状态机
**  默认状态为静止, 概率全部分配给静止
*********************************************************************/
void sm_init(state_machine_t *sm);

#endif

