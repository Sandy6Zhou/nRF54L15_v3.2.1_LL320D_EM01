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

/* ============================================================
 *  四元数结构体
 * ============================================================ */
typedef struct
{
    float q0; // w
    float q1; // x
    float q2; // y
    float q3; // z
} quaternion_t;

/* ============================================================
 *  欧拉角结构体 (单位: 度)
 *  roll  - 绕X轴旋转 (横滚)
 *  pitch - 绕Y轴旋转 (俯仰)
 *  yaw   - 绕Z轴旋转 (航向)
 * ============================================================ */
typedef struct
{
    float roll;
    float pitch;
    float yaw;
} euler_angle_t;

/* ============================================================
 *  姿态解算上下文
 * ============================================================ */
typedef struct
{
    quaternion_t quat;   // 当前四元数
    euler_angle_t euler; // 当前欧拉角
    bool initialized;    // 是否已初始化
    float kp;            // Mahony 滤波器比例增益
    float ki;            // Mahony 滤波器积分增益
    float integral_fb_x; // 积分反馈项 X
    float integral_fb_y; // 积分反馈项 Y
    float integral_fb_z; // 积分反馈项 Z
    float acc_lpf_x;     // 加速度低通滤波 X (运输场景消振)
    float acc_lpf_y;     // 加速度低通滤波 Y
    float acc_lpf_z;     // 加速度低通滤波 Z
    float gyro_bias_x;   // 陀螺仪零偏估计 X (rad/s, 在线逐步追踪)
    float gyro_bias_y;   // 陀螺仪零偏估计 Y (rad/s)
    float gyro_bias_z;   // 陀螺仪零偏估计 Z (rad/s)
} attitude_ctx_t;

/********************************************************************
**函数名称:  attitude_init
**入口参数:  ctx      ---        姿态解算上下文指针
**出口参数:  无
**函数功能:  初始化姿态解算上下文，四元数设为单位四元数
**返 回 值:  0 表示成功，负值表示参数错误
*********************************************************************/
int attitude_init(attitude_ctx_t *ctx);

/********************************************************************
**函数名称:  attitude_update
**入口参数:  ctx      ---        姿态解算上下文指针
**           reading  ---        IMU 6轴数据 (acc: m/s^2, gyro: rad/s)
**           dt       ---        采样间隔 (秒)
**出口参数:  无
**函数功能:  基于 Mahony 互补滤波更新姿态，融合加速度计与陀螺仪
**返 回 值:  0 表示成功，负值表示参数错误
**注意事项:  无磁力计，yaw 仅有陀螺仪积分，存在漂移
*********************************************************************/
int attitude_update(attitude_ctx_t *ctx, const imu_reading_t *reading, float dt);

/********************************************************************
**函数名称:  attitude_get_euler
**入口参数:  ctx      ---        姿态解算上下文指针
**出口参数:  euler    ---        输出欧拉角 (度)
**函数功能:  从当前四元数提取欧拉角 (roll, pitch, yaw)
**返 回 值:  0 表示成功，负值表示参数错误
*********************************************************************/
int attitude_get_euler(const attitude_ctx_t *ctx, euler_angle_t *euler);

/********************************************************************
**函数名称:  attitude_get_quaternion
**入口参数:  ctx      ---        姿态解算上下文指针
**出口参数:  quat     ---        输出四元数
**函数功能:  获取当前姿态四元数
**返 回 值:  0 表示成功，负值表示参数错误
*********************************************************************/
int attitude_get_quaternion(const attitude_ctx_t *ctx, quaternion_t *quat);

/********************************************************************
**函数名称:  attitude_read_imu_and_update
**入口参数:  ctx      ---        姿态解算上下文指针
**出口参数:  euler    ---        输出当前欧拉角 (度)，可为 NULL
**函数功能:  读取 BMI325 IMU 数据并更新姿态 (便捷接口)
**返 回 值:  0 表示成功，负值表示失败
**注意事项:  内部调用 imu_read() 获取数据并自动换算单位
*********************************************************************/
int attitude_read_imu_and_update(attitude_ctx_t *ctx, euler_angle_t *euler);

#endif
