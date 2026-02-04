/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        my_motor.h
**文件描述:        电机控制模块实现头文件
**当前版本:        V1.0
**作    者:        周森达 (zhousenda@jimiiot.com)
**完成日期:        2026.02.04
*********************************************************************/

#ifndef _MY_MOTOR_H_
#define _MY_MOTOR_H_

void motor_power_set(bool on);
void motor_forward(void);
void motor_reverse(void);
void motor_stop(void);
int motor_gpio_init(void);

#endif