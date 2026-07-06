/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        accelerometer_api.h
**文件描述:        加速度传感器模块统一 API 接口头文件
**当前版本:        V1.0
**作    者:       周森达 (zhousenda@jimiiot.com)
**完成日期:        2026.07.06
*********************************************************************
** 功能描述:       提供统一的加速度传感器功能接口，屏蔽底层驱动差异
**                支持 DA213 系列三轴加速度传感器
*********************************************************************/

#ifndef _ACCELEROMETER_API_H_
#define _ACCELEROMETER_API_H_

#include <stdint.h>
#include <stdbool.h>

/* 加速度传感器中断回调函数类型 */
typedef void (*accelerometer_int_callback_t)(void);

/* 加速度传感器操作结果 */
typedef enum
{
    ACCEL_SUCCESS = 0,
    ACCEL_ERROR_INIT,                   /* 初始化失败或模块未完成初始化 */
    ACCEL_ERROR_COMM,                   /* 寄存器访问或总线通信失败 */
    ACCEL_ERROR_CHIP_ID,                /* 芯片 ID 校验失败 */
    ACCEL_ERROR_PARAM,                  /* 输入参数无效 */
    ACCEL_ERROR_NOT_READY,              /* 预留结果码：模块未就绪 */
} accel_result_t;

/* 加速度传感器量程 */
typedef enum
{
    ACCEL_RANGE_2G = 0,
    ACCEL_RANGE_4G,
    ACCEL_RANGE_8G,
    ACCEL_RANGE_16G,
    ACCEL_RANGE_MAX,
} accel_range_t;

/* 加速度传感器输出数据率 */
typedef enum
{
    ACCEL_ODR_1HZ = 0,
    ACCEL_ODR_1_95HZ,
    ACCEL_ODR_3_9HZ,
    ACCEL_ODR_7_81HZ,
    ACCEL_ODR_15_63HZ,
    ACCEL_ODR_31_25HZ,
    ACCEL_ODR_62_5HZ,
    ACCEL_ODR_125HZ,
    ACCEL_ODR_250HZ,
    ACCEL_ODR_500HZ,
    ACCEL_ODR_1000HZ,
    ACCEL_ODR_MAX,
} accel_odr_t;

/* 加速度传感器电源模式 */
typedef enum
{
    ACCEL_POWER_NORMAL = 0,
    ACCEL_POWER_LOW_POWER,
    ACCEL_POWER_SUSPEND,
    ACCEL_POWER_MAX,
} accel_power_mode_t;

/* 中断锁存模式 */
typedef enum
{
    ACCEL_INT_LATCH_NON = 0,
    ACCEL_INT_LATCH_250MS,
    ACCEL_INT_LATCH_500MS,
    ACCEL_INT_LATCH_1S,
    ACCEL_INT_LATCH_2S,
    ACCEL_INT_LATCH_4S,
    ACCEL_INT_LATCH_8S,
    ACCEL_INT_LATCH_HOLD,
    ACCEL_INT_LATCH_NON2,
    ACCEL_INT_LATCH_1MS,
    ACCEL_INT_LATCH_1MS2,
    ACCEL_INT_LATCH_2MS,
    ACCEL_INT_LATCH_25MS,
    ACCEL_INT_LATCH_50MS,
    ACCEL_INT_LATCH_100MS,
    ACCEL_INT_LATCH_HOLD2,
} accel_int_latch_t;

/* 中断类型 */
typedef enum
{
    ACCEL_INT_ACTIVE = 0,               /* 运动检测 */
    ACCEL_INT_SINGLE_TAP,               /* 单击 */
    ACCEL_INT_DOUBLE_TAP,               /* 双击 */
    ACCEL_INT_FREEFALL,                 /* 自由落体 */
    ACCEL_INT_ORIENT,                   /* 方向识别 */
    ACCEL_INT_NEWDATA,                  /* 新数据就绪 */
} accel_int_type_t;

/* 三轴加速度数据结构体（mg 为单位） */
struct accel_data
{
    int32_t x_mg;                       /* X轴加速度，单位 mg */
    int32_t y_mg;                       /* Y轴加速度，单位 mg */
    int32_t z_mg;                       /* Z轴加速度，单位 mg */
};

/* 三轴原始数据结构体 */
struct accel_raw_data
{
    int16_t x;                          /* X轴原始值 */
    int16_t y;                          /* Y轴原始值 */
    int16_t z;                          /* Z轴原始值 */
};

/* 加速度传感器初始化配置 */
struct accel_config
{
    accel_range_t range;                /* 量程 */
    accel_odr_t odr;                    /* 输出数据率 */
    accel_power_mode_t power_mode;      /* 电源模式 */
};

/* Active（运动检测）中断配置 */
struct accel_active_int_config
{
    uint8_t threshold;                  /* 阈值寄存器值，LSB与当前量程配置相关 */
    uint8_t duration;                   /* 持续时间寄存器值(0~3)，实际时间=(duration+1)ms */
    bool enable_x;                      /* X轴使能 */
    bool enable_y;                      /* Y轴使能 */
    bool enable_z;                      /* Z轴使能 */
};

/* Tap（敲击检测）中断配置 */
struct accel_tap_int_config
{
    uint8_t threshold;                  /* 阈值寄存器值，仅低5位有效，LSB与当前量程配置相关 */
    uint8_t quiet;                      /* 静默时间配置位：0=30ms，1=20ms */
    uint8_t shock;                      /* 冲击时间配置位：0=50ms，1=70ms */
    uint8_t duration;                   /* 双击窗口寄存器值，仅低3位有效 */
    bool enable_single;                 /* 使能单击 */
    bool enable_double;                 /* 使能双击 */
};

/* Freefall（自由落体）中断配置 */
struct accel_freefall_int_config
{
    uint8_t threshold;                  /* 阈值寄存器值，LSB=7.81mg */
    uint8_t duration;                   /* 持续时间寄存器值，实际时间=(duration+1)*2ms */
    uint8_t hysteresis;                 /* 迟滞寄存器值，仅低2位有效，LSB=125mg */
    bool sum_mode;                      /* true=求和模式，false=单轴模式 */
};

/* Orient（方向识别）中断配置 */
struct accel_orient_int_config
{
    uint8_t mode;                       /* 方向识别模式：0=对称，1=高不对称，2=低不对称 */
    uint8_t blocking;                   /* 阻塞条件寄存器值(0~3) */
    uint8_t hysteresis;                 /* 迟滞寄存器值，仅低3位有效，LSB=62.5mg */
    uint8_t z_blocking;                 /* Z轴阻塞寄存器值，仅低4位有效，LSB=62.5mg */
};

/* 方向状态 */
struct accel_orient_status
{
    uint8_t orient_xy;                  /* XY平面方向状态值：0=竖屏正，1=竖屏反，2=横屏左，3=横屏右 */
    uint8_t orient_z;                   /* Z轴方向状态值：0=朝上，1=朝下 */
};

/* 中断状态 */
struct accel_int_status
{
    bool active;                        /* Active（运动检测）中断触发标志 */
    bool single_tap;                    /* Single Tap（单击）中断触发标志 */
    bool double_tap;                    /* Double Tap（双击）中断触发标志 */
    bool freefall;                      /* Freefall（自由落体）中断触发标志 */
    bool orient;                        /* Orient（方向识别）中断触发标志 */
};

/* 初始化与基本控制 */
accel_result_t accelerometer_init(const struct accel_config *config);
accel_result_t accelerometer_set_power_mode(accel_power_mode_t mode);
accel_result_t accelerometer_set_range(accel_range_t range);
accel_result_t accelerometer_set_odr(accel_odr_t odr);
accel_result_t accelerometer_get_chip_id(uint8_t *id);

/* 数据读取 */
accel_result_t accelerometer_read(struct accel_data *data);
accel_result_t accelerometer_read_raw(struct accel_raw_data *data);

/* 中断配置 */
accel_result_t accelerometer_config_active_int(const struct accel_active_int_config *config);
accel_result_t accelerometer_config_tap_int(const struct accel_tap_int_config *config);
accel_result_t accelerometer_config_freefall_int(const struct accel_freefall_int_config *config);
accel_result_t accelerometer_config_orient_int(const struct accel_orient_int_config *config);
accel_result_t accelerometer_register_int_callback(accelerometer_int_callback_t callback);
accel_result_t accelerometer_disable_int(accel_int_type_t type);
accel_result_t accelerometer_set_int_latch(accel_int_latch_t latch_mode);

/* 状态读取 */
accel_result_t accelerometer_read_int_status(struct accel_int_status *status);
accel_result_t accelerometer_read_orient_status(struct accel_orient_status *status);
accel_result_t accelerometer_reset_int(void);

#endif /* _ACCELEROMETER_API_H_ */
