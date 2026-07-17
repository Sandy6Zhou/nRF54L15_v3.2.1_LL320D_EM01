/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        imu_api.h
**文件描述:        BMI325 IMU 模块统一接口头文件
**当前版本:        V1.0
*********************************************************************/

#ifndef _IMU_API_H_
#define _IMU_API_H_

#include <stdbool.h>
#include <stdint.h>
#include "../vendor/bmi325.h"

typedef enum
{
    IMU_SUCCESS = 0,
    IMU_ERROR_INIT,
    IMU_ERROR_COMM,
    IMU_ERROR_CHIP_ID,
    IMU_ERROR_PARAM,
    IMU_ERROR_TIMEOUT,
} imu_result_t;

typedef enum
{
    IMU_ACC_RANGE_2G = 0,
    IMU_ACC_RANGE_4G,
    IMU_ACC_RANGE_8G,
    IMU_ACC_RANGE_16G,
    IMU_ACC_RANGE_MAX,
} imu_acc_range_t;

typedef enum
{
    IMU_GYR_RANGE_125DPS = 0,
    IMU_GYR_RANGE_250DPS,
    IMU_GYR_RANGE_500DPS,
    IMU_GYR_RANGE_1000DPS,
    IMU_GYR_RANGE_2000DPS,
    IMU_GYR_RANGE_MAX,
} imu_gyr_range_t;

typedef enum
{
    IMU_ODR_12_5HZ = 0,
    IMU_ODR_25HZ,
    IMU_ODR_50HZ,
    IMU_ODR_100HZ,
    IMU_ODR_200HZ,
    IMU_ODR_400HZ,
    IMU_ODR_800HZ,
    IMU_ODR_1600HZ,
    IMU_ODR_MAX,
} imu_odr_t;

typedef enum
{
    IMU_POWER_SUSPEND = 0,
    IMU_POWER_LOW_POWER,
    IMU_POWER_NORMAL,
    IMU_POWER_HIGH_PERF,
    IMU_POWER_MAX,
} imu_power_mode_t;

typedef enum
{
    IMU_INT_NONE = 0,
    IMU_INT_PIN1,
    IMU_INT_PIN2,              /* 预留 INT2 映射，当前硬件板级未接入 MCU GPIO */
    IMU_INT_PIN_MAX,
} imu_int_pin_t;

typedef enum
{
    IMU_INT_SRC_ACC_DRDY = 0,
    IMU_INT_SRC_GYR_DRDY,
    IMU_INT_SRC_TEMP_DRDY,
    IMU_INT_SRC_FIFO_WATERMARK,
    IMU_INT_SRC_FIFO_FULL,
    IMU_INT_SRC_ANY_MOTION,
    IMU_INT_SRC_NO_MOTION,
    IMU_INT_SRC_FLAT,
    IMU_INT_SRC_ORIENTATION,
    IMU_INT_SRC_STEP_DETECTOR,
    IMU_INT_SRC_STEP_COUNTER,
    IMU_INT_SRC_SIG_MOTION,
    IMU_INT_SRC_TILT,
    IMU_INT_SRC_TAP,
    IMU_INT_SRC_FEATURE_STATUS,
    IMU_INT_SRC_MAX,
} imu_int_src_t;

/* BMI325 中断状态标志位定义（导出给用户使用） */
#define IMU_INT_STATUS_NO_MOTION       BMI3_INT_STATUS_NO_MOTION
#define IMU_INT_STATUS_ANY_MOTION      BMI3_INT_STATUS_ANY_MOTION
#define IMU_INT_STATUS_FLAT            BMI3_INT_STATUS_FLAT
#define IMU_INT_STATUS_ORIENTATION     BMI3_INT_STATUS_ORIENTATION
#define IMU_INT_STATUS_STEP_DETECTOR   BMI3_INT_STATUS_STEP_DETECTOR
#define IMU_INT_STATUS_STEP_COUNTER    BMI3_INT_STATUS_STEP_COUNTER
#define IMU_INT_STATUS_SIG_MOTION      BMI3_INT_STATUS_SIG_MOTION
#define IMU_INT_STATUS_TILT            BMI3_INT_STATUS_TILT
#define IMU_INT_STATUS_TAP             BMI3_INT_STATUS_TAP
#define IMU_INT_STATUS_I3C             BMI3_INT_STATUS_I3C
#define IMU_INT_STATUS_ERR             BMI3_INT_STATUS_ERR
#define IMU_INT_STATUS_TEMP_DRDY       BMI3_INT_STATUS_TEMP_DRDY
#define IMU_INT_STATUS_GYR_DRDY        BMI3_INT_STATUS_GYR_DRDY
#define IMU_INT_STATUS_ACC_DRDY        BMI3_INT_STATUS_ACC_DRDY
#define IMU_INT_STATUS_FWM             BMI3_INT_STATUS_FWM
#define IMU_INT_STATUS_FFULL           BMI3_INT_STATUS_FFULL  /* FIFO 满中断 */

typedef enum
{
    IMU_FEATURE_NO_MOTION = 0,
    IMU_FEATURE_ANY_MOTION,
    IMU_FEATURE_FLAT,
    IMU_FEATURE_ORIENTATION,
    IMU_FEATURE_STEP_DETECTOR,
    IMU_FEATURE_STEP_COUNTER,
    IMU_FEATURE_SIG_MOTION,
    IMU_FEATURE_TILT,
    IMU_FEATURE_TAP_SINGLE,
    IMU_FEATURE_TAP_DOUBLE,
    IMU_FEATURE_TAP_TRIPLE,
    IMU_FEATURE_MAX,
} imu_feature_t;

struct imu_data
{
    int32_t acc_x;
    int32_t acc_y;
    int32_t acc_z;
    int32_t gyr_x;
    int32_t gyr_y;
    int32_t gyr_z;
    int16_t temperature;
};

struct imu_raw_data
{
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t gyr_x;
    int16_t gyr_y;
    int16_t gyr_z;
    int16_t temperature;
};

struct imu_config
{
    imu_acc_range_t acc_range;
    imu_odr_t acc_odr;
    imu_gyr_range_t gyr_range;
    imu_odr_t gyr_odr;
    imu_power_mode_t power_mode;
};

struct imu_fifo_config
{
    bool acc_en;
    bool gyr_en;
    bool temp_en;
    bool time_en;
    bool stop_on_full;
    uint16_t watermark;
};

struct imu_int_config
{
    bool active_high;
    bool open_drain;
    bool latch;
};

struct imu_axis_map
{
    uint8_t axis_map;
    bool invert_x;
    bool invert_y;
    bool invert_z;
};

struct imu_any_motion_config
{
    uint16_t slope_thres;    // 斜率阈值 0~4095
    uint16_t duration;       // 持续时间（单位：20ms） 0~8191
    uint16_t hysteresis;     // 滞回值 0~1023
    uint16_t wait_time;      // 等待时间（单位：20ms） 0~7
    uint8_t acc_ref_up;      // 参考更新模式（0=OnEvent, 1=Always）
};

typedef void (*imu_int_callback_t)(void);

/********************************************************************
**函数名称:  imu_init
**入口参数:  config   ---        初始化配置参数（输入）
**出口参数:  无
**函数功能:  初始化 BMI325 IMU 模块并配置加速度计与陀螺仪
**返回值:    IMU_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
imu_result_t imu_init(const struct imu_config *config);

/********************************************************************
**函数名称:  imu_get_chip_id
**入口参数:  id       ---        芯片 ID 存储指针（输出）
**出口参数:  id       ---        读取到的芯片 ID
**函数功能:  获取 BMI325 芯片 ID
**返回值:    IMU_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
imu_result_t imu_get_chip_id(uint8_t *id);

/********************************************************************
**函数名称:  imu_read_raw
**入口参数:  raw      ---        原始数据存储指针（输出）
**出口参数:  raw      ---        原始 6 轴数据和温度
**函数功能:  读取 BMI325 原始数据
**返回值:    IMU_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
imu_result_t imu_read_raw(struct imu_raw_data *raw);

/********************************************************************
**函数名称:  imu_read
**入口参数:  data     ---        换算数据存储指针（输出）
**出口参数:  data     ---        换算后的 6 轴数据和温度
**函数功能:  读取并换算 BMI325 数据
**返回值:    IMU_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
imu_result_t imu_read(struct imu_data *data);

/********************************************************************
**函数名称:  imu_set_config
**入口参数:  config   ---        运行期配置参数（输入）
**出口参数:  无
**函数功能:  运行期配置 BMI325 加速度计与陀螺仪
**返回值:    IMU_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
imu_result_t imu_set_config(const struct imu_config *config);

/********************************************************************
**函数名称:  imu_set_power_mode
**入口参数:  mode     ---        电源模式（输入）
**出口参数:  无
**函数功能:  设置 BMI325 电源模式
**返回值:    IMU_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
imu_result_t imu_set_power_mode(imu_power_mode_t mode);

/********************************************************************
**函数名称:  imu_read_status
**入口参数:  status   ---        状态寄存器存储指针（输出）
**出口参数:  status   ---        状态寄存器值
**函数功能:  读取 BMI325 状态寄存器
**返回值:    IMU_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
imu_result_t imu_read_status(uint16_t *status);

/********************************************************************
**函数名称:  imu_read_err_reg
**入口参数:  err_reg  ---        错误寄存器存储指针（输出）
**出口参数:  err_reg  ---        错误寄存器值
**函数功能:  读取 BMI325 错误寄存器
**返回值:    IMU_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
imu_result_t imu_read_err_reg(uint16_t *err_reg);

/********************************************************************
**函数名称:  imu_read_sensor_time
**入口参数:  sensor_time ---     传感器时间存储指针（输出）
**出口参数:  sensor_time ---     传感器时间
**函数功能:  读取 BMI325 传感器时间
**返回值:    IMU_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
imu_result_t imu_read_sensor_time(uint32_t *sensor_time);

/********************************************************************
**函数名称:  imu_int_pin_config
**入口参数:  pin      ---        中断引脚（输入）
**           config   ---        中断引脚配置（输入）
**出口参数:  无
**函数功能:  配置 BMI325 中断引脚
**返回值:    IMU_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
imu_result_t imu_int_pin_config(imu_int_pin_t pin, const struct imu_int_config *config);

/********************************************************************
**函数名称:  imu_int_map
**入口参数:  src      ---        中断源（输入）
**           pin      ---        目标中断引脚（输入）
**出口参数:  无
**函数功能:  映射 BMI325 中断源到指定中断引脚
**返回值:    IMU_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
imu_result_t imu_int_map(imu_int_src_t src, imu_int_pin_t pin);

/********************************************************************
**函数名称:  imu_register_int_callback
**入口参数:  callback ---        中断回调函数（输入）
**出口参数:  无
**函数功能:  注册 BMI325 INT GPIO 中断回调
**返回值:    IMU_SUCCESS 表示成功，其他表示错误码
**注意事项:  当前硬件仅将 INT1 接入 MCU GPIO，INT2 仅做寄存器映射预留
*********************************************************************/
imu_result_t imu_register_int_callback(imu_int_callback_t callback);

/********************************************************************
**函数名称:  imu_read_int_status
**入口参数:  pin      ---        中断引脚（输入）
**           status   ---        中断状态存储指针（输出）
**出口参数:  status   ---        中断状态
**函数功能:  读取 BMI325 中断状态
**返回值:    IMU_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
imu_result_t imu_read_int_status(imu_int_pin_t pin, uint16_t *status);

/********************************************************************
**函数名称:  imu_fifo_config
**入口参数:  config   ---        FIFO 配置参数（输入）
**出口参数:  无
**函数功能:  配置 BMI325 FIFO
**返回值:    IMU_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
imu_result_t imu_fifo_config(const struct imu_fifo_config *config);

/********************************************************************
**函数名称:  imu_fifo_read
**入口参数:  frames      ---     FIFO 帧存储指针（输出）
**           max_frames  ---     最大读取帧数（输入）
**出口参数:  frames      ---     FIFO 帧数据
**           frame_count ---     实际读取帧数
**函数功能:  读取 BMI325 FIFO 数据
**返回值:    IMU_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
imu_result_t imu_fifo_read(struct imu_raw_data *frames, uint16_t max_frames, uint16_t *frame_count);

/********************************************************************
**函数名称:  imu_fifo_flush
**入口参数:  无
**出口参数:  无
**函数功能:  清空 BMI325 FIFO
**返回值:    IMU_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
imu_result_t imu_fifo_flush(void);

/********************************************************************
**函数名称:  imu_feature_enable
**入口参数:  feature  ---        特征类型（输入）
**           enable   ---        true 使能，false 禁用（输入）
**出口参数:  无
**函数功能:  使能或禁用 BMI325 feature engine 特征
**返回值:    IMU_SUCCESS 表示成功，其他表示错误码
**注意事项:  调用本接口会触发 feature engine 重载，可能清零当前累计步数
*********************************************************************/
imu_result_t imu_feature_enable(imu_feature_t feature, bool enable);

/********************************************************************
**函数名称:  imu_step_counter_enable
**入口参数:  enable   ---        true 使能，false 禁用（输入）
**出口参数:  无
**函数功能:  显式使能或禁用 BMI325 计步器
**返回值:    IMU_SUCCESS 表示成功，其他表示错误码
**注意事项:  使能操作会加载计步器配置并触发 feature engine 重载
*********************************************************************/
imu_result_t imu_step_counter_enable(bool enable);

/********************************************************************
**函数名称:  imu_get_step_count
**入口参数:  count    ---        步数存储指针（输出）
**出口参数:  count    ---        当前步数
**函数功能:  读取 BMI325 计步值
**返回值:    IMU_SUCCESS 表示成功，其他表示错误码
**注意事项:  调用前需先通过 imu_step_counter_enable(true) 使能计步器
*********************************************************************/
imu_result_t imu_get_step_count(uint32_t *count);

/********************************************************************
**函数名称:  imu_set_axis_map
**入口参数:  axis_map ---        轴映射配置（输入）
**出口参数:  无
**函数功能:  设置 BMI325 轴映射
**返回值:    IMU_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
imu_result_t imu_set_axis_map(const struct imu_axis_map *axis_map);

/********************************************************************
**函数名称:  imu_get_axis_map
**入口参数:  axis_map ---        轴映射配置存储指针（输出）
**出口参数:  axis_map ---        当前轴映射配置
**函数功能:  读取 BMI325 轴映射
**返回值:    IMU_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
imu_result_t imu_get_axis_map(struct imu_axis_map *axis_map);

/********************************************************************
**函数名称:  imu_set_any_motion_config
**入口参数:  config   ---        ANY_MOTION 配置参数（输入）
**出口参数:  无
**函数功能:  配置 BMI325 ANY_MOTION 检测参数
**返回值:    IMU_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
imu_result_t imu_set_any_motion_config(const struct imu_any_motion_config *config);

#endif /* _IMU_API_H_ */
