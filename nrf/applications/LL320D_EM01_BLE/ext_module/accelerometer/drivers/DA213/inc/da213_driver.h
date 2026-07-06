/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        da213_driver.h
**文件描述:        DA213 三轴加速度传感器驱动头文件
**当前版本:        V1.0
**作    者:       周森达 (zhousenda@jimiiot.com)
**完成日期:        2026.07.06
*********************************************************************
** 功能描述:       DA213 三轴加速度传感器驱动头文件，定义寄存器地址、芯片ID、
**                工作模式枚举与配置结构体，并对外声明驱动初始化、工作模式设置、
**                加速度数据读取、中断配置及芯片ID读取等接口
*********************************************************************/

#ifndef _DA213_DRIVER_H_
#define _DA213_DRIVER_H_

#include <stdint.h>
#include <stdbool.h>

/* DA213 GPIO 中断回调函数类型 */
typedef void (*da213_int_callback_t)(void);

/* DA213 I2C 设备地址（7-bit），SA0=1 时为 0x27，SA0=0 时为 0x26 */
#define DA213_I2C_ADDR                  0x27

/* 芯片 ID 期望值 */
#define DA213_CHIP_ID_VALUE             0x13

/* 寄存器地址定义 */
#define DA213_REG_SOFT_RESET            0x00
#define DA213_REG_CHIP_ID               0x01
#define DA213_REG_ACC_X_LSB             0x02
#define DA213_REG_ACC_X_MSB             0x03
#define DA213_REG_ACC_Y_LSB             0x04
#define DA213_REG_ACC_Y_MSB             0x05
#define DA213_REG_ACC_Z_LSB             0x06
#define DA213_REG_ACC_Z_MSB             0x07
#define DA213_REG_MOTION_FLAG           0x09
#define DA213_REG_NEWDATA_FLAG          0x0A
#define DA213_REG_TAP_ACTIVE_STATUS     0x0B
#define DA213_REG_ORIENT_STATUS         0x0C
#define DA213_REG_RESOLUTION_RANGE      0x0F
#define DA213_REG_ODR_AXIS              0x10
#define DA213_REG_MODE_BW               0x11
#define DA213_REG_SWAP_POLARITY         0x12
#define DA213_REG_INT_SET1              0x16
#define DA213_REG_INT_SET2              0x17
#define DA213_REG_INT_MAP1              0x19
#define DA213_REG_INT_MAP2              0x1A
#define DA213_REG_INT_MAP3              0x1B
#define DA213_REG_INT_CONFIG            0x20
#define DA213_REG_INT_LATCH             0x21
#define DA213_REG_FREEFALL_DUR          0x22
#define DA213_REG_FREEFALL_THS          0x23
#define DA213_REG_FREEFALL_HYST         0x24
#define DA213_REG_ACTIVE_DUR            0x27
#define DA213_REG_ACTIVE_THS            0x28
#define DA213_REG_TAP_DUR               0x2A
#define DA213_REG_TAP_THS               0x2B
#define DA213_REG_ORIENT_HYST           0x2C
#define DA213_REG_Z_BLOCK               0x2D
#define DA213_REG_SELF_TEST             0x32
#define DA213_REG_CUSTOM_OFFSET_X       0x38
#define DA213_REG_CUSTOM_OFFSET_Y       0x39
#define DA213_REG_CUSTOM_OFFSET_Z       0x3A
#define DA213_REG_CUSTOM_FLAG           0x4E
#define DA213_REG_CUSTOM_CODE           0x4F
#define DA213_REG_Z_CAL_EN              0x50
#define DA213_REG_Z_ROT_HOLD_TM         0x51
#define DA213_REG_Z_ROT_DUR             0x52
#define DA213_REG_ROT_TH_H              0x53
#define DA213_REG_ROT_TH_L              0x54

/* 运动标志位定义 (MOTION_FLAG 0x09) */
#define DA213_MOTION_ORIENT_INT         (1 << 6)
#define DA213_MOTION_S_TAP_INT          (1 << 5)
#define DA213_MOTION_D_TAP_INT          (1 << 4)
#define DA213_MOTION_ACTIVE_INT         (1 << 2)
#define DA213_MOTION_FREEFALL_INT       (1 << 0)

/* TAP_ACTIVE_STATUS 位定义 (0x0B) */
#define DA213_TAP_SIGN_NEGATIVE         (1 << 7)
#define DA213_TAP_FIRST_X               (1 << 6)
#define DA213_TAP_FIRST_Y               (1 << 5)
#define DA213_TAP_FIRST_Z               (1 << 4)
#define DA213_ACTIVE_SIGN_NEGATIVE      (1 << 3)
#define DA213_ACTIVE_FIRST_X            (1 << 2)
#define DA213_ACTIVE_FIRST_Y            (1 << 1)
#define DA213_ACTIVE_FIRST_Z            (1 << 0)

/* 量程枚举 */
typedef enum
{
    DA213_RANGE_2G = 0x00,
    DA213_RANGE_4G = 0x01,
    DA213_RANGE_8G = 0x02,
    DA213_RANGE_16G = 0x03,
} da213_range_t;

/* 分辨率枚举 */
typedef enum
{
    DA213_RESOLUTION_14BIT = 0x00,
    DA213_RESOLUTION_12BIT = 0x01,
    DA213_RESOLUTION_10BIT = 0x02,
    DA213_RESOLUTION_8BIT = 0x03,
} da213_resolution_t;

/* 输出数据率 ODR 枚举 */
typedef enum
{
    DA213_ODR_1HZ = 0x00,
    DA213_ODR_1_95HZ = 0x01,
    DA213_ODR_3_9HZ = 0x02,
    DA213_ODR_7_81HZ = 0x03,
    DA213_ODR_15_63HZ = 0x04,
    DA213_ODR_31_25HZ = 0x05,
    DA213_ODR_62_5HZ = 0x06,
    DA213_ODR_125HZ = 0x07,
    DA213_ODR_250HZ = 0x08,
    DA213_ODR_500HZ = 0x09,
    DA213_ODR_1000HZ = 0x0A,
} da213_odr_t;

/* 电源模式枚举 */
typedef enum
{
    DA213_MODE_NORMAL = 0x00,
    DA213_MODE_LOW_POWER = 0x01,
    DA213_MODE_SUSPEND = 0x02,
} da213_power_mode_t;

/* 低功耗带宽枚举 */
typedef enum
{
    DA213_LP_BW_1_95HZ = 0x00,
    DA213_LP_BW_3_9HZ = 0x03,
    DA213_LP_BW_7_81HZ = 0x04,
    DA213_LP_BW_15_63HZ = 0x05,
    DA213_LP_BW_31_25HZ = 0x06,
    DA213_LP_BW_62_5HZ = 0x07,
    DA213_LP_BW_125HZ = 0x08,
    DA213_LP_BW_250HZ = 0x09,
    DA213_LP_BW_500HZ = 0x0A,
} da213_lp_bw_t;

/* 中断锁存模式枚举 */
typedef enum
{
    DA213_LATCH_NON_LATCHED = 0x00,
    DA213_LATCH_250MS = 0x01,
    DA213_LATCH_500MS = 0x02,
    DA213_LATCH_1S = 0x03,
    DA213_LATCH_2S = 0x04,
    DA213_LATCH_4S = 0x05,
    DA213_LATCH_8S = 0x06,
    DA213_LATCH_LATCHED = 0x07,
    DA213_LATCH_NON_LATCHED2 = 0x08,
    DA213_LATCH_1MS = 0x09,
    DA213_LATCH_1MS2 = 0x0A,
    DA213_LATCH_2MS = 0x0B,
    DA213_LATCH_25MS = 0x0C,
    DA213_LATCH_50MS = 0x0D,
    DA213_LATCH_100MS = 0x0E,
    DA213_LATCH_LATCHED2 = 0x0F,
} da213_int_latch_t;

/* 中断引脚输出类型枚举 */
typedef enum
{
    DA213_INT_PUSH_PULL = 0x00,
    DA213_INT_OPEN_DRAIN = 0x01,
} da213_int_output_t;

/* 中断引脚有效电平枚举 */
typedef enum
{
    DA213_INT_ACTIVE_LOW = 0x00,
    DA213_INT_ACTIVE_HIGH = 0x01,
} da213_int_level_t;

/* 方向识别模式枚举 */
typedef enum
{
    DA213_ORIENT_SYMMETRICAL = 0x00,
    DA213_ORIENT_HIGH_ASYM = 0x01,
    DA213_ORIENT_LOW_ASYM = 0x02,
} da213_orient_mode_t;

/* 方向识别阻塞条件枚举 */
typedef enum
{
    DA213_ORIENT_BLOCK_NONE = 0x00,
    DA213_ORIENT_BLOCK_Z = 0x01,
    DA213_ORIENT_BLOCK_Z_SLOPE = 0x02,
    DA213_ORIENT_BLOCK_NONE2 = 0x03,
} da213_orient_block_t;

/* 自由落体检测模式枚举 */
typedef enum
{
    DA213_FREEFALL_SINGLE = 0x00,
    DA213_FREEFALL_SUM = 0x01,
} da213_freefall_mode_t;

/* 中断引脚选择枚举 */
typedef enum
{
    DA213_INT_PIN_1 = 0x00,
    DA213_INT_PIN_2 = 0x01,
} da213_int_pin_t;

/* 基本配置结构体 */
typedef struct
{
    da213_range_t range;
    da213_resolution_t resolution;
    da213_odr_t odr;
    da213_power_mode_t power_mode;
    da213_lp_bw_t lp_bandwidth;
} da213_config_t;

/* Active（运动检测）中断配置 */
typedef struct
{
    uint8_t threshold;
    uint8_t duration;
    bool enable_x;
    bool enable_y;
    bool enable_z;
    da213_int_pin_t int_pin;
} da213_active_int_config_t;

/* Tap（敲击检测）中断配置 */
typedef struct
{
    uint8_t threshold;
    uint8_t quiet;
    uint8_t shock;
    uint8_t duration;
    bool enable_single;
    bool enable_double;
    da213_int_pin_t int_pin;
} da213_tap_int_config_t;

/* Freefall（自由落体）中断配置 */
typedef struct
{
    uint8_t threshold;
    uint8_t duration;
    uint8_t hysteresis;
    da213_freefall_mode_t mode;
    da213_int_pin_t int_pin;
} da213_freefall_int_config_t;

/* Orient（方向识别）中断配置 */
typedef struct
{
    da213_orient_mode_t mode;
    da213_orient_block_t blocking;
    uint8_t hysteresis;
    uint8_t z_blocking;
    da213_int_pin_t int_pin;
} da213_orient_int_config_t;

/* 中断引脚配置 */
typedef struct
{
    da213_int_output_t output_type;
    da213_int_level_t active_level;
} da213_int_pin_config_t;

/* 三轴加速度原始数据 */
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} da213_raw_data_t;

/* 方向状态 */
typedef struct
{
    uint8_t orient_xy;
    uint8_t orient_z;
} da213_orient_status_t;

/* Tap/Active 状态 */
typedef struct
{
    bool tap_sign_negative;
    bool tap_first_x;
    bool tap_first_y;
    bool tap_first_z;
    bool active_sign_negative;
    bool active_first_x;
    bool active_first_y;
    bool active_first_z;
} da213_tap_active_status_t;

/* 驱动接口声明 */
int da213_driver_init(const da213_config_t *config);
int da213_driver_read_chip_id(uint8_t *id);
int da213_driver_set_power_mode(da213_power_mode_t mode);
int da213_driver_set_range(da213_range_t range);
int da213_driver_set_resolution(da213_resolution_t resolution);
int da213_driver_set_odr(da213_odr_t odr);
int da213_driver_set_lp_bandwidth(da213_lp_bw_t bandwidth);
int da213_driver_read_raw_data(da213_raw_data_t *data);
int da213_driver_set_axis_enable(bool enable_x, bool enable_y, bool enable_z);
int da213_driver_set_axis_polarity(bool invert_x, bool invert_y, bool invert_z);
int da213_driver_set_xy_swap(bool swap);
int da213_driver_set_int_pin_config(da213_int_pin_t pin, const da213_int_pin_config_t *config);
int da213_driver_set_int_latch(da213_int_latch_t latch_mode);
int da213_driver_register_int_callback(da213_int_callback_t callback);
int da213_driver_reset_int(void);
int da213_driver_enable_newdata_int(da213_int_pin_t pin, bool enable);
int da213_driver_config_active_int(const da213_active_int_config_t *config);
int da213_driver_disable_active_int(void);
int da213_driver_config_tap_int(const da213_tap_int_config_t *config);
int da213_driver_disable_tap_int(void);
int da213_driver_config_freefall_int(const da213_freefall_int_config_t *config);
int da213_driver_disable_freefall_int(void);
int da213_driver_config_orient_int(const da213_orient_int_config_t *config);
int da213_driver_disable_orient_int(void);
int da213_driver_read_motion_flag(uint8_t *flag);
int da213_driver_read_newdata_flag(bool *new_data);
int da213_driver_read_tap_active_status(da213_tap_active_status_t *status);
int da213_driver_read_orient_status(da213_orient_status_t *status);
int da213_driver_set_offset(int8_t offset_x, int8_t offset_y, int8_t offset_z);
int da213_driver_get_offset(int8_t *offset_x, int8_t *offset_y, int8_t *offset_z);

#endif /* _DA213_DRIVER_H_ */
