/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        spa06_driver.h
**文件描述:        SPA06-003 气压传感器驱动头文件
**当前版本:        V1.0
**作    者:       周森达 (zhousenda@jimiiot.com)
**完成日期:        2026.06.08
*********************************************************************
** 功能描述:       SPA06-003 气压传感器驱动头文件，定义寄存器地址、芯片 ID、
**                工作模式枚举与校准系数结构体，并对外声明驱动初始化、工作
**                模式设置、测量数据读取及芯片 ID 读取等接口
*********************************************************************/

#ifndef _SPA06_DRIVER_H_
#define _SPA06_DRIVER_H_

#include <stdint.h>
#include <stdbool.h>

/* SPA06-003 I2C 设备地址（7-bit） */
#define SPA06_I2C_ADDR              0x77

/* 寄存器地址定义 */
#define SPA06_REG_PSR_B2            0x00    /* 气压数据 [23:16] */
#define SPA06_REG_PSR_B1            0x01    /* 气压数据 [15:8] */
#define SPA06_REG_PSR_B0            0x02    /* 气压数据 [7:0] */
#define SPA06_REG_TMP_B2            0x03    /* 温度数据 [23:16] */
#define SPA06_REG_TMP_B1            0x04    /* 温度数据 [15:8] */
#define SPA06_REG_TMP_B0            0x05    /* 温度数据 [7:0] */
#define SPA06_REG_PRS_CFG           0x06    /* 气压配置（采样率 + 过采样） */
#define SPA06_REG_TMP_CFG           0x07    /* 温度配置（采样率 + 过采样） */
#define SPA06_REG_MEAS_CFG          0x08    /* 测量模式 + 就绪标志 */
#define SPA06_REG_CFG_REG           0x09    /* 中断/FIFO/位移配置 */
#define SPA06_REG_INT_STS           0x0A    /* 中断状态 */
#define SPA06_REG_FIFO_STS          0x0B    /* FIFO 状态 */
#define SPA06_REG_RESET             0x0C    /* 软复位 */
#define SPA06_REG_CHIP_ID           0x0D    /* 芯片 ID */
#define SPA06_REG_COEF_START        0x10    /* 校准系数起始地址 */

/* 芯片 ID 期望值 */
#define SPA06_CHIP_ID_VALUE         0x11

/* 软复位命令 */
#define SPA06_RESET_CMD             0x09

/* 测量模式命令（写入 MEAS_CFG） */
#define SPA06_MEAS_STANDBY          0x00    /* 待机模式 */
#define SPA06_MEAS_PRS_SINGLE       0x01    /* 单次气压测量 */
#define SPA06_MEAS_TMP_SINGLE       0x02    /* 单次温度测量 */
#define SPA06_MEAS_PRS_CONTINUOUS   0x05    /* 连续气压测量 */
#define SPA06_MEAS_TMP_CONTINUOUS   0x06    /* 连续温度测量 */
#define SPA06_MEAS_BOTH_CONTINUOUS  0x07    /* 连续气压和温度测量 */

/* MEAS_CFG 就绪标志位 */
#define SPA06_MEAS_PRS_RDY          0x10    /* 气压数据就绪 */
#define SPA06_MEAS_TMP_RDY          0x20    /* 温度数据就绪 */
#define SPA06_MEAS_SENSOR_RDY       0x40    /* 传感器初始化完成 */
#define SPA06_MEAS_COEF_RDY         0x80    /* 校准系数可读 */

/* CFG_REG 位移标志（过采样 > 8x 时需设置） */
#define SPA06_CFG_PRS_SHIFT         0x04
#define SPA06_CFG_TMP_SHIFT         0x08

/* 校准系数结构体 */
struct spa06_coef
{
    int16_t c0;             /* 12-bit，符号扩展 */
    int16_t c1;             /* 12-bit，符号扩展 */
    int32_t c00;            /* 20-bit，符号扩展 */
    int32_t c10;            /* 20-bit，符号扩展 */
    int16_t c01;            /* 16-bit */
    int16_t c11;            /* 16-bit */
    int16_t c20;            /* 16-bit */
    int16_t c21;            /* 16-bit */
    int16_t c30;            /* 16-bit */
    int16_t c31;            /* 12-bit，符号扩展（SPA06-003 特有） */
    int16_t c40;            /* 12-bit，符号扩展（SPA06-003 特有） */
};

/* SPA06 工作模式 */
typedef enum
{
    SPA06_MODE_STOP = 0,
    SPA06_MODE_SINGLE_TEMPERATURE,
    SPA06_MODE_SINGLE_PRESSURE,
    SPA06_MODE_SINGLE_BOTH,
    SPA06_MODE_CONTINUOUS_TEMPERATURE,
    SPA06_MODE_CONTINUOUS_PRESSURE,
    SPA06_MODE_CONTINUOUS_BOTH,
} spa06_work_mode_t;

/* SPA06 采样配置结构体（初始化时一次性设定，运行期不再变更） */
typedef struct
{
    uint8_t pressure_sample_rate_hz;        /* 气压采样率，单位 Hz */
    uint8_t pressure_oversampling;          /* 气压过采样倍率 */
    uint8_t temperature_sample_rate_hz;     /* 温度采样率，单位 Hz */
    uint8_t temperature_oversampling;       /* 温度过采样倍率 */
} spa06_config_t;

/********************************************************************
**函数名称:  spa06_driver_init
**入口参数:  config   ---        采样配置参数（输入：采样率与过采样）
**出口参数:  无
**函数功能:  初始化 SPA06-003 驱动，包括 I2C 总线、校准系数读取，并按
**           传入配置设定气压/温度采样率和过采样
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
int spa06_driver_init(const spa06_config_t *config);

/********************************************************************
**函数名称:  spa06_driver_set_work_mode
**入口参数:  work_mode      ---        SPA06 工作模式（输入）
**出口参数:  无
**函数功能:  设置 SPA06 工作模式并更新测量控制寄存器
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
int spa06_driver_set_work_mode(spa06_work_mode_t work_mode);

/********************************************************************
**函数名称:  spa06_driver_read_measurement
**入口参数:  pressure_pa   ---        气压存储指针（输出）
**           temperature   ---        温度存储指针（输出）
**出口参数:  pressure_pa   ---        气压值，单位 Pa
**           temperature   ---        温度值，单位 0.01°C
**函数功能:  根据当前工作模式读取一次测量数据
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
int spa06_driver_read_measurement(int32_t *pressure_pa, int16_t *temperature);

/********************************************************************
**函数名称:  spa06_driver_read_chip_id
**入口参数:  id       ---        ID 存储指针（输出）
**出口参数:  id       ---        芯片 ID 值
**函数功能:  读取 SPA06-003 芯片 ID
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
int spa06_driver_read_chip_id(uint8_t *id);

#endif /* _SPA06_DRIVER_H_ */
