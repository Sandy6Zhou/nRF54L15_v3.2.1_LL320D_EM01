/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        battery_gauge_api.h
**文件描述:        OM70201WV 电量计统一接口头文件
**当前版本:        V1.0
**作    者:       周森达 (zhousenda@jimiiot.com)
**完成日期:        2026.07.21
*********************************************************************
** 功能描述:       定义电量计返回码、数据结构、工作模式和中断配置
**                 声明 OM70201WV 电量计模块对外公开接口
*********************************************************************/

#ifndef _BATTERY_GAUGE_API_H_
#define _BATTERY_GAUGE_API_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    BATTERY_GAUGE_SUCCESS = 0,
    BATTERY_GAUGE_ERROR_INIT,
    BATTERY_GAUGE_ERROR_COMM,
    BATTERY_GAUGE_ERROR_CHIP_ID,
    BATTERY_GAUGE_ERROR_PARAM,
    BATTERY_GAUGE_ERROR_TIMEOUT,
    BATTERY_GAUGE_ERROR_UNKNOWN,
} battery_gauge_result_t;

typedef enum
{
    BATTERY_GAUGE_INTERRUPT_NONE = 0,
    BATTERY_GAUGE_INTERRUPT_LOW_TEMPERATURE = (1U << 0),
    BATTERY_GAUGE_INTERRUPT_HIGH_TEMPERATURE = (1U << 1),
    BATTERY_GAUGE_INTERRUPT_SOC = (1U << 2),
} battery_gauge_interrupt_status_t;

typedef enum
{
    BATTERY_GAUGE_WORK_MODE_SLEEP = 0,
    BATTERY_GAUGE_WORK_MODE_NORMAL,
    BATTERY_GAUGE_WORK_MODE_MAX,
} battery_gauge_work_mode_t;

typedef struct
{
    uint16_t voltage_mv;       // 电池电压，单位 mV
    int16_t current_ma;        // 电池电流，单位 mA，符号方向以实机验证为准
    int8_t temperature_c;      // 电池温度，单位摄氏度
    uint8_t soc_percent;       // 电池剩余电量百分比，范围 0~100
    uint8_t soh_percent;       // 电池健康度百分比，范围 0~100
    uint16_t cycle_count;      // 电池累计充放电循环次数
} battery_gauge_data_t;

typedef struct
{
    bool soc_enable;                     // SOC 中断使能标志
    bool high_temperature_enable;        // 高温中断使能标志
    bool low_temperature_enable;         // 低温中断使能标志
    uint8_t soc_threshold_percent;       // SOC 中断阈值，范围 0~100
    int8_t high_temperature_c;           // 高温中断阈值，范围 -40~85 摄氏度
    int8_t low_temperature_c;            // 低温中断阈值，范围 -40~85 摄氏度
} battery_gauge_interrupt_config_t;

typedef void (*battery_gauge_interrupt_callback_t)(void);

/********************************************************************
**函数名称:  battery_gauge_init
**入口参数:  cycle_count_raw ---        需要恢复的原始循环计数，NULL 表示不恢复（输入）
**出口参数:  无
**函数功能:  初始化 OM70201WV 电量计并按需恢复循环计数
**返回值:    BATTERY_GAUGE_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
battery_gauge_result_t battery_gauge_init(const uint16_t *cycle_count_raw);

/********************************************************************
**函数名称:  battery_gauge_get_chip_id
**入口参数:  chip_id ---        芯片 ID 缓冲区（输入）
**出口参数:  chip_id ---        OM70201WV 芯片 ID（输出）
**函数功能:  读取并校验 OM70201WV 芯片 ID
**返回值:    BATTERY_GAUGE_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
battery_gauge_result_t battery_gauge_get_chip_id(uint8_t *chip_id);

/********************************************************************
**函数名称:  battery_gauge_read
**入口参数:  data ---        电量计数据缓冲区（输入）
**出口参数:  data ---        电压、电流、温度、SOC、SOH 和循环次数（输出）
**函数功能:  按厂家推荐顺序读取 OM70201WV 全部常用数据
**返回值:    BATTERY_GAUGE_SUCCESS 表示成功，其他表示错误码
**注意事项:  current_ma 保持芯片原始符号，实机确认方向后由 API 层统一约定
*********************************************************************/
battery_gauge_result_t battery_gauge_read(battery_gauge_data_t *data);

/********************************************************************
**函数名称:  battery_gauge_get_voltage
**入口参数:  voltage_mv ---        电压缓冲区（输入）
**出口参数:  voltage_mv ---        电池电压，单位 mV（输出）
**函数功能:  读取 OM70201WV 电池电压
**返回值:    BATTERY_GAUGE_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
battery_gauge_result_t battery_gauge_get_voltage(uint16_t *voltage_mv);

/********************************************************************
**函数名称:  battery_gauge_get_current
**入口参数:  current_ma ---        电流缓冲区（输入）
**出口参数:  current_ma ---        电池电流，单位 mA（输出）
**函数功能:  读取 OM70201WV 有符号电池电流
**返回值:    BATTERY_GAUGE_SUCCESS 表示成功，其他表示错误码
**注意事项:  当前硬件 CSP 接电池地、CSN 接系统地，采用低边电流采样
*********************************************************************/
battery_gauge_result_t battery_gauge_get_current(int16_t *current_ma);

/********************************************************************
**函数名称:  battery_gauge_get_temperature
**入口参数:  temperature_c ---        温度缓冲区（输入）
**出口参数:  temperature_c ---        电池温度，单位摄氏度（输出）
**函数功能:  读取 OM70201WV 电池温度
**返回值:    BATTERY_GAUGE_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
battery_gauge_result_t battery_gauge_get_temperature(int8_t *temperature_c);

/********************************************************************
**函数名称:  battery_gauge_get_soc
**入口参数:  soc_percent ---        SOC 缓冲区（输入）
**出口参数:  soc_percent ---        电池剩余电量百分比（输出）
**函数功能:  读取 OM70201WV SOC
**返回值:    BATTERY_GAUGE_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
battery_gauge_result_t battery_gauge_get_soc(uint8_t *soc_percent);

/********************************************************************
**函数名称:  battery_gauge_get_soh
**入口参数:  soh_percent ---        SOH 缓冲区（输入）
**出口参数:  soh_percent ---        电池健康度百分比（输出）
**函数功能:  读取 OM70201WV SOH
**返回值:    BATTERY_GAUGE_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
battery_gauge_result_t battery_gauge_get_soh(uint8_t *soh_percent);

/********************************************************************
**函数名称:  battery_gauge_get_cycle_count
**入口参数:  cycle_count ---        循环次数缓冲区（输入）
**出口参数:  cycle_count ---        电池循环次数（输出）
**函数功能:  读取 OM70201WV 电池循环次数
**返回值:    BATTERY_GAUGE_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
battery_gauge_result_t battery_gauge_get_cycle_count(uint16_t *cycle_count);

/********************************************************************
**函数名称:  battery_gauge_get_cycle_count_raw
**入口参数:  cycle_count_raw ---        原始循环计数缓冲区（输入）
**出口参数:  cycle_count_raw ---        以 1/32 次为单位的原始循环计数（输出）
**函数功能:  读取 OM70201WV 循环次数原始寄存器值
**返回值:    BATTERY_GAUGE_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
battery_gauge_result_t battery_gauge_get_cycle_count_raw(uint16_t *cycle_count_raw);

/********************************************************************
**函数名称:  battery_gauge_set_cycle_count
**入口参数:  cycle_count ---        需要设置的电池循环次数（输入）
**出口参数:  无
**函数功能:  设置 OM70201WV 电池循环次数并更新 SOH
**返回值:    BATTERY_GAUGE_SUCCESS 表示成功，其他表示错误码
**注意事项:  仅用于生产初始化、数据恢复和测试验证
*********************************************************************/
battery_gauge_result_t battery_gauge_set_cycle_count(uint16_t cycle_count);

/********************************************************************
**函数名称:  battery_gauge_set_work_mode
**入口参数:  work_mode ---        电量计工作模式（输入）
**出口参数:  无
**函数功能:  设置 OM70201WV 进入睡眠模式或正常工作模式
**返回值:    BATTERY_GAUGE_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
battery_gauge_result_t battery_gauge_set_work_mode(
    battery_gauge_work_mode_t work_mode);

/********************************************************************
**函数名称:  battery_gauge_interrupt_config
**入口参数:  config ---        中断使能和阈值配置（输入）
**出口参数:  无
**函数功能:  配置 OM70201WV SOC、高温和低温中断
**返回值:    BATTERY_GAUGE_SUCCESS 表示成功，其他表示错误码
**注意事项:  SOC 阈值为 100 时，SOC 每变化 1% 产生一次中断
*********************************************************************/
battery_gauge_result_t battery_gauge_interrupt_config(
    const battery_gauge_interrupt_config_t *config);

/********************************************************************
**函数名称:  battery_gauge_interrupt_get_status
**入口参数:  status ---        中断状态缓冲区（输入）
**出口参数:  status ---        中断状态位组合（输出）
**函数功能:  读取 OM70201WV SOC、高温和低温中断状态
**返回值:    BATTERY_GAUGE_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
battery_gauge_result_t battery_gauge_interrupt_get_status(
    battery_gauge_interrupt_status_t *status);

/********************************************************************
**函数名称:  battery_gauge_register_interrupt_callback
**入口参数:  callback ---        中断回调函数（输入）
**出口参数:  无
**函数功能:  注册 OM70201WV INTN GPIO 中断回调
**返回值:    BATTERY_GAUGE_SUCCESS 表示成功，其他表示错误码
**注意事项:  回调在 GPIO 中断上下文执行，禁止直接调用电量计读写接口
*********************************************************************/
battery_gauge_result_t battery_gauge_register_interrupt_callback(
    battery_gauge_interrupt_callback_t callback);

#endif
