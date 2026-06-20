/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        barometer_api.h
**文件描述:        气压模块统一 API 接口头文件
**当前版本:        V1.0
**作    者:       周森达 (zhousenda@jimiiot.com)
**完成日期:        2026.06.08
*********************************************************************
** 功能描述:       提供统一的气压功能接口，屏蔽底层驱动差异
**                支持 SPA06-003 系列气压传感器
*********************************************************************/

#ifndef _BAROMETER_API_H_
#define _BAROMETER_API_H_

#include <stdint.h>
#include <stdbool.h>

/* 气压操作结果 */
typedef enum
{
    BARO_SUCCESS = 0,
    BARO_ERROR_INIT,           /* 初始化错误 */
    BARO_ERROR_COMM,           /* 通信错误 */
    BARO_ERROR_CHIP_ID,        /* 芯片 ID 不匹配 */
    BARO_ERROR_PARAM,          /* 参数错误 */
    BARO_ERROR_TIMEOUT,        /* 测量超时 */
} barometer_result_t;

/* 气压工作模式 */
typedef enum
{
    BARO_MODE_STOP = 0,
    BARO_MODE_SINGLE_TEMPERATURE,
    BARO_MODE_SINGLE_PRESSURE,
    BARO_MODE_SINGLE_BOTH,
    BARO_MODE_CONTINUOUS_TEMPERATURE,
    BARO_MODE_CONTINUOUS_PRESSURE,
    BARO_MODE_CONTINUOUS_BOTH,
    BARO_MODE_MAX,                  /* 枚举边界，仅用于入参有效性校验，不可作为实际工作模式 */
} barometer_work_mode_t;

/* 气压数据结构体 */
struct barometer_data
{
    int32_t pressure_pa;       /* 气压值，单位 Pa，如 101325 表示标准大气压 */
    int16_t temperature;       /* 温度值，单位 0.01°C，如 2550 表示 25.50°C */
};

/* 气压模块采样配置结构体（初始化时一次性设定，运行期不再变更） */
struct barometer_config
{
    uint8_t pressure_sample_rate_hz;
    uint8_t pressure_oversampling;
    uint8_t temperature_sample_rate_hz;
    uint8_t temperature_oversampling;
};

/********************************************************************
**函数名称:  barometer_init
**入口参数:  config   ---        采样配置参数（输入：采样率与过采样）
**出口参数:  无
**函数功能:  初始化气压模块，包括底层驱动、校准系数读取，并按传入配置
**           设定气压/温度采样率和过采样
**返 回 值:  BARO_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
barometer_result_t barometer_init(const struct barometer_config *config);

/********************************************************************
**函数名称:  barometer_set_work_mode
**入口参数:  work_mode   ---        气压模块工作模式（输入）
**出口参数:  无
**函数功能:  设置气压模块工作模式
**返 回 值:  BARO_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
barometer_result_t barometer_set_work_mode(barometer_work_mode_t work_mode);

/********************************************************************
**函数名称:  barometer_read
**入口参数:  data     ---        气压数据存储指针（输出）
**出口参数:  data     ---        包含气压和温度的结构体
**函数功能:  执行一次气压和温度测量
**返 回 值:  BARO_SUCCESS 表示成功，其他表示错误码
**注意事项:  根据对应模式只填充实际测量的字段
*********************************************************************/
barometer_result_t barometer_read(struct barometer_data *data);

/********************************************************************
**函数名称:  barometer_get_chip_id
**入口参数:  id       ---        ID 存储指针（输出）
**出口参数:  id       ---        芯片 ID 值
**函数功能:  读取气压传感器芯片 ID
**返 回 值:  BARO_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
barometer_result_t barometer_get_chip_id(uint8_t *id);

#endif /* _BAROMETER_API_H_ */
