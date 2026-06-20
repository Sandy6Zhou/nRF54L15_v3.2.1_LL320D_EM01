/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        temp_humi_api.c
**文件描述:        温湿度模块统一 API 接口实现文件
**当前版本:        V1.0
**作    者:       周森达 (zhousenda@jimiiot.com)
**完成日期:        2026.06.03
*********************************************************************
** 功能描述:       封装 SHTC3 驱动，提供统一温湿度接口
*********************************************************************/

#include "temp_humi_api.h"
#include "../drivers/SHTC3/inc/shtc3_driver.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(temp_humi_api, LOG_LEVEL_INF);

/* 内部状态 */
static bool s_temp_humi_initialized = false;

/********************************************************************
**函数名称:  temp_humi_api_init
**入口参数:  无
**出口参数:  无
**函数功能:  初始化温湿度模块，包括底层驱动和电源使能
**返 回 值:  TEMP_HUMI_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
temp_humi_result_t temp_humi_api_init(void)
{
    int ret;

    if (s_temp_humi_initialized)
    {
        LOG_WRN("Temp/Humi already initialized");
        return TEMP_HUMI_SUCCESS;
    }

    ret = shtc3_driver_init();
    if (ret != 0)
    {
        LOG_ERR("SHTC3 driver init failed: %d", ret);
        return TEMP_HUMI_ERROR_INIT;
    }

    s_temp_humi_initialized = true;
    LOG_INF("Temp/Humi API initialized");
    return TEMP_HUMI_SUCCESS;
}

/********************************************************************
**函数名称:  temp_humi_api_read
**入口参数:  data     ---        温湿度数据存储指针（输出）
**出口参数:  data     ---        包含温度和湿度的结构体
**函数功能:  执行一次温湿度测量（唤醒→测量→休眠）
**返 回 值:  TEMP_HUMI_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
temp_humi_result_t temp_humi_api_read(temp_humi_data_t *data)
{
    int ret;

    if (!s_temp_humi_initialized)
    {
        LOG_ERR("Temp/Humi not initialized");
        return TEMP_HUMI_ERROR_INIT;
    }

    if (data == NULL)
    {
        return TEMP_HUMI_ERROR_PARAM;
    }

    ret = shtc3_driver_read_measurement(&data->temperature, &data->humidity);
    if (ret == -EIO)
    {
        return TEMP_HUMI_ERROR_CRC;
    }
    else if (ret < 0)
    {
        return TEMP_HUMI_ERROR_COMM;
    }

    return TEMP_HUMI_SUCCESS;
}

/********************************************************************
**函数名称:  temp_humi_api_get_id
**入口参数:  id       ---        ID 存储指针（输出）
**出口参数:  id       ---        传感器设备 ID
**函数功能:  读取温湿度传感器设备 ID
**返 回 值:  TEMP_HUMI_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
temp_humi_result_t temp_humi_api_get_id(uint16_t *id)
{
    int ret;

    if (!s_temp_humi_initialized)
    {
        return TEMP_HUMI_ERROR_INIT;
    }

    if (id == NULL)
    {
        return TEMP_HUMI_ERROR_PARAM;
    }

    ret = shtc3_driver_read_id(id);
    if (ret == -EIO)
    {
        return TEMP_HUMI_ERROR_CRC;
    }
    else if (ret < 0)
    {
        return TEMP_HUMI_ERROR_COMM;
    }

    return TEMP_HUMI_SUCCESS;
}

/********************************************************************
**函数名称:  temp_humi_api_sleep
**入口参数:  无
**出口参数:  无
**函数功能:  使传感器进入低功耗休眠模式
**返 回 值:  TEMP_HUMI_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
temp_humi_result_t temp_humi_api_sleep(void)
{
    int ret;

    if (!s_temp_humi_initialized)
    {
        return TEMP_HUMI_ERROR_INIT;
    }

    ret = shtc3_driver_sleep();
    if (ret < 0)
    {
        return TEMP_HUMI_ERROR_COMM;
    }

    return TEMP_HUMI_SUCCESS;
}

/********************************************************************
**函数名称:  temp_humi_api_wakeup
**入口参数:  无
**出口参数:  无
**函数功能:  唤醒传感器
**返 回 值:  TEMP_HUMI_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
temp_humi_result_t temp_humi_api_wakeup(void)
{
    int ret;

    if (!s_temp_humi_initialized)
    {
        return TEMP_HUMI_ERROR_INIT;
    }

    ret = shtc3_driver_wakeup();
    if (ret < 0)
    {
        return TEMP_HUMI_ERROR_COMM;
    }

    return TEMP_HUMI_SUCCESS;
}
