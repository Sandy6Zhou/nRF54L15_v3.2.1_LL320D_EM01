/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        my_ble_log.c
**文件描述:        蓝牙日志输出实现
**当前版本:        V1.0
**作    者:        Harrison Wu (wuyujiao@jimiiot.com)
**完成日期:        2026.03.17
*********************************************************************
** 功能描述:        提供模块化分级蓝牙日志输出功能
**                 1. 支持按模块开关控制日志输出
**                 2. 支持按日志等级过滤
**                 3. 需要蓝牙连接且总开关开启才允许发送蓝牙日志
**                 4. 蓝牙OTA期间禁用蓝牙日志发送
**                 5. DUMP日志一般数据量大，不支持从蓝牙发送
**                 6. 蓝牙日志发送时要检查递归调用，避免栈溢出
**                 7. 蓝牙日志吞吐量有限，避免发送大量数据
*********************************************************************/

/* 定义蓝牙日志模块ID为OTHER模块 - 必须在包含my_comm.h之前定义 */
#define BLE_LOG_MODULE_ID  BLE_LOG_MOD_OTHER

#include "my_comm.h"

/* 定义日志模块 */
LOG_MODULE_REGISTER(my_ble_log, LOG_LEVEL_INF);

/* 蓝牙日志发送就绪标志和时间戳 */
static bool ble_log_ready = false;
static int64_t ble_log_ready_time = 0;
#define BLE_LOG_READY_DELAY_MS  1000    /* 就绪后等待1秒才允许发送 */

/********************************************************************
**函数名称:  ble_log_check_enabled
**入口参数:  mod_id: 模块ID, level: 日志等级
**出口参数:  无
**函数功能:  检查指定模块和等级的蓝牙日志是否允许输出
**返 回 值:  true=允许, false=不允许
**检查条件:  1. 总开关开启
**           2. 蓝牙已连接
**           3. 该模块开关开启
**           4. 日志等级 <= 模块阈值（小于等于）
*********************************************************************/
bool ble_log_check_enabled(uint8_t mod_id, uint8_t level)
{
    BleLogConfig_t *config;

    config = my_param_get_ble_log_config();

    /* 检查总开关是否开启 */
    if (config->global_en == 0)
    {
        return false;
    }

    /* 必须等APP使能CCC通知后才能发送，否则可能导致栈溢出 */
    if (!ble_is_data_channel_ready())
    {
        return false;
    }

    /* 必须等配对鉴权完成并延迟1秒后才能发送，避免影响蓝牙权鉴流程 */
    if (!ble_log_is_ready())
    {
        return false;
    }

    /* DFU OTA 期间禁用蓝牙日志发送，避免干扰 OTA 传输 */
    if (jimi_dfu_is_in_progress())
    {
        return false;
    }

    /* 检查模块ID是否合法 */
    if (mod_id >= BLE_LOG_MOD_MAX)
    {
        return false;
    }

    /* 检查模块是否使能 */
    if (!BLE_LOG_MOD_IS_ENABLED(config, mod_id))
    {
        return false;
    }

    /* 检查日志等级是否符合要求 */
    if (level > config->mod_level[mod_id])
    {
        return false;
    }

    return true;
}

/********************************************************************
**函数名称:  ble_log_set_ready
**入口参数:  ready: true=设置就绪, false=清除就绪
**出口参数:  无
**函数功能:  设置蓝牙日志发送就绪状态
**           应在配对鉴权完成后调用，并等待至少1秒
**返 回 值:  无
*********************************************************************/
void ble_log_set_ready(bool ready)
{
    ble_log_ready = ready;
    if (ready)
    {
        ble_log_ready_time = k_uptime_get();
        LOG_INF("BLE log will be ready after %d ms", BLE_LOG_READY_DELAY_MS);
    }
    else
    {
        ble_log_ready_time = 0;
    }
}

/********************************************************************
**函数名称:  ble_log_is_ready
**入口参数:  无
**出口参数:  无
**函数功能:  检查蓝牙日志发送是否就绪（配对完成+延迟1秒）
**返 回 值:  true=就绪, false=未就绪
*********************************************************************/
bool ble_log_is_ready(void)
{
    if (!ble_log_ready)
    {
        return false;
    }

    /* 检查是否已过延迟时间 */
    if (k_uptime_get() - ble_log_ready_time < BLE_LOG_READY_DELAY_MS)
    {
        return false;
    }

    return true;
}

/********************************************************************
**函数名称:  ble_log_output
**入口参数:  mod_id: 模块ID
**           level: 日志等级
**           fmt: 格式化字符串
**           ...: 可变参数
**出口参数:  无
**函数功能:  格式化并发送蓝牙日志
**返 回 值:  无
*********************************************************************/
void ble_log_output(uint8_t mod_id, uint8_t level, const char *fmt, ...)
{
    va_list args;
    char buf[256];
    int len;

    if (!ble_log_check_enabled(mod_id, level))
    {
        return;
    }

    va_start(args, fmt);
    len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len <= 0)
    {
        return;
    }

    if (len < (int)sizeof(buf) - 1 && buf[len - 1] != '\n')
    {
        buf[len] = '\n';
        buf[len + 1] = '\0';
        len++;
    }

    ble_log_send((uint8_t *)buf, len);
}
