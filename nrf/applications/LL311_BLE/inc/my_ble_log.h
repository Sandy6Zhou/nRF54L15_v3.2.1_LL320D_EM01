/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        my_ble_log.h
**文件描述:        蓝牙日志输出头文件
**当前版本:        V1.0
**作    者:        Harrison Wu (wuyujiao@jimiiot.com)
**完成日期:        2026.03.17
*********************************************************************
** 功能描述:        提供模块化分级蓝牙日志输出功能
**                 1. 支持按模块开关控制
**                 2. 支持按日志等级过滤
**                 3. 需要蓝牙连接且总开关开启才发送
** 注意事项:
**                 1. DUMP日志一般数据量大，不支持从蓝牙发送
**                 2. 蓝牙日志发送时要检查递归调用，避免栈溢出
**                 3. 蓝牙日志吞吐量有限，避免发送大量数据
*********************************************************************/

#ifndef _MY_BLE_LOG_H_
#define _MY_BLE_LOG_H_

/* 日志等级定义 (与Zephyr一致) */
#define BLE_LOG_LEVEL_NONE  0   /* 不输出 */
#define BLE_LOG_LEVEL_ERR   1   /* 错误 */
#define BLE_LOG_LEVEL_WRN   2   /* 警告 */
#define BLE_LOG_LEVEL_INF   3   /* 信息 */
#define BLE_LOG_LEVEL_DBG   4   /* 调试 */

/* 模块ID需在编译时指定，默认为OTHER */
#ifndef BLE_LOG_MODULE_ID
#define BLE_LOG_MODULE_ID   BLE_LOG_MOD_OTHER
#endif

/********************************************************************
**函数名称:  ble_log_output
**入口参数:  mod_id: 模块ID
**           level: 日志等级
**           fmt: 格式化字符串
**           ...: 可变参数
**出口参数:  无
**函数功能:  输出蓝牙日志（内部使用）
**返 回 值:  无
*********************************************************************/
void ble_log_output(uint8_t mod_id, uint8_t level, const char *fmt, ...);

/********************************************************************
**函数名称:  ble_log_check_enabled
**入口参数:  mod_id: 模块ID, level: 日志等级
**出口参数:  无
**函数功能:  检查指定模块和等级的蓝牙日志是否允许输出
**返 回 值:  true=允许, false=不允许
*********************************************************************/
bool ble_log_check_enabled(uint8_t mod_id, uint8_t level);

/********************************************************************
**函数名称:  ble_log_set_ready
**入口参数:  ready: true=设置就绪, false=清除就绪
**出口参数:  无
**函数功能:  设置蓝牙日志发送就绪状态
**           应在配对鉴权完成后调用，并等待至少1秒
**返 回 值:  无
*********************************************************************/
void ble_log_set_ready(bool ready);

/********************************************************************
**函数名称:  ble_log_is_ready
**入口参数:  无
**出口参数:  无
**函数功能:  检查蓝牙日志发送是否就绪（配对完成+延迟1秒）
**返 回 值:  true=就绪, false=未就绪
*********************************************************************/
bool ble_log_is_ready(void);

/* 蓝牙日志输出宏 - 仅蓝牙 */
#define BLE_LOG_ERR(fmt, ...) \
    ble_log_output(BLE_LOG_MODULE_ID, BLE_LOG_LEVEL_ERR, "E: " fmt, ##__VA_ARGS__)

#define BLE_LOG_WRN(fmt, ...) \
    ble_log_output(BLE_LOG_MODULE_ID, BLE_LOG_LEVEL_WRN, "W: " fmt, ##__VA_ARGS__)

#define BLE_LOG_INF(fmt, ...) \
    ble_log_output(BLE_LOG_MODULE_ID, BLE_LOG_LEVEL_INF, "I: " fmt, ##__VA_ARGS__)

#define BLE_LOG_DBG(fmt, ...) \
    ble_log_output(BLE_LOG_MODULE_ID, BLE_LOG_LEVEL_DBG, "D: " fmt, ##__VA_ARGS__)

/* 同时输出RTT和蓝牙的日志宏 */
#define MY_LOG_ERR(fmt, ...) \
    do { \
        LOG_ERR(fmt, ##__VA_ARGS__); \
        BLE_LOG_ERR(fmt, ##__VA_ARGS__); \
    } while (0)

#define MY_LOG_WRN(fmt, ...) \
    do { \
        LOG_WRN(fmt, ##__VA_ARGS__); \
        BLE_LOG_WRN(fmt, ##__VA_ARGS__); \
    } while (0)

#define MY_LOG_INF(fmt, ...) \
    do { \
        LOG_INF(fmt, ##__VA_ARGS__); \
        BLE_LOG_INF(fmt, ##__VA_ARGS__); \
    } while (0)

#define MY_LOG_DBG(fmt, ...) \
    do { \
        LOG_DBG(fmt, ##__VA_ARGS__); \
        BLE_LOG_DBG(fmt, ##__VA_ARGS__); \
    } while (0)

#endif /* _MY_BLE_LOG_H_ */
