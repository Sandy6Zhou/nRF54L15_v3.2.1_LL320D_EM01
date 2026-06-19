/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        my_dfu_jimi.h
**文件描述:        Jimi 自定义 DFU 协议头文件
**当前版本:        V1.0
**作    者:        Harrison Wu (wuyujiao@jimiiot.com)
**完成日期:        2026.03.10
*********************************************************************
** 功能描述:        1. 基于几米自定义 BLE 3.0 DFU 协议解析(Jimi Iot 蓝牙通信协议V3.1.6_2026-3-5)
**                 2. 支持文件传输、MD5/CRC 校验、Flash 写入
**                 3. 与 MCUmgr OTA 方案共存
** 校验机制说明:    APP 端使用 MD5 算法计算整包校验值，但 NCS 3.2.1 SDK 已废弃 MD5 支持。
**                 为保证数据完整性，设备端采用片段 CRC16 校验替代：每帧数据写入 Flash
**                 后立即读回验证 CRC，若 CRC 正确则认为该片段数据完整，等效于 MD5 验证通过。
*********************************************************************/
#ifndef __MY_DFU_JIMI_H__
#define __MY_DFU_JIMI_H__

/* DFU 文件 MD5 缓冲区长度 (APP 端使用，设备端用 CRC 替代) */
#define FILE_MD5_BUF_LEN 16

/* DFU 结果 */
#define JIMI_DFU_OK   1
#define JIMI_DFU_FAIL 0

/* DFU 命令码 */
#define JIMI_DFU_START      0x01 // DFU 请求文件开始
#define JIMI_DFU_FILE_SIZE  0x02 // DFU 通知传输文件大小
#define JIMI_DFU_FILE_IMAGE 0x03 // DFU 请求发送文件片段
#define JIMI_DFU_FILE_END   0x04 // DFU 文件传输结束

/* DFU 结束响应码 */
#define JIMI_DFU_END_RESP_OK       0x00
#define JIMI_DFU_END_RESP_ERROR    0x01
#define JIMI_DFU_END_RESP_MD5      0x02 /* APP 端 MD5 校验失败 (设备端用 CRC 替代) */
#define JIMI_DFU_END_RESP_SIZE     0x03
#define JIMI_DFU_END_RESP_TIME_OUT 0x04

/* DFU 重置延迟 */
#define JIMI_DFU_RESET_DELAY 1000 // 单位: ms

/* DFU 命令处理函数指针 */
typedef void (*jimi_dfu_handler_fp_t)(uint8_t *data, uint16_t len);

/* DFU 命令处理函数表 */
typedef struct
{
    uint8_t opcode;
    jimi_dfu_handler_fp_t cmd_handler;
} jimi_dfu_handler_table_t;

/********************************************************************
**函数名称:  jimi_dfu_timer_init
**入口参数:  无
**出口参数:  无
**函数功能:  初始化 DFU 定时器
**返 回 值:  无
*********************************************************************/
void jimi_dfu_timer_init(void);

/********************************************************************
**函数名称:  jimi_dfu_cmd_handler
**入口参数:  cmd   ---   命令码
**           data  ---   数据缓冲区
**           len   ---   数据长度
**出口参数:  无
**函数功能:  DFU 命令分发处理
**返 回 值:  无
*********************************************************************/
void jimi_dfu_cmd_handler(uint8_t cmd, uint8_t *data, uint16_t len);

/********************************************************************
**函数名称:  jimi_dfu_is_in_progress
**入口参数:  无
**出口参数:  无
**函数功能:  检查 DFU OTA 是否正在进行中
**返 回 值:  true=进行中, false=未进行
**注意事项:  DFU 期间应禁用蓝牙日志发送，避免干扰 OTA 传输
*********************************************************************/
bool jimi_dfu_is_in_progress(void);

#endif /* __MY_DFU_JIMI_H__ */
