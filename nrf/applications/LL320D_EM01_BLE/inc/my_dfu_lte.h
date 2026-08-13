/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        my_dfu_lte.h
**文件描述:        LTE YModem OTA 头文件
**当前版本:        V1.0
**作    者:       周森达 (zhousenda@jimiiot.com)
**完成日期:        2026.08.11
*********************************************************************
** 功能描述:        1. 通过 LTE 串口接收 4G 端 YModem 固件
**                 2. 写入 mcuboot_secondary(image_1) 分区并触发 MCUboot 升级
**                 3. 与 BLE DFU(my_dfu_jimi) 互斥, 仅同时存在一个 DFU 会话
*********************************************************************/
#ifndef __MY_DFU_LTE_H__
#define __MY_DFU_LTE_H__

/********************************************************************
**枚举名称:  dfu_lte_fail_reason_t
**功能描述:  MCUOTA 升级失败原因 (对应协议 LTE+MCUOTA=FAIL,<reason> 字符串)
**注意事项:  取值顺序与命令层 reason 字符串映射表一一对应
*********************************************************************/
typedef enum {
    DFU_LTE_OK = 0,          /* 成功 */
    DFU_LTE_FAIL_BUSY,       /* 另一 OTA 会话进行中 */
    DFU_LTE_FAIL_PARAM,      /* START 参数格式/大小非法 */
    DFU_LTE_FAIL_SIZE,       /* total_size 超分区容量 或 与 Ymodem 首帧不一致 */
    DFU_LTE_FAIL_YMODEM,     /* Ymodem 协议接收失败 */
    DFU_LTE_FAIL_CRC,        /* 文件级 CRC32 校验不一致 */
    DFU_LTE_FAIL_FLASH,      /* Flash 写入失败 */
} dfu_lte_fail_reason_t;

/********************************************************************
**函数名称:  my_dfu_lte_prepare
**入口参数:  total_size  ---  4G 下发的固件总大小(字节)
**           expect_crc  ---  4G 下发的期望文件级 CRC32
**出口参数:  无
**函数功能:  升级前置检查并占用 OTA 会话 (校验大小合法性/分区容量/互斥)
**返 回 值:  DFU_LTE_OK 成功, 否则为对应失败原因
**注意事项:  需在 LTE 线程内调用; 成功后由命令层回 READY 再调 my_dfu_lte_start
*********************************************************************/
dfu_lte_fail_reason_t my_dfu_lte_prepare(uint32_t total_size, uint32_t expect_crc);

/********************************************************************
**函数名称:  my_dfu_lte_start
**入口参数:  无
**出口参数:  无
**函数功能:  启动 LTE YModem 固件接收(阻塞), 收完后做文件级 CRC32 校验,
**           校验通过请求 MCUboot 升级并调度延迟复位
**返 回 值:  DFU_LTE_OK 表示接收+校验成功, 否则为对应失败原因
**注意事项:  需在 my_dfu_lte_prepare 成功后于 LTE 线程内调用;
**           执行期间不再处理其他 LTE 消息
*********************************************************************/
dfu_lte_fail_reason_t my_dfu_lte_start(void);

#endif /* __MY_DFU_LTE_H__ */
