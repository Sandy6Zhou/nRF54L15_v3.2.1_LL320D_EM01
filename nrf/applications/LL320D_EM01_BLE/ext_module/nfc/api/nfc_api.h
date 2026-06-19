/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        nfc_api.h
**文件描述:        NFC 模块统一 API 接口头文件
**当前版本:        V1.0
**作    者:        Harrison Wu (wuyujiao@jimiiot.com)
**完成日期:        2026.02.28
*********************************************************************
** 功能描述:        提供统一的 NFC 功能接口，屏蔽底层驱动差异
**                 支持多厂商 NFC 芯片（FM175XX 等）
*********************************************************************/

#ifndef _NFC_API_H_
#define _NFC_API_H_

#include <stdint.h>
#include <stdbool.h>

/* NFC 卡片类型定义 */
typedef enum
{
    NFC_CARD_TYPE_UNKNOWN = 0,
    NFC_CARD_TYPE_MIFARE,      /* Mifare Classic */
    NFC_CARD_TYPE_NTAG,        /* NTag Type 2 */
} nfc_card_type_t;

/* NFC 卡片信息结构体 */
struct nfc_card_info
{
    nfc_card_type_t type;
    uint8_t uid[8];
    uint8_t uid_len;
    uint8_t data[64];
    uint8_t data_len;
};

/* NFC 操作结果 */
typedef enum
{
    NFC_SUCCESS = 0,
    NFC_ERROR_INIT,            /* 初始化错误 */
    NFC_ERROR_COMM,            /* 通信错误 */
    NFC_ERROR_NO_CARD,         /* 无卡片 */
    NFC_ERROR_AUTH,            /* 认证错误 */
    NFC_ERROR_PARAM,           /* 参数错误 */
} nfc_result_t;

/* 卡片检测回调函数类型 */
typedef void (*nfc_card_detected_cb_t)(nfc_card_type_t type,
                                        uint8_t *uid, uint8_t uid_len,
                                        uint8_t *data, uint8_t data_len);

/********************************************************************
**函数名称:  nfc_api_init
**入口参数:  cb       ---        卡片检测回调函数指针
**出口参数:  无
**函数功能:  初始化 NFC 模块，包括底层驱动和硬件复位
**返 回 值:  NFC_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
nfc_result_t nfc_api_init(nfc_card_detected_cb_t cb);

/********************************************************************
**函数名称:  nfc_api_deinit
**入口参数:  无
**出口参数:  无
**函数功能:  反初始化 NFC 模块，释放资源
**返 回 值:  NFC_SUCCESS 表示成功
*********************************************************************/
nfc_result_t nfc_api_deinit(void);

/********************************************************************
**函数名称:  nfc_api_poll_start
**入口参数:  无
**出口参数:  无
**函数功能:  启动 NFC 卡片轮询检测
**返 回 值:  NFC_SUCCESS 表示成功
*********************************************************************/
nfc_result_t nfc_api_poll_start(void);

/********************************************************************
**函数名称:  nfc_api_poll_stop
**入口参数:  无
**出口参数:  无
**函数功能:  停止 NFC 卡片轮询检测
**返 回 值:  NFC_SUCCESS 表示成功
*********************************************************************/
nfc_result_t nfc_api_poll_stop(void);

/********************************************************************
**函数名称:  nfc_api_read_block
**入口参数:  block    ---        块地址
**           data     ---        数据缓冲区指针
**           len      ---        缓冲区长度
**出口参数:  data     ---        读取到的数据
**函数功能:  读取 NFC 卡片指定块数据
**返 回 值:  NFC_SUCCESS 表示成功
*********************************************************************/
nfc_result_t nfc_api_read_block(uint8_t block, uint8_t *data, uint8_t len);

/********************************************************************
**函数名称:  nfc_api_write_block
**入口参数:  block    ---        块地址
**           data     ---        要写入的数据指针
**           len      ---        数据长度
**出口参数:  无
**函数功能:  向 NFC 卡片指定块写入数据
**返 回 值:  NFC_SUCCESS 表示成功
*********************************************************************/
nfc_result_t nfc_api_write_block(uint8_t block, const uint8_t *data, uint8_t len);

/********************************************************************
**函数名称:  nfc_api_get_version
**入口参数:  无
**出口参数:  version  ---        版本号存储指针
**函数功能:  获取 NFC 芯片版本号
**返 回 值:  NFC_SUCCESS 表示成功
*********************************************************************/
nfc_result_t nfc_api_get_version(uint8_t *version);

/********************************************************************
**函数名称:  nfc_api_enter_hpd
**入口参数:  无
**出口参数:  无
**函数功能:  进入 Hard Power Down (HPD) 低功耗模式
**返 回 值:  NFC_SUCCESS 表示成功
*********************************************************************/
nfc_result_t nfc_api_enter_hpd(void);

/********************************************************************
**函数名称:  nfc_api_exit_hpd
**入口参数:  无
**出口参数:  无
**函数功能:  退出 HPD 模式，恢复 NFC 功能
**返 回 值:  NFC_SUCCESS 表示成功
*********************************************************************/
nfc_result_t nfc_api_exit_hpd(void);

#endif /* _NFC_API_H_ */
