/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        nfc_mifare.h
**文件描述:        Mifare Classic卡操作头文件
**当前版本:        V1.0
**作    者:        Harrison Wu
**完成日期:        2026.01.22
*********************************************************************
** 功能描述:        封装Mifare Classic 1K/4K卡认证和块读写操作
*********************************************************************/

#ifndef _NFC_MIFARE_H_
#define _NFC_MIFARE_H_

#include <stdint.h>
#include "fm175xx_reg.h"

/* 认证模式 */
#define KEY_A_AUTH              0
#define KEY_B_AUTH              1

/* 块大小 */
#define MIFARE_BLOCK_SIZE       16

/* 外部变量 */
extern uint8_t MIFARE_SECTOR;
extern uint8_t MIFARE_BLOCK;
extern uint8_t MIFARE_BLOCK_NUM;
extern uint8_t MIFARE_BLOCK_DATA[MIFARE_BLOCK_SIZE];
extern uint8_t MIFARE_KEY_A[16][6];
extern uint8_t MIFARE_KEY_B[16][6];

/********************************************************************
**函数名称:  nfc_mifare_auth
**入口参数:  mode      --- KEY_A_AUTH或KEY_B_AUTH
**           sector    --- 扇区号(0-15)
**           key       --- 6字节密钥
**           card_uid  --- 4字节UID
**函数功能:  Mifare卡片认证
**返 回 值:  FM175XX_SUCCESS或错误码
*********************************************************************/
uint8_t nfc_mifare_auth(uint8_t mode, uint8_t sector, 
                        uint8_t *key, uint8_t *card_uid);

/********************************************************************
**函数名称:  nfc_mifare_read_block
**入口参数:  block     --- 块号(0-63)
**出口参数:  data_buf  --- 16字节数据缓冲区
**函数功能:  读取Mifare块数据
**返 回 值:  FM175XX_SUCCESS或错误码
*********************************************************************/
uint8_t nfc_mifare_read_block(uint8_t block, uint8_t *data_buf);

/********************************************************************
**函数名称:  nfc_mifare_write_block
**入口参数:  block     --- 块号(0-63)
**           data_buf  --- 16字节数据
**函数功能:  写入Mifare块数据
**返 回 值:  FM175XX_SUCCESS或错误码
*********************************************************************/
uint8_t nfc_mifare_write_block(uint8_t block, uint8_t *data_buf);

#endif /* _NFC_MIFARE_H_ */
