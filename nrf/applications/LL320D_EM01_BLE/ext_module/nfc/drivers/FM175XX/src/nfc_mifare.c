/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        nfc_mifare.c
**文件描述:        Mifare Classic卡操作实现
**当前版本:        V1.0
**作    者:        Harrison Wu
**完成日期:        2026.01.22
*********************************************************************
** 功能描述:        实现Mifare Classic 1K/4K卡认证和块读写
*********************************************************************/

#include "nfc_mifare.h"
#include "nfc_reader_api.h"
#include "fm175xx_driver.h"

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

/* 注册Mifare模块日志 */
LOG_MODULE_REGISTER(nfc_mifare, LOG_LEVEL_INF);

/* Mifare操作变量 */
uint8_t MIFARE_SECTOR = 0;
uint8_t MIFARE_BLOCK = 0;
uint8_t MIFARE_BLOCK_NUM = 0;
uint8_t MIFARE_BLOCK_DATA[MIFARE_BLOCK_SIZE] = {0};

/* 默认密钥A（16个扇区） */
uint8_t MIFARE_KEY_A[16][6] = {
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}
};

/* 默认密钥B（16个扇区） */
uint8_t MIFARE_KEY_B[16][6] = {
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}
};

/********************************************************************
**函数名称:  nfc_mifare_auth
**入口参数:  mode      --- KEY_A_AUTH或KEY_B_AUTH
**           sector    --- 扇区号(0-15)
**           key       --- 6字节密钥
**           card_uid  --- 4字节UID
**函数功能:  Mifare卡片认证
**返 回 值:  FM175XX_SUCCESS或错误码
*********************************************************************/
uint8_t nfc_mifare_auth(uint8_t mode, uint8_t sector, uint8_t *key, uint8_t *card_uid)
{
    struct command_struct cmd_status;
    uint8_t outbuf[12];
    uint8_t inbuf[1];
    uint8_t result;
    uint8_t reg_val;

    /* 构造认证命令 */
    outbuf[0] = (mode == KEY_A_AUTH) ? 0x60 : 0x61; /* 认证命令 */
    outbuf[1] = sector * 4;                         /* 块地址（扇区首块） */

    /* 拷贝密钥 */
    memcpy(&outbuf[2], key, 6);

    /* 拷贝UID */
    memcpy(&outbuf[8], card_uid, 4);

    cmd_status.SendCRCEnable = FM175XX_SET;
    cmd_status.ReceiveCRCEnable = FM175XX_RESET;
    cmd_status.pSendBuf = outbuf;
    cmd_status.pReceiveBuf = inbuf;
    cmd_status.nBytesToSend = 12;
    cmd_status.nBitsToSend = 0;
    cmd_status.nBytesToReceive = 0;
    cmd_status.nBitsToReceive = 0;
    cmd_status.Timeout = 10;
    cmd_status.Cmd = CMD_AUTHENT;

    result = nfc_command_execute(&cmd_status);

    if (result == FM175XX_SUCCESS)
    {
        /* 检查加密标志位 */
        fm175xx_read_reg(JREG_STATUS2, &reg_val);
        if (reg_val & 0x08)
        {
            LOG_INF("Mifare auth success, sector %d", sector);
            return FM175XX_SUCCESS;
        }
        else
        {
            LOG_WRN("Mifare auth failed, sector %d", sector);
            return FM175XX_AUTH_ERR;
        }
    }

    return FM175XX_AUTH_ERR;
}

/********************************************************************
**函数名称:  nfc_mifare_read_block
**入口参数:  block     --- 块号(0-63)
**出口参数:  data_buf  --- 16字节数据缓冲区
**函数功能:  读取Mifare块数据
**返 回 值:  FM175XX_SUCCESS或错误码
*********************************************************************/
uint8_t nfc_mifare_read_block(uint8_t block, uint8_t *data_buf)
{
    struct command_struct cmd_status;
    uint8_t outbuf[2];
    uint8_t inbuf[16];
    uint8_t result;

    outbuf[0] = 0x30; /* 读块命令 */
    outbuf[1] = block;

    cmd_status.SendCRCEnable = FM175XX_SET;
    cmd_status.ReceiveCRCEnable = FM175XX_SET;
    cmd_status.pSendBuf = outbuf;
    cmd_status.pReceiveBuf = inbuf;
    cmd_status.nBytesToSend = 2;
    cmd_status.nBitsToSend = 0;
    cmd_status.nBytesToReceive = 16;
    cmd_status.nBitsToReceive = 0;
    cmd_status.Timeout = 10;
    cmd_status.Cmd = CMD_TRANSCEIVE;

    result = nfc_command_execute(&cmd_status);

    if ((result == FM175XX_SUCCESS) && (cmd_status.nBytesReceived == 16))
    {
        memcpy(data_buf, inbuf, 16);
        LOG_INF("Mifare read block %d success", block);
        return FM175XX_SUCCESS;
    }

    LOG_ERR("Mifare read block %d failed", block);
    return FM175XX_COMM_ERR;
}

/********************************************************************
**函数名称:  nfc_mifare_write_block
**入口参数:  block     --- 块号(0-63)
**           data_buf  --- 16字节数据
**函数功能:  写入Mifare块数据
**返 回 值:  FM175XX_SUCCESS或错误码
*********************************************************************/
uint8_t nfc_mifare_write_block(uint8_t block, uint8_t *data_buf)
{
    struct command_struct cmd_status;
    uint8_t outbuf[18];
    uint8_t inbuf[1];
    uint8_t result;

    /* 第一步：发送写命令 */
    outbuf[0] = 0xA0; /* 写块命令 */
    outbuf[1] = block;

    cmd_status.SendCRCEnable = FM175XX_SET;
    cmd_status.ReceiveCRCEnable = FM175XX_RESET;
    cmd_status.pSendBuf = outbuf;
    cmd_status.pReceiveBuf = inbuf;
    cmd_status.nBytesToSend = 2;
    cmd_status.nBitsToSend = 0;
    cmd_status.nBytesToReceive = 1;
    cmd_status.nBitsToReceive = 4; /* 4位ACK */
    cmd_status.Timeout = 10;
    cmd_status.Cmd = CMD_TRANSCEIVE;

    result = nfc_command_execute(&cmd_status);

    /* 检查ACK */
    if ((result != FM175XX_SUCCESS) ||
        (cmd_status.nBitsReceived != 4) ||
        ((cmd_status.pReceiveBuf[0] & 0x0F) != 0x0A))
    {
        LOG_ERR("Mifare write block %d no ACK", block);
        return FM175XX_COMM_ERR;
    }

    /* 第二步：发送16字节数据 */
    memcpy(outbuf, data_buf, 16);

    cmd_status.SendCRCEnable = FM175XX_SET;
    cmd_status.ReceiveCRCEnable = FM175XX_RESET;
    cmd_status.pSendBuf = outbuf;
    cmd_status.pReceiveBuf = inbuf;
    cmd_status.nBytesToSend = 16;
    cmd_status.nBitsToSend = 0;
    cmd_status.nBytesToReceive = 1;
    cmd_status.nBitsToReceive = 4;
    cmd_status.Timeout = 10;
    cmd_status.Cmd = CMD_TRANSCEIVE;

    result = nfc_command_execute(&cmd_status);

    /* 检查ACK */
    if ((result == FM175XX_SUCCESS) &&
        (cmd_status.nBitsReceived == 4) &&
        ((cmd_status.pReceiveBuf[0] & 0x0F) == 0x0A))
    {
        LOG_INF("Mifare write block %d success", block);
        return FM175XX_SUCCESS;
    }

    LOG_ERR("Mifare write block %d failed", block);
    return FM175XX_COMM_ERR;
}
