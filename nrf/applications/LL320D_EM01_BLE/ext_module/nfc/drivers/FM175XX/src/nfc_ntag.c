/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        nfc_ntag.c
**文件描述:        NTag Type 2标签操作实现
**当前版本:        V1.0
**作    者:        Harrison Wu
**完成日期:        2026.01.22
*********************************************************************
** 功能描述:        实现NTag 213/215/216页读写操作
*********************************************************************/

#include "nfc_ntag.h"
#include "nfc_reader_api.h"

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

/* 注册NTag模块日志 */
LOG_MODULE_REGISTER(nfc_ntag, LOG_LEVEL_INF);

/* NTag操作变量 */
uint8_t NTAG_PAGE = 0;
uint8_t NTAG_PAGE_DATA[16] = {0};

/********************************************************************
**函数名称:  nfc_ntag_read
**入口参数:  page_addr --- 页地址
**出口参数:  data_buf  --- 16字节数据缓冲区
**函数功能:  读取NTag页数据（返回4页共16字节）
**返 回 值:  FM175XX_SUCCESS或错误码
*********************************************************************/
uint8_t nfc_ntag_read(uint8_t page_addr, uint8_t *data_buf)
{
    struct command_struct cmd_status;
    uint8_t outbuf[2];
    uint8_t inbuf[16];
    uint8_t result;

    outbuf[0] = 0x30; /* READ命令 */
    outbuf[1] = page_addr;

    cmd_status.SendCRCEnable = FM175XX_SET;
    cmd_status.ReceiveCRCEnable = FM175XX_SET;
    cmd_status.pSendBuf = outbuf;
    cmd_status.pReceiveBuf = inbuf;
    cmd_status.nBytesToSend = 2;
    cmd_status.nBitsToSend = 0;
    cmd_status.nBytesToReceive = 16; /* NTag读取返回16字节（4页） */
    cmd_status.nBitsToReceive = 0;
    cmd_status.Timeout = 5;
    cmd_status.Cmd = CMD_TRANSCEIVE;

    result = nfc_command_execute(&cmd_status);

    if ((result == FM175XX_SUCCESS) && (cmd_status.nBytesReceived == 16))
    {
        memcpy(data_buf, inbuf, 16);
        LOG_INF("NTag read page %d success", page_addr);
        return FM175XX_SUCCESS;
    }

    LOG_ERR("NTag read page %d failed, received %d bytes",
            page_addr, cmd_status.nBytesReceived);
    return FM175XX_COMM_ERR;
}

/********************************************************************
**函数名称:  nfc_ntag_write
**入口参数:  page_addr --- 页地址
**           data_buf  --- 4字节数据
**函数功能:  写入NTag页数据（每页4字节）
**返 回 值:  FM175XX_SUCCESS或错误码
*********************************************************************/
uint8_t nfc_ntag_write(uint8_t page_addr, uint8_t *data_buf)
{
    struct command_struct cmd_status;
    uint8_t outbuf[6];
    uint8_t inbuf[1];
    uint8_t result;

    outbuf[0] = 0xA2; /* WRITE命令 */
    outbuf[1] = page_addr;
    memcpy(&outbuf[2], data_buf, 4); /* 4字节数据 */

    cmd_status.SendCRCEnable = FM175XX_SET;
    cmd_status.ReceiveCRCEnable = FM175XX_RESET;
    cmd_status.pSendBuf = outbuf;
    cmd_status.pReceiveBuf = inbuf;
    cmd_status.nBytesToSend = 6;
    cmd_status.nBitsToSend = 0;
    cmd_status.nBytesToReceive = 1;
    cmd_status.nBitsToReceive = 4; /* 4位ACK */
    cmd_status.Timeout = 10;
    cmd_status.Cmd = CMD_TRANSCEIVE;

    result = nfc_command_execute(&cmd_status);

    /* 检查ACK */
    if ((result == FM175XX_SUCCESS) &&
        (cmd_status.nBitsReceived == 4) &&
        ((cmd_status.pReceiveBuf[0] & 0x0F) == 0x0A))
    {
        LOG_INF("NTag write page %d success", page_addr);
        return FM175XX_SUCCESS;
    }

    LOG_ERR("NTag write page %d failed", page_addr);
    return FM175XX_COMM_ERR;
}
