/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        nfc_reader_api.c
**文件描述:        NFC读卡器API层实现
**当前版本:        V1.0
**作    者:        Harrison Wu
**完成日期:        2026.01.22
*********************************************************************
** 功能描述:        实现Type A卡片操作接口
*********************************************************************/

#include "nfc_reader_api.h"
#include "fm175xx_driver.h"

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

/* 注册NFC Reader模块日志 */
LOG_MODULE_REGISTER(nfc_reader, LOG_LEVEL_INF);

/* Type A卡片信息 */
struct picc_a_struct PICC_A;

/* 命令代码 */
static const uint8_t RF_CMD_ANTICOL[3] = {0x93, 0x95, 0x97};

/********************************************************************
**函数名称:  nfc_set_send_crc
**入口参数:  mode --- 1使能，0禁用
**函数功能:  设置发送CRC
**返 回 值:  无
*********************************************************************/
void nfc_set_send_crc(uint8_t mode)
{
    if (mode)
    {
        fm175xx_modify_reg(JREG_TXMODE, JBIT_TXCRCEN, JBIT_TXCRCEN);
    }
    else
    {
        fm175xx_modify_reg(JREG_TXMODE, JBIT_TXCRCEN, FM175XX_RESET);
    }
}

/********************************************************************
**函数名称:  nfc_set_receive_crc
**入口参数:  mode --- 1使能，0禁用
**函数功能:  设置接收CRC
**返 回 值:  无
*********************************************************************/
void nfc_set_receive_crc(uint8_t mode)
{
    if (mode)
    {
        fm175xx_modify_reg(JREG_RXMODE, JBIT_RXCRCEN, JBIT_RXCRCEN);
    }
    else
    {
        fm175xx_modify_reg(JREG_RXMODE, JBIT_RXCRCEN, FM175XX_RESET);
    }
}

/********************************************************************
**函数名称:  nfc_set_timeout
**入口参数:  microseconds --- 超时时间（微秒）
**函数功能:  设置通信超时时间
**返 回 值:  无
*********************************************************************/
void nfc_set_timeout(uint32_t microseconds)
{
    uint32_t timereload;
    uint32_t prescaler;

    if (microseconds == 0)
    {
        microseconds = 1; /* 时间不能为0 */
    }

    prescaler = 0;
    timereload = 0;
    while (prescaler < 0xFFF)
    {
        timereload = ((microseconds * 13560UL) - 1) / (prescaler * 2 + 1);
        if (timereload < 0xFFFF)
        {
            break;
        }
        prescaler++;
    }
    timereload = timereload & 0xFFFF;

    /* JBIT_TAUTO打开计时器，TX结束自动启动计时，TPrescaler_Hi高4bit */
    fm175xx_write_reg(JREG_TMODE, JBIT_TAUTO | ((prescaler >> 8) & 0x0F));
    fm175xx_write_reg(JREG_TPRESCALER, prescaler & 0xFF); /* TPrescaler_Lo低8bit */
    fm175xx_write_reg(JREG_TRELOADHI, timereload >> 8);   /* TReloadVal_Hi */
    fm175xx_write_reg(JREG_TRELOADLO, timereload & 0xFF); /* TReloadVal_Lo */
}

/********************************************************************
**函数名称:  nfc_set_cw
**入口参数:  mode --- 载波模式(CW_DISABLE/CW1_ENABLE/CW2_ENABLE/CW_ENABLE)
**函数功能:  设置载波开关模式(CW_DISABLE/CW1_ENABLE/CW2_ENABLE/CW_ENABLE)
**返 回 值:  无
*********************************************************************/
void nfc_set_cw(uint8_t mode)
{
    if (mode == CW_DISABLE)
    {
        fm175xx_modify_reg(JREG_TXCONTROL, JBIT_TX1RFEN | JBIT_TX2RFEN, FM175XX_RESET); /* 关闭载波 */
    }
    else if (mode == CW1_ENABLE)
    {
        fm175xx_modify_reg(JREG_TXCONTROL, JBIT_TX1RFEN | JBIT_TX2RFEN, JBIT_TX1RFEN); /* 开启载波1 */
    }
    else if (mode == CW2_ENABLE)
    {
        fm175xx_modify_reg(JREG_TXCONTROL, JBIT_TX1RFEN | JBIT_TX2RFEN, JBIT_TX2RFEN); /* 开启载波2 */
    }
    else if (mode == CW_ENABLE)
    {
        fm175xx_modify_reg(JREG_TXCONTROL, JBIT_TX1RFEN | JBIT_TX2RFEN, JBIT_TX1RFEN | JBIT_TX2RFEN); /* 开启载波1和载波2 */
    }

    k_sleep(K_MSEC(5)); /* M1卡1ms，CPU卡或身份证卡5ms */
}

/********************************************************************
**函数名称:  nfc_command_execute
**入口参数:  cmd_status --- 命令状态结构体
**函数功能:  执行NFC命令
**返 回 值:  FM175XX_SUCCESS或错误码
*********************************************************************/
uint8_t nfc_command_execute(struct command_struct *cmd_status)
{
    uint8_t reg_data;
    uint8_t send_length, receive_length, send_finish;
    uint8_t irq;
    uint8_t result;
    uint32_t timeout_cnt = 0; /* 软件超时计数器 */

    send_length = cmd_status->nBytesToSend; /* 发送长度 */
    receive_length = 0;                     /* 接收长度 */
    send_finish = 0;                        /* 发送完成标志 */
    cmd_status->nBitsReceived = 0;          /* 接收位数 */
    cmd_status->nBytesReceived = 0;         /* 接收字节数 */
    cmd_status->CollPos = 0;                /* 碰撞位置 */
    cmd_status->Error = 0;                  /* 错误标志 */

    fm175xx_write_reg(JREG_COMMAND, CMD_IDLE);         /* 命令为空闲模式 */
    fm175xx_write_reg(JREG_FIFOLEVEL, JBIT_FLUSHFIFO); /* 清空FIFO */
    fm175xx_write_reg(JREG_COMMIRQ, 0x7F);             /* 使能命令中断 */
    fm175xx_write_reg(JREG_DIVIRQ, 0x7F);              /* 使能时钟中断 */
    fm175xx_write_reg(JREG_COMMIEN, 0x80);             /* 使能命令完成中断 */
    fm175xx_write_reg(JREG_DIVIEN, 0x00);              /* 禁用时钟中断 */
    fm175xx_write_reg(JREG_WATERLEVEL, 0x20);          /* 设置水位 */

    nfc_set_send_crc(cmd_status->SendCRCEnable);       /* 设置发送CRC */
    nfc_set_receive_crc(cmd_status->ReceiveCRCEnable); /* 设置接收CRC */
    nfc_set_timeout(cmd_status->Timeout);              /* 设置超时时间 */

    if (cmd_status->Cmd == CMD_AUTHENT)
    {
        fm175xx_write_fifo(send_length, cmd_status->pSendBuf); /* 写发送数据 */
        send_length = 0;
        fm175xx_write_reg(JREG_COMMAND, cmd_status->Cmd);                             /* 写命令 AUTHENT */
        fm175xx_write_reg(JREG_BITFRAMING, JBIT_STARTSEND | cmd_status->nBitsToSend); /* 写位 framing */
    }

    if (cmd_status->Cmd == CMD_TRANSCEIVE) /* 命令为 TRANSCEIVE */
    {
        fm175xx_write_reg(JREG_COMMAND, cmd_status->Cmd);                                                /* 写命令 */
        fm175xx_write_reg(JREG_BITFRAMING, (cmd_status->nBitsToReceive << 4) | cmd_status->nBitsToSend); /* 写位 framing，防冲突 */
    }

    while (1)
    {
        /* 软件超时保护，防止死循环 */
        if (++timeout_cnt > 100)
        {
            result = FM175XX_TIMER_ERR;
            LOG_ERR("Command execute timeout (software)");
            break;
        }

        fm175xx_read_reg(JREG_COMMIRQ, &irq); /* 读命令中断 */

        if (irq & JBIT_TIMERI)
        {                                                 /* 超时中断 */
            fm175xx_write_reg(JREG_COMMIRQ, JBIT_TIMERI); /* 清除超时中断 */
            result = FM175XX_TIMER_ERR;
            break;
        }

        if (irq & JBIT_ERRI)
        {                                            /* 错误中断 */
            fm175xx_read_reg(JREG_ERROR, &reg_data); /* 读错误标志 */
            cmd_status->Error = reg_data;
            if (cmd_status->Error & JBIT_COLLERR) /* 碰撞错误 */
            {
                fm175xx_read_reg(JREG_COLL, &reg_data); /* 读碰撞位置 */
                cmd_status->CollPos = reg_data & 0x1F;
                result = FM175XX_COLL_ERR;
                break;
            }
            result = FM175XX_COMM_ERR;                  /* 命令错误 */
            fm175xx_write_reg(JREG_COMMIRQ, JBIT_ERRI); /* 清除错误中断 */
            break;
        }

        /* 低水位中断 */
        if (irq & JBIT_LOALERTI)
        {
            if (send_length > 0)
            {
                if (send_length > 32)
                {
                    fm175xx_write_fifo(32, cmd_status->pSendBuf); /* 写发送数据 */
                    cmd_status->pSendBuf = cmd_status->pSendBuf + 32;
                    send_length = send_length - 32;
                }
                else
                {
                    fm175xx_write_fifo(send_length, cmd_status->pSendBuf);
                    send_length = 0;
                }
                fm175xx_modify_reg(JREG_BITFRAMING, JBIT_STARTSEND, JBIT_STARTSEND);
            }
            fm175xx_write_reg(JREG_COMMIRQ, JBIT_LOALERTI); /* 清除低水位中断 */
        }

        /* 高水位中断 */
        if (irq & JBIT_HIALERTI)
        {
            if (send_finish == 1) /* 发送完成标志 */
            {
                fm175xx_read_fifo(32, cmd_status->pReceiveBuf + cmd_status->nBytesReceived);
                cmd_status->nBytesReceived = cmd_status->nBytesReceived + 32;
            }
            fm175xx_write_reg(JREG_COMMIRQ, JBIT_HIALERTI);
        }

        if ((irq & JBIT_IDLEI) && (cmd_status->Cmd == CMD_AUTHENT)) /* 命令为 AUTHENT */
        {
            fm175xx_write_reg(JREG_COMMIRQ, JBIT_IDLEI); /* 清除空闲中断 */
            result = FM175XX_SUCCESS;
            break;
        }

        /* 接收完成标志 */
        if ((irq & JBIT_RXI) && (cmd_status->Cmd == CMD_TRANSCEIVE))
        {
            fm175xx_read_reg(JREG_CONTROL, &reg_data);
            cmd_status->nBitsReceived = reg_data & 0x07;
            fm175xx_read_reg(JREG_FIFOLEVEL, &reg_data);
            receive_length = reg_data & 0x7F;

            fm175xx_read_fifo(receive_length, cmd_status->pReceiveBuf + cmd_status->nBytesReceived);
            cmd_status->nBytesReceived = cmd_status->nBytesReceived + receive_length;

            if ((cmd_status->nBytesToReceive != cmd_status->nBytesReceived) && (cmd_status->nBytesToReceive != 0))
            {
                result = FM175XX_LENGTH_ERR;
                break;
            }
            fm175xx_write_reg(JREG_COMMIRQ, JBIT_RXI); /* 清除接收完成标志 */
            result = FM175XX_SUCCESS;
            break;
        }

        if (irq & JBIT_TXI)
        {
            fm175xx_write_reg(JREG_COMMIRQ, JBIT_TXI); /* 清除发送完成标志 */
            if (cmd_status->Cmd == CMD_TRANSCEIVE)
                send_finish = 1; /* 发送完成标志 */
        }
    }
    fm175xx_modify_reg(JREG_BITFRAMING, JBIT_STARTSEND, FM175XX_RESET); /* 清除发送完成标志 */
    fm175xx_write_reg(JREG_COMMAND, CMD_IDLE);                          /* 命令为空闲模式 */
    return result;
}

/********************************************************************
**函数名称:  nfc_init_reader_a
**入口参数:  无
**函数功能:  初始化Type A读卡器
**返 回 值:  无
*********************************************************************/
void nfc_init_reader_a(void)
{
    /* 设置TX/RX模式 */
    fm175xx_write_reg(JREG_TXMODE, 0x00);         /* TxCRCEnable=0, TxSpeed=106kbps, InvMode=0, TxFraming=ISO14443A */
    fm175xx_write_reg(JREG_RXMODE, JBIT_RXNOERR); /* RxCRCEnable=0, RxSpeed=106kbps, RxNoError=1, RxMultiple=0, RxFraming=ISO14443A */

    /* 强制100% ASK */
    fm175xx_modify_reg(JREG_TXAUTO, JBIT_FORCE100ASK, JBIT_FORCE100ASK);

    /* 设置调制宽度 */
    fm175xx_write_reg(JREG_MODWIDTH, MODWIDTH_106);

    /* 设置Initiator模式 */
    fm175xx_write_reg(JREG_CONTROL, JBIT_INITIATOR);

    /* 设置GSN */
    fm175xx_write_reg(JREG_GSN, 0xF1);

    /* 设置CWGSP */
    fm175xx_write_reg(JREG_CWGSP, 0x3F);

    /* 设置MODGSP */
    fm175xx_write_reg(JREG_MODGSP, 0x01);

    /* 设置接收增益 */
    fm175xx_write_reg(JREG_RFCFG, RXGAIN_A << 4);

    /* 设置接收阈值 */
    fm175xx_write_reg(JREG_RXTHRESHOLD, 0x84);

    /* 清空认证标志，针对M1卡 */
    fm175xx_modify_reg(JREG_STATUS2, JBIT_CRYPTO1ON, FM175XX_RESET);
}

/********************************************************************
**函数名称:  nfc_reader_a_wakeup
**入口参数:  无
**函数功能:  Type A唤醒卡片（WUPA命令）
**返 回 值:  FM175XX_SUCCESS或错误码
*********************************************************************/
uint8_t nfc_reader_a_wakeup(void)
{
    struct command_struct cmd_status;
    uint8_t outbuf[1] = {RF_CMD_WUPA}; /* WUPA命令 */
    uint8_t inbuf[2];
    uint8_t result;

    cmd_status.SendCRCEnable = FM175XX_RESET;
    cmd_status.ReceiveCRCEnable = FM175XX_RESET;
    cmd_status.pSendBuf = outbuf;
    cmd_status.pReceiveBuf = inbuf;
    cmd_status.nBytesToSend = 1;
    cmd_status.nBitsToSend = 7; /* 短帧，7位 */
    cmd_status.nBytesToReceive = 2;
    cmd_status.nBitsToReceive = 0;
    cmd_status.Timeout = 1; /* 超时时间设置为1MS，与原厂DEMO保持一致 */
    cmd_status.Cmd = CMD_TRANSCEIVE;

    result = nfc_command_execute(&cmd_status);

    if (result == FM175XX_SUCCESS && cmd_status.nBytesReceived == 2)
    {
        PICC_A.ATQA[0] = inbuf[0];
        PICC_A.ATQA[1] = inbuf[1];
        LOG_DBG("ATQA: %02X %02X", PICC_A.ATQA[0], PICC_A.ATQA[1]);
    }

    return result;
}

/********************************************************************
**函数名称:  nfc_reader_a_request
**入口参数:  无
**函数功能:  Type A请求卡片（REQA命令）
**返 回 值:  FM175XX_SUCCESS或错误码
*********************************************************************/
uint8_t nfc_reader_a_request(void)
{
    struct command_struct cmd_status;
    uint8_t outbuf[1] = {RF_CMD_REQA}; /* REQA命令 */
    uint8_t inbuf[2];
    uint8_t result;

    cmd_status.SendCRCEnable = FM175XX_RESET;
    cmd_status.ReceiveCRCEnable = FM175XX_RESET;
    cmd_status.pSendBuf = outbuf;
    cmd_status.pReceiveBuf = inbuf;
    cmd_status.nBytesToSend = 1;
    cmd_status.nBitsToSend = 7; /* 短帧，7位 */
    cmd_status.nBytesToReceive = 2;
    cmd_status.nBitsToReceive = 0;
    cmd_status.Timeout = 1; /* 超时时间设置为1MS，与原厂DEMO保持一致 */
    cmd_status.Cmd = CMD_TRANSCEIVE;

    result = nfc_command_execute(&cmd_status);

    if (result == FM175XX_SUCCESS && cmd_status.nBytesReceived == 2)
    {
        PICC_A.ATQA[0] = inbuf[0];
        PICC_A.ATQA[1] = inbuf[1];
        LOG_DBG("ATQA: %02X %02X", PICC_A.ATQA[0], PICC_A.ATQA[1]);
    }

    return result;
}

/********************************************************************
**函数名称:  nfc_reader_a_anticoll
**入口参数:  cascade_level --- 级联等级(0-2)
**函数功能:  Type A防冲突
**返 回 值:  FM175XX_SUCCESS或错误码
*********************************************************************/
uint8_t nfc_reader_a_anticoll(uint8_t cascade_level)
{
    struct command_struct cmd_status;
    uint8_t outbuf[2] = {RF_CMD_ANTICOL[cascade_level], 0x20};
    uint8_t inbuf[5];
    uint8_t result;

    if (cascade_level > 2)
    {
        return FM175XX_PARAM_ERR;
    }

    cmd_status.SendCRCEnable = FM175XX_RESET;
    cmd_status.ReceiveCRCEnable = FM175XX_RESET;
    cmd_status.pSendBuf = outbuf;
    cmd_status.pReceiveBuf = inbuf;
    cmd_status.nBytesToSend = 2;
    cmd_status.nBitsToSend = 0;
    cmd_status.nBytesToReceive = 5;
    cmd_status.nBitsToReceive = 0;
    cmd_status.Timeout = 1; /* 超时时间设置为1MS，与原厂DEMO保持一致 */
    cmd_status.Cmd = CMD_TRANSCEIVE;

    result = nfc_command_execute(&cmd_status); /* 执行命令 */

    fm175xx_modify_reg(JREG_COLL, JBIT_VALUESAFTERCOLL, JBIT_VALUESAFTERCOLL); /* 忽略冲突后值 */

    if ((result == FM175XX_SUCCESS) && (cmd_status.nBytesReceived == 5))
    {
        memcpy(PICC_A.UID + (cascade_level * 4), inbuf, 4);
        PICC_A.BCC[cascade_level] = inbuf[4];

        /* 校验BCC: UID[0] ^ UID[1] ^ UID[2] ^ UID[3] ^ BCC 应该等于 0 */
        if ((PICC_A.UID[cascade_level * 4] ^
             PICC_A.UID[cascade_level * 4 + 1] ^
             PICC_A.UID[cascade_level * 4 + 2] ^
             PICC_A.UID[cascade_level * 4 + 3] ^
             PICC_A.BCC[cascade_level]) != 0)
        {
            result = FM175XX_COMM_ERR;
        }
    }
    else
    {
        result = FM175XX_COMM_ERR;
    }

    return result;
}

/********************************************************************
**函数名称:  nfc_reader_a_select
**入口参数:  cascade_level --- 级联等级(0-2)
**函数功能:  Type A选择卡片
**返 回 值:  FM175XX_SUCCESS或错误码
*********************************************************************/
uint8_t nfc_reader_a_select(uint8_t cascade_level)
{
    struct command_struct cmd_status;
    uint8_t outbuf[7];
    uint8_t inbuf[1];
    uint8_t result;

    if (cascade_level > 2)
    {
        return FM175XX_PARAM_ERR;
    }

    outbuf[0] = RF_CMD_ANTICOL[cascade_level];
    outbuf[1] = 0x70;
    memcpy(&outbuf[2], PICC_A.UID + (cascade_level * 4), 4);
    outbuf[6] = PICC_A.BCC[cascade_level];

    cmd_status.SendCRCEnable = FM175XX_SET;
    cmd_status.ReceiveCRCEnable = FM175XX_SET;
    cmd_status.pSendBuf = outbuf;
    cmd_status.pReceiveBuf = inbuf;
    cmd_status.nBytesToSend = 7;
    cmd_status.nBitsToSend = 0;
    cmd_status.nBytesToReceive = 1;
    cmd_status.nBitsToReceive = 0;
    cmd_status.Timeout = 1; /* 超时时间设置为1MS，与原厂DEMO保持一致 */
    cmd_status.Cmd = CMD_TRANSCEIVE;

    result = nfc_command_execute(&cmd_status);

    if ((result == FM175XX_SUCCESS) && (cmd_status.nBytesReceived == 1))
    {
        PICC_A.SAK[cascade_level] = inbuf[0];
        LOG_DBG("SAK%d: %02X", cascade_level, PICC_A.SAK[cascade_level]);
    }
    else
    {
        result = FM175XX_COMM_ERR;
    }

    return result;
}

/********************************************************************
**函数名称:  nfc_reader_a_halt
**入口参数:  无
**函数功能:  Type A休眠卡片
**返 回 值:  FM175XX_SUCCESS或错误码
*********************************************************************/
uint8_t nfc_reader_a_halt(void)
{
    struct command_struct cmd_status;
    uint8_t outbuf[2] = {0x50, 0x00};
    uint8_t inbuf[1];
    uint8_t result;

    cmd_status.SendCRCEnable = FM175XX_SET;
    cmd_status.ReceiveCRCEnable = FM175XX_SET;
    cmd_status.pSendBuf = outbuf;
    cmd_status.pReceiveBuf = inbuf;
    cmd_status.nBytesToSend = 2;
    cmd_status.nBitsToSend = 0;
    cmd_status.nBytesToReceive = 0;
    cmd_status.nBitsToReceive = 0;
    cmd_status.Timeout = 1; /* 超时时间设置为1MS，与原厂DEMO保持一致 */
    cmd_status.Cmd = CMD_TRANSCEIVE;

    result = nfc_command_execute(&cmd_status);

    /* HALT命令无响应表示成功 */
    if (result == FM175XX_TIMER_ERR)
    {
        result = FM175XX_SUCCESS;
    }

    return result;
}

/********************************************************************
**函数名称:  nfc_reader_a_card_activate
**入口参数:  无
**函数功能:  Type A激活卡片（完整流程）
**返 回 值:  FM175XX_SUCCESS或错误码
*********************************************************************/
uint8_t nfc_reader_a_card_activate(void)
{
    uint8_t result;
    uint8_t cascade_level;

    /* 唤醒卡片 */
    result = nfc_reader_a_wakeup();
    if (result != FM175XX_SUCCESS)
    {
        return result;
    }
    LOG_DBG("Wakeup OK, ATQA: %02X %02X", PICC_A.ATQA[0], PICC_A.ATQA[1]);

    /* 根据ATQA判断级联等级 */
    if ((PICC_A.ATQA[0] & 0xC0) == 0x00)
    {
        cascade_level = 1; /* 4字节UID */
    }
    else if ((PICC_A.ATQA[0] & 0xC0) == 0x40)
    {
        cascade_level = 2; /* 7字节UID */
    }
    else if ((PICC_A.ATQA[0] & 0xC0) == 0x80)
    {
        cascade_level = 3; /* 10字节UID */
    }
    else
    {
        LOG_WRN("Invalid ATQA: %02X %02X", PICC_A.ATQA[0], PICC_A.ATQA[1]);
        return FM175XX_PARAM_ERR;
    }

    LOG_DBG("Cascade level: %d", cascade_level);

    /* 防冲突和选择流程 */
    for (PICC_A.CASCADE_LEVEL = 0; PICC_A.CASCADE_LEVEL < cascade_level; PICC_A.CASCADE_LEVEL++)
    {
        result = nfc_reader_a_anticoll(PICC_A.CASCADE_LEVEL);
        if (result != FM175XX_SUCCESS)
        {
            return result;
        }

        result = nfc_reader_a_select(PICC_A.CASCADE_LEVEL);
        if (result != FM175XX_SUCCESS)
        {
            return result;
        }
    }

    /* 重组UID: 处理级联标志0x88
     * 4字节UID: 直接返回 UID[0-3]
     * 7字节UID: 第一级包含0x88，需要重组为 UID[1-3] + UID[4-6]
     * 10字节UID: 类似处理
     */
    if (cascade_level == 2 && PICC_A.UID[0] == 0x88)
    {
        /* 7字节UID: 移除级联标志，左移重组
         * 原: 88 XX YY ZZ | AA BB CC DD
         * 后: XX YY ZZ AA BB CC DD
         */
        PICC_A.UID[0] = PICC_A.UID[1];
        PICC_A.UID[1] = PICC_A.UID[2];
        PICC_A.UID[2] = PICC_A.UID[3];
        PICC_A.UID[3] = PICC_A.UID[4];
        PICC_A.UID[4] = PICC_A.UID[5];
        PICC_A.UID[5] = PICC_A.UID[6];
        PICC_A.UID[6] = PICC_A.UID[7];
        LOG_DBG("7-byte UID reassembled, CT removed");
    }
    else if (cascade_level == 3)
    {
        /* 10字节UID: 需要处理两个级联标志 */
        if (PICC_A.UID[0] == 0x88)
        {
            /* 第一级有级联标志 */
            memmove(&PICC_A.UID[0], &PICC_A.UID[1], 11); /* 左移1字节 */
            if (PICC_A.UID[3] == 0x88)
            {
                /* 第二级也有级联标志 */
                memmove(&PICC_A.UID[3], &PICC_A.UID[4], 7); /* 再左移1字节 */
            }
            LOG_DBG("10-byte UID reassembled, CT removed");
        }
    }

    #if 0
    /* 打印UID */
    LOG_INF("UID: ");
    for (uint8_t i = 0; i < (cascade_level * 4); i++)
    {
        LOG_INF("%02X ", PICC_A.UID[i]);
    }
    #endif

    return FM175XX_SUCCESS;
}

/********************************************************************
**函数名称:  Type_A_App
**入口参数:  无
**函数功能:  Type A卡片应用主循环（兼容原厂DEMO）
**返 回 值:  FM175XX_SUCCESS或错误码
*********************************************************************/
uint8_t Type_A_App(void)
{
    uint8_t result;

    /* 执行硬件复位 */
    fm175xx_npd_reset();

    /* 初始化读卡器 */
    nfc_init_reader_a();

    /* 启动载波 */
    nfc_set_cw(CW_ENABLE);

    /* 激活卡片 */
    result = nfc_reader_a_card_activate();

    nfc_set_cw(CW_DISABLE); /* 关闭载波 */

    return result;
}
