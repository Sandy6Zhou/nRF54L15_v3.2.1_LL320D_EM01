/********************************************************************
**版权所有         上海复旦微电子集团股份有限公司
**文件名称:        fm175xx_reg.h
**文件描述:        FM175xx 寄存器定义头文件
**当前版本:        V1.0
**作    者:        移植自复旦微官方驱动
**完成日期:        2026.01.22
*********************************************************************
** 功能描述:        定义FM17550/FM17622芯片的寄存器地址和位掩码
*********************************************************************/

#ifndef _FM175XX_REG_H_
#define _FM175XX_REG_H_

/* Page 0 寄存器定义 */
#define JREG_PAGE0              0x00    /* Page寄存器 */
#define JREG_COMMAND            0x01    /* 命令寄存器 */
#define JREG_COMMIEN            0x02    /* 通信中断使能 */
#define JREG_DIVIEN             0x03    /* 分频中断使能 */
#define JREG_COMMIRQ            0x04    /* 通信中断请求 */
#define JREG_DIVIRQ             0x05    /* 分频中断请求 */
#define JREG_ERROR              0x06    /* 错误标志 */
#define JREG_STATUS1            0x07    /* 状态寄存器1 */
#define JREG_STATUS2            0x08    /* 状态寄存器2 */
#define JREG_FIFODATA           0x09    /* FIFO数据 */
#define JREG_FIFOLEVEL          0x0A    /* FIFO级别 */
#define JREG_WATERLEVEL         0x0B    /* 水位 */
#define JREG_CONTROL            0x0C    /* 控制寄存器 */
#define JREG_BITFRAMING         0x0D    /* 位帧 */
#define JREG_COLL               0x0E    /* 冲突 */
#define JREG_RFU0F              0x0F    /* 保留 */

/* Page 1 寄存器定义 */
#define JREG_PAGE1              0x10    /* Page寄存器 */
#define JREG_MODE               0x11    /* 模式 */
#define JREG_TXMODE             0x12    /* 发送模式 */
#define JREG_RXMODE             0x13    /* 接收模式 */
#define JREG_TXCONTROL          0x14    /* 发送控制 */
#define JREG_TXAUTO             0x15    /* 自动发送 */
#define JREG_TXSEL              0x16    /* 发送选择 */
#define JREG_RXSEL              0x17    /* 接收选择 */
#define JREG_RXTHRESHOLD        0x18    /* 接收阈值 */
#define JREG_DEMOD              0x19    /* 解调 */
#define JREG_FELICANFC          0x1A    /* FeliCa NFC */
#define JREG_FELICANFC2         0x1B    /* FeliCa NFC2 */
#define JREG_MIFARE             0x1C    /* Mifare */
#define JREG_MANUALRCV          0x1D    /* 手动接收 */
#define JREG_TYPEB              0x1E    /* Type B */
#define JREG_SERIALSPEED        0x1F    /* 串口速度 */

/* Page 2 寄存器定义 */
#define JREG_PAGE2              0x20    /* Page寄存器 */
#define JREG_CRCRESULT1         0x21    /* CRC结果高字节 */
#define JREG_CRCRESULT2         0x22    /* CRC结果低字节 */
#define JREG_GSNLOADMOD         0x23    /* GSN负载调制 */
#define JREG_MODWIDTH           0x24    /* 调制宽度 */
#define JREG_TXBITPHASE         0x25    /* 发送位相位 */
#define JREG_RFCFG              0x26    /* RF配置 */
#define JREG_GSN                0x27    /* GSN */
#define JREG_CWGSP              0x28    /* CWGSP */
#define JREG_MODGSP             0x29    /* MODGSP */
#define JREG_TMODE              0x2A    /* 定时器模式 */
#define JREG_TPRESCALER         0x2B    /* 定时器预分频 */
#define JREG_TRELOADHI          0x2C    /* 定时器重载高字节 */
#define JREG_TRELOADLO          0x2D    /* 定时器重载低字节 */
#define JREG_TCOUNTERVALHI      0x2E    /* 定时器计数高字节 */
#define JREG_TCOUNTERVALLO      0x2F    /* 定时器计数低字节 */

/* Page 3 寄存器定义 */
#define JREG_PAGE3              0x30    /* Page寄存器 */
#define JREG_TESTSEL1           0x31    /* 测试选择1 */
#define JREG_TESTSEL2           0x32    /* 测试选择2 */
#define JREG_TESTPINEN          0x33    /* 测试引脚使能 */
#define JREG_TESTPINVALUE       0x34    /* 测试引脚值 */
#define JREG_TESTBUS            0x35    /* 测试总线 */
#define JREG_AUTOTEST           0x36    /* 自动测试 */
#define JREG_VERSION            0x37    /* 版本 */
#define JREG_ANALOGTEST         0x38    /* 模拟测试 */
#define JREG_TESTDAC1           0x39    /* 测试DAC1 */
#define JREG_TESTDAC2           0x3A    /* 测试DAC2 */
#define JREG_TESTADC            0x3B    /* 测试ADC */
#define JREG_ANALOGUETEST1      0x3C    /* 模拟测试1 */
#define JREG_ANALOGUETEST0      0x3D    /* 模拟测试0 */
#define JREG_ANALOGUETPD_A      0x3E    /* 模拟测试TPD_A */
#define JREG_ANALOGUETPD_B      0x3F    /* 模拟测试TPD_B */

/* 命令定义 */
#define CMD_IDLE                0x00    /* 空闲 */
#define CMD_CONFIGURE           0x01    /* 配置 */
#define CMD_GEN_RAND_ID         0x02    /* 生成随机ID */
#define CMD_CALC_CRC            0x03    /* 计算CRC */
#define CMD_TRANSMIT            0x04    /* 发送 */
#define CMD_NOCMDCHANGE         0x07    /* 不改变命令 */
#define CMD_RECEIVE             0x08    /* 接收 */
#define CMD_TRANSCEIVE          0x0C    /* 收发 */
#define CMD_AUTOCOLL            0x0D    /* 自动防冲突 */
#define CMD_AUTHENT             0x0E    /* 认证 */
#define CMD_SOFT_RESET          0x0F    /* 软复位 */
#define CMD_MASK                0xF0    /* 命令掩码 */

/* 位定义 - Page 0 */
/* Command Register (0x01) */
#define JBIT_RCVOFF             0x20    /* 关闭接收 */
#define JBIT_POWERDOWN          0x10    /* 掉电模式 */

/* CommIEn Register (0x02) */
#define JBIT_IRQINV             0x80    /* 中断反转 */

/* DivIEn Register (0x03) */
#define JBIT_IRQPUSHPULL        0x80    /* 中断推挽模式 */

/* CommIEn and CommIrq Register (0x02, 0x04) */
#define JBIT_TXI                0x40    /* 发送中断 */
#define JBIT_RXI                0x20    /* 接收中断 */
#define JBIT_IDLEI              0x10    /* 空闲中断 */
#define JBIT_HIALERTI           0x08    /* 高告警中断 */
#define JBIT_LOALERTI           0x04    /* 低告警中断 */
#define JBIT_ERRI               0x02    /* 错误中断 */
#define JBIT_TIMERI             0x01    /* 定时器中断 */

/* DivIEn and DivIrq Register (0x03, 0x05) */
#define JBIT_SIGINACT           0x10    /* Sigin激活中断 */
#define JBIT_MODEI              0x08    /* 模式中断 */
#define JBIT_CRCI               0x04    /* CRC中断 */
#define JBIT_RFONI              0x02    /* RF开启中断 */
#define JBIT_RFOFFI             0x01    /* RF关闭中断 */

/* CommIrq and DivIrq Register (0x04, 0x05) */
#define JBIT_SET                0x80    /* 设置/清除中断位 */

/* Error Register (0x06) */
#define JBIT_WRERR              0x40    /* 写访问错误 */
#define JBIT_TEMPERR            0x40    /* 温度错误 */
#define JBIT_RFERR              0x20    /* RF错误 */
#define JBIT_BUFFEROVFL         0x10    /* 缓冲区溢出 */
#define JBIT_COLLERR            0x08    /* 冲突错误 */
#define JBIT_CRCERR             0x04    /* CRC错误 */
#define JBIT_PARITYERR          0x02    /* 奇偶校验错误 */
#define JBIT_PROTERR            0x01    /* 协议错误 */

/* Status 1 Register (0x07) */
#define JBIT_CRCOK              0x40    /* CRC正常 */
#define JBIT_CRCREADY           0x20    /* CRC就绪 */
#define JBIT_IRQ                0x10    /* 中断激活 */
#define JBIT_TRUNNUNG           0x08    /* 定时器运行 */
#define JBIT_RFON               0x04    /* RF开启 */
#define JBIT_HIALERT            0x02    /* 高告警 */
#define JBIT_LOALERT            0x01    /* 低告警 */

/* Status 2 Register (0x08) */
#define JBIT_TEMPSENSOFF        0x80    /* 温度传感器关闭 */
#define JBIT_I2CFORCEHS         0x40    /* I2C强制高速 */
#define JBIT_MFSELECTED         0x10    /* Mifare已选择 */
#define JBIT_CRYPTO1ON          0x08    /* Crypto1开启 */

/* FIFOLevel Register (0x0A) */
#define JBIT_FLUSHFIFO          0x80    /* 清空FIFO */

/* Control Register (0x0C) */
#define JBIT_TSTOPNOW           0x80    /* 立即停止定时器 */
#define JBIT_TSTARTNOW          0x40    /* 立即启动定时器 */
#define JBIT_WRNFCIDTOFIFO      0x20    /* NFCID写入FIFO */
#define JBIT_INITIATOR          0x10    /* 发起者模式 */

/* BitFraming Register (0x0D) */
#define JBIT_STARTSEND          0x80    /* 开始发送 */

/* Coll Register (0x0E) */
#define JBIT_VALUESAFTERCOLL    0x80    /* 冲突后保留数据 */

/* 位定义 - Page 1 */
/* Mode Register (0x11) */
#define JBIT_MSBFIRST           0x80    /* MSB优先 */
#define JBIT_DETECTSYNC         0x40    /* 检测同步 */
#define JBIT_TXWAITRF           0x20    /* 发送等待RF */
#define JBIT_RXWAITRF           0x10    /* 接收等待RF */
#define JBIT_POLSIGIN           0x08    /* Sigin极性 */
#define JBIT_MODEDETOFF         0x04    /* 关闭模式检测 */

/* TxMode Register (0x12) */
#define JBIT_TXCRCEN            0x80    /* TX CRC使能 */
#define JBIT_INVMOD             0x08    /* 反向调制 */
#define JBIT_TXMIX              0x04    /* TX混合 */

/* RxMode Register (0x13) */
#define JBIT_RXCRCEN            0x80    /* RX CRC使能 */
#define JBIT_RXNOERR            0x08    /* 接收无错误 */
#define JBIT_RXMULTIPLE         0x04    /* 多接收模式 */

/* Tx and Rx Speed (0x12, 0x13) */
#define JBIT_106KBPS            0x00    /* 106kbps */
#define JBIT_212KBPS            0x10    /* 212kbps */
#define JBIT_424KBPS            0x20    /* 424kbps */
#define JBIT_848KBPS            0x30    /* 848kbps */
#define JBIT_1_6MBPS            0x40    /* 1.6Mbps */
#define JBIT_3_2MBPS            0x50    /* 3.2Mbps */

#define JBIT_MIFARE             0x00    /* Mifare模式 */
#define JBIT_NFC                0x01    /* NFC模式 */
#define JBIT_FELICA             0x02    /* FeliCa模式 */

/* TxControl Register (0x14) */
#define JBIT_INVTX2ON           0x80    /* 反转TX2开启 */
#define JBIT_INVTX1ON           0x40    /* 反转TX1开启 */
#define JBIT_INVTX2OFF          0x20    /* 反转TX2关闭 */
#define JBIT_INVTX1OFF          0x10    /* 反转TX1关闭 */
#define JBIT_TX2CW              0x08    /* TX2常波 */
#define JBIT_CHECKRF            0x04    /* 检查RF */
#define JBIT_TX2RFEN            0x02    /* TX2 RF使能 */
#define JBIT_TX1RFEN            0x01    /* TX1 RF使能 */

/* TxAuto Register (0x15) */
#define JBIT_AUTORFOFF          0x80    /* 自动关闭RF */
#define JBIT_FORCE100ASK        0x40    /* 强制100% ASK */
#define JBIT_AUTOWAKEUP         0x20    /* 自动唤醒 */
#define JBIT_CAON               0x08    /* 冲突避免开启 */
#define JBIT_INITIALRFON        0x04    /* 初始RF开启 */
#define JBIT_TX2RFAUTOEN        0x02    /* TX2自动使能 */
#define JBIT_TX1RFAUTOEN        0x01    /* TX1自动使能 */

/* Demod Register (0x19) */
#define JBIT_FIXIQ              0x20    /* 固定IQ */

/* FeliCa/NFC2 Register (0x1B) */
#define JBIT_WAITFORSELECTED    0x80    /* 等待选择 */
#define JBIT_FASTTIMESLOT       0x40    /* 快速时隙 */

/* Mifare Register (0x1C) */
#define JBIT_MFHALTED           0x04    /* Mifare暂停 */

/* RFU 0x1D Register */
#define JBIT_PARITYDISABLE      0x10    /* 禁用奇偶校验 */
#define JBIT_LARGEBWPLL         0x08    /* 大带宽PLL */
#define JBIT_MANUALHPCF         0x04    /* 手动HPCF */

/* 位定义 - Page 2 */
/* TxBitPhase Register (0x25) */
#define JBIT_RCVCLKCHANGE       0x80    /* 接收时钟改变 */

/* RfCFG Register (0x26) */
#define JBIT_RFLEVELAMP         0x80    /* RF电平放大 */

/* TMode Register (0x2A) */
#define JBIT_TAUTO              0x80    /* 定时器自动模式 */
#define JBIT_TAUTORESTART       0x10    /* 定时器自动重启 */
#define JBIT_MASK_TPRESCALER_HI 0x0F    /* 预分频高4位掩码 */

/* 位定义 - Page 3 */
/* AutoTest Register (0x36) */
#define JBIT_AMPRCV             0x40    /* 放大接收 */

/* RF命令定义 */
#define RF_CMD_REQA             0x26    /* REQA命令 */
#define RF_CMD_WUPA             0x52    /* WUPA命令 */

/* 位掩码定义 */
/* Command register (0x01) */
#define JMASK_COMMAND           0x0F    /* 命令位掩码 */
#define JMASK_COMMAND_INV       0xF0    /* 命令位掩码取反 */

/* Waterlevel register (0x0B) */
#define JMASK_WATERLEVEL        0x3F    /* 水位掩码 */

/* Control register (0x0C) */
#define JMASK_RXBITS            0x07    /* 接收位掩码 */

/* Mode register (0x11) */
#define JMASK_CRCPRESET         0x03    /* CRC预设掩码 */

/* TxMode/RxMode register (0x12, 0x13) */
#define JMASK_SPEED             0x70    /* 速度掩码 */
#define JMASK_FRAMING           0x03    /* 帧格式掩码 */

/* TxSel register (0x16) */
#define JMASK_LOADMODSEL        0xC0    /* 负载调制选择掩码 */
#define JMASK_DRIVERSEL         0x30    /* 驱动选择掩码 */
#define JMASK_SIGOUTSEL         0x0F    /* 信号输出选择掩码 */

/* RxSel register (0x17) */
#define JMASK_UARTSEL           0xC0    /* UART选择掩码 */
#define JMASK_RXWAIT            0x3F    /* 接收等待掩码 */

/* RxThreshold register (0x18) */
#define JMASK_MINLEVEL          0xF0    /* 最小电平掩码 */
#define JMASK_COLLEVEL          0x07    /* 冲突电平掩码 */

/* Demod register (0x19) */
#define JMASK_ADDIQ             0xC0    /* ADDIQ掩码 */
#define JMASK_TAURCV            0x0C    /* Tau接收掩码 */
#define JMASK_TAUSYNC           0x03    /* Tau同步掩码 */

/* FeliCa/FeliCa2 register (0x1A, 0x1B) */
#define JMASK_FELICASYNCLEN     0xC0    /* FeliCa同步长度掩码 */
#define JMASK_FELICALEN         0x3F    /* FeliCa长度掩码 */

/* Mifare register (0x1C) */
#define JMASK_SENSMILLER        0xE0    /* SensMiller掩码 */
#define JMASK_TAUMILLER         0x18    /* TauMiller掩码 */
#define JMASK_TXWAIT            0x03    /* 发送等待掩码 */

/* Manual Rcv register (0x1D) */
#define JMASK_HPCF              0x03    /* HPCF掩码 */

/* TxBitPhase register (0x25) */
#define JMASK_TXBITPHASE        0x7F    /* 发送位相位掩码 */

/* RFCfg register (0x26) */
#define JMASK_RXGAIN            0x70    /* 接收增益掩码 */
#define JMASK_RFLEVEL           0x0F    /* RF电平掩码 */

/* GsN register (0x27) */
#define JMASK_CWGSN             0xF0    /* CWGsN掩码 */
#define JMASK_MODGSN            0x0F    /* ModGsN掩码 */

/* CWGsP register (0x28) */
#define JMASK_CWGSP             0x3F    /* CWGsP掩码 */

/* ModGsP register (0x29) */
#define JMASK_MODGSP            0x3F    /* ModGsP掩码 */

/* TMode register (0x2A) */
#define JMASK_TGATED            0x60    /* 定时器门控掩码 */
#define JMASK_TPRESCALER_HI     0x0F    /* 预分频高4位掩码 */

/* 载波控制 */
#define CW_DISABLE              0
#define CW1_ENABLE              1
#define CW2_ENABLE              2
#define CW_ENABLE               3

/* 通信速率 */
#define MODWIDTH_106            0x26    /* 106Kbps */
#define MODWIDTH_212            0x13    /* 212Kbps */
#define MODWIDTH_424            0x09    /* 424Kbps */
#define MODWIDTH_848            0x04    /* 848Kbps */

/* 错误码 */
#define FM175XX_SUCCESS         0x00
#define FM175XX_TIMER_ERR       0xF1    /* 接收超时 */
#define FM175XX_COMM_ERR        0xF2    /* 通信错误 */
#define FM175XX_COLL_ERR        0xF3    /* 冲突错误 */
#define FM175XX_PARAM_ERR       0xF4    /* 参数错误 */
#define FM175XX_LENGTH_ERR      0xF5    /* 长度错误 */
#define FM175XX_AUTH_ERR        0xF6    /* 认证错误 */

/* 布尔值定义 */
#define FM175XX_SET             1
#define FM175XX_RESET           0
#define FM175XX_ENABLE          1
#define FM175XX_DISABLE         0

#endif /* _FM175XX_REG_H_ */
