/**
 * @file    my_ymodem.c
 * @brief   Ymodem/Xmodem 协议实现（单线程、回调驱动、高度可移植）
 *
 * @details 本模块实现了完整的 Ymodem/Xmodem 文件传输协议，包括:
 *          - Ymodem (CRC16, 1024 字节帧, 支持文件信息帧)
 *          - Xmodem-CRC (128 字节帧, CRC16 校验)
 *          - Xmodem-1k (1024 字节帧, CRC16 校验)
 *          - Xmodem-checksum (128 字节帧, 8 位校验和)
 *
 *          发送端支持从文件系统 (fread) 或内存缓冲区直接发送。
 *          接收端自动检测协议类型和校验方式，可保存到文件或内存。
 *
 *          I/O 抽象: 通过 my_ymodem_io_t 注册 send/recv/flush
 *          三个硬件原语，栈内部自动完成超时控制和缓冲管理，
 *          无需用户编写任何状态机代码。
 *
 * 依赖:  stdint.h, stdlib.h, string.h, stdio.h (均为标准 C89+)
 * 移植:  实现 my_ymodem_io_t::{send, recv, flush} 并调用 init() 即可
 */

#include "my_ymodem.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/*===========================================================================
 * 内部常量
 *===========================================================================*/

/** 帧结构偏移量 */
#define FRAME_HEAD_OFF   0           /* 帧头标识 (SOH/STX/EOT) */
#define FRAME_SEQ_OFF    1           /* 帧序号 */
#define FRAME_SEQN_OFF   2           /* 帧序号反码 (~seq) */
#define FRAME_DATA_OFF   3           /* 数据区起始偏移 */

/** 帧缓冲区最大大小: 头(3) + 数据(1024) + CRC(2) + 余量 */
#define FRAME_BUF_SIZE   1032

/** 文件路径最大长度 */
#define MAX_PATH_LEN      256
/** 文件信息帧中文件名的最大长度 (128 字节数据区需容纳文件名 + 大小) */
#define INFO_MAX_NAME_LEN 100

/*===========================================================================
 * 状态机定义
 *
 * 接收端状态转换:
 *   CONNECTING -> CONNECTED -> FILEXFER -> ENDOFXFER -> ENDING -> ENDED
 *
 * 发送端状态转换:
 *   CONNECTING -> CONNECTED -> FILEXFER -> ENDOFXFER -> ENDED
 *===========================================================================*/
enum {
    STATE_CONNECTING,       /* 握手阶段: 等待/发送 'C' 或 NAK */
    STATE_CONNECTED,        /* 连接建立: 准备传输文件数据 */
    STATE_FILEXFER,         /* 数据传输: 收/发文件数据 */
    STATE_ENDOFXFER,        /* 传输收尾: 等待第二个 EOT (Ymodem) */
    STATE_ENDING,           /* 会话结束: 发送/接收结束帧 */
    STATE_ENDED             /* 传输完成: 会话正常终止 */
};

/*===========================================================================
 * Ymodem 句柄结构体
 *===========================================================================*/
/* 字段分组:
 *   - 状态:   state / crc_type / is_ymodem / xmodem_1k
 *   - 数据:   file_name / save_dir / file_size / data_src / data_dst /
 *             out_buf / out_buf_size / fp / mem_buf / mem_offset /
 *             written
 *   - 帧控制: expected_seq / frame */
struct my_ymodem {
    my_ymodem_io_t *io;             /* 用户提供的平台配置 (指针) */

    /* ---- 状态机 ---- */
    int state;                      /* 当前状态 (STATE_xxx) */
    int crc_type;                   /* 校验方式: 1=checksum, 2=CRC16 */
    int is_ymodem;                  /* 协议模式 (由接收端自动检测) */
    int xmodem_1k;                  /* Xmodem-1k 标志 (发送端使用) */

    /* ---- 文件 / 数据源 ---- */
    char file_name[MAX_PATH_LEN];   /* 传输的文件名 */
    char save_dir[MAX_PATH_LEN];    /* 接收文件存放目录 */
    int  file_size;                 /* 文件总大小 (字节), -1=未知 */

    int  data_src;                  /* 数据来源: 0=文件, 1=内存 */
    int  data_dst;                  /* 数据去向: 0=文件, 1=内存 */
    uint8_t *out_buf;               /* 内存接收缓冲区 (data_dst==1 时有效) */
    int  out_buf_size;              /* 接收缓冲区总大小 */
    FILE *fp;                       /* 文件指针 (data_src==0 / data_dst==0 时有效) */
    const uint8_t *mem_buf;         /* 内存缓冲区指针 (data_src==1 时有效) */
    int  mem_offset;                /* 已从 mem_buf 读取的字节偏移 */

    int  written;                   /* 已写入/接收的文件数据字节数 */
    uint8_t expected_seq;           /* 期望的下一帧序号 (重复/乱序帧检测) */

    /* ---- 帧组装 ---- */
    uint8_t frame[FRAME_BUF_SIZE];  /* 帧数据工作区 */
};

/*===========================================================================
 * CRC16 冗余校验计算
 *
 * 提供两种 CRC16 实现:
 *   - 查表法 (MY_YMODEM_USE_CRC_TABLE 已定义): 速度较快
 *   - 位运算法 (默认): 代码体积较小
 *
 * CRC16 多项式: X^16 + X^12 + X^5 + 1 (0x1021)
 *===========================================================================*/

#ifdef MY_YMODEM_USE_CRC_TABLE

/** CRC16 查找表 (256 项，预计算) */
static const uint16_t crc16_table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

/**
 * @brief  查表法计算 CRC16-CCITT。
 * @param  buf  数据缓冲区
 * @param  len  数据长度
 * @return 16 位 CRC 值
 */
static uint16_t calc_crc16(const uint8_t *buf, int len)
{
    uint16_t crc = 0;
    while (len-- > 0)
    {
        crc = (uint16_t)((crc << 8) ^ crc16_table[((crc >> 8) ^ *buf++) & 0xFF]);
    }
    return crc;
}

#else

/**
 * @brief  位运算法计算 CRC16-CCITT。
 * @param  buf  数据缓冲区
 * @param  len  数据长度
 * @return 16 位 CRC 值
 */
static uint16_t calc_crc16(const uint8_t *buf, int len)
{
    uint16_t crc = 0;
    while (--len >= 0)
    {
        crc = (uint16_t)(crc ^ ((uint16_t)*buf++ << 8));
        for (int i = 0; i < 8; i++)
        {
            if (crc & 0x8000)
            {
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            }
            else
            {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

#endif

/**
 * @brief  计算 8 位校验和（Xmodem 旧校验模式）。
 * @param  buf  数据缓冲区
 * @param  len  数据长度
 * @return 8 位累加校验和
 */
static uint8_t calc_checksum(const uint8_t *buf, int len)
{
    uint8_t sum = 0;
    for (int i = 0; i < len; i++)
    {
        sum = (uint8_t)(sum + buf[i]);
    }
    return sum;
}

/*===========================================================================
 * 配置读取辅助函数 — 从 io 结构体读取配置，0 值使用默认值
 *===========================================================================*/

/**
 * @brief  获取握手重试次数。
 */
static int get_handshake_retry(my_ymodem_t *ym)
{
    return ym->io->handshake_retry > 0
           ? ym->io->handshake_retry
           : MY_YMODEM_DEFAULT_HANDSHAKE_RETRY;
}

/**
 * @brief  获取数据帧发送重试次数。
 */
static int get_send_retry(my_ymodem_t *ym)
{
    return ym->io->send_retry > 0
           ? ym->io->send_retry
           : MY_YMODEM_DEFAULT_SEND_RETRY;
}

/**
 * @brief  获取单字节接收超时 (ms)。
 */
static int get_recv_timeout(my_ymodem_t *ym)
{
    return ym->io->recv_timeout_ms > 0
           ? ym->io->recv_timeout_ms
           : MY_YMODEM_DEFAULT_RECV_TIMEOUT_MS;
}

#if MY_YMODEM_ENABLE_SEND
/**
 * @brief  根据 io.protocol 解析协议参数，映射到句柄内部字段。
 *
 * @details 协议优先级: io.protocol > 0 时忽略函数参数，以配置为准;
 *          io.protocol == AUTO (0) 时沿用函数参数 (兼容旧行为)。
 *
 * @param  ym         Ymodem 句柄
 * @param  is_ymodem  函数参数: 1=Ymodem, 0=Xmodem
 * @param  xmodem_1k  函数参数: 1=Xmodem-1k, 0=Xmodem (128B)
 */
static void sender_apply_protocol(my_ymodem_t *ym,
                                  int is_ymodem, int xmodem_1k)
{
    int proto = ym->io->protocol;

    if (proto == MY_YMODEM_PROTO_YMODE)
    {
        ym->is_ymodem = 1;
        ym->xmodem_1k = 0;
    }
    else if (proto == MY_YMODEM_PROTO_XMODEM)
    {
        ym->is_ymodem = 0;
        ym->xmodem_1k = 0;
    }
    else if (proto == MY_YMODEM_PROTO_XMODEM_1K)
    {
        ym->is_ymodem = 0;
        ym->xmodem_1k = 1;
    }
    else
    {
        /* AUTO: 沿用函数参数 */
        ym->is_ymodem = is_ymodem;
        ym->xmodem_1k = xmodem_1k;
    }
}
#endif /* MY_YMODEM_ENABLE_SEND */

/*===========================================================================
 * 内部 I/O 原语 — 基于 io 回调统一封装
 *
 * 栈内部始终通过以下函数进行 I/O 操作，不直接调用 io->send/recv。
 *===========================================================================*/

#if MY_YMODEM_ENABLE_SEND
/**
 * @brief  发送 len 字节，循环调用 io->send 直到全部发送完成。
 * @return 实际发送字节数, -1 表示错误
 */
static int send_data(my_ymodem_t *ym, const uint8_t *data, int len)
{
    int sent = 0;
    while (sent < len)
    {
        int n = ym->io->send(data + sent, len - sent);
        if (n <= 0)
        {
            return -1;
        }
        sent += n;
    }
    return sent;
}
#endif /* MY_YMODEM_ENABLE_SEND */

#if MY_YMODEM_ENABLE_RECV
/**
 * @brief  接收数据，带超时
 *
 * @details 循环调用 io->recv() 直到收满 max_len 字节或超时。
 *          io->recv 内部已实现超时等待（滑动空闲超时），
 *          此处不再自行记账，避免双重超时导致等待时间放大。
 *
 * @return >0: 实际接收字节数; 0: 超时; <0: 错误
 */
static int recv_data(my_ymodem_t *ym, uint8_t *buf, int max_len,
                     int timeout_ms)
{
    int total = 0;

    while (total < max_len)
    {
        int n = ym->io->recv(buf + total, max_len - total, timeout_ms);
        if (n < 0)
        {
            return -1;          /* 硬件错误 */
        }
        if (n == 0)
        {
            break;              /* 超时: 返回已接收的数据 */
        }
        total += n;
    }
    return total;
}
#endif /* MY_YMODEM_ENABLE_RECV */

/**
 * @brief  发送单个字节。
 */
static int send_byte(my_ymodem_t *ym, uint8_t c)
{
    return ym->io->send(&c, 1);
}

#if MY_YMODEM_ENABLE_SEND
/**
 * @brief  接收单个字节，含超时。
 * @return 1 成功, 0 超时, -1 错误
 */
static int recv_byte(my_ymodem_t *ym, uint8_t *c, int timeout_ms)
{
    int n = ym->io->recv(c, 1, timeout_ms);
    if (n > 0)
    {
        return 1;
    }
    if (n == 0)
    {
        return 0;
    }
    return -1;
}
#endif /* MY_YMODEM_ENABLE_SEND */

/**
 * @brief  清空接收缓冲区 — 优先使用 io->flush，否则用 recv() 排空。
 */
static void port_flush(my_ymodem_t *ym)
{
    if (ym->io->flush)
    {
        ym->io->flush();
    }
    else
    {
        uint8_t dummy[64];
        while (ym->io->recv(dummy, sizeof(dummy), 0) > 0)
        {
            /* 排空缓冲区 */
        }
    }
}

/*===========================================================================
 * 路径/文件名辅助函数
 *===========================================================================*/

#if MY_YMODEM_ENABLE_SEND
/**
 * @brief  从完整路径中提取文件名。
 *
 * @details 反向扫描路径字符串，找到最后一个 '/' 或 '\\'
 *          作为文件名起始位置。若未找到分隔符则将整个路径
 *          作为文件名。
 *
 * @param  path     完整文件路径
 * @param  name     输出: 文件名缓冲区
 * @param  max_len  缓冲区最大长度
 */
static void extract_file_name(const char *path, char *name, int max_len)
{
    const char *last = path;
    for (const char *p = path; *p; p++)
    {
        if (*p == '/' || *p == '\\')
        {
            last = p + 1;
        }
    }
    if (*last == '\0')
    {
        last = path;  /* 路径无分隔符，则整个字符串作为文件名 */
    }

    int i = 0;
    while (last[i] && i < max_len - 1)
    {
        name[i] = last[i];
        i++;
    }
    name[i] = '\0';
}
#endif /* MY_YMODEM_ENABLE_SEND */

/*===========================================================================
 * 接收端内部实现
 *
 * 协议流程:
 *   1. 接收端发送 'C' (CRC16) 或 NAK (checksum) 发起握手
 *   2. 发送端响应帧 (Ymodem: 文件信息帧; Xmodem: 数据帧)
 *   3. 逐帧接收数据，每帧校验通过后回复 ACK
 *   4. 收到 EOT 后完成传输
 *===========================================================================*/

#if MY_YMODEM_ENABLE_RECV
/**
 * @brief  接收端发送握手请求字符，返回对应的校验长度。
 *
 * @details 根据当前设置的校验类型发送对应字符:
 *          - checksum 模式: 发送 NAK，校验字段为 1 字节
 *          - CRC16 模式 (默认): 发送 'C'，CRC 占 2 字节
 *
 * @param  ym  Ymodem 句柄
 * @return 1 (checksum) 或 2 (CRC16)
 */
static int receiver_send_request(my_ymodem_t *ym)
{
    if (ym->crc_type == 1)
    {
        /* checksum 模式: 发送 NAK，校验占 1 字节 */
        send_byte(ym, MY_YMODEM_NAK);
        return 1;
    }
    else
    {
        /* CRC16 模式 (默认): 发送 'C'，CRC 占 2 字节 */
        send_byte(ym, MY_YMODEM_CRC16_C);
        return 2;
    }
}

/**
 * @brief  校验帧序号和数据完整性。
 *
 * @details 验证项目:
 *          1. 校验帧序号: seq + ~seq 应等于 0xFF
 *          2. 根据 crc_len 校验数据:
 *             - 1 字节: 使用 checksum 校验
 *             - 2 字节: 使用 CRC16 校验
 *
 * @param  ym        Ymodem 句柄
 * @param  data_size 数据区大小
 * @param  crc_len   校验字段长度 (1 或 2)
 * @return MY_YMODEM_OK 成功, MY_YMODEM_ERR_SEQ 序号错误,
 *         MY_YMODEM_ERR_CRC 校验错误
 */
static int receiver_verify_frame(my_ymodem_t *ym, int data_size, int crc_len)
{
    uint8_t *f = ym->frame;

    /* 校验帧序号: seq + ~seq == 0xFF */
    if ((uint8_t)(f[FRAME_SEQ_OFF] + f[FRAME_SEQN_OFF]) != 0xFF)
    {
        return MY_YMODEM_ERR_SEQ;
    }

    uint8_t *data = f + FRAME_DATA_OFF;
    int crc_off = FRAME_DATA_OFF + data_size;

    if (crc_len == 1)
    {
        /* 8 位校验和 */
        uint8_t rx = f[crc_off];
        uint8_t calc = calc_checksum(data, data_size);
        if (rx != calc)
        {
            return MY_YMODEM_ERR_CRC;
        }
    }
    else
    {
        /* CRC16 校验 (双字节，高字节在前) */
        uint16_t rx = (uint16_t)((f[crc_off] << 8) | f[crc_off + 1]);
        uint16_t calc = calc_crc16(data, data_size);
        if (rx != calc)
        {
            return MY_YMODEM_ERR_CRC;
        }
    }
    return MY_YMODEM_OK;
}

/**
 * @brief  解析 Ymodem 首帧中的文件信息。
 *
 * @details Ymodem 文件信息帧数据格式:
 *          文件名 + '\0' + 文件大小(ASCII 十进制字符串)
 *          示例: "ble_fota.bin\01024576"
 *
 * @param  ym   Ymodem 句柄
 * @param  buf  首帧数据区指针
 */
static void receiver_parse_file_info(my_ymodem_t *ym, uint8_t *buf)
{
    int i;
    /* 读取文件名 (以 '\0' 结尾) */
    for (i = 0; i < MY_YMODEM_SOH_SIZE - 1 && buf[i] != 0; i++)
    {
        ym->file_name[i] = (char)buf[i];
    }
    ym->file_name[i] = '\0';

    /* 文件大小在 '\0' 之后为 ASCII 十进制数字字符串 */
    ym->file_size = (int)strtol((const char *)(buf + i + 1), NULL, 10);
    if (ym->file_size == 0)
    {
        ym->file_size = -1;  /* 未知大小 */
    }
}

/**
 * @brief  打开接收文件准备写入。
 *
 * @details 内存模式 (data_dst==1): 无需打开文件，直接返回成功。
 *          文件模式 (data_dst==0): 根据 save_dir 拼接路径，
 *          以 "wb" 模式打开文件。
 *
 * @param  ym  Ymodem 句柄
 * @return MY_YMODEM_OK 成功, MY_YMODEM_ERR_FILE 失败
 */
static int receiver_open_file(my_ymodem_t *ym)
{
    /* 内存模式: 无需打开文件 */
    if (ym->data_dst == 1)
    {
        ym->written = 0;
        return MY_YMODEM_OK;
    }

    /* 平台回调模式: 优先使用 save_open 落盘到自定义目标(如 Flash) */
    if (ym->io->save_open)
    {
        if (ym->io->save_open(ym->file_name, ym->file_size) != 0)
        {
            return MY_YMODEM_ERR_FILE;
        }
        ym->written = 0;
        return MY_YMODEM_OK;
    }

#ifndef MY_YMODEM_NO_FILE_IO
    char full_path[2 * MAX_PATH_LEN];

    if (ym->save_dir[0])
    {
        snprintf(full_path, sizeof(full_path), "%s/%s",
                 ym->save_dir, ym->file_name);
    }
    else
    {
        snprintf(full_path, sizeof(full_path), "%s", ym->file_name);
    }

    ym->fp = fopen(full_path, "wb");
    if (!ym->fp)
    {
        return MY_YMODEM_ERR_FILE;
    }

    ym->written = 0;
    return MY_YMODEM_OK;
#else
    /* 未提供 save_open 回调且无文件系统: 无法落盘 */
    return MY_YMODEM_ERR_FILE;
#endif
}

/**
 * @brief  关闭接收文件。
 *
 * @details 内存模式: 无需关闭文件。
 *          文件模式: 调用 fclose。
 *
 * @param  ym  Ymodem 句柄
 */
static void receiver_close_file(my_ymodem_t *ym)
{
    /* 内存模式: 无需关闭文件 */
    if (ym->data_dst == 1)
    {
        return;
    }

    /* 平台回调模式: 调用 save_close 收尾 (正常结束路径传 1) */
    if (ym->io->save_close)
    {
        ym->io->save_close(1);
        return;
    }

#ifndef MY_YMODEM_NO_FILE_IO
    if (ym->fp)
    {
        fclose(ym->fp);
        ym->fp = NULL;
    }
#endif
}

/**
 * @brief  保存接收到的一帧数据到目标 (文件或内存)。
 *
 * @details 内存模式: 检查缓冲区剩余空间，memcpy 到 out_buf。
 *          文件模式: fwrite 到文件。
 *          空间不足时发送 CAN 并返回 MY_YMODEM_ERR_FILE。
 *
 * @param  ym      Ymodem 句柄
 * @param  data    数据指针
 * @param  len     实际数据长度 (已去填充)
 * @return MY_YMODEM_OK 成功，否则返回错误码
 */
static int receiver_save_data(my_ymodem_t *ym, const uint8_t *data, int len)
{
    if (ym->data_dst == 1)
    {
        /* 内存模式: 检查缓冲剩余空间 */
        if (ym->written + len > ym->out_buf_size)
        {
            send_byte(ym, MY_YMODEM_CAN);
            return MY_YMODEM_ERR_FILE;
        }
        memcpy(ym->out_buf + ym->written, data, (size_t)len);
    }
    else if (ym->io->save_write)
    {
        /* 平台回调模式: 调用 save_write 写入目标 */
        if (ym->io->save_write(data, len) != 0)
        {
            send_byte(ym, MY_YMODEM_CAN);
            return MY_YMODEM_ERR_FILE;
        }
    }
#ifndef MY_YMODEM_NO_FILE_IO
    else
    {
        /* 文件模式: 写入文件 */
        if (fwrite(data, 1, (size_t)len, ym->fp) != (size_t)len)
        {
            send_byte(ym, MY_YMODEM_CAN);
            return MY_YMODEM_ERR_FILE;
        }
    }
#else
    else
    {
        /* 未提供 save_write 回调: 无法落盘 */
        send_byte(ym, MY_YMODEM_CAN);
        return MY_YMODEM_ERR_FILE;
    }
#endif
    ym->written += len;
    return MY_YMODEM_OK;
}

/**
 * @brief  接收端处理首帧（同时处理 Ymodem 文件信息帧和 Xmodem 数据帧）。
 *
 * @details 首帧处理逻辑:
 *          - 根据帧头 (SOH/STX) 确定数据区大小
 *          - 接收完整帧数据
 *          - 校验帧完整性
 *          - 若 seq == 0 且 ~seq == 0xFF: 判定为 Ymodem 文件信息帧
 *          - 否则: 判定为 Xmodem 数据帧，直接保存数据
 *          - 打开接收文件
 *
 * @param  ym      Ymodem 句柄
 * @param  crc_len 当前校验模式对应的校验字段长度
 * @return MY_YMODEM_OK 成功，否则返回错误码
 */
static int receiver_process_first_frame(my_ymodem_t *ym, int crc_len)
{
    uint8_t *frame = ym->frame;
    uint8_t head = frame[FRAME_HEAD_OFF];
    int data_size, frame_size, remain, recv_len;
    int recv_timeout = get_recv_timeout(ym);

    /* 根据帧头确定数据区大小 */
    if (head == MY_YMODEM_SOH)
    {
        data_size = MY_YMODEM_SOH_SIZE;     /* 128 字节 */
    }
    else if (head == MY_YMODEM_STX)
    {
        data_size = MY_YMODEM_STX_SIZE;     /* 1024 字节 */
    }
    else
    {
        return MY_YMODEM_ERR_HEAD;
    }

    /* 计算完整帧大小并接收剩余部分 */
    frame_size = FRAME_DATA_OFF + data_size + crc_len;
    remain = frame_size - 1;

    /* 已收到字节 0 (head)，读取帧剩余部分 */
    recv_len = recv_data(ym, frame + 1, remain, recv_timeout);
    if (recv_len < remain)
    {
        port_flush(ym);
        send_byte(ym, MY_YMODEM_NAK);
        return MY_YMODEM_ERR_SIZE;
    }

    /* 校验帧完整性 */
    int ret = receiver_verify_frame(ym, data_size, crc_len);
    if (ret)
    {
        port_flush(ym);
        send_byte(ym, MY_YMODEM_NAK);
        return ret;
    }

    /* Ymodem 判定: seq==0 且 ~seq==0xFF => 文件信息帧 */
    if (frame[FRAME_SEQ_OFF] == 0 && frame[FRAME_SEQN_OFF] == 0xFF)
    {
        ym->is_ymodem = 1;
        ym->expected_seq = 1;
        receiver_parse_file_info(ym, frame + FRAME_DATA_OFF);
        send_byte(ym, MY_YMODEM_ACK);
        receiver_send_request(ym);   /* 请求首帧数据 */
    }
    else
    {
        /* Xmodem 模式: 首帧即为数据帧 */
        ym->is_ymodem = 0;
        if (frame[FRAME_SEQ_OFF] != 1)
        {
            send_byte(ym, MY_YMODEM_NAK);
            return MY_YMODEM_ERR_SEQ;
        }
        ym->expected_seq = 2;
    }

    ym->state = STATE_CONNECTED;

    /* 打开接收文件 */
    if (receiver_open_file(ym) != MY_YMODEM_OK)
    {
        send_byte(ym, MY_YMODEM_CAN);
        return MY_YMODEM_ERR_FILE;
    }

    /* Xmodem: 首帧已携带数据 — 直接保存 */
    if (!ym->is_ymodem)
    {
        ret = receiver_save_data(ym, frame + FRAME_DATA_OFF, data_size);
        if (ret)
        {
            return ret;
        }
        send_byte(ym, MY_YMODEM_ACK);
    }

    return MY_YMODEM_OK;
}

/**
 * @brief  接收端处理常规帧（数据帧 / EOT / 结束帧）。
 *
 * @details 帧处理逻辑:
 *          - EOT (0x04): 传输结束信号 (Ymodem 需要两次 EOT)
 *          - SOH (0x01): 128 字节数据帧
 *          - STX (0x02): 1024 字节数据帧
 *          - Ymodem 结束帧: seq==0 且 data[0]==0x00
 *          - 数据帧: 写入文件/内存，最后一帧减去填充字节
 *
 * @param  ym      Ymodem 句柄
 * @param  crc_len 校验字段长度
 * @return MY_YMODEM_OK 成功，否则返回错误码
 */
static int receiver_process_frame(my_ymodem_t *ym, int crc_len)
{
    uint8_t *frame = ym->frame;
    uint8_t head;
    int data_size, frame_size, remain, recv_len, save_size;
    int ret;
    int recv_timeout = get_recv_timeout(ym);

    /* 读取帧头字节 */
    ret = recv_data(ym, frame, 1, recv_timeout);
    if (ret <= 0)
    {
        /* 帧头超时: 冲刷残留并请求重传 */
        port_flush(ym);
        send_byte(ym, MY_YMODEM_NAK);
        return MY_YMODEM_ERR_CRC;
    }

    head = frame[FRAME_HEAD_OFF];

    /* ---- EOT (传输结束) 处理 ---- */
    if (head == MY_YMODEM_EOT)
    {
        if (ym->is_ymodem)
        {
            /* Ymodem: 需要接收两个连续的 EOT */
            if (ym->state == STATE_FILEXFER)
            {
                ym->state = STATE_ENDOFXFER;
                send_byte(ym, MY_YMODEM_NAK);       /* 请求第二个 EOT */
            }
            else if (ym->state == STATE_ENDOFXFER)
            {
                /* 第二次 EOT: 应答 ACK 并请求结束帧 (标准: ACK + 'C') */
                ym->state = STATE_ENDING;
                send_byte(ym, MY_YMODEM_ACK);
                if (ym->crc_type == 1)
                {
                    send_byte(ym, MY_YMODEM_NAK);
                }
                else
                {
                    send_byte(ym, MY_YMODEM_CRC16_C);
                }
            }
            else if (ym->state == STATE_ENDING)
            {
                /* 多余 EOT: 忽略, 继续等待结束帧 */
                return MY_YMODEM_OK;
            }
            else
            {
                return MY_YMODEM_ERR_STATE;
            }
        }
        else
        {
            /* Xmodem: EOT 即表示传输完成 */
            ym->state = STATE_ENDED;
            send_byte(ym, MY_YMODEM_ACK);
            receiver_close_file(ym);
        }
        return MY_YMODEM_OK;
    }

    /* ---- 数据帧处理 ---- */
    if (head == MY_YMODEM_SOH)
    {
        data_size = MY_YMODEM_SOH_SIZE;
    }
    else if (head == MY_YMODEM_STX)
    {
        data_size = MY_YMODEM_STX_SIZE;
    }
    else
    {
        /* 无效的帧头 */
        send_byte(ym, MY_YMODEM_CAN);
        port_flush(ym);
        return MY_YMODEM_ERR_HEAD;
    }

    /* 接收帧剩余部分 */
    frame_size = FRAME_DATA_OFF + data_size + crc_len;
    remain = frame_size - 1;
    recv_len = recv_data(ym, frame + 1, remain, recv_timeout);
    if (recv_len < remain)
    {
        port_flush(ym);
        send_byte(ym, MY_YMODEM_NAK);
        return MY_YMODEM_ERR_SIZE;
    }

    /* 校验帧完整性 */
    if (receiver_verify_frame(ym, data_size, crc_len))
    {
        port_flush(ym);
        send_byte(ym, MY_YMODEM_NAK);
        return MY_YMODEM_ERR_CRC;
    }

    /* Ymodem seq==0 帧检测: 批量结束帧 或 新文件信息帧 */
    if (ym->is_ymodem && ym->state == STATE_ENDING
        && frame[FRAME_SEQ_OFF] == 0)
    {
        if (frame[FRAME_DATA_OFF] == 0)
        {
            /* 空结束帧: 整个会话结束 */
            ym->state = STATE_ENDED;
            send_byte(ym, MY_YMODEM_ACK);
            receiver_close_file(ym);
            return MY_YMODEM_OK;
        }
        else
        {
            /* 批传: 收到下一文件的文件信息帧 */
            receiver_close_file(ym);
            receiver_parse_file_info(ym, frame + FRAME_DATA_OFF);
            send_byte(ym, MY_YMODEM_ACK);
            receiver_send_request(ym);
            ym->expected_seq = 1;

            if (receiver_open_file(ym) != MY_YMODEM_OK)
            {
                send_byte(ym, MY_YMODEM_CAN);
                return MY_YMODEM_ERR_FILE;
            }
            ym->state = STATE_FILEXFER;
            return MY_YMODEM_OK;
        }
    }

    /* 帧序号连续性检查: 重复帧 ACK 丢弃, 乱序帧 NAK */
    if (frame[FRAME_SEQ_OFF] == ym->expected_seq)
    {
        ym->expected_seq = (uint8_t)(ym->expected_seq + 1);
    }
    else if (frame[FRAME_SEQ_OFF] == (uint8_t)(ym->expected_seq - 1))
    {
        /* 重复帧 (发送方未收到 ACK 重发): ACK 并丢弃 */
        send_byte(ym, MY_YMODEM_ACK);
        return MY_YMODEM_OK;
    }
    else
    {
        send_byte(ym, MY_YMODEM_NAK);
        return MY_YMODEM_ERR_SEQ;
    }

    /* 计算实际需要写入的数据量 (末帧可能不足一帧) */
    save_size = data_size;
    if (ym->is_ymodem && ym->file_size > 0
        && (ym->written + data_size) > ym->file_size)
    {
        save_size = ym->file_size - ym->written;
    }

    /* 写入数据 (内存或文件) */
    ret = receiver_save_data(ym, frame + FRAME_DATA_OFF, save_size);
    if (ret)
    {
        return ret;
    }
    send_byte(ym, MY_YMODEM_ACK);

    /* 文件数据接收完毕: 主动请求 EOT (Ymodem 且文件大小已知) */
    if (ym->is_ymodem && ym->file_size > 0
        && ym->written >= ym->file_size
        && ym->state == STATE_FILEXFER)
    {
        send_byte(ym, MY_YMODEM_NAK);
        ym->state = STATE_ENDOFXFER;
    }

    return MY_YMODEM_OK;
}

/**
 * @brief  接收端主循环 — 握手、首帧、数据帧的完整流程。
 *
 * @details 统一的接收流程，同时支持文件模式和内存模式。
 *          调用前需设置好 data_dst、out_buf/out_buf_size (内存模式)
 *          或 save_dir (文件模式)。
 *
 * @param  ym  Ymodem 句柄
 * @return MY_YMODEM_OK 成功，否则返回错误码
 */
static int receiver_run(my_ymodem_t *ym)
{
    int handshake_retry = get_handshake_retry(ym);
    int recv_timeout = get_recv_timeout(ym);

    /* ---- 握手阶段 ---- */
    for (int i = 0; i < handshake_retry; i++)
    {
        /* CRC 模式过半无响应, 回退到 checksum 模式 */
        if (ym->crc_type == 2 && i >= handshake_retry / 2)
        {
            ym->crc_type = 1;
        }

        int crc_len = receiver_send_request(ym);

        /* 等待首字节 (帧头) */
        int ret = recv_data(ym, ym->frame, 1, recv_timeout);
        if (ret <= 0)
        {
            continue;  /* 超时 — 重试握手 */
        }

        /* ---- 处理首帧 ---- */
        int timeout_flag = 0;
        int first_frame_retry = get_send_retry(ym);
        do
        {
            ret = receiver_process_first_frame(ym, crc_len);
            if (ret == MY_YMODEM_ERR_HEAD)
            {
                /* 无效帧头 — 尝试读取下一字节作为新帧开始 */
                ret = recv_data(ym, ym->frame, 1, recv_timeout);
                if (ret <= 0)
                {
                    timeout_flag = 1;
                    break;
                }
            }
            else if (ret != MY_YMODEM_OK)
            {
                /* 首帧校验失败 (NAK 已发送): 等待发送方重发首帧 */
                first_frame_retry--;
                if (first_frame_retry <= 0)
                {
                    return ret;
                }
                continue;
            }
            else
            {
                break;       /* 首帧处理成功 */
            }
        } while (1);

        if (timeout_flag)
        {
            continue;   /* 回到握手阶段重试 */
        }

        /* ---- 处理后续帧 ---- */
        ym->state = STATE_FILEXFER;
        int frame_error = 0;
        int max_frame_retry = get_send_retry(ym);
        while (1)
        {
            ret = receiver_process_frame(ym, crc_len);
            if (ret == MY_YMODEM_ERR_HEAD
                || ret == MY_YMODEM_ERR_STATE
                || ret == MY_YMODEM_ERR_IO)
            {
                /* 不可恢复的错误 */
                return ret;
            }
            if (ret != MY_YMODEM_OK)
            {
                /* 可恢复错误 (ERR_CRC / ERR_SIZE / ERR_SEQ):
                 * NAK 已发送, 发送方将重发当前帧 */
                frame_error++;
                if (frame_error >= max_frame_retry)
                {
                    send_byte(ym, MY_YMODEM_CAN);
                    return MY_YMODEM_ERR_RETRY;
                }
                continue;
            }
            frame_error = 0;
            if (ym->state == STATE_ENDED)
            {
                /* 传输完成 */
                port_flush(ym);
                receiver_close_file(ym);
                return MY_YMODEM_OK;
            }
        }
    }

    /* 握手重试耗尽 */
    port_flush(ym);
    return MY_YMODEM_ERR_RETRY;
}
#endif /* MY_YMODEM_ENABLE_RECV */

/*===========================================================================
 * 发送端内部实现
 *
 * 协议流程:
 *   1. 发送端等待接收端的 'C' (CRC16) 或 NAK (checksum) 握手
 *   2. Ymodem: 发送文件信息帧 (seq=0, 文件名 + 大小)
 *   3. 逐帧发送数据，每帧等待 ACK 应答
 *   4. 数据发送完毕，发送 EOT 并等待确认
 *   5. Ymodem: 发送空结束帧 (seq=0, 全零数据)
 *===========================================================================*/

#if MY_YMODEM_ENABLE_SEND
/**
 * @brief  从数据源读取下一块数据。
 *
 * @details 根据 data_src 类型从对应来源读取:
 *          - 文件源 (data_src==0): 使用 fread
 *          - 内存源 (data_src==1): 使用 memcpy
 *
 * @param  ym   Ymodem 句柄
 * @param  buf  输出缓冲区
 * @param  len  期望读取长度
 * @return 实际读取字节数 (0 表示 EOF/数据耗尽)
 */
static int sender_read_chunk(my_ymodem_t *ym, uint8_t *buf, int len)
{
    if (ym->data_src == 0)
    {
        /* 文件数据源 */
        return (int)fread(buf, 1, (size_t)len, ym->fp);
    }
    else
    {
        /* 内存缓冲区数据源 */
        int avail = ym->file_size - ym->mem_offset;
        if (avail <= 0)
        {
            return 0;
        }
        int n = (len < avail) ? len : avail;
        memcpy(buf, ym->mem_buf + ym->mem_offset, (size_t)n);
        ym->mem_offset += n;
        return n;
    }
}

/**
 * @brief  等待接收端握手信号。
 *
 * @details 循环等待接收端发送 'C' (CRC16 请求) 或 NAK (checksum 请求)。
 *          超时或收到非预期字节时刷新缓冲区并重试。
 *
 * @param  ym  Ymodem 句柄
 * @return 1 (checksum) 或 2 (CRC16), -1 表示握手超时
 */
static int sender_handshake(my_ymodem_t *ym)
{
    int retry = get_handshake_retry(ym);
    int recv_timeout = get_recv_timeout(ym);

    for (int i = 0; i < retry; i++)
    {
        uint8_t c;
        int n = recv_byte(ym, &c, recv_timeout);
        if (n > 0)
        {
            if (c == MY_YMODEM_CRC16_C)
            {
                ym->crc_type = 2;
                return 2;
            }
            if (c == MY_YMODEM_NAK)
            {
                ym->crc_type = 1;
                return 1;
            }
            /* 收到非预期字节 — 刷新缓冲区并继续等待 */
            port_flush(ym);
        }
    }
    return -1;  /* 握手超时 */
}

/**
 * @brief  发送 Ymodem 文件信息帧 (seq=0)。
 *
 * @details 文件信息帧格式 (128 字节数据区):
 *          - 文件名 (以 '\0' 结尾)
 *          - '\0'
 *          - 文件大小 (ASCII 十进制字符串)
 *          - 剩余空间填 '\0'
 *
 *          发送后等待 ACK + 'C' 确认。
 *
 * @param  ym  Ymodem 句柄
 * @return MY_YMODEM_OK 成功, MY_YMODEM_ERR_ACK 应答错误
 */
static int sender_send_file_frame(my_ymodem_t *ym)
{
    uint8_t *f = ym->frame;
    int recv_timeout = get_recv_timeout(ym);

    memset(f, 0, FRAME_BUF_SIZE);

    /* 组装帧头 */
    f[FRAME_HEAD_OFF]  = MY_YMODEM_SOH;
    f[FRAME_SEQ_OFF]   = 0x00;
    f[FRAME_SEQN_OFF]  = 0xFF;

    /* 组装文件信息: file_name + '\0' + file_size (ASCII) */
    /* 数据区仅 128 字节: 文件名过长时截断, 避免大小字段丢失 */
    char short_name[INFO_MAX_NAME_LEN];
    strncpy(short_name, ym->file_name, sizeof(short_name) - 1);
    short_name[sizeof(short_name) - 1] = '\0';
    int info_len = snprintf((char *)(f + FRAME_DATA_OFF),
                            MY_YMODEM_SOH_SIZE, "%s%c%d",
                            short_name, '\0', ym->file_size);
    info_len++;  /* 包含末尾的 '\0' */

    /* 计算并填写校验字段 */
    int crc_off = FRAME_DATA_OFF + MY_YMODEM_SOH_SIZE;
    int crc_len;
    if (ym->crc_type == 1)
    {
        f[crc_off] = calc_checksum(f + FRAME_DATA_OFF, MY_YMODEM_SOH_SIZE);
        crc_len = 1;
    }
    else
    {
        uint16_t crc = calc_crc16(f + FRAME_DATA_OFF, MY_YMODEM_SOH_SIZE);
        f[crc_off]     = (uint8_t)(crc >> 8);
        f[crc_off + 1] = (uint8_t)(crc & 0xFF);
        crc_len = 2;
    }

    /* 发送文件信息帧, 等待 ACK + 数据帧请求 (CRC: 'C', checksum: NAK) */
    int send_retry = get_send_retry(ym);
    uint8_t c;
    for (int i = 0; i < send_retry; i++)
    {
        send_data(ym, f, FRAME_DATA_OFF + MY_YMODEM_SOH_SIZE + crc_len);

        if (recv_byte(ym, &c, recv_timeout) <= 0 || c != MY_YMODEM_ACK)
        {
            continue;   /* 超时或应答异常: 重发信息帧 */
        }
        uint8_t expect = (ym->crc_type == 1) ? MY_YMODEM_NAK
                                             : MY_YMODEM_CRC16_C;
        if (recv_byte(ym, &c, recv_timeout) <= 0 || c != expect)
        {
            continue;
        }
        return MY_YMODEM_OK;
    }

    return MY_YMODEM_ERR_ACK;
}

/**
 * @brief  发送单个数据帧。
 *
 * @details 帧组装过程:
 *          1. 根据数据长度选择合适的帧类型 (SOH=128B / STX=1024B)
 *          2. 复制数据并填充 (不足部分填 0x1A)
 *          3. 填写帧序号和反码
 *          4. 计算并填写校验字段
 *          5. 发送帧并等待 ACK/NAK/CAN 应答
 *          6. NAK 时重试 (最多 send_retry 次)
 *
 * @param  ym    Ymodem 句柄
 * @param  data  数据缓冲区
 * @param  len   数据长度
 * @param  seq   帧序号 (从 1 开始递增)
 * @return MY_YMODEM_OK 成功，否则返回错误码
 */
static int sender_send_frame(my_ymodem_t *ym, const uint8_t *data,
                             int len, int seq)
{
    uint8_t *f = ym->frame;
    int recv_timeout = get_recv_timeout(ym);
    int send_retry = get_send_retry(ym);

    memset(f, 0, FRAME_BUF_SIZE);

    /* 根据数据长度选择帧类型 */
    uint8_t head;
    int data_size;
    if (ym->is_ymodem)
    {
        if (len <= MY_YMODEM_SOH_SIZE)
        {
            head = MY_YMODEM_SOH;
            data_size = MY_YMODEM_SOH_SIZE;
        }
        else
        {
            head = MY_YMODEM_STX;
            data_size = MY_YMODEM_STX_SIZE;
        }
    }
    else
    {
        /* 发送端根据 xmodem_1k 选择帧大小 */
        if (ym->xmodem_1k)
        {
            head = MY_YMODEM_STX;
            data_size = MY_YMODEM_STX_SIZE;
        }
        else
        {
            head = MY_YMODEM_SOH;
            data_size = MY_YMODEM_SOH_SIZE;
        }
    }

    /* 复制数据，不足部分填 0x1A (SUB 字符) */
    memcpy(f + FRAME_DATA_OFF, data, (size_t)len);
    if (len < data_size)
    {
        memset(f + FRAME_DATA_OFF + len, 0x1A, (size_t)(data_size - len));
    }

    /* 组装帧头 */
    f[FRAME_HEAD_OFF] = head;
    f[FRAME_SEQ_OFF]  = (uint8_t)(seq & 0xFF);
    f[FRAME_SEQN_OFF] = (uint8_t)(~(seq & 0xFF));

    /* 计算校验字段 */
    int crc_off = FRAME_DATA_OFF + data_size;
    int crc_len;
    if (ym->crc_type == 1)
    {
        f[crc_off] = calc_checksum(f + FRAME_DATA_OFF, data_size);
        crc_len = 1;
    }
    else
    {
        uint16_t crc = calc_crc16(f + FRAME_DATA_OFF, data_size);
        f[crc_off]     = (uint8_t)(crc >> 8);
        f[crc_off + 1] = (uint8_t)(crc & 0xFF);
        crc_len = 2;
    }

    int frame_size = FRAME_DATA_OFF + data_size + crc_len;

    /* 重试循环: 发送帧并等待应答 */
    for (int i = 0; i < send_retry; i++)
    {
        send_data(ym, f, frame_size);
        uint8_t c;
        if (recv_byte(ym, &c, recv_timeout) <= 0)
        {
            continue;
        }

        if (c == MY_YMODEM_ACK)
        {
            return MY_YMODEM_OK;
        }
        if (c == MY_YMODEM_CAN)
        {
            /* 接收端主动取消传输 */
            send_byte(ym, MY_YMODEM_CAN);
            return MY_YMODEM_ERR_CAN;
        }
        if (c != MY_YMODEM_NAK)
        {
            return MY_YMODEM_ERR_ACK;
        }
        /* NAK — 重试 */
    }

    /* 重试耗尽，取消传输 */
    send_byte(ym, MY_YMODEM_CAN);
    return MY_YMODEM_ERR_CAN;
}

/**
 * @brief  发送 EOT / 传输结束序列。
 *
 * @details Ymodem 结束序列:
 *          1. 发送 EOT
 *          2. 等待 NAK
 *          3. 发送第二个 EOT
 *          4. 等待 ACK + 'C'
 *          5. 发送空结束帧 (seq=0, 全零数据)
 *          6. 等待最终 ACK
 *
 *          Xmodem 结束序列:
 *          1. 发送 EOT
 *          2. 等待 ACK
 *
 * @param  ym  Ymodem 句柄
 * @return MY_YMODEM_OK 成功，否则返回错误码
 */
static int sender_send_eot(my_ymodem_t *ym)
{
    uint8_t c;
    int n;
    int recv_timeout = get_recv_timeout(ym);
    int send_retry = get_send_retry(ym);
    int i;

    /* Xmodem: 发送 EOT, 等待 ACK (超时重发) */
    if (!ym->is_ymodem)
    {
        for (i = 0; i < send_retry; i++)
        {
            send_byte(ym, MY_YMODEM_EOT);
            n = recv_byte(ym, &c, recv_timeout);
            if (n > 0 && c == MY_YMODEM_ACK)
            {
                return MY_YMODEM_OK;
            }
        }
        return MY_YMODEM_ERR_ACK;
    }

    /* ---- 第一次 EOT ----
     * 标准应答为 NAK; 兼容:
     *   - 接收端主动请求 EOT (先发 NAK)
     *   - 接收端直接确认第一次 EOT (ACK)
     *   - 接收端直接请求结束帧 ('C' 或 NAK) */
    int end_frame_requested = 0;
    for (i = 0; i < send_retry; i++)
    {
        send_byte(ym, MY_YMODEM_EOT);
        n = recv_byte(ym, &c, recv_timeout);
        if (n <= 0)
        {
            continue;   /* 超时: 重发 EOT */
        }
        if (c == MY_YMODEM_NAK)
        {
            break;      /* 标准应答: 进入第二次 EOT */
        }
        if (c == MY_YMODEM_ACK)
        {
            break;      /* 已确认第一次 EOT: 等待其请求结束帧 */
        }
        if (c == MY_YMODEM_CRC16_C || c == MY_YMODEM_NAK)
        {
            end_frame_requested = 1;   /* 直接请求结束帧 */
            break;
        }
        /* 其他字节: 继续重试 */
    }
    if (i >= send_retry)
    {
        return MY_YMODEM_ERR_ACK;
    }

    if (c == MY_YMODEM_ACK)
    {
        /* 等待结束帧请求 (CRC: 'C', checksum: NAK) */
        uint8_t expect = (ym->crc_type == 1) ? MY_YMODEM_NAK
                                             : MY_YMODEM_CRC16_C;
        for (i = 0; i < send_retry; i++)
        {
            n = recv_byte(ym, &c, recv_timeout);
            if (n <= 0)
            {
                continue;
            }
            if (c == expect)
            {
                break;
            }
            if (c == MY_YMODEM_ACK)
            {
                continue;   /* 多余 ACK: 忽略 */
            }
        }
        if (i >= send_retry)
        {
            return MY_YMODEM_ERR_ACK;
        }
    }
    else if (!end_frame_requested)
    {
        /* ---- 第二次 EOT: 等待 ACK (超时重发) ---- */
        for (i = 0; i < send_retry; i++)
        {
            send_byte(ym, MY_YMODEM_EOT);
            n = recv_byte(ym, &c, recv_timeout);
            if (n > 0 && c == MY_YMODEM_ACK)
            {
                break;
            }
        }
        if (i >= send_retry)
        {
            return MY_YMODEM_ERR_ACK;
        }

        /* 等待结束帧请求 (CRC: 'C', checksum: NAK) */
        uint8_t expect = (ym->crc_type == 1) ? MY_YMODEM_NAK
                                             : MY_YMODEM_CRC16_C;
        for (i = 0; i < send_retry; i++)
        {
            n = recv_byte(ym, &c, recv_timeout);
            if (n > 0 && c == expect)
            {
                break;
            }
            if (n > 0 && c == MY_YMODEM_ACK)
            {
                continue;   /* 多余 ACK: 忽略 */
            }
        }
        if (i >= send_retry)
        {
            return MY_YMODEM_ERR_ACK;
        }
    }

    /* ---- 发送零长度结束帧 (seq=0, 数据全 0, 128 字节) ---- */
    memset(ym->frame, 0, FRAME_BUF_SIZE);
    ym->frame[FRAME_HEAD_OFF] = MY_YMODEM_SOH;
    ym->frame[FRAME_SEQ_OFF]  = 0x00;
    ym->frame[FRAME_SEQN_OFF] = 0xFF;

    int crc_off = FRAME_DATA_OFF + MY_YMODEM_SOH_SIZE;
    int crc_len;
    if (ym->crc_type == 1)
    {
        ym->frame[crc_off] = calc_checksum(ym->frame + FRAME_DATA_OFF,
                                           MY_YMODEM_SOH_SIZE);
        crc_len = 1;
    }
    else
    {
        uint16_t crc = calc_crc16(ym->frame + FRAME_DATA_OFF,
                                  MY_YMODEM_SOH_SIZE);
        ym->frame[crc_off]     = (uint8_t)(crc >> 8);
        ym->frame[crc_off + 1] = (uint8_t)(crc & 0xFF);
        crc_len = 2;
    }
    int frame_size = FRAME_DATA_OFF + MY_YMODEM_SOH_SIZE + crc_len;

    /* 发送结束帧, 等待最终 ACK (超时重发) */
    for (i = 0; i < send_retry; i++)
    {
        send_data(ym, ym->frame, frame_size);
        n = recv_byte(ym, &c, recv_timeout);
        if (n > 0 && c == MY_YMODEM_ACK)
        {
            return MY_YMODEM_OK;
        }
    }
    return MY_YMODEM_ERR_ACK;
}

/*===========================================================================
 * 发送端内部执行函数 (文件和内存数据源通用)
 *===========================================================================*/

/**
 * @brief  执行发送协议。
 *
 * @details 调用前需已设置:
 *          ym->file_name, ym->file_size, ym->data_src,
 *          ym->fp 或 ym->mem_buf, ym->is_ymodem, ym->xmodem_1k。
 *          ym->mem_offset 会被重置为 0。
 *
 * @param  ym  Ymodem 句柄
 * @return MY_YMODEM_OK 成功，否则返回错误码
 */
static int sender_run(my_ymodem_t *ym)
{
    ym->state      = STATE_CONNECTING;
    ym->mem_offset = 0;

    /* ---- 握手阶段 ---- */
    int crc_len = sender_handshake(ym);
    if (crc_len < 0)
    {
        return MY_YMODEM_ERR_RETRY;
    }

    ym->state = STATE_CONNECTED;

    /* ---- 发送文件信息帧 (仅 Ymodem) ---- */
    if (ym->is_ymodem)
    {
        int ret = sender_send_file_frame(ym);
        if (ret)
        {
            return ret;
        }
    }

    /* ---- 发送数据帧 ---- */
    /* Ymodem 默认 1024B 帧; Xmodem 根据 xmodem_1k 选择 */
    int data_size = ym->is_ymodem ? MY_YMODEM_STX_SIZE :
                    ym->xmodem_1k  ? MY_YMODEM_STX_SIZE : MY_YMODEM_SOH_SIZE;

    ym->state = STATE_FILEXFER;

    uint8_t buf[MY_YMODEM_STX_SIZE];
    int seq = 1;        /* 帧序号从 1 开始 (0 号帧为文件信息帧) */
    while (1)
    {
        int n = sender_read_chunk(ym, buf, data_size);
        if (n == 0)
        {
            break;      /* 数据读取完毕 */
        }

        int ret = sender_send_frame(ym, buf, n, seq);
        if (ret)
        {
            return ret;
        }
        seq++;
    }

    /* 文件大小恰为块大小整数倍时, 标准要求补发零长度数据块,
     * 向接收方明确表示 EOF, 避免其误认为仍有后续数据 */
    if (ym->is_ymodem && ym->file_size > 0
        && (ym->file_size % data_size) == 0)
    {
        int ret = sender_send_frame(ym, buf, 0, seq);
        if (ret)
        {
            return ret;
        }
        seq++;
    }

    ym->state = STATE_ENDOFXFER;

    /* ---- 发送 EOT 并结束 ---- */
    return sender_send_eot(ym);
}
#endif /* MY_YMODEM_ENABLE_SEND */

/*===========================================================================
 * 公开 API 实现
 *===========================================================================*/

/**
 * @brief  分配并初始化 Ymodem 实例。
 *
 * @param  io  平台配置 (send / recv 不可为 NULL)
 * @return 句柄指针，失败返回 NULL
 */
my_ymodem_t *my_ymodem_init(my_ymodem_io_t *io)
{
    if (!io || !io->send || !io->recv)
    {
        return NULL;
    }

    my_ymodem_t *ym = (my_ymodem_t *)calloc(1, sizeof(*ym));
    if (!ym)
    {
        return NULL;
    }

    /* 保存平台配置指针 */
    ym->io       = io;
    ym->crc_type = 2;       /* 默认使用 CRC16 校验 */
    ym->state    = STATE_CONNECTING;
    return ym;
}

/**
 * @brief  释放 Ymodem 实例。
 *
 * @details 关闭所有打开的文件并释放内存。
 *
 * @param  ym  Ymodem 句柄
 */
void my_ymodem_deinit(my_ymodem_t *ym)
{
    if (ym)
    {
#ifndef MY_YMODEM_NO_FILE_IO
        if (ym->data_src == 0 && ym->fp)
        {
            fclose(ym->fp);
        }
#endif
        free(ym);
    }
}

/* ---- 发送: 文件 ---- */

#if MY_YMODEM_ENABLE_SEND
/**
 * @brief  通过 Ymodem/Xmodem 发送文件（阻塞式）。
 *
 * @param  ym         Ymodem 句柄
 * @param  file_path  要发送的文件路径
 * @param  is_ymodem  1 = Ymodem, 0 = Xmodem
 * @param  xmodem_1k  1 = Xmodem-1k, 0 = Xmodem (128B)
 *                    当 io.protocol > 0 时以上两参数被忽略。
 * @return MY_YMODEM_OK 成功，否则返回错误码
 */
int my_ymodem_send(my_ymodem_t *ym, const char *file_path,
                   int is_ymodem, int xmodem_1k)
{
    if (!ym || !file_path)
    {
        return MY_YMODEM_ERR_PARAM;
    }

    /* 打开文件 */
    ym->fp = fopen(file_path, "rb");
    if (!ym->fp)
    {
        return MY_YMODEM_ERR_FILE;
    }

    /* 从路径提取文件名 */
    extract_file_name(file_path, ym->file_name, MAX_PATH_LEN);

    /* 获取文件大小 */
    fseek(ym->fp, 0, SEEK_END);
    ym->file_size = (int)ftell(ym->fp);
    fseek(ym->fp, 0, SEEK_SET);

    /* 设置数据源 */
    ym->data_src  = 0;              /* 文件 */
    ym->data_dst  = 0;              /* 无关 (发送端不使用) */
    ym->mem_buf   = NULL;

    /* 解析协议参数 */
    sender_apply_protocol(ym, is_ymodem, xmodem_1k);

    /* 执行发送 */
    int ret = sender_run(ym);

    /* 清理 */
    fclose(ym->fp);
    ym->fp = NULL;

    return ret;
}

/* ---- 发送: 内存 ---- */

/**
 * @brief  从内存缓冲区通过 Ymodem/Xmodem 发送数据（阻塞式）。
 *
 * @param  ym         Ymodem 句柄
 * @param  file_name  传输的文件名 (如 "ble_fota.bin")
 * @param  data       内存数据指针
 * @param  size       数据大小 (字节)
 * @param  is_ymodem  1 = Ymodem, 0 = Xmodem
 * @param  xmodem_1k  1 = Xmodem-1k, 0 = Xmodem (128B)
 *                    当 io.protocol > 0 时以上两参数被忽略。
 * @return MY_YMODEM_OK 成功，否则返回错误码
 */
int my_ymodem_send_from_mem(my_ymodem_t *ym,
                            const char *file_name,
                            const uint8_t *data, int size,
                            int is_ymodem, int xmodem_1k)
{
    if (!ym || !file_name || !data || size <= 0)
    {
        return MY_YMODEM_ERR_PARAM;
    }

    /* 复制文件名 */
    strncpy(ym->file_name, file_name, MAX_PATH_LEN - 1);
    ym->file_name[MAX_PATH_LEN - 1] = '\0';

    /* 设置内存数据源 */
    ym->file_size = size;
    ym->data_src  = 1;              /* 内存 */
    ym->data_dst  = 0;              /* 无关 (发送端不使用) */
    ym->mem_buf   = data;
    ym->fp        = NULL;

    /* 解析协议参数 */
    sender_apply_protocol(ym, is_ymodem, xmodem_1k);

    return sender_run(ym);
}
#endif /* MY_YMODEM_ENABLE_SEND */

/* ---- 接收: 文件 ---- */

#if MY_YMODEM_ENABLE_RECV
/**
 * @brief  接收文件 (Ymodem/Xmodem 自动检测，阻塞式)。
 *
 * @param  ym        Ymodem 句柄
 * @param  save_dir  接收文件存放目录 (NULL/"" 表示当前目录)
 * @return MY_YMODEM_OK 成功，否则返回负值错误码
 */
int my_ymodem_receive(my_ymodem_t *ym, const char *save_dir)
{
    if (!ym)
    {
        return MY_YMODEM_ERR_PARAM;
    }

    /* 设置存放目录 */
    if (save_dir && save_dir[0])
    {
        strncpy(ym->save_dir, save_dir, MAX_PATH_LEN - 1);
        ym->save_dir[MAX_PATH_LEN - 1] = '\0';
    }
    else
    {
        ym->save_dir[0] = '\0';
    }

    /* 重置状态 */
    ym->state     = STATE_CONNECTING;
    ym->is_ymodem = 0;
    ym->crc_type  = 2;
    ym->written   = 0;
    ym->fp        = NULL;
    ym->data_dst  = 0;

    return receiver_run(ym);
}

/* ---- 接收: 内存 ---- */

/**
 * @brief  接收文件到内存缓冲区 (阻塞式，自动检测协议类型)。
 *
 * @param  ym        Ymodem 句柄
 * @param  buf       接收缓冲区 (需预先分配)
 * @param  buf_size  缓冲区大小 (字节)
 * @param  out_name  输出: 文件名 (最长 256 字节), 可为 NULL
 * @param  out_size  输出: 实际数据大小 (字节), 可为 NULL
 * @return MY_YMODEM_OK 成功，否则返回负值错误码
 */
int my_ymodem_recv_mem(my_ymodem_t *ym,
                       uint8_t *buf, int buf_size,
                       char *out_name, int *out_size)
{
    int ret;

    if (!ym || !buf || buf_size <= 0)
    {
        return MY_YMODEM_ERR_PARAM;
    }

    /* 设置内存接收模式 */
    ym->data_dst      = 1;              /* 内存 */
    ym->out_buf       = buf;
    ym->out_buf_size  = buf_size;
    ym->save_dir[0]   = '\0';
    ym->fp            = NULL;

    /* 重置状态 */
    ym->state     = STATE_CONNECTING;
    ym->is_ymodem = 0;
    ym->crc_type  = 2;
    ym->written   = 0;

    /* 执行接收 */
    ret = receiver_run(ym);

    /* 输出文件名和实际大小 */
    if (ret == MY_YMODEM_OK)
    {
        if (out_name)
        {
            int name_len = (int)strlen(ym->file_name);
            if (name_len >= MAX_PATH_LEN)
            {
                name_len = MAX_PATH_LEN - 1;
            }
            memcpy(out_name, ym->file_name, (size_t)name_len);
            out_name[name_len] = '\0';
        }
        if (out_size)
        {
            *out_size = ym->written;
        }
    }

    return ret;
}
#endif /* MY_YMODEM_ENABLE_RECV */
