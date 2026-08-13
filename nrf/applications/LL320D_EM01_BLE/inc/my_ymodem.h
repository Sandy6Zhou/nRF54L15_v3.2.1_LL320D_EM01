/**
 * @file    my_ymodem.h
 * @brief   Ymodem/Xmodem 协议栈头文件（可移植 C 语言实现）
 *
 * @details 本模块由 my_ymodem.c + my_ymodem.h 两个文件组成，可独立用于任何
 *          嵌入式平台。用户只需提供 3 个硬件原语:
 *
 *          1. my_ymodem_init(&io)      — 初始化，注册硬件原语 + 协议配置
 *          2. my_ymodem_send_xxx(ym, ...) — 发送（文件/内存）
 *          3. my_ymodem_recv_xxx(ym, ...) — 接收（文件/内存）
 *          (+ my_ymodem_deinit(ym)      — 释放资源)
 *
 * 依赖:  标准 C 库 (stdint.h, stdlib.h, string.h, stdio.h)
 *
 * 协议支持:
 *   - Ymodem (CRC16, 1024 字节帧, 含文件信息帧)
 *   - Xmodem-CRC (128 字节帧, CRC16)
 *   - Xmodem-1k  (1024 字节帧, CRC16)
 *   - Xmodem-checksum (128 字节帧, 8 位校验和)
 *   接收端自动检测发送端协议类型和校验方式。
 */

#ifndef __MY_YMODEM_H__
#define __MY_YMODEM_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * 协议常量
 *===========================================================================*/
/* 帧格式 (Ymodem/Xmodem 通用):
 *   [帧头 1B]     SOH(0x01)=128B 数据, STX(0x02)=1024B 数据
 *   [帧序号 1B]   数据帧从 1 递增, mod 256 (0 号帧为文件信息/结束帧)
 *   [序号反码 1B] ~seq, 接收端校验 seq + ~seq == 0xFF
 *   [数据区]      128B 或 1024B, 不足部分填 0x1A (SUB)
 *   [校验字段]    CRC16(2B, 高字节在前) 或 checksum(1B) */
#define MY_YMODEM_SOH       0x01    /* 128 字节帧头标识 */
#define MY_YMODEM_STX       0x02    /* 1024 字节帧头标识 */
#define MY_YMODEM_EOT       0x04    /* 传输结束标识 */
#define MY_YMODEM_ACK       0x06    /* 确认应答 */
#define MY_YMODEM_NAK       0x15    /* 否定应答/请求重发 */
#define MY_YMODEM_CAN       0x18    /* 取消传输 */
#define MY_YMODEM_CRC16_C   0x43    /* 'C' — 请求 CRC16 模式 */

/*===========================================================================
 * 功能开关 (可在编译命令行 -D 或工程配置中覆盖):
 *   MY_YMODEM_ENABLE_SEND = 1  编译发送功能 (默认开启)
 *   MY_YMODEM_ENABLE_RECV = 1  编译接收功能 (默认开启)
 * 仅使用单向传输时可关闭另一方向, 以节省代码空间;
 * 关闭后对应 API 不再编译, 调用处需同步用 #if 隔离
 *===========================================================================*/
#define MY_YMODEM_ENABLE_SEND   0

#define MY_YMODEM_ENABLE_RECV   1

/* 无文件系统平台: 禁用标准 C 文件路径 (fopen/fread/fwrite/fclose),
 * 接收端必须通过 save_open/save_write/save_close 回调落盘。 */
#define MY_YMODEM_NO_FILE_IO    1
#define MY_YMODEM_SOH_SIZE  128     /* SOH 帧数据区大小 (字节) */
#define MY_YMODEM_STX_SIZE  1024    /* STX 帧数据区大小 (字节) */

/*===========================================================================
 * 协议类型（在 io.protocol 中指定）
 *===========================================================================*/
#define MY_YMODEM_PROTO_AUTO        0   /* 自动: 沿用函数参数 (默认) */
#define MY_YMODEM_PROTO_YMODE       1   /* 强制 Ymodem (1024B帧, 含文件信息帧) */
#define MY_YMODEM_PROTO_XMODEM      2   /* 强制 Xmodem (128B帧) */
#define MY_YMODEM_PROTO_XMODEM_1K   3   /* 强制 Xmodem-1k (1024B帧) */

/*===========================================================================
 * 默认配置值（对应 io_t 中为 0 时生效）
 *===========================================================================*/
/* 默认参数值 (io 结构体对应字段为 0 时生效):
 *   轮询间隔 30ms / 握手重试 10 次 / 帧重试 10 次 / 接收超时 3000ms */
#define MY_YMODEM_DEFAULT_POLL_INTERVAL_MS    30
#define MY_YMODEM_DEFAULT_HANDSHAKE_RETRY     10
#define MY_YMODEM_DEFAULT_SEND_RETRY          10
#define MY_YMODEM_DEFAULT_RECV_TIMEOUT_MS     3000

/*===========================================================================
 * 错误码定义
 *===========================================================================*/
/* 返回值约定: 0 = 成功, 负值 = 具体错误原因, 见下表 */
#define MY_YMODEM_OK         0      /* 传输成功 */
#define MY_YMODEM_ERR_HEAD  -10     /* 帧头标识无效 */
#define MY_YMODEM_ERR_SEQ   -11     /* 帧号校验失败 */
#define MY_YMODEM_ERR_CRC   -12     /* CRC/校验和错误 */
#define MY_YMODEM_ERR_CAN   -13     /* 收到取消传输指令 */
#define MY_YMODEM_ERR_ACK   -14     /* ACK 应答异常 */
#define MY_YMODEM_ERR_SIZE  -15     /* 帧大小不匹配 */
#define MY_YMODEM_ERR_STATE -16     /* 状态机异常 */
#define MY_YMODEM_ERR_IO    -17     /* I/O 操作失败 */
#define MY_YMODEM_ERR_RETRY -18     /* 重试次数耗尽 */
#define MY_YMODEM_ERR_FILE  -19     /* 文件操作失败 */
#define MY_YMODEM_ERR_PARAM -20     /* 参数无效 */

/*===========================================================================
 * 平台配置结构体（移植时只需填充此结构体）
 *
 * 用户提供 3 个硬件原语 + 协议选项，栈内部自动完成
 * 超时控制、重试逻辑和缓冲管理。
 *===========================================================================*/
typedef struct my_ymodem_io {
    /**
     * @brief  阻塞式写入 — len 字节全部发送后返回。
     *
     * @details 栈会循环调用此函数直到所有数据发送完成。
     *          对于可一次性发送全部数据的硬件 (如 UART 非阻塞发送)，
     *          直接返回发送字节数即可。
     *
     * @param  data  数据指针
     * @param  len   字节数
     * @return 实际发送字节数, <= 0 表示错误
     *
     * 示例 (UART): return uart_write_blocking(uart_id, data, len);
     * 示例 (SPI):  return spi_transmit(spi_id, data, len);
     */
    int (*send)(const uint8_t *data, int len);

    /**
     * @brief  非阻塞式接收 — 返回当前可读的字节数。
     *
     * @details 接收时循环轮询此函数，返回 0 表示当前无数据。
     *          可配合 delay_ms 实现超时，返回值 < 0 表示硬件错误。
     *
     * @param  buf     接收缓冲区
     * @param  max_len 缓冲区最大可用空间
     * @param  timeout_ms  超时时间 (毫秒)
     * @return > 0: 读取到的字节数; 0: 无数据; < 0: 错误
     *
     * 示例 (UART): return uart_read_available(uart_id, buf, max_len);
     * 示例 (DMA):  return ringbuf_pop(&rx_rb, buf, max_len);
     */
    int (*recv)(uint8_t *buf, int max_len, int timeout_ms);

    /**
     * @brief  清空接收缓冲区（可选 — 可为 NULL）。
     *
     * @details 在传输开始前和异常恢复时调用。若为 NULL，
     *          则使用 recv() 循环排空缓冲区。
     *
     * 示例: while(uart_read_available(id,buf,64)>0);
     */
    void (*flush)(void);

    /* ---- 协议配置（为 0 时使用默认值）---- */

    /**
     * @brief  协议类型: MY_YMODEM_PROTO_AUTO / YMODE / XMODEM / XMODEM_1K。
     *         默认: MY_YMODEM_PROTO_AUTO (0)，沿用函数参数。
     */
    int protocol;

    /**
     * @brief  接收轮询间隔 (ms)。值越小响应越快但 CPU 占用越高。
     *         默认: 30ms。推荐范围: 10~50ms。
     */
    int poll_interval_ms;

    /**
     * @brief  握手阶段最大重试次数。默认: 10（约 30s）。
     */
    int handshake_retry;

    /**
     * @brief  数据帧发送最大重试次数。默认: 10。
     */
    int send_retry;

    /**
     * @brief  单字节接收超时 (ms)。默认: 3000。
     */
    int recv_timeout_ms;
    /**
     * @brief  流式落盘回调: 打开接收目标 (可选, 用于无文件系统场景, 如写入 Flash)。
     *         data_dst != 1 且 save_open 非空时由接收端在首帧信息解析后调用。
     *         返回值: 0=成功, 非0=失败 (接收端将发 CAN 并返回 MY_YMODEM_ERR_FILE)。
     */
    int (*save_open)(const char *file_name, int file_size);

    /**
     * @brief  流式落盘回调: 写入接收数据 (可选, 与 save_open/save_close 配套)。
     *         每帧数据写入目标后返回; 非0=写入失败 (接收端将发 CAN 并返回 MY_YMODEM_ERR_FILE)。
     */
    int (*save_write)(const uint8_t *data, int len);

    /**
     * @brief  流式落盘回调: 收尾关闭接收目标 (可选)。
     * @param  result_ok  1=会话正常结束 (刷新残留数据), 0=异常中止 (放弃数据)。
     *         接收端成功路径与 deinit 兜底调用幂等。
     */
    int (*save_close)(int result_ok);
} my_ymodem_io_t;

/*===========================================================================
 * Ymodem 句柄类型（不透明 — 用户无需了解内部结构）
 *===========================================================================*/
typedef struct my_ymodem my_ymodem_t;

/*===========================================================================
 * API-1: 初始化 — 绑定平台配置，返回句柄
 *===========================================================================*/

/**
 * @brief  创建 Ymodem 实例并绑定平台硬件接口。
 *
 * @details 使用库的第一步：填充 my_ymodem_io_t 结构体，
 *          至少提供 send / recv 回调函数，
 *          然后调用此函数即可获得句柄，所有后续操作都通过此句柄进行。
 *
 * @param  io  平台配置（send / recv 不可为 NULL）
 * @return 句柄指针，失败时返回 NULL。
 *
 * 使用示例:
 * @code
 *   my_ymodem_io_t io = {
 *       .send = my_uart_send,
 *       .recv = my_uart_recv,
 *       // 其余字段使用默认值 (0)
 *   };
 *   my_ymodem_t *ym = my_ymodem_init(&io);
 * @endcode
 */
my_ymodem_t *my_ymodem_init(my_ymodem_io_t *io);

/*===========================================================================
 * API-2: 发送
 *===========================================================================*/

#if MY_YMODEM_ENABLE_SEND
/**
 * @brief  发送文件 — 从文件系统读取并通过协议发送到对端。
 *
 * @details 通过标准 C 的 fopen/fread 读取文件内容并发送。
 *          文件名自动从路径末尾提取。
 *
 * @param  ym         my_ymodem_init() 返回的句柄
 * @param  file_path  要发送的文件路径
 * @param  is_ymodem  1 = Ymodem, 0 = Xmodem
 * @param  xmodem_1k  1 = Xmodem-1k (1024 字节帧), 0 = Xmodem (128B)
 *                    当 is_ymodem == 1 时此参数忽略。
 *                    当 io.protocol > 0 时此参数忽略，以 io.protocol 为准。
 * @return MY_YMODEM_OK 成功，否则返回负值错误码。
 */
int my_ymodem_send(my_ymodem_t *ym, const char *file_path,
                   int is_ymodem, int xmodem_1k);

/**
 * @brief  发送内存数据 — 无需文件系统，直接从内存发送。
 *
 * @details 适用于固件已通过 HTTP 下载到 RAM 的场景，
 *          可避免文件系统读写开销。调用者持有 @p data 的所有权，
 *          函数返回后可立即释放。
 *
 * @param  ym         my_ymodem_init() 返回的句柄
 * @param  file_name  传输的文件名 (如 "ble_fota.bin")，最长 255 字符
 * @param  data       内存中的数据指针
 * @param  size       @p data 的字节数
 * @param  is_ymodem  1 = Ymodem, 0 = Xmodem
 * @param  xmodem_1k  1 = Xmodem-1k, 0 = Xmodem (128B)
 *                    当 is_ymodem == 1 时此参数忽略。
 *                    当 io.protocol > 0 时此参数忽略，以 io.protocol 为准。
 * @return MY_YMODEM_OK 成功，否则返回负值错误码。
 */
int my_ymodem_send_from_mem(my_ymodem_t *ym,
                            const char *file_name,
                            const uint8_t *data, int size,
                            int is_ymodem, int xmodem_1k);
#endif /* MY_YMODEM_ENABLE_SEND */

/*===========================================================================
 * API-3: 接收
 *===========================================================================*/

#if MY_YMODEM_ENABLE_RECV
/**
 * @brief  接收文件 — 保存到文件系统（可自动检测协议类型）。
 *
 * @details 文件名从发送端的 Ymodem 帧自动获取，接收到的文件保存到
 *          @p save_dir 目录下。若 @p save_dir 为 NULL 或 ""，
 *          则保存到当前目录。
 *
 *          支持自动检测: Ymodem (CRC16)、Xmodem-CRC、
 *          Xmodem-1k、Xmodem-checksum。
 *
 * @param  ym        my_ymodem_init() 返回的句柄
 * @param  save_dir  接收文件存放目录（可为 NULL / ""）
 * @return MY_YMODEM_OK 成功，否则返回负值错误码。
 */
int my_ymodem_receive(my_ymodem_t *ym, const char *save_dir);

/**
 * @brief  接收文件到内存缓冲区 — 无需文件系统（可自动检测协议类型）。
 *
 * @details 与 my_ymodem_receive() 不同，此函数将接收到的数据写入
 *          用户提供的内存缓冲区，无需文件系统。适用于无文件系统的
 *          嵌入式平台（如 RTOS 小内存设备）。
 *
 *          文件名从发送端的 Ymodem 帧自动获取。
 *          若接收数据量超过 buf_size，返回 MY_YMODEM_ERR_FILE。
 *
 * @param  ym        my_ymodem_init() 返回的句柄
 * @param  buf       接收缓冲区（需预先分配）
 * @param  buf_size  缓冲区大小（字节）
 * @param  out_name  输出: 接收到的文件名（最长 256 字节），可为 NULL
 * @param  out_size  输出: 实际接收的数据大小（字节），可为 NULL
 * @return MY_YMODEM_OK 成功，否则返回负值错误码。
 */
int my_ymodem_recv_mem(my_ymodem_t *ym,
                       uint8_t *buf, int buf_size,
                       char *out_name, int *out_size);
#endif /* MY_YMODEM_ENABLE_RECV */

/*===========================================================================
 * API-4: 释放
 *===========================================================================*/

/**
 * @brief  释放 Ymodem 实例及其所有资源。
 *
 * @details 关闭所有打开的文件并释放内存。
 *          调用后 @p ym 指针不再有效，不可继续使用。
 *
 * @param  ym  my_ymodem_init() 返回的句柄
 */
void my_ymodem_deinit(my_ymodem_t *ym);


/*===========================================================================
 * API-5: BLE-UART 适配层封装（Port 层）
 *
 * 以下函数封装了 init -> 操作 -> deinit 的完整流程，
 * 必须在 BLE 任务上下文中调用。
 *===========================================================================*/

#if MY_YMODEM_ENABLE_SEND
/**
 * @brief  [BLE-OTA] 通过 Ymodem 发送固件文件 (从文件系统读取)
 *
 * @param  file_path  固件文件路径
 * @return MY_YMODEM_OK 成功, 负值错误码
 */
int my_ymodem_ble_send_file(const char *file_path);

/**
 * @brief  [BLE-OTA] 通过 Ymodem 发送固件 (内存缓冲区, 如 HTTP 下载数据)
 *
 * @param  file_name  信息帧中传输的文件名 (如 "ble_fota.bin")
 * @param  data       固件数据指针
 * @param  size       固件大小 (字节)
 * @return MY_YMODEM_OK 成功, 负值错误码
 */
int my_ymodem_ble_send_from_mem(const char *file_name,
                                const uint8_t *data, int size);
#endif /* MY_YMODEM_ENABLE_SEND */

#if MY_YMODEM_ENABLE_RECV
/**
 * @brief  [BLE-OTA] 通过 Ymodem 接收固件并保存到文件
 *
 * @param  save_dir   保存目录, NULL/"" 表示当前目录
 * @return MY_YMODEM_OK 成功, 负值错误码
 */
int my_ymodem_ble_recv_file(const char *save_dir);

/**
 * @brief  [BLE-OTA] 通过 Ymodem 接收固件到内存缓冲区
 *
 * @param  buf        接收缓冲区 (需预分配)
 * @param  buf_size   缓冲区大小 (字节)
 * @param  out_name   输出: 接收到的文件名, 可为 NULL
 * @param  out_size   输出: 实际接收字节数, 可为 NULL
 * @return MY_YMODEM_OK 成功, 负值错误码
 */
int my_ymodem_ble_recv_mem(uint8_t *buf, int buf_size,
                           char *out_name, int *out_size);
#endif /* MY_YMODEM_ENABLE_RECV */
#ifdef __cplusplus
}
#endif

#endif /* __MY_YMODEM_H__ */
