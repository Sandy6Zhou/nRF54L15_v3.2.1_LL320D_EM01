/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        my_dfu_lte.c
**文件描述:        LTE YModem OTA 实现
**当前版本:        V1.0
**作    者:       周森达 (zhousenda@jimiiot.com)
**完成日期:        2026.08.11
*********************************************************************
** 功能描述:        1. 通过 LTE 串口(UART30) 接收 4G 端 YModem 固件
**                 2. 写入 mcuboot_secondary(image_1) 分区并触发 MCUboot 升级
**                 3. 通过统一 OTA Flash 模块(my_ota_flash) 完成落盘与复位调度
**                 4. 与 BLE DFU(my_dfu_jimi) 互斥, 仅同时存在一个 DFU 会话
*********************************************************************/

/* 必须在包含 my_comm.h 之前定义 BLE_LOG_MODULE_ID，避免与 my_ble_log.h 中的默认定义冲突 */
#define BLE_LOG_MODULE_ID BLE_LOG_MOD_DFU

#include "my_comm.h"

/* 日志模块注册 */
LOG_MODULE_REGISTER(dfu_lte, LOG_LEVEL_INF);

/* Flash 分区定义 (与 BLE DFU 一致) */
#define DFU_LTE_PARTITION    image_1

/* 传输完成后复位延迟 (与 BLE DFU 一致) */
#define DFU_LTE_RESET_DELAY_MS 6500

/* YModem 接收轮询间隔 (及时读取环形缓冲, 防溢出) */
#define DFU_LTE_POLL_INTERVAL_MS 10

/* MCUOTA 会话参数 (my_dfu_lte_prepare 写入, my_dfu_lte_start / 落盘回调消费) */
static uint32_t s_dfu_total_size;      /* 4G 下发的目标固件总大小 */
static uint32_t s_dfu_expect_crc;      /* 4G 下发的期望文件级 CRC32 */
static uint32_t s_dfu_crc;             /* save_write 流式累计的 CRC32 */
static bool     s_dfu_size_mismatch;   /* Ymodem 首帧大小与 total_size 不一致 */

/********************************************************************
**函数名称:  dfu_lte_save_open
**入口参数:  file_name  ---   接收文件名(来自 YModem 首帧)
**           file_size  ---   接收文件总大小(字节)
**出口参数:  无
**函数功能:  落盘回调: 校验大小、擦除 image_1 分区并初始化缓存
**返 回 值:  0 表示成功, 负值表示失败
**注意事项:  与 BLE DFU 互斥; 文件超 mcuboot_primary 容量上限返回 -EFBIG
*********************************************************************/
static int dfu_lte_save_open(const char *file_name, int file_size)
{
    int ret;

    if (file_name == NULL || file_size <= 0)
    {
        LOG_ERR("LTE OTA: invalid file name/size");
        return -EINVAL;
    }

    /* 校验 Ymodem 首帧大小与 START 下发的 total_size 一致 (不一致按 CAN 中断) */
    if ((uint32_t)file_size != s_dfu_total_size)
    {
        LOG_ERR("LTE OTA: size mismatch, ymodem=%d expect=%u",
                file_size, s_dfu_total_size);
        s_dfu_size_mismatch = true;
        return -EINVAL;
    }

    /* 固件大小上限由 my_ota_flash_open 内按 image_1 分区容量统一校验 */
    ret = my_ota_flash_open((uint32_t)file_size);
    if (ret != 0)
    {
        LOG_ERR("LTE OTA: flash open/erase failed, ret=%d", ret);
        return ret; /* YModem 对任意负值按 CAN 处理 */
    }
    LOG_INF("LTE OTA: erase complete, file size=%d", file_size);

    return 0;
}

/********************************************************************
**函数名称:  dfu_lte_save_write
**入口参数:  data  ---   数据指针
**           len   ---   数据长度
**出口参数:  无
**函数功能:  落盘回调: 将数据聚合进 1KB 缓存, 满块或收尾时写 Flash
**返 回 值:  0 表示成功, 负值表示失败
**注意事项:  最后一包由库裁剪至实际大小
*********************************************************************/
static int dfu_lte_save_write(const uint8_t *data, int len)
{
    if (data == NULL || len < 0)
    {
        return -EINVAL;
    }
    if (len == 0)
    {
        /* Ymodem 零长度结束块: 不落盘, 不参与文件级 CRC, 视为成功 (固件大小为 1024 整倍数时标准发送端会发) */
        return 0;
    }

    /* 增量累计文件级 CRC32 (覆盖实际固件字节, 与 4G 端全量计算一致) */
    s_dfu_crc = my_crc32_update(s_dfu_crc, data, (uint32_t)len);

    /* 写入共享 1KB 聚合缓存(满块/末块自动刷写 Flash 并 CRC 读回校验) */
    return my_ota_flash_write_block(data, (uint32_t)len, NULL);
}

/********************************************************************
**函数名称:  dfu_lte_save_close
**入口参数:  result_ok  ---   1=会话正常结束; 0=会话异常中止
**出口参数:  无
**函数功能:  落盘回调: 正常结束时刷新残留缓存, 异常时放弃数据
**返 回 值:  0 表示成功, 负值表示失败
*********************************************************************/
static int dfu_lte_save_close(int result_ok)
{
    int ret;

    /* 正常结束刷新残留缓存/异常放弃; 模块内部无论成败都会释放会话(busy) */
    ret = my_ota_flash_close(result_ok ? true : false);
    if (result_ok)
    {
        LOG_INF("LTE OTA: receive success");
    }
    else
    {
        LOG_ERR("LTE OTA: receive aborted");
    }

    return ret;
}

/********************************************************************
**函数名称:  dfu_lte_uart_write
**入口参数:  data  ---   待发送数据指针
**           len   ---   数据长度
**出口参数:  无
**函数功能:  平台回调: 裸发送数据到 LTE 模块(不附加唤醒字节)
**返 回 值:  实际发送字节数; 负值表示失败
*********************************************************************/
static int dfu_lte_uart_write(const uint8_t *data, int len)
{
    int ret;

    ret = my_lte_uart_send_ymodem(data, (uint16_t)len);
    if (ret != 0)
    {
        return ret;
    }

    return len;
}

/********************************************************************
**函数名称:  dfu_lte_uart_read
**入口参数:  buf        ---   接收缓冲区
**           max_len    ---   缓冲区可用空间
**           timeout_ms ---   整窗超时时间(毫秒)
**出口参数:  buf        ---   存储读出的数据
**函数功能:  平台回调: 从 LTE UART 环形缓冲区非阻塞读取
**          1. 先立即读取一次, 最多取 max_len 字节 (不超取防越界)
**          2. 不足 max_len 时按 DFU_LTE_POLL_INTERVAL_MS 轮询续收,
**             直到取满 max_len 或累计超时 timeout_ms
**返 回 值:  累计已读字节数; 整窗零数据时返回 0 (超时); 负值表示真实硬件错误
**注意事项:  供 LTE YModem OTA 使用, 轮询间隔使用模块常量, 与 io 字段无关
*********************************************************************/
static int dfu_lte_uart_read(uint8_t *buf, int max_len, int timeout_ms)
{
    int total = 0;
    int n;
    int64_t end_time;

    if (buf == NULL || max_len <= 0)
    {
        return -EINVAL;
    }

    /* 阶段1: 立即读取一次, 最多 max_len 字节 */
    n = my_lte_uart_read(buf, (uint32_t)max_len);
    if (n < 0)
    {
        return n;   /* 真实硬件错误 */
    }
    total = n;
    if (total >= max_len)
    {
        return total;
    }

    /* 阶段2: 轮询续收直到取满 max_len 或累计超时 */
    end_time = k_uptime_get() + (int64_t)timeout_ms;
    while (total < max_len && k_uptime_get() < end_time)
    {
        k_sleep(K_MSEC(DFU_LTE_POLL_INTERVAL_MS));
        n = my_lte_uart_read(buf + total, (uint32_t)(max_len - total));
        if (n < 0)
        {
            return n;   /* 真实硬件错误 */
        }
        total += n;
    }

    return total;
}

/********************************************************************
**函数名称:  dfu_lte_uart_flush
**入口参数:  无
**出口参数:  无
**函数功能:  平台回调: 排空 LTE UART 环形缓冲区
**返 回 值:  无
*********************************************************************/
static void dfu_lte_uart_flush(void)
{
    my_lte_uart_flush();
}

/********************************************************************
**函数名称:  my_dfu_lte_prepare
**入口参数:  total_size  ---  4G 下发的固件总大小(字节)
**           expect_crc  ---  4G 下发的期望文件级 CRC32
**出口参数:  无
**函数功能:  升级前置检查并占用 OTA 会话 (校验大小合法性/分区容量/互斥)
**返 回 值:  DFU_LTE_OK 成功, 否则为对应失败原因
**注意事项:  需在 LTE 线程内调用; 成功后由命令层回 READY 再调 my_dfu_lte_start
*********************************************************************/
dfu_lte_fail_reason_t my_dfu_lte_prepare(uint32_t total_size, uint32_t expect_crc)
{
    if (total_size == 0)
    {
        LOG_ERR("LTE OTA: invalid total_size=%u", total_size);
        return DFU_LTE_FAIL_PARAM;
    }

    if (total_size > FLASH_AREA_SIZE(DFU_LTE_PARTITION))
    {
        LOG_ERR("LTE OTA: total_size %u exceeds partition %u",
                total_size, (uint32_t)FLASH_AREA_SIZE(DFU_LTE_PARTITION));
        return DFU_LTE_FAIL_SIZE;
    }

    /* 蓝牙 OTA 会话互斥: 占用失败说明已有 OTA(APP 或 4G 来源)在进行 */
    if (!my_ota_flash_occupy())
    {
        LOG_ERR("LTE OTA rejected: another OTA in progress");
        return DFU_LTE_FAIL_BUSY;
    }

    /* 记录会话参数, 供 my_dfu_lte_start / 落盘回调消费 */
    s_dfu_total_size    = total_size;
    s_dfu_expect_crc    = expect_crc;
    s_dfu_crc           = my_crc32_init();
    s_dfu_size_mismatch = false;

    return DFU_LTE_OK;
}

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
dfu_lte_fail_reason_t my_dfu_lte_start(void)
{
    my_ymodem_io_t io;
    my_ymodem_t *ym;
    int ret;
    uint32_t calc_crc;

    /* 丢弃无关消息, 停止无关数据发送 */
    my_lte_purge_dfu_queues();

    /* 配置 YModem 平台回调 (新版 io_t API, protocol 默认 AUTO) */
    memset(&io, 0, sizeof(io));
    io.send             = dfu_lte_uart_write;
    io.recv             = dfu_lte_uart_read;
    io.flush            = dfu_lte_uart_flush;
    io.poll_interval_ms = DFU_LTE_POLL_INTERVAL_MS;
    io.save_open        = dfu_lte_save_open;
    io.save_write       = dfu_lte_save_write;
    io.save_close       = dfu_lte_save_close;

    /* 创建 YModem 实例并阻塞接收 */
    ym = my_ymodem_init(&io);
    if (ym == NULL)
    {
        LOG_ERR("LTE OTA: my_ymodem_init failed");
        my_ota_flash_close(false); /* 释放 my_dfu_lte_prepare 占用的会话 */
        return DFU_LTE_FAIL_YMODEM;
    }

    ret = my_ymodem_receive(ym, NULL);
    my_ymodem_deinit(ym);

    /* 退出 YModem 会话: 阻塞期间 UART 收包会积压 MY_MSG_LTE_REV 等消息,
     * 清空 LTE 消息队列与残留缓冲, 避免陈旧数据干扰后续流程 */
    my_lte_purge_dfu_queues();

    if (ret != MY_YMODEM_OK)
    {
        LOG_ERR("LTE OTA: receive failed, ret=%d", ret);
        /* 异常退出时 my_ymodem 不会回调 save_close, 必须在此释放 OTA 会话
         * 占用, 否则 busy 标志永不清除, 后续升级将被拒为 FAIL,BUSY */
        my_ota_flash_close(false);
        if (s_dfu_size_mismatch)
        {
            /* Ymodem 首帧大小与 START 下发的 total_size 不一致 (save_open 中断) */
            return DFU_LTE_FAIL_SIZE;
        }
        if (ret == MY_YMODEM_ERR_FILE)
        {
            /* save 回调失败: Flash 落盘错误 */
            return DFU_LTE_FAIL_FLASH;
        }
        return DFU_LTE_FAIL_YMODEM;
    }

    /* 协议层接收成功, 校验文件级完整性 */
    if (s_dfu_size_mismatch)
    {
        LOG_ERR("LTE OTA: size mismatch");
        my_ota_flash_close(false);
        return DFU_LTE_FAIL_SIZE;
    }
    calc_crc = my_crc32_final(s_dfu_crc);
    if (calc_crc != s_dfu_expect_crc)
    {
        LOG_ERR("LTE OTA: crc mismatch, calc=0x%08x expect=0x%08x",
                calc_crc, s_dfu_expect_crc);
        my_ota_flash_close(false);
        return DFU_LTE_FAIL_CRC;
    }
    LOG_INF("LTE OTA: crc32 verified ok, 0x%08x", calc_crc);

    /* 请求 MCUboot 升级(必须在线程上下文调用) */
    boot_request_upgrade(BOOT_UPGRADE_PERMANENT);
    /* 延迟复位 (文件级校验通过才允许进入升级) */
    my_ota_flash_schedule_reset(DFU_LTE_RESET_DELAY_MS);

    return DFU_LTE_OK;
}
