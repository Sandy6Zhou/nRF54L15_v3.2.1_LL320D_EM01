/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        my_dfu_jimi.c
**文件描述:        Jimi 自定义 DFU 协议实现
**当前版本:        V1.0
**作    者:        Harrison Wu (wuyujiao@jimiiot.com)
**完成日期:        2026.03.10
*********************************************************************
** 功能描述:        1. 基于几米自定义 BLE 3.0 DFU 协议解析(Jimi Iot 蓝牙通信协议V3.1.6_2026-3-5)
**                 2. Flash 擦除、写入、读取操作
**                 3. 片段 CRC 校验替代 MD5、定时器管理
**                 4. 与 MCUboot 配合完成 OTA 升级
** 校验机制说明:    APP 端发送 MD5 校验值，但 NCS 3.2.1 SDK 已废弃 MD5 支持。
**                 设备端采用片段 CRC16 校验：每帧数据写入 Flash 后读回验证，
**                 CRC 正确即表示该片段数据完整，达到与 MD5 等效的数据完整性校验。
** OTA 效率优化:    1. 动态分包：根据 MTU 动态计算分包大小（MTU≥384 时用 128/256/384B，否则 128B）
**                 2. 1KB 缓存聚合：数据先写入 1KB 缓存，满 1KB 或最后一包时写入 Flash
**                 3. 16 字节对齐：最后一包向下对齐到 16 字节（AES 加密需要），剩余数据继续请求
**                 优化效果：365KB 固件写入次数从 ~2920 次降至 ~365 次（约 8 倍提升）
** 注意事项：      1. dfu时不启用蓝牙日志，避免日志干扰DFU过程,只使用RTT日志
*********************************************************************/

/* 必须在包含 my_comm.h 之前定义 BLE_LOG_MODULE_ID，避免与 my_ble_log.h 中的默认定义冲突 */
#define BLE_LOG_MODULE_ID BLE_LOG_MOD_DFU

#include "my_comm.h"

/* 日志模块注册 */
LOG_MODULE_REGISTER(dfu_jimi, LOG_LEVEL_INF);

/* Flash 分区定义 */
#define DFU_FLASH_PARTITION    image_1
#define DFU_FLASH_PARTITION_ID FIXED_PARTITION_ID(DFU_FLASH_PARTITION)

/* nRF54L15 Flash 扇区大小为 4KB */
#define FLASH_SECTOR_SIZE 4096

/**
 * @brief CRC16 多项式（与 APP 端保持一致）
 * 在几米自定义 BLE 3.0 DFU 协议解析(Jimi Iot 蓝牙通信协议V3.1.6_2026-3-5)中，没有注明使用该多项式
 * 通过询问几米的开发人员，得知他们使用的是 0xA001 多项式
 * 这里要提醒开发者注意，CRC16 多项式在几米自定义 BLE 3.0 DFU 协议解析(Jimi Iot 蓝牙通信协议V3.1.6_2026-3-5)中没有注明使用该多项式
 * 要通知文档管理员更新文档，说明 CRC16 多项式在几米自定义 BLE 3.0 DFU 协议解析(Jimi Iot 蓝牙通信协议V3.1.6_2026-3-5)中没有注明使用该多项式
 * 而不是使用文档后面所符的CRC校验
 * */
#define CRC16_POLYNOMIAL 0xA001

/* DFU 状态结构体 */
struct jimi_dfu_image_info
{
    uint32_t fw_copy_src_addr;
    uint32_t fw_copy_dst_addr;
    uint32_t fw_copy_size;
};

/* 全局变量 */
static struct jimi_dfu_image_info s_dfu_image; /* DFU 镜像信息 */
static uint32_t s_req_file_addr;               /* 请求文件地址 */
static uint8_t s_repeat_req_count;             /* 重复请求计数 */
static bool s_dfu_end_flag = false;            /* DFU 结束标志 */

static struct k_timer s_file_trans_wait_timer; /* 文件传输等待定时器 */
static struct k_work s_dfu_timeout_work;       /* DFU 超时工作项，用于线程上下文处理 */
static struct k_work s_dfu_retry_work;         /* DFU 重试工作项，用于线程上下文处理 */

/* 函数声明 */
static void dfu_timer_wait_callback(struct k_timer *timer);                               /* 文件传输等待定时器回调函数 */
static void dfu_timeout_work_handler(struct k_work *work);                                /* DFU 超时工作项处理函数 */
static void dfu_retry_work_handler(struct k_work *work);                                  /* DFU 重试工作项处理函数 */

/********************************************************************
**函数名称:  jimi_dfu_calc_pkt_size
**入口参数:  mtu           ---   当前 BLE MTU
**           remain_size   ---   剩余未下载数据大小
**           buf_remain    ---   1KB缓存剩余空间
**出口参数:  无
**函数功能:  根据 MTU 和缓存空间计算最优分包大小
**返 回 值:  分包大小（128 的倍数）
*********************************************************************/
static uint16_t jimi_dfu_calc_pkt_size(uint16_t mtu, uint32_t remain_size, uint16_t buf_remain)
{
    uint16_t pkt_size;
    uint16_t max_data = (mtu > 23) ? (mtu - 3 - 20) : 128; /* MTU - ATT头 - 协议头 */

    if (mtu >= 384)
    {
        /* 大 MTU：使用 128 的倍数，最大 384 */
        pkt_size = (max_data / 128) * 128;
        if (pkt_size >= 384)
            pkt_size = 384;
        else if (pkt_size >= 256)
            pkt_size = 256;
        else if (pkt_size < 128)
            pkt_size = 128;
    }
    else
    {
        /* 小 MTU：固定 128 */
        pkt_size = 128;
    }

    LOG_DBG("DFU calc: mtu=%d, max_data=%d, init_pkt=%d, buf_remain=%d, file_remain=%d",
            mtu, max_data, pkt_size, buf_remain, remain_size);

    /* 不能超过缓存剩余空间（关键：确保1KB缓存不溢出） */
    if (pkt_size > buf_remain)
    {
        pkt_size = (buf_remain / 128) * 128;
        if (pkt_size < 128)
            pkt_size = 128; /* 最小128，如果buf_remain<128会在后续处理 */
        LOG_DBG("DFU calc: limited by buf_remain, new_pkt=%d", pkt_size);
    }

    /* 不能超过文件剩余空间 */
    if (pkt_size > remain_size)
    {
        pkt_size = (uint16_t)remain_size;
        LOG_DBG("DFU calc: limited by remain_size, new_pkt=%d", pkt_size);
    }

    /* AES加密需要16字节对齐（最后一包特殊处理：向上取整到16字节） */
    if (pkt_size % 16 != 0)
    {
        uint16_t aligned_size = ((pkt_size + 15) / 16) * 16;
        LOG_DBG("DFU calc: align to 16B, %d -> %d", pkt_size, aligned_size);
        pkt_size = aligned_size;
    }

    LOG_DBG("DFU calc result: pkt_size=%d", pkt_size);
    return pkt_size;
}

/********************************************************************
**函数名称:  jimi_dfu_image_down_req
**入口参数:  addr    ---   请求的文件地址
**           length  ---   请求的数据长度
**出口参数:  无
**函数功能:  发送镜像数据下载请求
**返 回 值:  无
*********************************************************************/
static void jimi_dfu_image_down_req(uint32_t addr, uint32_t length)
{
    uint8_t rsp_buf[10] = {0};

    rsp_buf[0] = addr & 0xFF;
    rsp_buf[1] = (addr >> 8) & 0xFF;
    rsp_buf[2] = (addr >> 16) & 0xFF;
    rsp_buf[3] = (addr >> 24) & 0xFF;
    rsp_buf[4] = length & 0xFF;
    rsp_buf[5] = (length >> 8) & 0xFF;
    rsp_buf[6] = (length >> 16) & 0xFF;
    rsp_buf[7] = (length >> 24) & 0xFF;

    /* 通过 BLE 发送响应 */
    my_ble_dfu_send_response(JIMI_DFU_FILE_IMAGE, rsp_buf, sizeof(rsp_buf));
}

/********************************************************************
**函数名称:  dfu_timer_wait_callback
**入口参数:  work  ---   工作项句柄
**出口参数:  无
**函数功能:  文件传输等待回调
**返 回 值:  无
*********************************************************************/
static void dfu_timeout_work_handler(struct k_work *work)
{
    uint8_t rsp_buf[10] = {0};

    ARG_UNUSED(work);

    rsp_buf[0] = JIMI_DFU_END_RESP_TIME_OUT;
    my_ble_dfu_send_response(JIMI_DFU_FILE_END, rsp_buf, sizeof(rsp_buf));
    my_ota_flash_close(false); /* 释放 busy/缓存, 允许后续 OTA 重新开始 */
    LOG_ERR("DFU timeout");

    /* 通知 main 线程 OTA 超时 */
    my_send_msg(MOD_BLE, MOD_MAIN, MY_MSG_DFU_TIMEOUT);
}

/********************************************************************
**函数名称:  dfu_retry_work_handler
**入口参数:  work  ---   工作项句柄
**出口参数:  无
**函数功能:  DFU 重试回调
**返 回 值:  无
*********************************************************************/
static void dfu_retry_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    uint32_t remain = s_dfu_image.fw_copy_size - s_req_file_addr;
    uint16_t buf_remain = my_ota_flash_cache_remain();
    uint16_t retry_pkt_size = jimi_dfu_calc_pkt_size(g_ble_server_mtu, remain, buf_remain);
    jimi_dfu_image_down_req(s_req_file_addr, retry_pkt_size);
    LOG_DBG("DFU retry request addr: 0x%x, size: %d", s_req_file_addr, retry_pkt_size);
    k_timer_start(&s_file_trans_wait_timer, K_MSEC(1000), K_NO_WAIT);
}

/********************************************************************
**函数名称:  dfu_timer_wait_callback
**入口参数:  work  ---   工作项句柄
**出口参数:  无
**函数功能:  文件传输等待回调
**返 回 值:  无
*********************************************************************/
static void dfu_timer_wait_callback(struct k_timer *timer)
{
    ARG_UNUSED(timer);

    if (++s_repeat_req_count <= 10)
    {
        /* 在中断上下文中不能直接调用 BLE 发送，提交工作项到统一 OTA 工作队列执行 */
        k_work_submit_to_queue(my_ota_flash_get_workq(), &s_dfu_retry_work);
    }
    else
    {
        s_repeat_req_count = 0;
        /* 提交工作项到统一 OTA 工作队列执行 */
        k_work_submit_to_queue(my_ota_flash_get_workq(), &s_dfu_timeout_work);
    }
}

/********************************************************************
**函数名称:  jimi_dfu_start
**入口参数:  data  ---   数据缓冲区
**           len   ---   数据长度
**出口参数:  无
**函数功能:  开始 DFU 升级
**返 回 值:  无
*********************************************************************/
static void jimi_dfu_start(uint8_t *data, uint16_t len)
{
    uint8_t rsp_buf[10] = {0};

    ARG_UNUSED(data);
    ARG_UNUSED(len);

    if (get_show_percent() < 10)
    {
        LOG_INF("DFU start, battery low");
        // TODO: 待跟APP确定OTA失败协议具体在哪个包里面

        ble_packet_trans_send((uint8_t *)"DFU start, battery low", strlen("DFU start, battery low"));
        return;
    }

    /* 蓝牙 OTA 会话互斥: 占用失败说明已有蓝牙 OTA(APP 或 4G 来源)在进行 */
    if (!my_ota_flash_occupy())
    {
        LOG_ERR("BLE DFU rejected: another OTA in progress");
        return;
    }

    s_dfu_end_flag = false;

    LOG_INF("DFU start");

    /* 发送响应 */
    my_ble_dfu_send_response(JIMI_DFU_START, rsp_buf, sizeof(rsp_buf));

    /* 通知 main 线程 OTA 开始 */
    my_send_msg(MOD_BLE, MOD_MAIN, MY_MSG_DFU_START);
}

/********************************************************************
**函数名称:  jimi_dfu_rx_file_size
**入口参数:  data  ---   数据缓冲区
**           len   ---   数据长度
**出口参数:  无
**函数功能:  接收文件大小信息
**返 回 值:  无
*********************************************************************/
static void jimi_dfu_rx_file_size(uint8_t *data, uint16_t len)
{
    uint8_t rsp_buf[10] = {0};
    uint32_t file_size;
    uint16_t first_pkt_size; // 第一包数据大小临时数据
    uint32_t partition_size; // 分区大小临时数据

    ARG_UNUSED(len);

    /* 解析文件大小（小端） */
    file_size = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);

    /* 获取 image-1 分区地址 */
    s_dfu_image.fw_copy_src_addr = FLASH_AREA_OFFSET(DFU_FLASH_PARTITION);
    s_dfu_image.fw_copy_dst_addr = FLASH_AREA_OFFSET(image_0); /* 目标地址 */
    s_dfu_image.fw_copy_size = file_size;

    /* APP 端发送的 MD5 值 (data[10]~data[25])，因 NCS 3.2.1 废弃 MD5 支持，
     * 设备端改用片段 CRC16 校验替代，此处仅保留注释说明协议兼容性。
     * 如需使用 MD5：memcpy(dfu_file_md5, data + 10, FILE_MD5_BUF_LEN);
     */

    LOG_INF("DFU file size: %d bytes, addr: 0x%x", file_size, s_dfu_image.fw_copy_src_addr);

    /* 检查大小 */
    partition_size = FLASH_AREA_SIZE(DFU_FLASH_PARTITION);
    if (file_size > partition_size)
    {
        LOG_ERR("DFU file too large: %d > %d", file_size, partition_size);
        rsp_buf[0] = JIMI_DFU_END_RESP_SIZE;
        my_ble_dfu_send_response(JIMI_DFU_FILE_END, rsp_buf, sizeof(rsp_buf));
        my_ota_flash_close(false); /* 释放会话占用 */
        return;
    }

    /* 开始 OTA 落盘: 擦除分区 + 初始化 1KB 缓存 */
    if (my_ota_flash_open(file_size) != 0)
    {
        LOG_ERR("DFU flash open/erase failed");
        rsp_buf[0] = JIMI_DFU_END_RESP_ERROR;
        my_ble_dfu_send_response(JIMI_DFU_FILE_END, rsp_buf, sizeof(rsp_buf));
        my_ota_flash_close(false); /* 释放会话占用 */
        return;
    }
    LOG_INF("DFU flash erase complete");

    /* 请求第一包数据 - 动态计算分包大小（初始缓存为空） */
    first_pkt_size = jimi_dfu_calc_pkt_size(g_ble_server_mtu, file_size, my_ota_flash_cache_remain());
    jimi_dfu_image_down_req(0x00, first_pkt_size);
    s_req_file_addr = 0x00;
    s_repeat_req_count = 0;

    LOG_INF("DFU first request addr: 0x%x, size: %d", s_req_file_addr, first_pkt_size);

    /* 启动超时定时器 */
    k_timer_start(&s_file_trans_wait_timer, K_MSEC(3000), K_NO_WAIT);
}

/********************************************************************
**函数名称:  jimi_dfu_write_image
**入口参数:  data  ---   数据缓冲区
**           len   ---   数据长度
**出口参数:  无
**函数功能:  写入镜像数据
**返 回 值:  无
*********************************************************************/
static void jimi_dfu_write_image(uint8_t *data, uint16_t len)
{
    uint8_t rsp_buf[10] = {0};
    uint32_t wrt_addr;
    uint32_t wrt_len = 0;
    uint16_t rx_crc;
    uint16_t calc_crc;
    bool is_last = false;       // 是否是最后一包(由 my_ota_flash_write_block 输出)
    uint16_t buf_remain;        // 缓存剩余空间
    uint32_t remain = 0;        // 剩余数据大小
    uint16_t next_pkt_size = 0; // 下一包数据大小临时数据

    /* 解析地址和长度（协议：起始地址4B + 片段长度4B + 片段内容NB + CRC2B） */
    wrt_addr = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24); // 起始地址（little-endian）
    wrt_len = data[4] | (data[5] << 8) | (data[6] << 16) | (data[7] << 24);  // 片段长度（little-endian）
    rx_crc = (data[8 + wrt_len] << 8) | data[9 + wrt_len];                   // CRC（little-endian）几米自定义 BLE 3.0 DFU 协议解析(Jimi Iot 蓝牙通信协议V3.1.6_2026-3-5)写的是大端模式，但是与APP的联调是小端模式

    /* 计算接收数据的 CRC16（仅校验片段内容） */
    calc_crc = my_crc16_calc(data + 8, wrt_len, CRC16_POLYNOMIAL);

    if (len != (wrt_len + 10))
    {
        LOG_ERR("DFU write len error: %d != %d + 10", len, wrt_len);
        return;
    }
    else if (calc_crc != rx_crc)
    {
        LOG_ERR("DFU write CRC error: calc=0x%x, rx=0x%x", calc_crc, rx_crc);
        return;
    }
    else if (wrt_addr != s_req_file_addr)
    {
        LOG_ERR("DFU write addr error: rx=0x%x, req=0x%x", wrt_addr, s_req_file_addr);
        return;
    }

    /* 停止超时定时器 */
    k_timer_stop(&s_file_trans_wait_timer);

    LOG_INF("DFU write: addr=0x%x, len=%d", wrt_addr, wrt_len);

    /* 写入 1KB 聚合缓存(满块/末块自动刷写 Flash 并 CRC 读回校验), is_last 由模块内部判定 */
    if (my_ota_flash_write_block(data + 8, wrt_len, &is_last) != 0)
    {
        LOG_ERR("DFU 1KB buffer write/flush failed");
        rsp_buf[0] = JIMI_DFU_END_RESP_ERROR;
        my_ble_dfu_send_response(JIMI_DFU_FILE_END, rsp_buf, sizeof(rsp_buf));
        my_ota_flash_close(false); /* 释放 busy/缓存 */
        /* 通知 main 线程 OTA 失败 */
        my_send_msg(MOD_BLE, MOD_MAIN, MY_MSG_DFU_FAIL);
        return;
    }

    if (is_last)
    {
        rsp_buf[0] = JIMI_DFU_END_RESP_OK;
        s_dfu_end_flag = true;
        LOG_INF("DFU image write complete");

        my_ble_dfu_send_response(JIMI_DFU_FILE_END, rsp_buf, sizeof(rsp_buf));

        /* 正常收尾: 刷新残留缓存并释放会话 */
        my_ota_flash_close(true);

        /* 请求 MCUboot 升级（必须在线程上下文调用） */
        boot_request_upgrade(BOOT_UPGRADE_PERMANENT);
        /* 通知 main 线程 OTA 完成 */
        my_send_msg(MOD_BLE, MOD_MAIN, MY_MSG_DFU_COMPLETE);

        my_ota_flash_schedule_reset(6500);
    }
    else
    {
        /* 请求下一包 - 动态计算分包大小 */
        s_repeat_req_count = 0;
        s_req_file_addr = wrt_addr + wrt_len;
        remain = s_dfu_image.fw_copy_size - s_req_file_addr;
        buf_remain = my_ota_flash_cache_remain();
        next_pkt_size = jimi_dfu_calc_pkt_size(g_ble_server_mtu, remain, buf_remain);
        jimi_dfu_image_down_req(s_req_file_addr, next_pkt_size);

        LOG_INF("DFU next request addr: 0x%x, size: %d", s_req_file_addr, next_pkt_size);
        k_timer_start(&s_file_trans_wait_timer, K_MSEC(3000), K_NO_WAIT);
    }
}

/********************************************************************
**函数名称:  jimi_dfu_end_image
**入口参数:  data  ---   数据缓冲区
**           len   ---   数据长度
**出口参数:  无
**函数功能:  结束 DFU 升级
**返 回 值:  无
*********************************************************************/
static void jimi_dfu_end_image(uint8_t *data, uint16_t len)
{
    ARG_UNUSED(data);
    ARG_UNUSED(len);

    LOG_INF("DFU end image");

    k_timer_stop(&s_file_trans_wait_timer);

    /* 协议层收到结束包: 放弃残留缓存并释放会话(幂等) */
    my_ota_flash_close(false);

    /* 启动复位定时器 */
    my_ota_flash_schedule_reset(1000);
}

/* 命令处理表 */
jimi_dfu_handler_table_t g_jimi_dfu_handler_tbl[] = {
    {JIMI_DFU_START,      jimi_dfu_start       },
    {JIMI_DFU_FILE_SIZE,  jimi_dfu_rx_file_size},
    {JIMI_DFU_FILE_IMAGE, jimi_dfu_write_image },
    {JIMI_DFU_FILE_END,   jimi_dfu_end_image   },
    {0x00,                NULL                 },
};

/********************************************************************
**函数名称:  jimi_dfu_cmd_handler
**入口参数:  cmd   ---   命令码
**           data  ---   数据缓冲区
**           len   ---   数据长度
**出口参数:  无
**函数功能:  DFU 命令分发处理
**返 回 值:  无
*********************************************************************/
void jimi_dfu_cmd_handler(uint8_t cmd, uint8_t *data, uint16_t len)
{
    uint8_t i = 0;

    while (g_jimi_dfu_handler_tbl[i].opcode != 0x00)
    {
        if (g_jimi_dfu_handler_tbl[i].opcode == cmd)
        {
            if (g_jimi_dfu_handler_tbl[i].cmd_handler)
            {
                g_jimi_dfu_handler_tbl[i].cmd_handler(data, len);
            }
            break;
        }
        i++;
    }

    if (g_jimi_dfu_handler_tbl[i].opcode == 0x00)
    {
        LOG_WRN("DFU unknown cmd: 0x%x", cmd);
    }
}

/********************************************************************
**函数名称:  jimi_dfu_timer_init
**入口参数:  无
**出口参数:  无
**函数功能:  初始化 DFU 定时器
**返 回 值:  无
*********************************************************************/
void jimi_dfu_timer_init(void)
{
    k_timer_init(&s_file_trans_wait_timer, dfu_timer_wait_callback, NULL);
    k_work_init(&s_dfu_timeout_work, dfu_timeout_work_handler);
    k_work_init(&s_dfu_retry_work, dfu_retry_work_handler);

    /* BLE 协议 work(超时/重试)提交到统一 OTA workq(my_ota_flash_get_workq), 该队列由 my_ota_flash_init 启动 */
    LOG_INF("DFU timers initialized");
}
