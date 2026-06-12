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

/* MCUboot 升级支持 - 使用 Zephyr DFU API 替代直接引用内部头文件 */
#include <zephyr/dfu/mcuboot.h>

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
static bool s_dfu_in_progress = false;         /* DFU 进行中标志 */

/**
 * @brief DFU 1KB 缓存
 * 为了提升DFU的效率，采用1KB DFU 缓存写FLASH，缓存满1KB或最后一包数据时写入Flash
 * 必须小于FLASH_SECTOR_SIZE（4096）的大小，且FLASH_SECTOR_SIZE必须是它的整倍数
 */
#define DFU_BUFFER_SIZE 1024
static uint8_t s_dfu_buffer[DFU_BUFFER_SIZE]; /* 1KB 数据缓存 */
static uint16_t s_dfu_buf_offset = 0;         /* 缓存内当前偏移 */
static uint32_t s_dfu_block_addr = 0;         /* 当前 1KB 块起始地址 */

static struct k_timer s_file_trans_wait_timer; /* 文件传输等待定时器 */
static struct k_timer s_dfu_reset_timer;       /* DFU 重置定时器 */
static struct k_work s_dfu_timeout_work;       /* DFU 超时工作项，用于线程上下文处理 */
static struct k_work s_dfu_retry_work;         /* DFU 重试工作项，用于线程上下文处理 */
static struct k_work s_dfu_reset_work;         /* DFU 复位工作项，用于线程上下文处理 */

/* 自定义工作队列，避免系统工作队列栈溢出 */
static K_THREAD_STACK_DEFINE(dfu_workq_stack, 4096);
static struct k_work_q s_dfu_workq;

/* 函数声明 */
static void dfu_timer_wait_callback(struct k_timer *timer);                               /* 文件传输等待定时器回调函数 */
static void dfu_finish_wait_callback(struct k_timer *timer);                              /* DFU 完成等待定时器回调函数 */
static void dfu_reset_callback(struct k_timer *timer);                                    /* DFU 重置定时器回调函数 */
static void dfu_timeout_work_handler(struct k_work *work);                                /* DFU 超时工作项处理函数 */
static void dfu_retry_work_handler(struct k_work *work);                                  /* DFU 重试工作项处理函数 */
static void dfu_reset_work_handler(struct k_work *work);                                  /* DFU 复位工作项处理函数 */
static int jimi_dfu_flash_read(uint32_t *off_addr, void *out_buf, uint32_t out_buf_size); /* Flash 读取函数 */

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
**函数名称:  jimi_dfu_flash_erase
**入口参数:  off_set  ---   Flash 偏移地址
**           size     ---   擦除大小
**出口参数:  无
**函数功能:  擦除 Flash 区域
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
static int jimi_dfu_flash_erase(uint32_t off_set, uint32_t size)
{
    const struct device *flash_dev = FLASH_AREA_DEVICE(DFU_FLASH_PARTITION);
    uint32_t partition_offset = FLASH_AREA_OFFSET(DFU_FLASH_PARTITION);
    int ret;
    uint32_t erase_addr = partition_offset + off_set;
    uint32_t erase_size = size;

    if (!flash_dev)
    {
        LOG_ERR("Flash device not found");
        return -ENODEV;
    }

    while (erase_size > 0)
    {
        ret = flash_erase(flash_dev, erase_addr, FLASH_SECTOR_SIZE);
        if (ret != 0)
        {
            LOG_ERR("Flash erase failed at 0x%x, ret=%d", erase_addr, ret);
            return ret;
        }

        if (erase_size > FLASH_SECTOR_SIZE)
        {
            erase_size -= FLASH_SECTOR_SIZE;
            erase_addr += FLASH_SECTOR_SIZE;
        }
        else
        {
            break;
        }

        k_usleep(10);
    }

    return 0;
}

/********************************************************************
**函数名称:  jimi_dfu_flash_write
**入口参数:  wrt_addr     ---   写入地址
**           in_buf       ---   输入数据缓冲区
**           in_buf_size  ---   输入数据大小
**出口参数:  无
**函数功能:  写入数据到 Flash
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
static int jimi_dfu_flash_write(uint32_t wrt_addr, const void *in_buf, uint32_t in_buf_size)
{
    const struct device *flash_dev = FLASH_AREA_DEVICE(DFU_FLASH_PARTITION);
    uint32_t partition_offset = FLASH_AREA_OFFSET(DFU_FLASH_PARTITION);
    uint32_t abs_addr = partition_offset + wrt_addr;
    int ret;

    if (!flash_dev)
    {
        LOG_ERR("Flash device not found");
        return -ENODEV;
    }

    ret = flash_write(flash_dev, abs_addr, in_buf, in_buf_size);
    if (ret != 0)
    {
        LOG_ERR("Flash write failed at 0x%x, ret=%d", wrt_addr, ret);
        return ret;
    }

    return 0;
}

/********************************************************************
**函数名称:  jimi_dfu_flush_buffer
**入口参数:  is_last  ---   是否是最后一包
**出口参数:  无
**函数功能:  将 s_dfu_buffer 缓存写入 Flash，包含 CRC 校验
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
static int jimi_dfu_flush_buffer(bool is_last)
{
    int ret;
    uint16_t write_crc, read_crc;
    uint32_t read_addr = s_dfu_block_addr;
    uint8_t *read_back_buf = NULL;

    if (s_dfu_buf_offset == 0 && !is_last)
    {
        return 0; /* 缓存为空且不是最后一包，无需写入 */
    }

    /* 动态分配读回缓冲区 */
    MY_MALLOC_BUFFER(read_back_buf, DFU_BUFFER_SIZE);
    if (read_back_buf == NULL)
    {
        LOG_ERR("Failed to allocate read_back_buf");
        return -ENOMEM;
    }

    /* 最后一包不足 1KB，填充 0xFF */
    if (s_dfu_buf_offset < DFU_BUFFER_SIZE)
    {
        memset(&s_dfu_buffer[s_dfu_buf_offset], 0xFF, DFU_BUFFER_SIZE - s_dfu_buf_offset);
    }

    /* 计算写入前 CRC */
    write_crc = my_crc16_calc(s_dfu_buffer, DFU_BUFFER_SIZE, CRC16_POLYNOMIAL);

    /* 写入 Flash */
    ret = jimi_dfu_flash_write(s_dfu_block_addr, s_dfu_buffer, DFU_BUFFER_SIZE);
    if (ret != 0)
    {
        LOG_ERR("1KB buffer flash write failed at 0x%x", s_dfu_block_addr);
        MY_FREE_BUFFER(read_back_buf);
        return ret;
    }

    /* 读回验证 */
    ret = jimi_dfu_flash_read(&read_addr, read_back_buf, DFU_BUFFER_SIZE);
    if (ret != 0)
    {
        LOG_ERR("1KB buffer flash read back failed at 0x%x", s_dfu_block_addr);
        MY_FREE_BUFFER(read_back_buf);
        return ret;
    }

    /* 计算读回 CRC */
    read_crc = my_crc16_calc(read_back_buf, DFU_BUFFER_SIZE, CRC16_POLYNOMIAL);

    if (write_crc != read_crc)
    {
        LOG_ERR("1KB buffer CRC verify failed at 0x%x: write=0x%04x, read=0x%04x",
                s_dfu_block_addr, write_crc, read_crc);
        MY_FREE_BUFFER(read_back_buf);
        return -EIO;
    }

    LOG_INF("1KB buffer flushed: addr=0x%x, crc=0x%04x", s_dfu_block_addr, read_crc);

    /* 准备下一个 1KB 块 */
    s_dfu_block_addr += DFU_BUFFER_SIZE;
    s_dfu_buf_offset = 0;
    memset(s_dfu_buffer, 0xFF, DFU_BUFFER_SIZE);

    MY_FREE_BUFFER(read_back_buf);
    return 0;
}

/********************************************************************
**函数名称:  jimi_dfu_flash_read
**入口参数:  off_addr      ---   偏移地址指针
**           out_buf       ---   输出缓冲区
**           out_buf_size  ---   读取大小
**出口参数:  off_addr      ---   更新后的偏移地址
**函数功能:  从 Flash 读取数据
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
int jimi_dfu_flash_read(uint32_t *off_addr, void *out_buf, uint32_t out_buf_size)
{
    const struct device *flash_dev = FLASH_AREA_DEVICE(DFU_FLASH_PARTITION);
    uint32_t partition_offset = FLASH_AREA_OFFSET(DFU_FLASH_PARTITION);
    uint32_t abs_addr = partition_offset + *off_addr;
    int ret;

    if (!flash_dev)
    {
        LOG_ERR("Flash device not found");
        return -ENODEV;
    }

    ret = flash_read(flash_dev, abs_addr, out_buf, out_buf_size);
    if (ret != 0)
    {
        LOG_ERR("Flash read failed at 0x%x (abs:0x%x), ret=%d", *off_addr, abs_addr, ret);
        return ret;
    }

    *off_addr += out_buf_size;
    return 0;
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
**函数名称:  dfu_reset_work_handler
**入口参数:  work  ---   工作项句柄
**出口参数:  无
**函数功能:  DFU 复位回调函数
**返 回 值:  无
*********************************************************************/
static void dfu_reset_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    sys_reboot(SYS_REBOOT_WARM);
}

/********************************************************************
**函数名称:  dfu_reset_callback
**入口参数:  timer  ---   定时器句柄
**出口参数:  无
**函数功能:  DFU 复位回调
**返 回 值:  无
*********************************************************************/
static void dfu_reset_callback(struct k_timer *timer)
{
    ARG_UNUSED(timer);

    /* 提交到工作队列，在线程上下文执行复位 */
    k_work_submit_to_queue(&s_dfu_workq, &s_dfu_reset_work);
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
    s_dfu_in_progress = false;
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
    uint16_t buf_remain = DFU_BUFFER_SIZE - s_dfu_buf_offset;
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
        /* 在中断上下文中不能直接调用 BLE 发送，提交工作项到自定义工作队列执行 */
        k_work_submit_to_queue(&s_dfu_workq, &s_dfu_retry_work);
    }
    else
    {
        s_repeat_req_count = 0;
        /* 提交工作项到自定义工作队列执行 */
        k_work_submit_to_queue(&s_dfu_workq, &s_dfu_timeout_work);
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

    s_dfu_end_flag = false;
    s_dfu_in_progress = true;

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
        s_dfu_in_progress = false;
        return;
    }

    /* 擦除 Flash */
    if (jimi_dfu_flash_erase(0, file_size) != 0)
    {
        LOG_ERR("DFU flash erase failed");
        rsp_buf[0] = JIMI_DFU_END_RESP_ERROR;
        my_ble_dfu_send_response(JIMI_DFU_FILE_END, rsp_buf, sizeof(rsp_buf));
        s_dfu_in_progress = false;
        return;
    }
    LOG_INF("DFU flash erase complete");

    /* 初始化 1KB 缓存 */
    s_dfu_block_addr = 0;
    s_dfu_buf_offset = 0;
    memset(s_dfu_buffer, 0xFF, DFU_BUFFER_SIZE);

    /* 请求第一包数据 - 动态计算分包大小（初始缓存为空） */
    first_pkt_size = jimi_dfu_calc_pkt_size(g_ble_server_mtu, file_size, DFU_BUFFER_SIZE);
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
    bool is_last = false;       // 是否是最后一包
    uint16_t buf_pos = 0;       // 缓存位置
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

    /* 将数据存入 1KB 缓存 */
    buf_pos = wrt_addr % DFU_BUFFER_SIZE;
    if (buf_pos + wrt_len > DFU_BUFFER_SIZE)
    {
        LOG_ERR("DFU buffer overflow: pos=%d, len=%d", buf_pos, wrt_len);
        rsp_buf[0] = JIMI_DFU_END_RESP_ERROR;
        my_ble_dfu_send_response(JIMI_DFU_FILE_END, rsp_buf, sizeof(rsp_buf));
        s_dfu_in_progress = false;
        /* 通知 main 线程 OTA 失败 */
        my_send_msg(MOD_BLE, MOD_MAIN, MY_MSG_DFU_FAIL);
        return;
    }
    memcpy(&s_dfu_buffer[buf_pos], data + 8, wrt_len);
    s_dfu_buf_offset = buf_pos + wrt_len;

    /* 检查是否完成 */
    is_last = (wrt_addr + wrt_len) >= s_dfu_image.fw_copy_size;

    /* 缓存满 1KB 或最后一包，写入 Flash */
    if (s_dfu_buf_offset >= DFU_BUFFER_SIZE || is_last)
    {
        if (jimi_dfu_flush_buffer(is_last) != 0)
        {
            LOG_ERR("DFU 1KB buffer flush failed");
            rsp_buf[0] = JIMI_DFU_END_RESP_ERROR;
            my_ble_dfu_send_response(JIMI_DFU_FILE_END, rsp_buf, sizeof(rsp_buf));
            s_dfu_in_progress = false;
            /* 通知 main 线程 OTA 失败 */
            my_send_msg(MOD_BLE, MOD_MAIN, MY_MSG_DFU_FAIL);
            return;
        }
    }

    if (is_last)
    {
        rsp_buf[0] = JIMI_DFU_END_RESP_OK;
        s_dfu_end_flag = true;
        LOG_INF("DFU image write complete");

        my_ble_dfu_send_response(JIMI_DFU_FILE_END, rsp_buf, sizeof(rsp_buf));

        /* 请求 MCUboot 升级（必须在线程上下文调用） */
        boot_request_upgrade(BOOT_UPGRADE_PERMANENT);
        /* 通知 main 线程 OTA 完成 */
        my_send_msg(MOD_BLE, MOD_MAIN, MY_MSG_DFU_COMPLETE);

        k_timer_start(&s_dfu_reset_timer, K_MSEC(6500), K_NO_WAIT);
    }
    else
    {
        /* 请求下一包 - 动态计算分包大小 */
        s_repeat_req_count = 0;
        s_req_file_addr = wrt_addr + wrt_len;
        remain = s_dfu_image.fw_copy_size - s_req_file_addr;
        buf_remain = DFU_BUFFER_SIZE - s_dfu_buf_offset;
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

    /* 启动复位定时器 */
    k_timer_start(&s_dfu_reset_timer, K_MSEC(1000), K_NO_WAIT);
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
    k_timer_init(&s_dfu_reset_timer, dfu_reset_callback, NULL);
    k_work_init(&s_dfu_timeout_work, dfu_timeout_work_handler);
    k_work_init(&s_dfu_retry_work, dfu_retry_work_handler);
    k_work_init(&s_dfu_reset_work, dfu_reset_work_handler);

    /* 启动自定义工作队列，使用较高优先级 */
    k_work_queue_start(&s_dfu_workq, dfu_workq_stack, K_THREAD_STACK_SIZEOF(dfu_workq_stack),
                       K_PRIO_PREEMPT(1), NULL);

    LOG_INF("DFU timers and workq initialized");
}

/********************************************************************
**函数名称:  jimi_dfu_is_in_progress
**入口参数:  无
**出口参数:  无
**函数功能:  检查 DFU OTA 是否正在进行中
**返 回 值:  true=进行中, false=未进行
**注意事项:  DFU 期间应禁用蓝牙日志发送，避免干扰 OTA 传输
*********************************************************************/
bool jimi_dfu_is_in_progress(void)
{
    return s_dfu_in_progress;
}
