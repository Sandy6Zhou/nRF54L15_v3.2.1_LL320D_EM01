/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        my_ota_flash.c
**文件描述:        统一 OTA Flash 落盘模块
**当前版本:        V1.0
**作    者:        周森达 (zhousenda@jimiiot.com)
**完成日期:        2026.08.11
*********************************************************************
** 功能描述:        1. 统一 image_1(mcuboot_secondary) 分区擦/写/读原语
**                 2. 共享 1KB 聚合写缓存 + 写前/读回 CRC16 校验
**                 3. OTA 会话互斥(busy) 与复位调度链路
**                 4. 供 BLE DFU(my_dfu_jimi) 与 LTE YModem OTA(my_dfu_lte) 共用
** 设计要点:        同一时刻仅存在一个 OTA 会话(occupy 经原子 CAS 保证唯一), 故 1KB
**                 聚合缓存为单实例静态; 读回校验缓冲同样静态化, 避免每块动态分配
*********************************************************************/

/* 必须在包含 my_comm.h 之前定义 BLE_LOG_MODULE_ID，避免与 my_ble_log.h 中的默认定义冲突 */
#define BLE_LOG_MODULE_ID BLE_LOG_MOD_DFU

#include "my_comm.h"

/* 日志模块注册 */
LOG_MODULE_REGISTER(ota_flash, LOG_LEVEL_INF);

/* Flash 分区定义 (与 BLE DFU / LTE OTA 一致) */
#define MY_OTA_FLASH_PARTITION image_1

/* nRF54L15 Flash 扇区大小为 4KB */
#define FLASH_SECTOR_SIZE 4096

/**
 * @brief 1KB 聚合写缓存
 * 必须小于 FLASH_SECTOR_SIZE(4096) 的大小，且 FLASH_SECTOR_SIZE 必须是它的整倍数
 */
#define MY_OTA_BUFFER_SIZE 1024

/* CRC16 多项式 (与 APP / YModem 端保持一致) */
#define MY_OTA_CRC16_POLYNOMIAL 0xA001

/* OTA 落盘会话状态 (occupy 经原子 CAS 置位, close 清除, 保证唯一会话; 初始为空闲) */
static atomic_t s_ota_busy = ATOMIC_INIT(0);

/* 1KB 聚合写缓存 (单实例静态, 串行安全; 读回校验复用本缓存, 写入完成后覆盖) */
static uint8_t s_ota_buf[MY_OTA_BUFFER_SIZE];      /* 1KB 数据缓存 */
static uint16_t s_ota_buf_offset;                  /* 缓存内当前偏移 */
static uint32_t s_ota_block_addr;                  /* 当前 1KB 块在分区内偏移 */
static uint32_t s_ota_total_written;               /* 已写入 Flash 的总字节数 */
static uint32_t s_ota_file_size;                   /* 目标固件总大小 */

/* 通用 OTA workq: 服务 BLE 协议 work(超时/重试, 含 BLE 发送链)与复位 work.
 * 复位 handler 链(IMU 零偏->zms_write, ZMS_BLOCK_SIZE=32)峰值约 1.2KB, */
static K_THREAD_STACK_DEFINE(s_ota_workq_stack, 2048);
static struct k_work_q s_ota_workq;

/* 复位调度 (保存 IMU 零偏后软复位), 提交到上述通用 workq */
static struct k_timer s_ota_reset_timer;                 /* 复位调度定时器 */
static struct k_work s_ota_reset_work;                   /* 复位工作项 */

/********************************************************************
**函数名称:  my_ota_flash_erase
**入口参数:  off_set  ---   Flash 分区内偏移地址
**           size     ---   擦除大小（字节）
**出口参数:  无
**函数功能:  擦除 image_1 分区指定区域（按 4KB 扇区循环）
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
static int my_ota_flash_erase(uint32_t off_set, uint32_t size)
{
    const struct device *flash_dev = FLASH_AREA_DEVICE(MY_OTA_FLASH_PARTITION);
    uint32_t partition_offset = FLASH_AREA_OFFSET(MY_OTA_FLASH_PARTITION);
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
**函数名称:  my_ota_flash_write
**入口参数:  wrt_addr     ---   分区内写入地址
**           in_buf       ---   输入数据缓冲区
**           in_buf_size  ---   写入数据大小
**出口参数:  无
**函数功能:  写入数据到 image_1 分区 Flash
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
static int my_ota_flash_write(uint32_t wrt_addr, const void *in_buf, uint32_t in_buf_size)
{
    const struct device *flash_dev = FLASH_AREA_DEVICE(MY_OTA_FLASH_PARTITION);
    uint32_t partition_offset = FLASH_AREA_OFFSET(MY_OTA_FLASH_PARTITION);
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
**函数名称:  my_ota_flash_read
**入口参数:  off_addr      ---   偏移地址指针(读取后回写新偏移)
**           out_buf       ---   输出缓冲区
**           out_buf_size  ---   读取大小
**出口参数:  off_addr      ---   更新后的偏移地址
**函数功能:  从 image_1 分区 Flash 读取数据
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
static int my_ota_flash_read(uint32_t *off_addr, void *out_buf, uint32_t out_buf_size)
{
    const struct device *flash_dev = FLASH_AREA_DEVICE(MY_OTA_FLASH_PARTITION);
    uint32_t partition_offset = FLASH_AREA_OFFSET(MY_OTA_FLASH_PARTITION);
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
**函数名称:  my_ota_flash_flush
**入口参数:  is_last  ---   是否为最后一个 1KB 块
**出口参数:  无
**函数功能:  将 s_ota_buf 缓存写入 Flash，并读回做 CRC16 校验
**返 回 值:  0 表示成功，负值表示失败
**注意事项:  不足 1KB 时用 0xFF 填充; 校验失败返回 -EIO
*********************************************************************/
static int my_ota_flash_flush(bool is_last)
{
    int ret;
    uint16_t write_crc;
    uint16_t read_crc;
    uint32_t read_addr = s_ota_block_addr;

    if (s_ota_buf_offset == 0 && !is_last)
    {
        return 0; /* 缓存为空且非末块, 无需写入 */
    }

    /* 末块不足 1KB, 填充 0xFF */
    if (s_ota_buf_offset < MY_OTA_BUFFER_SIZE)
    {
        memset(&s_ota_buf[s_ota_buf_offset], 0xFF,
               MY_OTA_BUFFER_SIZE - s_ota_buf_offset);
    }

    /* 计算写入前 CRC */
    write_crc = my_crc16_calc(s_ota_buf, MY_OTA_BUFFER_SIZE, MY_OTA_CRC16_POLYNOMIAL);

    /* 写入 Flash */
    ret = my_ota_flash_write(s_ota_block_addr, s_ota_buf, MY_OTA_BUFFER_SIZE);
    if (ret != 0)
    {
        LOG_ERR("OTA flash write failed at 0x%x", s_ota_block_addr);
        return ret;
    }

    /* 读回验证 (复用写缓存 s_ota_buf, 写入已完成覆盖无碍, 尾部会重新填充 0xFF) */
    ret = my_ota_flash_read(&read_addr, s_ota_buf, MY_OTA_BUFFER_SIZE);
    if (ret != 0)
    {
        LOG_ERR("OTA flash read back failed at 0x%x", s_ota_block_addr);
        return ret;
    }

    /* 计算读回 CRC */
    read_crc = my_crc16_calc(s_ota_buf, MY_OTA_BUFFER_SIZE, MY_OTA_CRC16_POLYNOMIAL);

    if (write_crc != read_crc)
    {
        LOG_ERR("OTA 1KB buffer CRC verify failed at 0x%x: write=0x%04x, read=0x%04x",
                s_ota_block_addr, write_crc, read_crc);
        return -EIO;
    }

    LOG_INF("OTA 1KB buffer flushed: addr=0x%x, crc=0x%04x", s_ota_block_addr, read_crc);

    /* 准备下一个 1KB 块 */
    s_ota_block_addr += MY_OTA_BUFFER_SIZE;
    s_ota_buf_offset = 0;
    memset(s_ota_buf, 0xFF, MY_OTA_BUFFER_SIZE);

    return 0;
}

/********************************************************************
**函数名称:  ota_reset_work_handler
**入口参数:  work  ---   工作项句柄
**出口参数:  无
**函数功能:  OTA 复位工作项处理: 保存 IMU 零偏并软复位系统
**返 回 值:  无
*********************************************************************/
static void ota_reset_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    my_gsensor_save_imu_bias(); /* 保存 IMU 零偏 */

    k_sleep(K_MSEC(500));       /* 等待数据写入完成 */

    sys_reboot(SYS_REBOOT_WARM);
}

/********************************************************************
**函数名称:  ota_reset_callback
**入口参数:  timer  ---   定时器句柄
**出口参数:  无
**函数功能:  OTA 复位定时器回调: 提交工作项到注册的复位工作队列
**返 回 值:  无
*********************************************************************/
static void ota_reset_callback(struct k_timer *timer)
{
    ARG_UNUSED(timer);

    k_work_submit_to_queue(&s_ota_workq, &s_ota_reset_work);
}

/********************************************************************
**函数名称:  my_ota_flash_init
**入口参数:  无
**出口参数:  无
**函数功能:  初始化 OTA Flash 模块(复位定时器/工作项)
**返 回 值:  无
**注意事项:  幂等(可重复调用)
*********************************************************************/
void my_ota_flash_init(void)
{
    k_timer_init(&s_ota_reset_timer, ota_reset_callback, NULL);
    k_work_init(&s_ota_reset_work, ota_reset_work_handler);

    /* 启动通用 OTA 工作队列, 使用较高优先级 */
    k_work_queue_start(&s_ota_workq, s_ota_workq_stack, K_THREAD_STACK_SIZEOF(s_ota_workq_stack),
                       K_PRIO_PREEMPT(1), NULL);

    LOG_INF("OTA flash module initialized");
}

/********************************************************************
**函数名称:  my_ota_flash_get_workq
**入口参数:  无
**出口参数:  无
**函数功能:  获取通用 OTA 工作队列指针(my_ota_flash 自持)
**返 回 值:  通用 OTA workq 指针(由 my_ota_flash_init 启动, 非 NULL)
**注意事项:  供 BLE 协议层提交超时/重试等工作项; 复位工作由模块内部直接使用该队列
*********************************************************************/
struct k_work_q *my_ota_flash_get_workq(void)
{
    return &s_ota_workq;
}

/********************************************************************
**函数名称:  my_ota_flash_occupy
**入口参数:  无
**出口参数:  无
**函数功能:  占用一次蓝牙 OTA 会话(APP BLE DFU 与 4G YModem OTA 共用互斥)
**返 回 值:  true=占用成功; false=已有 OTA 会话在进行中
**注意事项:  协议层在会话开始时调用(开始接收数据前), 由 my_ota_flash_close 释放;
**          经原子 CAS 保证并发下仅一个调用方成功(BLE/LTE 线程并发安全)
*********************************************************************/
bool my_ota_flash_occupy(void)
{
    /* 原子 compare-and-swap: 原值为 0 时置 1 并返回 true(占用成功);
     * 已被占用时值不变, 返回 false */
    return atomic_cas(&s_ota_busy, 0, 1);
}

/********************************************************************
**函数名称:  my_ota_flash_open
**入口参数:  file_size  ---   本次 OTA 固件总大小(字节)
**出口参数:  无
**函数功能:  开始 OTA 落盘: 要求会话已占用(my_ota_flash_occupy), 擦除分区并初始化缓存
**返 回 值:  0 表示成功; -EBUSY 会话未占用; -EFBIG 固件过大; -EIO 擦除失败
**注意事项:  擦除失败保持会话占用, 由协议层调 my_ota_flash_close(false) 释放
*********************************************************************/
int my_ota_flash_open(uint32_t file_size)
{
    if (!atomic_get(&s_ota_busy))
    {
        LOG_ERR("OTA flash open: session not occupied");
        return -EBUSY;
    }

    if (file_size == 0)
    {
        LOG_ERR("OTA flash open: invalid file size 0");
        return -EINVAL;
    }

    if (file_size > FLASH_AREA_SIZE(MY_OTA_FLASH_PARTITION))
    {
        LOG_ERR("OTA flash open: file too large: %d > %d", file_size,
                FLASH_AREA_SIZE(MY_OTA_FLASH_PARTITION));
        return -EFBIG;
    }

    if (my_ota_flash_erase(0, file_size) != 0)
    {
        LOG_ERR("OTA flash erase failed");
        return -EIO; /* busy 保持占用, 由协议层 close(false) 释放 */
    }

    /* 初始化 1KB 缓存状态 */
    s_ota_buf_offset    = 0;
    s_ota_block_addr    = 0;
    s_ota_total_written = 0;
    s_ota_file_size     = file_size;
    memset(s_ota_buf, 0xFF, MY_OTA_BUFFER_SIZE);

    return 0;
}

/********************************************************************
**函数名称:  my_ota_flash_write_block
**入口参数:  data     ---   待写入数据缓冲区
**           len      ---   待写入数据长度(字节)
**           is_last  ---   输出参数(可传 NULL): 本次写入后是否到达文件末尾
**出口参数:  is_last  ---   true=已达文件末尾, false=尚未
**函数功能:  写入一块数据: 聚合进 1KB 缓存, 满 1KB 或末块时内部刷写 Flash
**           并读回做 CRC16 校验
**返 回 值:  0 表示成功, 负值表示失败(-EOVERFLOW 缓存溢出)
*********************************************************************/
int my_ota_flash_write_block(const uint8_t *data, uint32_t len, bool *is_last)
{
    bool last;

    if (!atomic_get(&s_ota_busy))
    {
        return -EIO; /* 必须先 open */
    }

    if (data == NULL)
    {
        return -EINVAL;
    }

    if (len == 0)
    {
        /* 零长度结束块: 不落盘 (YModem 在固件为 1024 整倍数时会发) */
        if (is_last)
        {
            *is_last = (s_ota_total_written >= s_ota_file_size);
        }
        return 0;
    }

    if ((uint32_t)s_ota_buf_offset + len > MY_OTA_BUFFER_SIZE)
    {
        LOG_ERR("OTA write_block overflow: offset=%d len=%d", s_ota_buf_offset, len);
        return -EOVERFLOW;
    }

    /* 数据拷入 1KB 缓存 */
    memcpy(&s_ota_buf[s_ota_buf_offset], data, len);
    s_ota_buf_offset += (uint16_t)len;
    s_ota_total_written += len;

    last = (s_ota_total_written >= s_ota_file_size);
    if (is_last)
    {
        *is_last = last;
    }

    /* 缓存满 1KB 或最后一包, 写入 Flash */
    if (s_ota_buf_offset >= MY_OTA_BUFFER_SIZE || last)
    {
        return my_ota_flash_flush(last);
    }

    return 0;
}

/********************************************************************
**函数名称:  my_ota_flash_close
**入口参数:  result_ok  ---   1=会话正常结束(刷新残留缓存); 0=异常中止(放弃数据)
**出口参数:  无
**函数功能:  关闭一次 OTA 落盘会话: 按结果刷新/放弃残留缓存, 并清除 busy
**返 回 值:  0 表示成功, 负值表示刷新残留失败
**注意事项:  幂等(可重复调用); 无论成功失败都会清除 busy 与缓存状态
*********************************************************************/
int my_ota_flash_close(bool result_ok)
{
    int ret = 0;

    if (!atomic_get(&s_ota_busy))
    {
        return 0; /* 幂等 */
    }

    if (result_ok && s_ota_buf_offset > 0)
    {
        /* 正常结束: 刷新残留缓存 */
        ret = my_ota_flash_flush(true);
    }

    /* 无论成败, 都释放会话与缓存状态; 先重置缓存字段, 最后清 busy,
     * 避免新会话(atomic_cas 成功)在旧会话仍重置缓存时进入 */
    s_ota_buf_offset = 0;
    s_ota_block_addr = 0;
    s_ota_total_written = 0;
    atomic_set(&s_ota_busy, 0);

    return ret;
}

/********************************************************************
**函数名称:  my_ota_flash_cache_remain
**入口参数:  无
**出口参数:  无
**函数功能:  获取当前 1KB 聚合缓存剩余空间
**返 回 值:  缓存剩余字节数(0~1024)
**注意事项:  供 BLE DFU 动态分包计算 buf_remain 使用
*********************************************************************/
uint16_t my_ota_flash_cache_remain(void)
{
    return (uint16_t)(MY_OTA_BUFFER_SIZE - s_ota_buf_offset);
}

/********************************************************************
**函数名称:  my_ota_flash_is_busy
**入口参数:  无
**出口参数:  无
**函数功能:  检查蓝牙 OTA 会话是否在进行中(APP 或 4G 来源)
**返 回 值:  true=进行中, false=空闲
**注意事项:  由 my_ota_flash_occupy / my_ota_flash_close 经原子操作维护
*********************************************************************/
bool my_ota_flash_is_busy(void)
{
    return (atomic_get(&s_ota_busy) != 0);
}

/********************************************************************
**函数名称:  my_ota_flash_schedule_reset
**入口参数:  delay_ms  ---   复位延迟时间（毫秒）
**出口参数:  无
**函数功能:  调度 OTA 完成复位: 到期后保存 IMU 零偏并软复位系统
**返 回 值:  无
**注意事项:  供 BLE DFU 与 LTE YModem OTA 共用同一复位链路
*********************************************************************/
void my_ota_flash_schedule_reset(uint32_t delay_ms)
{
    k_timer_start(&s_ota_reset_timer, K_MSEC(delay_ms), K_NO_WAIT);
}
