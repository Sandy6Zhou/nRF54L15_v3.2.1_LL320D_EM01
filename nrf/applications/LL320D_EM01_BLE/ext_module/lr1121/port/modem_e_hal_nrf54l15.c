#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "modem_e_hal.h"
#include "modem_e_modem.h"
#include "modem_e_modem_hal.h"

LOG_MODULE_REGISTER(lr1121_hal, LOG_LEVEL_INF);

/* 仅在定位特定 SPI 事务问题时开启，正常构建默认关闭。 */
#ifndef LR1121_HAL_DEBUG_ENABLE
#define LR1121_HAL_DEBUG_ENABLE 0
#endif

/* 等待 BUSY 变为目标电平的通用软件超时上限。 */
#define LR1121_BUSY_TIMEOUT_MS 1000U
/* 唤醒前等待 BUSY 进入高电平的超时上限。 */
#define LR1121_BUSY_HIGH_TIMEOUT_MS 5000U
/* 数据手册要求 NRESET 低电平超过 100 us，这里使用 1 ms 裕量。 */
#define LR1121_RESET_PULSE_US 1000U
/* 板级实测复位释放后约 300 ms 才能唤醒，这里保留 500 ms 启动裕量。 */
#define LR1121_RESET_SETTLE_MS 500U
/* 数据手册规定 NSS 唤醒低电平保持 100 us。 */
#define LR1121_WAKEUP_PULSE_US 100U
/* 等待 LR1121 上报复位事件的总超时上限。 */
#define LR1121_RESET_EVENT_TIMEOUT_MS 5000U
/* 复位事件轮询间隔。 */
#define LR1121_RESET_EVENT_POLL_MS 10U
/* NOP 读取缓冲区最大长度，防止响应帧超过静态缓冲区。 */
#define LR1121_NOP_BUFFER_SIZE 255U

/* BUSY 和 RESET 由 LR1121 管理；常规 CS 由 SPI 控制器管理。 */
#define LR1121_NODE DT_NODELABEL(lr1121)

static const struct spi_dt_spec lr1121_spi =
    SPI_DT_SPEC_GET(LR1121_NODE, SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0);
static const struct gpio_dt_spec lr1121_busy = GPIO_DT_SPEC_GET(LR1121_NODE, busy_gpios);
static const struct gpio_dt_spec lr1121_reset = GPIO_DT_SPEC_GET(LR1121_NODE, reset_gpios);
static const struct gpio_dt_spec lr1121_nss =
    GPIO_DT_SPEC_GET_BY_IDX(DT_PARENT(LR1121_NODE), cs_gpios, 0);

/*********************************************************************
**函数名称:  lr1121_ready
**入口参数:  无
**出口参数:  无
**函数功能:  检查并初始化 LR1121 使用的 SPI、BUSY、RESET 和 NSS 引脚
**返 回 值:  0 表示成功，负值表示初始化失败
*********************************************************************/
static int lr1121_ready(void)
{
    int err;

    if (!spi_is_ready_dt(&lr1121_spi) || !gpio_is_ready_dt(&lr1121_busy) ||
        !gpio_is_ready_dt(&lr1121_reset) || !gpio_is_ready_dt(&lr1121_nss))
    {
        LOG_ERR("LR1121 device is not ready: %d", -ENODEV);
        return -ENODEV;
    }

    err = gpio_pin_configure_dt(&lr1121_busy, GPIO_INPUT);
    if (err != 0)
    {
        LOG_ERR("LR1121 BUSY GPIO configuration failed: %d", err);
        return err;
    }

    err = gpio_pin_configure_dt(&lr1121_reset, GPIO_OUTPUT_INACTIVE);
    if (err != 0)
    {
        LOG_ERR("LR1121 RESET GPIO configuration failed: %d", err);
        return err;
    }

    err = gpio_pin_configure_dt(&lr1121_nss, GPIO_OUTPUT_INACTIVE);
    if (err != 0)
    {
        LOG_ERR("LR1121 NSS GPIO configuration failed: %d", err);
    }

    return err;
}

/*********************************************************************
**函数名称:  lr1121_wait_busy
**入口参数:  level      -- 期望的 BUSY 电平
**           timeout_ms -- 最大等待时间，单位为毫秒
**出口参数:  无
**函数功能:  等待 LR1121 BUSY 引脚达到指定电平
**返 回 值:  0 表示成功，负值表示 GPIO 读取失败或等待超时
*********************************************************************/
static int lr1121_wait_busy(int level, uint32_t timeout_ms)
{
    int64_t deadline = k_uptime_get() + timeout_ms;
    int busy;

    /* BUSY 为高电平有效，直接比较引脚实际电平。 */
    while (true)
    {
        busy = gpio_pin_get_raw(lr1121_busy.port, lr1121_busy.pin);
        if (busy < 0)
        {
            return busy;
        }

        if (busy == level)
        {
            return 0;
        }

        if (k_uptime_get() >= deadline)
        {
            return -ETIMEDOUT;
        }

        k_busy_wait(50);
    }
}

/*********************************************************************
**函数名称:  lr1121_transfer
**入口参数:  tx -- SPI 发送缓冲区
**           rx -- SPI 接收缓冲区
**出口参数:  无
**函数功能:  执行一次 LR1121 SPI 传输
**返 回 值:  0 表示成功，负值表示 SPI 传输失败
*********************************************************************/
static int lr1121_transfer(const struct spi_buf_set* tx, const struct spi_buf_set* rx)
{
    return spi_transceive_dt(&lr1121_spi, tx, rx);
}

/*********************************************************************
**函数名称:  lr1121_wakeup
**入口参数:  无
**出口参数:  无
**函数功能:  通过单次 NSS 唤醒脉冲唤醒 LR1121，并等待 BUSY 拉低
**返 回 值:  0 表示成功，负值表示唤醒失败或 BUSY 等待超时
*********************************************************************/
static int lr1121_wakeup(void)
{
    int err;

    /* Modem-E SPI 命令必须从 BUSY 高的休眠态开始唤醒。 */
    err = lr1121_wait_busy(1, LR1121_BUSY_HIGH_TIMEOUT_MS);
    if (err != 0)
    {
        LOG_ERR("LR1121 wakeup BUSY-high wait failed: %d", err);
        return err;
    }

    (void)gpio_pin_set_dt(&lr1121_nss, 1);

    k_busy_wait(LR1121_WAKEUP_PULSE_US);

    (void)gpio_pin_set_dt(&lr1121_nss, 0);

    err = lr1121_wait_busy(0, LR1121_BUSY_TIMEOUT_MS);

    if (err != 0)
    {
        LOG_ERR("LR1121 wakeup BUSY-low wait failed: %d, BUSY=%d", err,
            gpio_pin_get_raw(lr1121_busy.port, lr1121_busy.pin));
    }

    return err;
}

/*********************************************************************
**函数名称:  lr1121_write_frame
**入口参数:  command        -- SPI 命令数据
**           command_length -- 命令长度
**           data           -- SPI 参数数据
**           data_length    -- 参数长度
**出口参数:  无
**函数功能:  组帧并发送带 CRC 的 LR1121 SPI 写命令
**返 回 值:  0 表示成功，负值表示 SPI 传输失败
*********************************************************************/
static int lr1121_write_frame(const uint8_t* command, uint16_t command_length, const uint8_t* data,
    uint16_t data_length)
{
    uint8_t crc = modem_e_modem_compute_crc(0xFF, command, command_length);
    struct spi_buf buffers[3] = {
        {.buf = (void*)command, .len = command_length},
        {.buf = (void*)data, .len = data_length},
        {.buf = &crc, .len = 1},
    };
    struct spi_buf_set tx = {
        .buffers = buffers,
        .count = ARRAY_SIZE(buffers),
    };

#if LR1121_HAL_DEBUG_ENABLE
    if ((command_length >= 3U) && (command[0] == 0x06U) && (command[1] == 0x02U) &&
        (command[2] == 0x12U))
    {
        LOG_INF("LR1121 Request TX: command_len=%u, data_len=%u", command_length, data_length);
        LOG_HEXDUMP_INF(command, command_length, "LR1121 Request TX command");
        if ((data != NULL) && (data_length > 0U))
        {
            LOG_HEXDUMP_INF(data, data_length, "LR1121 Request TX data");
        }
    }
#endif

    crc = modem_e_modem_compute_crc(crc, data, data_length);
    return lr1121_transfer(&tx, NULL);
}

/*********************************************************************
**函数名称:  modem_e_modem_hal_write
**入口参数:  context        -- HAL 上下文
**           command        -- SPI 命令数据
**           command_length -- 命令长度
**           data           -- SPI 参数数据
**           data_length    -- 参数长度
**出口参数:  无
**函数功能:  唤醒 LR1121，发送命令并等待完整响应
**返 回 值:  Modem-E HAL 状态码
*********************************************************************/
modem_e_modem_hal_status_t modem_e_modem_hal_write(const void* context, const uint8_t* command,
    uint16_t command_length, const uint8_t* data,
    uint16_t data_length)
{
    uint8_t status = 0;
    uint8_t status_crc = 0;
    uint8_t expected_status_crc;

    ARG_UNUSED(context);

    /* Step 1: 唤醒 + 发送命令 + 等待 BUSY */
    if ((lr1121_wakeup() != 0) ||
        (lr1121_write_frame(command, command_length, data, data_length) != 0) ||
        (lr1121_wait_busy(1, LR1121_BUSY_TIMEOUT_MS) != 0))
    {
        return MODEM_E_MODEM_HAL_STATUS_BUSY_TIMEOUT;
    }

    /* Step 2: 读取 status + CRC（2字节） */
    {
        static const uint8_t nop[2] = {0};
        uint8_t rx[2] = {0};
        struct spi_buf tx_buf = {.buf = (void*)nop, .len = 2};
        struct spi_buf rx_buf = {.buf = rx, .len = 2};
        struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};
        struct spi_buf_set rx_set = {.buffers = &rx_buf, .count = 1};

        if (lr1121_transfer(&tx, &rx_set) != 0)
        {
            LOG_ERR("LR1121 SPI status read failed");
            return MODEM_E_MODEM_HAL_STATUS_ERROR;
        }

        status = rx[0];
        status_crc = rx[1];
    }

    /* Step 3: 验证 status 的 CRC */
    expected_status_crc = modem_e_modem_compute_crc(0xFF, &status, 1);
    if (expected_status_crc != status_crc)
    {
        LOG_ERR("LR1121 write status CRC mismatch: expected=0x%02x, got=0x%02x, status=0x%02x",
            expected_status_crc, status_crc, status);
        return MODEM_E_MODEM_HAL_STATUS_BAD_FRAME;
    }

    /* Step 4: 等待 BUSY 拉低 */
    if (lr1121_wait_busy(0, LR1121_BUSY_TIMEOUT_MS) != 0)
    {
        LOG_ERR("LR1121 response kept BUSY high");
        return MODEM_E_MODEM_HAL_STATUS_BUSY_TIMEOUT;
    }

    return (modem_e_modem_hal_status_t)status;
}

/*********************************************************************
**函数名称:  modem_e_modem_hal_read
**入口参数:  context        -- HAL 上下文
**           command        -- SPI 命令数据
**           command_length -- 命令长度
**           data           -- 接收数据的缓冲区
**           data_length    -- 接收数据长度
**出口参数:  data -- 写入 Modem-E 响应数据
**函数功能:  发送读取命令并读取 LR1121 响应
**返 回 值:  Modem-E HAL 状态码
*********************************************************************/
modem_e_modem_hal_status_t modem_e_modem_hal_read(const void* context, const uint8_t* command,
    uint16_t command_length, uint8_t* data,
    uint16_t data_length)
{
    int err;
    uint8_t status = 0;
    uint8_t response_crc = 0;
    uint8_t expected_crc;
    static const uint8_t s_nop_buffer[LR1121_NOP_BUFFER_SIZE];
    struct spi_buf tx_bufs[3];
    struct spi_buf rx_bufs[3];
    struct spi_buf_set tx;
    struct spi_buf_set rx;

    ARG_UNUSED(context);

    /* Step 1: 唤醒 LR1121 */
    err = lr1121_wakeup();
    if (err != 0)
    {
        LOG_ERR("LR1121 wakeup failed: %d", err);
        return MODEM_E_MODEM_HAL_STATUS_BUSY_TIMEOUT;
    }

    /* Step 2: 发送命令 */
    err = lr1121_write_frame(command, command_length, NULL, 0);
    if (err != 0)
    {
        LOG_ERR("LR1121 SPI command transfer failed: %d", err);
        return MODEM_E_MODEM_HAL_STATUS_BUSY_TIMEOUT;
    }

    /* Step 3: 等待 BUSY 拉高 */
    err = lr1121_wait_busy(1, LR1121_BUSY_TIMEOUT_MS);
    if (err != 0)
    {
        LOG_ERR("LR1121 did not assert BUSY after SPI command, BUSY=%d",
            gpio_pin_get_dt(&lr1121_busy));
        return MODEM_E_MODEM_HAL_STATUS_BUSY_TIMEOUT;
    }

    /*
     * 单次 SPI 事务必须保持完整响应帧连续：OK 为 status + data + CRC，
     * 非 OK 为 status + CRC。先按最长帧读取，再依据 status 选择 CRC 位置。
     */
    if (data_length > (LR1121_NOP_BUFFER_SIZE - 2U))
    {
        LOG_ERR("LR1121 response length too large: %u", data_length);
        return MODEM_E_MODEM_HAL_STATUS_ERROR;
    }

    tx_bufs[0] = (struct spi_buf){.buf = (void*)s_nop_buffer, .len = 1};
    tx_bufs[1] = (struct spi_buf){.buf = (void*)s_nop_buffer, .len = data_length};
    tx_bufs[2] = (struct spi_buf){.buf = (void*)s_nop_buffer, .len = 1};
    rx_bufs[0] = (struct spi_buf){.buf = &status, .len = 1};
    rx_bufs[1] = (struct spi_buf){.buf = data, .len = data_length};
    rx_bufs[2] = (struct spi_buf){.buf = &response_crc, .len = 1};
    tx = (struct spi_buf_set){.buffers = tx_bufs, .count = ARRAY_SIZE(tx_bufs)};
    rx = (struct spi_buf_set){.buffers = rx_bufs, .count = ARRAY_SIZE(rx_bufs)};

    if (lr1121_transfer(&tx, &rx) != 0)
    {
        LOG_ERR("LR1121 SPI response read failed");
        return MODEM_E_MODEM_HAL_STATUS_ERROR;
    }

    expected_crc = modem_e_modem_compute_crc(0xFF, &status, 1);
    if (status == MODEM_E_MODEM_HAL_STATUS_OK)
    {
        expected_crc = modem_e_modem_compute_crc(expected_crc, data, data_length);
    }
    else if (data_length > 0U)
    {
        /* 非 OK 响应的 CRC 紧跟 status，位于 data 缓冲区首字节。 */
        response_crc = data[0];
    }

    if (expected_crc != response_crc)
    {
        LOG_ERR("LR1121 CRC mismatch: expected=0x%02x, got=0x%02x, status=0x%02x",
            expected_crc, response_crc, status);
        return MODEM_E_MODEM_HAL_STATUS_BAD_FRAME;
    }

    /* Step 5: 等待 BUSY 拉低 */
    if (lr1121_wait_busy(0, LR1121_BUSY_TIMEOUT_MS) != 0)
    {
        LOG_ERR("LR1121 response kept BUSY high");
        return MODEM_E_MODEM_HAL_STATUS_BUSY_TIMEOUT;
    }

    return (modem_e_modem_hal_status_t)status;
}

/*********************************************************************
**函数名称:  modem_e_modem_hal_write_read
**入口参数:  context        -- HAL 上下文
**           command        -- SPI 发送数据
**           data           -- SPI 接收数据缓冲区
**           data_length    -- 传输长度
**出口参数:  data -- 写入接收数据
**函数功能:  执行一次唤醒后的全双工 SPI 传输
**返 回 值:  Modem-E HAL 状态码
*********************************************************************/
modem_e_modem_hal_status_t modem_e_modem_hal_write_read(const void* context, const uint8_t* command,
    uint8_t* data, uint16_t data_length)
{
    struct spi_buf tx_buf = {.buf = (void*)command, .len = data_length};
    struct spi_buf rx_buf = {.buf = data, .len = data_length};
    struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};
    struct spi_buf_set rx = {.buffers = &rx_buf, .count = 1};

    ARG_UNUSED(context);

    if ((lr1121_wakeup() != 0) || (lr1121_transfer(&tx, &rx) != 0) ||
        (lr1121_wait_busy(1, LR1121_BUSY_TIMEOUT_MS) != 0))
    {
        return MODEM_E_MODEM_HAL_STATUS_BUSY_TIMEOUT;
    }

    return MODEM_E_MODEM_HAL_STATUS_OK;
}

/*********************************************************************
**函数名称:  modem_e_modem_hal_write_without_rc
**入口参数:  context        -- HAL 上下文
**           command        -- SPI 命令数据
**           command_length -- 命令长度
**           data           -- SPI 参数数据
**           data_length    -- 参数长度
**出口参数:  无
**函数功能:  发送命令帧但不等待 Modem-E 响应
**返 回 值:  Modem-E HAL 状态码
*********************************************************************/
modem_e_modem_hal_status_t modem_e_modem_hal_write_without_rc(const void* context,
    const uint8_t* command,
    uint16_t command_length,
    const uint8_t* data,
    uint16_t data_length)
{
    ARG_UNUSED(context);

    return ((lr1121_wakeup() == 0) &&
               (lr1121_write_frame(command, command_length, data, data_length) == 0))
               ? MODEM_E_MODEM_HAL_STATUS_OK
               : MODEM_E_MODEM_HAL_STATUS_ERROR;
}

/*********************************************************************
**函数名称:  modem_e_modem_hal_reset
**入口参数:  context -- HAL 上下文
**出口参数:  无
**函数功能:  硬件复位 LR1121，等待启动完成并确认复位事件
**返 回 值:  Modem-E HAL 状态码
*********************************************************************/
modem_e_modem_hal_status_t modem_e_modem_hal_reset(const void* context)
{
    modem_e_event_fields_t event;
    modem_e_response_code_t response;
    int err;
    int64_t deadline;

    ARG_UNUSED(context);

    err = lr1121_ready();
    if (err != 0)
    {
        LOG_ERR("LR1121 reset initialization failed: %d", err);
        return MODEM_E_MODEM_HAL_STATUS_ERROR;
    }

    /* reset-gpios 为低电平有效，逻辑 1 表示拉低并保持复位。 */
    LOG_INF("LR1121 hardware reset started");
    (void)gpio_pin_set_dt(&lr1121_reset, 1);

    k_busy_wait(LR1121_RESET_PULSE_US);

    (void)gpio_pin_set_dt(&lr1121_reset, 0);

    /* 芯片复位释放后需要等待内部固件和校准完成，再进行首次 NSS 唤醒。 */
    k_sleep(K_MSEC(LR1121_RESET_SETTLE_MS));

    /*
     * 复位释放并不等于内部固件已经完成启动；先等待板级实测的启动窗口，
     * 再通过标准 GET_EVENT 命令确认 LR1121 已经产生复位事件。
     */
    deadline = k_uptime_get() + LR1121_RESET_EVENT_TIMEOUT_MS;
    do
    {
        /* modem_e_get_event() 内部会执行一次单次 NSS 唤醒和 SPI 事务。 */
        response = modem_e_get_event(context, &event);
        if ((response == MODEM_E_RESPONSE_CODE_OK) &&
            (event.event_type == MODEM_E_LORAWAN_EVENT_RESET))
        {
            /* 只有收到明确的复位事件，才允许上层继续配置 Modem-E。 */
            LOG_INF("LR1121 reset event received");
            return MODEM_E_MODEM_HAL_STATUS_OK;
        }

        /* 启动期间暂未收到复位事件时，按固定间隔重新轮询，避免忙等。 */
        k_sleep(K_MSEC(LR1121_RESET_EVENT_POLL_MS));
    } while (k_uptime_get() < deadline);

    LOG_ERR("LR1121 reset event timeout: response=0x%02x BUSY=%d", response,
        gpio_pin_get_raw(lr1121_busy.port, lr1121_busy.pin));
    return MODEM_E_MODEM_HAL_STATUS_BUSY_TIMEOUT;
}

/*********************************************************************
**函数名称:  modem_e_modem_hal_enter_dfu
**入口参数:  context -- HAL 上下文
**出口参数:  无
**函数功能:  进入 Modem-E DFU 模式（当前平台未实现）
*********************************************************************/
void modem_e_modem_hal_enter_dfu(const void* context)
{
    ARG_UNUSED(context);
}

/*********************************************************************
**函数名称:  modem_e_modem_hal_wakeup
**入口参数:  context -- HAL 上下文
**出口参数:  无
**函数功能:  对外提供 Modem-E 唤醒接口
**返 回 值:  Modem-E HAL 状态码
*********************************************************************/
modem_e_modem_hal_status_t modem_e_modem_hal_wakeup(const void* context)
{
    ARG_UNUSED(context);

    return (lr1121_wakeup() == 0) ? MODEM_E_MODEM_HAL_STATUS_OK : MODEM_E_MODEM_HAL_STATUS_BUSY_TIMEOUT;
}

/*********************************************************************
**函数名称:  modem_e_hal_write
**入口参数:  context        -- HAL 上下文
**           command        -- SPI 命令数据
**           command_length -- 命令长度
**           data           -- SPI 参数数据
**           data_length    -- 参数长度
**出口参数:  无
**函数功能:  提供通用 HAL 写接口，不等待命令响应
**返 回 值:  HAL 状态码
*********************************************************************/
modem_e_hal_status_t modem_e_hal_write(const void* context, const uint8_t* command,
    uint16_t command_length, const uint8_t* data,
    uint16_t data_length)
{
    return (modem_e_modem_hal_write_without_rc(context, command, command_length, data,
                data_length) == MODEM_E_MODEM_HAL_STATUS_OK)
               ? MODEM_E_HAL_STATUS_OK
               : MODEM_E_HAL_STATUS_ERROR;
}

/*********************************************************************
**函数名称:  modem_e_hal_read
**入口参数:  context        -- HAL 上下文
**           command        -- SPI 命令数据
**           command_length -- 命令长度
**           data           -- 接收数据缓冲区
**           data_length    -- 接收数据长度
**出口参数:  data -- 写入响应数据
**函数功能:  提供通用 HAL 读接口
**返 回 值:  HAL 状态码
*********************************************************************/
modem_e_hal_status_t modem_e_hal_read(const void* context, const uint8_t* command,
    uint16_t command_length, uint8_t* data, uint16_t data_length)
{
    return (modem_e_modem_hal_read(context, command, command_length, data, data_length) ==
               MODEM_E_MODEM_HAL_STATUS_OK)
               ? MODEM_E_HAL_STATUS_OK
               : MODEM_E_HAL_STATUS_ERROR;
}

/*********************************************************************
**函数名称:  modem_e_hal_write_read
**入口参数:  context     -- HAL 上下文
**           command     -- SPI 发送数据
**           data        -- SPI 接收数据缓冲区
**           data_length -- 传输长度
**出口参数:  data -- 写入接收数据
**函数功能:  提供通用 HAL 全双工 SPI 接口
**返 回 值:  HAL 状态码
*********************************************************************/
modem_e_hal_status_t modem_e_hal_write_read(const void* context, const uint8_t* command,
    uint8_t* data, uint16_t data_length)
{
    return (modem_e_modem_hal_write_read(context, command, data, data_length) ==
               MODEM_E_MODEM_HAL_STATUS_OK)
               ? MODEM_E_HAL_STATUS_OK
               : MODEM_E_HAL_STATUS_ERROR;
}

/*********************************************************************
**函数名称:  modem_e_hal_direct_read
**入口参数:  context     -- HAL 上下文
**           data        -- 接收数据缓冲区
**           data_length -- 接收数据长度
**出口参数:  data -- 写入 SPI 接收数据
**函数功能:  直接执行一次 SPI 读取，不附加 Modem-E 唤醒和协议处理
**返 回 值:  HAL 状态码
*********************************************************************/
modem_e_hal_status_t modem_e_hal_direct_read(const void* context, uint8_t* data, uint16_t data_length)
{
    struct spi_buf tx_buf = {.buf = NULL, .len = data_length};
    struct spi_buf rx_buf = {.buf = data, .len = data_length};
    struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};
    struct spi_buf_set rx = {.buffers = &rx_buf, .count = 1};

    ARG_UNUSED(context);

    return (lr1121_transfer(&tx, &rx) == 0) ? MODEM_E_HAL_STATUS_OK : MODEM_E_HAL_STATUS_ERROR;
}

/*********************************************************************
**函数名称:  modem_e_hal_reset
**入口参数:  context -- HAL 上下文
**出口参数:  无
**函数功能:  提供通用 HAL 硬件复位接口
**返 回 值:  HAL 状态码
*********************************************************************/
modem_e_hal_status_t modem_e_hal_reset(const void* context)
{
    return (modem_e_modem_hal_reset(context) == MODEM_E_MODEM_HAL_STATUS_OK) ? MODEM_E_HAL_STATUS_OK : MODEM_E_HAL_STATUS_ERROR;
}

/*********************************************************************
**函数名称:  modem_e_hal_wakeup
**入口参数:  context -- HAL 上下文
**出口参数:  无
**函数功能:  提供通用 HAL 唤醒接口
**返 回 值:  HAL 状态码
*********************************************************************/
modem_e_hal_status_t modem_e_hal_wakeup(const void* context)
{
    return (modem_e_modem_hal_wakeup(context) == MODEM_E_MODEM_HAL_STATUS_OK) ? MODEM_E_HAL_STATUS_OK : MODEM_E_HAL_STATUS_ERROR;
}
