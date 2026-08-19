#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "modem_e_hal.h"
#include "modem_e_modem_hal.h"

LOG_MODULE_REGISTER(lr1121_hal, LOG_LEVEL_INF);

#define LR1121_BUSY_TIMEOUT_MS         1000U
#define LR1121_BOOT_BUSY_TIMEOUT_MS    5000U
#define LR1121_RESET_PULSE_US          1000U
#define LR1121_RESET_SETTLE_MS         100U
#define LR1121_NOP_BUFFER_SIZE          255U

/* BUSY and RESET belong to LR1121; CS is owned by the SPI controller. */
#define LR1121_NODE                    DT_NODELABEL(lr1121)

static const struct spi_dt_spec lr1121_spi =
    SPI_DT_SPEC_GET(LR1121_NODE, SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0);
static const struct gpio_dt_spec lr1121_busy = GPIO_DT_SPEC_GET(LR1121_NODE, busy_gpios);
static const struct gpio_dt_spec lr1121_reset = GPIO_DT_SPEC_GET(LR1121_NODE, reset_gpios);
static const struct gpio_dt_spec lr1121_nss =
    GPIO_DT_SPEC_GET_BY_IDX(DT_PARENT(LR1121_NODE), cs_gpios, 0);

int lr1121_ready(void)
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

static int lr1121_wait_busy(int level, uint32_t timeout_ms)
{
    int64_t deadline = k_uptime_get() + timeout_ms;

    /* BUSY is active-high; compare the physical pin level directly. */
    while (gpio_pin_get_raw(lr1121_busy.port, lr1121_busy.pin) != level)
    {
        if (k_uptime_get() >= deadline)
        {
            return -ETIMEDOUT;
        }

        k_busy_wait(50);
    }

    return 0;
}

static int lr1121_transfer(const struct spi_buf_set *tx, const struct spi_buf_set *rx)
{
    return spi_transceive_dt(&lr1121_spi, tx, rx);
}

static int lr1121_wakeup(void)
{
    int err;
    int busy;

    busy = gpio_pin_get_raw(lr1121_busy.port, lr1121_busy.pin);
    if (busy < 0)
    {
        LOG_ERR("LR1121 BUSY GPIO read failed: %d", busy);
        return busy;
    }

    /* BUSY 为低表示 Modem-E 已可接收命令。 */
    if (busy == 0)
    {
        return 0;
    }

    err = lr1121_wait_busy(1, LR1121_BOOT_BUSY_TIMEOUT_MS);
    if (err != 0)
    {
        LOG_ERR("LR1121 wakeup BUSY-high wait failed: %d", err);
        return err;
    }

    err = gpio_pin_set_dt(&lr1121_nss, 1);
    if (err != 0)
    {
        LOG_ERR("LR1121 wakeup NSS assert failed: %d", err);
        return err;
    }

    k_busy_wait(100);
    err = gpio_pin_set_dt(&lr1121_nss, 0);
    if (err != 0)
    {
        LOG_ERR("LR1121 wakeup NSS deassert failed: %d", err);
        return err;
    }

    err = lr1121_wait_busy(0, LR1121_BUSY_TIMEOUT_MS);
    if (err != 0)
    {
        LOG_ERR("LR1121 wakeup BUSY-low wait failed: %d", err);
    }

    return err;
}

static int lr1121_write_frame(const uint8_t *command, uint16_t command_length, const uint8_t *data,
                              uint16_t data_length)
{
    uint8_t crc = modem_e_modem_compute_crc(0xFF, command, command_length);
    struct spi_buf buffers[3] = {
        {.buf = (void *)command, .len = command_length},
        {.buf = (void *)data, .len = data_length},
        {.buf = &crc, .len = 1},
    };
    struct spi_buf_set tx = {
        .buffers = buffers,
        .count = ARRAY_SIZE(buffers),
    };

    crc = modem_e_modem_compute_crc(crc, data, data_length);
    return lr1121_transfer(&tx, NULL);
}

static modem_e_modem_hal_status_t lr1121_read_response(uint8_t *data, uint16_t data_length)
{
    static const uint8_t s_nop_buffer[LR1121_NOP_BUFFER_SIZE];
    uint8_t status = 0;
    uint8_t crc = 0;
    uint8_t expected;
    struct spi_buf tx_bufs[3] = {
        {.buf = (void *)s_nop_buffer, .len = 1},
        {.buf = (void *)s_nop_buffer, .len = data_length},
        {.buf = (void *)s_nop_buffer, .len = 1},
    };
    struct spi_buf rx_bufs[3] = {
        {.buf = &status, .len = 1},
        {.buf = data, .len = data_length},
        {.buf = &crc, .len = 1},
    };
    struct spi_buf_set tx = {
        .buffers = tx_bufs,
        .count = ARRAY_SIZE(tx_bufs),
    };
    struct spi_buf_set rx = {
        .buffers = rx_bufs,
        .count = ARRAY_SIZE(rx_bufs),
    };

    if (data_length > LR1121_NOP_BUFFER_SIZE)
    {
        return MODEM_E_MODEM_HAL_STATUS_ERROR;
    }
    if (lr1121_transfer(&tx, &rx) != 0)
    {
        return MODEM_E_MODEM_HAL_STATUS_ERROR;
    }
    if (lr1121_wait_busy(0, LR1121_BUSY_TIMEOUT_MS) != 0)
    {
        LOG_ERR("LR1121 response kept BUSY high: status=0x%02x crc=0x%02x", status, crc);
        if ((data != NULL) && (data_length != 0U))
        {
            LOG_HEXDUMP_ERR(data, data_length, "LR1121 raw response");
        }
        return MODEM_E_MODEM_HAL_STATUS_BUSY_TIMEOUT;
    }

    expected = modem_e_modem_compute_crc(0xFF, &status, 1);
    if (status == MODEM_E_MODEM_HAL_STATUS_OK)
    {
        expected = modem_e_modem_compute_crc(expected, data, data_length);
    }

    return (expected == crc) ? (modem_e_modem_hal_status_t)status :
                               MODEM_E_MODEM_HAL_STATUS_BAD_FRAME;
}

modem_e_modem_hal_status_t modem_e_modem_hal_write(const void *context, const uint8_t *command,
                                                    uint16_t command_length, const uint8_t *data,
                                                    uint16_t data_length)
{
    ARG_UNUSED(context);

    if ((lr1121_wakeup() != 0) ||
        (lr1121_write_frame(command, command_length, data, data_length) != 0) ||
        (lr1121_wait_busy(1, LR1121_BUSY_TIMEOUT_MS) != 0))
    {
        return MODEM_E_MODEM_HAL_STATUS_BUSY_TIMEOUT;
    }

    return lr1121_read_response(NULL, 0);
}

modem_e_modem_hal_status_t modem_e_modem_hal_read(const void *context, const uint8_t *command,
                                                   uint16_t command_length, uint8_t *data,
                                                   uint16_t data_length)
{
    int err;

    ARG_UNUSED(context);

    err = lr1121_wakeup();
    if (err != 0)
    {
        LOG_ERR("LR1121 wakeup failed: %d, BUSY=%d", err, gpio_pin_get_dt(&lr1121_busy));
        return MODEM_E_MODEM_HAL_STATUS_BUSY_TIMEOUT;
    }

    err = lr1121_write_frame(command, command_length, NULL, 0);
    if (err != 0)
    {
        LOG_ERR("LR1121 SPI command transfer failed: %d", err);
        return MODEM_E_MODEM_HAL_STATUS_BUSY_TIMEOUT;
    }

    err = lr1121_wait_busy(1, LR1121_BUSY_TIMEOUT_MS);
    if (err != 0)
    {
        LOG_ERR("LR1121 did not assert BUSY after SPI command, BUSY=%d",
                gpio_pin_get_dt(&lr1121_busy));
        return MODEM_E_MODEM_HAL_STATUS_BUSY_TIMEOUT;
    }

    return lr1121_read_response(data, data_length);
}

modem_e_modem_hal_status_t modem_e_modem_hal_write_read(const void *context, const uint8_t *command,
                                                         uint8_t *data, uint16_t data_length)
{
    struct spi_buf tx_buf = {.buf = (void *)command, .len = data_length};
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

modem_e_modem_hal_status_t modem_e_modem_hal_write_without_rc(const void *context,
                                                               const uint8_t *command,
                                                               uint16_t command_length,
                                                               const uint8_t *data,
                                                               uint16_t data_length)
{
    ARG_UNUSED(context);

    return ((lr1121_wakeup() == 0) &&
            (lr1121_write_frame(command, command_length, data, data_length) == 0)) ?
            MODEM_E_MODEM_HAL_STATUS_OK : MODEM_E_MODEM_HAL_STATUS_ERROR;
}

modem_e_modem_hal_status_t modem_e_modem_hal_reset(const void *context)
{
    int err;

    ARG_UNUSED(context);

    err = lr1121_ready();
    if (err != 0)
    {
        LOG_ERR("LR1121 reset initialization failed: %d", err);
        return MODEM_E_MODEM_HAL_STATUS_ERROR;
    }

    /* reset-gpios is GPIO_ACTIVE_LOW: logical 1 asserts reset. */
    err = gpio_pin_set_dt(&lr1121_reset, 1);
    if (err != 0)
    {
        LOG_ERR("LR1121 reset assert failed: %d", err);
        return MODEM_E_MODEM_HAL_STATUS_ERROR;
    }

    k_busy_wait(LR1121_RESET_PULSE_US);

    err = gpio_pin_set_dt(&lr1121_reset, 0);
    if (err != 0)
    {
        LOG_ERR("LR1121 reset release failed: %d", err);
        return MODEM_E_MODEM_HAL_STATUS_ERROR;
    }

    k_sleep(K_MSEC(LR1121_RESET_SETTLE_MS));

    return MODEM_E_MODEM_HAL_STATUS_OK;
}

void modem_e_modem_hal_enter_dfu(const void *context)
{
    ARG_UNUSED(context);
}

modem_e_modem_hal_status_t modem_e_modem_hal_wakeup(const void *context)
{
    ARG_UNUSED(context);

    return (lr1121_wakeup() == 0) ? MODEM_E_MODEM_HAL_STATUS_OK :
                                    MODEM_E_MODEM_HAL_STATUS_BUSY_TIMEOUT;
}

modem_e_hal_status_t modem_e_hal_write(const void *context, const uint8_t *command,
                                       uint16_t command_length, const uint8_t *data,
                                       uint16_t data_length)
{
    return (modem_e_modem_hal_write_without_rc(context, command, command_length, data,
                                               data_length) == MODEM_E_MODEM_HAL_STATUS_OK) ?
               MODEM_E_HAL_STATUS_OK : MODEM_E_HAL_STATUS_ERROR;
}

modem_e_hal_status_t modem_e_hal_read(const void *context, const uint8_t *command,
                                      uint16_t command_length, uint8_t *data, uint16_t data_length)
{
    return (modem_e_modem_hal_read(context, command, command_length, data, data_length) ==
            MODEM_E_MODEM_HAL_STATUS_OK) ? MODEM_E_HAL_STATUS_OK : MODEM_E_HAL_STATUS_ERROR;
}

modem_e_hal_status_t modem_e_hal_write_read(const void *context, const uint8_t *command,
                                            uint8_t *data, uint16_t data_length)
{
    return (modem_e_modem_hal_write_read(context, command, data, data_length) ==
            MODEM_E_MODEM_HAL_STATUS_OK) ? MODEM_E_HAL_STATUS_OK : MODEM_E_HAL_STATUS_ERROR;
}

modem_e_hal_status_t modem_e_hal_direct_read(const void *context, uint8_t *data, uint16_t data_length)
{
    struct spi_buf tx_buf = {.buf = NULL, .len = data_length};
    struct spi_buf rx_buf = {.buf = data, .len = data_length};
    struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};
    struct spi_buf_set rx = {.buffers = &rx_buf, .count = 1};

    ARG_UNUSED(context);

    return (lr1121_transfer(&tx, &rx) == 0) ? MODEM_E_HAL_STATUS_OK : MODEM_E_HAL_STATUS_ERROR;
}

modem_e_hal_status_t modem_e_hal_reset(const void *context)
{
    return (modem_e_modem_hal_reset(context) == MODEM_E_MODEM_HAL_STATUS_OK) ?
               MODEM_E_HAL_STATUS_OK : MODEM_E_HAL_STATUS_ERROR;
}

modem_e_hal_status_t modem_e_hal_wakeup(const void *context)
{
    return (modem_e_modem_hal_wakeup(context) == MODEM_E_MODEM_HAL_STATUS_OK) ?
               MODEM_E_HAL_STATUS_OK : MODEM_E_HAL_STATUS_ERROR;
}
