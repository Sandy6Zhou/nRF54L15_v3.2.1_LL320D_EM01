/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        fm175xx_driver.c
**文件描述:        FM175XX NFC 芯片驱动实现文件
**当前版本:        V1.0
**作    者:        Harrison Wu (wuyujiao@jimiiot.com)
**完成日期:        2026.02.28
*********************************************************************
** 功能描述:        实现 FM17550/FM17622 芯片 I2C 通信和卡片操作
*********************************************************************/

#include "../inc/fm175xx_driver.h"
#include "../inc/fm175xx_reg.h"
#include "../inc/nfc_reader_api.h"

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(fm175xx_driver, LOG_LEVEL_INF);

/* I2C 设备配置 */
#define NFC_I2C_NODE DT_ALIAS(nfc_i2c)
static const struct device *nfc_i2c_dev = DEVICE_DT_GET(NFC_I2C_NODE);

/* NPD 复位引脚配置 */
#define NFC_NPD_NODE DT_ALIAS(nfc_npd_ctrl)
static const struct gpio_dt_spec nfc_npd_gpio = GPIO_DT_SPEC_GET(NFC_NPD_NODE, gpios);

/* 轮询状态 */
static bool g_poll_running = false;
static struct k_work_delayable g_poll_work;
static nfc_card_detected_cb_t g_card_cb = NULL;

/* 前向声明轮询处理函数 */
static void fm175xx_poll_handler(struct k_work *work);

/* 卡片信息 */
static struct
{
    uint8_t uid[16];
    uint8_t uid_len;
    uint8_t sak;
    bool present;
} g_card_info;

/********************************************************************
**函数名称:  fm175xx_npd_reset
**入口参数:  无
**出口参数:  无
**函数功能:  执行 NPD 硬件复位
**返 回 值:  无
*********************************************************************/
void fm175xx_npd_reset(void)
{
    /* NPD 低电平复位 */
    gpio_pin_set_dt(&nfc_npd_gpio, 0);
    k_sleep(K_MSEC(2));

    /* NPD 高电平释放 */
    gpio_pin_set_dt(&nfc_npd_gpio, 1);
    k_sleep(K_MSEC(2));
}

/********************************************************************
**函数名称:  fm175xx_read_reg
**入口参数:  reg      ---        寄存器地址
**           data     ---        数据缓冲区
**出口参数:  data     ---        读取到的数据
**函数功能:  读取 FM175XX 寄存器
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
int fm175xx_read_reg(uint8_t reg, uint8_t *data)
{
    int ret;
    uint8_t tx_buf = reg & 0x3F;

    ret = i2c_write(nfc_i2c_dev, &tx_buf, 1, FM175XX_I2C_ADDR);
    if (ret < 0)
    {
        return ret;
    }

    ret = i2c_read(nfc_i2c_dev, data, 1, FM175XX_I2C_ADDR);
    return ret;
}

/********************************************************************
**函数名称:  fm175xx_write_reg
**入口参数:  reg      ---        寄存器地址
**           data     ---        要写入的数据
**出口参数:  无
**函数功能:  写入 FM175XX 寄存器
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
int fm175xx_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t tx_buf[2] = {reg & 0x3F, data};
    return i2c_write(nfc_i2c_dev, tx_buf, 2, FM175XX_I2C_ADDR);
}

/********************************************************************
**函数名称:  fm175xx_driver_init
**入口参数:  无
**出口参数:  无
**函数功能:  初始化 FM175XX 驱动
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
int fm175xx_driver_init(void)
{
    int ret;
    uint8_t version;

    /* 检查 I2C 设备 */
    if (!device_is_ready(nfc_i2c_dev))
    {
        LOG_ERR("NFC I2C device not ready");
        return -ENODEV;
    }

    /* 检查 NPD GPIO */
    if (!device_is_ready(nfc_npd_gpio.port))
    {
        LOG_ERR("NFC NPD GPIO not ready");
        return -ENODEV;
    }

    /* 配置 NPD GPIO 为输出模式，默认高电平 */
    ret = gpio_pin_configure_dt(&nfc_npd_gpio, GPIO_OUTPUT_HIGH);
    if (ret < 0)
    {
        LOG_ERR("Failed to configure NPD GPIO: %d", ret);
        return ret;
    }

    /* 执行硬件复位 */
    fm175xx_npd_reset();

    /* 读取版本寄存器 */
    ret = fm175xx_read_reg(JREG_VERSION, &version);
    if (ret < 0)
    {
        LOG_ERR("Failed to read FM175XX version: %d", ret);
        return ret;
    }

    LOG_INF("FM175XX Version: 0x%02X", version);
    k_sleep(K_MSEC(1));

    /* 验证 FIFO 读写功能 */
    ret = fm175xx_check_fifo();
    if (ret < 0)
    {
        LOG_ERR("FM175XX FIFO check failed: %d", ret);
        return ret;
    }

    /* 初始化轮询工作项 */
    k_work_init_delayable(&g_poll_work, fm175xx_poll_handler);
    g_poll_running = false;
    g_card_cb = NULL;
    g_card_info.present = false;

    LOG_INF("FM175XX driver initialized");
    return 0;
}

/********************************************************************
**函数名称:  fm175xx_driver_deinit
**入口参数:  无
**出口参数:  无
**函数功能:  反初始化 FM175XX 驱动
**返 回 值:  0 表示成功
*********************************************************************/
int fm175xx_driver_deinit(void)
{
    fm175xx_poll_stop();
    return 0;
}

/********************************************************************
**函数名称:  fm175xx_get_version
**入口参数:  无
**出口参数:  version  ---        版本号存储指针
**函数功能:  读取 FM175XX 芯片版本号
**返 回 值:  0 表示成功
*********************************************************************/
int fm175xx_get_version(uint8_t *version)
{
    return fm175xx_read_reg(JREG_VERSION, version);
}

/********************************************************************
**函数名称:  fm175xx_poll_handler
**入口参数:  work     ---        工作项指针
**出口参数:  无
**函数功能:  卡片轮询处理函数
**返 回 值:  无
*********************************************************************/
static void fm175xx_poll_handler(struct k_work *work)
{
    uint8_t result;
    uint8_t uid_len;

    if (!g_poll_running)
    {
        return;
    }

    /* 使用原厂兼容的 Type A 应用流程 */
    result = Type_A_App();
    if (result == FM175XX_SUCCESS)
    {
        /* 根据 ATQA 计算 UID 长度 */
        if ((PICC_A.ATQA[0] & 0xC0) == 0x00)
        {
            uid_len = 4;
        }
        else if ((PICC_A.ATQA[0] & 0xC0) == 0x40)
        {
            uid_len = 7;
        }
        else
        {
            uid_len = 10;
        }

        LOG_DBG("Card detected, UID: %02X%02X%02X%02X",
                PICC_A.UID[0], PICC_A.UID[1], PICC_A.UID[2], PICC_A.UID[3]);

        /* 调用回调函数通知上层 */
        if (g_card_cb != NULL)
        {
            g_card_cb(NFC_CARD_TYPE_MIFARE, PICC_A.UID, uid_len, NULL, 0);
        }

        g_card_info.present = true;
        memcpy(g_card_info.uid, PICC_A.UID, 16);
        g_card_info.uid_len = uid_len;
    }
    else
    {
        LOG_DBG("No card detected (result=%d)", result);
    }

    /* 重新调度轮询 */
    if (g_poll_running)
    {
        k_work_reschedule(&g_poll_work, K_MSEC(500));
    }
}

/********************************************************************
**函数名称:  fm175xx_poll_start
**入口参数:  cb       ---        卡片检测回调函数
**出口参数:  无
**函数功能:  启动卡片轮询
**返 回 值:  0 表示成功
*********************************************************************/
int fm175xx_poll_start(nfc_card_detected_cb_t cb)
{
    if (g_poll_running)
    {
        return 0;
    }

    g_card_cb = cb;
    g_poll_running = true;

    k_work_schedule(&g_poll_work, K_NO_WAIT);

    return 0;
}

/********************************************************************
**函数名称:  fm175xx_poll_stop
**入口参数:  无
**出口参数:  无
**函数功能:  停止卡片轮询
**返 回 值:  0 表示成功
*********************************************************************/
int fm175xx_poll_stop(void)
{
    struct k_work_sync sync;

    if (!g_poll_running)
    {
        return 0;
    }

    g_poll_running = false;

    /* 取消并等待当前工作项完成，避免在 HPD 模式下进行 I2C 通信 */
    k_work_cancel_delayable_sync(&g_poll_work, &sync);

    return 0;
}

/********************************************************************
**函数名称:  fm175xx_modify_reg
**入口参数:  reg_addr --- 寄存器地址
**           mask     --- 要修改的位掩码
**           set      --- 要设置的值
**函数功能:  修改 FM175XX 寄存器的特定位
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
int fm175xx_modify_reg(uint8_t reg_addr, uint8_t mask, uint8_t set)
{
    int ret;
    uint8_t reg_val;

    ret = fm175xx_read_reg(reg_addr, &reg_val);
    if (ret < 0)
    {
        return ret;
    }

    reg_val = (reg_val & ~mask) | (set & mask);

    return fm175xx_write_reg(reg_addr, reg_val);
}

/********************************************************************
**函数名称:  fm175xx_read_fifo
**入口参数:  length   --- 要读取的字节数
**出口参数:  data_buf --- 数据缓冲区
**函数功能:  从 FIFO 读取数据
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
int fm175xx_read_fifo(uint8_t length, uint8_t *data_buf)
{
    int ret;
    uint8_t reg_addr;

    if (length > 64)
    {
        LOG_ERR("FIFO read length %d exceeds maximum 64", length);
        return -EINVAL;
    }

    /* I2C 连续读：先写寄存器地址，然后读数据 */
    reg_addr = JREG_FIFODATA & 0x3F;

    /* 使用 i2c_write_read 实现连续读 */
    ret = i2c_write_read(nfc_i2c_dev, FM175XX_I2C_ADDR, &reg_addr, 1, data_buf, length);
    if (ret < 0)
    {
        LOG_ERR("FIFO read failed: %d", ret);
        return ret;
    }

    return 0;
}

/********************************************************************
**函数名称:  fm175xx_write_fifo
**入口参数:  length   --- 要写入的字节数
**           data_buf --- 数据缓冲区
**函数功能:  向 FIFO 写入数据
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
int fm175xx_write_fifo(uint8_t length, uint8_t *data_buf)
{
    int ret;
    uint8_t tx_buf[65]; /* 最大 64 字节数据 + 1 字节寄存器地址 */

    if (length > 64)
    {
        LOG_ERR("FIFO write length %d exceeds maximum 64", length);
        return -EINVAL;
    }

    /* 构建 I2C 传输缓冲区：寄存器地址 + 数据 */
    tx_buf[0] = JREG_FIFODATA & 0x3F;
    memcpy(&tx_buf[1], data_buf, length);

    /* 单次 I2C 传输写入所有数据 */
    ret = i2c_write(nfc_i2c_dev, tx_buf, length + 1, FM175XX_I2C_ADDR);
    if (ret < 0)
    {
        LOG_ERR("Write FIFO failed: %d", ret);
        return ret;
    }

    return 0;
}

/********************************************************************
**函数名称:  fm175xx_clear_fifo
**入口参数:  无
**函数功能:  清空 FIFO
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
int fm175xx_clear_fifo(void)
{
    int ret;
    uint8_t reg_val;

    /* 设置 FlushBuffer 位 */
    ret = fm175xx_read_reg(JREG_FIFOLEVEL, &reg_val);
    if (ret < 0)
    {
        return ret;
    }

    reg_val |= 0x80; /* FlushBuffer bit */
    ret = fm175xx_write_reg(JREG_FIFOLEVEL, reg_val);
    if (ret < 0)
    {
        return ret;
    }

    /* 验证 FIFO 已清空 */
    ret = fm175xx_read_reg(JREG_FIFOLEVEL, &reg_val);
    if (ret < 0)
    {
        return ret;
    }

    if ((reg_val & 0x7F) != 0)
    {
        LOG_WRN("FIFO not empty after clear: %d bytes", reg_val & 0x7F);
        return -EIO;
    }

    return 0;
}

/********************************************************************
**函数名称:  fm175xx_check_fifo
**入口参数:  无
**函数功能:  验证 FIFO 读写功能是否正常
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
int fm175xx_check_fifo(void)
{
    int ret;
    uint8_t reg_val;
    uint8_t test_data[10] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA};
    uint8_t read_data[10] = {0};
    uint8_t i;

    /* 读取 CONTROL 寄存器 */
    ret = fm175xx_read_reg(JREG_CONTROL, &reg_val);
    if (ret < 0)
    {
        LOG_ERR("Failed to read CONTROL register: 0x%02X", reg_val);
        return ret;
    }

    LOG_DBG("CONTROL register: 0x%02X", reg_val);
    k_sleep(K_MSEC(1));

    /* 清空 FIFO */
    ret = fm175xx_clear_fifo();
    if (ret < 0)
    {
        LOG_ERR("Failed to clear FIFO: %d", ret);
        return ret;
    }

    /* 写入测试数据到 FIFO */
    ret = fm175xx_write_fifo(10, test_data);
    if (ret < 0)
    {
        LOG_ERR("Failed to write FIFO: %d", ret);
        return ret;
    }

    /* 从 FIFO 读取数据 */
    ret = fm175xx_read_fifo(10, read_data);
    if (ret < 0)
    {
        LOG_ERR("Failed to read FIFO: %d", ret);
        return ret;
    }

    /* 验证数据 */
    if (memcmp(test_data, read_data, 10) != 0)
    {
        LOG_ERR("FIFO data mismatch");
        return -EIO;
    }

    LOG_INF("FIFO check passed");

    /* 清空 FIFO */
    fm175xx_clear_fifo();

    k_sleep(K_MSEC(1));
    return 0;
}

/********************************************************************
**函数名称:  fm175xx_enter_hpd
**入口参数:  无
**出口参数:  无
**函数功能:  进入 Hard Power Down (HPD) 模式，NPD 拉低
**返 回 值:  0 表示成功
*********************************************************************/
int fm175xx_enter_hpd(void)
{
    /* 停止轮询 */
    fm175xx_poll_stop();

    /* NPD 拉低进入 HPD 模式 */
    gpio_pin_set_dt(&nfc_npd_gpio, 0);

    return 0;
}

/********************************************************************
**函数名称:  fm175xx_exit_hpd
**入口参数:  无
**出口参数:  无
**函数功能:  退出 HPD 模式，重新初始化芯片
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
int fm175xx_exit_hpd(void)
{
    int ret;
    uint8_t version;

    /* 执行硬件复位 */
    fm175xx_npd_reset();

    /* 读取版本寄存器验证芯片已唤醒 */
    ret = fm175xx_read_reg(JREG_VERSION, &version);
    if (ret < 0)
    {
        LOG_ERR("Failed to read FM175XX version after HPD exit: %d", ret);
        return ret;
    }

    return 0;
}
