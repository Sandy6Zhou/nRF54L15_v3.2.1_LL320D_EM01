/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        shtc3_driver.c
**文件描述:        SHTC3 温湿度传感器驱动实现文件
**当前版本:        V1.0
**作    者:       周森达 (zhousenda@jimiiot.com)
**完成日期:        2026.06.03
*********************************************************************
** 功能描述:       实现 SHTC3 温湿度传感器 I2C 通信和测量操作
*********************************************************************/

#include "../inc/shtc3_driver.h"

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(shtc3_driver, LOG_LEVEL_INF);

/* I2C 设备配置 */
#define TEMP_HUMI_I2C_NODE DT_ALIAS(temp_humi_i2c)
static const struct device *g_i2c_dev = DEVICE_DT_GET(TEMP_HUMI_I2C_NODE);

/* CRC-8 多项式和初始值 */
#define SHTC3_CRC_POLYNOMIAL    0x31
#define SHTC3_CRC_INIT          0xFF

/********************************************************************
**函数名称:  shtc3_crc8
**入口参数:  data     ---        待校验数据指针
**           len      ---        数据长度
**出口参数:  无
**函数功能:  计算 CRC-8 校验值（多项式 0x31，初始值 0xFF）
**返 回 值:  CRC-8 校验结果
*********************************************************************/
static uint8_t shtc3_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = SHTC3_CRC_INIT;
    uint8_t i;
    uint8_t j;

    for (i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (j = 0; j < 8; j++)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ SHTC3_CRC_POLYNOMIAL;
            }
            else
            {
                crc = (crc << 1);
            }
        }
    }
    return crc;
}

/********************************************************************
**函数名称:  shtc3_send_cmd
**入口参数:  cmd      ---        16-bit 命令字
**出口参数:  无
**函数功能:  通过 I2C 发送 16-bit 命令（MSB 优先）
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
static int shtc3_send_cmd(uint16_t cmd)
{
    uint8_t tx_buf[2];

    tx_buf[0] = (uint8_t)(cmd >> 8);
    tx_buf[1] = (uint8_t)(cmd & 0xFF);

    return i2c_write(g_i2c_dev, tx_buf, 2, SHTC3_I2C_ADDR);
}

/********************************************************************
**函数名称:  shtc3_driver_wakeup
**入口参数:  无
**出口参数:  无
**函数功能:  唤醒传感器
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
int shtc3_driver_wakeup(void)
{
    int ret;

    ret = shtc3_send_cmd(SHTC3_CMD_WAKEUP);
    if (ret < 0)
    {
        LOG_ERR("SHTC3 wakeup failed: %d", ret);
        return ret;
    }

    k_sleep(K_MSEC(1));
    return 0;
}

/********************************************************************
**函数名称:  shtc3_driver_sleep
**入口参数:  无
**出口参数:  无
**函数功能:  使传感器进入休眠模式
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
int shtc3_driver_sleep(void)
{
    return shtc3_send_cmd(SHTC3_CMD_SLEEP);
}

/********************************************************************
**函数名称:  shtc3_driver_init
**入口参数:  无
**出口参数:  无
**函数功能:  初始化 SHTC3 驱动，包括 I2C 总线和电源控制
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
int shtc3_driver_init(void)
{
    int ret;
    uint16_t id;

    /* 检查 I2C 设备 */
    if (!device_is_ready(g_i2c_dev))
    {
        LOG_ERR("SHTC3 I2C device not ready");
        return -ENODEV;
    }

    /* 等待传感器上电稳定 */
    k_sleep(K_MSEC(5));

    /* 防止设备异常重启后,sensor处于休眠状态,需先唤醒 */
    ret = shtc3_driver_wakeup();
    if (ret < 0)
    {
        LOG_ERR("shtc3_driver_wakeup fail");
        return ret;
    }

    ret = shtc3_send_cmd(SHTC3_CMD_SOFT_RESET);
    if (ret < 0)
    {
        LOG_ERR("SHTC3 soft reset failed: %d", ret);
        return ret;
    }
    k_sleep(K_MSEC(1));

    /* 读取 ID 验证通信 */
    ret = shtc3_driver_read_id(&id);
    if (ret < 0)
    {
        LOG_ERR("SHTC3 read ID failed: %d", ret);
        return ret;
    }
    LOG_INF("SHTC3 ID: 0x%04X", id);

    /* 初始化完成后进入休眠 */
    ret = shtc3_driver_sleep();
    if (ret < 0)
    {
        LOG_WRN("SHTC3 enter sleep failed after init: %d", ret);
    }

    LOG_INF("SHTC3 driver initialized");
    return 0;
}

/********************************************************************
**函数名称:  shtc3_driver_read_id
**入口参数:  id       ---        ID 存储指针（输出）
**出口参数:  id       ---        传感器设备 ID
**函数功能:  读取 SHTC3 设备 ID
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
int shtc3_driver_read_id(uint16_t *id)
{
    int ret;
    uint8_t rx_buf[3];

    /* 唤醒传感器 */
    ret = shtc3_driver_wakeup();
    if (ret < 0)
    {
        return ret;
    }

    /* 发送读 ID 命令 */
    ret = shtc3_send_cmd(SHTC3_CMD_READ_ID);
    if (ret < 0)
    {
        goto END;
    }
    k_sleep(K_MSEC(1));

    /* 读取 3 字节：ID_MSB, ID_LSB, CRC */
    ret = i2c_read(g_i2c_dev, rx_buf, 3, SHTC3_I2C_ADDR);
    if (ret < 0)
    {
        goto END;
    }

    /* CRC 校验 */
    if (shtc3_crc8(rx_buf, 2) != rx_buf[2])
    {
        LOG_ERR("SHTC3 read ID CRC error");
        ret = -EIO;
        goto END;
    }

    *id = ((uint16_t)rx_buf[0] << 8) | rx_buf[1];

END:
    /* 读完后休眠 */
    shtc3_driver_sleep();
    return ret;
}

/********************************************************************
**函数名称:  shtc3_driver_read_measurement
**入口参数:  temperature  ---        温度存储指针（输出）
**           humidity     ---        湿度存储指针（输出）
**出口参数:  temperature  ---        温度值，单位 0.01°C
**           humidity     ---        湿度值，单位 0.01%RH
**函数功能:  执行一次完整测量（唤醒→测量→休眠）
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
int shtc3_driver_read_measurement(int16_t *temperature, uint16_t *humidity)
{
    int ret;
    uint8_t rx_buf[6];
    uint16_t raw_temp;
    uint16_t raw_humi;
    int32_t temp_calc;
    uint32_t humi_calc;

    /* 唤醒传感器 */
    ret = shtc3_driver_wakeup();
    if (ret < 0)
    {
        return ret;
    }

    /* 发送测量命令（正常模式，温度优先） */
    ret = shtc3_send_cmd(SHTC3_CMD_MEASURE);
    if (ret < 0)
    {
        goto END;
    }

    /* 等待测量完成（低功耗模式约 1.5ms，取 3ms 保证余量） */
    k_sleep(K_MSEC(3));

    /* 读取 6 字节：T_MSB, T_LSB, T_CRC, H_MSB, H_LSB, H_CRC */
    ret = i2c_read(g_i2c_dev, rx_buf, 6, SHTC3_I2C_ADDR);
    if (ret < 0)
    {
        goto END;
    }

    /* 温度 CRC 校验 */
    if (shtc3_crc8(&rx_buf[0], 2) != rx_buf[2])
    {
        LOG_ERR("SHTC3 temperature CRC error");
        ret = -EIO;
        goto END;
    }

    /* 湿度 CRC 校验 */
    if (shtc3_crc8(&rx_buf[3], 2) != rx_buf[5])
    {
        LOG_ERR("SHTC3 humidity CRC error");
        ret = -EIO;
        goto END;
    }

    /* 组合原始数据 */
    raw_temp = ((uint16_t)rx_buf[0] << 8) | rx_buf[1];
    raw_humi = ((uint16_t)rx_buf[3] << 8) | rx_buf[4];

    /* 温度转换：T = -45 + 175 * raw / 65535，单位 0.01°C */
    temp_calc = (int32_t)raw_temp;
    temp_calc = -4500 + (17500 * temp_calc) / 65535;
    *temperature = (int16_t)temp_calc;

    /* 湿度转换：RH = 100 * raw / 65535，单位 0.01%RH */
    humi_calc = (uint32_t)raw_humi;
    humi_calc = (10000 * humi_calc) / 65535;
    *humidity = (uint16_t)humi_calc;

END:
    /* 测量完成后休眠 */
    shtc3_driver_sleep();
    return ret;
}
