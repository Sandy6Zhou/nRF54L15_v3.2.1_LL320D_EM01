/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        nfc_api.c
**文件描述:        NFC 模块统一 API 接口实现文件
**当前版本:        V1.0
**作    者:        Harrison Wu (wuyujiao@jimiiot.com)
**完成日期:        2026.02.28
*********************************************************************
** 功能描述:        封装 FM175XX 驱动，提供统一 NFC 接口
*********************************************************************/

#include "nfc_api.h"
#include "../drivers/FM175XX/inc/fm175xx_driver.h"
#include "../drivers/FM175XX/inc/nfc_mifare.h"
#include "../drivers/FM175XX/inc/nfc_reader_api.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(nfc_api, LOG_LEVEL_INF);

/* 内部状态 */
static bool g_nfc_initialized = false;
static nfc_card_detected_cb_t g_card_detected_cb = NULL;

/********************************************************************
**函数名称:  nfc_api_init
**入口参数:  cb       ---        卡片检测回调函数指针
**出口参数:  无
**函数功能:  初始化 NFC 模块，包括底层驱动和硬件复位
**返 回 值:  NFC_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
nfc_result_t nfc_api_init(nfc_card_detected_cb_t cb)
{
    int ret;

    if (g_nfc_initialized)
    {
        LOG_WRN("NFC already initialized");
        return NFC_SUCCESS;
    }

    /* 初始化底层驱动 */
    ret = fm175xx_driver_init();
    if (ret != 0)
    {
        LOG_ERR("FM175XX driver init failed: %d", ret);
        return NFC_ERROR_INIT;
    }

    /* 保存回调函数 */
    g_card_detected_cb = cb;
    g_nfc_initialized = true;

    LOG_INF("NFC API initialized successfully");
    return NFC_SUCCESS;
}

/********************************************************************
**函数名称:  nfc_api_deinit
**入口参数:  无
**出口参数:  无
**函数功能:  反初始化 NFC 模块，释放资源
**返 回 值:  NFC_SUCCESS 表示成功
*********************************************************************/
nfc_result_t nfc_api_deinit(void)
{
    if (!g_nfc_initialized)
    {
        return NFC_SUCCESS;
    }

    fm175xx_driver_deinit();
    g_card_detected_cb = NULL;
    g_nfc_initialized = false;

    LOG_INF("NFC API deinitialized");
    return NFC_SUCCESS;
}

/********************************************************************
**函数名称:  nfc_api_poll_start
**入口参数:  无
**出口参数:  无
**函数功能:  启动 NFC 卡片轮询检测
**返 回 值:  NFC_SUCCESS 表示成功
*********************************************************************/
nfc_result_t nfc_api_poll_start(void)
{
    if (!g_nfc_initialized)
    {
        LOG_ERR("NFC not initialized");
        return NFC_ERROR_INIT;
    }

    fm175xx_poll_start(g_card_detected_cb);
    return NFC_SUCCESS;
}

/********************************************************************
**函数名称:  nfc_api_poll_stop
**入口参数:  无
**出口参数:  无
**函数功能:  停止 NFC 卡片轮询检测
**返 回 值:  NFC_SUCCESS 表示成功
*********************************************************************/
nfc_result_t nfc_api_poll_stop(void)
{
    if (!g_nfc_initialized)
    {
        return NFC_SUCCESS;
    }

    fm175xx_poll_stop();
    return NFC_SUCCESS;
}

/********************************************************************
**函数名称:  nfc_api_read_block
**入口参数:  block    ---        块地址
**           data     ---        数据缓冲区指针
**           len      ---        缓冲区长度
**出口参数:  data     ---        读取到的数据
**函数功能:  读取 NFC 卡片指定块数据
**返 回 值:  NFC_SUCCESS 表示成功
*********************************************************************/
nfc_result_t nfc_api_read_block(uint8_t block, uint8_t *data, uint8_t len)
{
    int ret;

    if (!g_nfc_initialized || data == NULL || len == 0)
    {
        return NFC_ERROR_PARAM;
    }

    ret = nfc_mifare_read_block(block, data);
    if (ret != FM175XX_SUCCESS)
    {
        return NFC_ERROR_COMM;
    }

    return NFC_SUCCESS;
}

/********************************************************************
**函数名称:  nfc_api_write_block
**入口参数:  block    ---        块地址
**           data     ---        要写入的数据指针
**           len      ---        数据长度
**出口参数:  无
**函数功能:  向 NFC 卡片指定块写入数据
**返 回 值:  NFC_SUCCESS 表示成功
*********************************************************************/
nfc_result_t nfc_api_write_block(uint8_t block, const uint8_t *data, uint8_t len)
{
    int ret;

    if (!g_nfc_initialized || data == NULL || len == 0)
    {
        return NFC_ERROR_PARAM;
    }

    ret = nfc_mifare_write_block(block, (uint8_t *)data);
    if (ret != FM175XX_SUCCESS)
    {
        return NFC_ERROR_COMM;
    }

    return NFC_SUCCESS;
}

/********************************************************************
**函数名称:  nfc_api_get_version
**入口参数:  无
**出口参数:  version  ---        版本号存储指针
**函数功能:  获取 NFC 芯片版本号
**返 回 值:  NFC_SUCCESS 表示成功
*********************************************************************/
nfc_result_t nfc_api_get_version(uint8_t *version)
{
    int ret;

    if (version == NULL)
    {
        return NFC_ERROR_PARAM;
    }

    ret = fm175xx_get_version(version);
    if (ret != 0)
    {
        return NFC_ERROR_COMM;
    }

    return NFC_SUCCESS;
}

/********************************************************************
**函数名称:  nfc_api_enter_hpd
**入口参数:  无
**出口参数:  无
**函数功能:  进入 Hard Power Down (HPD) 低功耗模式
**返 回 值:  NFC_SUCCESS 表示成功
*********************************************************************/
nfc_result_t nfc_api_enter_hpd(void)
{
    fm175xx_enter_hpd();
    return NFC_SUCCESS;
}

/********************************************************************
**函数名称:  nfc_api_exit_hpd
**入口参数:  无
**出口参数:  无
**函数功能:  退出 HPD 模式，恢复 NFC 功能
**返 回 值:  NFC_SUCCESS 表示成功
*********************************************************************/
nfc_result_t nfc_api_exit_hpd(void)
{
    int ret;

    ret = fm175xx_exit_hpd();
    if (ret != 0)
    {
        return NFC_ERROR_COMM;
    }

    return NFC_SUCCESS;
}
