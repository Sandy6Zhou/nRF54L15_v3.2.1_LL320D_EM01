/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        my_wdt.c
**文件描述:        看门狗管理实现文件
**当前版本:        V1.0
**作    者:        Harrison Wu (wuyujiao@jimiiot.com)
**完成日期:        2026.02.04
*********************************************************************
** 功能描述:        1. 实现系统看门狗初始化和喂狗功能
**                 2. 提供定时器自动喂狗机制
**                 3. 支持各模块线程检活机制
*********************************************************************/

/* 必须在包含 my_comm.h 之前定义 BLE_LOG_MODULE_ID，避免与 my_ble_log.h 中的默认定义冲突 */
#define BLE_LOG_MODULE_ID BLE_LOG_MOD_WDT

#include "my_comm.h"
#include <zephyr/drivers/watchdog.h>
#include "my_wdt.h"

/* 注册看门狗模块日志 */
LOG_MODULE_REGISTER(my_wdt, LOG_LEVEL_INF);

/* 看门狗设备 */
#define WDT_NODE DT_NODELABEL(wdt31)
static const struct device *wdt_dev = DEVICE_DT_GET(WDT_NODE);

/* 看门狗通道 ID */
static int s_wdt_channel_id = -1;

/* 看门狗配置参数（90秒超时） */
#define WDT_TIMEOUT_MS 90000
#define WDT_FEED_INTERVAL_MS 30000  /* 每30秒喂一次狗 */

/********************************************************************
**函数名称:  wdt_feed_timer_callback
**入口参数:  p1    ---        定时器指针 (实际传入的是 k_timer，但签名需匹配 TIMER_FUN)
**出口参数:  无
**函数功能:  定时器回调，定期喂狗
**返 回 值:  无
*********************************************************************/
static void wdt_feed_timer_callback(void *p1)
{
    ARG_UNUSED(p1);

    /* 喂狗：重置看门狗倒计时，防止系统复位
     * wdt_feed() 标注了 isr-ok 属性，可在中断上下文中安全调用。
     */
    wdt_feed(wdt_dev, s_wdt_channel_id);
}

/********************************************************************
**函数名称:  my_wdt_init
**入口参数:  无
**出口参数:  无
**函数功能:  初始化看门狗并启动喂狗定时器
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
int my_wdt_init(void)
{
    int err;

    /* 检查看门狗设备是否就绪 */
    if (!device_is_ready(wdt_dev))
    {
        MY_LOG_ERR("Watchdog device not ready");
        return -ENODEV;
    }

    /* 配置看门狗选项 */
    struct wdt_timeout_cfg wdt_config = {
        .flags = WDT_FLAG_RESET_SOC,  /* 超时后复位整个系统 */
        .window.min = 0,              /* 最小窗口 */
        .window.max = WDT_TIMEOUT_MS, /* 最大超时时间 */
        .callback = NULL,             /* 不使用回调，直接复位 */
    };

    /* 安装看门狗 */
    s_wdt_channel_id = wdt_install_timeout(wdt_dev, &wdt_config);
    if (s_wdt_channel_id < 0)
    {
        MY_LOG_ERR("Failed to install watchdog timeout (err %d)", s_wdt_channel_id);
        return s_wdt_channel_id;
    }

    /* 启动看门狗，不设置任何暂停选项（options = 0） */
    err = wdt_setup(wdt_dev, 0);
    if (err)
    {
        MY_LOG_ERR("Failed to setup watchdog (err %d)", err);
        return err;
    }

    /* 启动周期定时器前先喂一次狗，确保首个周期窗口从当前时刻开始计算 */
    wdt_feed(wdt_dev, s_wdt_channel_id);

    /* 启动定时喂狗定时器（周期性） */
    my_start_timer(MY_TIMER_WDT_FEED, WDT_FEED_INTERVAL_MS, true, wdt_feed_timer_callback);

    MY_LOG_INF("Watchdog initialized (timeout=%dms, feed_interval=%dms)",
            WDT_TIMEOUT_MS, WDT_FEED_INTERVAL_MS);

    return 0;
}
