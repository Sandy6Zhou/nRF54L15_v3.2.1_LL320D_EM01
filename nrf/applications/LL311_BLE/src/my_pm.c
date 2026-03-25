/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        my_pm.c
**文件描述:        全模块统一电源管理实现文件
**当前版本:        V1.0
**作    者:        Harrison Wu (wuyujiao@jimiiot.com)
**完成日期:        2026.03.19
*********************************************************************
** 功能描述:        1. 实现全模块统一的电源管理接口
**                 2. 使用 pm_device_action_run 管理 I2C21/I2C22/UART 总线
**                 3. 支持独立总线的引用计数管理
**                 4. 提供统一的设备状态机管理
**                 5. I2C21: G-Sensor, I2C22: NFC, UART: LTE
*********************************************************************/

/* 必须在包含 my_comm.h 之前定义 BLE_LOG_MODULE_ID，避免与 my_ble_log.h 中的默认定义冲突 */
#define BLE_LOG_MODULE_ID BLE_LOG_MOD_MAIN

#include "my_comm.h"

/* 注册 PM 模块日志 */
LOG_MODULE_REGISTER(my_pm, LOG_LEVEL_INF);

/* ========== I2C22 总线设备节点（NFC使用） ========== */
#define I2C22_NODE DT_ALIAS(nfc_i2c)

/* ========== I2C21 总线设备节点（G-Sensor使用） ========== */
#define I2C21_NODE DT_ALIAS(i2c21)

/* ========== LTE UART 总线设备节点 ========== */
#define LTE_UART_NODE DT_ALIAS(lte_uart)

/* ========== 设备上下文数组 ========== */
static struct PM_DEVICE_CTX l_st_pm_devices[MY_PM_DEV_MAX];

/* ========== 总线引用计数（仅 I2C 总线需要，UART 为点对点通信无需计数） ========== */
static uint8_t l_u8_i2c22_ref_count = 0;
static uint8_t l_u8_i2c21_ref_count = 0;

/********************************************************************
**函数名称:  my_pm_get_i2c22_device
**入口参数:  无
**出口参数:  无
**函数功能:  获取 I2C22 总线设备句柄
**返 回 值:  I2C 设备指针，未就绪返回 NULL
*********************************************************************/
static const struct device *my_pm_get_i2c22_device(void)
{
    const struct device *dev = DEVICE_DT_GET_OR_NULL(I2C22_NODE);
    return dev;
}

/********************************************************************
**函数名称:  my_pm_i2c22_resume
**入口参数:  无
**出口参数:  无
**函数功能:  恢复 I2C22 总线（引用计数管理）
**           只有第一个引用者才实际调用 pm_device_action_run
**返 回 值:  0 表示成功，负值表示错误码
*********************************************************************/
static int my_pm_i2c22_resume(void)
{
    const struct device *dev = my_pm_get_i2c22_device();
    int ret = 0;

    if (dev == NULL)
    {
        MY_LOG_ERR("I2C22 device not found");
        return -ENODEV;
    }

    /* 引用计数增加 */
    l_u8_i2c22_ref_count++;

    /* 只有第一个引用者才实际 resume 总线 */
    if (l_u8_i2c22_ref_count == 1)
    {
        ret = pm_device_action_run(dev, PM_DEVICE_ACTION_RESUME);
        if (ret < 0 && ret != -EALREADY)
        {
            MY_LOG_ERR("I2C22 resume failed: %d", ret);
            l_u8_i2c22_ref_count--; /* 恢复失败，回退引用计数 */
            return ret;
        }
        MY_LOG_DBG("I2C22 resumed (ref_count=%d)", l_u8_i2c22_ref_count);
    }
    else
    {
        MY_LOG_DBG("I2C22 already resumed (ref_count=%d)", l_u8_i2c22_ref_count);
    }

    return 0;
}

/********************************************************************
**函数名称:  my_pm_i2c22_suspend
**入口参数:  无
**出口参数:  无
**函数功能:  挂起 I2C22 总线（引用计数管理）
**           只有最后一个释放者才实际调用 pm_device_action_run
**返 回 值:  0 表示成功，负值表示错误码
*********************************************************************/
static int my_pm_i2c22_suspend(void)
{
    const struct device *dev = my_pm_get_i2c22_device();
    int ret = 0;

    if (dev == NULL)
    {
        MY_LOG_ERR("I2C22 device not found");
        return -ENODEV;
    }

    /* 检查引用计数 */
    if (l_u8_i2c22_ref_count == 0)
    {
        MY_LOG_WRN("I2C22 suspend called with ref_count=0");
        return -EINVAL;
    }

    /* 引用计数减少 */
    l_u8_i2c22_ref_count--;

    /* 只有最后一个释放者才实际 suspend 总线 */
    if (l_u8_i2c22_ref_count == 0)
    {
        ret = pm_device_action_run(dev, PM_DEVICE_ACTION_SUSPEND);
        if (ret < 0 && ret != -EALREADY)
        {
            MY_LOG_ERR("I2C22 suspend failed: %d", ret);
            l_u8_i2c22_ref_count++; /* 恢复失败，回退引用计数 */
            return ret;
        }
        MY_LOG_DBG("I2C22 suspended (ref_count=0)");
    }
    else
    {
        MY_LOG_DBG("I2C22 keep resumed (ref_count=%d)", l_u8_i2c22_ref_count);
    }

    return 0;
}

/********************************************************************
**函数名称:  my_pm_get_i2c21_device
**入口参数:  无
**出口参数:  无
**函数功能:  获取 I2C21 总线设备句柄
**返 回 值:  I2C 设备指针，未就绪返回 NULL
*********************************************************************/
static const struct device *my_pm_get_i2c21_device(void)
{
    const struct device *dev = DEVICE_DT_GET_OR_NULL(I2C21_NODE);
    return dev;
}

/********************************************************************
**函数名称:  my_pm_i2c21_resume
**入口参数:  无
**出口参数:  无
**函数功能:  恢复 I2C21 总线（引用计数管理）
**           只有第一个引用者才实际调用 pm_device_action_run
**返 回 值:  0 表示成功，负值表示错误码
*********************************************************************/
static int my_pm_i2c21_resume(void)
{
    const struct device *dev = my_pm_get_i2c21_device();
    int ret = 0;

    if (dev == NULL)
    {
        MY_LOG_ERR("I2C21 device not found");
        return -ENODEV;
    }

    /* 引用计数增加 */
    l_u8_i2c21_ref_count++;

    /* 只有第一个引用者才实际 resume 总线 */
    if (l_u8_i2c21_ref_count == 1)
    {
        ret = pm_device_action_run(dev, PM_DEVICE_ACTION_RESUME);
        if (ret < 0 && ret != -EALREADY)
        {
            MY_LOG_ERR("I2C21 resume failed: %d", ret);
            l_u8_i2c21_ref_count--; /* 恢复失败，回退引用计数 */
            return ret;
        }
        MY_LOG_DBG("I2C21 resumed (ref_count=%d)", l_u8_i2c21_ref_count);
    }
    else
    {
        MY_LOG_DBG("I2C21 already resumed (ref_count=%d)", l_u8_i2c21_ref_count);
    }

    return 0;
}

/********************************************************************
**函数名称:  my_pm_i2c21_suspend
**入口参数:  无
**出口参数:  无
**函数功能:  挂起 I2C21 总线（引用计数管理）
**           只有最后一个释放者才实际调用 pm_device_action_run
**返 回 值:  0 表示成功，负值表示错误码
*********************************************************************/
static int my_pm_i2c21_suspend(void)
{
    const struct device *dev = my_pm_get_i2c21_device();
    int ret = 0;

    if (dev == NULL)
    {
        MY_LOG_ERR("I2C21 device not found");
        return -ENODEV;
    }

    /* 检查引用计数 */
    if (l_u8_i2c21_ref_count == 0)
    {
        MY_LOG_WRN("I2C21 suspend called with ref_count=0");
        return -EINVAL;
    }

    /* 引用计数减少 */
    l_u8_i2c21_ref_count--;

    /* 只有最后一个释放者才实际 suspend 总线 */
    if (l_u8_i2c21_ref_count == 0)
    {
        ret = pm_device_action_run(dev, PM_DEVICE_ACTION_SUSPEND);
        if (ret < 0 && ret != -EALREADY)
        {
            MY_LOG_ERR("I2C21 suspend failed: %d", ret);
            l_u8_i2c21_ref_count++; /* 恢复失败，回退引用计数 */
            return ret;
        }
        MY_LOG_DBG("I2C21 suspended (ref_count=0)");
    }
    else
    {
        MY_LOG_DBG("I2C21 keep resumed (ref_count=%d)", l_u8_i2c21_ref_count);
    }

    return 0;
}

/********************************************************************
**函数名称:  my_pm_get_lte_uart_device
**入口参数:  无
**出口参数:  无
**函数功能:  获取 LTE UART 设备句柄
**返 回 值:  UART 设备指针，未就绪返回 NULL
*********************************************************************/
static const struct device *my_pm_get_lte_uart_device(void)
{
    const struct device *dev = DEVICE_DT_GET_OR_NULL(LTE_UART_NODE);
    return dev;
}

/********************************************************************
**函数名称:  my_pm_lte_uart_resume
**入口参数:  无
**出口参数:  无
**函数功能:  恢复 LTE UART 总线（点对点通信，直接操作）
**返 回 值:  0 表示成功，负值表示错误码
*********************************************************************/
static int my_pm_lte_uart_resume(void)
{
    const struct device *dev = my_pm_get_lte_uart_device();
    int ret = 0;

    if (dev == NULL)
    {
        MY_LOG_ERR("LTE UART device not found");
        return -ENODEV;
    }

    ret = pm_device_action_run(dev, PM_DEVICE_ACTION_RESUME);
    if (ret < 0 && ret != -EALREADY)
    {
        MY_LOG_ERR("LTE UART resume failed: %d", ret);
        return ret;
    }
    MY_LOG_DBG("LTE UART resumed");

    return 0;
}

/********************************************************************
**函数名称:  my_pm_lte_uart_suspend
**入口参数:  无
**出口参数:  无
**函数功能:  挂起 LTE UART 总线（点对点通信，直接操作）
**返 回 值:  0 表示成功，负值表示错误码
*********************************************************************/
static int my_pm_lte_uart_suspend(void)
{
    const struct device *dev = my_pm_get_lte_uart_device();
    int ret = 0;

    if (dev == NULL)
    {
        MY_LOG_ERR("LTE UART device not found");
        return -ENODEV;
    }

    ret = pm_device_action_run(dev, PM_DEVICE_ACTION_SUSPEND);
    if (ret < 0 && ret != -EALREADY)
    {
        MY_LOG_ERR("LTE UART suspend failed: %d", ret);
        return ret;
    }
    MY_LOG_DBG("LTE UART suspended");

    return 0;
}

/********************************************************************
**函数名称:  my_pm_device_register
**入口参数:  dev_id   ---        设备ID
**           ops      ---        设备操作回调结构体指针
**出口参数:  无
**函数功能:  注册设备的电源管理回调函数
**返 回 值:  0 表示成功，负值表示错误码
*********************************************************************/
int my_pm_device_register(MY_PM_DEV_ID_T dev_id, const struct PM_DEVICE_OPS *ops)
{
    if (dev_id >= MY_PM_DEV_MAX)
    {
        MY_LOG_ERR("Invalid device ID: %d", dev_id);
        return -EINVAL;
    }

    if (ops == NULL)
    {
        MY_LOG_ERR("NULL ops for device %d", dev_id);
        return -EINVAL;
    }

    /* 初始化设备状态为关闭状态 */
    l_st_pm_devices[dev_id].state = MY_PM_STATE_OFF;
    l_st_pm_devices[dev_id].ops = ops;

    /* 如果提供了 init 回调，立即执行初始化 */
    if (ops->init != NULL)
    {
        int ret = ops->init();
        if (ret < 0)
        {
            MY_LOG_ERR("Device %d init failed: %d", dev_id, ret);
            l_st_pm_devices[dev_id].ops = NULL;
            return ret;
        }
        MY_LOG_INF("Device %d initialized", dev_id);
    }

    MY_LOG_INF("Device %d registered", dev_id);
    return 0;
}

/********************************************************************
**函数名称:  my_pm_device_resume
**入口参数:  dev_id   ---        设备ID
**出口参数:  无
**函数功能:  恢复指定设备到正常运行状态
**           1. 如果是 I2C 设备，先 resume 总线（引用计数）
**           2. 调用模块的 resume 回调
**返 回 值:  0 表示成功，负值表示错误码
*********************************************************************/
int my_pm_device_resume(MY_PM_DEV_ID_T dev_id)
{
    struct PM_DEVICE_CTX *ctx;
    int ret = 0;

    if (dev_id >= MY_PM_DEV_MAX)
    {
        MY_LOG_ERR("Invalid device ID: %d", dev_id);
        return -EINVAL;
    }

    /* 获取设备上下文 */
    ctx = &l_st_pm_devices[dev_id];

    if (ctx->ops == NULL)
    {
        MY_LOG_ERR("Device %d not registered", dev_id);
        return -ENODEV;
    }

    /* 检查当前状态，避免重复 resume */
    if (ctx->state == MY_PM_STATE_RESUME)
    {
        MY_LOG_DBG("Device %d already resumed", dev_id);
        return 0;
    }

    MY_LOG_INF("Resuming device %d from state %d", dev_id, ctx->state);

    /* 根据设备ID，调用相应的 resume 函数 */
    switch (dev_id)
    {
        case MY_PM_DEV_NFC:
            ret = my_pm_i2c22_resume();
            break;

        case MY_PM_DEV_GSENSOR:
            ret = my_pm_i2c21_resume();
            break;

        case MY_PM_DEV_LTE:
            ret = my_pm_lte_uart_resume();
            break;

        default:
            MY_LOG_ERR("Device %d not supported", dev_id);
            return -ENOTSUP;
    }

    /* 检查 resume 是否成功 */
    if (ret < 0)
    {
        MY_LOG_ERR("Device %d resume failed: %d", dev_id, ret);
        return ret;
    }

    /* 调用 resume 回调, 如果有ops->resume()，但回调失败，需要回退总线状态，避免设备永久无法使用 */
    if (ctx->ops->resume != NULL)
    {
        ret = ctx->ops->resume();
        if (ret < 0)
        {
            MY_LOG_ERR("Device %d resume failed: %d", dev_id, ret);

            /* resume 失败，回退总线状态，避免设备永久无法使用 */
            switch (dev_id)
            {
                case MY_PM_DEV_NFC:
                    ret = my_pm_i2c22_suspend();
                    break;

                case MY_PM_DEV_GSENSOR:
                    ret = my_pm_i2c21_suspend();
                    break;

                case MY_PM_DEV_LTE:
                    ret = my_pm_lte_uart_suspend();
                    break;

                default:
                    MY_LOG_ERR("Device %d not supported", dev_id);
                    return -ENOTSUP;
            }

            /* resume 失败，回退总线状态也失败 */
            if (ret < 0)
            {
                MY_LOG_WRN("Device %d failed to rollback bus after resume failure: %d", dev_id, ret);
            }
            return ret;
        }
    }

    /* 更新设备状态为活动状态 */
    ctx->state = MY_PM_STATE_RESUME;
    MY_LOG_INF("Device %d resumed successfully", dev_id);
    return 0;
}

/********************************************************************
**函数名称:  my_pm_device_suspend
**入口参数:  dev_id   ---        设备ID
**出口参数:  无
**函数功能:  挂起指定设备，进入低功耗状态
**           1. 调用模块的 suspend 回调
**           2. 如果是 I2C 设备，最后 suspend 总线（引用计数）
**返 回 值:  0 表示成功，负值表示错误码
*********************************************************************/
int my_pm_device_suspend(MY_PM_DEV_ID_T dev_id)
{
    struct PM_DEVICE_CTX *ctx;
    int ret = 0;

    if (dev_id >= MY_PM_DEV_MAX)
    {
        MY_LOG_ERR("Invalid device ID: %d", dev_id);
        return -EINVAL;
    }

    /* 获取设备上下文 */
    ctx = &l_st_pm_devices[dev_id];

    if (ctx->ops == NULL)
    {
        MY_LOG_ERR("Device %d not registered", dev_id);
        return -ENODEV;
    }

    /* 检查当前状态, 确保设备处于挂起状态 */
    if (ctx->state == MY_PM_STATE_SUSPENDED)
    {
        MY_LOG_DBG("Device %d already suspended", dev_id);
        return 0;
    }

    MY_LOG_INF("Suspending device %d from state %d", dev_id, ctx->state);

    /* 根据设备ID，调用相应的 suspend 函数 */
    switch (dev_id)
    {
        case MY_PM_DEV_NFC:
            ret = my_pm_i2c22_suspend();
            break;

        case MY_PM_DEV_GSENSOR:
            ret = my_pm_i2c21_suspend();
            break;

        case MY_PM_DEV_LTE:
            ret = my_pm_lte_uart_suspend();
            break;

        default:
            MY_LOG_ERR("Device %d not supported", dev_id);
            return -ENOTSUP;
    }

    /* 检查 suspend 是否成功 */
    if (ret < 0)
    {
        MY_LOG_ERR("Device %d suspend failed: %d", dev_id, ret);
        return ret;
    }

    /* 调用模块的 suspend 回调，如果有ops->suspend()，但回调失败，需要恢复设备状态，避免设备永久无法使用 */
    if (ctx->ops->suspend != NULL)
    {
        ret = ctx->ops->suspend();
        if (ret < 0)
        {
            MY_LOG_ERR("Device %d suspend failed: %d", dev_id, ret);

            /* suspend 失败，尝试恢复设备状态，避免设备永久无法使用 */
            switch (dev_id)
            {
                case MY_PM_DEV_NFC:
                    ret = my_pm_i2c22_resume();
                    break;

                case MY_PM_DEV_GSENSOR:
                    ret = my_pm_i2c21_resume();
                    break;

                case MY_PM_DEV_LTE:
                    ret = my_pm_lte_uart_resume();
                    break;

                default:
                    MY_LOG_ERR("Device %d not supported", dev_id);
                    return -ENOTSUP;
            }

            /* suspend 失败，尝试恢复设备状态也失败 */
            if (ret < 0)
            {
                MY_LOG_WRN("Device %d Failed to rollback state after suspend failure: %d", dev_id, ret);
            }
            return ret;
        }
    }

    /* 更新设备状态为挂起状态 */
    ctx->state = MY_PM_STATE_SUSPENDED;
    MY_LOG_INF("Device %d suspended successfully", dev_id);
    return 0;
}

/********************************************************************
**函数名称:  my_pm_device_get_state
**入口参数:  dev_id   ---        设备ID
**出口参数:  无
**函数功能:  获取指定设备的当前电源状态
**返 回 值:  当前状态（pm_state_t）
*********************************************************************/
MY_PM_STATE_T my_pm_device_get_state(MY_PM_DEV_ID_T dev_id)
{
    if (dev_id >= MY_PM_DEV_MAX)
    {
        MY_LOG_ERR("Invalid device ID: %d", dev_id);
        return MY_PM_STATE_OFF;
    }

    return l_st_pm_devices[dev_id].state;
}

/********************************************************************
**函数名称:  my_pm_init
**入口参数:  无
**出口参数:  无
**函数功能:  初始化电源管理子系统
**返 回 值:  0 表示成功
*********************************************************************/
int my_pm_init(void)
{
    /* 清零设备上下文数组 */
    memset(l_st_pm_devices, 0, sizeof(l_st_pm_devices));

    /* 初始化 I2C 总线引用计数 */
    l_u8_i2c22_ref_count = 0;
    l_u8_i2c21_ref_count = 0;

    MY_LOG_INF("Power management subsystem initialized");
    return 0;
}
