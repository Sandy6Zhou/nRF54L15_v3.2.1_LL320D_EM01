/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        my_pm.c
**文件描述:        全模块统一电源管理实现文件
**当前版本:        V1.0
**作    者:        Harrison Wu (wuyujiao@jimiiot.com)
**完成日期:        2026.03.19
*********************************************************************
** 功能描述:  全模块统一电源管理框架
**
** 核心功能:
**   1. 设备生命周期管理：注册 → SUSPENDED(低功耗) ↔ RESUME(工作)
**   2. 总线电源管理：通过 pm_device_runtime_get/put 控制 I2C/UART 状态
**
** 总线分配:
**   - I2C22: NFC 模块
**   - I2C21: G-Sensor 模块
**   - UART : LTE 模块
**
** 设计特点:
**   - 单设备独占总线，无需引用计数
**   - 注册后默认进入 SUSPENDED 状态（低功耗就绪）
**   - 各模块在线程内完成注册和初始化
**
** 扩展提示:
**   如未来需多设备共享总线，需引入引用计数机制
*********************************************************************/

/* 必须在包含 my_comm.h 之前定义 BLE_LOG_MODULE_ID，避免与 my_ble_log.h 中的默认定义冲突 */
#define BLE_LOG_MODULE_ID BLE_LOG_MOD_MAIN

#include "my_comm.h"

/* 注册 PM 模块日志 */
LOG_MODULE_REGISTER(my_pm, LOG_LEVEL_INF);

/* ========== I2C22 总线设备节点（NFC使用） ========== */
#define I2C22_NODE DT_ALIAS(nfc_i2c)

/* ========== I2C21 总线设备节点（G-Sensor使用） ========== */
#define I2C21_NODE DT_ALIAS(gsensor_i2c)

/* ========== LTE UART 总线设备节点 ========== */
#define LTE_UART_NODE DT_ALIAS(lte_uart)

/* ========== 设备上下文数组 ========== */
static struct PM_DEVICE_CTX l_st_pm_devices[MY_PM_DEV_MAX];

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
**函数功能:  恢复 I2C22 总线（使用 Runtime PM API）
**           与 CONFIG_PM_DEVICE_RUNTIME=y 兼容
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

    /* 使用 Runtime PM API，引用计数+1，首次调用时自动 resume */
    ret = pm_device_runtime_get(dev);
    if (ret < 0)
    {
        MY_LOG_ERR("I2C22 runtime get failed: %d", ret);
        return ret;
    }

    MY_LOG_DBG("I2C22 runtime get OK");
    return 0;
}

/********************************************************************
**函数名称:  my_pm_i2c22_suspend
**入口参数:  无
**出口参数:  无
**函数功能:  挂起 I2C22 总线（使用 Runtime PM API）
**           与 CONFIG_PM_DEVICE_RUNTIME=y 兼容
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

    /* 使用 Runtime PM API，引用计数-1，为0时自动 suspend */
    ret = pm_device_runtime_put(dev);
    if (ret < 0)
    {
        MY_LOG_ERR("I2C22 runtime put failed: %d", ret);
        return ret;
    }

    MY_LOG_DBG("I2C22 runtime put OK");
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
**函数功能:  恢复 I2C21 总线（使用 Runtime PM API）
**           与 CONFIG_PM_DEVICE_RUNTIME=y 兼容
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

    ret = pm_device_runtime_get(dev);
    if (ret < 0)
    {
        MY_LOG_ERR("I2C21 runtime get failed: %d", ret);
        return ret;
    }

    MY_LOG_DBG("I2C21 runtime get OK");
    return 0;
}

/********************************************************************
**函数名称:  my_pm_i2c21_suspend
**入口参数:  无
**出口参数:  无
**函数功能:  挂起 I2C21 总线（使用 Runtime PM API）
**           与 CONFIG_PM_DEVICE_RUNTIME=y 兼容
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

    ret = pm_device_runtime_put(dev);
    if (ret < 0)
    {
        MY_LOG_ERR("I2C21 runtime put failed: %d", ret);
        return ret;
    }

    MY_LOG_DBG("I2C21 runtime put OK");
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
**函数功能:  恢复 LTE UART 总线（使用 Runtime PM API）
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

    ret = pm_device_runtime_get(dev);
    if (ret < 0)
    {
        MY_LOG_ERR("LTE UART runtime get failed: %d", ret);
        return ret;
    }

    MY_LOG_DBG("LTE UART runtime get OK");
    return 0;
}

/********************************************************************
**函数名称:  my_pm_lte_uart_suspend
**入口参数:  无
**出口参数:  无
**函数功能:  挂起 LTE UART 总线（使用 Runtime PM API）
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

    ret = pm_device_runtime_put(dev);
    if (ret < 0)
    {
        MY_LOG_ERR("LTE UART runtime put failed: %d", ret);
        return ret;
    }

    MY_LOG_DBG("LTE UART runtime put OK");
    return 0;
}

/********************************************************************
**函数名称:  my_pm_device_register
**入口参数:  dev_id   ---        设备ID
**           ops      ---        设备操作回调结构体指针
**出口参数:  无
**函数功能:  注册设备的电源管理回调函数
**           注册流程：init() → resume总线 → suspend总线 → state=SUSPENDED
**           设备注册后默认进入 SUSPENDED 状态（低功耗就绪）
**注意：     1. 此函数内部会自动完成总线 resume/suspend 操作，调用方无需手动 suspend
**           2. 如果注册后需要立即使用设备，需显式调用 pm_device_runtime_get()
**           3. 必须在设备对应的线程上下文内调用此函数
**返 回 值:  0 表示成功，负值表示错误码
*********************************************************************/
int my_pm_device_register(MY_PM_DEV_ID_T dev_id, const struct PM_DEVICE_OPS *ops)
{
    int ret;

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

    /* 初始化设备状态为未初始化状态 */
    l_st_pm_devices[dev_id].state = MY_PM_STATE_NOT_INIT;
    l_st_pm_devices[dev_id].ops = ops;

    /* 如果提供了 init 回调，执行完整初始化流程，最后将设备状态设置为 SUSPENDED */
    if (ops->init != NULL)
    {
        /* 先执行设备 init（配置 GPIO 等，检查设备树，不依赖总线），这里必须放到前面执行 */
        ret = ops->init();
        if (ret < 0)
        {
            MY_LOG_ERR("Device %d init failed: %d", dev_id, ret);
            l_st_pm_devices[dev_id].ops = NULL;
            return ret;
        }
        MY_LOG_INF("Device %d initialized", dev_id);

        /* 先执行一次 Resume 总线（确保总线可用） */
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
                ret = -ENOTSUP;
                break;
        }
        if (ret < 0)
        {
            MY_LOG_ERR("Failed to resume bus for device %d: %d", dev_id, ret);
            l_st_pm_devices[dev_id].ops = NULL;
            return ret;
        }

        /* 再执行一次 Suspend 总线（进入低功耗） */
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
                ret = -ENOTSUP;
                break;
        }
        if (ret < 0)
        {
            MY_LOG_WRN("Failed to suspend bus after init for device %d: %d", dev_id, ret);

            /* 初始化失败，标记为未初始化状态 */
            l_st_pm_devices[dev_id].state = MY_PM_STATE_NOT_INIT;
            l_st_pm_devices[dev_id].ops = NULL;
            return ret;
        }
    }

    /* 注册成功：设备默认状态为 SUSPENDED（低功耗就绪） */
    l_st_pm_devices[dev_id].state = MY_PM_STATE_SUSPENDED;
    MY_LOG_INF("Device %d registered in SUSPENDED state", dev_id);
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

    /* 只能从 SUSPENDED 状态恢复 */
    if (ctx->state != MY_PM_STATE_SUSPENDED)
    {
        MY_LOG_ERR("Device %d not in SUSPENDED state (state=%d)", dev_id, ctx->state);
        return -EINVAL;
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
    int rollback_ret = 0;

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

    /* 先执行模块 suspend 回调，确保设备在总线仍可访问时完成寄存器配置或收尾动作 */
    if (ctx->ops->suspend != NULL)
    {
        ret = ctx->ops->suspend();
        if (ret < 0)
        {
            MY_LOG_ERR("Device %d suspend failed: %d", dev_id, ret);
            return ret;
        }
    }

    /* 再根据设备ID挂起对应总线，避免模块回调阶段访问已挂起的总线 */
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
            MY_LOG_WRN("Device %d not supported", dev_id);
            break;
    }

    // 总线挂起失败时尝试恢复模块到工作态，避免设备内部状态与总线状态不一致
    if (ret < 0)
    {
        MY_LOG_ERR("Device %d suspend failed: %d", dev_id, ret);

        if (ctx->ops->resume != NULL)
        {
            rollback_ret = ctx->ops->resume();
            if (rollback_ret < 0)
            {
                MY_LOG_WRN("Device %d failed to rollback device state after suspend failure: %d", dev_id, rollback_ret);
            }
        }
        return ret;
    }

    /* 更新设备状态为 SUSPENDED */
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
        return MY_PM_STATE_NOT_INIT;
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

    MY_LOG_INF("Power management subsystem initialized");
    return 0;
}
