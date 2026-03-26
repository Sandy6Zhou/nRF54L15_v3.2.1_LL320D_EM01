/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        my_motor.c
**文件描述:        电机控制模块实现文件
**当前版本:        V1.0
**作    者:        周森达 (zhousenda@jimiiot.com)
**完成日期:        2026.02.04
*********************************************************************
** 功能描述:        1. 电机正反转、对应电源使能及检测中断
*********************************************************************/

/* 必须在包含 my_comm.h 之前定义 BLE_LOG_MODULE_ID，避免与 my_ble_log.h 中的默认定义冲突 */
#define BLE_LOG_MODULE_ID BLE_LOG_MOD_MOTOR

#include "my_comm.h"

LOG_MODULE_REGISTER(my_motor, LOG_LEVEL_INF);

static const struct gpio_dt_spec motor_a = GPIO_DT_SPEC_GET(DT_ALIAS(motor_ctrl_a), gpios);
static const struct gpio_dt_spec motor_b = GPIO_DT_SPEC_GET(DT_ALIAS(motor_ctrl_b), gpios);
static const struct gpio_dt_spec motor_pwr_en = GPIO_DT_SPEC_GET(DT_ALIAS(motor_pwr_ctrl), gpios);
static const struct gpio_dt_spec motor_pos_a = GPIO_DT_SPEC_GET(DT_ALIAS(motor_posdet_a), gpios);
static const struct gpio_dt_spec motor_pos_b = GPIO_DT_SPEC_GET(DT_ALIAS(motor_posdet_b), gpios);

#define LOCK_TIMEOUT_MAXTIME 5000   /* 暂定开/关锁最长时间：5000ms,后续根据实际情况调整 */
#define LOCK_DEBOUNCE_MS    100    /* 消抖时间：100ms */

typedef enum
{
    OPEN_LOCK,
    CLOSE_LOCK,
} LOCK_STATE;

typedef struct
{
    struct k_timer timer;       /* 消抖定时器 */
    bool state;                 /* 当前开/关锁状态（true=到达限位，false=未到限位） */
    bool debouncing;            /* 消抖中标志 */
} lock_posdet;

static struct gpio_callback motor_pos_cb;
/* 电机超时定时器，用于开锁/关锁操作的超时保护 */
static struct k_timer motor_timeout_timer;
/* 开锁位置检测结构，用于检测开锁限位状态 */
static lock_posdet openlock_posdet;
/* 关锁位置检测结构，用于检测关锁限位状态 */
static lock_posdet closelock_posdet;
/* 开锁状态标志，true表示正在开锁 */
static bool s_bOpenLockingState = false;

/* 电源控制：1 = 打开，0 = 关闭 */
void motor_power_set(bool on)
{
    LOG_INF("%s:%d", __func__, on);
    gpio_pin_set_dt(&motor_pwr_en, on ? 1 : 0);
}

/********************************************************************
**函数名称:  req_open_lock_action
**入口参数:  无
**出口参数:  无
**函数功能:  请求执行上锁操作
**返 回 值:  无
*********************************************************************/
void req_open_lock_action(void)
{
    /* 标记正在开锁中 */
    s_bOpenLockingState = true;
    k_timer_stop(&motor_timeout_timer);

    MY_LOG_INF("%s:run", __func__);
    /* A 高，B 低 -> 正转（具体取决于你的驱动电路） */
    gpio_pin_set_dt(&motor_a, 1);
    gpio_pin_set_dt(&motor_b, 0);
    k_timer_start(&motor_timeout_timer, K_MSEC(LOCK_TIMEOUT_MAXTIME), K_NO_WAIT);
}

/********************************************************************
**函数名称:  req_close_lock_action
**入口参数:  无
**出口参数:  无
**函数功能:  请求执行关锁操作
**返 回 值:  无
*********************************************************************/
void req_close_lock_action(void)
{
    k_timer_stop(&motor_timeout_timer);

    MY_LOG_INF("%s:run", __func__);
    /* A 低，B 高 -> 反转 */
    gpio_pin_set_dt(&motor_a, 0);
    gpio_pin_set_dt(&motor_b, 1);
    k_timer_start(&motor_timeout_timer, K_MSEC(LOCK_TIMEOUT_MAXTIME), K_NO_WAIT);
}

/********************************************************************
**函数名称:  stop_lock_action
**入口参数:  无
**出口参数:  无
**函数功能:  执行停止开/关锁操作
**返 回 值:  无
*********************************************************************/
void stop_lock_action(void)
{
    /* 清空开锁中的状态 */
    s_bOpenLockingState = false;
    k_timer_stop(&motor_timeout_timer);

    MY_LOG_INF("%s:run", __func__);
    /* 关掉方向输出 */
    gpio_pin_set_dt(&motor_a, 0);
    gpio_pin_set_dt(&motor_b, 0);
}

/********************************************************************
**函数名称:  lock_posdet_edge_handler
**入口参数:  state  ---        输入，锁状态（OPEN_LOCK/CLOSE_LOCK）
**出口参数:  无
**函数功能:  锁位置检测中断处理函数，根据状态执行对应操作
**返 回 值:  无
*********************************************************************/
static void lock_posdet_edge_handler(LOCK_STATE state)
{
    if (state == OPEN_LOCK)
    {
        if (!openlock_posdet.debouncing)
        {
            openlock_posdet.debouncing = true;
            k_timer_start(&openlock_posdet.timer, K_MSEC(LOCK_DEBOUNCE_MS), K_NO_WAIT);
        }
    }
    else if (state == CLOSE_LOCK)
    {
        if (!closelock_posdet.debouncing)
        {
            closelock_posdet.debouncing = true;
            k_timer_start(&closelock_posdet.timer, K_MSEC(LOCK_DEBOUNCE_MS), K_NO_WAIT);
        }
    }
}

/********************************************************************
**函数名称:  openlock_posdet_timer_handler
**入口参数:  timer  ---        输入，定时器句柄（未使用）
**出口参数:  无
**函数功能:  开锁消抖定时器回调函数，根据开锁状态执行对应操作
**返 回 值:  无
*********************************************************************/
static void openlock_posdet_timer_handler(struct k_timer *timer)
{
    ARG_UNUSED(timer);

    int level = gpio_pin_get(motor_pos_a.port, motor_pos_a.pin);
    bool new_state = (level == 1);/* level==1 表示物理接地（Active Low 激活） */

    if (new_state != openlock_posdet.state)
    {
        openlock_posdet.state = new_state;
        // MY_LOG_INF("openlock det changed: %s", new_state ? "Openlocked" : "Unknown");
        if (new_state)
        {
            /* 电机停止 */
            my_send_msg(MOD_MAIN, MOD_CTRL, MY_MSG_CTRL_STOPLOCK);
            /* 开锁后闪烁LED 18秒 */
            my_lock_led_msg_send(LOCK_LED_UNLOCK);
            // MY_MSG_CTRL_OPENLOCKED:先发送消息通知MAIN线程，再经过LTE线程上报开锁状态成功
        }
    }

    openlock_posdet.debouncing = false;
}

/********************************************************************
**函数名称:  closelock_posdet_timer_handler
**入口参数:  timer  ---        输入，定时器句柄（未使用）
**出口参数:  无
**函数功能:  关锁消抖定时器回调函数，根据关锁状态执行对应操作
**返 回 值:  无
*********************************************************************/
static void closelock_posdet_timer_handler(struct k_timer *timer)
{
    ARG_UNUSED(timer);

    int level = gpio_pin_get(motor_pos_b.port, motor_pos_b.pin);
    bool new_state = (level == 1);/* level==1 表示物理接地（Active Low 激活） */

    if (new_state != closelock_posdet.state)
    {
        closelock_posdet.state = new_state;
        // MY_LOG_INF("closelock det changed: %s", new_state ? "Closelocked" : "Unknown");
        if (new_state)
        {
            /* 电机停止 */
            my_send_msg(MOD_MAIN, MOD_CTRL, MY_MSG_CTRL_STOPLOCK);
            /* 关锁后关闭LED */
            my_lock_led_msg_send(LOCK_LED_CLOSE);
            // TODO MY_MSG_CTRL_CLOSELOCKED:发送消息给LTE线程上报关锁状态成功
        }
        else
        {
            /* 非法解锁:没执行开锁动作,关锁的限位检测却断开(锁没有完全关掉) */
            if (!s_bOpenLockingState)
            {
                // TODO 非法解锁上报,直接发消息给LTE线程,由4G判断是否要上报

                /* 蜂鸣器报警方式 */
                if (g_device_cmd_config.lockerr_buzzer == 1)
                {
                    // TODO 发消息到ctrl线程,报警30s
                }
                else if (g_device_cmd_config.lockerr_buzzer == 2)
                {
                    // TODO 发消息到ctrl线程,持续报警直到收到关闭蜂鸣器报警指令
                }
            }

            /* 清空开锁中的状态 */
            s_bOpenLockingState = false;
        }
    }

    closelock_posdet.debouncing = false;
}

/********************************************************************
**函数名称:  get_closelock_state
**入口参数:  无
**出口参数:  无
**函数功能:  获取关锁限位状态
**返 回 值:  true  ---        关锁到位（限位检测到）
            false ---        未关锁到位
*********************************************************************/
bool get_closelock_state(void)
{
    return closelock_posdet.state;
}

static void motor_pos_isr(const struct device *dev,
                        struct gpio_callback *cb,
                        uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);

    if (pins & BIT(motor_pos_a.pin))
    {
        /* 下降沿触发，调用开锁处理函数 */
        lock_posdet_edge_handler(OPEN_LOCK);
    }

    if (pins & BIT(motor_pos_b.pin))
    {
        /* 边沿触发，调用关锁处理函数 */
        lock_posdet_edge_handler(CLOSE_LOCK);
    }
}

/********************************************************************
**函数名称:  motor_timer_timeout_handler
**入口参数:  timer  ---        输入，定时器句柄（未使用）
**出口参数:  无
**函数功能:  电机超时定时器回调函数，当开锁/关锁超时时，通知ctrl模块停止电机
**返 回 值:  无
*********************************************************************/
static void motor_timer_timeout_handler(struct k_timer *timer)
{
    ARG_UNUSED(timer);

    // MY_LOG_INF("%s:run", __func__);
    // TODO 发送锁状态失败通知
    my_send_msg(MOD_MAIN, MOD_CTRL, MY_MSG_CTRL_STOPLOCK);
}

int motor_gpio_init(void)
{
    int ret;
    int openlockDetInitialLevel;
    int closelockDetInitialLevel;

    /* 检查所有端口是否就绪 */
    if (!device_is_ready(motor_a.port) ||
        !device_is_ready(motor_b.port) ||
        !device_is_ready(motor_pwr_en.port) ||
        !device_is_ready(motor_pos_a.port) ||
        !device_is_ready(motor_pos_b.port))
    {
        return -ENODEV;
    }

    /* 输出引脚配置：P2.0 / P2.1 / P2.5 */
    ret = gpio_pin_configure_dt(&motor_a, GPIO_OUTPUT_INACTIVE);
    if (ret) return ret;

    ret = gpio_pin_configure_dt(&motor_b, GPIO_OUTPUT_INACTIVE);
    if (ret) return ret;

    /* 电源默认关闭 */
    ret = gpio_pin_configure_dt(&motor_pwr_en, GPIO_OUTPUT_INACTIVE);
    if (ret) return ret;

    /* 输入引脚配置：P0.2 / P0.3，默认上拉，高电平，下降沿触发 */
    ret = gpio_pin_configure_dt(&motor_pos_a, GPIO_INPUT);
    if (ret) return ret;

    ret = gpio_pin_configure_dt(&motor_pos_b, GPIO_INPUT);
    if (ret) return ret;

    /* 配置中断为下降沿触发 */
    ret = gpio_pin_interrupt_configure_dt(&motor_pos_a, GPIO_INT_EDGE_FALLING);
    if (ret) return ret;
    /* 配置中断为边沿触发 */
    ret = gpio_pin_interrupt_configure_dt(&motor_pos_b, GPIO_INT_EDGE_BOTH);
    if (ret) return ret;

    /* 初始化开锁检测定时器和状态 */
    k_timer_init(&openlock_posdet.timer, openlock_posdet_timer_handler, NULL);
    openlock_posdet.state = false;
    openlock_posdet.debouncing = false;
    /* 读取初始状态 */
    openlockDetInitialLevel = gpio_pin_get(motor_pos_a.port, motor_pos_a.pin);
    openlock_posdet.state = (openlockDetInitialLevel == 1);
    MY_LOG_INF("openlock det changed: %s", openlock_posdet.state ? "Openlocked" : "Unknown");

    /* 初始化关锁检测定时器和状态 */
    k_timer_init(&closelock_posdet.timer, closelock_posdet_timer_handler, NULL);
    closelock_posdet.state = false;
    closelock_posdet.debouncing = false;
    /* 读取初始状态 */
    closelockDetInitialLevel = gpio_pin_get(motor_pos_b.port, motor_pos_b.pin);
    closelock_posdet.state = (closelockDetInitialLevel == 1);
    MY_LOG_INF("closelock det changed: %s", closelock_posdet.state ? "Closelocked" : "Unknown");

    /* 注册回调 */
    gpio_init_callback(&motor_pos_cb, motor_pos_isr, BIT(motor_pos_a.pin) | BIT(motor_pos_b.pin));
    gpio_add_callback(motor_pos_a.port, &motor_pos_cb);

    k_timer_init(&motor_timeout_timer, motor_timer_timeout_handler, NULL);

    return 0;
}