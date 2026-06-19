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

#define LOCK_TIMEOUT_MAXTIME 5000   // 实测开关锁时间大概在3.8秒，这里设置开/关锁超时时间为5秒
#define LOCK_DEBOUNCE_MS    100    /* 消抖时间：100ms */

#define LOCK_INTERNAL_PULLUP 1      // 是否启用开关锁内部上拉电阻,现阶段硬件上是没有外部上拉的，所以现在软件上采用内部上拉，如果硬件上加了外部上拉，这个宏就需要关闭
typedef enum
{
    OPEN_LOCK,
    CLOSE_LOCK,
} lock_status_t;

typedef struct
{
    struct k_timer timer;       /* 消抖定时器 */
    bool state;                 /* 当前开/关锁状态（true=到达限位，false=未到限位） */
    bool debouncing;            /* 消抖中标志 */
} lock_posdet_t;

static struct gpio_callback s_motor_pos_cb;
/* 电机超时定时器，用于开锁/关锁操作的超时保护 */
static struct k_timer s_motor_timeout_timer;
/* 开锁位置检测结构，用于检测开锁限位状态 */
static lock_posdet_t s_openlock_posdet;
/* 关锁位置检测结构，用于检测关锁限位状态 */
static lock_posdet_t s_closelock_posdet;

// 蓝牙操作锁状态：默认停止锁操作
lock_state_t g_bBleLockState = LOCK_STOP;
// 网络操作锁状态：默认停止锁操作
lock_state_t g_netLockState = LOCK_STOP;

const char unlock_success[] = "Unlock success";
const char unlock_fail[] = "Unlock failed. No unlock state detected.";
const char lock_success[] = "Lock success";
const char lock_fail[] = "Lock failed. No lock state detected.";

/********************************************************************
**函数名称:  get_lock_state
**入口参数:  无
**出口参数:  无
**函数功能:  获取当前锁状态
**返 回 值:  true=已锁，false=未锁
*********************************************************************/
bool get_lock_state(void)
{
    return s_closelock_posdet.state;
}

/* 电源控制：1 = 打开，0 = 关闭 */
void motor_power_set(bool on)
{
    LOG_INF("%s:%d", __func__, on);
    gpio_pin_set_dt(&motor_pwr_en, on ? 1 : 0);
}

#if LOCK_INTERNAL_PULLUP
/********************************************************************
**函数名称:  motor_det_irq_disable
**入口参数:  spec   ---        GPIO设备结构体指针
            port   ---        GPIO端口号
**出口参数:  无
**函数功能:  断开电机检测引脚，关闭中断、输入缓冲及上拉电阻以降低功耗
**返 回 值:  无
*********************************************************************/
void motor_det_irq_disable(const struct gpio_dt_spec *spec, uint8_t port)
{
    // 先关闭中断
    gpio_pin_interrupt_configure_dt(spec, GPIO_INT_DISABLE);

    // 断开输入缓冲 + 关闭上拉
    nrf_gpio_cfg(
        NRF_GPIO_PIN_MAP(port, spec->pin),
        NRF_GPIO_PIN_DIR_INPUT,
        NRF_GPIO_PIN_INPUT_DISCONNECT,  // 断开输入缓冲
        NRF_GPIO_PIN_NOPULL,            // 关闭上拉，防止漏电
        NRF_GPIO_PIN_S0S1,
        NRF_GPIO_PIN_NOSENSE
    );
}

/********************************************************************
**函数名称:  motor_det_irq_enable
**入口参数:  无
**出口参数:  无
**函数功能:  使能电机位置检测引脚中断，配置为输入上拉及双边沿触发
**返 回 值:  无
*********************************************************************/
static void motor_det_irq_enable(void)
{
    gpio_pin_configure_dt(&motor_pos_a, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_interrupt_configure_dt(&motor_pos_a, GPIO_INT_EDGE_BOTH);

    gpio_pin_configure_dt(&motor_pos_b, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_interrupt_configure_dt(&motor_pos_b, GPIO_INT_EDGE_BOTH);
}
#endif

/********************************************************************
**函数名称:  req_open_lock_action
**入口参数:  无
**出口参数:  无
**函数功能:  请求执行开锁操作
**返 回 值:  无
*********************************************************************/
void req_open_lock_action(void)
{
#if LOCK_INTERNAL_PULLUP
    motor_det_irq_enable();
#endif

    //开启电机电源
    motor_power_set(true);
    k_timer_stop(&s_motor_timeout_timer);

    MY_LOG_INF("%s:run", __func__);
    /* A 低，B 高 -> 正转（具体取决于你的驱动电路） */
    gpio_pin_set_dt(&motor_a, 0);
    gpio_pin_set_dt(&motor_b, 1);
    k_timer_start(&s_motor_timeout_timer, K_MSEC(LOCK_TIMEOUT_MAXTIME), K_NO_WAIT);
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
#if LOCK_INTERNAL_PULLUP
    motor_det_irq_enable();
#endif

    //开启电机电源
    motor_power_set(true);
    k_timer_stop(&s_motor_timeout_timer);

    MY_LOG_INF("%s:run", __func__);
    /* A 高，B 低 -> 反转 */
    gpio_pin_set_dt(&motor_a, 1);
    gpio_pin_set_dt(&motor_b, 0);
    k_timer_start(&s_motor_timeout_timer, K_MSEC(LOCK_TIMEOUT_MAXTIME), K_NO_WAIT);
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
    k_timer_stop(&s_motor_timeout_timer);

    MY_LOG_INF("%s:run", __func__);
    /* 关掉方向输出 */
    gpio_pin_set_dt(&motor_a, 0);
    gpio_pin_set_dt(&motor_b, 0);

    //关闭电机电源
    motor_power_set(false);
}

void send_lock_result_event(const char *lock_msg_data)
{
    msg_t msg;
    msg.msgID = MY_MSG_BLE_LOCK_RESULT;
    msg.pData = lock_msg_data;
    msg.DataLen = strlen(lock_msg_data);
    my_send_msg_data(MOD_MAIN, MOD_BLE, &msg);
}

/********************************************************************
**函数名称:  respond_netlock_result
**入口参数:  lock_msg_data   ---   需要发送的锁状态结果字符串
**出口参数:  无
**函数功能:  响应网络上锁结果：将结果数据通过消息队列发送给蓝牙模块去做匹配处理
**返 回 值:  无
*********************************************************************/
void respond_netlock_result(char *cmd_name, const char *lock_msg_data)
{
    char *p;
    msg_t msg;
    char buf[80];

    // 拼接消息（指令头,回复内容）
    snprintf(buf, sizeof(buf), "%s,%s", cmd_name, lock_msg_data);
    buf[sizeof(buf) - 1] = '\0'; // 确保字符串终止

    MY_MALLOC_BUFFER(p, strlen(buf) + 1);
    if (p == NULL)
    {
        MY_LOG_ERR("p malloc failed");
        return;
    }

    // 拼接回复消息
    strcpy(p, buf);

    // 发消息到蓝牙线程处理异步回复
    msg.msgID = MY_MSG_LTE_CMD_ASYNC_RESP;
    msg.pData = p;
    my_send_msg_data(MOD_LTE, MOD_BLE, &msg);
}

/********************************************************************
**函数名称:  lock_posdet_edge_handler
**入口参数:  state  ---        输入，锁状态（OPEN_LOCK/CLOSE_LOCK）
**出口参数:  无
**函数功能:  锁位置检测中断处理函数，根据状态执行对应操作
**返 回 值:  无
*********************************************************************/
static void lock_posdet_edge_handler(lock_status_t state)
{
    if (state == OPEN_LOCK)
    {
        if (!s_openlock_posdet.debouncing)
        {
            s_openlock_posdet.debouncing = true;
            k_timer_start(&s_openlock_posdet.timer, K_MSEC(LOCK_DEBOUNCE_MS), K_NO_WAIT);
        }
    }
    else if (state == CLOSE_LOCK)
    {
        if (!s_closelock_posdet.debouncing)
        {
            s_closelock_posdet.debouncing = true;
            k_timer_start(&s_closelock_posdet.timer, K_MSEC(LOCK_DEBOUNCE_MS), K_NO_WAIT);
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

    if (new_state != s_openlock_posdet.state)
    {
        s_openlock_posdet.state = new_state;
        // MY_LOG_INF("openlock det changed: %s", new_state ? "Openlocked" : "Unknown");
        if (new_state)
        {
            /* 电机停止 */
            my_send_msg(MOD_MAIN, MOD_CTRL, MY_MSG_CTRL_STOPLOCK);
            /* 开锁后闪烁LED 18秒 */
            my_lock_led_msg_send(LOCK_LED_UNLOCK);
            // 上报开锁成功告警
            send_alarm_message_to_lte(ALARM_UNLOCK, NULL);

            /* 如果是蓝牙开锁触发，通知蓝牙线程开锁成功 */
            if (g_bBleLockState == UNLOCKING)
            {
                g_bBleLockState = LOCK_STOP;

                send_lock_result_event(unlock_success);
            }
            // 如果是网络开锁触发，需走异步回复
            if (g_netLockState == UNLOCKING)
            {
                g_netLockState = LOCK_STOP;

                // 定时器触发开锁，不需要回复
                if (g_net_unlock.start_timer_flag)
                {
                    g_net_unlock.start_timer_flag = 0;
                }
                else
                {
                    respond_netlock_result("CUNLOCK",unlock_success);
                }

                g_net_unlock.netunlock_flag = 1;
                k_timer_start(&g_net_unlock.delay_timer, K_MSEC(g_net_unlock.delay_sec * 1000), K_NO_WAIT);
            }
            // MY_MSG_CTRL_OPENLOCKED:先发送消息通知MAIN线程，再经过LTE线程上报开锁状态成功

            //解锁成功提示音
            my_set_buzzer_mode(BUZZER_UNLOCK_SUCCESS);
#if LOCK_INTERNAL_PULLUP
            motor_det_irq_disable(&motor_pos_a, 0);
#endif
        }
    }

    s_openlock_posdet.debouncing = false;
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

    if (new_state != s_closelock_posdet.state)
    {
        s_closelock_posdet.state = new_state;
        // MY_LOG_INF("closelock det changed: %s", new_state ? "Closelocked" : "Unknown");
        if (new_state)
        {
            /* 电机停止 */
            my_send_msg(MOD_MAIN, MOD_CTRL, MY_MSG_CTRL_STOPLOCK);
            /* 关锁后关闭LED */
            my_lock_led_msg_send(LOCK_LED_CLOSE);
            // 上报关锁成功告警
            send_alarm_message_to_lte(ALARM_LOCK, NULL);

            /* 如果是蓝牙关锁触发，通知蓝牙线程关锁成功 */
            if (g_bBleLockState == LOCKING)
            {
                g_bBleLockState = LOCK_STOP;
                send_lock_result_event(lock_success);
            }
            // 如果是网络关锁触发，需走异步回复
            if (g_netLockState == LOCKING)
            {
                g_netLockState = LOCK_STOP;
                respond_netlock_result("CLOCK",lock_success);
            }
            // TODO MY_MSG_CTRL_CLOSELOCKED:发送消息给LTE线程上报关锁状态成功

            // 上锁成功，直接取消窗口期倒计时
            k_timer_stop(&g_net_unlock.delay_timer);
            // 允许自动上锁
            g_net_unlock.netunlock_flag = 0;

            //上锁成功提示
            my_set_buzzer_mode(BUZZER_EVENT_LOCK_SUCCESS);
#if LOCK_INTERNAL_PULLUP
            motor_det_irq_disable(&motor_pos_b, 0);
#endif
        }
    }

    s_closelock_posdet.debouncing = false;
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
    return s_closelock_posdet.state;
}

/********************************************************************
**函数名称:  get_openlock_state
**入口参数:  无
**出口参数:  无
**函数功能:  获取开锁限位状态
**返 回 值:  true  ---        开锁到位（限位检测到）
            false ---        未开锁到位
*********************************************************************/
bool get_openlock_state(void)
{
    return s_openlock_posdet.state;
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
    /* 如果是蓝牙开锁触发，通知蓝牙线程开锁失败 */
    if (g_bBleLockState == UNLOCKING)
    {
        g_bBleLockState = LOCK_STOP;

        send_lock_result_event(unlock_fail);
    }
    /* 如果是蓝牙关锁触发，通知蓝牙线程关锁失败 */
    else if (g_bBleLockState == LOCKING)
    {
        g_bBleLockState = LOCK_STOP;

        send_lock_result_event(lock_fail);
    }

    if (g_netLockState == UNLOCKING)
    {
        g_netLockState = LOCK_STOP;
        // 定时器触发的开锁不用回复
        if (g_net_unlock.start_timer_flag)
        {
            g_net_unlock.start_timer_flag = 0;
        }
        else
        {
            respond_netlock_result("CUNLOCK",unlock_fail);
        }
    }
    // 如果是网络关锁触发，走异步回复
    else if (g_netLockState == LOCKING)
    {
        g_netLockState = LOCK_STOP;
        respond_netlock_result("CLOCK",lock_fail);
    }

    /* 发送停止锁操作消息 */
    my_send_msg(MOD_MAIN, MOD_CTRL, MY_MSG_CTRL_STOPLOCK);
    //上锁/解锁失败提示
    my_set_buzzer_mode(BUZZER_EVENT_LOCK_FAIL);
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

    //电源默认关闭(目前这个IO控制只用在电机，NFC和蜂鸣器直接电池供电)
    ret = gpio_pin_configure_dt(&motor_pwr_en, GPIO_OUTPUT_INACTIVE);
    if (ret) return ret;

#if LOCK_INTERNAL_PULLUP
    motor_det_irq_enable();
#else
    /* 输入引脚配置：P0.2 / P0.3，默认上拉，高电平，下降沿触发 */
    ret = gpio_pin_configure_dt(&motor_pos_a, GPIO_INPUT);
    if (ret) return ret;

    ret = gpio_pin_configure_dt(&motor_pos_b, GPIO_INPUT);
    if (ret) return ret;

    // 配置中断为边沿触发
    ret = gpio_pin_interrupt_configure_dt(&motor_pos_a, GPIO_INT_EDGE_BOTH);
    if (ret) return ret;
    /* 配置中断为边沿触发 */
    ret = gpio_pin_interrupt_configure_dt(&motor_pos_b, GPIO_INT_EDGE_BOTH);
    if (ret) return ret;
#endif
    /* 初始化开锁检测定时器和状态 */
    k_timer_init(&s_openlock_posdet.timer, openlock_posdet_timer_handler, NULL);
    s_openlock_posdet.state = false;
    s_openlock_posdet.debouncing = false;
    /* 读取初始状态 */
    openlockDetInitialLevel = gpio_pin_get(motor_pos_a.port, motor_pos_a.pin);
    s_openlock_posdet.state = (openlockDetInitialLevel == 1);
    MY_LOG_INF("openlock det changed: %s", s_openlock_posdet.state ? "Openlocked" : "Unknown");

    /* 初始化关锁检测定时器和状态 */
    k_timer_init(&s_closelock_posdet.timer, closelock_posdet_timer_handler, NULL);
    s_closelock_posdet.state = false;
    s_closelock_posdet.debouncing = false;
    /* 读取初始状态 */
    closelockDetInitialLevel = gpio_pin_get(motor_pos_b.port, motor_pos_b.pin);
    s_closelock_posdet.state = (closelockDetInitialLevel == 1);
    MY_LOG_INF("closelock det changed: %s", s_closelock_posdet.state ? "Closelocked" : "Unknown");

#if LOCK_INTERNAL_PULLUP
    // 初始化时断开两个引脚，不耗电
    motor_det_irq_disable(&motor_pos_a, 0);
    motor_det_irq_disable(&motor_pos_b, 0);
#endif
    /* 注册回调 */
    gpio_init_callback(&s_motor_pos_cb, motor_pos_isr, BIT(motor_pos_a.pin) | BIT(motor_pos_b.pin));
    gpio_add_callback(motor_pos_a.port, &s_motor_pos_cb);

    k_timer_init(&s_motor_timeout_timer, motor_timer_timeout_handler, NULL);

    // 默认高阻状态
    stop_lock_action();
    return 0;
}