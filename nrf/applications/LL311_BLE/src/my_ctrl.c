/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        my_ctrl.c
**文件描述:        系统控制模块实现文件 (LED, Buzzer, Key)
**当前版本:        V1.0
**作    者:        Harrison Wu (wuyujiao@jimiiot.com)
**完成日期:        2026.01.15
*********************************************************************
** 功能描述:        1. 整合 LED 与蜂鸣器控制接口
**                 2. 实现独立线程处理按键扫描与逻辑
*********************************************************************/

#include "my_comm.h"

/* 注册控制模块日志 */
LOG_MODULE_REGISTER(my_ctrl, LOG_LEVEL_INF);

#define WAKEUP_PIN_PSEL  4   /* P0.4 -> PSEL = 4 */

/* 硬件设备树定义 */
static const struct gpio_dt_spec fun_key = GPIO_DT_SPEC_GET(DT_ALIAS(fun_key), gpios);
static const struct pwm_dt_spec buzzer = PWM_DT_SPEC_GET(DT_ALIAS(buzzer_pwm)); 
static const struct gpio_dt_spec light_det = GPIO_DT_SPEC_GET(DT_ALIAS(light_detect), gpios);
static const struct gpio_dt_spec cut_det = GPIO_DT_SPEC_GET(DT_ALIAS(cut_detect), gpios);
static const struct gpio_dt_spec lock_led = GPIO_DT_SPEC_GET(DT_ALIAS(lock_led0), gpios);
static const struct gpio_dt_spec batt_leds[] = {
    GPIO_DT_SPEC_GET(DT_ALIAS(battery_led0), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(battery_led1), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(battery_led2), gpios),
};

static struct gpio_callback misc_io_cb;

/* 消息队列定义 */
K_MSGQ_DEFINE(my_ctrl_msgq, sizeof(MSG_S), 10, 4);

/* 线程数据与栈定义 */
K_THREAD_STACK_DEFINE(my_ctrl_task_stack, MY_CTRL_TASK_STACK_SIZE);
static struct k_thread my_ctrl_task_data;

/* --- 休眠唤醒功能实现 --- */

static void enable_wakeup_pin(void)
{
    /* 1. 配置 P0.4 为输入，并根据外部电路选择上拉/下拉 */
    nrf_gpio_cfg_input(WAKEUP_PIN_PSEL, NRF_GPIO_PIN_PULLUP);

    /* 2. 配置 SENSE 条件，低电平唤醒 */
    nrf_gpio_cfg_sense_set(WAKEUP_PIN_PSEL, NRF_GPIO_PIN_SENSE_LOW);
}

static void go_to_system_off(void)
{
    LOG_INF("Config wakeup pin and enter System OFF");

    /* 清 RESETREAS，避免立即被旧的唤醒原因拉起（手册要求） */
    nrf_reset_resetreas_clear(NRF_RESET, 0xFFFFFFFF);

    enable_wakeup_pin();

    k_sleep(K_SECONDS(2));// 确保上面的日志有打印出来

    /* 进入 System OFF（深度睡眠） */
    sys_poweroff();
}

// 光感、剪线检测功能
static void misc_io_isr(const struct device *dev,
                   struct gpio_callback *cb,
                   uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);

    if (pins & BIT(fun_key.pin))
    {
        int level = gpio_pin_get_dt(&fun_key);
        // LOG_INF("Key interrupt, level=%d", level);
        /* TODO: 在这里处理按键事件 */
    }

    if (pins & BIT(light_det.pin))
    {
        int level = gpio_pin_get_dt(&light_det);
        // LOG_INF("Light detect interrupt, level=%d", level);
        /* TODO: 在这里处理光感变化事件 */
    }

    if (pins & BIT(cut_det.pin))
    {
        int level = gpio_pin_get_dt(&cut_det);
        // LOG_INF("Cut detect interrupt, level=%d", level);
        /* TODO: 在这里处理剪线事件 */
    }
}

/* --- 输入IO中断功能实现 --- */

// 输入按键、光感、剪线检测初始化
static int misc_io_init(void)
{
    int ret;

    if (!device_is_ready(fun_key.port) ||
        !device_is_ready(light_det.port) ||
        !device_is_ready(cut_det.port)) {
        return -ENODEV;
    }

    /* 配置为输入（上拉在 DTS 中已指定） */
    ret = gpio_pin_configure_dt(&fun_key, GPIO_INPUT);
    if (ret) return ret;

    ret = gpio_pin_configure_dt(&light_det, GPIO_INPUT);
    if (ret) return ret;

    ret = gpio_pin_configure_dt(&cut_det, GPIO_INPUT);
    if (ret) return ret;

    /* 配置中断：
     * 默认上拉，高电平；外部拉低时触发，可以用 GPIO_INT_EDGE_FALLING。
     * 如果希望高/低变化都触发，可用 GPIO_INT_EDGE_BOTH。
     */
    ret = gpio_pin_interrupt_configure_dt(&fun_key, GPIO_INT_EDGE_FALLING);
    if (ret) return ret;

    ret = gpio_pin_interrupt_configure_dt(&light_det, GPIO_INT_EDGE_FALLING);
    if (ret) return ret;

    ret = gpio_pin_interrupt_configure_dt(&cut_det, GPIO_INT_EDGE_FALLING);
    if (ret) return ret;

    /* 一个回调处理三个引脚 */
    gpio_init_callback(&misc_io_cb, misc_io_isr,
                       BIT(fun_key.pin) |
                       BIT(light_det.pin) |
                       BIT(cut_det.pin));
    gpio_add_callback(fun_key.port, &misc_io_cb);

    return 0;
}

/* --- 蜂鸣器功能实现 --- */

void my_ctrl_stop_buzzer(void)
{
    pwm_set_pulse_dt(&buzzer, 0);
}

int my_ctrl_buzzer_play_tone(uint32_t freq_hz, uint32_t duration_ms)
{
    if (freq_hz == 0)
    {
        my_ctrl_stop_buzzer();
    }
    else
    {
        uint32_t period = (uint32_t)(1000000000ULL / freq_hz);
        uint32_t pulse = period / 2;

        int err = pwm_set_dt(&buzzer, period, pulse);
        if (err)
        {
            LOG_ERR("Failed to set PWM (err %d)", err);
            return err;
        }
    }

    if (duration_ms > 0)
    {
        k_msleep(duration_ms);
        my_ctrl_stop_buzzer();
    }

    return 0;
}

int my_ctrl_buzzer_play_sequence(const struct my_buzzer_note *notes, uint32_t num_notes)
{
    if (notes == NULL || num_notes == 0)
        return -EINVAL;

    for (uint32_t i = 0; i < num_notes; i++)
    {
        my_ctrl_buzzer_play_tone(notes[i].freq_hz, notes[i].duration_ms);
        k_msleep(10);
    }
    return 0;
}

/* --- LED 功能实现 --- */

static int leds_init(void)
{
    int ret;

    /* 所有电量 LED 共用同一个 port（gpio2），检查第一个即可 */
    if (!device_is_ready(batt_leds[0].port) ||
        !device_is_ready(lock_led.port)) {
        return -ENODEV;
    }

    /* 配置电量指示灯为输出，默认灭 */
    for (size_t i = 0; i < ARRAY_SIZE(batt_leds); i++)
    {
        ret = gpio_pin_configure_dt(&batt_leds[i], GPIO_OUTPUT_INACTIVE);
        if (ret) return ret;
    }

    /* 配置锁状态指示灯为输出，默认灭 */
    ret = gpio_pin_configure_dt(&lock_led, GPIO_OUTPUT_INACTIVE);
    if (ret) return ret;

    return 0;
}

/* level: 0~3
 * 0 -> 全灭
 * 1 -> 只亮 batt_led0
 * 2 -> 亮 batt_led0, batt_led1
 * 3 -> 亮 batt_led0, batt_led1, batt_led2
 */
static int batt_led_set_level(uint8_t level)
{
    int ret;
    int on;

    if (level > 3) {
        level = 3;
    }

    for (size_t i = 0; i < ARRAY_SIZE(batt_leds); i++)
    {
        on = (i < level) ? 1 : 0;
        ret = gpio_pin_set_dt(&batt_leds[i], on);
        if (ret < 0) return ret;
    }

    return 0;
}

/* on = true 点亮锁状态灯，false 熄灭 */
static void lock_led_set(bool on)
{
    LOG_INF("%s:%d", __func__, on);
    gpio_pin_set_dt(&lock_led, on ? 1 : 0);
}

int my_ctrl_push_msg(const MSG_S *msg)
{
    if (msg == NULL)
        return -EINVAL;

    return k_msgq_put(&my_ctrl_msgq, msg, K_NO_WAIT);
}

/********************************************************************
**函数名称:  my_ctrl_task
**入口参数:  无
**出口参数:  无
**函数功能:  控制模块主线程，处理按键扫描与状态机
**返 回 值:  无
*********************************************************************/
static void my_ctrl_task(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    MSG_S msg;

    LOG_INF("Control thread started");

    for (;;)
    {
        my_recv_msg(&my_ctrl_msgq, (void *)&msg, sizeof(MSG_S), K_FOREVER);

        switch (msg.msgID)
        {
                /* TODO: Add message handling cases */

            default:
                break;
        }
    }

#if 0
        /* 1. 处理来自外部的消息队列任务 */
        if (k_msgq_get(&my_ctrl_msgq, &msg, K_NO_WAIT) == 0) {
            switch (msg.type) {
                case MY_CTRL_MSG_TYPE_LED:
                    dk_set_led(msg.data.led.led_idx, msg.data.led.val);
                    break;
                case MY_CTRL_MSG_TYPE_BUZZER:
                    my_ctrl_buzzer_play_tone(msg.data.buzzer.freq_hz, msg.data.buzzer.duration_ms);
                    break;
                default:
                    LOG_WRN("Unknown ctrl msg type: %d", msg.type);
                    break;
            }
        }

        /* 2. 按键扫描 */
        int val = gpio_pin_get_dt(&fun_key);
        if (val != last_key_val) {
            if (val == 1) {
                LOG_INF("FUN_KEY Pressed");
                my_ctrl_buzzer_play_tone(2000, 50);
            } else {
                LOG_INF("FUN_KEY Released");
            }
            last_key_val = val;
        }

        /* 3. 运行指示灯闪烁 */
        uint32_t now = k_uptime_get_32();
        if (now - last_blink_time >= 1000) {
            dk_set_led(DK_LED1, (++blink_status) % 2);
            last_blink_time = now;
        }

        k_msleep(20);
    }
#endif
}

/********************************************************************
**函数名称:  my_ctrl_init
**入口参数:  tid      ---        指向线程 ID 变量的指针
**出口参数:  tid      ---        存储启动后的线程 ID
**函数功能:  初始化控制模块 (GPIO, PWM) 并启动控制线程
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
int my_ctrl_init(k_tid_t *tid)
{
    int err;

    /* 1. 初始化按键、光感、剪线、LED GPIO、motor、batt */
    misc_io_init();
    leds_init();
    motor_gpio_init();
    batt_gpio_init();
    batt_adc_init();

    /* 2. 初始化蜂鸣器 PWM */
    if (!pwm_is_ready_dt(&buzzer))
    {
        LOG_ERR("Buzzer PWM not ready");
        return -ENODEV;
    }

    /* 3. 初始化消息队列 */
    my_init_msg_handler(MOD_CTRL, &my_ctrl_msgq);

    /* 4. 启动控制线程 */
    *tid = k_thread_create(&my_ctrl_task_data, my_ctrl_task_stack,
                           K_THREAD_STACK_SIZEOF(my_ctrl_task_stack),
                           my_ctrl_task, NULL, NULL, NULL,
                           MY_CTRL_TASK_PRIORITY, 0, K_NO_WAIT);

    /* 5. 设置线程名称 */
    k_thread_name_set(*tid, "MY_CTRL");

    LOG_INF("Control module initialized");
    return 0;
}