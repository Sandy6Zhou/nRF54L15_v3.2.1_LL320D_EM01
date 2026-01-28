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

#include <dk_buttons_and_leds.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "my_ctrl.h"
#include "my_main.h"

/* 注册控制模块日志 */
LOG_MODULE_REGISTER(my_ctrl, LOG_LEVEL_INF);

/* 硬件设备树定义 */
static const struct gpio_dt_spec fun_key = GPIO_DT_SPEC_GET(DT_ALIAS(fun_key), gpios);
static const struct pwm_dt_spec buzzer = PWM_DT_SPEC_GET(DT_ALIAS(buzzer_pwm));

/* 消息队列定义 */
K_MSGQ_DEFINE(my_ctrl_msgq, sizeof(struct my_ctrl_msg), 10, 4);

/* 线程数据与栈定义 */
K_THREAD_STACK_DEFINE(my_ctrl_task_stack, MY_CTRL_TASK_STACK_SIZE);
static struct k_thread my_ctrl_task_data;

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

void my_ctrl_led_set(uint32_t led_idx, uint32_t val)
{
    dk_set_led(led_idx, val);
}

int my_ctrl_push_msg(const struct my_ctrl_msg *msg)
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

    /* 1. 初始化按键 GPIO */
    if (!gpio_is_ready_dt(&fun_key))
    {
        LOG_ERR("FUN_KEY GPIO not ready");
        return -ENODEV;
    }
    err = gpio_pin_configure_dt(&fun_key, GPIO_INPUT);
    if (err)
    {
        LOG_ERR("Failed to configure FUN_KEY (err %d)", err);
        return err;
    }

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