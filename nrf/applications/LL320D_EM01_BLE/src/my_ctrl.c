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
**                 3. 实现 FUN_KEY 按键短按/长按检测（下降沿中断+50ms轮询），实现按键事件发送到主任务
**                 4. 实现光感(light sensor)检测中断处理,消抖处理，产生有光/无光事件并发送到主任务
*********************************************************************/

/* 必须在包含 my_comm.h 之前定义 BLE_LOG_MODULE_ID，避免与 my_ble_log.h 中的默认定义冲突 */
#define BLE_LOG_MODULE_ID BLE_LOG_MOD_CTRL

#include "my_comm.h"

/* 注册控制模块日志 */
LOG_MODULE_REGISTER(my_ctrl, LOG_LEVEL_INF);

/* 循环定时器周期：50ms */
#define KEY_POLL_PERIOD_MS   50
/* 长按阈值：1.5秒 = 30个周期 */
#define KEY_LONG_PRESS_COUNT (1500 / KEY_POLL_PERIOD_MS)
/* 光感消抖时间：100ms */
#define LIGHT_DEBOUNCE_MS 100

/* 硬件设备树定义 */
static const struct gpio_dt_spec fun_key = GPIO_DT_SPEC_GET(DT_ALIAS(fun_key), gpios);
static const struct pwm_dt_spec buzzer = PWM_DT_SPEC_GET(DT_ALIAS(buzzer_pwm));
static const struct gpio_dt_spec light_det = GPIO_DT_SPEC_GET(DT_ALIAS(light_detect), gpios);
static const struct gpio_dt_spec batt_leds[] = {
    GPIO_DT_SPEC_GET(DT_ALIAS(battery_led0), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(battery_led1), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(battery_led2), gpios),
};

static int ctrl_pwm_pm_init(void);
// PWM 电源管理操作回调结构体
static const pm_device_ops_t s_ctrl_pwm_pm_ops =
{
    .init = ctrl_pwm_pm_init,
    .suspend = NULL,
    .resume = NULL,
};

/* 按键控制结构 */
static struct
{
    struct k_timer timer; /* 50ms 轮询定时器 */
    uint32_t press_count; /* 按下计数器 (50ms单位) */
    bool pressed;         /* 按键是否按下 */
} key_ctrl_t;

/* 光感检测控制结构 */
static struct
{
    struct k_timer timer;       /* 消抖定时器 */
    bool state;                 /* 当前光感状态（true=有光，false=无光） */
    bool debouncing;            /* 消抖中标志 */
} light_sensor_t;

static buzzer_ctrl_t s_buzzer_ctrl = { 0 };

/* 定时器回调前向声明 */
static void key_timer_handler(struct k_timer *timer);
static void light_sensor_timer_handler(struct k_timer *timer);

/* 消息队列定义 */
K_MSGQ_DEFINE(my_ctrl_msgq, sizeof(msg_t), 10, 4);

/* 线程数据与栈定义 */
K_THREAD_STACK_DEFINE(my_ctrl_task_stack, MY_CTRL_TASK_STACK_SIZE);
static struct k_thread s_my_ctrl_task_data;
static struct gpio_callback s_misc_io_cb;

/* 蜂鸣器100ms定时器 */
static struct k_timer s_buzzer_timer;

/************************************************************
**函数名称:  get_light_state
**入口参数:  无
**出口参数:  无
**函数功能:  获取当前光感状态
**返回值:    光感状态（true=有光，false=无光）
*********************************************************************/
bool get_light_state(void)
{
    return light_sensor_t.state;
}

/********************************************************************
**函数名称:  send_alarm_message_to_lte
**入口参数:  alarm_type    ---    告警类型枚举(输入)
**          additional_info   ---    附加信息字符串指针(输入，可为NULL)
**出口参数:  无
**函数功能:  发送告警消息到LTE模块
**返回值:    无
*********************************************************************/
void send_alarm_message_to_lte(alarm_type_t alarm_type, const char *additional_info)
{
    char alarm_msg[64] = {0};
    uint8_t rpt = 0;

    // 每次发送告警消息前设置开机原因
    set_lte_boot_reason(LTE_BOOT_REASON_ALARM);

    // 根据告警类型映射字符串，设置上报方式和告警类型字符串
    switch(alarm_type)
    {
        case ALARM_OPEN:
            rpt = gConfigParam.remalm_config.remalm_mode;
            break;

        case ALARM_STILL:
        case ALARM_SEA:
        case ALARM_LAND:
            rpt = gConfigParam.motdet_config.motdet_report_type;
            break;

        case ALARM_BATT:
            switch(atoi(additional_info))
            {
                case BATT_EMPTY:
                    rpt = gConfigParam.batlevel_config.batlevel_empty_rpt;
                    break;

                case BATT_LOW:
                    rpt = gConfigParam.batlevel_config.batlevel_low_rpt;
                    break;

                case BATT_NORMAL:
                    rpt = gConfigParam.batlevel_config.batlevel_normal_rpt;
                    break;

                case BATT_FAIR:
                    rpt = gConfigParam.batlevel_config.batlevel_fair_rpt;
                    break;

                case BATT_HIGH:
                    rpt = gConfigParam.batlevel_config.batlevel_high_rpt;
                    break;

                case BATT_FULL:
                    rpt = gConfigParam.batlevel_config.batlevel_full_rpt;
                    break;

                default:
                    LOG_ERR("unknown BATT level");
                    break;
            }
            break;

        case ALARM_CHARGE_OUT:
        case ALARM_CHARGE_IN:
            rpt = gConfigParam.batlevel_config.chargesta_report;
            break;

        case ALARM_CHARGE_FULL:
            rpt = gConfigParam.batlevel_config.chargesta_report;
            break;

        case ALARM_IMPACT:
            rpt = gConfigParam.shockalarm_config.shockalarm_type;
            break;

        default:
            MY_LOG_ERR("unknown alarm type");
            return;
    }

    // 检查是否需要上报方式
    if(rpt > REPORT_MODE_NONE)
    {
        // 构建告警消息字符串
        if (additional_info != NULL && strlen(additional_info) > 0)
        {
            // 包含附加信息的格式："<告警类型>,<时间戳>,<上报方式>,<附加信息>"
            snprintf(alarm_msg, sizeof(alarm_msg), "%d,%lld,%d,%s", alarm_type, my_get_system_time_sec(), rpt, additional_info);
        }
        else
        {
            // 不包含附加信息的格式："<告警类型>,<时间戳>,<上报方式>"
            snprintf(alarm_msg, sizeof(alarm_msg), "%d,%lld,%d", alarm_type, my_get_system_time_sec(), rpt);
        }

        // 发送告警消息到LTE模块
        #if RETRANSMIT_CHECK_ENABLED
            lte_send_cmd_with_retry("ALARM", alarm_msg);
        #else
            lte_send_command("ALARM", alarm_msg);
        #endif


        // 告警唤醒4G时,根据配置的扫描模式决定是否上报扫描数据
        my_scan_upload_on_lte_wakeup();
    }
}

/* --- 休眠唤醒功能实现 --- */

/********************************************************************
**函数名称:  enable_wakeup_pin
**入口参数:  无
**出口参数:  无
**函数功能:  配置唤醒引脚
**返 回 值:  无
**功能描述:  1. 配置 P0.4 为输入，启用内部上拉
**           2. 配置 SENSE 条件为低电平唤醒
*********************************************************************/
static void enable_wakeup_pin(void)
{
    /*  配置 P1.9 为输入，并根据外部电路选择上拉/下拉,配置 SENSE 条件，高电平唤醒*/
    nrf_gpio_cfg_sense_input(NRF_GPIO_PIN_MAP(1, 9), NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_SENSE_HIGH);

}

/********************************************************************
**函数名称:  go_to_system_off
**入口参数:  无
**出口参数:  无
**函数功能:  进入系统深度休眠
**返 回 值:  无
**功能描述:  1. 清除 RESETREAS 避免立即唤醒
**           2. 配置唤醒引脚
**           3. 延迟 2 秒确保日志输出
**           4. 进入 System OFF 模式
*********************************************************************/
void go_to_system_off(void)
{
    MY_LOG_INF("Config wakeup pin and enter System OFF");

    /* 清 RESETREAS，避免立即被旧的唤醒原因拉起（手册要求） */
    nrf_reset_resetreas_clear(NRF_RESET, 0xFFFFFFFF);

    enable_wakeup_pin();
    k_sleep(K_SECONDS(2));// 确保上面的日志有打印出来

    /* 进入 System OFF（深度睡眠） */
    sys_poweroff();
}

/********************************************************************
**函数名称:  key_timer_init
**入口参数:  无
**出口参数:  无
**函数功能:  初始化按键检测定时器
**返 回 值:  无
**功能描述:  1. 初始化50ms轮询定时器
**           2. 复位按键状态和计数器
*********************************************************************/
static void key_timer_init(void)
{
    k_timer_init(&key_ctrl_t.timer, key_timer_handler, NULL);
    key_ctrl_t.pressed = false;
    key_ctrl_t.press_count = 0;
}

/********************************************************************
**函数名称:  send_key_event
**入口参数:  msg_id   ---   消息ID (短按/长按)
**出口参数:  无
**函数功能:  发送按键事件到主任务
**返 回 值:  无
**功能描述:  封装按键事件消息并通过my_send_msg_data发送到MAIN模块
*********************************************************************/
static void send_key_event(uint32_t msg_id)
{
    msg_t msg;
    msg.msgID = msg_id;
    msg.pData = NULL;
    msg.DataLen = 0;
    my_send_msg_data(MOD_CTRL, MOD_MAIN, &msg);
}

/********************************************************************
**函数名称:  key_timer_handler
**入口参数:  timer    ---   定时器指针
**出口参数:  无
**函数功能:  50ms轮询定时器回调，检测按键状态
**返 回 值:  无
**功能描述:  1. 每50ms读取按键电平
**           2. 按键按下时计数器累加，达到24次(1.2s)触发长按事件
**           3. 按键释放时停止定时器，根据计数判断短按(>=100ms)并发送事件
*********************************************************************/
static void key_timer_handler(struct k_timer *timer)
{
    ARG_UNUSED(timer);

    int level = gpio_pin_get(fun_key.port, fun_key.pin);

    if (level == 1)
    {
        /* 按键持续按下 */
        if (!key_ctrl_t.pressed)
        {
            key_ctrl_t.pressed = true;
            key_ctrl_t.press_count = 0;
        }

        /* 计数器增加 */
        key_ctrl_t.press_count++;

        /* 检查是否达到长按阈值(1.2s) */
        if (key_ctrl_t.press_count == KEY_LONG_PRESS_COUNT)
        {
            send_key_event(MY_MSG_CTRL_KEY_LONG_PRESS);
        }
    }
    else
    {
        /* 按键已释放 */
        if (key_ctrl_t.pressed)
        {
            key_ctrl_t.pressed = false;
            k_timer_stop(&key_ctrl_t.timer);

            /* 短按判断：大于等于100ms且小于1.2s */
            if (key_ctrl_t.press_count < KEY_LONG_PRESS_COUNT && key_ctrl_t.press_count >= 2)
            {
                send_key_event(MY_MSG_CTRL_KEY_SHORT_PRESS);
            }
            key_ctrl_t.press_count = 0;
        }
    }
}

/********************************************************************
**函数名称:  light_sensor_timer_handler
**入口参数:  timer    ---   定时器指针
**出口参数:  无
**函数功能:  光感消抖定时器回调
**返 回 值:  无
**功能描述:  1. 100ms后读取光感电平确认状态
**           2. 状态变化时发送消息到主任务
**           3. 清除消抖标志
*********************************************************************/
static void light_sensor_timer_handler(struct k_timer *timer)
{
    ARG_UNUSED(timer);

    int level = gpio_pin_get(light_det.port, light_det.pin);
    bool new_state = (level == 1);

    if (new_state != light_sensor_t.state)
    {
        light_sensor_t.state = new_state;
        // MY_LOG_INF("Light state changed: %s", new_state ? "LIGHT" : "DARK");

        /* 发送光感状态变化消息到主任务 */
        msg_t msg;
        msg.msgID = new_state ? MY_MSG_CTRL_LIGHT_SENSOR_BRIGHT : MY_MSG_CTRL_LIGHT_SENSOR_DARK;
        msg.pData = NULL;
        msg.DataLen = 0;
        my_send_msg_data(MOD_CTRL, MOD_CTRL, &msg);
    }

    light_sensor_t.debouncing = false;
}

/********************************************************************
**函数名称:  light_sensor_edge_handler
**入口参数:  无
**出口参数:  无
**函数功能:  光感双边沿中断处理
**返 回 值:  无
**功能描述:  1. 检测光感状态变化（有光/无光）
**           2. 启动100ms消抖定时器
**           3. 避免重复触发消抖
*********************************************************************/
static void light_sensor_edge_handler(void)
{
    if (!light_sensor_t.debouncing)
    {
        light_sensor_t.debouncing = true;
        k_timer_start(&light_sensor_t.timer, K_MSEC(LIGHT_DEBOUNCE_MS), K_NO_WAIT);
    }
}

/********************************************************************
**函数名称:  misc_io_isr
**入口参数:  dev      ---   GPIO 设备指针
**           cb       ---   回调结构体指针
**           pins     ---   触发中断的引脚位图
**出口参数:  无
**函数功能:  杂项 IO 中断服务程序
**返 回 值:  无
**功能描述:  1. 处理 FUN_KEY 按键中断
**           2. 处理光感检测中断
*********************************************************************/
static void misc_io_isr(const struct device *dev,
                   struct gpio_callback *cb,
                   uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);

    if (pins & BIT(fun_key.pin))
    {
        if (!k_timer_remaining_get(&key_ctrl_t.timer))
        {
            //定时器不在运行就启动定时器
            k_timer_start(&key_ctrl_t.timer, K_MSEC(KEY_POLL_PERIOD_MS), K_MSEC(KEY_POLL_PERIOD_MS));
        }
    }

    if (pins & BIT(light_det.pin))
    {
        /* 双边沿触发，调用光感处理函数 */
        light_sensor_edge_handler();
    }
}

/********************************************************************
**函数名称:  misc_io_init
**入口参数:  无
**出口参数:  无
**函数功能:  初始化按键、光感检测 IO
**返 回 值:  0 表示成功，负值表示失败
**功能描述:  1. 检查 GPIO 设备就绪状态
**           2. 配置 fun_key 为输入（内部下拉）
**           3. 配置 light_det 为输入
**           4. 配置引脚中断触发
**           5. 初始化按键定时器并注册中断回调
*********************************************************************/
static int misc_io_init(void)
{
    int ret;
    int light_initial_level;

    if (!device_is_ready(fun_key.port) ||
        !device_is_ready(light_det.port))
    {
        return -ENODEV;
    }

    /* 配置为输入（fun_key 配置内部下拉） */
    ret = gpio_pin_configure(fun_key.port, fun_key.pin, GPIO_INPUT | GPIO_PULL_DOWN);
    if (ret)
    {
        MY_LOG_ERR("Failed to configure fun_key: %d", ret);
        return ret;
    }

    /* 配置为输入（light_det 配置为输入） */
    ret = gpio_pin_configure_dt(&light_det, GPIO_INPUT);
    if (ret)
    {
        MY_LOG_ERR("Failed to configure light_det: %d", ret);
        return ret;
    }

    /* 配置按键中断：上升沿触发 */
    ret = gpio_pin_interrupt_configure_dt(&fun_key, GPIO_INT_EDGE_RISING);
    if (ret)
    {
        MY_LOG_ERR("Failed to configure fun_key interrupt: %d", ret);
        return ret;
    }

    /* 光感检测配置：配置光感中断：双边沿触发 */
    ret = gpio_pin_interrupt_configure_dt(&light_det, GPIO_INT_EDGE_BOTH);
    if (ret)
    {
        MY_LOG_ERR("Failed to configure light_det interrupt: %d", ret);
        return ret;
    }

    /* 初始化光感定时器和状态 */
    k_timer_init(&light_sensor_t.timer, light_sensor_timer_handler, NULL);
    light_sensor_t.state = false;
    light_sensor_t.debouncing = false;
    /* 读取初始状态 */
    light_initial_level = gpio_pin_get(light_det.port, light_det.pin);
    light_sensor_t.state = (light_initial_level == 1);
    MY_LOG_INF("Light initial state: %s", light_sensor_t.state ? "LIGHT" : "DARK");

    /* 初始化按键定时器 */
    key_timer_init();

    /* 一个回调处理两个引脚 */
    gpio_init_callback(&s_misc_io_cb, misc_io_isr,
                       BIT(fun_key.pin) |
                       BIT(light_det.pin));
    gpio_add_callback(fun_key.port, &s_misc_io_cb);

    return 0;
}

/********************************************************************
**函数名称:  ctrl_pwm_pm_init
**入口参数:  无
**出口参数:  无
**函数功能:  PWM 电源管理初始化
**返 回 值:  0 表示成功
*********************************************************************/
static int ctrl_pwm_pm_init(void)
{
    // 初始化蜂鸣器 PWM
    if (!pwm_is_ready_dt(&buzzer))
    {
        MY_LOG_ERR("Buzzer PWM not ready");
        return -ENODEV;
    }
    return 0;
}

/********************************************************************
**函数名称:  my_ctrl_pwm_pm_register
**入口参数:  无
**出口参数:  无
**函数功能:  将 PWM 模块注册到统一 PM 框架
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
int my_ctrl_pwm_pm_register(void)
{
    return my_pm_device_register(MY_PM_DEV_PWM, &s_ctrl_pwm_pm_ops);
}

/********************************************************************
**函数名称:  my_ctrl_stop_buzzer
**入口参数:  无
**出口参数:  无
**函数功能:  停止蜂鸣器发声
**返 回 值:  无
**功能描述:  将 PWM 占空比设为 0，关闭蜂鸣器
*********************************************************************/
void my_ctrl_stop_buzzer(void)
{
    pwm_set_pulse_dt(&buzzer, 0);
}

/********************************************************************
**函数名称:  my_ctrl_start_buzzer
**入口参数:  无
**出口参数:  无
**函数功能:  开启蜂鸣器发声
**返 回 值:  无
**功能描述:  将 PWM 占空比设为 50，开启蜂鸣器(当前脉宽暂时设置250000,由于开机响的时候设置了周期，此值为周期一半)
*********************************************************************/
void my_ctrl_start_buzzer(void)
{
    pwm_set_pulse_dt(&buzzer, 250000);
}

/********************************************************************
**函数名称:  my_ctrl_buzzer_play_tone
**入口参数:  freq_hz       ---   频率(Hz)，0 表示停止
**           duration_ms   ---   持续时间(ms)，0 表示持续发声
**出口参数:  无
**函数功能:  播放指定频率的声音
**返 回 值:  0 表示成功，负值表示失败
**功能描述:  1. 根据频率计算 PWM 周期和占空比
**           2. 设置 PWM 输出
**           3. 若指定持续时间，则延时后自动停止
*********************************************************************/
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
            MY_LOG_ERR("Failed to set PWM (err %d)", err);
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

/********************************************************************
**函数名称:  my_ctrl_buzzer_play_sequence
**入口参数:  notes      ---   音符数组指针
**           num_notes  ---   音符数量
**出口参数:  无
**函数功能:  播放音符序列
**返 回 值:  0 表示成功，负值表示失败
**功能描述:  依次播放音符数组中的每个音符，间隔 10ms
*********************************************************************/
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

/********************************************************************
**函数名称:  leds_init
**入口参数:  无
**出口参数:  无
**函数功能:  初始化 LED GPIO
**返 回 值:  0 表示成功，负值表示失败
**功能描述:  1. 检查 GPIO 设备就绪状态
**           2. 配置电量指示灯为输出，默认灭
*********************************************************************/
static int leds_init(void)
{
    int ret;

    /* 所有电量 LED 共用同一个 port（gpio2），检查第一个即可 */
    if (!device_is_ready(batt_leds[0].port))
    {
        return -ENODEV;
    }

    /* 配置电量指示灯为输出，默认灭 */
    for (size_t i = 0; i < ARRAY_SIZE(batt_leds); i++)
    {
        ret = gpio_pin_configure_dt(&batt_leds[i], GPIO_OUTPUT_INACTIVE);
        if (ret)
        {
            return ret;
        }
    }

    return 0;
}

/********************************************************************
**函数名称:  my_led_process
**入口参数:  led_id       --  LED ID，BATT_LED0~BATT_LED3
**           led_cmd      --  LED 操作命令，OPEN_LED、CLOSE_LED、TOGGLE_LED、FICKER_LED
**出口参数:  无
**函数功能:  根据指定的操作命令执行相应的 LED 控制操作
**返 回 值:  无
*********************************************************************/
void my_ctrl_led_process(my_led_id_t led_id, my_led_ctrl_cmd_t led_cmd)
{
    // MY_LOG_INF("my_led:%d cmd:%d", led_id, led_cmd);
    // 根据 LED 操作命令执行不同的操作
    switch (led_cmd)
    {
        case OPEN_LED:
            gpio_pin_set_dt(&batt_leds[led_id], 1);
            break;

        case CLOSE_LED:
            gpio_pin_set_dt(&batt_leds[led_id], 0);
            break;

         case TOGGLE_LED:
            gpio_pin_toggle_dt(&batt_leds[led_id]);
            break;

        default:
            /* 忽略未知操作 */
            break;
    }
}

/********************************************************************
**函数名称:  batt_led_set_level
**入口参数:  level    ---   电量等级 (0~3)
**出口参数:  无
**函数功能:  设置电量指示灯等级
**返 回 值:  0 表示成功，负值表示失败
**功能描述:  根据 level 值点亮对应数量的电量 LED
**           0 -> 全灭
**           1 -> 只亮 batt_led0
**           2 -> 亮 batt_led0, batt_led1
**           3 -> 亮 batt_led0, batt_led1, batt_led2
*********************************************************************/
int batt_led_set_level(uint8_t level)
{
    int ret;
    int on;

    if (level > 3)
    {
        level = 3;
    }

    for (size_t i = 0; i < ARRAY_SIZE(batt_leds); i++)
    {
        on = (i < level) ? 1 : 0;
#if 0
        ret = gpio_pin_set_dt(&batt_leds[i], on);
        if (ret < 0)
        {
            return ret;
        }
#endif
        my_ctrl_led_process(i, on);
    }

    return 0;
}

/**
********************************************************************
**函数名称：  my_set_buzzer_mode
**入口参数：  buzzer_mode - 蜂鸣器模式枚举值 (MY_BUZZER_MODE)
**                        例如: BUZZER_STOP 等
**出口参数：  无
**函数功能：  设置蜂鸣器工作模式并触发控制任务处理
**返 回 值：  无
**功能描述：  向控制模块 (MOD_CTRL) 发送消息 (MY_MSG_CTRL_BUZZER_MODE)
********************************************************************
*/
void my_set_buzzer_mode(my_buzzer_mode_t buzzer_mode)
{
    msg_t msg;
    my_buzzer_mode_t *buzzer_mode_loc;

    MY_MALLOC_BUFFER(buzzer_mode_loc, sizeof(my_buzzer_mode_t));
    if(buzzer_mode_loc == NULL)
    {
        MY_LOG_ERR("buzzer_mode_loc malloc failed");
        return;
    }
    *buzzer_mode_loc = buzzer_mode;

    // 构建消息结构体并发送给MAIN模块
    msg.msgID = MY_MSG_CTRL_BUZZER_MODE;
    msg.pData = buzzer_mode_loc;

    my_send_msg_data(MOD_CTRL, MOD_CTRL, &msg);
}

/**
********************************************************************
**函数名称：  g_buzzer_ctrl_config
**入口参数：  on_time   - 蜂鸣器单次“响”的持续时间 (单位: 100ms tick数)
**           off_time  - 蜂鸣器单次“停”的持续时间 (单位: 100ms tick数)
**           repeat    - 重复次数 (0: 无限循环; >0: 指定次数)
**出口参数:  无
**函数功能:  配置蜂鸣器控制结构体的基本参数
**返 回 值:  无
**功能描述:  将传入的时间参数和重复次数保存到全局控制结构体,repeat是重复次数，如果
            持续X秒，计算方式repeat = X*10/(on_time+off_time)
********************************************************************
*/
void g_buzzer_ctrl_config(int on_time, int off_time, int repeat)
{
    s_buzzer_ctrl.on_time = on_time;
    s_buzzer_ctrl.off_time = off_time;
    s_buzzer_ctrl.repeat = repeat;
    //启动蜂鸣器，状态为1
    s_buzzer_ctrl.state = 1;
    s_buzzer_ctrl.tick = 0;
}

/**
********************************************************************
**函数名称:  my_buzzer_play
**入口参数:  buzzer_mode - 蜂鸣器音效类型枚举值 (MY_BUZZER_MODE)
**出口参数:  无
**函数功能:  根据传入的类型选择对应的蜂鸣器音效模式并启动
**返 回 值:  无
**功能描述:  1. 解析 buzzer_type，调用 config 函数设置对应的响/停时间及重复次数。
**           2. 发送开启蜂鸣器消息 (MY_MSG_CTRL_BUZZER_ON)。
**           3. 重置内部状态机 (state=1, tick=0)。
**           4. 重启定时器 buzzer_timer，设置为每 100ms 触发一次中断。
**注意事项:  时间单位统一为 100ms。定时器周期固定为 100ms。
********************************************************************
*/
void my_buzzer_play(my_buzzer_mode_t buzzer_mode)
{
    MY_LOG_INF("buzzer_mode = %d", buzzer_mode);
    switch(buzzer_mode)
    {
        case BUZZER_STOP:
            k_timer_stop(&s_buzzer_timer);
            my_send_msg(MOD_CTRL, MOD_CTRL, MY_MSG_CTRL_BUZZER_OFF);
            return;

        case BUZZER_CONTINUOUS_ALARM:
            //持续报警：响200ms, 停500ms
            g_buzzer_ctrl_config(2, 5, 0);
            break;

        case BUZZER_FAIL_TONE:
            //失败提示：响200ms, 停200ms, 重复3次
            g_buzzer_ctrl_config(2, 2, 3);
            break;

        case BUZZER_ERROR_TONE:
            //异常提示：响100ms, 停100ms, 重复5次
            g_buzzer_ctrl_config(1, 1, 5);
            break;

        case BUZZER_GENERAL_ALARM:
            //一般报警：响200ms, 停300ms, 重复60次 (30秒)
            g_buzzer_ctrl_config(2, 3, 60);
            break;

    }

    my_send_msg(MOD_CTRL, MOD_CTRL, MY_MSG_CTRL_BUZZER_ON);

    k_timer_stop(&s_buzzer_timer);
    //初始化计数器
    k_timer_start(&s_buzzer_timer, K_MSEC(100), K_MSEC(100));
}

/**
********************************************************************
**函数名称:  buzzer_timer_handler
**入口参数:  timer - 定时器对象指针 (未使用)
**出口参数:  无
**函数功能:  蜂鸣器时序控制中断回调函数 (每100ms触发一次)
**返 回 值:  无
**功能描述:  实现蜂鸣器的“响-停-响”节奏控制状态机。
**           1. 累加 tick 计数 (每个tick代表100ms)。
**           2. 根据当前 state (1:响, 0:停) 判断是否达到设定时间。
**           3. 状态切换时发送开/关消息，并重置 tick。
**           4. 在“响”转“停”时递减 repeat 计数，若为0则停止定时器。
**注意事项:  此函数运行在中断上下文或高优先级线程中，应避免耗时操作。
********************************************************************
*/
static void buzzer_timer_handler(struct k_timer *timer)
{
    ARG_UNUSED(timer);

    s_buzzer_ctrl.tick++;

    if (s_buzzer_ctrl.state)
    {
        // 当前是“响”（判断是否等于响多久的时间）
        if (s_buzzer_ctrl.tick >= s_buzzer_ctrl.on_time)
        {
            //达到响多久即可关闭蜂鸣器
            my_send_msg(MOD_CTRL, MOD_CTRL, MY_MSG_CTRL_BUZZER_OFF);
            s_buzzer_ctrl.state = 0;
            s_buzzer_ctrl.tick = 0;

            //重复次数--，为0即可关闭定时器
            if (s_buzzer_ctrl.repeat > 0)
            {
                s_buzzer_ctrl.repeat--;
                if (s_buzzer_ctrl.repeat == 0)
                    k_timer_stop(&s_buzzer_timer);
            }
        }
    }
    else
    {
        // 当前是“关”
        if (s_buzzer_ctrl.tick >= s_buzzer_ctrl.off_time)
        {
            my_send_msg(MOD_CTRL, MOD_CTRL, MY_MSG_CTRL_BUZZER_ON);
            s_buzzer_ctrl.state = 1;
            s_buzzer_ctrl.tick = 0;
        }
    }

}

/********************************************************************
**函数名称:  my_ctrl_task
**入口参数:  p1, p2, p3   ---   线程参数（未使用）
**出口参数:  无
**函数功能:  控制模块主线程
**返 回 值:  无
**功能描述:  1. 循环接收消息队列消息
**           2. 根据消息 ID 分发处理不同事件
**           3. 处理按键短按/长按事件等
*********************************************************************/
static void my_ctrl_task(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    msg_t msg;
    int ret;

    MY_LOG_INF("Control thread started");

    for (;;)
    {
        my_recv_msg(&my_ctrl_msgq, (void *)&msg, sizeof(msg_t), K_FOREVER);

        switch (msg.msgID)
        {
            case MY_MSG_SHOW_CHARG:
                my_battery_show_chgled();//显示充电状态LED
                break;

            case MY_MSG_UPDATE_BATTERY:
                my_battery_update_state();//更新电池状态
                break;

            case MY_MSG_CTRL_BUZZER_MODE:
                if (msg.pData)
                {
                    my_buzzer_play(*(my_buzzer_mode_t *)msg.pData);
                    //释放内存
                    MY_FREE_BUFFER(msg.pData);
                    msg.pData = NULL;
                }

               break;

            case MY_MSG_CTRL_BUZZER_ON:
                ret = my_pm_device_resume(MY_PM_DEV_PWM);
                if (ret < 0)
                {
                    MY_LOG_ERR("Beep PM resume failed: %d", ret);
                    break;
                }
                my_ctrl_start_buzzer();
                break;

            case MY_MSG_CTRL_BUZZER_OFF:
                my_ctrl_stop_buzzer();
                ret = my_pm_device_suspend(MY_PM_DEV_PWM);
                if (ret < 0)
                {
                    MY_LOG_ERR("Beep PM suspend failed: %d", ret);
                    break;
                }
                break;

            case MY_MSG_CTRL_LIGHT_SENSOR_BRIGHT:
                MY_LOG_INF("Light sensor detected: BRIGHT");
                //上报拆除检测告警
                if(gConfigParam.remalm_config.remalm_sw)
                {
                    send_alarm_message_to_lte(ALARM_OPEN, NULL);
                }
                break;

            case MY_MSG_CTRL_LIGHT_SENSOR_DARK:
                MY_LOG_INF("Light sensor detected: DARK");
                break;

            default:
                break;
        }
    }
}

/********************************************************************
**函数名称:  my_ctrl_init
**入口参数:  tid      ---   指向线程 ID 变量的指针
**出口参数:  tid      ---   存储启动后的线程 ID
**函数功能:  初始化控制模块并启动控制线程
**返 回 值:  0 表示成功，负值表示失败
**功能描述:  1. 初始化按键、光感 IO 中断
**           2. 初始化 LED GPIO
**           3. 初始化电池 ADC GPIO
**           4. 检查蜂鸣器 PWM 就绪状态
**           5. 初始化消息队列处理
**           6. 启动控制线程并设置名称
**           7. 播放启动提示音
*********************************************************************/
int my_ctrl_init(k_tid_t *tid)
{
    int ret;

    // 初始化消息队列
    my_init_msg_handler(MOD_CTRL, &my_ctrl_msgq);

    // 启动控制线程
    *tid = k_thread_create(&s_my_ctrl_task_data, my_ctrl_task_stack,
                           K_THREAD_STACK_SIZEOF(my_ctrl_task_stack),
                           my_ctrl_task, NULL, NULL, NULL,
                           MY_CTRL_TASK_PRIORITY, 0, K_NO_WAIT);

    // 设置线程名称
    k_thread_name_set(*tid, "MY_CTRL");

    //  初始化按键、光感、LED GPIO、batt
    misc_io_init();
    leds_init();
    // 注：初始化中会立即开启定时器触发batt_update_timer_handler回调，会向ctrl发送消息（由于未初始化会丢消息），需放在ctrl初始化之后
    batt_adc_init();
    batt_gpio_init();

    ret = my_battery_pm_register();
    if (ret < 0)
    {
        MY_LOG_ERR("Battery PM registration failed: %d", ret);
        return ret;
    }

    ret = my_ctrl_pwm_pm_register();
    if (ret < 0)
    {
        MY_LOG_ERR("PWM PM registration failed: %d", ret);
        return ret;
    }

    /* 启动时响一声提示音 */
    my_ctrl_buzzer_play_tone(2000, 100);

    k_timer_init(&s_buzzer_timer, buzzer_timer_handler, NULL);

    MY_LOG_INF("Control module initialized");
    return 0;
}
