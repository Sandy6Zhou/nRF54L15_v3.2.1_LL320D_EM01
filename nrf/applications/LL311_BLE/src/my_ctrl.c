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
**                 5. 实现锁销(lock pin)检测中断处理，消抖处理，产生插入/断开事件并发送到主任务
*********************************************************************/

#include "my_comm.h"

/* 注册控制模块日志 */
LOG_MODULE_REGISTER(my_ctrl, LOG_LEVEL_INF);

#define WAKEUP_PIN_PSEL  4   /* P0.4 -> PSEL = 4 */

/* 硬件设备树定义 */
static const struct gpio_dt_spec fun_key = GPIO_DT_SPEC_GET(DT_ALIAS(fun_key), gpios);
static const struct pwm_dt_spec buzzer = PWM_DT_SPEC_GET(DT_ALIAS(buzzer_pwm)); 
static const struct gpio_dt_spec light_det = GPIO_DT_SPEC_GET(DT_ALIAS(light_detect), gpios);
static const struct gpio_dt_spec lock_pin_det = GPIO_DT_SPEC_GET(DT_ALIAS(lock_pin_det), gpios);
static const struct gpio_dt_spec lock_led = GPIO_DT_SPEC_GET(DT_ALIAS(lock_led0), gpios);
static const struct gpio_dt_spec batt_leds[] = {
    GPIO_DT_SPEC_GET(DT_ALIAS(battery_led0), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(battery_led1), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(battery_led2), gpios),
};

static struct gpio_callback misc_io_cb;

/* 按键控制结构 */
static struct
{
    struct k_timer timer; /* 50ms 轮询定时器 */
    uint32_t press_count; /* 按下计数器 (50ms单位) */
    bool pressed;         /* 按键是否按下 */
} key_ctrl;

/* 定时器周期：50ms */
#define KEY_POLL_PERIOD_MS   50
/* 长按阈值：1.5秒 = 30个周期 */
#define KEY_LONG_PRESS_COUNT (1500 / KEY_POLL_PERIOD_MS)

/* 光感检测控制结构 */
static struct
{
    struct k_timer timer;       /* 消抖定时器 */
    bool state;                 /* 当前光感状态（true=有光，false=无光） */
    bool debouncing;            /* 消抖中标志 */
} light_sensor;

/* 光感消抖时间：100ms */
#define LIGHT_DEBOUNCE_MS 100

/* 锁销检测控制结构 */
static struct
{
    struct k_timer timer;       /* 消抖定时器 */
    bool inserted;              /* 当前锁销状态（true=插入，false=断开） */
    bool debouncing;            /* 消抖中标志 */
} lock_pin_ctrl;

/* 锁销消抖时间：100ms */
#define LOCK_PIN_DEBOUNCE_MS 100

/* 定时器回调前向声明 */
static void key_timer_handler(struct k_timer *timer);
static void light_sensor_timer_handler(struct k_timer *timer);
static void lock_pin_timer_handler(struct k_timer *timer);

/* 消息队列定义 */
K_MSGQ_DEFINE(my_ctrl_msgq, sizeof(MSG_S), 10, 4);

/* 线程数据与栈定义 */
K_THREAD_STACK_DEFINE(my_ctrl_task_stack, MY_CTRL_TASK_STACK_SIZE);
static struct k_thread my_ctrl_task_data;

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
    /* 1. 配置 P0.4 为输入，并根据外部电路选择上拉/下拉 */
    nrf_gpio_cfg_input(WAKEUP_PIN_PSEL, NRF_GPIO_PIN_PULLUP);

    /* 2. 配置 SENSE 条件，低电平唤醒 */
    nrf_gpio_cfg_sense_set(WAKEUP_PIN_PSEL, NRF_GPIO_PIN_SENSE_LOW);
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
    k_timer_init(&key_ctrl.timer, key_timer_handler, NULL);
    key_ctrl.pressed = false;
    key_ctrl.press_count = 0;
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
    MSG_S msg;
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

    if (level == 0)
    {
        /* 按键持续按下 */
        if (!key_ctrl.pressed)
        {
            key_ctrl.pressed = true;
            key_ctrl.press_count = 0;
        }

        /* 计数器增加 */
        key_ctrl.press_count++;

        /* 检查是否达到长按阈值(1.2s) */
        if (key_ctrl.press_count == KEY_LONG_PRESS_COUNT)
        {
            send_key_event(MY_MSG_CTRL_KEY_LONG_PRESS);
        }
    }
    else
    {
        /* 按键已释放 */
        if (key_ctrl.pressed)
        {
            key_ctrl.pressed = false;
            k_timer_stop(&key_ctrl.timer);

            /* 短按判断：大于等于100ms且小于1.2s */
            if (key_ctrl.press_count < KEY_LONG_PRESS_COUNT && key_ctrl.press_count >= 2)
            {
                send_key_event(MY_MSG_CTRL_KEY_SHORT_PRESS);
            }
            key_ctrl.press_count = 0;
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

    if (new_state != light_sensor.state)
    {
        light_sensor.state = new_state;
        // LOG_INF("Light state changed: %s", new_state ? "LIGHT" : "DARK");

        /* 发送光感状态变化消息到主任务 */
        MSG_S msg;
        msg.msgID = new_state ? MY_MSG_CTRL_LIGHT_SENSOR_BRIGHT : MY_MSG_CTRL_LIGHT_SENSOR_DARK;
        msg.pData = NULL;
        msg.DataLen = 0;
        my_send_msg_data(MOD_CTRL, MOD_MAIN, &msg);
    }

    light_sensor.debouncing = false;
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
    if (!light_sensor.debouncing)
    {
        light_sensor.debouncing = true;
        k_timer_start(&light_sensor.timer, K_MSEC(LIGHT_DEBOUNCE_MS), K_NO_WAIT);
    }
}

/********************************************************************
**函数名称:  lock_pin_timer_handler
**入口参数:  timer    ---   定时器指针
**出口参数:  无
**函数功能:  锁销消抖定时器回调
**返 回 值:  无
**功能描述:  1. 100ms后读取锁销电平确认状态
**           2. 状态变化时发送消息到主任务
**           3. 清除消抖标志
*********************************************************************/
static void lock_pin_timer_handler(struct k_timer *timer)
{
    ARG_UNUSED(timer);

    int level = gpio_pin_get(lock_pin_det.port, lock_pin_det.pin);
    bool new_state = (level == 1);

    if (new_state != lock_pin_ctrl.inserted)
    {
        lock_pin_ctrl.inserted = new_state;
        // LOG_INF("Lock pin state changed: %s", new_state ? "INSERTED" : "DISCONNECTED");

        /* 发送锁销状态变化消息到主任务 */
        MSG_S msg;
        msg.msgID = new_state ? MY_MSG_CTRL_LOCK_PIN_INSERTED : MY_MSG_CTRL_LOCK_PIN_DISCONNECTED;
        msg.pData = NULL;
        msg.DataLen = 0;
        my_send_msg_data(MOD_CTRL, MOD_MAIN, &msg);
    }

    lock_pin_ctrl.debouncing = false;
}

/********************************************************************
**函数名称:  lock_pin_edge_handler
**入口参数:  无
**出口参数:  无
**函数功能:  锁销双边沿中断处理
**返 回 值:  无
**功能描述:  1. 检测锁销状态变化（插入/断开）
**           2. 启动100ms消抖定时器
**           3. 避免重复触发消抖
*********************************************************************/
static void lock_pin_edge_handler(void)
{
    if (!lock_pin_ctrl.debouncing)
    {
        lock_pin_ctrl.debouncing = true;
        k_timer_start(&lock_pin_ctrl.timer, K_MSEC(LOCK_PIN_DEBOUNCE_MS), K_NO_WAIT);
    }
}

/********************************************************************
**函数名称:  key_falling_edge_handler
**入口参数:  无
**出口参数:  无
**函数功能:  按键下降沿中断处理
**返 回 值:  无
**功能描述:  1. 检测按键下降沿(按下)中断
**           2. 若定时器未运行则启动50ms循环定时器
**           3. 避免重复启动定时器
*********************************************************************/
static void key_falling_edge_handler(void)
{
    /* 如果定时器未运行，才启动 */
    if (!k_timer_remaining_get(&key_ctrl.timer))
    {
        /* 启动循环定时器，每 50ms 检查一次按键状态 */
        k_timer_start(&key_ctrl.timer, K_MSEC(KEY_POLL_PERIOD_MS), K_MSEC(KEY_POLL_PERIOD_MS));
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
**功能描述:  1. 处理 FUN_KEY 按键下降沿中断
**           2. 处理光感检测中断
**           3. 处理剪线检测中断
*********************************************************************/
static void misc_io_isr(const struct device *dev,
                   struct gpio_callback *cb,
                   uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);

    if (pins & BIT(fun_key.pin))
    {
        /* 下降沿触发，调用处理函数 */
        key_falling_edge_handler();
    }

    if (pins & BIT(light_det.pin))
    {
        /* 双边沿触发，调用光感处理函数 */
        light_sensor_edge_handler();
    }

    if (pins & BIT(lock_pin_det.pin))
    {
        /* 双边沿触发，调用锁销处理函数 */
        lock_pin_edge_handler();
    }
}

/********************************************************************
**函数名称:  misc_io_init
**入口参数:  无
**出口参数:  无
**函数功能:  初始化按键、光感、剪线检测 IO
**返 回 值:  0 表示成功，负值表示失败
**功能描述:  1. 检查 GPIO 设备就绪状态
**           2. 配置 fun_key 为输入（内部上拉）
**           3. 配置 light_det、cut_det 为输入
**           4. 配置三个引脚为下降沿中断触发
**           5. 初始化按键定时器并注册中断回调
*********************************************************************/
static int misc_io_init(void)
{
    int ret;
    int light_initial_level;
    int lock_initial_level;

    if (!device_is_ready(fun_key.port) ||
        !device_is_ready(light_det.port) ||
        !device_is_ready(lock_pin_det.port))
    {
        return -ENODEV;
    }

    /* 配置为输入（fun_key 配置内部上拉） */
    ret = gpio_pin_configure(fun_key.port, fun_key.pin, GPIO_INPUT | GPIO_PULL_UP);
    if (ret)
    {
        LOG_ERR("Failed to configure fun_key: %d", ret);
        return ret;
    }

    /* 配置为输入（light_det 和 cut_det 配置为输入） */
    ret = gpio_pin_configure_dt(&light_det, GPIO_INPUT);
    if (ret)
    {
        LOG_ERR("Failed to configure light_det: %d", ret);
        return ret;
    }

    /* 锁销检测配置：配置为输入（lock_pin_det 配置为输入） */
    ret = gpio_pin_configure_dt(&lock_pin_det, GPIO_INPUT);
    if (ret)
    {
        LOG_ERR("Failed to configure lock_pin_det: %d", ret);
        return ret;
    }

    /* 配置按键中断：下降沿触发 */
    ret = gpio_pin_interrupt_configure_dt(&fun_key, GPIO_INT_EDGE_FALLING);
    if (ret)
    {
        LOG_ERR("Failed to configure fun_key interrupt: %d", ret);
        return ret;
    }

    /* 光感检测配置：配置光感中断：下降沿触发 */
    ret = gpio_pin_interrupt_configure_dt(&light_det, GPIO_INT_EDGE_FALLING);
    if (ret)
    {
        LOG_ERR("Failed to configure light_det interrupt: %d", ret);
        return ret;
    }

    /* 初始化光感定时器和状态 */
    k_timer_init(&light_sensor.timer, light_sensor_timer_handler, NULL);
    light_sensor.state = false;
    light_sensor.debouncing = false;
    /* 读取初始状态 */
    light_initial_level = gpio_pin_get(light_det.port, light_det.pin);
    light_sensor.state = (light_initial_level == 1);
    LOG_INF("Light initial state: %s", light_sensor.state ? "LIGHT" : "DARK");

    /* 配置锁销中断：双边沿触发 */
    ret = gpio_pin_interrupt_configure_dt(&lock_pin_det, GPIO_INT_EDGE_BOTH);
    if (ret)
    {
        LOG_ERR("Failed to configure lock_pin_det interrupt: %d", ret);
        return ret;
    }

    /* 初始化锁销定时器和状态 */
    k_timer_init(&lock_pin_ctrl.timer, lock_pin_timer_handler, NULL);
    lock_pin_ctrl.inserted = false;
    lock_pin_ctrl.debouncing = false;
    /* 读取初始状态 */
    lock_initial_level = gpio_pin_get(lock_pin_det.port, lock_pin_det.pin);
    lock_pin_ctrl.inserted = (lock_initial_level == 1);
    LOG_INF("Lock pin initial state: %s", lock_pin_ctrl.inserted ? "INSERTED" : "DISCONNECTED");

    /* 初始化按键定时器 */
    key_timer_init();

    /* 一个回调处理三个引脚 */
    gpio_init_callback(&misc_io_cb, misc_io_isr,
                       BIT(fun_key.pin) |
                       BIT(light_det.pin) |
                       BIT(lock_pin_det.pin));
    gpio_add_callback(fun_key.port, &misc_io_cb);

    return 0;
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
**           3. 配置锁状态指示灯为输出，默认灭
*********************************************************************/
static int leds_init(void)
{
    int ret;

    /* 所有电量 LED 共用同一个 port（gpio2），检查第一个即可 */
    if (!device_is_ready(batt_leds[0].port) ||
        !device_is_ready(lock_led.port))
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

    /* 配置锁状态指示灯为输出，默认灭 */
    ret = gpio_pin_configure_dt(&lock_led, GPIO_OUTPUT_INACTIVE);
    if (ret)
    {
        return ret;
    }

    return 0;
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
static int batt_led_set_level(uint8_t level)
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
        ret = gpio_pin_set_dt(&batt_leds[i], on);
        if (ret < 0)
        {
            return ret;
        }
    }

    return 0;
}

/********************************************************************
**函数名称:  lock_led_set
**入口参数:  on       ---   true 点亮，false 熄灭
**出口参数:  无
**函数功能:  设置锁状态指示灯
**返 回 值:  无
**功能描述:  根据 on 参数控制锁状态 LED 亮灭
*********************************************************************/
static void lock_led_set(bool on)
{
    LOG_INF("%s:%d", __func__, on);
    gpio_pin_set_dt(&lock_led, on ? 1 : 0);
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
}

/********************************************************************
**函数名称:  my_ctrl_init
**入口参数:  tid      ---   指向线程 ID 变量的指针
**出口参数:  tid      ---   存储启动后的线程 ID
**函数功能:  初始化控制模块并启动控制线程
**返 回 值:  0 表示成功，负值表示失败
**功能描述:  1. 初始化按键、光感、剪线 IO 中断
**           2. 初始化 LED GPIO
**           3. 初始化电机、电池 ADC GPIO
**           4. 检查蜂鸣器 PWM 就绪状态
**           5. 初始化消息队列处理
**           6. 启动控制线程并设置名称
**           7. 播放启动提示音
*********************************************************************/
int my_ctrl_init(k_tid_t *tid)
{
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

    /* 启动时响一声提示音 */
    my_ctrl_buzzer_play_tone(2000, 100);

    LOG_INF("Control module initialized");
    return 0;
}
