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

/* 必须在包含 my_comm.h 之前定义 BLE_LOG_MODULE_ID，避免与 my_ble_log.h 中的默认定义冲突 */
#define BLE_LOG_MODULE_ID BLE_LOG_MOD_CTRL

#include "my_comm.h"

/* 注册控制模块日志 */
LOG_MODULE_REGISTER(my_ctrl, LOG_LEVEL_INF);

#define WAKEUP_PIN_PSEL  4   /* P0.4 -> PSEL = 4 */
/* 循环定时器周期：50ms */
#define KEY_POLL_PERIOD_MS   50
/* 长按阈值：1.5秒 = 30个周期 */
#define KEY_LONG_PRESS_COUNT (1500 / KEY_POLL_PERIOD_MS)
/* 光感消抖时间：100ms */
#define LIGHT_DEBOUNCE_MS 100
/* 锁销消抖时间：100ms */
#define LOCK_PIN_DEBOUNCE_MS 100
/* 定义NFC刷卡记录缓存的最大数量（可根据实际需求调整） */
#define NFC_CACHE_MAX_NUM 10
/* 重复刷卡判断时间阈值：60秒 */
#define NFC_REPEAT_INTERVAL 60

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

/* 按键控制结构 */
static struct
{
    struct k_timer timer; /* 50ms 轮询定时器 */
    uint32_t press_count; /* 按下计数器 (50ms单位) */
    bool pressed;         /* 按键是否按下 */
} key_ctrl;

/* 光感检测控制结构 */
static struct
{
    struct k_timer timer;       /* 消抖定时器 */
    bool state;                 /* 当前光感状态（true=有光，false=无光） */
    bool debouncing;            /* 消抖中标志 */
} light_sensor;

/* 锁销检测控制结构 */
static struct
{
    struct k_timer timer;       /* 消抖定时器 */
    bool inserted;              /* 当前锁销状态（true=插入，false=断开） */
    bool debouncing;            /* 消抖中标志 */
} lock_pin_ctrl;

/**
 * @brief 锁 LED 控制结构体
 * 
 * 用于控制锁 LED 的闪烁模式，包含定时器和闪烁参数
 */
static struct
{
    struct k_timer timer;       /**< LED 控制定时器，用于定时控制 LED 亮灭 */
    uint16_t on_ms;             /**< LED 点亮的时间（单位：100毫秒） */
    uint16_t period_ms;         /**< LED 闪烁的周期（单位：100毫秒） */
    uint32_t duration_ms;       /**< LED 闪烁的总持续时间（单位：100毫秒），0表示持续闪烁 */
    uint32_t timer_count;       /**< 定时器计数，用于跟踪闪烁状态 */
}s_lock_led_ctrl;

/* NFC刷卡记录缓存结构体 */
typedef struct {
    uint8_t card_id[16];        /* 存储NFC卡号（根据实际卡号长度调整） */
    uint8_t id_len;             /* 卡号实际长度 */
    time_t last_swipe_time;     /* 最近一次刷卡时间戳 */
} NfcCardCache;

static BUZZER_Ctrl_S g_buzzer_ctrl = { 0 };
int g_buzzer_mode = 0;//蜂鸣器状态，默认stop

/* 定时器回调前向声明 */
static void key_timer_handler(struct k_timer *timer);
static void light_sensor_timer_handler(struct k_timer *timer);
static void lock_pin_timer_handler(struct k_timer *timer);
static void lock_led_timer_handler(struct k_timer *timer);

/* 消息队列定义 */
K_MSGQ_DEFINE(my_ctrl_msgq, sizeof(MSG_S), 10, 4);

/* 线程数据与栈定义 */
K_THREAD_STACK_DEFINE(my_ctrl_task_stack, MY_CTRL_TASK_STACK_SIZE);
static struct k_thread my_ctrl_task_data;
static struct gpio_callback misc_io_cb;

/* 当前选中的NFC卡在缓存中的索引，-1表示未选中 */
static int8_t current_card_index = -1;
/* NFC卡号缓存数组，存储最近刷过的NFC卡信息（最多NFC_CACHE_MAX_NUM张） */
static NfcCardCache g_nfc_card_cache[NFC_CACHE_MAX_NUM] = {0};
/* 自动上锁定时器，用于锁销插入后的自动上锁计时 */
struct k_timer auto_lock_timer;
//蜂鸣器100ms定时器
struct k_timer buzzer_timer;

/********************************************************************
**函数名称:  find_card_in_cache
**入口参数:  card_id    ---        输入，待查找的NFC卡号指针
            id_len     ---        输入，卡号长度
**出口参数:  无
**函数功能:  在NFC卡号缓存中查找指定卡号
**返 回 值:  返回卡号在缓存中的索引，未找到返回-1
*********************************************************************/
static int find_card_in_cache(const uint8_t *card_id, uint8_t id_len)
{
    for (int i = 0; i < NFC_CACHE_MAX_NUM; i++)
    {
        /* 卡号长度一致且内容匹配 */
        if (g_nfc_card_cache[i].id_len == id_len &&
            memcmp(g_nfc_card_cache[i].card_id, card_id, id_len) == 0)
        {
            return i;
        }
    }
    return -1;
}

/********************************************************************
**函数名称:  update_card_cache
**入口参数:  card_id    ---        输入，待更新或添加的NFC卡号指针
            id_len     ---        输入，卡号长度
            swipe_time ---        输入，刷卡时间戳
**出口参数:  无
**函数功能:  更新或添加NFC卡号到缓存，若卡号已存在则更新时间，否则添加新记录
**返 回 值:  无
*********************************************************************/
static void update_card_cache(const uint8_t *card_id, uint8_t id_len, time_t swipe_time)
{
    int index;
    int oldest_index;
    time_t oldest_time;
    int i;

    index = find_card_in_cache(card_id, id_len);

    if (index >= 0)
    {
        /* 已存在，更新刷卡时间 */
        g_nfc_card_cache[index].last_swipe_time = swipe_time;
    }
    else
    {
        /* 未找到，找第一个空位置或覆盖最旧记录
         * 当缓存空间满了之后,再次刷新的卡会把时间最早的卡给覆盖掉
         */
        oldest_index = 0;
        oldest_time = g_nfc_card_cache[0].last_swipe_time;

        for (i = 1; i < NFC_CACHE_MAX_NUM; i++)
        {
            /* 优先使用空位置（时间为0） */
            if (g_nfc_card_cache[i].last_swipe_time == 0)
            {
                oldest_index = i;
                break;
            }
            /* 找最旧的记录 */
            if (g_nfc_card_cache[i].last_swipe_time < oldest_time)
            {
                oldest_time = g_nfc_card_cache[i].last_swipe_time;
                oldest_index = i;
            }
        }

        /* 写入新卡号和时间 */
        memcpy(g_nfc_card_cache[oldest_index].card_id, card_id, id_len);
        g_nfc_card_cache[oldest_index].id_len = id_len;
        g_nfc_card_cache[oldest_index].last_swipe_time = swipe_time;
    }
}

/********************************************************************
**函数名称:  is_need_location_upload
**入口参数:  card_id    ---        输入，待判断的NFC卡号指针
            id_len     ---        输入，卡号长度
**出口参数:  无
**函数功能:  判断是否需要执行定位上传，60秒内重复刷卡返回0，否则返回1
**返 回 值:  0表示无需定位上传（60秒内重复刷卡），1表示需要定位上传
*********************************************************************/
static int is_need_location_upload(const uint8_t *card_id, uint8_t id_len)
{
    int index;
    time_t now;
    time_t time_diff;

    index = find_card_in_cache(card_id, id_len);
    now = time(NULL);

    if (index >= 0)
    {
        // 计算时间差（秒）
        time_diff = now - g_nfc_card_cache[index].last_swipe_time;
        if (time_diff >= 0 && time_diff <= NFC_REPEAT_INTERVAL)
        {
            // 60秒内重复刷卡，无需定位上传
            return 0;
        }
    }

    // 首次刷卡或超时，需要定位上传并更新缓存
    update_card_cache(card_id, id_len, now);
    return 1;
}

/********************************************************************
**函数名称:  nfc_card_detected
**入口参数:  card_id     ---        输入，NFC卡号指针
            id_len      ---        输入，卡号长度
            card_index  ---        输出，匹配的授权卡片索引
**出口参数:  card_index  ---        输出，存储匹配的授权卡片索引
**函数功能:  检测NFC卡片是否有效，验证卡片ID、时间范围、解锁次数等条件
**返 回 值:  -1表示验证失败，1表示需要位置验证，0表示验证通过
*********************************************************************/
int nfc_card_detected(uint8_t *card_id, uint8_t id_len, uint8_t *card_index)
{
    uint8_t i;
    int time_check_result;
    time_t current_time;
    NfcAuthCard *current_card;

    /* 初始化当前卡片索引为无效值 */
    current_card_index = -1;

    /* 检查输入参数有效性 */
    if (card_id == NULL || id_len == 0)
    {
        return -1;
    }

    /* 遍历所有授权卡片 */
    for (i = 0; i < g_device_cmd_config.nfcauth_card_count; i++)
    {
        if (strncmp(card_id, g_device_cmd_config.nfcauth_cards[i].nfc_no, id_len) == 0)
        {
            /* 记录匹配的卡片索引 */
            current_card_index = i;
            break;
        }
    }

    /* 检查是否找到匹配的卡片 */
    if (current_card_index == -1)
    {
        /* 未找到匹配卡片，返回失败 */
        MY_LOG_INF("No matching card was found.");
        return -1;
    }

    current_card = &g_device_cmd_config.nfcauth_cards[current_card_index];  /* 获取当前卡片指针 */

    /* 检查是否需要时间验证 */
    if (current_card->time_valid == 1)
    {
        current_time = my_get_system_time_sec();
        if (current_time == (time_t)-1)
        {
            /* 时间获取失败，返回失败 */
            MY_LOG_INF("get time fail");
            return -1;
        }

        /* 检查当前时间是否在允许范围内 */
        time_check_result = is_time_in_range(current_card->start_time, current_card->end_time, current_time);
        if (time_check_result != 1)
        {
            /* 时间不在允许范围内，返回失败 */
            MY_LOG_INF("Time is not within the scope.");
            return -1;
        }
    }

    /* 检查剩余解锁次数 */
    if (current_card->unlock_times == 0)
    {
        /* 解锁次数为0，返回失败 */
        MY_LOG_INF("unlock_times is 0.");
        return -1;
    }

    /* 检查是否需要位置验证 */
    if (current_card->lat_lon_valid == 1)
    {
        /* 需要位置验证，返回1 */
        MY_LOG_INF("need location verification");
        return 1;
    }

    /* 若卡的次数有限,需要消耗次数(-1为无限次数) */
    if (current_card->unlock_times > 0)
    {
        current_card->unlock_times--;
    }

    MY_LOG_INF("current_card->unlock_times:%d", current_card->unlock_times);

    *card_index = current_card_index;
    /* 验证通过，返回0 */
    return 0;
}

/********************************************************************
**函数名称:  handle_nfc_card_event
**入口参数:  card_id    ---        输入，NFC卡号指针
            id_len     ---        输入，卡号长度
**出口参数:  无
**函数功能:  处理NFC刷卡事件，检查重复刷卡、验证卡片权限并执行相应操作
**返 回 值:  无
*********************************************************************/
void handle_nfc_card_event(uint8_t *card_id, uint8_t id_len)
{
    int ret;
    uint8_t card_index = 0;

    /* 重复刷卡缓存记录检查 */
    ret = is_need_location_upload(card_id, id_len);
    if (ret == 0)
    {
        MY_LOG_INF("repeated swiping of the card id:%s", card_id);
        return ;
    }

    // TODO 4G就绪后发送NFC刷卡事件：BLE+ALARM=<告警类型>(NFC),< 时间戳 >,< 附加信息 >(NFC卡号)
    my_send_msg(MOD_CTRL, MOD_LTE, MY_MSG_LTE_PWRON);

    ret = nfc_card_detected(card_id, id_len, &card_index);
    /* 符合开锁规则 */
    if (ret == 0)
    {
        // TODO 发消息控制成功提示音
        /* 启动开锁操作 */
        my_send_msg(MOD_CTRL, MOD_CTRL, MY_MSG_CTRL_OPENLOCKING);
        MY_LOG_INF("start to openlock");
    }
    /* 需要位置验证 */
    else if (ret == 1)
    {
        // TODO 后续通过发消息通知4G需要获取经纬度信息
        // ?设置一个标记位，待拿到经纬度信息后，再判断开锁规则
    }
    /* 权限不足 */
    else if (ret == -1)
    {
        // TODO 发消息控制异常提示音
        MY_LOG_INF("card no permission");
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
        // MY_LOG_INF("Light state changed: %s", new_state ? "LIGHT" : "DARK");

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

    uint32_t msgID;
    int level = gpio_pin_get(lock_pin_det.port, lock_pin_det.pin);
    bool new_state = (level == 1);

    if (new_state != lock_pin_ctrl.inserted)
    {
        lock_pin_ctrl.inserted = new_state;
        // MY_LOG_INF("Lock pin state changed: %s", new_state ? "INSERTED" : "DISCONNECTED");

        /* 发送锁销状态变化消息到主任务 */
        msgID = new_state ? MY_MSG_CTRL_LOCK_PIN_INSERTED : MY_MSG_CTRL_LOCK_PIN_DISCONNECTED;
        if (new_state)
        {
            my_send_msg(MOD_CTRL, MOD_CTRL, msgID);
        }
        else
        {
            /* 锁销被拔出时,检测到锁是关闭状态 */
            if (get_closelock_state())
            {
                // MY_LOG_INF("The locking pin was illegally pulled out.");
                // TODO 锁销非法拔出上报,直接发消息给LTE线程,由4G判断是否要上报

                /* 蜂鸣器报警 */
                if (g_device_cmd_config.lockpincyt_buzzer == 1)
                {
                    // TODO 发消息到ctrl线程,报警30s
                }
                else if (g_device_cmd_config.lockpincyt_buzzer == 2)
                {
                    // TODO 发消息到ctrl线程,持续报警直到收到关闭蜂鸣器报警指令
                }
            }
        }
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
**函数名称:  auto_lock_detection
**入口参数:  无
**出口参数:  无
**函数功能:  启动自动上锁定时器，根据配置的倒计时时间执行自动上锁
**返 回 值:  无
*********************************************************************/
static void auto_lock_detection(void)
{
    MY_LOG_INF("%s:run", __func__);
    if (g_device_cmd_config.lockcd_countdown == 0)
    {
        return ;
    }

    // 自动上锁定时器
    k_timer_start(&auto_lock_timer, K_MSEC(g_device_cmd_config.lockcd_countdown * 1000), K_NO_WAIT);
}

/********************************************************************
**函数名称:  auto_lock_handler
**入口参数:  timer  ---        输入，定时器句柄（未使用）
**出口参数:  无
**函数功能:  自动上锁定时器回调函数，发送关锁消息到控制模块
**返 回 值:  无
*********************************************************************/
static void auto_lock_handler(struct k_timer *timer)
{
    ARG_UNUSED(timer);

    my_send_msg(MOD_CTRL, MOD_CTRL, MY_MSG_CTRL_CLOSELOCKING);
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
        MY_LOG_ERR("Failed to configure fun_key: %d", ret);
        return ret;
    }

    /* 配置为输入（light_det 和 cut_det 配置为输入） */
    ret = gpio_pin_configure_dt(&light_det, GPIO_INPUT);
    if (ret)
    {
        MY_LOG_ERR("Failed to configure light_det: %d", ret);
        return ret;
    }

    /* 锁销检测配置：配置为输入（lock_pin_det 配置为输入） */
    ret = gpio_pin_configure_dt(&lock_pin_det, GPIO_INPUT);
    if (ret)
    {
        MY_LOG_ERR("Failed to configure lock_pin_det: %d", ret);
        return ret;
    }

    /* 配置按键中断：下降沿触发 */
    ret = gpio_pin_interrupt_configure_dt(&fun_key, GPIO_INT_EDGE_FALLING);
    if (ret)
    {
        MY_LOG_ERR("Failed to configure fun_key interrupt: %d", ret);
        return ret;
    }

    /* 光感检测配置：配置光感中断：下降沿触发 */
    ret = gpio_pin_interrupt_configure_dt(&light_det, GPIO_INT_EDGE_FALLING);
    if (ret)
    {
        MY_LOG_ERR("Failed to configure light_det interrupt: %d", ret);
        return ret;
    }

    /* 初始化光感定时器和状态 */
    k_timer_init(&light_sensor.timer, light_sensor_timer_handler, NULL);
    light_sensor.state = false;
    light_sensor.debouncing = false;
    /* 读取初始状态 */
    light_initial_level = gpio_pin_get(light_det.port, light_det.pin);
    light_sensor.state = (light_initial_level == 1);
    MY_LOG_INF("Light initial state: %s", light_sensor.state ? "LIGHT" : "DARK");

    /* 配置锁销中断：双边沿触发 */
    ret = gpio_pin_interrupt_configure_dt(&lock_pin_det, GPIO_INT_EDGE_BOTH);
    if (ret)
    {
        MY_LOG_ERR("Failed to configure lock_pin_det interrupt: %d", ret);
        return ret;
    }

    /* 初始化锁销定时器和状态 */
    k_timer_init(&lock_pin_ctrl.timer, lock_pin_timer_handler, NULL);
    lock_pin_ctrl.inserted = false;
    lock_pin_ctrl.debouncing = false;
    /* 读取初始状态 */
    lock_initial_level = gpio_pin_get(lock_pin_det.port, lock_pin_det.pin);
    lock_pin_ctrl.inserted = (lock_initial_level == 1);
    MY_LOG_INF("Lock pin initial state: %s", lock_pin_ctrl.inserted ? "INSERTED" : "DISCONNECTED");

    /* 初始化按键定时器 */
    key_timer_init();

    /* 初始化自动上锁定时器 */
    k_timer_init(&auto_lock_timer, auto_lock_handler, NULL);

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

     k_timer_init(&s_lock_led_ctrl.timer, lock_led_timer_handler, NULL);

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
void my_ctrl_led_process(MY_LED_ID led_id, MY_LED_CTRL_CMD led_cmd)
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
    //MY_LOG_INF("%s:%d", __func__, on);
    gpio_pin_set_dt(&lock_led, on ? 1 : 0);
}

/*********************************************************************
**函数名称:  my_battery_show_chgled
**入口参数:  timer  ---        输入，定时器句柄（未使用）
**出口参数:  无
**函数功能:  作为锁 LED 控制定时器的回调函数，用于控制锁 LED 的闪烁模式，
**           根据配置的参数实现不同的闪烁效果，包括持续闪烁和指定时间后停止闪烁两种模式。
*********************************************************************/
static void lock_led_timer_handler(struct k_timer *timer)
{
    // 根据定时器计数和配置的参数控制 LED 点亮和熄灭
    if (s_lock_led_ctrl.timer_count % s_lock_led_ctrl.period_ms == 0)
    {
        lock_led_set(true);  // 周期开始时点亮 LED
    }
    else if (s_lock_led_ctrl.timer_count % s_lock_led_ctrl.period_ms == s_lock_led_ctrl.on_ms)
    {
        lock_led_set(false);  // 点亮指定时间后熄灭 LED
    }

    s_lock_led_ctrl.timer_count++;  // 增加定时器计数

    // 处理持续闪烁模式（duration_ms 为 0）
    if (s_lock_led_ctrl.duration_ms == 0)
    {
        // 当计数达到周期时，重置计数，实现持续闪烁
        if (s_lock_led_ctrl.timer_count >= s_lock_led_ctrl.period_ms)
        {
            s_lock_led_ctrl.timer_count = 0;
        }
    }
    else
    {
        // 处理指定时间后停止闪烁模式
        if (s_lock_led_ctrl.timer_count >= s_lock_led_ctrl.duration_ms)
        {
            k_timer_stop(&s_lock_led_ctrl.timer);  // 停止定时器
            lock_led_set(false);  // 熄灭 LED
        }
    }
}

/*********************************************************************
**函数名称:  my_lock_led_ctrl_start
**入口参数:  on_ms  ---        LED 点亮的时间（毫秒）
**           off_ms ---        LED 熄灭的时间（毫秒）
**           duration_ms ---        LED 闪烁的总持续时间（毫秒），0 表示持续闪烁
**出口参数:  无
**函数功能:  用于启动锁 LED 的闪烁控制，根据传入的参数配置 LED 的闪烁模式。
**           可以设置 LED 点亮时间、熄灭时间和总持续时间。
**           函数将传入的时间必须为100的倍数，因为定时器的周期是 100 毫秒。
**           当 on_ms 为 0 时，函数会关闭 LED 并停止定时器。
**           当 off_ms为0时，LED会常亮。
**           当 duration_ms为0时，LED会持续闪烁。
*********************************************************************/
void my_lock_led_ctrl_start(uint16_t on_ms, uint16_t off_ms, uint32_t duration_ms)
{
    if (on_ms == 0)
    {
        k_timer_stop(&s_lock_led_ctrl.timer);  // 停止 LED 控制定时器
        lock_led_set(false);  // 关闭 LED
    }
    else
    {
        s_lock_led_ctrl.on_ms = on_ms / 100;  // 将点亮时间转换为 100 毫秒为单位
        s_lock_led_ctrl.period_ms = (on_ms+off_ms) / 100;  // 计算闪烁周期并转换为 100 毫秒为单位
        s_lock_led_ctrl.duration_ms = duration_ms / 100;  // 将总持续时间转换为 100 毫秒为单位
        s_lock_led_ctrl.timer_count = 0;  // 重置定时器计数
        k_timer_start(&s_lock_led_ctrl.timer, K_MSEC(0), K_MSEC(100));  // 启动定时器，立即执行一次，然后 100 毫秒循环执行
    }
}

/*********************************************************************
**函数名称:  my_lock_led_set_mode
**入口参数:  mode  ---        LED 显示模式，使用 MY_LOCK_LED_MODE 枚举类型
**出口参数:  无
**函数功能:  该函数根据传入的模式参数，设置锁 LED 的不同显示模式，
**          包括关闭、NFC启动、解锁中和上锁中和已解锁模式。
*********************************************************************/
void my_lock_led_set_mode(MY_LOCK_LED_MODE mode)
{
    switch (mode)
    {
        case LOCK_LED_CLOSE:
            // 关闭 LED 模式
            my_lock_led_ctrl_start(0, 0, 0);  // 传入 on_ms=0，关闭 LED
            break;

        case LOCK_LED_NFC_START:
            // NFC 启动模式，LED 以 200ms 亮、500ms 灭的频率闪烁
            my_lock_led_ctrl_start(200, 500, 0);
            break;

        case LOCK_LED_LOCKED:
            // 解锁中和上锁中模式，LED 以 200ms 亮、200ms 灭的频率持续闪烁
            my_lock_led_ctrl_start(200, 200, 0);  // duration_ms=0，表示持续闪烁
            break;

        case LOCK_LED_UNLOCK:
            // 已解锁模式，LED 以 500ms 亮、1000ms 灭的频率闪烁，持续 18 秒
            my_lock_led_ctrl_start(500, 1000, 18000);
            break;

        default:
            // 处理未定义的模式
            break;
    }
}

/*********************************************************************
**函数名称:  my_lock_led_msg_send
**入口参数:  mode  ---        LED 显示模式，使用 MY_LOCK_LED_MODE 枚举类型
**出口参数:  无
**函数功能:  用于发送锁 LED 控制消息到控制模块，设置锁 LED 的显示模式
*********************************************************************/
void my_lock_led_msg_send(MY_LOCK_LED_MODE mode)
{
    MSG_S msg;  // 消息结构体，用于发送 LED 控制消息
    static MY_LOCK_LED_MODE led_mode;  // 静态存储 LED 模式，确保消息处理时数据有效

    led_mode = mode;  // 存储 LED 模式

    msg.msgID = MY_MSG_CTRL_LOCK_LED;  // 设置消息 ID 为锁 LED 控制消息
    msg.pData = &led_mode;  // 设置消息数据为 LED 模式的地址

    my_send_msg_data(MOD_CTRL, MOD_CTRL, &msg);  // 发送消息到控制模块
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
    g_buzzer_ctrl.on_time = on_time;
    g_buzzer_ctrl.off_time = off_time;
    g_buzzer_ctrl.repeat = repeat;
    //启动蜂鸣器，状态为1
    g_buzzer_ctrl.state = 1;
    g_buzzer_ctrl.tick = 0;
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
void my_buzzer_play(int buzzer_mode)
{
    MY_LOG_INF("buzzer_mode = %d", buzzer_mode);
    switch(buzzer_mode)
    {
        case BUZZER_STOP:
            //定时器在运行就停止定时器
            if (k_timer_remaining_get(&buzzer_timer) != 0)
            {
                k_timer_stop(&buzzer_timer);
            }
            my_send_msg(MOD_CTRL, MOD_CTRL, MY_MSG_CTRL_BUZZER_OFF);
            return;

        case BUZZER_CONTINUOUS_ALARM:
            //持续报警：响200ms, 停500ms
            g_buzzer_ctrl_config(2, 5, 0);
            break;

        case BUZZER_UNLOCK_SUCCESS:
            //解锁成功：响500ms, 不停顿, 仅播放一次
            g_buzzer_ctrl_config(5, 0, 1);
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

        case BUZZER_EVENT_LOCK_SUCCESS:
            //上锁成功：响100ms, 停300ms, 重复2次
            g_buzzer_ctrl_config(1, 3, 2);
            break;

        case BUZZER_EVENT_LOCK_FAIL:
            //上锁/解锁失败：响1000ms, 停500ms, 重复3次
            g_buzzer_ctrl_config(10, 5, 3);
            break;

        case BUZZER_EVENT_UNAUTHORIZED:
            // 未授权提示：响100ms, 停100ms, 重复5次
            g_buzzer_ctrl_config(1, 1, 5);
            break;

        case BUZZER_EVENT_NFC_ACTIVATE:
            //NFC激活：响100ms, 无停顿, 仅播放一次
            g_buzzer_ctrl_config(1, 0, 1);//提示100ms
            break;
    }

    my_send_msg(MOD_CTRL, MOD_CTRL, MY_MSG_CTRL_BUZZER_ON);

    if (k_timer_remaining_get(&buzzer_timer) != 0)
    {
        k_timer_stop(&buzzer_timer);
    }
    //初始化计数器
    k_timer_start(&buzzer_timer, K_MSEC(100), K_MSEC(100));
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

    g_buzzer_ctrl.tick++;

    if (g_buzzer_ctrl.state)
    {
        // 当前是“响”（判断是否等于响多久的时间）
        if (g_buzzer_ctrl.tick >= g_buzzer_ctrl.on_time)
        {
            //达到响多久即可关闭蜂鸣器
            my_send_msg(MOD_CTRL, MOD_CTRL, MY_MSG_CTRL_BUZZER_OFF);
            g_buzzer_ctrl.state = 0;
            g_buzzer_ctrl.tick = 0;

            //重复次数--，为0即可关闭定时器
            if (g_buzzer_ctrl.repeat > 0)
            {
                g_buzzer_ctrl.repeat--;
                if (g_buzzer_ctrl.repeat == 0)
                    k_timer_stop(&buzzer_timer);
            }
        }
    }
    else
    {
        // 当前是“关”
        if (g_buzzer_ctrl.tick >= g_buzzer_ctrl.off_time)
        {
            my_send_msg(MOD_CTRL, MOD_CTRL, MY_MSG_CTRL_BUZZER_ON);
            g_buzzer_ctrl.state = 1;
            g_buzzer_ctrl.tick = 0;
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

    MSG_S msg;

    MY_LOG_INF("Control thread started");

    for (;;)
    {
        my_recv_msg(&my_ctrl_msgq, (void *)&msg, sizeof(MSG_S), K_FOREVER);

        switch (msg.msgID)
        {
            case MY_MSG_SHOW_CHARG:
                my_battery_show_chgled();//显示充电状态LED
                break;

            case MY_MSG_UPDATE_BATTERY:
                my_battery_update_state();//更新电池状态
                break;

            case MY_MSG_CTRL_OPENLOCKING:
                req_open_lock_action();
                /* 开锁中闪烁LED */
                my_lock_led_set_mode(LOCK_LED_LOCKED);
                break;

            case MY_MSG_CTRL_CLOSELOCKING:
                req_close_lock_action();
                /* 关锁中闪烁LED */
                my_lock_led_set_mode(LOCK_LED_LOCKED);
                break;

            case MY_MSG_CTRL_STOPLOCK:
                stop_lock_action();
                break;

            case MY_MSG_CTRL_LOCK_PIN_INSERTED:
                auto_lock_detection();
                break;

            case MY_MSG_CTRL_LOCK_PIN_DISCONNECTED:
                MY_LOG_INF("Lock pin detected: DISCONNECTED");
                break;

            case MY_MSG_CTRL_LOCK_LED:
                my_lock_led_set_mode(*(MY_LOCK_LED_MODE *)msg.pData);
                break;

            case MY_MSG_CTRL_BUZZER_MODE:
                my_buzzer_play(g_buzzer_mode);
               break;

            case MY_MSG_CTRL_BUZZER_ON:
                my_ctrl_start_buzzer();
                break;

            case MY_MSG_CTRL_BUZZER_OFF:
                my_ctrl_stop_buzzer();
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
        MY_LOG_ERR("Buzzer PWM not ready");
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

    k_timer_init(&buzzer_timer, buzzer_timer_handler, NULL);

    MY_LOG_INF("Control module initialized");
    return 0;
}