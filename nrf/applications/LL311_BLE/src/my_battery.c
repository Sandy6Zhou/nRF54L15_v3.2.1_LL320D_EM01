/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        my_battery.c
**文件描述:        电池管理模块实现文件
**当前版本:        V1.0
**作    者:        周森达 (zhousenda@jimiiot.com)
**完成日期:        2026.02.04
*********************************************************************
** 功能描述:        1. 电池电压、NTC采集以及电源开关使能
**                  2. 电池状态、充电状态检测
*********************************************************************/

/* 必须在包含 my_comm.h 之前定义 BLE_LOG_MODULE_ID，避免与 my_ble_log.h 中的默认定义冲突 */
#define BLE_LOG_MODULE_ID BLE_LOG_MOD_BATTERY

#include "my_comm.h"

LOG_MODULE_REGISTER(my_battery, LOG_LEVEL_INF);

/* zephyr,user 里有两个 io-channels：index 0 = 通道0(电池)，index 1 = 通道1(NTC) */
#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

// 电池电压-百分比映射表（3.0V-4.2V锂电）
// 格式：{电压(mV), 百分比(%)}
// TODO: 现在是根据网上找到的电池电压-百分比映射表，等拿到电池规格书后需要根据实际情况调整
#define BATT_VOLT_PERCENT_MAP \
{ \
    {3000, 0},   /* 3.0V -> 0% */ \
    {3300, 10},  /* 3.3V -> 10% */ \
    {3500, 20},  /* 3.5V -> 20% */ \
    {3600, 30},  /* 3.6V -> 30% */ \
    {3700, 40},  /* 3.7V -> 40% */ \
    {3800, 50},  /* 3.8V -> 50% */ \
    {3900, 60},  /* 3.9V -> 60% */ \
    {4000, 70},  /* 4.0V -> 70% */ \
    {4100, 80},  /* 4.1V -> 80% */ \
    {4150, 90},  /* 4.15V -> 90% */ \
    {4200, 100}, /* 4.2V -> 100% */ \
}

#define BATT_ADC_SAMPLE_NUM 3  /**< 电池电压采样次数，用于提高测量准确性, 数字必须大于3, 去掉一个最大值和最小值*/
#define BATT_UPDATE_COUNT 3      /**< 电池电量更新计数阈值，用于实现电量显示的平滑过渡 */

#define BATT_TIMER_MS 100  /**< 电池状态检查定时器的周期（毫秒），用于控制LED亮灭 */
#define BATT_LED_TIMER_MS  5000  /**< 正常状态电池 LED 显示的总时间（毫秒），超过此时间后关闭所有 LED */
#define CHG_DEBOUNCE_MS 50 /**< 充电引脚相关消抖时间（毫秒） */
#define CHG_CTRL_LED_MS 500/**< 充电状态LED控制定时器的周期（毫秒） */
#define CHG_UPDATE_S 10 /**< 充电状态电池电量更新周期（秒） */
#define BATT_UPDATE_S 30 /**< 正常状态电池电量更新周期（秒） */

//电量状态阈值, 百分比表示, 0~100%
#define BATT_EMPTY_VAL 5    /**< 电池电量为空的阈值，当电量低于此值时，电池状态为 BATT_EMPTY */
#define BATT_LOW_VAL 30     /**< 电池电量低的阈值，当电量低于此值时，电池状态为 BATT_LOW */
#define BATT_NORMAL_VAL 45  /**< 电池电量正常的阈值，当电量低于此值时，电池状态为 BATT_NORMAL */
#define BATT_FAIR_VAL 65    /**< 电池电量良好的阈值，当电量低于此值时，电池状态为 BATT_FAIR */
#define BATT_HIGH_VAL 85    /**< 电池电量高的阈值，当电量低于此值时，电池状态为 BATT_HIGH */

#define CHG_BATT_LOW_VAL 30     /**< 充电时电池电量低的阈值，当电量低于此值时，充电状态为 CHG_POWER_LOW */
#define CHG_BATT_FAIR_VAL 60    /**< 充电时电池电量中等的阈值，当电量低于此值时，充电状态为 CHG_POWER_FAIR */
#define CHG_BATT_HIGH_VAL 85    /**< 充电时电池电量高的阈值，当电量低于此值时，充电状态为 CHG_POWER_HIGH */

/* 通道 0：电池电压 */
static const struct adc_dt_spec batt_adc = ADC_DT_SPEC_GET_BY_IDX(ZEPHYR_USER_NODE, 0);

static const struct gpio_dt_spec batt_pwr_en = GPIO_DT_SPEC_GET(DT_ALIAS(batt_pwr_ctrl), gpios);
static const struct gpio_dt_spec charge_state = GPIO_DT_SPEC_GET(DT_ALIAS(batt_state), gpios);
static const struct gpio_dt_spec charge_det = GPIO_DT_SPEC_GET(DT_ALIAS(charge_detect), gpios);

static struct gpio_callback batt_gpio_cb;
// ADC原始采样值
static int16_t batt_raw;

static struct adc_sequence batt_seq = {
    .buffer = &batt_raw,
    .buffer_size = sizeof(batt_raw),
};

static struct k_timer g_chg_stat_timer; // 充电状态检测消抖定时器，用于充电状态引脚的消抖处理
static struct k_timer g_chg_det_timer;  // 充电检测消抖定时器，用于充电检测引脚的消抖处理
static struct k_timer g_batt_update_timer;     // 电池电量检测定时器

// 正常状态LED控制结构体，包含定时器、电池状态和计数器
Batt_LED_Ctrl_S g_batt_led_ctrl = { 0 };

// 充电状态LED控制结构体，包含定时器、充电电池状态和计数器
CHG_LED_Ctrl_S g_chg_led_ctrl = { 0 };

static int g_chg_stat_level = 0;        // 当前充电状态引脚电平，用于消抖处理
static int g_chg_det_level = 0;         // 当前充电检测引脚电平，用于消抖处理

static bool g_batt_disable_flag = false;  /**< 电池充电禁止标志，用于控制充电路径的开关，消除充电电压对电池电压测量的影响 */

// 电源状态，初始值为未连接
MY_CHG_STATE g_charg_state = NO_CHARGING;

/*********************************************************************
**函数名称:  batt_timer_handler
**入口参数:  timer 定时器指针，由系统自动传递
**出口参数:  无
**函数功能:  定时器回调函数，正常状态按键按下LED控制
*********************************************************************/
static void batt_timer_handler(struct k_timer *timer)
{
    // note:只执行电平翻转，不能执行打印日志这些耗时操作
    int mode = 0;
    int level = 0;

    mode = g_batt_led_ctrl.state % 2;    // 电池状态LED模式，0为闪烁状态，1为常亮状态
    level = g_batt_led_ctrl.state / 2;  // 电池状态LED等级，代表几个led亮

    // 根据LED显示模式控制LED
    if (mode == 0)
    {
        // 模式 0：LED 闪烁模式
        if (g_batt_led_ctrl.time_count % 10 == 0)
        {
            // 每10个时间单位，点亮比当前级别高一级的LED
            batt_led_set_level(level + 1);
        }
        else if (g_batt_led_ctrl.time_count % 10 == 3)
        {
            // 每10个时间单位的第3个单位，点亮当前级别的LED
            batt_led_set_level(level);
        }
    }
    else
    {
        // 其他模式：LED 常亮模式
        batt_led_set_level(level + 1);
    }

    g_batt_led_ctrl.time_count++;  // 增加电池状态时间计数器

    // 当时间计数器超过阈值时，关闭所有 LED 并停止定时器
    if(g_batt_led_ctrl.time_count >= BATT_LED_TIMER_MS / 100)
    {
        k_timer_stop(g_batt_led_ctrl.timer);  // 停止LED控制定时器
        batt_led_set_level(0);  // 关闭所有LED
    }
}

/* 读取电池电压原始值，并转换为 mV（不含分压系数） */
int batt_read_mv(int32_t *mv)
{
    int err;
    int32_t val_mv;

    err = adc_read(batt_adc.dev, &batt_seq);
    if (err < 0) return err;

    val_mv = batt_raw;
    // TODO 后续需取均值，以及开关ADC处理
    err = adc_raw_to_millivolts_dt(&batt_adc, &val_mv);
    if (err < 0) return err;

    *mv = val_mv;
    return 0;
}

int batt_adc_init(void)
{
    int err;

    /* 检查 ADC 是否就绪 */
    if (!adc_is_ready_dt(&batt_adc))
    {
        return -ENODEV;
    }

    /* 配置电池电压通道（配置来自 devicetree） */
    err = adc_channel_setup_dt(&batt_adc);
    if (err < 0) return err;

    /* 初始化 sequence（会填充通道掩码等） */
    err = adc_sequence_init_dt(&batt_adc, &batt_seq);
    if (err < 0) return err;

    return 0;
}

/* 电池使能：true=打开，false=关闭 */
void batt_enable(bool on)
{
    gpio_pin_set_dt(&batt_pwr_en, on ? 1 : 0);
}

/*********************************************************************
**函数名称:  chg_timer_handler
**入口参数:  timer 定时器指针，由系统自动传递
**出口参数:  无
**函数功能:  充电状态LED控制定时器的回调函数，用于控制充电状态LED的显示
*********************************************************************/
void chg_timer_handler(struct k_timer *timer)
{
    // note:只执行电平翻转，不能执行打印日志这些耗时操作
    // 根据时间计数器的值，控制LED的闪烁模式
    if (g_chg_led_ctrl.time_count % 3 == 0)
    {
        // 在第1.5s的前1s点亮LED
        batt_led_set_level(g_chg_led_ctrl.state + 1);
    }
    else if (g_chg_led_ctrl.time_count % 3 == 2)
    {
        // 在第1s处熄灭对应LED
        batt_led_set_level(g_chg_led_ctrl.state);
    }

    g_chg_led_ctrl.time_count++;  // 增加时间计数器
}

/*********************************************************************
**函数名称:  my_chg_event_reporting
**入口参数:  无
**出口参数:  无
**函数功能:  通过比较当前充电状态与上次记录的充电状态，
**           当状态发生变化且充电状态报告功能开启时，
**           生成包含时间戳、报告配置和当前充电状态的命令参数，
**           并通过 LTE 网络发送 ALARM 命令上报事件。
*********************************************************************/
void my_chg_event_reporting()
{
    static MY_CHG_STATE s_last_chg_state = CHARG_UNKNOWN;     // 上次充电状态
    char cmd_param[2] = {0};                 // 命令参数缓冲区

    // 当充电状态发生变化且报告功能开启时，上报事件
    if (s_last_chg_state != g_charg_state)
    {
        // 格式化命令参数：包含时间戳、报告配置和当前充电状态
        snprintf(cmd_param, sizeof(cmd_param), "%d", g_charg_state);

        // 通过 LTE 发送 ALARM 命令
        send_alarm_message_to_lte(ALARM_CHARGE, cmd_param);

        // 更新上次充电状态
        s_last_chg_state = g_charg_state;
    }
}

/*********************************************************************
**函数名称:  my_battery_show_chgled
**入口参数:  无
**出口参数:  无
**函数功能:  检查充电检测引脚的状态，根据检测结果控制LED显示。
**           如果检测到充电，则启动充电LED控制定时器；
**           如果未检测到充电，则关闭所有电池LED。
*********************************************************************/
void my_battery_show_chgled()
{
    // 检查充电检测引脚状态，如果为 1 表示正在充电
    if (gpio_pin_get_dt(&charge_det) == 1)
    {
        LOG_INF("The charger is plugged in.");
         // 检查充电状态引脚，如果为低电平表示充电已满
        if (gpio_pin_get_dt(&charge_state) == 0)
        {
            g_charg_state = CHARG_FULL;  // 设置充电状态为已满
            LOG_INF("charge full");  // 输出充电已满信息
        }
        else
        {
            g_charg_state = CHARGING;  // 设置充电状态为正在充电
        }

        my_battery_update_state();      // 更新电池状态
        // 检查 LED 显示功能是否启用
        if (gConfigParam.led_config.led_display == 1)
        {
            g_chg_led_ctrl.time_count = 0;  // 重置充电LED控制定时器的时间计数
            // 启动充电LED控制定时器，立即执行一次，然后以 CHG_CTRL_LED_MS 为周期重复执行
            k_timer_start(g_chg_led_ctrl.timer, K_MSEC(0), K_MSEC(CHG_CTRL_LED_MS));
        }

        // 检查正常情况下的电池状态LED控制定时器是否正在运行
        if (k_timer_remaining_get(g_batt_led_ctrl.timer) != 0)
        {
            // 如果正在运行，停止定时器，避免与充电状态LED显示冲突
            k_timer_stop(g_batt_led_ctrl.timer);
        }
    }
    else
    {
        k_timer_stop(g_chg_led_ctrl.timer);  // 停止充电LED控制定时器
        g_charg_state = NO_CHARGING;  // 设置充电状态为未充电
        batt_enable(true);  // 启用电池电源
        batt_led_set_level(0);  // 关闭所有电池LED
        LOG_INF("The charger is not plugged in.");
    }

    // 充电状态发生变化时上报事件
    my_chg_event_reporting();
}

/*********************************************************************
**函数名称:  batt_chg_stat_handle
**入口参数:  timer 定时器指针，由系统自动传递
**出口参数:  无
**函数功能:  定时器回调函数，比较当前检测到的电平与上一次保存的电平，实现消抖功能，
**向控制模块发送消息作LED控制，当充电状态引脚电平为 0 时，表示充电已满
*********************************************************************/
void batt_chg_stat_handle(struct k_timer *timer)
{
    static int s_last_chg_stat_level = 0;   // 上一次充电状态引脚电平，用于消抖处理

    // 如果当前检测到的电平与上一次保存的电平相同，说明是抖动，直接返回
    if (g_chg_stat_level == s_last_chg_stat_level)
        return;

    // 再次读取引脚电平，如果与之前检测到的电平一致，则认为是稳定状态
    if (g_chg_stat_level == gpio_pin_get_dt(&charge_state))
    {
        s_last_chg_stat_level = g_chg_stat_level;  // 更新上一次保存的电平

        my_send_msg(MOD_CTRL, MOD_CTRL, MY_MSG_SHOW_CHARG);     // 发送消息到控制模块
    }
}

/*********************************************************************
**函数名称:  batt_chg_det_handle
**入口参数:  timer 定时器指针，由系统自动传递
**出口参数:  无
**函数功能:  定时器回调函数，比较当前检测到的电平与上一次保存的电平，实现消抖功能，向控制模块发送消息作LED控制
*********************************************************************/
void batt_chg_det_handle(struct k_timer *timer)
{
    static int s_last_chg_det_level = 0;    // 上一次充电检测引脚电平，用于消抖处理

    // 如果当前检测到的电平与上一次保存的电平相同，说明是抖动，直接返回
    if (g_chg_det_level == s_last_chg_det_level)
        return;

    // 再次读取引脚电平，如果与之前检测到的电平一致，则认为是稳定状态
    if (g_chg_det_level == gpio_pin_get_dt(&charge_det))
    {
        s_last_chg_det_level = g_chg_det_level;  // 更新上一次保存的电平

        my_send_msg(MOD_CTRL, MOD_CTRL, MY_MSG_SHOW_CHARG);     // 发送消息到控制模块
    }
}

/* 共用 ISR：根据 pins 掩码区分是 P1.7 还是 P1.8 触发 */
static void batt_gpio_isr(const struct device *dev,
                        struct gpio_callback *cb,
                        uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);

    if (pins & BIT(charge_state.pin))
    {
        /* P1.7 触发：电池状态变化（充电中 / 充满） */
        g_chg_stat_level = gpio_pin_get_dt(&charge_state);
        //50ms消抖处理
        k_timer_start(&g_chg_stat_timer, K_MSEC(CHG_DEBOUNCE_MS), K_NO_WAIT);
    }

    if (pins & BIT(charge_det.pin))
    {
        /* P1.8 触发：充电检测变化（开始充电 / 停止充电） */
        g_chg_det_level = gpio_pin_get_dt(&charge_det);
        //50ms消抖处理
        k_timer_start(&g_chg_det_timer, K_MSEC(CHG_DEBOUNCE_MS), K_NO_WAIT);
    }
}

/*********************************************************************
**函数名称:  batt_update_timer_handler
**入口参数:  timer 定时器指针，由系统自动传递
**出口参数:  无
**函数功能:  作为电池状态更新定时器的回调函数，在采集电池电压时关闭充电路径，
**          采集完毕后打开供电路径，以消除充电电压对电池电压测量的影响，
**          并定期发送电池状态更新消息。
*********************************************************************/
void batt_update_timer_handler(struct k_timer *timer)
{
    // 当时间计数器达到特定值且充电状态为高电量或满电量时，禁用电池电源,当电池电压过低时，不能禁用充电功能
    // 消除充电电压对电池电压抬高效应的影响。在采集电池电压输出高1s关闭充电路径，采集完毕输出低打开供电路径
    if (g_batt_disable_flag == true && g_chg_led_ctrl.state > CHG_BATT_FAIR)
    {
        batt_enable(false);  // 禁用电池电源
    }

    // 发送电池状态更新消息到控制模块
    my_send_msg(MOD_CTRL, MOD_CTRL, MY_MSG_UPDATE_BATTERY);
}

int batt_gpio_init(void)
{
    int ret;
    // 正常状态LED控制定时器，用于控制正常状态LED的显示
    static struct k_timer s_batt_timer;
    // 充电状态LED控制定时器，用于控制充电状态LED的显示
    static struct k_timer s_chg_timer;

    if (!device_is_ready(batt_pwr_en.port) ||
        !device_is_ready(charge_state.port) ||
        !device_is_ready(charge_det.port))
    {
        return -ENODEV;
    }

    /* 使能口：输出，默认打开 */
    ret = gpio_pin_configure_dt(&batt_pwr_en, GPIO_OUTPUT_ACTIVE);
    if (ret) return ret;

    gpio_pin_set_dt(&batt_pwr_en, 0);

    /* 状态/充电检测：输入（上拉在 DTS 里已经配置） */
    ret = gpio_pin_configure_dt(&charge_state, GPIO_INPUT);
    if (ret) return ret;

    ret = gpio_pin_configure_dt(&charge_det, GPIO_INPUT);
    if (ret) return ret;

    /* 配置中断：
     * 根据硬件逻辑选择：
     *  - GPIO_INT_EDGE_FALLING：只在拉低时触发
     *  - GPIO_INT_EDGE_RISING：只在拉高时触发
     *  - GPIO_INT_EDGE_BOTH：高/低变化都触发
     */
    ret = gpio_pin_interrupt_configure_dt(&charge_state, GPIO_INT_EDGE_BOTH);
    if (ret) return ret;

    ret = gpio_pin_interrupt_configure_dt(&charge_det, GPIO_INT_EDGE_BOTH);
    if (ret) return ret;

    /* 一个回调处理两个引脚 */
    gpio_init_callback(&batt_gpio_cb, batt_gpio_isr,
                       BIT(charge_state.pin) | BIT(charge_det.pin));
    gpio_add_callback(charge_state.port, &batt_gpio_cb);
    gpio_add_callback(charge_det.port, &batt_gpio_cb);

    // 初始化正常状态LED控制定时器
    g_batt_led_ctrl.timer = &s_batt_timer;
    k_timer_init(g_batt_led_ctrl.timer, batt_timer_handler, NULL);

    // 初始化充电状态LED控制定时器
    g_chg_led_ctrl.timer = &s_chg_timer;
    k_timer_init(g_chg_led_ctrl.timer, chg_timer_handler, NULL);

    // 初始化充电状态(是否充满)检测引脚消抖定时器
    k_timer_init(&g_chg_stat_timer, batt_chg_stat_handle, NULL);

    // 初始化充电检测（是否有充电器插入）消抖定时器
    k_timer_init(&g_chg_det_timer, batt_chg_det_handle, NULL);

    // 初始化电池状态更新定时器
    k_timer_init(&g_batt_update_timer, batt_update_timer_handler, NULL);
    // 启动电池状态更新定时器
    k_timer_start(&g_batt_update_timer, K_MSEC(0), K_NO_WAIT);

    return 0;
}

/*********************************************************************
**函数名称:  my_battery_read_mv
**入口参数:  *mv  --  指向存储电池电压值的指针，单位为毫伏
**出口参数:  err  --  错误码，0 表示成功，非 0 表示失败
**函数功能:  读取电池电压值（毫伏）,函数采用多次采样并去除极值的方法提高测量准确性,
**           最终结果会乘以 2，是因为硬件电路中的分压系数
*********************************************************************/
int my_battery_read_mv(int32_t *mv)
{
    int err = 0;            // 错误码
    int i = 0;              // 循环计数器
    int32_t max_mv = 0;     // 最大电压值
    int32_t min_mv = 10000; // 最小电压值，初始化为一个较大的值
    int32_t sum_mv = 0;     // 电压总和

    // 进行多次采样
    for (i = 0; i < BATT_ADC_SAMPLE_NUM; i++)
    {
        // 读取单次电池电压
        err = batt_read_mv(mv);
        if (err != 0)
        {
            LOG_ERR("Battery voltage acquisition failed!");  // 输出错误日志
            return err;  // 返回错误码
        }

        // 更新最大电压值
        if (*mv > max_mv)
        {
            max_mv = *mv;
        }

        // 更新最小电压值
        if (*mv < min_mv)
        {
            min_mv = *mv;
        }

        // 累加电压值
        sum_mv += *mv;
    }

    // 计算平均电压：去掉最大值和最小值后取平均，再乘以 2(硬件电路中的分压系数)
    *mv = ((sum_mv-max_mv-min_mv) * 2) / (BATT_ADC_SAMPLE_NUM-2);

    return 0;  // 返回成功
}

/*********************************************************************
**函数名称:  my_battery_read_percent
**入口参数:  无
**出口参数:  battery_percent_val  --  电池电量百分比，范围为 0-100
**函数功能:  读取电池电压值，然后根据电压-电量映射表进行线性插值计算，
**           得到当前电池的电量百分比。
*********************************************************************/
int8_t my_battery_read_percent()
{
    // 电压-电量映射表
    static const Batt_Volt_Percent_Map_S s_batt_volt_map[] = BATT_VOLT_PERCENT_MAP;
    // 映射表条目数量
    static const int s_batt_map_entries = (sizeof(s_batt_volt_map) / sizeof(s_batt_volt_map[0]));
    int i = 0;                          // 循环变量，用于遍历电压-电量映射表数组
    int read_mv = 0;                    // 电池电压值（毫伏）
    int8_t battery_percent_val = 0;     // 电池电量百分比

    my_battery_read_mv(&read_mv);  // 读取电池电压值（毫伏）
    LOG_INF("read_mv:%d", read_mv);  // 输出读取到的电池电压值（毫伏）

    // 处理电压低于最低映射值的情况
    if (read_mv < s_batt_volt_map[0].mv)
    {
        return 0;  // 返回 0% 电量
    }
    // 处理电压高于最高映射值的情况
    else if (read_mv > s_batt_volt_map[s_batt_map_entries-1].mv)
    {
        return 100;  // 返回 100% 电量
    }

    // 查找电压值所在的映射区间，-1是防止数组越界
    for (i = 0; i < s_batt_map_entries - 1; i++)
    {
        if (read_mv >= s_batt_volt_map[i].mv &&
            read_mv <= s_batt_volt_map[i+1].mv)
        {
            break;  // 找到电压值所在的区间，退出循环
        }
    }

    // 使用线性插值计算电池电量百分比
    battery_percent_val = s_batt_volt_map[i].percent + ((read_mv - s_batt_volt_map[i].mv) * \
    (s_batt_volt_map[i+1].percent - s_batt_volt_map[i].percent) / (s_batt_volt_map[i+1].mv - s_batt_volt_map[i].mv));

    return battery_percent_val;  // 返回计算得到的电池电量百分比
}

/*********************************************************************
**函数名称:  my_battery_update_state
**入口参数:  无
**出口参数:  无
**函数功能:  通过比较当前电池状态与上次记录的电池状态，
**           当状态发生变化时，根据不同的电池状态和对应的上报配置，
**           生成包含时间戳、报告配置和当前电池状态的命令参数，
**           并通过 LTE 网络发送 ALARM 命令上报事件。
*********************************************************************/
void my_battery_event_reporting()
{
    static MY_BATT_STATE s_last_batt_state = BATT_UNKNOWN;   // 上次电池状态
    char cmd_param[2] = {0};                 // 命令参数缓冲区

    // 当电池状态发生变化时，根据不同状态进行处理
    if (g_batt_led_ctrl.state != s_last_batt_state)
    {
        snprintf(cmd_param, sizeof(cmd_param), "%d", g_batt_led_ctrl.state);
        send_alarm_message_to_lte(ALARM_BATT, cmd_param);

        // 更新上次电池状态
        s_last_batt_state = g_batt_led_ctrl.state;
    }
}

/*********************************************************************
**函数名称:  my_battery_update_state
**入口参数:  无
**出口参数:  无
**函数功能:  根据电池电量百分比更新电池状态和充电状态,实现电量显示的平滑过渡，
**           并根据充电状态调整更新频率
*********************************************************************/
void my_battery_update_state()
{
    static bool s_batt_first_update = true;  // 上电首次更新标志
    static int8_t s_show_percent = 0;        // 显示的电池电量百分比
    static int s_batt_update_count = 0;       // 电池更新计数器
    static int8_t s_jump_percent = -1;         // 电量跳变值
    int8_t percent = 0;                       // 当前电池电量百分比

    // 根据充电状态设置不同的更新定时器
    if (g_charg_state == NO_CHARGING)
    {
        g_batt_disable_flag = false;
        // 未充电时，使用较长的更新周期
        k_timer_start(&g_batt_update_timer, K_MSEC(BATT_UPDATE_S * 1000), K_NO_WAIT);
    }
    else
    {
        // 充电时，使用较短的更新周期
        // 提前1s进入中断关闭充电使能, 因为在充电中检测电压会比正常情况高，导致电量显示异常
        if (g_batt_disable_flag == true)
        {
            g_batt_disable_flag = false;
            k_timer_start(&g_batt_update_timer, K_MSEC(1000), K_NO_WAIT);
            return;
        }
        else
        {
            g_batt_disable_flag = true;
            k_timer_start(&g_batt_update_timer, K_MSEC((CHG_UPDATE_S - 1) * 1000), K_NO_WAIT);
        }
    }

    // 读取当前电池电量百分比
    percent = my_battery_read_percent();
    LOG_INF("percent:%d", percent);  // 输出测量得到的电量百分比

    // 处理首次更新的情况
    if (s_batt_first_update == true)
    {
        s_batt_first_update = false;
        // 初始化更新计数器和显示电量百分比
        s_batt_update_count = 0;
        s_show_percent = percent;
        // 初始化时发送一次充电消息, 函数会在里面检测是否是充电状态, 确保在上电时显示正确的充电状态
        my_send_msg(MOD_CTRL, MOD_CTRL, MY_MSG_SHOW_CHARG);
    }

    // 未充电时的电量更新逻辑
    if (g_charg_state == NO_CHARGING)
    {
        // 电量下降时，实现平滑过渡
        if (percent < s_show_percent)
        {
            s_batt_update_count++;
            // 当连续检测到电量下降时，实现平滑过渡，每次更新减少1个单位
            if (s_batt_update_count >= BATT_UPDATE_COUNT)
            {
                s_batt_update_count = 0;
                s_show_percent--;
            }
        }
        else
        {
            s_batt_update_count = 0;
        }
    }
    else
    {
        // 充电时的电量更新逻辑
        if (percent > s_show_percent)
        {
            s_batt_update_count++;
            // 当连续检测到电量上升时，实现平滑过渡，每次更新增加1个单位
            if (s_batt_update_count >= BATT_UPDATE_COUNT)
            {
                s_batt_update_count = 0;
                s_show_percent++;
            }
        }
        else
        {
            s_batt_update_count = 0;
        }
    }

    // 处理电量跳变的情况,比如温度变化导致电量跳变
    if (abs(percent - s_show_percent) >= 5)
    {
        if (s_jump_percent < 0)
        {
            s_jump_percent = percent;
        }
        else
        {
            // 当电量连续跳变超过5个单位并相差1个单位时，更新显示电量百分比
            if (abs(percent - s_jump_percent) <= 1)
            {
                s_show_percent = percent;
                s_jump_percent = -1;
            }
            else
            {
                s_jump_percent = percent;
            }
        }
    }
    else
    {
        s_jump_percent = -1;
    }

    LOG_INF("s_show_percent:%d", s_show_percent);  // 输出当前显示的电量百分比

    // 根据电池电量判断电池状态
    if (s_show_percent <= BATT_EMPTY_VAL)
    {
        g_batt_led_ctrl.state = BATT_EMPTY;  // 电池电量为空
    }
    else if (s_show_percent <= BATT_LOW_VAL)
    {
        g_batt_led_ctrl.state = BATT_LOW;  // 电池电量低
    }
    else if (s_show_percent <= BATT_NORMAL_VAL)
    {
        g_batt_led_ctrl.state = BATT_NORMAL;  // 电池电量正常
    }
    else if (s_show_percent <= BATT_FAIR_VAL)
    {
        g_batt_led_ctrl.state = BATT_FAIR;  // 电池电量良好
    }
    else if (s_show_percent <= BATT_HIGH_VAL)
    {
        g_batt_led_ctrl.state = BATT_HIGH;  // 电池电量高
    }
    else
    {
        g_batt_led_ctrl.state = BATT_FULL;  // 电池电量满
    }

    LOG_INF("battery class:%d", g_batt_led_ctrl.state);  // 输出电池状态等级
    // 电池状态发生变化时上报事件
    my_battery_event_reporting();

    // 如果正在充电
    if (g_charg_state != NO_CHARGING)
    {
        // 根据电池电量判断充电状态
        if (s_show_percent <= CHG_BATT_LOW_VAL)
        {
            g_chg_led_ctrl.state = CHG_BATT_LOW;  // 充电电量低
        }
        else if (s_show_percent <= CHG_BATT_FAIR_VAL)
        {
            g_chg_led_ctrl.state = CHG_BATT_FAIR;  // 充电电量中等
        }
        else if (s_show_percent <= CHG_BATT_HIGH_VAL)
        {
            g_chg_led_ctrl.state = CHG_BATT_HIGH;  // 充电电量高
        }
        else
        {
            g_chg_led_ctrl.state = CHG_BATT_FULL;  // 充电电量满
        }

        // 检查充电状态引脚，如果为高电平表示充电中
        if (gpio_pin_get_dt(&charge_state) == 1)
        {
            batt_enable(true);  // 启用充电使能,在定时器回调中检测前1s关闭了充电使能
        }

        LOG_INF("charge class:%d", g_chg_led_ctrl.state);  // 输出充电状态等级
    }
}

/*********************************************************************
**函数名称:  my_battery_show
**入口参数:  无
**出口参数:  无
**函数功能:  启动100ms循环定时器控制LED显示电池状态
*********************************************************************/
void my_battery_show()
{
    if(g_charg_state != NO_CHARGING)
    {
        MY_LOG_INF("be charging");  // 设备正在充电，输出充电状态
        return;  // 直接返回，不进行后续的电池状态检查
    }

    // 检查 LED 显示功能是否启用
    if (gConfigParam.led_config.led_display == 1)
    {
        //当按键重复按下时，重新开始定时器
        g_batt_led_ctrl.time_count = 0;  // 重置时间计数器
        // 启动电池状态LED控制定时器，立即执行一次，然后以 100ms 为周期重复执行
        k_timer_start(g_batt_led_ctrl.timer, K_MSEC(0), K_MSEC(BATT_TIMER_MS));
    }
}
