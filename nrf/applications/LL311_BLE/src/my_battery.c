/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        my_battery.c
**文件描述:        电池管理模块实现文件
**当前版本:        V1.0
**作    者:        周森达 (zhousenda@jimiiot.com)
**完成日期:        2026.02.04
*********************************************************************
** 功能描述:        1. 电池电压、NTC采集以及电源开关使能
**                 2. 电池状态、充电状态检测
*********************************************************************/

/* 必须在包含 my_comm.h 之前定义 BLE_LOG_MODULE_ID，避免与 my_ble_log.h 中的默认定义冲突 */
#define BLE_LOG_MODULE_ID BLE_LOG_MOD_BATTERY

#include "my_comm.h"

LOG_MODULE_REGISTER(my_battery, LOG_LEVEL_INF);

/* zephyr,user 里有两个 io-channels：index 0 = 通道0(电池)，index 1 = 通道1(NTC) */
#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

#define BATT_TIMER_MS 100  /**< 电池状态检查定时器的周期（毫秒），用于控制LED亮灭 */

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

// 电池状态检查定时器
static struct k_timer g_batt_timer;

// 电池状态检查结构体，包含定时器、状态和计数器
Batt_LED_Ctrl_S g_batt_led_ctrl =
{
    .timer = &g_batt_timer,    // 关联电池定时器
    .state = BATT_NORMAL,            // 初始电池状态设为正常
    .time_count = 0,                 // 初始时间计数器为0
};

// 电源状态，初始值为未连接
MY_CHG_STATE g_charg_state = NO_CHARGING;

/*********************************************************************
**函数名称:  batt_timer_handler
**入口参数:  timer 定时器指针，由系统自动传递
**出口参数:  无
**函数功能:  定时器回调函数，向控制模块发送电池状态作LED控制
*********************************************************************/
static void batt_timer_handler(struct k_timer *timer)
{
    MSG_S msg;  // 消息结构体，用于发送电池状态显示消息

    g_batt_led_ctrl.time_count++;  // 增加电池状态时间计数器

    msg.msgID = MY_MSG_SHOW_BATTERY;  // 设置消息ID为电池状态显示消息
    msg.DataLen = sizeof(g_batt_led_ctrl);  // 设置消息数据长度
    msg.pData = (void *)&g_batt_led_ctrl;  // 设置消息数据指针为电池LED控制结构体地址

    /*
    定时器发送消息到CTRL线程中处理控制LED,如果线程阻塞，会导致LED控制出现异常
    后续需要考虑是否直接在中断中处理，避免线程阻塞
    */
    my_send_msg_data(MOD_CTRL, MOD_CTRL, &msg);  // 发送消息到控制模块
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
    MY_LOG_INF("%s:%d", __func__, on);
    gpio_pin_set_dt(&batt_pwr_en, on ? 1 : 0);
}

/* 共用 ISR：根据 pins 掩码区分是 P1.7 还是 P1.8 触发 */
static void batt_gpio_isr(const struct device *dev,
                        struct gpio_callback *cb, 
                        uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    int level;

    if (pins & BIT(charge_state.pin))
    {
        /* P1.7 触发：电池状态变化（充电中 / 充满） */
        level = gpio_pin_get_dt(&charge_state);
        // MY_LOG_INF("charge_state:%d", level);
        //TODO 发消息给ctrl去再次获取该引脚电平状态，以及消抖处理
    }

    if (pins & BIT(charge_det.pin))
    {
        /* P1.8 触发：充电检测变化（开始充电 / 停止充电） */
        level = gpio_pin_get_dt(&charge_det);
        // MY_LOG_INF("charge_det:%d", level);
        //TODO 发消息给ctrl去再次获取该引脚电平状态，以及消抖处理
    }
}

int batt_gpio_init(void)
{
    int ret;

    if (!device_is_ready(batt_pwr_en.port) ||
        !device_is_ready(charge_state.port) ||
        !device_is_ready(charge_det.port))
    {
        return -ENODEV;
    }

    /* 使能口：输出，默认关闭 */
    ret = gpio_pin_configure_dt(&batt_pwr_en, GPIO_OUTPUT_INACTIVE);
    if (ret) return ret;

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

    // 初始化电池状态LED控制定时器
    k_timer_init(g_batt_led_ctrl.timer, batt_timer_handler, NULL);

    return 0;
}

/*********************************************************************
**函数名称:  my_battery_updata_state
**入口参数:  无
**出口参数:  无
**函数功能:  检查并更新电池状态
*********************************************************************/
void my_battery_updata_state()
{
    //TODO 更加ADC值检测电压，根据电压判断电池电量,更新电池状态到g_batt_led_ctrl.state

    switch(g_batt_led_ctrl.state)
    {
        case BATT_EMPTY:
            MY_LOG_INF("battery empty");  // 电池电量为空
            break;
        case BATT_LOW:
            MY_LOG_INF("battery low");    // 电池电量低
            break;
        case BATT_NORMAL:
            MY_LOG_INF("battery normal"); // 电池电量正常
            break;
        case BATT_FAIR:
            MY_LOG_INF("battery fair");   // 电池电量良好
            break;
        case BATT_HIGH:
            MY_LOG_INF("battery high");  // 电池电量高
            break;
        case BATT_FULL:
            MY_LOG_INF("battery full");   // 电池电量满
            break;
        default:
            break;  // 处理未定义的状态
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

    my_battery_updata_state();  // 检查并输出当前电池状态

    //当按键重复按下时，重新开始定时器
    g_batt_led_ctrl.time_count = 0;  // 重置时间计数器
    // 启动电池状态LED控制定时器，立即执行一次，然后以 100ms 为周期重复执行
    k_timer_start(g_batt_led_ctrl.timer, K_MSEC(0), K_MSEC(BATT_TIMER_MS));
}
