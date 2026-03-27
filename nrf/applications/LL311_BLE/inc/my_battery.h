/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        my_battery.h
**文件描述:        电池管理模块实现头文件
**当前版本:        V1.0
**作    者:        周森达 (zhousenda@jimiiot.com)
**完成日期:        2026.02.04
*********************************************************************/

#ifndef _MY_BATTERY_H_
#define _MY_BATTERY_H_

// 电池状态枚举, 定义了电池的不同状态等级，从空到满
typedef enum
{
    BATT_EMPTY,   /* 电池电量为空 */
    BATT_LOW,     /* 电池电量低 */
    BATT_NORMAL,  /* 电池电量正常 */
    BATT_FAIR,    /* 电池电量良好 */
    BATT_HIGH,    /* 电池电量高 */
    BATT_FULL,    /* 电池电量满 */
} MY_BATT_STATE;

// 正常状态 LED 控制结构体, 用于控制电池状态 LED 显示的结构体
typedef struct
{
    struct k_timer *timer;       /* 定时器指针，用于控制 LED 显示的定时 */
    MY_BATT_STATE state;            /* 当前电池状态，使用枚举类型 */
    uint8_t time_count;          /* 时间计数器，用于控制 LED 闪烁和显示持续时间, 按键按下, LED 执行5s, 定时器设置为100ms , 50次*/
} Batt_LED_Ctrl_S;

// 电源状态枚举, 定义了设备的电源连接状态
typedef enum
{
    NO_CHARGING,    /* 充电器未连接 */
    CHARGING,       /* 充电器已连接（充电中） */
    CHARG_FULL,     /* 充电器已连接且电池已充满 */
} MY_CHG_STATE;

typedef enum
{
    CHG_BATT_LOW,    /* 充电状态电池电量低 */
    CHG_BATT_FAIR,   /* 充电状态电池电量良好 */
    CHG_BATT_HIGH,   /* 充电状态电池电量高 */
    CHG_BATT_FULL,   /* 充电状态电池电量满，电池已充满 */
}MY_CHG_BATT_STATE;

// 充电状态 LED 控制结构体, 用于控制电池状态 LED 显示的结构体
typedef struct
{
    struct k_timer *timer;       /* 定时器指针，用于控制 LED 显示的定时 */
    MY_CHG_BATT_STATE state;            /* 当前电池状态，使用枚举类型 */
    uint16_t time_count;          /* 时间计数器，用于控制 LED 闪烁和显示持续时间以及固定时间(10s)检测更新充电状态,定时器500ms循环*/
} CHG_LED_Ctrl_S;

//电池电压-电量映射结构体,用于存储电池电压值与对应电量百分比的映射关系,用于通过电池电压计算电量百分比.
typedef struct
{
    int32_t mv;       /**< 电池电压值，单位为毫伏 (mV) */
    int8_t percent;   /**< 对应的电池电量百分比，范围为 0-100 */
} Batt_Volt_Percent_Map_S;

// 正常状态LED控制结构体，包含定时器、电池状态和计数器
extern Batt_LED_Ctrl_S g_batt_led_ctrl;

// 充电状态LED控制结构体，包含定时器、充电电池状态和计数器
extern CHG_LED_Ctrl_S g_chg_led_ctrl;

int batt_read_mv(int32_t *mv);
int ntc_read_raw(int16_t *raw);
int batt_adc_init(void);
void batt_enable(bool on);
int batt_gpio_init(void);

/*********************************************************************
**函数名称:  show_battary
**入口参数:  无
**出口参数:  无
**函数功能:  显示电池状态
**返 回 值:  无
*********************************************************************/
void my_battery_show(void);

/*********************************************************************
**函数名称:  my_battery_update_state
**入口参数:  无
**出口参数:  无
**函数功能:  检查并更新电池状态
*********************************************************************/
void my_battery_update_state(void);

/*********************************************************************
**函数名称:  my_battery_show_chgled
**入口参数:  无
**出口参数:  无
**函数功能:  检查充电检测引脚的状态，根据检测结果更新充电状态并控制LED显示。
** 如果检测到充电，则启动充电LED控制定时器；如果未检测到充电，则关闭所有电池LED。
*********************************************************************/
void my_battery_show_chgled();

#endif