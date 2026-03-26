/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        my_ctrl.h
**文件描述:        系统控制模块头文件 (LED, Buzzer, Key)
**当前版本:        V1.0
**作    者:        Harrison Wu (wuyujiao@jimiiot.com)
**完成日期:        2026.01.15
*********************************************************************
** 功能描述:        1. 整合 LED 指示灯控制
**                 2. 整合蜂鸣器 PWM 驱动
**                 3. 实现 FUN_KEY (P1.09) 按键短按/长按检测
**                 4. 实现光感、剪线检测中断接口
**                 5. 提供独立线程处理控制任务
*********************************************************************/

#ifndef _MY_CTRL_H_
#define _MY_CTRL_H_

/* --- 蜂鸣器相关定义 --- */
struct my_buzzer_note
{
    uint32_t freq_hz;     /* 频率 (Hz) */
    uint32_t duration_ms; /* 持续时间 (ms) */
};

typedef enum
{
    /* 指令控制类型 (BUZZER 指令) */
    BUZZER_STOP = 0,             // 0: 停止蜂鸣器
    BUZZER_CONTINUOUS_ALARM,     // 1: 持续报警 (200ms ON, 500ms OFF, 不停止)
    BUZZER_UNLOCK_SUCCESS,       // 2: 成功提示音 (500ms ON)/解锁成功提示: 长鸣 500ms
    BUZZER_FAIL_TONE,            // 3: 失败提示音 (200ms ON, 200ms OFF, 响3声)
    BUZZER_ERROR_TONE,           // 4: 异常提示音 (100ms ON, 100ms OFF, 持续1s)
    BUZZER_GENERAL_ALARM,        // 5: 一般报警音 (200ms ON, 300ms OFF, 持续30s)

    BUZZER_EVENT_LOCK_SUCCESS = 6,          // 上锁成功提示: 短鸣2次, 每次100ms, 间隔300ms
    BUZZER_EVENT_LOCK_FAIL,             // 上锁/解锁失败提示(滑块异常): 3次长鸣, 每次1000ms, 间隔500ms
    BUZZER_EVENT_UNAUTHORIZED,          // 未授权提示: NFC/蓝牙上锁/解锁未授权, 5次短鸣, 每次100ms, 间隔100ms
    BUZZER_EVENT_NFC_ACTIVATE,          // NFC激活提示: 蜂鸣器提示 100ms
} MY_BUZZER_MODE;

extern int g_buzzer_mode;

typedef struct {
    int tick;          // 当前计数（单位：100ms）
    int on_time;       // 响多久（单位：tick）
    int off_time;      // 停多久
    int repeat;        // 剩余次数（0表示无限;>0指定次数）
    uint8_t state;     // 0=关，1=开
} BUZZER_Ctrl_S;

typedef enum
{
    CLOSE_LED,   /* 关闭 LED */
    OPEN_LED,    /* 打开 LED */
    TOGGLE_LED,  /* 切换 LED 状态 */
} MY_LED_CTRL_CMD;

//定义了电池相关 LED ID，用于标识不同的 LED 指示灯
typedef enum
{
    BATT_LED1,  /**< 电池 LED 1 */
    BATT_LED2,  /**< 电池 LED 2 */
    BATT_LED3,  /**< 电池 LED 3 */
} MY_LED_ID;

typedef enum
{
    LOCK_LED_CLOSE,              /**< 关闭锁 LED 模式 */
    LOCK_LED_NFC_START,          /**< NFC 启动模式，LED 以 200ms 亮、500ms 灭的频率闪烁*/
    LOCK_LED_LOCKED,   /**< 解锁中和上锁中模式，LED 以 200ms 亮、200ms 灭的频率持续闪烁 */
    LOCK_LED_UNLOCK,             /**< 已解锁模式，LED 以 500ms 亮、1000ms 灭的频率闪烁，持续 18 秒 */
} MY_LOCK_LED_MODE;

/* --- 接口函数 --- */

/********************************************************************
**函数名称:  my_ctrl_init
**入口参数:  tid      ---   指向线程 ID 变量的指针
**出口参数:  tid      ---   存储启动后的线程 ID
**函数功能:  初始化控制模块并启动控制线程
**返 回 值:  0 表示成功，负值表示失败
**功能描述:  1. 初始化按键、光感、剪线 IO 中断
**           2. 初始化 LED、蜂鸣器、电机、电池 ADC
**           3. 启动控制线程
*********************************************************************/
int my_ctrl_init(k_tid_t *tid);

/********************************************************************
**函数名称:  my_ctrl_buzzer_play_tone
**入口参数:  freq_hz       ---   频率 (Hz)
**           duration_ms   ---   持续时间 (ms)
**出口参数:  无
**函数功能:  播放一个指定频率和时长的单音
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
int my_ctrl_buzzer_play_tone(uint32_t freq_hz, uint32_t duration_ms);

/********************************************************************
**函数名称:  my_ctrl_buzzer_play_sequence
**入口参数:  notes         ---   音符数组指针
**           num_notes     ---   音符数量
**出口参数:  无
**函数功能:  播放一组音符序列
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
int my_ctrl_buzzer_play_sequence(const struct my_buzzer_note *notes, uint32_t num_notes);

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
int batt_led_set_level(uint8_t level);

/********************************************************************
**函数名称:  handle_nfc_card_event
**入口参数:  card_id    ---        输入，NFC卡号指针
            id_len     ---        输入，卡号长度
**出口参数:  无
**函数功能:  处理NFC刷卡事件，检查重复刷卡、验证卡片权限并执行相应操作
**返 回 值:  无
*********************************************************************/
void handle_nfc_card_event(uint8_t *card_id, uint8_t id_len);

/*********************************************************************
**函数名称:  my_lock_led_set_mode
**入口参数:  mode  ---        LED 显示模式，使用 MY_LOCK_LED_MODE 枚举类型
**出口参数:  无
**函数功能:  该函数根据传入的模式参数，设置锁 LED 的不同显示模式，
**          包括关闭、NFC启动、解锁中和上锁中和已解锁模式。
*********************************************************************/
void my_lock_led_set_mode(MY_LOCK_LED_MODE mode);

#endif /* _MY_CTRL_H_ */
