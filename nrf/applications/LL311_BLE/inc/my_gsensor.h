/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        my_gsensor.h
**文件描述:        G-Sensor 管理模块头文件 (LSM6DSV16X)
**当前版本:        V1.0
**作    者:        Harrison Wu (wuyujiao@jimiiot.com)
**完成日期:        2026.01.15
*********************************************************************
** 功能描述:        1. 提供 LSM6DSV16X 操作接口
**                 2. 提供三轴数据读取与电源管理功能
*********************************************************************/

#ifndef _MY_GSENSOR_H_
#define _MY_GSENSOR_H_

/* 智能模式时间间隔定义（单位：秒） */
#define STATIC_INTERVAL         (24 * 60 * 60)   // 静止状态：24小时

#define SHOCKALARM_TAP_STEP_THRESHOLD       4           // 敲击步进阈值
#define GSENSOR_WAKEUP_DURATION             2           // 唤醒持续时间（2=3个ODR周期@15Hz≈200ms，过滤瞬态噪声）
#define GSENSOR_INT_DEBOUNCE_MS             50          // INT1 唤醒中断消抖时间，避免电平抖动导致重复触发

// 传感器相关宏定义（基于LSM6DSVD文档特性）
#define LSM6DSVD_ACC_SENSITIVITY            0.061f     // 灵敏度 0.061 mg/LSB（±2g时，文档Table 3）
#define LSM6DSVD_GYRO_SENSITIVITY           4.375f     // 灵敏度 4.375 mdps/LSB（±125dps量程，分辨率加倍）

#define WINDOW_SIZE                         250         // 滑动窗口大小（约0.42秒数据，120Hz×50≈0.417s）

/* 智能模式状态枚举 */
typedef enum
{
    STATE_UNKNOWN = 0,      // 未知状态
    STATE_STATIC,           // 静止状态
    STATE_LAND_TRANSPORT,   // 陆运状态
    STATE_SEA_TRANSPORT,    // 海运状态
} gsensor_state_t;

/* 三轴数据结构体 */
typedef struct {
    int16_t acc_raw_x;
    int16_t acc_raw_y;
    int16_t acc_raw_z;
    int16_t gyro_raw_x;
    int16_t gyro_raw_y;
    int16_t gyro_raw_z;
} gsensor_data_t;

/* ============================================================
 *  IMU原始数据结构: 单个采样点的6轴数据
 * ============================================================ */
typedef struct {
    float acc_x;
    float acc_y;
    float acc_z;                  /* 三轴加速度 (m/s^2) */
    float gyro_x;
    float gyro_y;
    float gyro_z;               /* 三轴角速度 (rad/s) */
} imu_reading_t;

/* GSENSOR 运行时上下文结构体 */
typedef struct
{
    /* 滑动窗口相关 */
    imu_reading_t imu_readings[WINDOW_SIZE];
    uint16_t window_index;                       // 当前窗口写入索引

    /* 传感器状态 */
    bool sensor_ready;                          // 传感器是否已初始化并就绪

    /* 运动状态判定相关 */
    uint32_t sample_count;                      // 累计采样次数，用于判断滑动窗口是否已填满
    bool window_ready;                          // 滑动窗口是否已满，满后才能进行状态判断
    gsensor_state_t last_gsensor_state;         // 上次上报的运动状态，用于检测状态变化避免重复上报
    gsensor_state_t current_gsensor_state;      // 当前运动状态（静止/陆运/海运/未知）
} gsensor_runtime_ctx_t;

extern gsensor_runtime_ctx_t g_gsensor_runtime_ctx;

/********************************************************************
**函数名称:  my_gsensor_init
**入口参数:  tid      ---        指向线程 ID 变量的指针
**出口参数:  tid      ---        存储启动后的线程 ID
**函数功能:  初始化 G-Sensor 相关的 I2C 设备与 GPIO 引脚，并启动 G-Sensor 线程
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
int my_gsensor_init(k_tid_t *tid);

/********************************************************************
**函数名称:  my_gsensor_pwr_on
**入口参数:  on       ---        true 开启，false 关闭
**出口参数:  无
**函数功能:  控制 G-Sensor 的电源 (P2.6)
**返 回 值:  0 表示成功
*********************************************************************/
int my_gsensor_pwr_on(bool on);

/********************************************************************
**函数名称:  my_gsensor_read_data
**入口参数:  无
**出口参数:  data     ---        存储读取到的三轴数据
**函数功能:  读取当前加速度传感器的三轴原始数据
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
int my_gsensor_read_data(gsensor_data_t *data);

/********************************************************************
**函数名称:  my_lsm6dsv16x_init
**入口参数:  无
**出口参数:  无
**函数功能:  初始化 LSM6DSV16X 传感器设备
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
int my_lsm6dsv16x_init(void);

/********************************************************************
**函数名称:  lsm6dsv16x_check_id
**入口参数:  无
**出口参数:  无
**函数功能:  检查 LSM6DSV16X 是否在线
**返 回 值:  true 表示识别成功
*********************************************************************/
bool lsm6dsv16x_check_id(void);

/********************************************************************
**函数名称:  get_chip_id
**入口参数:  无
**出口参数:  无
**函数功能:  获取 LSM6DSV16X 芯片ID
**返 回 值:  LSM6DSV16X 芯片ID
*********************************************************************/
uint8_t get_chip_id(void);

/********************************************************************
**函数名称:  get_motion_status
**入口参数:  无
**出口参数:  无
**函数功能:  获取当前运动状态，根据状态设置LTE电源定时器间隔
**返 回 值:  无
*********************************************************************/
void get_motion_status(void);


#endif /* _MY_GSENSOR_H_ */
