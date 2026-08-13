/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        my_zms_param.h
**文件描述:        参数存储头文件
**当前版本:        V1.0
**作    者:        周森达 (zhousenda@jimiiot.com)
**完成日期:        2026.02.27
*********************************************************************/

#ifndef _MY_ZMS_PARAM_H_
#define _MY_ZMS_PARAM_H_

#define LICENSE_FF_STR_LEN                  (29 * 2)
#define LICENSE_GG_STR_LEN                  (24 * 2)

#define FLAG_VALID 0xAA

#define GSM_SN_LENGTH                       12
#define DEV_NAME_USE_SN_POS                 10
#define MY_MAC_LENGTH                       6

/* 蓝牙日志模块ID定义 */
#define BLE_LOG_MOD_MAIN     0 /* main模块 */
#define BLE_LOG_MOD_BLE      1 /* 蓝牙模块 */
#define BLE_LOG_MOD_DFU      2 /* DFU模块 */
#define BLE_LOG_MOD_SENSOR   3 /* 传感器模块 */
#define BLE_LOG_MOD_LTE      4 /* LTE模块 */
#define BLE_LOG_MOD_CTRL     5 /* 控制模块 */
#define BLE_LOG_MOD_SHELL    6 /* Shell模块 */
#define BLE_LOG_MOD_BATTERY  7 /* 电池模块 */
#define BLE_LOG_MOD_CMD      8 /* 命令设置模块 */
#define BLE_LOG_MOD_TOOL     9 /* 工具模块 */
#define BLE_LOG_MOD_PARAM   10 /* 参数模块 */
#define BLE_LOG_MOD_WDT     11 /* 看门狗模块 */
#define BLE_LOG_MOD_ALGORITHM 12 /* 算法模块 */
#define BLE_LOG_MOD_MAGNETIC_UART 13 /* 磁吸串口模块 */
#define BLE_LOG_MOD_OTHER   14 /* 其他模块 */
#define BLE_LOG_MOD_MAX     15 /* 最大模块数 */

/* 获取指定模块在 mod_en bitmap 中的开关状态
 * 使用32位bitmap，mod_id 直接对应位位置 (0-31) */
#define BLE_LOG_MOD_IS_ENABLED(config, mod_id) \
    ((mod_id) < 32 ? ((config)->mod_en & (1U << (mod_id))) : 0)

typedef enum                           // 参数ID定义
{
    ZMS_ID_FF = 0,                     // FF参数ID
    ZMS_ID_GG,                         // GG参数ID
    ZMS_ID_ADV_VALID,                  // 广播有效值参数ID
    ZMS_ID_ECDH_G,                     // ECDH_G参数ID
    ZMS_ID_SN,                         // 设备序列号SN参数ID
    ZMS_ID_MAC,                        // 设备MAC地址参数ID
    ZMS_ID_BLE_TX_POWER,               // 蓝牙发射功率参数ID
    ZMS_ID_BLE_LOG_CONFIG,             // 蓝牙日志配置参数ID
    ZMS_ID_WORK_MODE_CONFIG,           // 设备工作模式配置参数ID
    ZMS_ID_REM_ALM_CONFIG,             // 防拆壳报警配置参数ID
    ZMS_ID_PULL_ALM_CONFIG,            // 防拆卸报警配置参数ID
    ZMS_ID_PATMALM_CONFIG,             // 气压报警配置参数ID
    ZMS_ID_TEMPALM_CONFIG,             // 温湿度报警配置参数ID
    ZMS_ID_MOT_DET_CONFIG,             // 运动检测配置参数ID
    ZMS_ID_MOTDETALM_CONFIG,           // 运动检测报警报警配置参数ID
    ZMS_ID_BAT_LEVEL_CONFIG,           // 电池状态和充电状态报警配置参数ID
    ZMS_ID_STARTR_CONFIG,              // 数据记录功能配置参数ID
    ZMS_ID_PWRLIMIT_CONFIG,            // 限制按键关机配置参数ID
    ZMS_ID_BT_UPDATA_CONFIG,           // 蓝牙数据上传配置参数ID
    ZMS_ID_BLUETOOTH_CONFIG,           // 蓝牙开启配置参数ID
    ZMS_ID_BTCONNECT_CONFIG,           // 蓝牙连接配置参数ID
    ZMS_ID_TAG_CONFIG,                 // Tag定位功能配置参数ID
    ZMS_ID_LED_CONFIG,                 // LED显示配置参数ID
    ZMS_ID_LTINT_CONFIG,               // 光感过滤配置参数ID
    ZMS_ID_BUZZER_CONFIG,              // 蜂鸣器配置参数ID
    ZMS_ID_BT_PARMAC_CONFIG,           // 透传MAC地址配置参数ID
    ZMS_ID_LPSLEEP_CONFIG,             // 低功耗运行配置参数ID
    ZMS_ID_BLE_TAG_STORE_META,         // BLE TAG扫描数据循环存储区元数据ID
    ZMS_ID_BLE_MAC_STORE_META,         // BLE 透传MAC扫描数据循环存储区元数据ID
    ZMS_ID_PATM_TIMER_CONFIG,          // 气压定时上传配置参数ID
    ZMS_ID_TEMP_TIMER_CONFIG,          // 温湿度定时上传配置参数ID
    ZMS_ID_IMU_ALM_CONFIG,             // IMU翻转报警配置参数ID
    ZMS_ID_IMU_ZERO_BIAS_CONFIG,       // IMU零偏配置参数ID
    ZMS_ID_BLE_TH_STORE_META,          // BLE 温湿度循环存储区元数据ID
    ZMS_ID_BLE_BP_STORE_META,          // BLE 气压循环存储区元数据ID
} my_zms_id_t;

typedef struct                              // 存储的LICENSE GG信息
{
    uint8_t flag;                           // 参数有效标志
    uint8_t hex[(LICENSE_GG_STR_LEN / 2) + (LICENSE_GG_STR_LEN % 2)]; // GG参数值
} lic_gg_t;

typedef struct                              // 存储的LICENSE FF信息
{
    uint8_t flag;                           // 参数有效标志
    uint8_t hex[(LICENSE_FF_STR_LEN / 2) + (LICENSE_FF_STR_LEN % 2)]; // FF参数值
} lic_ff_t;

typedef struct                              // 广播有效值参数
{
    uint8_t flag;                           // 参数有效标志
    uint8_t AppleValid;                     // Apple有效值
    uint8_t GoogleValid;                    // Google有效值
} adv_valid_value_t;

typedef struct                              // 存储的ECDH_G信息
{
    uint8_t flag;                           // 参数有效标志
    uint16_t ecdh_g; // ECDH_G参数值
} ecdh_g_t;

typedef struct                              // 存储的SN信息
{
    uint8_t flag;                           // 参数有效标志
    uint8_t hex[GSM_SN_LENGTH];             // 设备序列号SN
} gsm_sn_t;

typedef struct                              // 存储的MAC地址信息
{
    uint8_t flag;                           // 参数有效标志
    uint8_t hex[MY_MAC_LENGTH];             // 设备MAC地址
} macaddr_t;

typedef struct                              // 存储的蓝牙发射功率参数
{
    uint8_t flag;                           // 参数有效标志
    int8_t tx_power;                        // 发射功率(dBm)，范围: -10 ~ +7(NRF54L15 QFN封装)
} ble_tx_power_t;

typedef struct                              // 存储的蓝牙日志配置参数
{
    uint8_t  flag;                          // 参数有效标志
    uint8_t  global_en;                     // 总开关: 0=关闭, 1=开启
    uint8_t  reserved[2];                   // 预留对齐，确保mod_en 4字节对齐
    uint32_t mod_en;                        // 模块开关bitmap，每位对应一个模块
    uint8_t  mod_level[BLE_LOG_MOD_MAX];    // 各模块日志等级阈值
} ble_log_config_t;

typedef struct                              // 存储的设备工作模式配置参数
{
    uint8_t flag;                           // 参数有效标志
    device_work_mode_config_t workmode_config;   // 设备工作模式配置结构体
} work_mode_config_t;

typedef struct                              // 存储的防拆报警配置参数
{
    uint8_t flag;                           // 参数有效标志
    uint8_t remalm_sw;                      // 防拆报警开关: 0-OFF, 1-ON
    uint8_t remalm_mode;                    // 报警上报方式: 0-不上报, 1-GPRS, 2-GPRS+SMS, 3-GPRS+SMS+CALL
} remalm_config_t;

typedef struct                              // 存储的防拆卸报警配置参数
{
    uint8_t flag;                           // 参数有效标志
    uint8_t pullalm_sw;                     // 防拆卸报警开关: 0-OFF, 1-ON
    uint8_t pullalm_mode;                   // 报警上报方式: 0-不上报, 1-GPRS, 2-GPRS+SMS, 3-GPRS+SMS+CALL
} pullalm_config_t;

typedef struct                              // 存储的气压报警配置参数
{
    uint8_t flag;                          // 参数有效标志
    uint8_t patalm_sw;                     // 气压报警开关: 0-OFF, 1-ON
    uint8_t patalm_low_threshold;          // 低压报警阈值: 30-250 (单位：kPa,设置为255表示低压不报警)
    uint8_t patalm_high_threshold;         // 高压报警阈值: 30-250 (单位：kPa,设置为255表示高压不报警)
    uint8_t patalm_report_type;            // 气压报警上报方式: 0-不上报, 1-GPRS, 2-GPRS+SMS, 3-GPRS+SMS+CALL
    uint8_t patalm_report_interval;        // 重复气压报警上报时间间隔: 0-60 (单位：分钟,设置为0表示不重复上报)
} patalm_config_t;

typedef struct                              // 存储的温湿度报警配置参数
{
    uint8_t flag;                           // 参数有效标志
    uint8_t tempalm_sw;                     // 温报警开关: 0-OFF, 1-ON
    int temp_low_threshold;                 // 低温报警阈值: -30 - 100 (单位：℃,设置为255表示低温不报警)
    int temp_high_threshold;                // 高温报警阈值: -30 - 100 (单位：℃,设置为255表示高温不报警)
    uint8_t humi_low_threshold;             // 低湿度报警阈值: 0-100 (单位：%,设置为255表示低湿度不报警)
    uint8_t humi_high_threshold;            // 高湿度报警阈值: 0-100 (单位：%,设置为255表示高湿度不报警)
    uint8_t tempalm_report_type;            // 温湿度报警上报方式: 0-不上报, 1-GPRS, 2-GPRS+SMS, 3-GPRS+SMS+CALL
    uint8_t tempalm_report_interval;        // 重复温湿度报警上报时间间隔: 0-60 (单位：分钟,设置为0表示不重复上报)
} tempalm_config_t;

typedef struct                              // 存储的运动检测报警配置参数
{
    uint8_t flag;                           // 参数有效标志
    uint16_t motdet_vibration;              // 运动检测震动次数 (1-500)
    uint16_t motdet_duration;               // 运动检测判断时间 (1-3600s)
} mot_det_config_t;

typedef struct                              // 存储的运动检测报警报警配置参数
{
    uint8_t flag;                           // 参数有效标志
    uint8_t motdetalm_sw;                   // 运动检测报警报警开关: 0-OFF, 1-ON
    uint8_t motdetalm_report_type;            // 报警上报方式: 0-不上报, 1-GPRS, 2-GPRS+SMS, 3-GPRS+SMS+CALL
} motdetalm_config_t;

typedef struct                              // 存储的电池状态和充电状态报警配置参数
{
    uint8_t flag;                           // 参数有效标志
    uint8_t batlevel_empty_rpt;             // Empty状态上报方式:0-不上报, 1-GPRS, 2-GPRS+SMS, 3-GPRS+SMS+CALL
    uint8_t batlevel_low_rpt;               // Low状态上报方式
    uint8_t batlevel_normal_rpt;            // Normal状态上报方式
    uint8_t batlevel_fair_rpt;              // Fair状态上报方式
    uint8_t batlevel_high_rpt;              // High状态上报方式
    uint8_t batlevel_full_rpt;              // Full状态上报方式
    uint8_t chargesta_report;               // 充电状态上报方式: 0-不上报, 1-GPRS, 2-GPRS+SMS, 3-GPRS+SMS+CALL
} bat_level_config_t;

typedef struct                              // 存储的数据记录功能配置参数
{
    uint8_t flag;                           // 参数有效标志
    uint8_t startr_sw;                      // 数据记录功能开关: 0-OFF, 1-ON
} startr_config_t;

typedef struct                              // 存储的限制按键关机配置参数
{
    uint8_t flag;                           // 参数有效标志
    uint8_t pwrlimit_sw;                    // 限制按键关机开关: 0-OFF, 1-ON
} pwrlimit_config_t;

typedef struct                              // 存储的低功耗运行配置参数
{
    uint8_t flag;                           // 参数有效标志
    uint8_t lprunning_sw;                   // 功能开关: 0-OFF, 1-ON
    uint8_t lprunning_threshold;            // 进入低功耗运行的电量百分比阈值 (10~50, 默认20)
    uint8_t lprunning_interval;             // 定时唤醒间隔T (1~48小时, 默认24)
} lprunning_config_t;

typedef struct                              // 存储的蓝牙数据上传配置参数
{
    uint8_t flag;                           // 参数有效标志
    uint8_t  bt_updata_mode;                // 工作方式: 0-不开启, 1-持续收集Cell启动上传, 2-持续收集Cell启动上传或唤醒上传
    uint32_t bt_updata_scan_interval;       // 蓝牙数据收集间隔: 5-86400秒
    uint32_t bt_updata_scan_length;         // 每次收集搜索时长: 5-86400秒
    uint32_t bt_updata_updata_interval;     // 蓝牙唤醒间隔: 120-86400秒
} bt_updata_config_t;

typedef struct                              // 存储的蓝牙开启配置参数
{
    uint8_t flag;                           // 参数有效标志
    uint8_t bluetooth_sw;                   // 蓝牙开启开关: 0-OFF, 1-ON
    uint8_t bluetooth_a;                    // 蓝牙开启A参数: 1~5  开启广播条件 现在只能设置5
    uint8_t bluetooth_b;                    // 蓝牙开启B参数: 0~30min 开启广播时间间隔
    uint8_t bluetooth_flag;                 // 指令是否携带参数: 0-未携带, 1-携带
} bluetooth_config_t;

typedef struct                              // 存储的蓝牙连接配置参数
{
    uint8_t flag;                           // 参数有效标志
    uint8_t btconnect_sw;                   // 蓝牙连接开关: 0-OFF, 1-ON
    uint16_t btconnect_interval;            // 连接间隔: 300-43200s
    uint8_t btconnect_report;               // 连接上报方式: 0-不上报, 1-GPRS, 2-GPRS+SMS, 3-GPRS+SMS+CALL
} btconnect_config_t;

typedef struct                              // 存储的Tag定位功能配置参数
{
    uint8_t flag;                           // 参数有效标志
    uint8_t tag_sw;                         // Tag定位功能开关: 0-OFF, 1-ON
    uint16_t tag_interval;                  // 广播间隔: 100-60000ms
} tag_config_t;

typedef struct                              // 存储的LED显示配置参数
{
    uint8_t flag;                           // 参数有效标志
    uint8_t led_display;                    // LED显示模式: 0-一直关闭, 1-按键显示, 2-全时显示
} led_config_t;

typedef struct                              // 存储的光感过滤配置参数
{
    uint8_t flag;                           // 参数有效标志
    uint16_t T1;                            // 检测到光的连续时间超过T1时，切换为“Light”状态 100~5000ms
    uint16_t T2;                            // 检测到暗的连续时间超过T2时，切换为“Dark”状态 100~5000ms
} ltint_config_t;

typedef struct                              // 存储的蜂鸣器配置参数
{
    uint8_t flag;                           // 参数有效标志
    uint8_t buzzer_operator;                // 蜂鸣器操作: 0-停止, 1-持续报警, 2-成功提示音, 3-失败提示音, 4-异常提示音, 5-一般报警音
} buzzer_config_t;

typedef struct                              // 存储的透传mac地址配置数据
{
    uint8_t flag;                                   // 参数有效标志
    bt_addr_le_t bt_parmac_macs[TRAN_MAC_MAX_NUM];  // 透传MAC地址列表，最多20个
    uint8_t      bt_parmac_mac_count;               // 已配置MAC数量
} bparmac_config_t;

typedef struct                              // 存储的气压定时上传配置参数
{
    uint8_t flag;                           // 参数有效标志
    uint16_t interval_min;                  // 上传间隔(分钟): 0或10~1440
    uint8_t wakeup_cell_sw;                 // 4G离线时是否唤醒: 0-OFF, 1-ON
} patm_timer_config_t;

typedef struct                              // 存储的温湿度定时上传配置参数
{
    uint8_t flag;                           // 参数有效标志
    uint16_t interval_min;                  // 上传间隔(分钟): 0或10~1440
    uint8_t wakeup_cell_sw;                 // 4G离线时是否唤醒: 0-OFF, 1-ON
} temp_timer_config_t;

typedef struct                              // 存储的IMU翻转报警配置参数
{
    uint8_t flag;                           // 参数有效标志
    uint8_t imu_alm_sw;                     // IMU翻转报警开关: 0-OFF, 1-ON
    uint8_t imu_alm_report;                 // IMU翻转报警上报方式: 0-不上报, 1-GPRS, 2-GPRS+SMS, 3-GPRS+SMS+CALL
    uint8_t imu_roll_threshold;             // IMU横滚角报警阈值: 5~60度，若设置255则关闭该维度的角度检测
    uint8_t imu_pitch_threshold;            // IMU俯仰角报警阈值: 5~60度，若设置255则关闭该维度的角度检测
    uint8_t imu_yaw_threshold;              // IMU偏航角报警阈值: 5~60度，若设置255则关闭该维度的角度检测
    uint8_t imu_duration_time;              // IMU超过阈值持续时间: 1~180s
    uint8_t imu_duration_count;             // IMU超过阈值持续次数: 1~10次,预留接口
    uint8_t recover_time;                   // 恢复时间: 1~30s
} imu_alm_config_t;

typedef struct                              // 存储的IMU零偏配置参数
{
    uint8_t flag;                           // 参数有效标志
    float gyro_bias_x;                      // 陀螺仪零偏估计 X (rad/s, 在线逐步追踪)
    float gyro_bias_y;                      // 陀螺仪零偏估计 Y (rad/s)
    float gyro_bias_z;                      // 陀螺仪零偏估计 Z (rad/s)
} imu_zero_bias_config_t;

typedef struct
{
    lic_ff_t                    lic_ff;                     // 存储的LICENSE FF信息
    lic_gg_t                    lic_gg;                     // 存储的LICENSE GG信息
    adv_valid_value_t           adv_valid_value;            // 广播有效值
    ecdh_g_t                    ECDH_GValue;                // ECDH_GValue值
    gsm_sn_t                    gsm_sn;                     // 设备序列号SN
    macaddr_t                   my_macaddr;                 // 设备MAC地址
    ble_tx_power_t              ble_tx_power;               // 蓝牙发射功率
    ble_log_config_t            ble_log_config;             // 蓝牙日志配置
    work_mode_config_t          device_workmode_config;     // 设备工作模式配置
    remalm_config_t             remalm_config;              // 防拆壳报警配置
    pullalm_config_t            pullalm_config;             // 防拆卸报警配置
    patalm_config_t             patalm_config;              // 气压报警配置
    tempalm_config_t            tempalm_config;             // 温湿度报警配置
    mot_det_config_t            motdet_config;              // 运动检测报警配置
    motdetalm_config_t          motdetalm_config;           // 运动检测报警报警配置
    bat_level_config_t          batlevel_config;            // 电池状态和充电状态报警配置
    startr_config_t             startr_config;              // 数据记录功能配置
    pwrlimit_config_t           pwrlimit_config;            // 限制按键关机配置
    lprunning_config_t          lprunning_config;           // 低功耗运行配置
    bt_updata_config_t          bt_updata_config;           // 蓝牙数据上传配置
    bluetooth_config_t          bluetooth_config;           // 蓝牙开启配置
    btconnect_config_t          btconnect_config;           // 蓝牙连接配置
    tag_config_t                tag_config;                 // Tag定位功能配置
    led_config_t                led_config;                 // LED显示配置
    ltint_config_t              ltint_config;               // 光感过滤配置
    buzzer_config_t             buzzer_config;              // 蜂鸣器配置
    bparmac_config_t            bparmac_config;             // 透传mac地址配置
    patm_timer_config_t         patm_timer_config;          // 气压定时上传配置
    temp_timer_config_t         temp_timer_config;          // 温湿度定时上传配置
    imu_alm_config_t            imu_alm_config;             // IMU翻转报警配置
    imu_zero_bias_config_t      imu_zero_bias_config;       // IMU零偏配置
} config_param_t;

extern config_param_t    gConfigParam;

/********************************************************************
**函数名称:  my_param_load_config
**入口参数:  无
**出口参数:  无
**函数功能:  加载配置参数（从ZMS存储中读取所有配置数据）
**返 回 值:  无
*********************************************************************/
void my_param_load_config(void);
/********************************************************************
**函数名称:  my_param_set_ff
**入口参数:  param: 要设置的iOS许可证数据, len: 数据长度
**出口参数:  无
**函数功能:  设置iOS数据到flash中
**返 回 值:  true表示成功，false表示失败
*********************************************************************/
bool my_param_set_ff(char *param, uint8_t len);
/********************************************************************
**函数名称:  my_param_get_ff
**入口参数:  无
**出口参数:  无
**函数功能:  获取iOS配置数据
**返 回 值:  返回iOS许可证结构体指针
*********************************************************************/
const lic_ff_t *my_param_get_ff(void);
/********************************************************************
**函数名称:  my_param_set_gg
**入口参数:  param: 要设置的Google许可证数据, len: 数据长度
**出口参数:  无
**函数功能:  设置Google数据到flash中
**返 回 值:  true表示成功，false表示失败
*********************************************************************/
bool my_param_set_gg(char *param, uint8_t len);
/********************************************************************
**函数名称:  my_param_get_gg
**入口参数:  无
**出口参数:  无
**函数功能:  获取Google配置数据
**返 回 值:  返回Google许可证结构体指针
*********************************************************************/
const lic_gg_t *my_param_get_gg(void);
/********************************************************************
**函数名称:  my_param_set_jatag_or_jgtag
**入口参数:  cmd: 命令字符串, param: 参数字符串
**出口参数:  无
**函数功能:  设置哪一路广播数据开启或关闭(google或ios)
**返 回 值:  0表示成功，负值表示失败
*********************************************************************/
int my_param_set_jatag_or_jgtag(char *cmd, char *param);
/********************************************************************
**函数名称:  my_param_set_Gvalue
**入口参数:  param: 要设置的ECDH G值字符串
**出口参数:  无
**函数功能:  设置ECDH G值
**返 回 值:  0表示成功，负值表示失败
*********************************************************************/
int my_param_set_Gvalue(char *param);
/********************************************************************
**函数名称:  my_param_get_Gvalue
**入口参数:  无
**出口参数:  无
**函数功能:  获取ECDH G值
**返 回 值:  返回ECDH G值
*********************************************************************/
const uint16_t my_param_get_Gvalue(void);
/********************************************************************
**函数名称:  my_param_set_sn
**入口参数:  param: 要设置的设备序列号SN, len: 数据长度
**出口参数:  无
**函数功能:  设置设备序列号SN
**返 回 值:  0表示成功，负值表示失败
*********************************************************************/
int my_param_set_sn(char *param, uint8_t len);
/********************************************************************
**函数名称:  my_param_get_sn
**入口参数:  无
**出口参数:  无
**函数功能:  获取设备序列号SN配置数据
**返 回 值:  返回设备序列号SN结构体指针
*********************************************************************/
const gsm_sn_t *my_param_get_sn(void);
/********************************************************************
**函数名称:  my_param_set_mac
**入口参数:  param: 要设置的MAC地址, len: 数据长度
**出口参数:  无
**函数功能:  设置MAC地址
**返 回 值:  0表示成功，负值表示失败
*********************************************************************/
int my_param_set_mac(char *param, uint8_t len);
/********************************************************************
**函数名称:  my_param_get_macaddr
**入口参数:  无
**出口参数:  无
**函数功能:  获取mac addr配置数据
**返 回 值:  返回MAC地址结构体指针
*********************************************************************/
const macaddr_t *my_param_get_macaddr(void);
/********************************************************************
**函数名称:  my_param_get_ble_tx_power
**入口参数:  无
**出口参数:  无
**函数功能:  获取蓝牙发射功率参数
**返 回 值:  发射功率(dBm)，如果参数无效返回默认值0
*********************************************************************/
int8_t my_param_get_ble_tx_power(void);
/********************************************************************
**函数名称:  my_param_set_ble_log_config
**入口参数:  config: 蓝牙日志配置结构体指针
**出口参数:  无
**函数功能:  设置蓝牙日志完整配置
**返 回 值:  0表示成功，负值表示失败
*********************************************************************/
int my_param_set_ble_log_config(const ble_log_config_t *config);
/********************************************************************
**函数名称:  my_param_get_ble_log_config
**入口参数:  无
**出口参数:  无
**函数功能:  获取蓝牙日志配置
**返 回 值:  返回蓝牙日志配置结构体指针
*********************************************************************/
ble_log_config_t *my_param_get_ble_log_config(void);
/********************************************************************
**函数名称:  my_param_set_ble_log_global
**入口参数:  en: 总开关状态 (0=关闭, 1=开启)
**出口参数:  无
**函数功能:  设置蓝牙日志总开关
**返 回 值:  0表示成功，负值表示失败
*********************************************************************/
int my_param_set_ble_log_global(uint8_t en);
/********************************************************************
**函数名称:  my_param_set_ble_log_mod
**入口参数:  mod_id: 模块ID, en: 开关状态 (0=关闭, 1=开启)
**出口参数:  无
**函数功能:  设置指定模块的蓝牙日志开关
**返 回 值:  0表示成功，负值表示失败
*********************************************************************/
int my_param_set_ble_log_mod(uint8_t mod_id, uint8_t en);
/********************************************************************
**函数名称:  my_param_set_ble_log_level
**入口参数:  mod_id: 模块ID, level: 日志等级阈值
**出口参数:  无
**函数功能:  设置指定模块的蓝牙日志等级阈值
**返 回 值:  0表示成功，负值表示失败
*********************************************************************/
int my_param_set_ble_log_level(uint8_t mod_id, uint8_t level);

/********************************************************************
**函数名称:  my_user_data_write
**入口参数:  id: ZMS ID（32位）, data: 指向要写入的数据缓冲区, len: 数据长度（最大64 KiB）
**出口参数:  无
**函数功能:  通用写接口：按 ID 写入任意数据
**返 回 值:  >=0 写入的字节数；负值为错误码
*********************************************************************/
int my_user_data_write(uint32_t id, const void *data, int len);

/********************************************************************
**函数名称:  my_user_data_read
**入口参数:  id      ---        ZMS ID（32位）（输入）
**           data    ---        指向接收数据的缓冲区（输出）
**           len     ---        缓冲区最大长度（输入）
**出口参数:  data    ---        存储读取到的数据
**函数功能:  通用读接口：按 ID 读取任意数据
**返 回 值:  >0 实际读取的字节数；0 表示未找到该 ID；负值为错误码
*********************************************************************/
int my_user_data_read(uint32_t id, void *data, int len);

/********************************************************************
**函数名称:  my_param_check_license
**入口参数:  param: 要检查的许可证数据, len: 数据长度, id: 许可证ID
**出口参数:  无
**函数功能:  检查许可证数据是否有效
**返 回 值:  true表示有效，false表示无效
*********************************************************************/
bool my_param_check_license(char *param, uint8_t len, my_zms_id_t id);

/********************************************************************
**函数名称:  my_param_factory_reset
**入口参数:  无
**出口参数:  无
**函数功能:  重置所有参数为出厂值
*********************************************************************/
int my_param_factory_reset(void);

/********************************************************************
**函数名称:  my_param_log_config
**入口参数:  无
**出口参数:  无
**函数功能:  打印所有配置参数（格式：指令名:参数, 参数）
**返 回 值:  无
*********************************************************************/
void my_param_log_config(void);

#endif