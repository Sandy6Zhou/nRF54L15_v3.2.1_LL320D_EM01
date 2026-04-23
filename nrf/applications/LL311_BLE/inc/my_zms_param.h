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

#define GSM_IMEI_LENGTH                     15
#define DEV_NAME_USE_IMEI_POS               10
#define MY_MAC_LENGTH                       6

/* 蓝牙日志模块ID定义 */
#define BLE_LOG_MOD_MAIN     0 /* main模块 */
#define BLE_LOG_MOD_BLE      1 /* 蓝牙模块 */
#define BLE_LOG_MOD_DFU      2 /* DFU模块 */
#define BLE_LOG_MOD_SENSOR   3 /* 传感器模块 */
#define BLE_LOG_MOD_LTE      4 /* LTE模块 */
#define BLE_LOG_MOD_CTRL     5 /* 控制模块 */
#define BLE_LOG_MOD_SHELL    6 /* Shell模块 */
#define BLE_LOG_MOD_NFC      7 /* NFC模块 */
#define BLE_LOG_MOD_BATTERY  8 /* 电池模块 */
#define BLE_LOG_MOD_MOTOR    9 /* 电机模块 */
#define BLE_LOG_MOD_CMD     10 /* 命令设置模块 */
#define BLE_LOG_MOD_TOOL    11 /* 工具模块 */
#define BLE_LOG_MOD_PARAM   12 /* 参数模块 */
#define BLE_LOG_MOD_WDT     13 /* 看门狗模块 */
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
    ZMS_ID_IMEI,                       // GSM IMEI参数ID
    ZMS_ID_MAC,                        // 设备MAC地址参数ID
    ZMS_ID_BLE_TX_POWER,               // 蓝牙发射功率参数ID
    ZMS_ID_BLE_LOG_CONFIG,             // 蓝牙日志配置参数ID
    ZMS_ID_WORK_MODE_CONFIG,           // 设备工作模式配置参数ID
    ZMS_ID_REM_ALM_CONFIG,             // 防拆报警配置参数ID
    ZMS_ID_LOCK_PIN_CYT_CONFIG,        // 锁销非法拔除报警配置参数ID
    ZMS_ID_LOCK_ERR_CONFIG,            // 锁状态异常报警配置参数ID
    ZMS_ID_PIN_STAT_CONFIG,            // 锁销状态报警配置参数ID
    ZMS_ID_LOCK_STAT_CONFIG,           // 锁状态报警配置参数ID
    ZMS_ID_MOT_DET_CONFIG,             // 运动检测报警配置参数ID
    ZMS_ID_BAT_LEVEL_CONFIG,           // 电池状态和充电状态报警配置参数ID
    ZMS_ID_SHOCK_ALARM_CONFIG,         // 撞击报警配置参数ID
    ZMS_ID_STARTR_CONFIG,              // 数据记录功能配置参数ID
    ZMS_ID_PWSAVE_CONFIG,              // 低功耗运输状态配置参数ID
    ZMS_ID_BT_UPDATA_CONFIG,           // 蓝牙数据上传配置参数ID
    ZMS_ID_TAG_CONFIG,                 // Tag定位功能配置参数ID
    ZMS_ID_LOCKED_CONFIG,              // 自动上锁配置参数ID
    ZMS_ID_LED_CONFIG,                 // LED显示配置参数ID
    ZMS_ID_BUZZER_CONFIG,              // 蜂鸣器配置参数ID
    ZMS_ID_NFTRIG_CONFIG,              // NFC触发规则配置参数ID
    ZMS_ID_NFCAUTH_CONFIG,             // NFC卡权限配置参数ID
    ZMS_ID_BT_KEY_CONFIG,              // 蓝牙解锁密钥配置参数ID
    ZMS_ID_OTA_CONFIG,                 // OTA升级相关配置参数ID
    ZMS_ID_BT_PARMAC_CONFIG,           // 透传MAC地址配置参数ID
} ZMS_ID;

typedef struct                              // 存储的LICENSE GG信息
{
    uint8_t flag;                           // 参数有效标志
    uint8_t hex[(LICENSE_GG_STR_LEN / 2) + (LICENSE_GG_STR_LEN % 2)]; // GG参数值
} lic_gg_struct;

typedef struct                              // 存储的LICENSE FF信息
{
    uint8_t flag;                           // 参数有效标志
    uint8_t hex[(LICENSE_FF_STR_LEN / 2) + (LICENSE_FF_STR_LEN % 2)]; // FF参数值
} lic_ff_struct;

typedef struct                              // 广播有效值参数
{
    uint8_t flag;                           // 参数有效标志
    uint8_t AppleValid;                     // Apple有效值
    uint8_t GoogleValid;                    // Google有效值
} AdvValidValue_t;

typedef struct                              // 存储的IMEI信息
{
    uint8_t flag;                           // 参数有效标志
    uint8_t hex[GSM_IMEI_LENGTH];           // GSM IMEI
} GsmImei_t;

typedef struct                              // 存储的MAC地址信息
{
    uint8_t flag;                           // 参数有效标志
    uint8_t hex[MY_MAC_LENGTH];             // 设备MAC地址
} macaddr_t;

typedef struct                              // 存储的蓝牙发射功率参数
{
    uint8_t flag;                           // 参数有效标志
    int8_t tx_power;                        // 发射功率(dBm)，范围: -10 ~ +7(NRF54L15 QFN封装)
} BleTxPower_t;

typedef struct                              // 存储的蓝牙日志配置参数
{
    uint8_t  flag;                          // 参数有效标志
    uint8_t  global_en;                     // 总开关: 0=关闭, 1=开启
    uint8_t  reserved[2];                   // 预留对齐，确保mod_en 4字节对齐
    uint32_t mod_en;                        // 模块开关bitmap，每位对应一个模块
    uint8_t  mod_level[BLE_LOG_MOD_MAX];    // 各模块日志等级阈值
} BleLogConfig_t;

typedef struct                              // 存储的设备工作模式配置参数
{
    uint8_t flag;                           // 参数有效标志
    DeviceWorkModeConfig workmode_config;   // 设备工作模式配置结构体
} WorkModeConfig_t;

typedef struct                              // 存储的防拆报警配置参数
{
    uint8_t flag;                           // 参数有效标志
    uint8_t remalm_sw;                      // 防拆报警开关: 0-OFF, 1-ON
    uint8_t remalm_mode;                    // 报警上报方式: 0-GPRS, 1-GPRS+SMS, 2-GPRS+SMS+CALL
} RemAlmConfig_t;

typedef struct                              // 存储的锁销非法拔除报警配置参数
{
    uint8_t flag;                           // 参数有效标志
    uint8_t lockpincyt_report;              // 锁销非法拔除上报方式: 0-不上报, 1-GPRS, 2-GPRS+SMS, 3-GPRS+SMS+CALL
    uint8_t lockpincyt_buzzer;              // 锁销非法拔除蜂鸣器报警方式: 0-不报警, 1-报警30s, 2-持续报警
} LockPinCytConfig_t;

typedef struct                              // 存储的锁状态异常报警配置参数
{
    uint8_t flag;                           // 参数有效标志
    uint8_t lockerr_report;                 // 锁状态异常上报方式: 0-不上报, 1-GPRS, 2-GPRS+SMS, 3-GPRS+SMS+CALL
    uint8_t lockerr_buzzer;                 // 锁状态异常蜂鸣器报警方式: 0-不报警, 1-报警30s, 2-持续报警
} LockErrConfig_t;

typedef struct                              // 存储的锁销状态报警配置参数
{
    uint8_t flag;                           // 参数有效标志
    uint8_t pinstat_report;                 // 锁销状态上报方式: 0-不上报, 1-GPRS, 2-GPRS+SMS, 3-GPRS+SMS+CALL
    uint8_t pinstat_trigger;                // 锁销状态触发方式: 0-都不触发, 1-插入触发, 2-拔出触发, 3-插入拔出均触发
} PinStatConfig_t;

typedef struct                              // 存储的锁状态报警配置参数
{
    uint8_t flag;                           // 参数有效标志
    uint8_t lockstat_report;                // 锁状态上报方式: 1-GPRS, 2-GPRS+SMS, 3-GPRS+SMS+CALL
    uint8_t lockstat_trigger;               // 锁状态触发方式: 0-都不触发, 1-上锁触发, 2-解锁触发, 3-上锁解锁均触发
} LockStatConfig_t;

typedef struct                              // 存储的运动检测报警配置参数
{
    uint8_t flag;                           // 参数有效标志
    uint16_t motdet_static_g;               // 静止G值方差阈值 (1-500 mg)
    uint16_t motdet_land_g;                 // 陆运G值方差阈值 (1-3000 mg)
    uint16_t motdet_static_land_length;     // 静止进入陆运投票时长 (30-600 s)
    uint16_t motdet_sea_transport_time;     // 进入海运投票时长 (10-600 s)
    uint8_t  motdet_report_type;            // 模式切换上报方式: 1-GPRS, 2-GPRS+SMS, 3-GPRS+SMS+CALL
} MotDetConfig_t;

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
} BatlevelConfig_t;

typedef struct                              // 存储的撞击报警配置参数
{
    uint8_t flag;                           // 参数有效标志
    uint8_t shockalarm_sw;                  // 撞击报警开关: 0-OFF, 1-ON
    uint8_t shockalarm_level;               // 撞击力度阈值: 1-5 (1最不敏感,5最敏感)
    uint8_t shockalarm_type;                // 告警上报方式: 1-GPRS, 2-GPRS+SMS, 3-GPRS+SMS+CALL
} ShockAlarmConfig_t;

typedef struct                              // 存储的数据记录功能配置参数
{
    uint8_t flag;                           // 参数有效标志
    uint8_t startr_sw;                      // 数据记录功能开关: 0-OFF, 1-ON
} StartrConfig_t;

typedef struct                              // 存储的低功耗运输状态配置参数
{
    uint8_t flag;                           // 参数有效标志
    uint8_t pwsave_sw;                      // 低功耗运输状态开关: 0-OFF, 1-ON
} PWRsaveConfig_t;

typedef struct                              // 存储的蓝牙数据上传配置参数
{
    uint8_t flag;                           // 参数有效标志
    uint8_t  bt_updata_mode;                // 工作方式: 0-不开启, 1-Cell启动时开启, 2-持续收集Cell启动上传, 3-持续收集Cell启动上传或唤醒上传
    uint32_t bt_updata_scan_interval;       // 蓝牙数据收集间隔: 1-86400秒
    uint32_t bt_updata_scan_length;         // 每次收集搜索时长: 1-86400秒
    uint32_t bt_updata_updata_interval;     // 蓝牙唤醒间隔: 1-86400秒
} BtUpdataConfig_t;

typedef struct                              // 存储的Tag定位功能配置参数
{
    uint8_t flag;                           // 参数有效标志
    uint8_t tag_sw;                         // Tag定位功能开关: 0-OFF, 1-ON
    uint16_t tag_interval;                  // 广播间隔: 100-60000ms
} TagConfig_t;

typedef struct                              // 存储的自动上锁配置参数
{
    uint8_t flag;                           // 参数有效标志
    uint16_t lockcd_countdown;              // 插入后上锁倒计时: 0-3600秒, 0代表不自动上锁
} LockedConfig_t;

typedef struct                              // 存储的LED显示配置参数
{
    uint8_t flag;                           // 参数有效标志
    uint8_t led_display;                    // LED显示开关: 0-OFF, 1-ON
} LedConfig_t;

typedef struct                              // 存储的蜂鸣器配置参数
{
    uint8_t flag;                           // 参数有效标志
    uint8_t buzzer_operator;                // 蜂鸣器操作: 0-停止, 1-持续报警, 2-成功提示音, 3-失败提示音, 4-异常提示音, 5-一般报警音
} BuzzerConfig_t;

typedef struct                              // 存储的NFC触发规则配置参数
{
    uint8_t flag;                           // 参数有效标志
    nfctrig_table_t nfctrig_table;          // NFC触发规则表，管理所有已配置的NFC触发规则
} NfctrigConfig_t;

typedef struct                              // 存储的NFC卡权限配置参数
{
    uint8_t flag;                           // 参数有效标志
    NfcAuthCard nfcauth_cards[10];          // NFC卡权限数组，最多10张卡
    uint8_t     nfcauth_card_count;         // 已授权卡数量
} NfcauthConfig_t;

typedef struct                              // 存储的蓝牙解锁密钥配置参数
{
    uint8_t flag;                           // 参数有效标志
    char        bt_key[7];                  // 蓝牙解锁密钥，6位数字 + 结束符
} BkeyConfig_t;

typedef struct
{
    uint8_t flag;                           // 参数有效标志
    bool ble_ota_reboot;                  // 蓝牙OTA升级成功重启设备标志位
} OtaConfig_t;

typedef struct                              // 存储的透传mac地址配置数据
{
    uint8_t flag;                                   // 参数有效标志
    bt_addr_le_t bt_parmac_macs[TRAN_MAC_MAX_NUM];  // 透传MAC地址列表，最多20个
    uint8_t      bt_parmac_mac_count;               // 已配置MAC数量
} BparmacConfig_t;

typedef struct
{
    lic_ff_struct               lic_ff;                     // 存储的LICENSE FF信息
    lic_gg_struct               lic_gg;                     // 存储的LICENSE GG信息
    AdvValidValue_t             adv_valid_value;            // 广播有效值
    uint16_t                    ECDH_GValue;                // ECDH_GValue值
    GsmImei_t                   gsm_imei;                   // GSM IMEI
    macaddr_t                   my_macaddr;                 // 设备MAC地址
    BleTxPower_t                ble_tx_power;               // 蓝牙发射功率
    BleLogConfig_t              ble_log_config;             // 蓝牙日志配置
    WorkModeConfig_t            device_workmode_config;     // 设备工作模式配置
    RemAlmConfig_t              remalm_config;              // 防拆报警配置
    LockPinCytConfig_t          lockpincyt_config;          // 锁销非法拔除报警配置
    LockErrConfig_t             lockerr_config;             // 锁状态异常报警配置
    PinStatConfig_t             pinstat_config;             // 锁销状态报警配置
    LockStatConfig_t            lockstat_config;            // 锁状态报警配置
    MotDetConfig_t              motdet_config;              // 运动检测报警配置
    BatlevelConfig_t            batlevel_config;            // 电池状态和充电状态报警配置
    ShockAlarmConfig_t          shockalarm_config;          // 撞击报警配置
    StartrConfig_t              startr_config;              // 数据记录功能配置
    PWRsaveConfig_t             pwsave_config;              // 低功耗运输状态配置
    BtUpdataConfig_t            bt_updata_config;           // 蓝牙数据上传配置
    TagConfig_t                 tag_config;                 // Tag定位功能配置
    LockedConfig_t              locked_config;              // 自动上锁配置
    LedConfig_t                 led_config;                 // LED显示配置
    BuzzerConfig_t              buzzer_config;              // 蜂鸣器配置
    NfctrigConfig_t             nfctrig_config;             // NFC触发规则配置
    NfcauthConfig_t             nfcauth_config;             // NFC卡权限配置
    BkeyConfig_t                bkey_config;                // 蓝牙解锁密钥配置
    OtaConfig_t                 ota_config;                 // OTA升级相关配置
    BparmacConfig_t             bparmac_config;             // 透传mac地址配置
} ConfigParamStruct;

extern ConfigParamStruct    gConfigParam;

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
const lic_ff_struct *my_param_get_ff(void);
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
const lic_gg_struct *my_param_get_gg(void);
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
**函数名称:  my_param_set_imei
**入口参数:  param: 要设置的IMEI值, len: 数据长度
**出口参数:  无
**函数功能:  设置IMEI值
**返 回 值:  0表示成功，负值表示失败
*********************************************************************/
int my_param_set_imei(char *param, uint8_t len);
/********************************************************************
**函数名称:  my_param_get_imei
**入口参数:  无
**出口参数:  无
**函数功能:  获取IMEI配置数据
**返 回 值:  返回IMEI结构体指针
*********************************************************************/
const GsmImei_t *my_param_get_imei(void);
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
int my_param_set_ble_log_config(const BleLogConfig_t *config);
/********************************************************************
**函数名称:  my_param_get_ble_log_config
**入口参数:  无
**出口参数:  无
**函数功能:  获取蓝牙日志配置
**返 回 值:  返回蓝牙日志配置结构体指针
*********************************************************************/
BleLogConfig_t *my_param_get_ble_log_config(void);
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

#endif