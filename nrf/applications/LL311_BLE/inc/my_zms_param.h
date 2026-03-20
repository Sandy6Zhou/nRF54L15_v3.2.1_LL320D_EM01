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

#define ZMS_ID_FF             0
#define ZMS_ID_GG             1
#define ZMS_ID_ADV_VALID      2
#define ZMS_ID_ECDH_G         3
#define ZMS_ID_IMEI           4
#define ZMS_ID_MAC            5
#define ZMS_ID_BLE_TX_POWER   6
#define ZMS_ID_BLE_LOG_CONFIG 7

#define LICENSE_FF_STR_LEN                  (29 * 2)
#define LICENSE_GG_STR_LEN                  (24 * 2)

#define FLAG_VALID 0xAA

#define GSM_IMEI_LENGTH                     15
#define DEV_NAME_USE_IMEI_POS               10
#define MY_MAC_LENGTH                       6

typedef struct /* 存储的LICENSE GG信息 */
{
    uint8_t flag;
    uint8_t hex[(LICENSE_GG_STR_LEN / 2) + (LICENSE_GG_STR_LEN % 2)];
}lic_gg_struct;

typedef struct /* 存储的LICENSE FF信息 */
{
    uint8_t flag;
    uint8_t hex[(LICENSE_FF_STR_LEN / 2) + (LICENSE_FF_STR_LEN % 2)];
}lic_ff_struct;

typedef struct
{
    uint8_t AppleValid;
    uint8_t GoogleValid;
}AdvValidValue_t;

typedef struct /* 存储的IMEI信息 */
{
    uint8_t flag;
    uint8_t hex[GSM_IMEI_LENGTH];
}GsmImei_t;

typedef struct /* 存储的IMEI信息 */
{
    uint8_t flag;
    uint8_t hex[MY_MAC_LENGTH];
}macaddr_t;

typedef struct /* 存储的蓝牙发射功率参数 */
{
    uint8_t flag;    /* 参数有效标志 */
    int8_t tx_power; /* 发射功率(dBm)，范围: -8 ~ +8 */
} BleTxPower_t;

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

typedef struct /* 存储的蓝牙日志配置参数 */
{
    uint8_t  flag;                           /* 参数有效标志 */
    uint8_t  global_en;                      /* 总开关: 0=关闭, 1=开启 */
    uint8_t  reserved[2];                    /* 预留对齐，确保mod_en 4字节对齐 */
    uint32_t mod_en;                         /* 模块开关bitmap，每位对应一个模块 */
    uint8_t  mod_level[BLE_LOG_MOD_MAX];     /* 各模块日志等级阈值 */
} BleLogConfig_t;

/* 获取指定模块在 mod_en bitmap 中的开关状态
 * 使用32位bitmap，mod_id 直接对应位位置 (0-31) */
#define BLE_LOG_MOD_IS_ENABLED(config, mod_id) \
    ((mod_id) < 32 ? ((config)->mod_en & (1U << (mod_id))) : 0)

typedef struct
{
    lic_ff_struct               lic_ff;
    lic_gg_struct               lic_gg;
    AdvValidValue_t             adv_valid_value;
    uint16_t                    ECDH_GValue;
    GsmImei_t                   gsm_imei;
    macaddr_t                   my_macaddr;
    DeviceWorkModeConfig        *workmode_config;
    BleTxPower_t                ble_tx_power;       /* 蓝牙发射功率 */
    BleLogConfig_t              ble_log_config;     /* 蓝牙日志配置 */
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
**函数名称:  my_param_set_ble_tx_power
**入口参数:  tx_power: 发射功率(dBm)，范围: -40 ~ +8
**出口参数:  无
**函数功能:  设置蓝牙发射功率参数
**返 回 值:  0表示成功，负值表示失败
*********************************************************************/
int my_param_set_ble_tx_power(int8_t tx_power);
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
#endif