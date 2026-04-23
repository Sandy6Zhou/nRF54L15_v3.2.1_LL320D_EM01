/* 必须在包含 my_comm.h 之前定义 BLE_LOG_MODULE_ID，避免与 my_ble_log.h 中的默认定义冲突 */
#define BLE_LOG_MODULE_ID BLE_LOG_MOD_PARAM

#include "my_comm.h"

LOG_MODULE_REGISTER(my_zms_param, LOG_LEVEL_INF);

#define DEFAULT_ECDH_G_VALUE      0x83A5
/* 全局配置参数 */
ConfigParamStruct    gConfigParam = {0};
/* 全局 ZMS 文件系统对象 */
static struct zms_fs user_data_fs;

const AdvValidValue_t gDefaultAdvValidValue =
{
    .flag = FLAG_VALID,
    .AppleValid = 1,
    .GoogleValid = 0,
};

const GsmImei_t gDefaultImeiValue =
{
    .flag = 0,
    .hex = {'1','2','3','4','5','6','7','8','9','0','1','2','3','4','5'}
};

const GsmImei_t gDefaultMacAddr =
{
    .flag = 0,
    .hex = {0x66, 0x55, 0x44, 0x33, 0x22, 0x11}
};

/* 蓝牙默认_TX_POWER配置 */
const BleTxPower_t gDefaultBleTxPower =
{
    .flag = FLAG_VALID,
    .tx_power = 0  /* 默认 0 dBm ，范围：-8dbm ~ +8dbm */
};

/* 蓝牙日志默认配置
 * 重要说明：以下模块不支持蓝牙日志（使用 MY_LOG_* 会导致递归或干扰）
 * - BLE 模块 (bit1): 蓝牙核心模块，使用蓝牙日志会导致递归发送
 * - DFU 模块 (bit2): OTA升级期间使用蓝牙日志会干扰升级流程
 * - SHELL 模块 (bit6): Shell 通过 RTT 交互，无需蓝牙日志
 * - CMD 模块 (bit10): 蓝牙指令处理模块，指令响应已通过 BLE 通道返回
 * 以上模块即使开启开关，也应保持 mod_level 为 LOG_LEVEL_NONE
 */
const BleLogConfig_t gDefaultBleLogConfig =
{
    .flag = FLAG_VALID,
    .global_en = 0,                         /* 默认关闭总开关 */
    .reserved = {0, 0},
    .mod_en =
        (1U << BLE_LOG_MOD_MAIN)   |   /* bit0: MAIN    - 开启 */
        (0U << BLE_LOG_MOD_BLE)    |   /* bit1: BLE     - 关闭，避免递归 */
        (0U << BLE_LOG_MOD_DFU)    |   /* bit2: DFU     - 关闭，避免干扰 */
        (1U << BLE_LOG_MOD_SENSOR) |   /* bit3: SENSOR  - 开启 */
        (1U << BLE_LOG_MOD_LTE)    |   /* bit4: LTE     - 开启 */
        (1U << BLE_LOG_MOD_CTRL)   |   /* bit5: CTRL    - 开启 */
        (0U << BLE_LOG_MOD_SHELL)  |   /* bit6: SHELL   - 关闭，shell模块没有必要加蓝牙日志 */
        (1U << BLE_LOG_MOD_NFC)    |   /* bit7: NFC     - 开启 */
        (1U << BLE_LOG_MOD_BATTERY)|   /* bit8: BATTERY - 开启 */
        (1U << BLE_LOG_MOD_MOTOR)  |   /* bit9: MOTOR   - 开启 */
        (0U << BLE_LOG_MOD_CMD)    |   /* bit10: CMD    - 关闭，指令模块避免递归 */
        (1U << BLE_LOG_MOD_TOOL)   |   /* bit11: TOOL   - 开启 */
        (1U << BLE_LOG_MOD_PARAM)  |   /* bit12: PARAM  - 开启 */
        (1U << BLE_LOG_MOD_WDT)    |   /* bit13: WDT    - 开启 */
        (0U << BLE_LOG_MOD_OTHER),     /* bit14: OTHER  - 关闭 */
    .mod_level = {
        [BLE_LOG_MOD_MAIN]   = LOG_LEVEL_INF,    /* bit0: MAIN    - 开启 */
        [BLE_LOG_MOD_BLE]    = LOG_LEVEL_NONE,   /* bit1: BLE     - 关闭，避免递归 */
        [BLE_LOG_MOD_DFU]    = LOG_LEVEL_NONE,   /* bit2: DFU     - 关闭，避免干扰 */
        [BLE_LOG_MOD_SENSOR] = LOG_LEVEL_INF,    /* bit3: SENSOR  - 开启 */
        [BLE_LOG_MOD_LTE]    = LOG_LEVEL_INF,    /* bit4: LTE     - 开启 */
        [BLE_LOG_MOD_CTRL]   = LOG_LEVEL_INF,    /* bit5: CTRL    - 开启 */
        [BLE_LOG_MOD_SHELL]  = LOG_LEVEL_INF,    /* bit6: SHELL   - 开启 */
        [BLE_LOG_MOD_NFC]    = LOG_LEVEL_INF,    /* bit7: NFC     - 开启 */
        [BLE_LOG_MOD_BATTERY] = LOG_LEVEL_INF,   /* bit8: BATTERY - 开启 */
        [BLE_LOG_MOD_MOTOR]  = LOG_LEVEL_INF,    /* bit9: MOTOR   - 开启 */
        [BLE_LOG_MOD_CMD]    = LOG_LEVEL_NONE,   /* bit10: CMD    - 关闭，指令模块避免递归 */
        [BLE_LOG_MOD_TOOL]   = LOG_LEVEL_INF,    /* bit11: TOOL   - 开启 */
        [BLE_LOG_MOD_PARAM]  = LOG_LEVEL_INF,    /* bit12: PARAM  - 开启 */
        [BLE_LOG_MOD_WDT]    = LOG_LEVEL_INF,    /* bit13: WDT    - 开启 */
        [BLE_LOG_MOD_OTHER]  = LOG_LEVEL_NONE,   /* bit14: OTHER  - 关闭 */
    }
};

const WorkModeConfig_t gDefaultWorkModeConfig =
{
    .flag = FLAG_VALID,
    .workmode_config =                          // 默认工作模式配置
    {
        .current_mode = MY_MODE_SMART,          // 默认智能模式
        .long_battery = // 长电池模式
        {
            .reporting_interval_min = 240,      // 默认240分钟上报一次
            .start_time = "0001",               // 默认00:01开始上报
        },
        .intelligent = // 智能模式
        {
            .stop_status_interval_sec = 86400,  // 默认86400秒上报一次
            .land_status_interval_sec = 15,     // 默认15秒上报一次
            .land_status_interval_dis = 100,    // 默认100米上报一次
            .sea_status_interval_sec = 14400,   // 默认14400秒上报一次
            .sleep_switch = 2,                  // 休眠开关
        },
    }
};

const RemAlmConfig_t gDefaultRemAlmConfig =
{
    .flag = FLAG_VALID,
    .remalm_sw = 0,                    /* 默认关闭 */
    .remalm_mode = REPORT_MODE_GPRS,                  /* 默认GPRS */
};

const LockPinCytConfig_t gDefaultLockPinCytConfig =
{
    .flag = FLAG_VALID,
    .lockpincyt_report = REPORT_MODE_GPRS,             /* 默认GPRS */
    .lockpincyt_buzzer = ALARM_TEMPORARY,             /* 默认报警30s */
};

const LockErrConfig_t gDefaultLockErrConfig =
{
    .flag = FLAG_VALID,
    .lockerr_report = REPORT_MODE_GPRS,               /* 默认GPRS */
    .lockerr_buzzer = ALARM_TEMPORARY,               /* 默认报警30s */
};

const PinStatConfig_t gDefaultPinStatConfig =
{
    .flag = FLAG_VALID,
    .pinstat_report = REPORT_MODE_GPRS,               /* 默认GPRS */
    .pinstat_trigger = PINSTAT_TRIGGER_MODE_BOTH,              /* 默认都触发 */
};

const LockStatConfig_t gDefaultLockStatConfig =
{
    .flag = FLAG_VALID,
    .lockstat_report = REPORT_MODE_GPRS,              /* 默认GPRS */
    .lockstat_trigger = LOCK_TRIGGER_NONE,             /* 默认都不触发 */
};

const MotDetConfig_t gDefaultMotDetConfig =
{
    .flag = FLAG_VALID,
    .motdet_static_g = 10,             /* 默认10 mg */
    .motdet_land_g = 2000,             /* 默认2000 mg */
    .motdet_static_land_length = 50,   /* 默认50 s */
    .motdet_sea_transport_time = 10,   /* 默认10 s */
    .motdet_report_type = REPORT_MODE_GPRS,           /* 默认GPRS */
};

const BatlevelConfig_t gDefaultBatlevelConfig =
{
    .flag = FLAG_VALID,
    .batlevel_empty_rpt = REPORT_MODE_GPRS,           /* 默认GPRS */
    .batlevel_low_rpt = REPORT_MODE_GPRS,             /* 默认GPRS */
    .batlevel_normal_rpt = REPORT_MODE_GPRS,          /* 默认GPRS */
    .batlevel_fair_rpt = REPORT_MODE_GPRS,            /* 默认GPRS */
    .batlevel_high_rpt = REPORT_MODE_GPRS,            /* 默认GPRS */
    .batlevel_full_rpt = REPORT_MODE_GPRS,            /* 默认GPRS */
    .chargesta_report = REPORT_MODE_GPRS,             /* 默认GPRS */
};

const ShockAlarmConfig_t gDefaultShockAlarmConfig =
{
    .flag = FLAG_VALID,
    .shockalarm_sw = 0,                /* 默认关闭 */
    .shockalarm_level = 3,             /* 默认中等敏感度 */
    .shockalarm_type = REPORT_MODE_GPRS,              /* 默认GPRS */
};

const StartrConfig_t gDefaultStartrConfig =
{
    .flag = FLAG_VALID,
    .startr_sw = 0,                    /* 默认关闭 */
};

const PWRsaveConfig_t gDefaultPWRsaveConfig =
{
    .flag = FLAG_VALID,
    .pwsave_sw = 0,                    /* 默认关闭 */
};

const BtUpdataConfig_t gDefaultBtUpdataConfig =
{
    .flag = FLAG_VALID,
    .bt_updata_mode = 0,               /* 默认不开启 */
    .bt_updata_scan_interval = 600,    /* 默认600秒 */
    .bt_updata_scan_length = 10,       /* 默认10秒 */
    .bt_updata_updata_interval = 14400,/* 默认14400秒 */
};

const TagConfig_t gDefaultTagConfig =
{
    .flag = FLAG_VALID,
    .tag_sw = 0,                       /* 默认关闭 */
    .tag_interval = 2000,              /* 默认2000ms */
};

const LockedConfig_t gDefaultLockedConfig =
{
    .flag = FLAG_VALID,
    .lockcd_countdown = 3,             /* 默认3秒 */
};

const LedConfig_t gDefaultLedConfig =
{
    .flag = FLAG_VALID,
    .led_display = 0,                  /* 默认关闭 */
};

const BuzzerConfig_t gDefaultBuzzerConfig =
{
    .flag = FLAG_VALID,
    .buzzer_operator = 0,             /* 默认停止 */
};

const NfctrigConfig_t gDefaultNfctrigConfig =
{
    .flag = FLAG_VALID,
    .nfctrig_table = 0,                    /* 默认关闭 */
};

const NfcauthConfig_t gDefaultNfcauthConfig =
{
    .flag = FLAG_VALID,
    .nfcauth_card_count = 0,         /* 默认0张卡 */
};

const BkeyConfig_t gDefaultBkeyConfig =
{
    .flag = FLAG_VALID,
    .bt_key = "000000",              /* 默认密钥 */
};

const OtaConfig_t gDefaultOtaConfig =
{
    .flag = FLAG_VALID,
    .ble_ota_reboot = false,
};

const BparmacConfig_t gDefaultBparmacConfig =
{
    .flag = FLAG_VALID,
    .bt_parmac_mac_count = 0,         /* 默认0个MAC地址 */
    .bt_parmac_macs = {0},           /* 默认0个MAC地址 */
};

/**
/********************************************************************
**函数名称:  my_user_data_storage_init
**入口参数:  无
**出口参数:  无
**函数功能:  初始化用户数据存储（ZMS文件系统）
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
static int my_user_data_storage_init(void)
{
    static bool inited;
    int err;
    const struct flash_area *fa;

    if (inited) {
        return 0;
    }

    /* 打开 pm_static.yml 中的 settings_storage 分区 */
    err = flash_area_open(FLASH_AREA_ID(settings_storage), &fa);
    if (err) {
        MY_LOG_INF("flash_area_open failed: %d", err);
        return err;
    }

    if (!device_is_ready(fa->fa_dev)) {
        MY_LOG_INF("Flash device for settings_storage not ready");
        flash_area_close(fa);
        return -ENODEV;
    }

    /* 填充 zms_fs 关键字段 */
    user_data_fs.offset       = fa->fa_off;   /* 分区起始偏移 */
    user_data_fs.flash_device = fa->fa_dev;   /* 底层 RRAM 设备 */

    /* 下面两行需要根据 nRF54L15 RRAM 的擦除块大小来设置：
     * ZMS 要求 sector_size 是擦除块大小的整数倍，sector_count 为扇区个数。
     * nRF54L15 RRAM 的擦除块为 4 kB
     */
    user_data_fs.sector_size  = 4096U;
    user_data_fs.sector_count = fa->fa_size / user_data_fs.sector_size;

    err = zms_mount(&user_data_fs);
    if (err) {
        MY_LOG_INF("zms_mount failed: %d", err);
        flash_area_close(fa);
        return err;
    }

    flash_area_close(fa);
    inited = true;
    return 0;
}

/********************************************************************
**函数名称:  my_user_data_write
**入口参数:  id: ZMS ID（32位）, data: 指向要写入的数据缓冲区, len: 数据长度（最大64 KiB）
**出口参数:  无
**函数功能:  通用写接口：按 ID 写入任意数据
**返 回 值:  >=0 写入的字节数；负值为错误码
*********************************************************************/
int my_user_data_write(uint32_t id, const void *data, int len)
{
    int ret;

    if (data == NULL || len == 0) {
        return -EINVAL;
    }

    ret = zms_write(&user_data_fs, id, data, len);
    if (ret < 0) {
        MY_LOG_INF("write data failed: %d (id=0x%08x)", (int)ret, id);
    }

    return ret;
}

/********************************************************************
**函数名称:  my_user_data_read
**入口参数:  id: ZMS ID（32位）, data: 指向接收数据的缓冲区, len: 缓冲区最大长度
**出口参数:  无
**函数功能:  通用读接口：按 ID 读取任意数据
**返 回 值:  >0 实际读取的字节数；0 表示未找到该 ID；负值为错误码
*********************************************************************/
static int my_user_data_read(uint32_t id, void *data, int len)
{
    int ret;

    if (data == NULL || len == 0) {
        return -EINVAL;
    }

    ret = zms_read(&user_data_fs, id, data, len);
    if (ret < 0) {
        MY_LOG_INF("read data failed: %d (id=0x%08x)", (int)ret, id);
    }

    return ret;
}

/********************************************************************
**函数名称:  my_param_load_config
**入口参数:  无
**出口参数:  无
**函数功能:  加载配置参数（从ZMS存储中读取所有配置数据）
**返 回 值:  无
*********************************************************************/
void my_param_load_config(void)
{
    int length;
    int ret;
    uint8_t data_buff[64] = {0};

    // 先初始化数据存储
    ret = my_user_data_storage_init();
    if (ret != 0) {
        MY_LOG_INF("Storage init failed: %d", ret);
        return;
    }

    //--------Load license ff data ---------------------
    length = sizeof(lic_ff_struct);
    ret = my_user_data_read(ZMS_ID_FF, &gConfigParam.lic_ff, length);
    if (ret != length)
    {
        MY_LOG_INF("get zms ff fail");
        memset(&gConfigParam.lic_ff, 0, length);
    }

    //--------Load license gg data ---------------------
    length = sizeof(lic_gg_struct);
    ret = my_user_data_read(ZMS_ID_GG, &gConfigParam.lic_gg, length);
    if (ret != length)
    {
        MY_LOG_INF("get zms gg fail");
        memset(&gConfigParam.lic_gg, 0, length);
    }

    //--------Load adv valid value data ---------------------
    length = sizeof(AdvValidValue_t);
    ret = my_user_data_read(ZMS_ID_ADV_VALID, &gConfigParam.adv_valid_value, length);
    if (ret != length)
    {
        memcpy(&gConfigParam.adv_valid_value, &gDefaultAdvValidValue, length);
        MY_LOG_INF("Adv valid value not found. Use default:AppleValid(%d),GoogleValid(%d)",
                gConfigParam.adv_valid_value.AppleValid,
                gConfigParam.adv_valid_value.GoogleValid);
    }

    set_adv_valid_status(APPLE_ADV_TYPE, gConfigParam.adv_valid_value.AppleValid);
    set_adv_valid_status(GOOGLE_ADV_TYPE, gConfigParam.adv_valid_value.GoogleValid);

    //--------Load ECDH G Value ---------------------
    length = sizeof(gConfigParam.ECDH_GValue);
    ret = my_user_data_read(ZMS_ID_ECDH_G, &gConfigParam.ECDH_GValue, length);
    if (ret != length)
    {
        gConfigParam.ECDH_GValue = DEFAULT_ECDH_G_VALUE;
        MY_LOG_INF("ECDH G value not found. Use default:ECDH G value(0x%04x)", gConfigParam.ECDH_GValue);
    }

    //--------Load IMEI Value ---------------------
    length = sizeof(GsmImei_t);
    ret = my_user_data_read(ZMS_ID_IMEI, &gConfigParam.gsm_imei, length);
    if (ret != length)
    {
        memcpy(&gConfigParam.gsm_imei, &gDefaultImeiValue, length);
        memcpy(data_buff, gConfigParam.gsm_imei.hex, sizeof(gConfigParam.gsm_imei.hex));
        MY_LOG_INF("imei not found. Use default:imei value(%s)", data_buff);
    }

    //--------Load mac addr ---------------------
    length = sizeof(macaddr_t);
    ret = my_user_data_read(ZMS_ID_MAC, &gConfigParam.my_macaddr, length);
    if (ret != length)
    {
        memcpy(&gConfigParam.my_macaddr, &gDefaultMacAddr, length);
        memcpy(data_buff, gConfigParam.my_macaddr.hex, sizeof(gConfigParam.my_macaddr.hex));
        MY_LOG_INF("mac addr not set. Use default:mac addr(%02x:%02x:%02x:%02x:%02x:%02x)",
            data_buff[5], data_buff[4], data_buff[3], data_buff[2], data_buff[1], data_buff[0]);
    }

    //--------Load BLE TX Power ---------------------
    length = sizeof(BleTxPower_t);
    ret = my_user_data_read(ZMS_ID_BLE_TX_POWER, &gConfigParam.ble_tx_power, length);
    if (ret != length)
    {
        memcpy(&gConfigParam.ble_tx_power, &gDefaultBleTxPower, length);
        MY_LOG_INF("BLE TX power not set. Use default:%d dBm", gConfigParam.ble_tx_power.tx_power);
    }
    else
    {
        MY_LOG_INF("BLE TX power loaded:%d dBm", gConfigParam.ble_tx_power.tx_power);
    }

    //--------Load BLE Log Config ---------------------
    length = sizeof(BleLogConfig_t);
    ret = my_user_data_read(ZMS_ID_BLE_LOG_CONFIG, &gConfigParam.ble_log_config, length);
    if (ret != length)
    {
        memcpy(&gConfigParam.ble_log_config, &gDefaultBleLogConfig, length);
        MY_LOG_INF("BLE log config not set. Use default: global_en=%d",
                gConfigParam.ble_log_config.global_en);
    }
    else
    {
        MY_LOG_INF("BLE log config loaded: global_en=%d", gConfigParam.ble_log_config.global_en);
    }

    //--------Load Device Workmode Config ---------------------
    length = sizeof(WorkModeConfig_t);
    ret = my_user_data_read(ZMS_ID_WORK_MODE_CONFIG, &gConfigParam.device_workmode_config, length);
    if (ret != length)
    {
        memcpy(&gConfigParam.device_workmode_config, &gDefaultWorkModeConfig, length);
        MY_LOG_INF("Device workmode config not found. Use default.");
    }

    //--------Load Remote Alarm Config ---------------------
    length = sizeof(RemAlmConfig_t);
    ret = my_user_data_read(ZMS_ID_REM_ALM_CONFIG, &gConfigParam.remalm_config, length);
    if (ret != length)
    {
        memcpy(&gConfigParam.remalm_config, &gDefaultRemAlmConfig, length);
        MY_LOG_INF("Remote alarm config not found. Use default:remalm_mode(%d), remalm_sw(%d)",
                    gConfigParam.remalm_config.remalm_mode, gConfigParam.remalm_config.remalm_sw);
    }
    else
    {
        MY_LOG_INF("Remote alarm config loaded: remalm_mode(%d), remalm_sw(%d)",
                    gConfigParam.remalm_config.remalm_mode, gConfigParam.remalm_config.remalm_sw);
    }

    //--------Load Lock Pin CyT Config ---------------------
    length = sizeof(LockPinCytConfig_t);
    ret = my_user_data_read(ZMS_ID_LOCK_PIN_CYT_CONFIG, &gConfigParam.lockpincyt_config, length);
    if (ret != length)
    {
        memcpy(&gConfigParam.lockpincyt_config, &gDefaultLockPinCytConfig, length);
        MY_LOG_INF("Lock pin cyt config not found. Use default:lockpincyt_buzzer(%d), lockpincyt_report(%d)",
                    gConfigParam.lockpincyt_config.lockpincyt_buzzer, gConfigParam.lockpincyt_config.lockpincyt_report);
    }
    else
    {
        MY_LOG_INF("Lock pin cyt config loaded: lockpincyt_buzzer(%d), lockpincyt_report(%d)",
                    gConfigParam.lockpincyt_config.lockpincyt_buzzer, gConfigParam.lockpincyt_config.lockpincyt_report);
    }

    //--------Load Lock Err Config ---------------------
    length = sizeof(LockErrConfig_t);
    ret = my_user_data_read(ZMS_ID_LOCK_ERR_CONFIG, &gConfigParam.lockerr_config, length);
    if (ret != length)
    {
        memcpy(&gConfigParam.lockerr_config, &gDefaultLockErrConfig, length);
        MY_LOG_INF("Lock err config not found. Use default:lockerr_buzzer(%d), lockerr_report(%d)",
                    gConfigParam.lockerr_config.lockerr_buzzer, gConfigParam.lockerr_config.lockerr_report);
    }
    else
    {
        MY_LOG_INF("Lock err config loaded: lockerr_buzzer(%d), lockerr_report(%d)",
                    gConfigParam.lockerr_config.lockerr_buzzer, gConfigParam.lockerr_config.lockerr_report);
    }

    //--------Load Pin Stat Config ---------------------
    length = sizeof(PinStatConfig_t);
    ret = my_user_data_read(ZMS_ID_PIN_STAT_CONFIG, &gConfigParam.pinstat_config, length);
    if (ret != length)
    {
        memcpy(&gConfigParam.pinstat_config, &gDefaultPinStatConfig, length);
        MY_LOG_INF("Pin stat config not found. Use default:pinstat_report(%d), pinstat_trigger(%d)",
                    gConfigParam.pinstat_config.pinstat_report, gConfigParam.pinstat_config.pinstat_trigger);
    }
    else
    {
        MY_LOG_INF("Pin stat config loaded: pinstat_report(%d), pinstat_trigger(%d)",
                    gConfigParam.pinstat_config.pinstat_report, gConfigParam.pinstat_config.pinstat_trigger);
    }

    //--------Load Lock Stat Config ---------------------
    length = sizeof(LockStatConfig_t);
    ret = my_user_data_read(ZMS_ID_LOCK_STAT_CONFIG, &gConfigParam.lockstat_config, length);
    if (ret != length)
    {
        memcpy(&gConfigParam.lockstat_config, &gDefaultLockStatConfig, length);
        MY_LOG_INF("Lock stat config not found. Use default:lockstat_report(%d), lockstat_trigger(%d)",
                    gConfigParam.lockstat_config.lockstat_report, gConfigParam.lockstat_config.lockstat_trigger);
    }
    else
    {
        MY_LOG_INF("Lock stat config loaded: lockstat_report(%d), lockstat_trigger(%d)",
                    gConfigParam.lockstat_config.lockstat_report, gConfigParam.lockstat_config.lockstat_trigger);
    }

    //--------Load Mot Det Config ---------------------
    length = sizeof(MotDetConfig_t);
    ret = my_user_data_read(ZMS_ID_MOT_DET_CONFIG, &gConfigParam.motdet_config, length);
    if (ret != length)
    {
        memcpy(&gConfigParam.motdet_config, &gDefaultMotDetConfig, length);
        MY_LOG_INF("Mot det config not found. Use default:motdet_land_g(%d), motdet_report_type(%d), "
                   "motdet_sea_transport_time(%d), motdet_static_g(%d), motdet_static_land_length(%d)",
                    gConfigParam.motdet_config.motdet_land_g, gConfigParam.motdet_config.motdet_report_type,
                    gConfigParam.motdet_config.motdet_sea_transport_time, gConfigParam.motdet_config.motdet_static_g,
                    gConfigParam.motdet_config.motdet_static_land_length);
    }
    else
    {
        MY_LOG_INF("Mot det config loaded: motdet_land_g(%d), motdet_report_type(%d), "
                    "motdet_sea_transport_time(%d), motdet_static_g(%d), motdet_static_land_length(%d)",
                    gConfigParam.motdet_config.motdet_land_g, gConfigParam.motdet_config.motdet_report_type,
                    gConfigParam.motdet_config.motdet_sea_transport_time, gConfigParam.motdet_config.motdet_static_g,
                    gConfigParam.motdet_config.motdet_static_land_length);
    }

    //--------Load Batlevel Config ---------------------
    length = sizeof(BatlevelConfig_t);
    ret = my_user_data_read(ZMS_ID_BAT_LEVEL_CONFIG, &gConfigParam.batlevel_config, length);
    if (ret != length)
    {
        memcpy(&gConfigParam.batlevel_config, &gDefaultBatlevelConfig, length);
        MY_LOG_INF("Batlevel config not found. Use default:batlevel_empty_rpt(%d), batlevel_low_rpt(%d), batlevel_normal_rpt(%d), "
                   "batlevel_fair_rpt(%d), batlevel_high_rpt(%d), batlevel_full_rpt(%d), chargesta_report(%d)",
                    gConfigParam.batlevel_config.batlevel_empty_rpt, gConfigParam.batlevel_config.batlevel_low_rpt,
                    gConfigParam.batlevel_config.batlevel_normal_rpt, gConfigParam.batlevel_config.batlevel_fair_rpt,
                    gConfigParam.batlevel_config.batlevel_high_rpt, gConfigParam.batlevel_config.batlevel_full_rpt,
                    gConfigParam.batlevel_config.chargesta_report);
    }
    else
    {
        MY_LOG_INF("Batlevel config loaded: batlevel_empty_rpt(%d), batlevel_low_rpt(%d), batlevel_normal_rpt(%d), "
                   "batlevel_fair_rpt(%d), batlevel_high_rpt(%d), batlevel_full_rpt(%d), ",
                    gConfigParam.batlevel_config.batlevel_empty_rpt, gConfigParam.batlevel_config.batlevel_low_rpt,
                    gConfigParam.batlevel_config.batlevel_normal_rpt, gConfigParam.batlevel_config.batlevel_fair_rpt,
                    gConfigParam.batlevel_config.batlevel_high_rpt, gConfigParam.batlevel_config.batlevel_full_rpt,
                    gConfigParam.batlevel_config.chargesta_report);
    }

    //--------Load Shock Alarm Config ---------------------
    length = sizeof(ShockAlarmConfig_t);
    ret = my_user_data_read(ZMS_ID_SHOCK_ALARM_CONFIG, &gConfigParam.shockalarm_config, length);
    if (ret != length)
    {
        memcpy(&gConfigParam.shockalarm_config, &gDefaultShockAlarmConfig, length);
        MY_LOG_INF("Shock alarm config not found. Use default:shockalarm_level(%d), shockalarm_sw(%d), shockalarm_type(%d)",
                    gConfigParam.shockalarm_config.shockalarm_level, gConfigParam.shockalarm_config.shockalarm_sw,
                    gConfigParam.shockalarm_config.shockalarm_type);
    }
    else
    {
        MY_LOG_INF("Shock alarm config loaded: shockalarm_level(%d), shockalarm_sw(%d), shockalarm_type(%d)",
                    gConfigParam.shockalarm_config.shockalarm_level, gConfigParam.shockalarm_config.shockalarm_sw,
                    gConfigParam.shockalarm_config.shockalarm_type);
    }

    //--------Load Startr Config ---------------------
    length = sizeof(StartrConfig_t);
    ret = my_user_data_read(ZMS_ID_STARTR_CONFIG, &gConfigParam.startr_config, length);
    if (ret != length)
    {
        memcpy(&gConfigParam.startr_config, &gDefaultStartrConfig, length);
        MY_LOG_INF("Startr config not found. Use default:startr_sw(%d)", gConfigParam.startr_config.startr_sw);
    }
    else
    {
        MY_LOG_INF("Startr config loaded: startr_sw(%d)", gConfigParam.startr_config.startr_sw);
    }

    //--------Load PWRsave Config ---------------------
    length = sizeof(PWRsaveConfig_t);
    ret = my_user_data_read(ZMS_ID_PWSAVE_CONFIG, &gConfigParam.pwsave_config, length);
    if (ret != length)
    {
        memcpy(&gConfigParam.pwsave_config, &gDefaultPWRsaveConfig, length);
        MY_LOG_INF("PWRsave config not found. Use default:pwsave_sw(%d)", gConfigParam.pwsave_config.pwsave_sw);
    }
    else
    {
        MY_LOG_INF("PWRsave config loaded: pwsave_sw(%d)", gConfigParam.pwsave_config.pwsave_sw);
    }

    //--------Load BTUPDATA Config ---------------------
    length = sizeof(BtUpdataConfig_t);
    ret = my_user_data_read(ZMS_ID_BT_UPDATA_CONFIG, &gConfigParam.bt_updata_config, length);
    if (ret != length)
    {
        memcpy(&gConfigParam.bt_updata_config, &gDefaultBtUpdataConfig, length);
        MY_LOG_INF("BTUPDATA config not found. Use default:bt_updata_mode(%d), bt_updata_scan_interval(%d), bt_updata_scan_length(%d), bt_updata_updata_interval(%d)",
                    gConfigParam.bt_updata_config.bt_updata_mode, gConfigParam.bt_updata_config.bt_updata_scan_interval,
                    gConfigParam.bt_updata_config.bt_updata_scan_length, gConfigParam.bt_updata_config.bt_updata_updata_interval);
    }
    else
    {
        MY_LOG_INF("BTUPDATA config loaded: bt_updata_mode(%d), bt_updata_scan_interval(%d), bt_updata_scan_length(%d), bt_updata_updata_interval(%d)",
                    gConfigParam.bt_updata_config.bt_updata_mode, gConfigParam.bt_updata_config.bt_updata_scan_interval,
                    gConfigParam.bt_updata_config.bt_updata_scan_length, gConfigParam.bt_updata_config.bt_updata_updata_interval);
    }

    //--------Load Tag Config ---------------------
    length = sizeof(TagConfig_t);
    ret = my_user_data_read(ZMS_ID_TAG_CONFIG, &gConfigParam.tag_config, length);
    if (ret != length)
    {
        memcpy(&gConfigParam.tag_config, &gDefaultTagConfig, length);
        MY_LOG_INF("Tag config not found. Use default:tag_sw(%d), tag_interval(%d)", gConfigParam.tag_config.tag_sw, gConfigParam.tag_config.tag_interval);
    }
    else
    {
        MY_LOG_INF("Tag config loaded: tag_sw(%d), tag_interval(%d)", gConfigParam.tag_config.tag_sw, gConfigParam.tag_config.tag_interval);
    }

    //--------Load Locked Config ---------------------
    length = sizeof(LockedConfig_t);
    ret = my_user_data_read(ZMS_ID_LOCKED_CONFIG, &gConfigParam.locked_config, length);
    if (ret != length)
    {
        memcpy(&gConfigParam.locked_config, &gDefaultLockedConfig, length);
        MY_LOG_INF("Locked config not found. Use default:locked_countdown(%d)", gConfigParam.locked_config.lockcd_countdown);
    }
    else
    {
        MY_LOG_INF("Locked config loaded: locked_countdown(%d)", gConfigParam.locked_config.lockcd_countdown);
    }

    //--------Load Led Config ---------------------
    length = sizeof(LedConfig_t);
    ret = my_user_data_read(ZMS_ID_LED_CONFIG, &gConfigParam.led_config, length);
    if (ret != length)
    {
        memcpy(&gConfigParam.led_config, &gDefaultLedConfig, length);
        MY_LOG_INF("Led config not found. Use default:led_display(%d)", gConfigParam.led_config.led_display);
    }
    else
    {
        MY_LOG_INF("Led config loaded: led_display(%d)", gConfigParam.led_config.led_display);
    }

    //--------Load Buzzer Config ---------------------
    length = sizeof(BuzzerConfig_t);
    ret = my_user_data_read(ZMS_ID_BUZZER_CONFIG, &gConfigParam.buzzer_config, length);
    if (ret != length)
    {
        memcpy(&gConfigParam.buzzer_config, &gDefaultBuzzerConfig, length);
        MY_LOG_INF("Buzzer config not found. Use default:buzzer_operator(%d)", gConfigParam.buzzer_config.buzzer_operator);
    }
    else
    {
        MY_LOG_INF("Buzzer config loaded: buzzer_operator(%d)", gConfigParam.buzzer_config.buzzer_operator);
    }

    //--------Load Nfctrig Config ---------------------
    length = sizeof(NfctrigConfig_t);
    ret = my_user_data_read(ZMS_ID_NFTRIG_CONFIG, &gConfigParam.nfctrig_config, length);
    if (ret != length)
    {
        memcpy(&gConfigParam.nfctrig_config, &gDefaultNfctrigConfig, length);
        MY_LOG_INF("Nfctrig config not found. Use default.");
    }

    //--------Load Nfcauth Config ---------------------
    length = sizeof(NfcauthConfig_t);
    ret = my_user_data_read(ZMS_ID_NFCAUTH_CONFIG, &gConfigParam.nfcauth_config, length);
    if (ret != length)
    {
        memcpy(&gConfigParam.nfcauth_config, &gDefaultNfcauthConfig, length);
        MY_LOG_INF("Nfcauth config not found. Use default:card_count(%d)", gConfigParam.nfcauth_config.nfcauth_card_count);
    }
    else
    {
        MY_LOG_INF("Nfcauth config loaded: card_count(%d)", gConfigParam.nfcauth_config.nfcauth_card_count);
    }

    //--------Load Bkey Config ---------------------
    length = sizeof(BkeyConfig_t);
    ret = my_user_data_read(ZMS_ID_BT_KEY_CONFIG, &gConfigParam.bkey_config, length);
    if (ret != length)
    {
        memcpy(&gConfigParam.bkey_config, &gDefaultBkeyConfig, length);
        MY_LOG_INF("Bkey config not found. Use default.");
    }

    //--------Load Ota Config ---------------------
    length = sizeof(OtaConfig_t);
    ret = my_user_data_read(ZMS_ID_OTA_CONFIG, &gConfigParam.ota_config, length);
    MY_LOG_INF("Ota config loaded: ble_ota_reboot(%d)", gConfigParam.ota_config.ble_ota_reboot);
    if (ret != length)
    {
        memcpy(&gConfigParam.ota_config, &gDefaultOtaConfig, length);
        MY_LOG_INF("Ota config not found. Use default.");
    }

    //--------Load Bparmac Config ---------------------
    length = sizeof(BparmacConfig_t);
    ret = my_user_data_read(ZMS_ID_BT_PARMAC_CONFIG, &gConfigParam.bparmac_config, length);
    if (ret != length)
    {
        memcpy(&gConfigParam.bparmac_config, &gDefaultBparmacConfig, length);
        MY_LOG_INF("Bparmac config not found. Use default.");
    }
    else
    {
        MY_LOG_INF("Bparmac config loaded");
    }
}

/********************************************************************
**函数名称:  my_param_set_ff
**入口参数:  param: 要设置的iOS许可证数据, len: 数据长度
**出口参数:  无
**函数功能:  设置iOS数据到flash中
**返 回 值:  true表示成功，false表示失败
*********************************************************************/
bool my_param_set_ff(char *param, uint8_t len)
{
    int ret;
    int lic_stuct_len = sizeof(lic_ff_struct);

    if (len != LICENSE_FF_STR_LEN)
    {
        MY_LOG_INF("my_param_set_ff len error!");
        return false;
    }

    if (string_check_is_hex_str((const char *)param) != LICENSE_FF_STR_LEN)
    {
        MY_LOG_INF("invalid param");
        return false;
    }

    gConfigParam.lic_ff.flag = FLAG_VALID;
    hexstr_to_hex((uint8_t *)gConfigParam.lic_ff.hex, sizeof(gConfigParam.lic_ff.hex), param);

    ret = my_user_data_write(ZMS_ID_FF, &gConfigParam.lic_ff, lic_stuct_len);
    if (ret != lic_stuct_len)
    {
        MY_LOG_INF("zms set ff Error!!!");
        return false;
    }
    else
    {
        MY_LOG_INF("zms set ff OK!!!");
    }

    return true;
}

/********************************************************************
**函数名称:  my_param_get_ff
**入口参数:  无
**出口参数:  无
**函数功能:  获取iOS配置数据
**返 回 值:  返回iOS许可证结构体指针
*********************************************************************/
const lic_ff_struct *my_param_get_ff(void)
{
    return &gConfigParam.lic_ff;
}

/********************************************************************
**函数名称:  my_param_set_gg
**入口参数:  param: 要设置的Google许可证数据, len: 数据长度
**出口参数:  无
**函数功能:  设置Google数据到flash中
**返 回 值:  true表示成功，false表示失败
*********************************************************************/
bool my_param_set_gg(char *param, uint8_t len)
{
    int ret;
    int lic_stuct_len = sizeof(lic_gg_struct);

    if (len != LICENSE_GG_STR_LEN)
    {
        MY_LOG_INF("my_param_set_gg len error!");
        return false;
    }

    if (string_check_is_hex_str((const char *)param) != LICENSE_GG_STR_LEN)
    {
        MY_LOG_INF("invalid param");
        return false;
    }

    gConfigParam.lic_gg.flag = FLAG_VALID;
    hexstr_to_hex((uint8_t *)gConfigParam.lic_gg.hex, sizeof(gConfigParam.lic_gg.hex), param);

    ret = my_user_data_write(ZMS_ID_GG, &gConfigParam.lic_gg, lic_stuct_len);
    if (ret != lic_stuct_len)
    {
        MY_LOG_INF("zms set gg Error!!!");
        return false;
    }
    else
    {
        MY_LOG_INF("zms set gg OK!!!");
    }

    return true;
}

/********************************************************************
**函数名称:  my_param_get_gg
**入口参数:  无
**出口参数:  无
**函数功能:  获取Google配置数据
**返 回 值:  返回Google许可证结构体指针
*********************************************************************/
const lic_gg_struct *my_param_get_gg(void)
{
    return &gConfigParam.lic_gg;
}

/********************************************************************
**函数名称:  my_param_set_jatag_or_jgtag
**入口参数:  cmd: 命令字符串, param: 参数字符串
**出口参数:  无
**函数功能:  设置哪一路广播数据开启或关闭(google或ios)
**返 回 值:  0表示成功，负值表示失败
*********************************************************************/
int my_param_set_jatag_or_jgtag(char *cmd, char *param)
{
    int valid_len = sizeof(AdvValidValue_t);
    int ret;

    if (cmd == NULL || param == NULL) {
        MY_LOG_INF("cmd or param is null");
        return -1;
    }

    if (CMD_MATCHED(cmd, "JATAG"))
    {
        if (CMD_MATCHED(param, "ON"))
        {
            set_adv_valid_status(APPLE_ADV_TYPE, 1);
            gConfigParam.adv_valid_value.AppleValid = 1;

            ret = my_user_data_write(ZMS_ID_ADV_VALID, &gConfigParam.adv_valid_value, valid_len);
            if (ret != valid_len)
            {
                MY_LOG_INF("zms set jatag Error!!!");
                return -1;
            }
            else
            {
                MY_LOG_INF("zms set jatag OK!!!");
            }
        }
        else if (CMD_MATCHED(param, "OFF"))
        {
            if (gConfigParam.adv_valid_value.GoogleValid == 1)
            {
                set_adv_valid_status(APPLE_ADV_TYPE, 0);
                gConfigParam.adv_valid_value.AppleValid = 0;

                ret = my_user_data_write(ZMS_ID_ADV_VALID, &gConfigParam.adv_valid_value, valid_len);
                if (ret != valid_len)
                {
                    MY_LOG_INF("zms set jatag Error!!!");
                    return -1;
                }
                else
                {
                    MY_LOG_INF("zms set jatag OK!!!");
                }
            }
            else
            {
                return -1;
            }
        }
        else
        {
            return -1;
        }
    }
    else if (CMD_MATCHED(cmd, "JGTAG"))
    {
        if (CMD_MATCHED(param, "ON"))
        {
            set_adv_valid_status(GOOGLE_ADV_TYPE, 1);
            gConfigParam.adv_valid_value.GoogleValid = 1;

            ret = my_user_data_write(ZMS_ID_ADV_VALID, &gConfigParam.adv_valid_value, valid_len);
            if (ret != valid_len)
            {
                MY_LOG_INF("zms set jgtag Error!!!");
                return -1;
            }
            else
            {
                MY_LOG_INF("zms set jgtag OK!!!");
            }
        }
        else if (CMD_MATCHED(param, "OFF"))
        {
            if (gConfigParam.adv_valid_value.AppleValid == 1)
            {
                set_adv_valid_status(GOOGLE_ADV_TYPE, 0);
                gConfigParam.adv_valid_value.GoogleValid = 0;

                ret = my_user_data_write(ZMS_ID_ADV_VALID, &gConfigParam.adv_valid_value, valid_len);
                if (ret != valid_len)
                {
                    MY_LOG_INF("zms set jgtag Error!!!");
                    return -1;
                }
                else
                {
                    MY_LOG_INF("zms set jgtag OK!!!");
                }
            }
            else
            {
                return -1;
            }
        }
        else
        {
            return -1;
        }
    }

    return 0;
}

/********************************************************************
**函数名称:  my_param_set_Gvalue
**入口参数:  param: 要设置的ECDH G值字符串
**出口参数:  无
**函数功能:  设置ECDH G值
**返 回 值:  0表示成功，负值表示失败
*********************************************************************/
int my_param_set_Gvalue(char *param)
{
    uint16_t Gvalue;
    int Gvalue_len;
    int ret;

    if (string_check_is_number(0, param) == 0)
    {
        MY_LOG_INF("Gvalue param is not number");
        return -1;
    }

    Gvalue = atoi(param);
    if (Gvalue < 10000 || Gvalue > 60000)
    {
        MY_LOG_INF("MODIFYGV set fail, range(10000~60000)");
        return -1;
    }

    Gvalue_len = sizeof(gConfigParam.ECDH_GValue);
    gConfigParam.ECDH_GValue = Gvalue;

    ret = my_user_data_write(ZMS_ID_ECDH_G, &gConfigParam.ECDH_GValue, Gvalue_len);
    if (ret != Gvalue_len)
    {
        MY_LOG_INF("zms set Gvalue Error!!!");
        return -1;
    }
    else
    {
        MY_LOG_INF("zms set Gvalue OK!!!");
    }

    return 0;
}

/********************************************************************
**函数名称:  my_param_get_Gvalue
**入口参数:  无
**出口参数:  无
**函数功能:  获取ECDH G值
**返 回 值:  返回ECDH G值
*********************************************************************/
const uint16_t my_param_get_Gvalue(void)
{
    return gConfigParam.ECDH_GValue;
}

/********************************************************************
**函数名称:  my_param_set_imei
**入口参数:  param: 要设置的IMEI值, len: 数据长度
**出口参数:  无
**函数功能:  设置IMEI值
**返 回 值:  0表示成功，负值表示失败
*********************************************************************/
int my_param_set_imei(char *param, uint8_t len)
{
    int ret;
    int GsmImei_struct_len = sizeof(GsmImei_t);

    if (len != GSM_IMEI_LENGTH)
    {
        MY_LOG_INF("my_param_set_imei len error!");
        return -1;
    }

    if (string_check_is_hex_str((const char *)param) != GSM_IMEI_LENGTH)
    {
        MY_LOG_INF("invalid param");
        return -1;
    }

    gConfigParam.gsm_imei.flag = FLAG_VALID;
    memcpy(gConfigParam.gsm_imei.hex, param, len);

    ret = my_user_data_write(ZMS_ID_IMEI, &gConfigParam.gsm_imei, GsmImei_struct_len);
    if (ret != GsmImei_struct_len)
    {
        MY_LOG_INF("zms set imei Error!!!");
        return -1;
    }
    else
    {
        MY_LOG_INF("zms set imei OK!!!");
    }

    return 0;
}

/********************************************************************
**函数名称:  my_param_get_imei
**入口参数:  无
**出口参数:  无
**函数功能:  获取IMEI配置数据
**返 回 值:  返回IMEI结构体指针
*********************************************************************/
const GsmImei_t *my_param_get_imei(void)
{
    return &gConfigParam.gsm_imei;
}

/********************************************************************
**函数名称:  my_param_set_mac
**入口参数:  param: 要设置的MAC地址, len: 数据长度
**出口参数:  无
**函数功能:  设置MAC地址
**返 回 值:  0表示成功，负值表示失败
*********************************************************************/
int my_param_set_mac(char *param, uint8_t len)
{
    int ret;
    uint8_t my_macaddr[MY_MAC_LENGTH] = {0};
    uint8_t my_macaddr_reorder[MY_MAC_LENGTH] = {0};
    int macaddr_t_len = sizeof(macaddr_t);

    if (macstr_to_hex(param, my_macaddr) == 0)
    {
        return -1;
    }

    // nordic这里mac地址需要翻转,这样通过指令设置的mac地址与蓝牙广播出来的mac地址会一致
    char_array_reverse(my_macaddr, sizeof(my_macaddr), my_macaddr_reorder, sizeof(my_macaddr_reorder));

    gConfigParam.my_macaddr.flag = FLAG_VALID;
    memcpy(gConfigParam.my_macaddr.hex, my_macaddr_reorder, sizeof(my_macaddr_reorder));

    ret = my_user_data_write(ZMS_ID_MAC, &gConfigParam.my_macaddr, macaddr_t_len);
    if (ret != macaddr_t_len)
    {
        MY_LOG_INF("zms set mac Error!!!");
        return -1;
    }
    else
    {
        MY_LOG_INF("zms set mac OK!!!");
    }

    return 0;
}

/********************************************************************
**函数名称:  my_param_get_macaddr
**入口参数:  无
**出口参数:  无
**函数功能:  获取mac addr配置数据
**返 回 值:  返回MAC地址结构体指针
*********************************************************************/
const macaddr_t *my_param_get_macaddr(void)
{
    return &gConfigParam.my_macaddr;
}

/********************************************************************
**函数名称:  my_param_get_ble_tx_power
**入口参数:  无
**出口参数:  无
**函数功能:  获取蓝牙发射功率参数
**返 回 值:  发射功率(dBm)，如果参数无效返回默认值0
*********************************************************************/
int8_t my_param_get_ble_tx_power(void)
{
    return gConfigParam.ble_tx_power.tx_power;
}

/********************************************************************
**函数名称:  my_param_set_ble_log_config
**入口参数:  config: 蓝牙日志配置结构体指针
**出口参数:  无
**函数功能:  设置蓝牙日志完整配置
**返 回 值:  0表示成功，负值表示失败
*********************************************************************/
int my_param_set_ble_log_config(const BleLogConfig_t *config)
{
    int ret;
    int config_len = sizeof(BleLogConfig_t);

    if (config == NULL)
    {
        return -EINVAL;
    }

    memcpy(&gConfigParam.ble_log_config, config, config_len);
    gConfigParam.ble_log_config.flag = FLAG_VALID;

    ret = my_user_data_write(ZMS_ID_BLE_LOG_CONFIG, &gConfigParam.ble_log_config, config_len);
    if (ret != config_len)
    {
        MY_LOG_INF("zms set ble log config Error!!!");
        return -1;
    }
    else
    {
        MY_LOG_INF("zms set ble log config OK: global_en=%d", gConfigParam.ble_log_config.global_en);
    }

    return 0;
}

/********************************************************************
**函数名称:  my_param_get_ble_log_config
**入口参数:  无
**出口参数:  无
**函数功能:  获取蓝牙日志配置
**返 回 值:  返回蓝牙日志配置结构体指针
*********************************************************************/
BleLogConfig_t *my_param_get_ble_log_config(void)
{
    return &gConfigParam.ble_log_config;
}

/********************************************************************
**函数名称:  my_param_set_ble_log_global
**入口参数:  en: 总开关状态 (0=关闭, 1=开启)
**出口参数:  无
**函数功能:  设置蓝牙日志总开关
**返 回 值:  0表示成功，负值表示失败
*********************************************************************/
int my_param_set_ble_log_global(uint8_t en)
{
    BleLogConfig_t *config;

    config = my_param_get_ble_log_config();
    config->global_en = (en != 0) ? 1 : 0;

    return my_param_set_ble_log_config(config);
}

/********************************************************************
**函数名称:  my_param_set_ble_log_mod
**入口参数:  mod_id: 模块ID, en: 开关状态 (0=关闭, 1=开启)
**出口参数:  无
**函数功能:  设置指定模块的蓝牙日志开关
**返 回 值:  0表示成功，负值表示失败
*********************************************************************/
int my_param_set_ble_log_mod(uint8_t mod_id, uint8_t en)
{
    BleLogConfig_t *config;

    if (mod_id >= BLE_LOG_MOD_MAX || mod_id >= 32)
    {
        return -EINVAL;
    }

    config = my_param_get_ble_log_config();

    if (en)
    {
        config->mod_en |= (1U << mod_id);
    }
    else
    {
        config->mod_en &= ~(1U << mod_id);
    }

    return my_param_set_ble_log_config(config);
}

/********************************************************************
**函数名称:  my_param_set_ble_log_level
**入口参数:  mod_id: 模块ID, level: 日志等级阈值
**出口参数:  无
**函数功能:  设置指定模块的蓝牙日志等级阈值
**返 回 值:  0表示成功，负值表示失败
*********************************************************************/
int my_param_set_ble_log_level(uint8_t mod_id, uint8_t level)
{
    BleLogConfig_t *config;

    if (mod_id >= BLE_LOG_MOD_MAX)
    {
        return -EINVAL;
    }

    config = my_param_get_ble_log_config();
    config->mod_level[mod_id] = level;

    return my_param_set_ble_log_config(config);
}