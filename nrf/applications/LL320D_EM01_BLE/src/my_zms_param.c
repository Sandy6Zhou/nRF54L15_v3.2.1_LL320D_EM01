/* 必须在包含 my_comm.h 之前定义 BLE_LOG_MODULE_ID，避免与 my_ble_log.h 中的默认定义冲突 */
#define BLE_LOG_MODULE_ID BLE_LOG_MOD_PARAM

#include "my_comm.h"

LOG_MODULE_REGISTER(my_zms_param, LOG_LEVEL_INF);

#define DEFAULT_ECDH_G_VALUE      0x83A5
/* 全局配置参数 */
config_param_t    gConfigParam = {0};
/* 全局 ZMS 文件系统对象 */
static struct zms_fs s_user_data_fs;

const adv_valid_value_t gDefaultAdvValidValue =
{
    .flag = FLAG_VALID,
    .AppleValid = 1,
    .GoogleValid = 0,
};

const ecdh_g_t gDefaultEcdhGValue =
{
    .flag = FLAG_VALID,
    .ecdh_g = DEFAULT_ECDH_G_VALUE,
};

const gsm_sn_t gDefaultSnValue =
{
    .flag = 0,
    .hex = {'0','0','0','0','0','0','0','0','0','0','0','0'}
};

const macaddr_t gDefaultMacAddr =
{
    .flag = 0,
    .hex = {0x66, 0x55, 0x44, 0x33, 0x22, 0x11}
};

/* 蓝牙默认_TX_POWER配置 */
const ble_tx_power_t gDefaultBleTxPower =
{
    .flag = FLAG_VALID,
    .tx_power = 0  /* 默认 0 dBm ，范围：-8dbm ~ +8dbm */
};

/* 蓝牙日志默认配置
 * 重要说明：以下模块不支持蓝牙日志（使用 MY_LOG_* 会导致递归或干扰）
 * - BLE 模块 (bit1): 蓝牙核心模块，使用蓝牙日志会导致递归发送
 * - DFU 模块 (bit2): OTA升级期间使用蓝牙日志会干扰升级流程
 * - SHELL 模块 (bit6): Shell 通过 RTT 交互，无需蓝牙日志
 * - CMD 模块 (bit8): 蓝牙指令处理模块，指令响应已通过 BLE 通道返回
 * 以上模块即使开启开关，也应保持 mod_level 为 LOG_LEVEL_NONE
 */
const ble_log_config_t gDefaultBleLogConfig =
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
        (1U << BLE_LOG_MOD_BATTERY)|   /* bit7: BATTERY - 开启 */
        (0U << BLE_LOG_MOD_CMD)    |   /* bit8: CMD    - 关闭，指令模块避免递归 */
        (1U << BLE_LOG_MOD_TOOL)   |   /* bit9: TOOL   - 开启 */
        (1U << BLE_LOG_MOD_PARAM)  |   /* bit10: PARAM  - 开启 */
        (1U << BLE_LOG_MOD_WDT)    |   /* bit11: WDT    - 开启 */
        (0U << BLE_LOG_MOD_ALGORITHM) | /* bit12: ALGORITHM - 关闭 */
        (1U << BLE_LOG_MOD_MAGNETIC_UART) | /* bit13: MAGNETIC_UART - 开启 */
        (0U << BLE_LOG_MOD_OTHER),     /* bit14: OTHER  - 关闭 */
    .mod_level = {
        [BLE_LOG_MOD_MAIN]   = LOG_LEVEL_INF,    /* bit0: MAIN    - 开启 */
        [BLE_LOG_MOD_BLE]    = LOG_LEVEL_NONE,   /* bit1: BLE     - 关闭，避免递归 */
        [BLE_LOG_MOD_DFU]    = LOG_LEVEL_NONE,   /* bit2: DFU     - 关闭，避免干扰 */
        [BLE_LOG_MOD_SENSOR] = LOG_LEVEL_INF,    /* bit3: SENSOR  - 开启 */
        [BLE_LOG_MOD_LTE]    = LOG_LEVEL_INF,    /* bit4: LTE     - 开启 */
        [BLE_LOG_MOD_CTRL]   = LOG_LEVEL_INF,    /* bit5: CTRL    - 开启 */
        [BLE_LOG_MOD_SHELL]  = LOG_LEVEL_INF,    /* bit6: SHELL   - 开启 */
        [BLE_LOG_MOD_BATTERY] = LOG_LEVEL_INF,   /* bit7: BATTERY - 开启 */
        [BLE_LOG_MOD_CMD]    = LOG_LEVEL_NONE,   /* bit8: CMD    - 关闭，指令模块避免递归 */
        [BLE_LOG_MOD_TOOL]   = LOG_LEVEL_INF,    /* bit9: TOOL   - 开启 */
        [BLE_LOG_MOD_PARAM]  = LOG_LEVEL_INF,    /* bit10: PARAM  - 开启 */
        [BLE_LOG_MOD_WDT]    = LOG_LEVEL_INF,    /* bit11: WDT    - 开启 */
        [BLE_LOG_MOD_ALGORITHM] = LOG_LEVEL_NONE, /* bit12: ALGORITHM - 关闭 */
        [BLE_LOG_MOD_MAGNETIC_UART] = LOG_LEVEL_INF, /* bit13: MAGNETIC_UART - 开启 */
        [BLE_LOG_MOD_OTHER]  = LOG_LEVEL_NONE,   /* bit14: OTHER  - 关闭 */
    }
};

const work_mode_config_t gDefaultWorkModeConfig =
{
    .flag = FLAG_VALID,
    .workmode_config =                          // 默认工作模式配置
    {
        .current_mode = MY_MODE_SMART,          // 默认智能模式
        .continuous_tracking = // 连续追踪模式
        {
            .reporting_interval_sec = 30,      // 默认30秒上报一次
            .reporting_interval_dis = 100,    // 默认100米上报一次
        },
        .long_battery = // 长续航模式
        {
            .reporting_interval_min = 240,      // 默认240分钟上报一次
            .start_time = "0001",               // 默认00:01开始上报
            .gnss_sw = 1,                       // 默认GNSS开启(ON)
        },
        .intelligent = // 智能模式
        {
            .sub_mode = 5,                      // 默认子模式5（Cell+GNSS常开/秒）
            .static_interval = {300, 300, 300, 300, 300, 10},              // 默认静止间隔(单位:秒)
            .moving_interval = {300, 300, 10, 300, 10, 10},              // 默认运动间隔（单位:秒）
        },
    }
};

const remalm_config_t gDefaultRemAlmConfig =
{
    .flag = FLAG_VALID,
    .remalm_sw = 0,                             /* 默认关闭 */
    .remalm_mode = REPORT_MODE_GPRS,            /* 默认GPRS */
};

const pullalm_config_t gDefaultPullAlmConfig =
{
    .flag = FLAG_VALID,
    .pullalm_sw = 0,                             /* 默认关闭 */
    .pullalm_mode = REPORT_MODE_GPRS,            /* 默认GPRS */
};

const patalm_config_t gDefaultPatAlmConfig =
{
    .flag = FLAG_VALID,
    .patalm_sw = 0,                             /* 默认关闭 */
    .patalm_low_threshold = 255,                /* 默认低压不报警 */
    .patalm_high_threshold = 255,               /* 默认高压不报警 */
    .patalm_report_type = REPORT_MODE_GPRS,     /* 默认GPRS */
    .patalm_report_interval = 0,                /* 默认不重复上报 */
};

const tempalm_config_t gDefaultTempAlmConfig =
{
    .flag = FLAG_VALID,
    .tempalm_sw = 0,                             /* 默认关闭 */
    .temp_low_threshold = 255,                   /* 默认低温不报警 */
    .temp_high_threshold = 255,                  /* 默认高温不报警 */
    .humi_low_threshold = 255,                   /* 默认低湿度不报警 */
    .humi_high_threshold = 255,                  /* 默认高湿度不报警 */
    .tempalm_report_type = REPORT_MODE_GPRS,     /* 默认GPRS */
    .tempalm_report_interval = 0,                /* 默认不重复上报 */
};

const mot_det_config_t gDefaultMotDetConfig =
{
    .flag = FLAG_VALID,
    .motdet_vibration = 5,           /* 默认5次 */
    .motdet_duration = 10,           /* 默认10秒 */
};

const motdetalm_config_t gDefaultMotDetAlmConfig =
{
    .flag = FLAG_VALID,
    .motdetalm_sw = 1,                                    /* 默认开启 */
    .motdetalm_report_type = REPORT_MODE_GPRS,            /* 默认GPRS */
};

const bat_level_config_t gDefaultBatlevelConfig =
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

const startr_config_t gDefaultStartrConfig =
{
    .flag = FLAG_VALID,
    .startr_sw = 0,                    /* 默认关闭 */
};

const pwrlimit_config_t gDefaultPWRlimitConfig =
{
    .flag = FLAG_VALID,
    .pwrlimit_sw = 0,                   /* 默认允许开机 */
};

const lprunning_config_t gDefaultlprunningConfig =
{
    .flag = FLAG_VALID,
    .lprunning_sw = 0,                   /* 默认关闭 */
    .lprunning_threshold = 20,           /* 默认20%电量阈值 */
    .lprunning_interval = 24,            /* 默认24小时唤醒间隔 */
};

const bt_updata_config_t gDefaultBtUpdataConfig =
{
    .flag = FLAG_VALID,
    .bt_updata_mode = 0,               /* 默认不开启 */
    .bt_updata_scan_interval = 600,    /* 默认600秒 */
    .bt_updata_scan_length = 10,       /* 默认10秒 */
    .bt_updata_updata_interval = 14400,/* 默认14400秒 */
};

const bluetooth_config_t gDefaultBluetoothConfig =
{
    .flag = FLAG_VALID,
    .bluetooth_sw = 0,                   /* 默认关闭 */
    .bluetooth_a = 5,                    /* 默认5 */
    .bluetooth_b = 2,                    /* 默认2 min */
    .bluetooth_flag = 0,                 /* 默认未携带参数 */
};

const btconnect_config_t gDefaultBtconnectConfig =
{
    .flag = FLAG_VALID,
    .btconnect_sw = 1,                 /* 默认开启 */
    .btconnect_interval = 1800,        /* 默认1800秒 */
    .btconnect_report = 1,             /* 默认GPRS */
};

const tag_config_t gDefaultTagConfig =
{
    .flag = FLAG_VALID,
    .tag_sw = 0,                       /* 默认关闭 */
    .tag_interval = 2000,              /* 默认2000ms */
};

const led_config_t gDefaultLedConfig =
{
    .flag = FLAG_VALID,
    .led_display = 1,                  /* 默认模式1 */
};

const ltint_config_t gDefaultLtintConfig =
{
    .flag = FLAG_VALID,
    .T1 = 1000,                          /* 默认1000ms */
    .T2 = 1000,                          /* 默认1000ms */
};

const buzzer_config_t gDefaultBuzzerConfig =
{
    .flag = FLAG_VALID,
    .buzzer_operator = 0,             /* 默认停止 */
};

const bparmac_config_t gDefaultBparmacConfig =
{
    .flag = FLAG_VALID,
    .bt_parmac_mac_count = 0,         /* 默认0个MAC地址 */
    .bt_parmac_macs = {0},           /* 默认0个MAC地址 */
};

const patm_timer_config_t gDefaultPatmTimerConfig =
{
    .flag = FLAG_VALID,         // 默认配置有效
    .interval_min = 20,         // 默认每20分钟采集/上报一次气压
    .wakeup_cell_sw = 0,        // 默认不因气压缓存主动拉起LTE
};

const temp_timer_config_t gDefaultTempTimerConfig =
{
    .flag = FLAG_VALID,         // 默认配置有效
    .interval_min = 20,         // 默认每20分钟采集/上报一次温湿度
    .wakeup_cell_sw = 0,        // 默认不因温湿度缓存主动拉起LTE
};

const imu_alm_config_t gDefaultImuAlmConfig =
{
    .flag = FLAG_VALID,             // 默认配置有效
    .imu_alm_sw = 1,                // 默认开启
    .imu_alm_report = 1,            // 默认GPRS
    .imu_roll_threshold = 30,       // 默认报警阈值30度
    .imu_pitch_threshold = 25,      // 默认报警阈值25度
    .imu_yaw_threshold = 255,       // 默认不报警
    .imu_duration_time = 3,         // 默认3秒
    .imu_duration_count = 3,        // 默认3次
    .recover_time = 10,             // 默认10秒
};

const imu_zero_bias_config_t gDefaultImuZeroBiasConfig =
{
    .flag = FLAG_VALID,             // 默认配置有效
    .gyro_bias_x = 0.0f,            // 默认陀螺仪零偏估计 X (rad/s, 在线逐步追踪)
    .gyro_bias_y = 0.0f,            // 默认陀螺仪零偏估计 Y (rad/s)
    .gyro_bias_z = 0.0f,            // 默认陀螺仪零偏估计 Z (rad/s)
};
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
    s_user_data_fs.offset       = fa->fa_off;   /* 分区起始偏移 */
    s_user_data_fs.flash_device = fa->fa_dev;   /* 底层 RRAM 设备 */

    /* 下面两行需要根据 nRF54L15 RRAM 的擦除块大小来设置：
     * ZMS 要求 sector_size 是擦除块大小的整数倍，sector_count 为扇区个数。
     * nRF54L15 RRAM 的擦除块为 4 kB
     */
    s_user_data_fs.sector_size  = 4096U;
    s_user_data_fs.sector_count = fa->fa_size / s_user_data_fs.sector_size;

    err = zms_mount(&s_user_data_fs);
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

    ret = zms_write(&s_user_data_fs, id, data, len);
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
int my_user_data_read(uint32_t id, void *data, int len)
{
    int ret;

    if (data == NULL || len == 0) {
        return -EINVAL;
    }

    ret = zms_read(&s_user_data_fs, id, data, len);
    if (ret < 0) {
        MY_LOG_INF("read data failed: %d (id=0x%08x)", (int)ret, id);
    }

    return ret;
}

/********************************************************************
**函数名称:  my_param_log_config
**入口参数:  无
**出口参数:  无
**函数功能:  打印所有配置参数（格式：指令名:参数, 参数）
**返 回 值:  无
*********************************************************************/
void my_param_log_config(void)
{
    uint8_t data_buff[64] = {0};

    MY_LOG_INF("adv valid value: AppleValid(%d), GoogleValid(%d)",
                gConfigParam.adv_valid_value.AppleValid,
                gConfigParam.adv_valid_value.GoogleValid);
    MY_LOG_INF("ECDH G value: 0x%04x", gConfigParam.ECDH_GValue.ecdh_g);
    memcpy(data_buff, gConfigParam.gsm_sn.hex, sizeof(gConfigParam.gsm_sn.hex));
    MY_LOG_INF("sn value: %s", data_buff);
    memcpy(data_buff, gConfigParam.my_macaddr.hex, sizeof(gConfigParam.my_macaddr.hex));
    MY_LOG_INF("mac addr: %02x:%02x:%02x:%02x:%02x:%02x",
                data_buff[5], data_buff[4], data_buff[3], data_buff[2], data_buff[1], data_buff[0]);
    MY_LOG_INF("BLE TX power: %d dBm", gConfigParam.ble_tx_power.tx_power);
    MY_LOG_INF("BLE log config: global_en(%d)", gConfigParam.ble_log_config.global_en);
    MY_LOG_INF("remote alarm config: remalm_mode(%d), remalm_sw(%d)",
                gConfigParam.remalm_config.remalm_mode, gConfigParam.remalm_config.remalm_sw);
    MY_LOG_INF("pull alarm config: pullalm_mode(%d), pullalm_sw(%d)",
                gConfigParam.pullalm_config.pullalm_mode, gConfigParam.pullalm_config.pullalm_sw);
    MY_LOG_INF("pat alarm config: patalm_low_threshold(%d), patalm_high_threshold(%d), patalm_report_type(%d), patalm_report_interval(%d)",
                gConfigParam.patalm_config.patalm_low_threshold,
                gConfigParam.patalm_config.patalm_high_threshold,
                gConfigParam.patalm_config.patalm_report_type,
                gConfigParam.patalm_config.patalm_report_interval);
    MY_LOG_INF("temp alarm config: temp_low_threshold(%d), temp_high_threshold(%d), humi_low_threshold(%d), humi_high_threshold(%d), tempalm_report_type(%d), tempalm_report_interval(%d)",
                gConfigParam.tempalm_config.temp_low_threshold,
                gConfigParam.tempalm_config.temp_high_threshold,
                gConfigParam.tempalm_config.humi_low_threshold,
                gConfigParam.tempalm_config.humi_high_threshold,
                gConfigParam.tempalm_config.tempalm_report_type,
                gConfigParam.tempalm_config.tempalm_report_interval);
    MY_LOG_INF("mot det config: motdet_vibration(%d), motdet_duration(%d)",
                gConfigParam.motdet_config.motdet_vibration, gConfigParam.motdet_config.motdet_duration);
    MY_LOG_INF("mot det alarm config: motdetalm_sw(%d), motdetalm_report_type(%d)",
                gConfigParam.motdetalm_config.motdetalm_sw, gConfigParam.motdetalm_config.motdetalm_report_type);
    MY_LOG_INF("batlevel config: batlevel_empty_rpt(%d), batlevel_low_rpt(%d), batlevel_normal_rpt(%d), batlevel_fair_rpt(%d), batlevel_high_rpt(%d), batlevel_full_rpt(%d), chargesta_report(%d)",
                gConfigParam.batlevel_config.batlevel_empty_rpt, gConfigParam.batlevel_config.batlevel_low_rpt,
                gConfigParam.batlevel_config.batlevel_normal_rpt, gConfigParam.batlevel_config.batlevel_fair_rpt,
                gConfigParam.batlevel_config.batlevel_high_rpt, gConfigParam.batlevel_config.batlevel_full_rpt,
                gConfigParam.batlevel_config.chargesta_report);
    MY_LOG_INF("startr config: startr_sw(%d)", gConfigParam.startr_config.startr_sw);
    MY_LOG_INF("PWRLimit config: pwrlimit_sw(%d)", gConfigParam.pwrlimit_config.pwrlimit_sw);
    MY_LOG_INF("LPSLEEP config: lprunning_sw(%d), lprunning_threshold(%d), lprunning_interval(%d)",
                gConfigParam.lprunning_config.lprunning_sw,
                gConfigParam.lprunning_config.lprunning_threshold,
                gConfigParam.lprunning_config.lprunning_interval);
    MY_LOG_INF("BTUPDATA config: bt_updata_mode(%d), bt_updata_scan_interval(%d), bt_updata_scan_length(%d), bt_updata_updata_interval(%d)",
                gConfigParam.bt_updata_config.bt_updata_mode, gConfigParam.bt_updata_config.bt_updata_scan_interval,
                gConfigParam.bt_updata_config.bt_updata_scan_length, gConfigParam.bt_updata_config.bt_updata_updata_interval);
    MY_LOG_INF("bluetooth config: bluetooth_sw(%d), bluetooth_a(%d), bluetooth_b(%d), bluetooth_flag(%d)",
                gConfigParam.bluetooth_config.bluetooth_sw, gConfigParam.bluetooth_config.bluetooth_a,
                gConfigParam.bluetooth_config.bluetooth_b, gConfigParam.bluetooth_config.bluetooth_flag);
    MY_LOG_INF("BTCONNECT config: btconnect_sw(%d), btconnect_interval(%d), btconnect_report(%d)",
                gConfigParam.btconnect_config.btconnect_sw, gConfigParam.btconnect_config.btconnect_interval,
                gConfigParam.btconnect_config.btconnect_report);
    MY_LOG_INF("tag config: tag_sw(%d), tag_interval(%d)",
                gConfigParam.tag_config.tag_sw, gConfigParam.tag_config.tag_interval);
    MY_LOG_INF("led config: led_display(%d)", gConfigParam.led_config.led_display);
    MY_LOG_INF("ltint config: T1(%d), T2(%d)",
                gConfigParam.ltint_config.T1, gConfigParam.ltint_config.T2);
    MY_LOG_INF("buzzer config: buzzer_operator(%d)", gConfigParam.buzzer_config.buzzer_operator);
    MY_LOG_INF("patm timer config: interval_min(%d), wakeup_cell_sw(%d)",
                gConfigParam.patm_timer_config.interval_min,
                gConfigParam.patm_timer_config.wakeup_cell_sw);
    MY_LOG_INF("temp timer config: interval_min(%d), wakeup_cell_sw(%d)",
                gConfigParam.temp_timer_config.interval_min,
                gConfigParam.temp_timer_config.wakeup_cell_sw);
    MY_LOG_INF("imu zero bias config: gyro_bias_x(%f), gyro_bias_y(%f), gyro_bias_z(%f)",
                gConfigParam.imu_zero_bias_config.gyro_bias_x,
                gConfigParam.imu_zero_bias_config.gyro_bias_y,
                gConfigParam.imu_zero_bias_config.gyro_bias_z);
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

    // 先初始化数据存储
    ret = my_user_data_storage_init();
    if (ret != 0) {
        MY_LOG_INF("Storage init failed: %d", ret);
        return;
    }

    //--------Load license ff data ---------------------
    length = sizeof(lic_ff_t);
    ret = my_user_data_read(ZMS_ID_FF, &gConfigParam.lic_ff, length);
    if (gConfigParam.lic_ff.flag != FLAG_VALID || ret != length)
    {
        memset(&gConfigParam.lic_ff, 0, length);
    }

    //--------Load license gg data ---------------------
    length = sizeof(lic_gg_t);
    ret = my_user_data_read(ZMS_ID_GG, &gConfigParam.lic_gg, length);
    if (gConfigParam.lic_gg.flag != FLAG_VALID || ret != length)
    {
        memset(&gConfigParam.lic_gg, 0, length);
    }

    //--------Load adv valid value data ---------------------
    length = sizeof(adv_valid_value_t);
    ret = my_user_data_read(ZMS_ID_ADV_VALID, &gConfigParam.adv_valid_value, length);
    if (gConfigParam.adv_valid_value.flag != FLAG_VALID || ret != length)
    {
        memcpy(&gConfigParam.adv_valid_value, &gDefaultAdvValidValue, length);
    }

    set_adv_valid_status(APPLE_ADV_TYPE, gConfigParam.adv_valid_value.AppleValid);
    set_adv_valid_status(GOOGLE_ADV_TYPE, gConfigParam.adv_valid_value.GoogleValid);

    //--------Load ECDH G Value ---------------------
    length = sizeof(gConfigParam.ECDH_GValue);
    ret = my_user_data_read(ZMS_ID_ECDH_G, &gConfigParam.ECDH_GValue, length);
    if (gConfigParam.ECDH_GValue.flag != FLAG_VALID || ret != length)
    {
        memcpy(&gConfigParam.ECDH_GValue, &gDefaultEcdhGValue, length);
    }

    //--------Load SN Value ---------------------
    length = sizeof(gsm_sn_t);
    ret = my_user_data_read(ZMS_ID_SN, &gConfigParam.gsm_sn, length);
    if (gConfigParam.gsm_sn.flag != FLAG_VALID || ret != length)
    {
        memcpy(&gConfigParam.gsm_sn, &gDefaultSnValue, length);
    }

    //--------Load mac addr ---------------------
    length = sizeof(macaddr_t);
    ret = my_user_data_read(ZMS_ID_MAC, &gConfigParam.my_macaddr, length);
    if (gConfigParam.my_macaddr.flag != FLAG_VALID || ret != length)
    {
        memcpy(&gConfigParam.my_macaddr, &gDefaultMacAddr, length);
    }

    //--------Load BLE TX Power ---------------------
    length = sizeof(ble_tx_power_t);
    ret = my_user_data_read(ZMS_ID_BLE_TX_POWER, &gConfigParam.ble_tx_power, length);
    if (gConfigParam.ble_tx_power.flag != FLAG_VALID || ret != length)
    {
        memcpy(&gConfigParam.ble_tx_power, &gDefaultBleTxPower, length);
    }

    //--------Load BLE Log Config ---------------------
    length = sizeof(ble_log_config_t);
    ret = my_user_data_read(ZMS_ID_BLE_LOG_CONFIG, &gConfigParam.ble_log_config, length);
    if (gConfigParam.ble_log_config.flag != FLAG_VALID || ret != length)
    {
        memcpy(&gConfigParam.ble_log_config, &gDefaultBleLogConfig, length);
    }

    //--------Load Device Workmode Config ---------------------
    length = sizeof(work_mode_config_t);
    ret = my_user_data_read(ZMS_ID_WORK_MODE_CONFIG, &gConfigParam.device_workmode_config, length);
    if (gConfigParam.device_workmode_config.flag != FLAG_VALID || ret != length)
    {
        memcpy(&gConfigParam.device_workmode_config, &gDefaultWorkModeConfig, length);
    }

    //--------Load Remote Alarm Config ---------------------
    length = sizeof(remalm_config_t);
    ret = my_user_data_read(ZMS_ID_REM_ALM_CONFIG, &gConfigParam.remalm_config, length);
    if (gConfigParam.remalm_config.flag != FLAG_VALID || ret != length)
    {
        memcpy(&gConfigParam.remalm_config, &gDefaultRemAlmConfig, length);
    }

    //--------Load Pull Alarm Config ---------------------
    length = sizeof(pullalm_config_t);
    ret = my_user_data_read(ZMS_ID_PULL_ALM_CONFIG, &gConfigParam.pullalm_config, length);
    if (gConfigParam.pullalm_config.flag != FLAG_VALID || ret != length)
    {
        memcpy(&gConfigParam.pullalm_config, &gDefaultPullAlmConfig, length);
    }

    //--------Load Patalm Config ---------------------
    length = sizeof(patalm_config_t);
    ret = my_user_data_read(ZMS_ID_PATMALM_CONFIG, &gConfigParam.patalm_config, length);
    if (gConfigParam.patalm_config.flag != FLAG_VALID || ret != length)
    {
        memcpy(&gConfigParam.patalm_config, &gDefaultPatAlmConfig, length);
    }

    //--------Load Temp Alm Config ---------------------
    length = sizeof(tempalm_config_t);
    ret = my_user_data_read(ZMS_ID_TEMPALM_CONFIG, &gConfigParam.tempalm_config, length);
    if (gConfigParam.tempalm_config.flag != FLAG_VALID || ret != length)
    {
        memcpy(&gConfigParam.tempalm_config, &gDefaultTempAlmConfig, length);
    }

    //--------Load Mot Det Config ---------------------
    length = sizeof(mot_det_config_t);
    ret = my_user_data_read(ZMS_ID_MOT_DET_CONFIG, &gConfigParam.motdet_config, length);
    if (gConfigParam.motdet_config.flag != FLAG_VALID || ret != length)
    {
        memcpy(&gConfigParam.motdet_config, &gDefaultMotDetConfig, length);
    }

    //--------Load Mot Det Alm Config ---------------------
    length = sizeof(motdetalm_config_t);
    ret = my_user_data_read(ZMS_ID_MOTDETALM_CONFIG, &gConfigParam.motdetalm_config, length);
    if (gConfigParam.motdetalm_config.flag != FLAG_VALID || ret != length)
    {
        memcpy(&gConfigParam.motdetalm_config, &gDefaultMotDetAlmConfig, length);
    }

    //--------Load Batlevel Config ---------------------
    length = sizeof(bat_level_config_t);
    ret = my_user_data_read(ZMS_ID_BAT_LEVEL_CONFIG, &gConfigParam.batlevel_config, length);
    if (gConfigParam.batlevel_config.flag != FLAG_VALID || ret != length)
    {
        memcpy(&gConfigParam.batlevel_config, &gDefaultBatlevelConfig, length);
    }

    //--------Load Startr Config ---------------------
    length = sizeof(startr_config_t);
    ret = my_user_data_read(ZMS_ID_STARTR_CONFIG, &gConfigParam.startr_config, length);
    if (gConfigParam.startr_config.flag != FLAG_VALID || ret != length)
    {
        memcpy(&gConfigParam.startr_config, &gDefaultStartrConfig, length);
    }

    //--------Load PWRLimit Config ---------------------
    length = sizeof(pwrlimit_config_t);
    ret = my_user_data_read(ZMS_ID_PWRLIMIT_CONFIG, &gConfigParam.pwrlimit_config, length);
    if (gConfigParam.pwrlimit_config.flag != FLAG_VALID || ret != length)
    {
        memcpy(&gConfigParam.pwrlimit_config, &gDefaultPWRlimitConfig, length);
    }

    //--------Load LPSLEEP Config ---------------------
    length = sizeof(lprunning_config_t);
    ret = my_user_data_read(ZMS_ID_LPSLEEP_CONFIG, &gConfigParam.lprunning_config, length);
    if (gConfigParam.lprunning_config.flag != FLAG_VALID || ret != length)
    {
        memcpy(&gConfigParam.lprunning_config, &gDefaultlprunningConfig, length);
    }

    //--------Load BTUPDATA Config ---------------------
    length = sizeof(bt_updata_config_t);
    ret = my_user_data_read(ZMS_ID_BT_UPDATA_CONFIG, &gConfigParam.bt_updata_config, length);
    if (gConfigParam.bt_updata_config.flag != FLAG_VALID || ret != length)
    {
        memcpy(&gConfigParam.bt_updata_config, &gDefaultBtUpdataConfig, length);
    }

    //--------Load Bluetooth Config ---------------------
    length = sizeof(bluetooth_config_t);
    ret = my_user_data_read(ZMS_ID_BLUETOOTH_CONFIG, &gConfigParam.bluetooth_config, length);
    if (gConfigParam.bluetooth_config.flag != FLAG_VALID || ret != length)
    {
        memcpy(&gConfigParam.bluetooth_config, &gDefaultBluetoothConfig, length);
    }

    //--------Load BTCONNECT Config ---------------------
    length = sizeof(btconnect_config_t);
    ret = my_user_data_read(ZMS_ID_BTCONNECT_CONFIG, &gConfigParam.btconnect_config, length);
    if (gConfigParam.btconnect_config.flag != FLAG_VALID || ret != length)
    {
        memcpy(&gConfigParam.btconnect_config, &gDefaultBtconnectConfig, length);
    }

    //--------Load Tag Config ---------------------
    length = sizeof(tag_config_t);
    ret = my_user_data_read(ZMS_ID_TAG_CONFIG, &gConfigParam.tag_config, length);
    if (gConfigParam.tag_config.flag != FLAG_VALID || ret != length)
    {
        memcpy(&gConfigParam.tag_config, &gDefaultTagConfig, length);
    }

    //--------Load Led Config ---------------------
    length = sizeof(led_config_t);
    ret = my_user_data_read(ZMS_ID_LED_CONFIG, &gConfigParam.led_config, length);
    if (gConfigParam.led_config.flag != FLAG_VALID || ret != length)
    {
        memcpy(&gConfigParam.led_config, &gDefaultLedConfig, length);
    }

    //--------Load Ltint Config ---------------------
    length = sizeof(ltint_config_t);
    ret = my_user_data_read(ZMS_ID_LTINT_CONFIG, &gConfigParam.ltint_config, length);
    if (gConfigParam.ltint_config.flag != FLAG_VALID || ret != length)
    {
        memcpy(&gConfigParam.ltint_config, &gDefaultLtintConfig, length);
    }

    //--------Load Buzzer Config ---------------------
    length = sizeof(buzzer_config_t);
    ret = my_user_data_read(ZMS_ID_BUZZER_CONFIG, &gConfigParam.buzzer_config, length);
    if (gConfigParam.buzzer_config.flag != FLAG_VALID || ret != length)
    {
        memcpy(&gConfigParam.buzzer_config, &gDefaultBuzzerConfig, length);
    }

    //--------Load Bparmac Config ---------------------
    length = sizeof(bparmac_config_t);
    ret = my_user_data_read(ZMS_ID_BT_PARMAC_CONFIG, &gConfigParam.bparmac_config, length);
    if (gConfigParam.bparmac_config.flag != FLAG_VALID || ret != length)
    {
        memcpy(&gConfigParam.bparmac_config, &gDefaultBparmacConfig, length);
    }

    //--------Load Patm Timer Config ---------------------
    length = sizeof(patm_timer_config_t);
    ret = my_user_data_read(ZMS_ID_PATM_TIMER_CONFIG, &gConfigParam.patm_timer_config, length);
    if (gConfigParam.patm_timer_config.flag != FLAG_VALID || ret != length)
    {
        memcpy(&gConfigParam.patm_timer_config, &gDefaultPatmTimerConfig, length);
    }

    //--------Load Temp Timer Config ---------------------
    length = sizeof(temp_timer_config_t);
    ret = my_user_data_read(ZMS_ID_TEMP_TIMER_CONFIG, &gConfigParam.temp_timer_config, length);
    if (gConfigParam.temp_timer_config.flag != FLAG_VALID || ret != length)
    {
        memcpy(&gConfigParam.temp_timer_config, &gDefaultTempTimerConfig, length);
    }

    //--------Load Imu Alm Config ---------------------
    length = sizeof(imu_alm_config_t);
    ret = my_user_data_read(ZMS_ID_IMU_ALM_CONFIG, &gConfigParam.imu_alm_config, length);
    if (gConfigParam.imu_alm_config.flag != FLAG_VALID || ret != length)
    {
        memcpy(&gConfigParam.imu_alm_config, &gDefaultImuAlmConfig, length);
    }

    //--------Load Imu Zero Bias Config ---------------------
    length = sizeof(imu_zero_bias_config_t);
    ret = my_user_data_read(ZMS_ID_IMU_ZERO_BIAS_CONFIG, &gConfigParam.imu_zero_bias_config, length);
    if (gConfigParam.imu_zero_bias_config.flag != FLAG_VALID || ret != length)
    {
        memcpy(&gConfigParam.imu_zero_bias_config, &gDefaultImuZeroBiasConfig, length);
    }
}

/********************************************************************
**函数名称:  my_param_factory_reset
**入口参数:  无
**出口参数:  无
**函数功能:  重置所有参数为出厂值
*********************************************************************/
int my_param_factory_reset(void)
{
    int ret;

    MY_LOG_INF("Factory reset started");

    /* 确保ZMS文件系统已初始化 */
    ret = my_user_data_storage_init();
    if (ret != 0)
    {
        MY_LOG_ERR("Storage init failed: %d", ret);
        return ret;
    }

    /* 清除整个ZMS分区 */
    ret = zms_clear(&s_user_data_fs);
    if (ret < 0)
    {
        MY_LOG_ERR("ZMS clear failed: %d", ret);
        return ret;
    }

    return 0;
}

/********************************************************************
**函数名称:  my_param_check_license
**入口参数:  param: 要检查的许可证数据, len: 数据长度, id: 许可证ID
**出口参数:  无
**函数功能:  检查许可证数据是否有效
**返 回 值:  true表示有效，false表示无效
*********************************************************************/
bool my_param_check_license(char *param, uint8_t len, my_zms_id_t id)
{
    switch (id)
    {
    case ZMS_ID_FF:
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
        break;

    case ZMS_ID_GG:
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
        break;

    case ZMS_ID_SN:
        if (len != GSM_SN_LENGTH)
        {
            MY_LOG_INF("my_param_set_sn len error!");
            return false;
        }

        if (string_check_is_hex_str((const char *)param) != GSM_SN_LENGTH)
        {
            MY_LOG_INF("invalid param");
            return false;
        }
        break;

    default:
        MY_LOG_INF("invalid id");
        return false;
    }

    return true;
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
    int lic_stuct_len = sizeof(lic_ff_t);

    if (!my_param_check_license(param, len, ZMS_ID_FF))
    {
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
const lic_ff_t *my_param_get_ff(void)
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
    int lic_stuct_len = sizeof(lic_gg_t);

    if (!my_param_check_license(param, len, ZMS_ID_GG))
    {
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
const lic_gg_t *my_param_get_gg(void)
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
    int valid_len = sizeof(adv_valid_value_t);
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
    gConfigParam.ECDH_GValue.flag = FLAG_VALID;
    gConfigParam.ECDH_GValue.ecdh_g = Gvalue;

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
    return gConfigParam.ECDH_GValue.ecdh_g;
}

/********************************************************************
**函数名称:  my_param_set_sn
**入口参数:  param: 要设置的设备序列号SN, len: 数据长度
**出口参数:  无
**函数功能:  设置设备序列号SN
**返 回 值:  0表示成功，-1表示参数非法，-2表示写入失败
*********************************************************************/
int my_param_set_sn(char *param, uint8_t len)
{
    int ret;
    int GsmSn_struct_len = sizeof(gsm_sn_t);

    if (!my_param_check_license(param, len, ZMS_ID_SN))
    {
        return -1;
    }

    gConfigParam.gsm_sn.flag = FLAG_VALID;
    memcpy(gConfigParam.gsm_sn.hex, param, len);

    ret = my_user_data_write(ZMS_ID_SN, &gConfigParam.gsm_sn, GsmSn_struct_len);
    if (ret != GsmSn_struct_len)
    {
        MY_LOG_INF("zms set sn Error!!!");
        return -2;
    }
    else
    {
        MY_LOG_INF("zms set sn OK!!!");
    }

    return 0;
}

/********************************************************************
**函数名称:  my_param_get_sn
**入口参数:  无
**出口参数:  无
**函数功能:  获取SN配置数据
**返 回 值:  返回SN结构体指针
*********************************************************************/
const gsm_sn_t *my_param_get_sn(void)
{
    return &gConfigParam.gsm_sn;
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
int my_param_set_ble_log_config(const ble_log_config_t *config)
{
    int ret;
    int config_len = sizeof(ble_log_config_t);

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
ble_log_config_t *my_param_get_ble_log_config(void)
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
    ble_log_config_t *config;

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
    ble_log_config_t *config;

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
    ble_log_config_t *config;

    if (mod_id >= BLE_LOG_MOD_MAX)
    {
        return -EINVAL;
    }

    config = my_param_get_ble_log_config();
    config->mod_level[mod_id] = level;

    return my_param_set_ble_log_config(config);
}