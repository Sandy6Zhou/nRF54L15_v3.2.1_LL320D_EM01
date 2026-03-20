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
static int my_user_data_write(uint32_t id, const void *data, int len)
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
**函数名称:  my_param_set_ble_tx_power
**入口参数:  tx_power: 发射功率(dBm)，范围: -40 ~ +8
**出口参数:  无
**函数功能:  设置蓝牙发射功率参数
**返 回 值:  0表示成功，负值表示失败
*********************************************************************/
int my_param_set_ble_tx_power(int8_t tx_power)
{
    int ret;
    int tx_power_len = sizeof(BleTxPower_t);

    /* 限制功率范围: -40 ~ +8 dBm */
    if (tx_power < -40)
    {
        tx_power = -40;
    }
    else if (tx_power > 8)
    {
        tx_power = 8;
    }

    gConfigParam.ble_tx_power.flag = FLAG_VALID;
    gConfigParam.ble_tx_power.tx_power = tx_power;

    ret = my_user_data_write(ZMS_ID_BLE_TX_POWER, &gConfigParam.ble_tx_power, tx_power_len);
    if (ret != tx_power_len)
    {
        MY_LOG_INF("zms set ble tx power Error!!!");
        return -1;
    }
    else
    {
        MY_LOG_INF("zms set ble tx power OK: %d dBm", tx_power);
    }

    return 0;
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
