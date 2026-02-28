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

#define ZMS_ID_FF           0
#define ZMS_ID_GG           1
#define ZMS_ID_ADV_VALID    2
#define ZMS_ID_ECDH_G       3
#define ZMS_ID_IMEI         4
#define ZMS_ID_MAC          5

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

typedef struct
{
    lic_ff_struct               lic_ff;
    lic_gg_struct               lic_gg;
    AdvValidValue_t             adv_valid_value;
    uint16_t                    ECDH_GValue;
    GsmImei_t                   gsm_imei;
    macaddr_t                   my_macaddr;
    DeviceWorkModeConfig        *workmode_config;
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
#endif