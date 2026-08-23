/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        lr1121_lora_api.c
**文件描述:        LR1121 LoRaWAN 统一接口实现文件
**当前版本:        V1.0
**作    者:        周森达 (zhousenda@jimiiot.com)
**完成日期:        2026.08.24
*********************************************************************
** 功能描述:       封装 Modem-E 驱动，提供 LoRaWAN 业务调用接口
*********************************************************************/

#include "lr1121_lora_api.h"

/* 板级适配层，隔离业务层与底层 Modem-E 原始接口。 */

/*********************************************************************
**函数名称:  lr1121_lora_reset
**入口参数:  无
**出口参数:  无
**函数功能:  通过 Modem-E HAL 复位板载 LR1121
**返 回 值:  Modem-E HAL 状态码
*********************************************************************/
modem_e_modem_hal_status_t lr1121_lora_reset(void)
{
    return modem_e_modem_hal_reset(NULL);
}

/*********************************************************************
**函数名称:  lr1121_lora_get_version
**入口参数:  version -- 用于保存 Modem-E 系统版本信息的指针
**出口参数:  version -- 返回 Modem-E 系统版本信息
**函数功能:  读取 Modem-E 系统版本
**返 回 值:  Modem-E 响应码
*********************************************************************/
modem_e_response_code_t lr1121_lora_get_version(modem_e_system_version_t* version)
{
    if (version == NULL)
    {
        return MODEM_E_RESPONSE_CODE_INVALID;
    }

    return modem_e_system_get_version(NULL, version);
}

/*********************************************************************
**函数名称:  lr1121_lora_get_lorawan_version
**入口参数:  version -- 用于保存 LoRaWAN 和 RP002 版本信息的指针
**出口参数:  version -- 返回 LoRaWAN 和 RP002 版本信息
**函数功能:  读取 Modem-E 的 LoRaWAN 和 RP002 版本
**返 回 值:  Modem-E 响应码
*********************************************************************/
modem_e_response_code_t lr1121_lora_get_lorawan_version(modem_e_lorawan_version_t* version)
{
    if (version == NULL)
    {
        return MODEM_E_RESPONSE_CODE_INVALID;
    }

    return modem_e_get_lorawan_version(NULL, version);
}

/*********************************************************************
**函数名称:  lr1121_lora_get_link_check_data
**入口参数:  margin -- 用于保存最近一次 Link Check 链路裕量的指针
**           gateway_count -- 用于保存收到最近一次 Link Check 的网关数量的指针
**出口参数:  margin -- 返回链路裕量，单位为 dB
**           gateway_count -- 返回网关数量
**函数功能:  读取最近一次 Link Check 的链路质量结果
**返 回 值:  Modem-E 响应码
*********************************************************************/
modem_e_response_code_t lr1121_lora_get_link_check_data(uint8_t* margin,
    uint8_t* gateway_count)
{
    if ((margin == NULL) || (gateway_count == NULL))
    {
        return MODEM_E_RESPONSE_CODE_INVALID;
    }

    return modem_e_get_link_check_data(NULL, margin, gateway_count);
}

/*********************************************************************
**函数名称:  lr1121_lora_get_class
**入口参数:  modem_class -- 用于保存当前 LoRaWAN Class 的指针
**出口参数:  modem_class -- 返回当前 LoRaWAN Class
**函数功能:  读取当前 LoRaWAN Class
**返 回 值:  Modem-E 响应码
*********************************************************************/
modem_e_response_code_t lr1121_lora_get_class(modem_e_classes_t* modem_class)
{
    if (modem_class == NULL)
    {
        return MODEM_E_RESPONSE_CODE_INVALID;
    }

    return modem_e_get_class(NULL, modem_class);
}

/*********************************************************************
**函数名称:  lr1121_lora_set_class
**入口参数:  modem_class -- 要设置的 LoRaWAN Class，仅支持 Class A、B、C
**出口参数:  无
**函数功能:  设置当前 LoRaWAN Class
**返 回 值:  Modem-E 响应码
*********************************************************************/
modem_e_response_code_t lr1121_lora_set_class(modem_e_classes_t modem_class)
{
    if ((modem_class != MODEM_E_LORAWAN_CLASS_A) &&
        (modem_class != MODEM_E_LORAWAN_CLASS_B) &&
        (modem_class != MODEM_E_LORAWAN_CLASS_C))
    {
        return MODEM_E_RESPONSE_CODE_INVALID;
    }

    return modem_e_set_class(NULL, modem_class);
}

/*********************************************************************
**函数名称:  lr1121_lora_configure_otaa
**入口参数:  dev_eui -- OTAA 设备 EUI 指针
**           join_eui -- OTAA Join EUI 指针
**           app_key -- OTAA 网络密钥指针
**出口参数:  无
**函数功能:  按顺序配置 LoRaWAN 区域和 OTAA 凭据
**返 回 值:  Modem-E 响应码
*********************************************************************/
modem_e_response_code_t lr1121_lora_configure_otaa(const uint8_t* dev_eui,
    const uint8_t* join_eui, const uint8_t* app_key)
{
    modem_e_response_code_t response;

    if ((dev_eui == NULL) || (join_eui == NULL) || (app_key == NULL))
    {
        return MODEM_E_RESPONSE_CODE_INVALID;
    }

    response = modem_e_set_region(NULL, MODEM_E_LORAWAN_REGION_EU868);
    if (response == MODEM_E_RESPONSE_CODE_OK)
    {
        response = modem_e_set_otaa_dev_eui(NULL, dev_eui);
    }

    if (response == MODEM_E_RESPONSE_CODE_OK)
    {
        response = modem_e_set_otaa_join_eui(NULL, join_eui);
    }

    if (response == MODEM_E_RESPONSE_CODE_OK)
    {
        response = modem_e_set_otaa_nwk_key(NULL, app_key);
    }

    return response;
}

/*********************************************************************
**函数名称:  lr1121_lora_join
**入口参数:  无
**出口参数:  无
**函数功能:  发起一次 OTAA 入网尝试
**返 回 值:  Modem-E 响应码
*********************************************************************/
modem_e_response_code_t lr1121_lora_join(void)
{
    return modem_e_join(NULL);
}

/*********************************************************************
**函数名称:  lr1121_lora_leave_network
**入口参数:  无
**出口参数:  无
**函数功能:  退出当前 LoRaWAN 网络并停止进行中的入网流程
**返 回 值:  Modem-E 响应码
*********************************************************************/
modem_e_response_code_t lr1121_lora_leave_network(void)
{
    return modem_e_leave_network(NULL);
}

/*********************************************************************
**函数名称:  lr1121_lora_get_event
**入口参数:  event -- 用于保存 Modem-E 事件的指针
**出口参数:  event -- 返回一条待处理的 Modem-E 事件
**函数功能:  读取一条待处理的 Modem-E 事件
**返 回 值:  Modem-E 响应码
*********************************************************************/
modem_e_response_code_t lr1121_lora_get_event(modem_e_event_fields_t* event)
{
    if (event == NULL)
    {
        return MODEM_E_RESPONSE_CODE_INVALID;
    }

    return modem_e_get_event(NULL, event);
}

/*********************************************************************
**函数名称:  lr1121_lora_request_tx
**入口参数:  fport -- LoRaWAN 应用端口
**           uplink_type -- 上行数据类型
**           payload -- 上行数据缓冲区指针
**           payload_length -- 上行数据长度
**出口参数:  无
**函数功能:  提交一条 LoRaWAN 上行请求
**返 回 值:  Modem-E 响应码
*********************************************************************/
modem_e_response_code_t lr1121_lora_request_tx(uint8_t fport, modem_e_uplink_type_t uplink_type,
    const uint8_t* payload, uint8_t payload_length)
{
    if ((payload == NULL) || (payload_length == 0U))
    {
        return MODEM_E_RESPONSE_CODE_INVALID;
    }

    return modem_e_request_tx(NULL, fport, uplink_type, payload, payload_length);
}

/*********************************************************************
**函数名称:  lr1121_lora_get_downlink_data_size
**入口参数:  payload_size -- 用于保存下行数据长度的指针
**           remaining -- 用于保存剩余下行数据数量的指针
**出口参数:  payload_size -- 返回下行数据长度
**           remaining -- 返回剩余下行数据数量
**函数功能:  读取下一条下行数据长度及剩余数据包数量
**返 回 值:  Modem-E 响应码
*********************************************************************/
modem_e_response_code_t lr1121_lora_get_downlink_data_size(uint8_t* payload_size, uint8_t* remaining)
{
    if ((payload_size == NULL) || (remaining == NULL))
    {
        return MODEM_E_RESPONSE_CODE_INVALID;
    }

    return modem_e_get_downlink_data_size(NULL, payload_size, remaining);
}

/*********************************************************************
**函数名称:  lr1121_lora_get_downlink_data
**入口参数:  payload -- 用于保存下行数据的缓冲区指针
**           payload_size -- 下行数据长度
**出口参数:  payload -- 返回下行数据载荷
**函数功能:  读取下一条下行数据载荷
**返 回 值:  Modem-E 响应码
*********************************************************************/
modem_e_response_code_t lr1121_lora_get_downlink_data(uint8_t* payload, uint8_t payload_size)
{
    if ((payload == NULL) && (payload_size != 0U))
    {
        return MODEM_E_RESPONSE_CODE_INVALID;
    }

    return modem_e_get_downlink_data(NULL, payload, payload_size);
}

/*********************************************************************
**函数名称:  lr1121_lora_get_downlink_metadata
**入口参数:  metadata -- 用于保存下行数据元数据的指针
**出口参数:  metadata -- 返回最近一次下行数据对应的元数据
**函数功能:  读取最近一次下行数据对应的元数据
**返 回 值:  Modem-E 响应码
*********************************************************************/
modem_e_response_code_t lr1121_lora_get_downlink_metadata(modem_e_downlink_metadata_t* metadata)
{
    if (metadata == NULL)
    {
        return MODEM_E_RESPONSE_CODE_INVALID;
    }

    return modem_e_get_downlink_metadata(NULL, metadata);
}
