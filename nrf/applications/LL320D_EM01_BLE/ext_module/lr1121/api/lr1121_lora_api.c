#include "lr1121_lora_api.h"

/* 板级适配层，隔离业务层与底层 Modem-E 原始接口。 */

/** 通过 Modem-E HAL 复位板载 LR1121。 */
modem_e_modem_hal_status_t lr1121_lora_reset(void)
{
    return modem_e_modem_hal_reset(NULL);
}

/** 读取 Modem-E 系统版本。 */
modem_e_response_code_t lr1121_lora_get_version(modem_e_system_version_t* version)
{
    return modem_e_system_get_version(NULL, version);
}

/** 读取 Modem-E 的 LoRaWAN 和 RP002 版本。 */
modem_e_response_code_t lr1121_lora_get_lorawan_version(modem_e_lorawan_version_t* version)
{
    return modem_e_get_lorawan_version(NULL, version);
}

/** 按顺序配置区域和 OTAA 凭据。 */
modem_e_response_code_t lr1121_lora_configure_otaa(const uint8_t* dev_eui,
    const uint8_t* join_eui,
    const uint8_t* app_key)
{
    modem_e_response_code_t response;

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

/** 发起一次 OTAA 入网尝试。 */
modem_e_response_code_t lr1121_lora_join(void)
{
    return modem_e_join(NULL);
}

/** 退出当前 LoRaWAN 网络，停止进行中的入网流程。 */
modem_e_response_code_t lr1121_lora_leave_network(void)
{
    return modem_e_leave_network(NULL);
}

/** 读取一条待处理的 Modem-E 事件。 */
modem_e_response_code_t lr1121_lora_get_event(modem_e_event_fields_t* event)
{
    return modem_e_get_event(NULL, event);
}

/** 提交一条 LoRaWAN 上行请求。 */
modem_e_response_code_t lr1121_lora_request_tx(uint8_t fport,
    modem_e_uplink_type_t uplink_type,
    const uint8_t* payload,
    uint8_t payload_length)
{
    return modem_e_request_tx(NULL, fport, uplink_type, payload, payload_length);
}

/** 读取下一条下行数据长度及剩余数据包数量。 */
modem_e_response_code_t lr1121_lora_get_downlink_data_size(uint8_t* payload_size,
    uint8_t* remaining)
{
    return modem_e_get_downlink_data_size(NULL, payload_size, remaining);
}

/** 读取下一条下行数据载荷。 */
modem_e_response_code_t lr1121_lora_get_downlink_data(uint8_t* payload,
    uint8_t payload_size)
{
    return modem_e_get_downlink_data(NULL, payload, payload_size);
}

/** 读取最近一次下行数据对应的元数据。 */
modem_e_response_code_t lr1121_lora_get_downlink_metadata(modem_e_downlink_metadata_t* metadata)
{
    return modem_e_get_downlink_metadata(NULL, metadata);
}
