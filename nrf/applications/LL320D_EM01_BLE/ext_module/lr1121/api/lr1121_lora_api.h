#ifndef LR1121_LORA_API_H
#define LR1121_LORA_API_H

#include <stdint.h>
#include <stddef.h>

#include "modem_e_lorawan.h"
#include "modem_e_modem.h"
#include "modem_e_modem_hal.h"
#include "modem_e_system.h"

/* 板级适配接口，所有调用均使用本板注册的 Modem-E 实例。 */
/** 复位 LR1121 Modem-E。 */
modem_e_modem_hal_status_t lr1121_lora_reset(void);
/** 读取 Modem-E 系统版本信息。 */
modem_e_response_code_t lr1121_lora_get_version(modem_e_system_version_t* version);
/** 读取 Modem-E 的 LoRaWAN 和 RP002 版本信息。 */
modem_e_response_code_t lr1121_lora_get_lorawan_version(modem_e_lorawan_version_t* version);
/** 配置 Modem-E 区域和 OTAA 凭据。 */
modem_e_response_code_t lr1121_lora_configure_otaa(const uint8_t* dev_eui,
    const uint8_t* join_eui,
    const uint8_t* app_key);
/** 发起一次 OTAA 入网尝试。 */
modem_e_response_code_t lr1121_lora_join(void);
/** 退出当前 LoRaWAN 网络，停止进行中的入网流程。 */
modem_e_response_code_t lr1121_lora_leave_network(void);
/** 获取一条待处理的 Modem-E 事件。 */
modem_e_response_code_t lr1121_lora_get_event(modem_e_event_fields_t* event);
/** 请求发送一条 LoRaWAN 上行。 */
modem_e_response_code_t lr1121_lora_request_tx(uint8_t fport,
    modem_e_uplink_type_t uplink_type,
    const uint8_t* payload,
    uint8_t payload_length);
/** 读取待处理下行长度和剩余队列数量。 */
modem_e_response_code_t lr1121_lora_get_downlink_data_size(uint8_t* payload_size,
    uint8_t* remaining);
/** 读取待处理下行数据。 */
modem_e_response_code_t lr1121_lora_get_downlink_data(uint8_t* payload,
    uint8_t payload_size);
/** 读取最近获取的下行数据元数据。 */
modem_e_response_code_t lr1121_lora_get_downlink_metadata(modem_e_downlink_metadata_t* metadata);

#endif
