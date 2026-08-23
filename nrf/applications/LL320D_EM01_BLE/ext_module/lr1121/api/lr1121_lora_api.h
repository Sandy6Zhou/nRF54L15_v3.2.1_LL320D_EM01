/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        lr1121_lora_api.h
**文件描述:        LR1121 LoRaWAN 统一接口头文件
**当前版本:        V1.0
**作    者:        周森达 (zhousenda@jimiiot.com)
**完成日期:        2026.08.24
*********************************************************************
** 功能描述:       声明 LR1121 LoRaWAN 配置、收发和状态查询接口
*********************************************************************/

#ifndef LR1121_LORA_API_H
#define LR1121_LORA_API_H

#include <stdint.h>
#include <stddef.h>

#include "modem_e_lorawan.h"
#include "modem_e_modem.h"
#include "modem_e_modem_hal.h"
#include "modem_e_system.h"

/* 板级适配接口，所有调用均使用本板注册的 Modem-E 实例。 */
/*********************************************************************
**函数名称:  lr1121_lora_reset
**入口参数:  无
**出口参数:  无
**函数功能:  复位板载 LR1121 Modem-E
**返 回 值:  Modem-E HAL 状态码
*********************************************************************/
modem_e_modem_hal_status_t lr1121_lora_reset(void);
/*********************************************************************
**函数名称:  lr1121_lora_get_version
**入口参数:  version -- 用于保存 Modem-E 系统版本信息的指针
**出口参数:  version -- 返回 Modem-E 系统版本信息
**函数功能:  读取 Modem-E 系统版本信息
**返 回 值:  Modem-E 响应码
*********************************************************************/
modem_e_response_code_t lr1121_lora_get_version(modem_e_system_version_t* version);
/*********************************************************************
**函数名称:  lr1121_lora_get_lorawan_version
**入口参数:  version -- 用于保存 LoRaWAN 和 RP002 版本信息的指针
**出口参数:  version -- 返回 LoRaWAN 和 RP002 版本信息
**函数功能:  读取 Modem-E 的 LoRaWAN 和 RP002 版本信息
**返 回 值:  Modem-E 响应码
*********************************************************************/
modem_e_response_code_t lr1121_lora_get_lorawan_version(modem_e_lorawan_version_t* version);
/*********************************************************************
**函数名称:  lr1121_lora_get_link_check_data
**入口参数:  margin -- 用于保存最近一次 Link Check 链路裕量的指针
**           gateway_count -- 用于保存收到最近一次 Link Check 的网关数量的指针
**出口参数:  margin -- 返回链路裕量，单位为 dB
**           gateway_count -- 返回网关数量
**函数功能:  读取最近一次 Link Check 的链路质量结果
**返 回 值:  Modem-E 响应码
*********************************************************************/
modem_e_response_code_t lr1121_lora_get_link_check_data(uint8_t* margin, uint8_t* gateway_count);
/*********************************************************************
**函数名称:  lr1121_lora_get_class
**入口参数:  modem_class -- 用于保存当前 LoRaWAN Class 的指针
**出口参数:  modem_class -- 返回当前 LoRaWAN Class
**函数功能:  读取当前 LoRaWAN Class
**返 回 值:  Modem-E 响应码
*********************************************************************/
modem_e_response_code_t lr1121_lora_get_class(modem_e_classes_t* modem_class);
/*********************************************************************
**函数名称:  lr1121_lora_set_class
**入口参数:  modem_class -- 要设置的 LoRaWAN Class，仅支持 Class A、B、C
**出口参数:  无
**函数功能:  设置当前 LoRaWAN Class
**返 回 值:  Modem-E 响应码
*********************************************************************/
modem_e_response_code_t lr1121_lora_set_class(modem_e_classes_t modem_class);
/*********************************************************************
**函数名称:  lr1121_lora_configure_otaa
**入口参数:  dev_eui -- OTAA 设备 EUI 指针
**           join_eui -- OTAA Join EUI 指针
**           app_key -- OTAA 网络密钥指针
**出口参数:  无
**函数功能:  配置 LoRaWAN 区域和 OTAA 凭据
**返 回 值:  Modem-E 响应码
*********************************************************************/
modem_e_response_code_t lr1121_lora_configure_otaa(const uint8_t* dev_eui, const uint8_t* join_eui, const uint8_t* app_key);
/*********************************************************************
**函数名称:  lr1121_lora_join
**入口参数:  无
**出口参数:  无
**函数功能:  发起一次 OTAA 入网尝试
**返 回 值:  Modem-E 响应码
*********************************************************************/
modem_e_response_code_t lr1121_lora_join(void);
/*********************************************************************
**函数名称:  lr1121_lora_leave_network
**入口参数:  无
**出口参数:  无
**函数功能:  退出当前 LoRaWAN 网络并停止进行中的入网流程
**返 回 值:  Modem-E 响应码
*********************************************************************/
modem_e_response_code_t lr1121_lora_leave_network(void);
/*********************************************************************
**函数名称:  lr1121_lora_get_event
**入口参数:  event -- 用于保存 Modem-E 事件的指针
**出口参数:  event -- 返回一条待处理的 Modem-E 事件
**函数功能:  获取一条待处理的 Modem-E 事件
**返 回 值:  Modem-E 响应码
*********************************************************************/
modem_e_response_code_t lr1121_lora_get_event(modem_e_event_fields_t* event);
/*********************************************************************
**函数名称:  lr1121_lora_request_tx
**入口参数:  fport -- LoRaWAN 应用端口
**           uplink_type -- 上行数据类型
**           payload -- 上行数据缓冲区指针
**           payload_length -- 上行数据长度
**出口参数:  无
**函数功能:  请求发送一条 LoRaWAN 上行数据
**返 回 值:  Modem-E 响应码
*********************************************************************/
modem_e_response_code_t lr1121_lora_request_tx(uint8_t fport,
    modem_e_uplink_type_t uplink_type,
    const uint8_t* payload,
    uint8_t payload_length);
/*********************************************************************
**函数名称:  lr1121_lora_get_downlink_data_size
**入口参数:  payload_size -- 用于保存下行数据长度的指针
**           remaining -- 用于保存剩余下行数据数量的指针
**出口参数:  payload_size -- 返回下行数据长度
**           remaining -- 返回剩余下行数据数量
**函数功能:  读取待处理下行数据长度和剩余队列数量
**返 回 值:  Modem-E 响应码
*********************************************************************/
modem_e_response_code_t lr1121_lora_get_downlink_data_size(uint8_t* payload_size,
    uint8_t* remaining);
/*********************************************************************
**函数名称:  lr1121_lora_get_downlink_data
**入口参数:  payload -- 用于保存下行数据的缓冲区指针
**           payload_size -- 下行数据长度
**出口参数:  payload -- 返回下行数据
**函数功能:  读取一条待处理的下行数据
**返 回 值:  Modem-E 响应码
*********************************************************************/
modem_e_response_code_t lr1121_lora_get_downlink_data(uint8_t* payload,
    uint8_t payload_size);
/*********************************************************************
**函数名称:  lr1121_lora_get_downlink_metadata
**入口参数:  metadata -- 用于保存下行数据元数据的指针
**出口参数:  metadata -- 返回最近获取的下行数据元数据
**函数功能:  读取最近获取的下行数据元数据
**返 回 值:  Modem-E 响应码
*********************************************************************/
modem_e_response_code_t lr1121_lora_get_downlink_metadata(modem_e_downlink_metadata_t* metadata);

#endif
