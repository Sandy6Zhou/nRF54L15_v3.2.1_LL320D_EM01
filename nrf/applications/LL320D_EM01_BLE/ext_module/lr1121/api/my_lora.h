#ifndef MY_LORA_H
#define MY_LORA_H

#include <stdbool.h>
#include <stdint.h>

#ifndef MY_LORA_SHELL_TEST_ENABLE
#define MY_LORA_SHELL_TEST_ENABLE    1
#endif

#define MY_LORA_MAX_PAYLOAD_LENGTH    70U

/* 1: 发送完整 A0 协议帧；0: 发送短 TEST 帧，用于 LoRaWAN 压力测试。 */
#ifndef MY_LORA_A0_TEST_PAYLOAD_ENABLE
#define MY_LORA_A0_TEST_PAYLOAD_ENABLE    0
#endif

int my_lora_init(void);
void my_lora_poll(void);

/*
 * 发送原始 LoRaWAN 应用载荷。payload 由调用方持有，函数返回后可立即复用；
 * 只能从应用线程调用，长度范围为 1..MY_LORA_MAX_PAYLOAD_LENGTH。
 */
int my_lora_send_payload(const uint8_t *payload, uint8_t payload_length);

#if MY_LORA_SHELL_TEST_ENABLE
typedef struct
{
    bool modem_detected;
    bool credentials_provisioned;
    bool joined;
    uint8_t hardware_version;
    uint16_t firmware_version;
} my_lora_status_t;

void my_lora_get_status(my_lora_status_t *status);
int my_lora_request_test_uplink(void);
#endif

#endif
