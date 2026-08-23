#ifndef MY_LORA_H
#define MY_LORA_H

#ifndef MY_LORA_SHELL_TEST_ENABLE
#define MY_LORA_SHELL_TEST_ENABLE 1
#endif

#define MY_LORA_MAX_PAYLOAD_LENGTH 70U

/* 1: 发送完整 A0 协议帧；0: 发送短 TEST 帧，用于 LoRaWAN 压力测试。 */
#ifndef MY_LORA_A0_TEST_PAYLOAD_ENABLE
#define MY_LORA_A0_TEST_PAYLOAD_ENABLE 1
#endif

/** 初始化 LoRaWAN 所有者线程，应用启动时调用一次。 */
int my_lora_init(void);

/** 异步提交开启请求，由 LoRa 线程串行处理。 */
int my_lora_enable(void);

/** 异步提交关闭请求，由 LoRa 线程串行处理。 */
int my_lora_disable(void);

/*
 * 发送原始 LoRaWAN 应用载荷。payload 由调用方持有，函数返回后可立即复用；
 * 只能由 LoRa owner thread 调用，长度范围为 1..MY_LORA_MAX_PAYLOAD_LENGTH；
 * 返回 0 表示已提交到底层，负值表示参数、状态或 Modem-E 错误。
 */
int my_lora_send_payload(const uint8_t* payload, uint8_t payload_length);

#if MY_LORA_SHELL_TEST_ENABLE
/** 异步提交测试上行请求，shell 不直接访问 Modem-E。 */
int my_lora_request_test_uplink(void);
#endif

#endif
