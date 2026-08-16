#ifndef MY_LORA_H
#define MY_LORA_H

#include <stdbool.h>
#include <stdint.h>

#ifndef MY_LORA_SHELL_TEST_ENABLE
#define MY_LORA_SHELL_TEST_ENABLE    1
#endif

int my_lora_init(void);
void my_lora_poll(void);

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
