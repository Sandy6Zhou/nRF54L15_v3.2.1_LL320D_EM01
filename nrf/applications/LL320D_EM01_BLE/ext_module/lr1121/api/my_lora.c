#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "modem_e_lorawan.h"
#include "modem_e_modem.h"
#include "modem_e_modem_hal.h"
#include "modem_e_system.h"
#include "my_comm.h"
#include "my_lora.h"
LOG_MODULE_REGISTER(my_lora, LOG_LEVEL_INF);

#define MY_LORA_A0_TEST_PAYLOAD_LENGTH    47U
#define MY_LORA_STRESS_TEST_PAYLOAD_LENGTH    4U

static const uint8_t s_stress_test_payload[MY_LORA_STRESS_TEST_PAYLOAD_LENGTH] =
{
    'T', 'E', 'S', 'T'
};

static const uint8_t s_a0_test_payload[MY_LORA_A0_TEST_PAYLOAD_LENGTH] =
{
    0x78, 0x78, 0x2A, 0xA0, 0x1A, 0x08, 0x14, 0x05,
    0x36, 0x21, 0xCF, 0x02, 0x6C, 0x15, 0x95, 0x0C,
    0x39, 0x8A, 0x49, 0x00, 0x14, 0x95, 0x81, 0xCC,
    0x00, 0x00, 0x00, 0x00, 0x24, 0x7F, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x90, 0xF8, 0x4F, 0x00, 0x00,
    0x00, 0x00, 0x11, 0xDF, 0xCE, 0x0D, 0x0A
};

typedef struct
{
    uint8_t dev_eui[8];
    uint8_t join_eui[8];
    uint8_t app_key[16];
    uint8_t uplink_fport;
    uint8_t downlink_fport;
    uint32_t uplink_interval_s;
    bool confirmed_uplink;
} my_lora_config_t;

/* Replace the zero credentials through the production provisioning path. */
static const my_lora_config_t s_lora_config =
{
    .dev_eui = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10},
    .join_eui = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x99},
    .app_key = {0x41, 0x78, 0xD6, 0x9D, 0x85, 0x56, 0xF2, 0x00,
                0x3F, 0x4D, 0xB5, 0x64, 0x1C, 0x48, 0x1B, 0x6D},
    .uplink_fport = 1,
    .downlink_fport = 1,
    .uplink_interval_s = 600,
    .confirmed_uplink = false,
};

static bool s_lora_joined = false;
static bool s_lora_modem_detected = false;
static int64_t s_next_uplink_ms;
static int64_t s_tx_request_ms;
static modem_e_system_version_t s_lora_version;

static const uint8_t *my_lora_get_test_payload(uint8_t *payload_length)
{
#if MY_LORA_A0_TEST_PAYLOAD_ENABLE
    *payload_length = sizeof(s_a0_test_payload);
    return s_a0_test_payload;
#else
    *payload_length = sizeof(s_stress_test_payload);
    return s_stress_test_payload;
#endif
}

static bool my_lora_is_provisioned(void)
{
    static const uint8_t empty_eui[8];
    static const uint8_t empty_key[16];

    return (memcmp(s_lora_config.dev_eui, empty_eui, sizeof(empty_eui)) != 0) &&
           (memcmp(s_lora_config.join_eui, empty_eui, sizeof(empty_eui)) != 0) &&
           (memcmp(s_lora_config.app_key, empty_key, sizeof(empty_key)) != 0);
}

static int my_lora_start_otaa(void)
{
    modem_e_response_code_t response;

    response = modem_e_set_region(NULL, MODEM_E_LORAWAN_REGION_EU868);
    if (response == MODEM_E_RESPONSE_CODE_OK)
    {
        response = modem_e_set_otaa_dev_eui(NULL, s_lora_config.dev_eui);
    }
    if (response == MODEM_E_RESPONSE_CODE_OK)
    {
        response = modem_e_set_otaa_join_eui(NULL, s_lora_config.join_eui);
    }
    if (response == MODEM_E_RESPONSE_CODE_OK)
    {
        response = modem_e_set_otaa_nwk_key(NULL, s_lora_config.app_key);
    }
    if (response == MODEM_E_RESPONSE_CODE_OK)
    {
        response = modem_e_join(NULL);
    }
    if (response != MODEM_E_RESPONSE_CODE_OK)
    {
        LOG_ERR("LoRaWAN OTAA setup failed: 0x%02x", response);
        return -EIO;
    }

    LOG_INF("LoRaWAN OTAA join started");
    return 0;
}

int my_lora_init(void)
{
    modem_e_system_version_t version = {0};
    modem_e_lorawan_version_t lorawan_version = {0};
    modem_e_response_code_t response;

    /* 等待LR1121完全稳定 */
    k_msleep(100);

    if (modem_e_modem_hal_reset(NULL) != MODEM_E_MODEM_HAL_STATUS_OK)
    {
        LOG_ERR("LR1121 hardware reset failed");
        return -EIO;
    }

    /* 复位后等待芯片完全启动 */
    k_msleep(50);

    response = modem_e_system_get_version(NULL, &version);
    if (response != MODEM_E_RESPONSE_CODE_OK)
    {
        LOG_ERR("LR1121 Modem-E version query failed: 0x%02x", response);
        return -EIO;
    }

    LOG_INF("Modem-E version response: hardware 0x%02x, type 0x%02x, firmware %u.%u",
            version.hw, version.type, version.fw >> 8, version.fw & 0xFF);

    response = modem_e_get_lorawan_version(NULL, &lorawan_version);
    if (response != MODEM_E_RESPONSE_CODE_OK)
    {
        LOG_ERR("LoRaWAN version query failed: 0x%02x", response);
        return -EIO;
    }

    LOG_INF("LoRaWAN version: %u.%u.%u.%u, RP002: %u.%u.%u.%u",
            lorawan_version.lorawan_major,
            lorawan_version.lorawan_minor,
            lorawan_version.lorawan_patch,
            lorawan_version.lorawan_revision,
            lorawan_version.rp_major,
            lorawan_version.rp_minor,
            lorawan_version.rp_patch,
            lorawan_version.rp_revision);

    // if (version.type != MODEM_E_SYSTEM_VERSION_TYPE_LR1121)
    // {
    //     LOG_ERR("Unexpected Modem-E device type: 0x%02x", version.type);
    //     return -ENODEV;
    // }

    s_lora_version = version;
    s_lora_modem_detected = true;

    if (!my_lora_is_provisioned())
    {
        LOG_WRN("LoRaWAN credentials are not provisioned; join and uplink are disabled");
        return 0;
    }

    return my_lora_start_otaa();
}

static void my_lora_schedule_uplink(void)
{
    const uint8_t *payload;
    uint8_t payload_length;

    if (!s_lora_joined || (k_uptime_get() < s_next_uplink_ms))
    {
        return;
    }

    payload = my_lora_get_test_payload(&payload_length);
    (void)my_lora_send_payload(payload, payload_length);
}

int my_lora_send_payload(const uint8_t *payload, uint8_t payload_length)
{
    modem_e_response_code_t response;

    if ((payload == NULL) || (payload_length == 0) ||
        (payload_length > MY_LORA_MAX_PAYLOAD_LENGTH))
    {
        return -EINVAL;
    }
    if (!s_lora_modem_detected)
    {
        return -ENODEV;
    }
    if (!my_lora_is_provisioned())
    {
        return -EACCES;
    }
    if (!s_lora_joined)
    {
        return -EAGAIN;
    }

    response = modem_e_request_tx(NULL, s_lora_config.uplink_fport,
                                  s_lora_config.confirmed_uplink ? MODEM_E_UPLINK_CONFIRMED :
                                                                    MODEM_E_UPLINK_UNCONFIRMED,
                                  payload, payload_length);
    if (response != MODEM_E_RESPONSE_CODE_OK)
    {
        LOG_WRN("LoRaWAN uplink request rejected: response=0x%02x, port=%u, length=%u",
                response, s_lora_config.uplink_fport, payload_length);
        return -EBUSY;
    }

    s_tx_request_ms = k_uptime_get();
    s_next_uplink_ms = k_uptime_get() +
                       ((int64_t)s_lora_config.uplink_interval_s * MSEC_PER_SEC);
    LOG_INF("LoRaWAN raw uplink queued: %u bytes", payload_length);
    LOG_HEXDUMP_INF(payload, payload_length, "LoRaWAN raw uplink payload");
    return 0;
}

#if MY_LORA_SHELL_TEST_ENABLE
void my_lora_get_status(my_lora_status_t *status)
{
    if (status == NULL)
    {
        return;
    }

    status->modem_detected = s_lora_modem_detected;
    status->credentials_provisioned = my_lora_is_provisioned();
    status->joined = s_lora_joined;
    status->hardware_version = s_lora_version.hw;
    status->firmware_version = s_lora_version.fw;
}

int my_lora_request_test_uplink(void)
{
    const uint8_t *payload;
    uint8_t payload_length;

    payload = my_lora_get_test_payload(&payload_length);
    return my_lora_send_payload(payload, payload_length);
}
#endif

void my_lora_poll(void)
{
    modem_e_event_fields_t modem_event;
    modem_e_response_code_t response;

    if (!my_lora_is_provisioned())
    {
        return;
    }

    response = modem_e_get_event(NULL, &modem_event);
    if (response == MODEM_E_RESPONSE_CODE_NO_EVENT)
    {
        my_lora_schedule_uplink();
        return;
    }
    if (response != MODEM_E_RESPONSE_CODE_OK)
    {
        LOG_ERR("LoRaWAN event read failed: 0x%02x", response);
        return;
    }

    switch (modem_event.event_type)
    {
    case MODEM_E_LORAWAN_EVENT_JOINED:
        s_lora_joined = true;
        s_next_uplink_ms = k_uptime_get();
        LOG_INF("LoRaWAN joined");
        break;

    case MODEM_E_LORAWAN_EVENT_JOIN_FAIL:
        LOG_WRN("LoRaWAN join attempt failed; Modem-E will retry");
        break;

    case MODEM_E_LORAWAN_EVENT_TX_DONE:
        if (s_tx_request_ms > 0)
        {
            LOG_INF("LoRaWAN uplink completed, request-to-tx-done=%lld ms",
                    k_uptime_get() - s_tx_request_ms);
            s_tx_request_ms = 0;
        }
        else
        {
            LOG_INF("LoRaWAN uplink completed without tracked request");
        }
        break;

    case MODEM_E_LORAWAN_EVENT_REGIONAL_DUTY_CYCLE:
        LOG_WRN("LoRaWAN regional duty-cycle event: data=0x%04x, missed=%u",
                modem_event.data, modem_event.missed_events_count);
        break;

    case MODEM_E_LORAWAN_EVENT_DOWN_DATA:
    {
        uint8_t payload[242];
        uint8_t payload_size;
        uint8_t remaining;
        modem_e_downlink_metadata_t metadata;

        response = modem_e_get_downlink_data_size(NULL, &payload_size, &remaining);
        if (response == MODEM_E_RESPONSE_CODE_OK)
        {
            response = modem_e_get_downlink_data(NULL, payload, payload_size);
        }
        if (response == MODEM_E_RESPONSE_CODE_OK)
        {
            response = modem_e_get_downlink_metadata(NULL, &metadata);
        }
        if (response != MODEM_E_RESPONSE_CODE_OK)
        {
            LOG_ERR("LoRaWAN downlink read failed: 0x%02x", response);
            break;
        }
        if (metadata.fport != s_lora_config.downlink_fport)
        {
            LOG_WRN("Ignoring downlink on FPort %u", metadata.fport);
            break;
        }

        LOG_HEXDUMP_INF(payload, payload_size, "LoRaWAN downlink command");
        break;
    }

    default:
        LOG_DBG("LoRaWAN event %u", modem_event.event_type);
        break;
    }

    my_lora_schedule_uplink();
}
