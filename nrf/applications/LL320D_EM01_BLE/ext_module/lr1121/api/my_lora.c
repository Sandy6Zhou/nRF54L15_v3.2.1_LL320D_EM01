#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include "modem_e_lorawan.h"
#include "modem_e_modem.h"
#include "modem_e_modem_hal.h"
#include "modem_e_system.h"
#include "my_comm.h"
#include "my_lora.h"

LOG_MODULE_REGISTER(my_lora, LOG_LEVEL_INF);

#define MY_LORA_TRACKER_PAYLOAD_V1_LENGTH    17U

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
    .dev_eui = {0},
    .join_eui = {0},
    .app_key = {0},
    .uplink_fport = 1,
    .downlink_fport = 1,
    .uplink_interval_s = 600,
    .confirmed_uplink = false,
};

static bool s_lora_joined = false;
static bool s_lora_modem_detected = false;
static int64_t s_next_uplink_ms;
static modem_e_system_version_t s_lora_version;

static bool my_lora_is_provisioned(void)
{
    static const uint8_t empty_eui[8];
    static const uint8_t empty_key[16];

    return (memcmp(s_lora_config.dev_eui, empty_eui, sizeof(empty_eui)) != 0) &&
           (memcmp(s_lora_config.join_eui, empty_eui, sizeof(empty_eui)) != 0) &&
           (memcmp(s_lora_config.app_key, empty_key, sizeof(empty_key)) != 0);
}

static int my_lora_build_tracker_payload(uint8_t *payload, uint8_t max_length,
                                         uint8_t *payload_length)
{
    int32_t speed_cms;
    int8_t battery_percent;

    if ((max_length < MY_LORA_TRACKER_PAYLOAD_V1_LENGTH) || (g_location_point.timestamp_s <= 0))
    {
        return -ENODATA;
    }

    speed_cms = (int32_t)(g_location_point.speed * 100.0f);
    battery_percent = get_show_percent();

    payload[0] = 1;
    sys_put_be32((uint32_t)g_location_point.lat, &payload[1]);
    sys_put_be32((uint32_t)g_location_point.lon, &payload[5]);
    sys_put_be16((uint16_t)CLAMP(speed_cms, 0, UINT16_MAX), &payload[9]);
    payload[11] = (uint8_t)CLAMP(battery_percent, 0, 100);
    sys_put_be32((uint32_t)g_location_point.timestamp_s, &payload[12]);
    payload[16] = (g_lte_gps_state == 2) ? BIT(0) : 0;
    *payload_length = MY_LORA_TRACKER_PAYLOAD_V1_LENGTH;

    return 0;
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
    modem_e_response_code_t response;

    if (modem_e_modem_hal_reset(NULL) != MODEM_E_MODEM_HAL_STATUS_OK)
    {
        LOG_ERR("LR1121 hardware reset failed");
        return -EIO;
    }

    response = modem_e_system_get_version(NULL, &version);
    if (response != MODEM_E_RESPONSE_CODE_OK)
    {
        LOG_ERR("LR1121 Modem-E version query failed: 0x%02x", response);
        return -EIO;
    }

    LOG_INF("Modem-E version response: hardware 0x%02x, type 0x%02x, firmware %u.%u",
            version.hw, version.type, version.fw >> 8, version.fw & 0xFF);

    if (version.type != MODEM_E_SYSTEM_VERSION_TYPE_LR1121)
    {
        LOG_ERR("Unexpected Modem-E device type: 0x%02x", version.type);
        return -ENODEV;
    }

    s_lora_version = version;
    s_lora_modem_detected = true;

    LOG_INF("LR1121 Modem-E detected: hardware 0x%02x, firmware %u.%u",
            version.hw, version.fw >> 8, version.fw & 0xFF);

    if (!my_lora_is_provisioned())
    {
        LOG_WRN("LoRaWAN credentials are not provisioned; join and uplink are disabled");
        return 0;
    }

    return my_lora_start_otaa();
}

static void my_lora_schedule_uplink(void)
{
    uint8_t payload[242];
    uint8_t payload_length = 0;
    modem_e_response_code_t response;

    if (!s_lora_joined || (k_uptime_get() < s_next_uplink_ms))
    {
        return;
    }

    if ((my_lora_build_tracker_payload(payload, sizeof(payload), &payload_length) != 0) ||
        (payload_length == 0) || (payload_length > sizeof(payload)))
    {
        return;
    }

    response = modem_e_request_tx(NULL, s_lora_config.uplink_fport,
                                  s_lora_config.confirmed_uplink ? MODEM_E_UPLINK_CONFIRMED :
                                                                    MODEM_E_UPLINK_UNCONFIRMED,
                                  payload, payload_length);
    if (response == MODEM_E_RESPONSE_CODE_OK)
    {
        s_next_uplink_ms = k_uptime_get() +
                           ((int64_t)s_lora_config.uplink_interval_s * MSEC_PER_SEC);
        LOG_INF("LoRaWAN uplink queued: %u bytes", payload_length);
    }
    else
    {
        LOG_WRN("LoRaWAN uplink request rejected: 0x%02x", response);
    }
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
    static const uint8_t test_payload[] = {'T', 'E', 'S', 'T'};
    modem_e_response_code_t response;

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
                                  test_payload, sizeof(test_payload));
    if (response != MODEM_E_RESPONSE_CODE_OK)
    {
        LOG_WRN("LoRaWAN test uplink rejected: 0x%02x", response);
        return -EBUSY;
    }

    s_next_uplink_ms = k_uptime_get() +
                       ((int64_t)s_lora_config.uplink_interval_s * MSEC_PER_SEC);
    LOG_INF("LoRaWAN test uplink queued: TEST");
    return 0;
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
        LOG_INF("LoRaWAN uplink completed");
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
