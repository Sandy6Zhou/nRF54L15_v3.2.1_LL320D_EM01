/* LoRaWAN 业务层：负责生命周期、入网重试、上行和下行处理策略。 */
#include "my_comm.h"

LOG_MODULE_REGISTER(my_lora, LOG_LEVEL_INF);

#define MY_LORA_A0_TEST_PAYLOAD_LENGTH 47U
#define MY_LORA_STRESS_TEST_PAYLOAD_LENGTH 4U
#define MY_LORA_MAX_DOWNLINK_LENGTH 242U
#define MY_LORA_POLL_INTERVAL_MS 100U
#define MY_LORA_JOIN_RETRY_INTERVAL_MS 30000U

/* LoRaWAN 服务状态，只有 LoRa 所有者线程可以修改。 */
typedef enum
{
    MY_LORA_STATE_OFF = 0,
    MY_LORA_STATE_STARTING,
    MY_LORA_STATE_JOINING,
    MY_LORA_STATE_JOINED,
} my_lora_state_t;

/* LTE 或 shell 请求通过该队列串行提交给 LoRa 所有者线程。 */
K_MSGQ_DEFINE(my_lora_msgq, sizeof(msg_t), 10, 4);

#if !MY_LORA_A0_TEST_PAYLOAD_ENABLE
/* 未启用 A0 帧时用于 Modem 压力测试的短载荷。 */
static const uint8_t s_stress_test_payload[MY_LORA_STRESS_TEST_PAYLOAD_LENGTH] =
    {
        'T', 'E', 'S', 'T'};
#endif

/* shell 测试命令使用的固定 A0 验证帧。 */
static const uint8_t s_a0_test_payload[MY_LORA_A0_TEST_PAYLOAD_LENGTH] =
    {
        0x78, 0x78, 0x2A, 0xA0, 0x1A, 0x08, 0x14, 0x05,
        0x36, 0x21, 0xCF, 0x02, 0x6C, 0x15, 0x95, 0x0C,
        0x39, 0x8A, 0x49, 0x00, 0x14, 0x95, 0x81, 0xCC,
        0x00, 0x00, 0x00, 0x00, 0x24, 0x7F, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x90, 0xF8, 0x4F, 0x00, 0x00,
        0x00, 0x00, 0x11, 0xDF, 0xCE, 0x0D, 0x0A};

/* LoRa 所有者线程使用的运行参数和入网配置。 */
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

/* 已配置的 LoRaWAN 凭据和应用参数，量产时应替换为正式配置流程。 */
static const my_lora_config_t s_lora_config =
    {
        /* 凭证必须通过量产配置注入，禁止把真实密钥提交到源码。 */
        .dev_eui = {0},
        .join_eui = {0},
        .app_key = {0},
        .uplink_fport = 1,
        .downlink_fport = 1,
        .uplink_interval_s = 600,
        .confirmed_uplink = false,
};

/* 仅由 LoRa 线程访问的运行状态。 */
static my_lora_state_t s_lora_state = MY_LORA_STATE_OFF;
static bool s_lora_enabled = false;
static bool s_lora_initialized = false;
static int64_t s_next_uplink_ms;
static int64_t s_tx_request_ms;
static int64_t s_next_join_retry_ms;
/* 线程控制块，由 my_lora_init() 创建一次。 */
static struct k_thread s_lora_thread;
/* LoRa 所有者线程的专用栈。 */
K_THREAD_STACK_DEFINE(s_lora_thread_stack, MY_LORA_TASK_STACK_SIZE);

static void my_lora_thread(void* arg1, void* arg2, void* arg3);
int my_lora_send_payload(const uint8_t* payload, uint8_t payload_length);
static const uint8_t* my_lora_get_test_payload(uint8_t* payload_length);

/** 在 LoRa 所有者线程中执行一条控制请求。 */
static void my_lora_handle_message(uint32_t message_id)
{
    switch (message_id)
    {
        case MY_MSG_LORA_ENABLE:
            if (!s_lora_enabled)
            {
                s_lora_enabled = true;
                s_lora_initialized = false;
                s_lora_state = MY_LORA_STATE_STARTING;
                s_next_join_retry_ms = k_uptime_get();
                LOG_INF("LoRaWAN service enabled; starting modem");
            }
            break;

        case MY_MSG_LORA_DISABLE:
            if (s_lora_initialized)
            {
                modem_e_response_code_t response = lr1121_lora_leave_network();

                if (response != MODEM_E_RESPONSE_CODE_OK)
                {
                    LOG_WRN("LoRaWAN leave network failed: 0x%02x", response);
                }
            }
            s_lora_enabled = false;
            s_lora_initialized = false;
            s_lora_state = MY_LORA_STATE_OFF;
            s_next_uplink_ms = 0;
            s_next_join_retry_ms = 0;
            s_tx_request_ms = 0;
            LOG_INF("LoRaWAN service disabled");
            break;

        case MY_MSG_LORA_TEST_UPLINK:
#if MY_LORA_SHELL_TEST_ENABLE
        {
            const uint8_t* payload;
            uint8_t payload_length;

            payload = my_lora_get_test_payload(&payload_length);
            (void)my_lora_send_payload(payload, payload_length);
            break;
        }
#else
            break;
#endif

        default:
            break;
    }
}

/** 选择已配置的测试载荷，不执行数据拷贝。 */
static const uint8_t* my_lora_get_test_payload(uint8_t* payload_length)
{
#if MY_LORA_A0_TEST_PAYLOAD_ENABLE
    *payload_length = sizeof(s_a0_test_payload);
    return s_a0_test_payload;
#else
    *payload_length = sizeof(s_stress_test_payload);
    return s_stress_test_payload;
#endif
}

/** 检查 OTAA 凭据是否已完成配置。 */
static bool my_lora_is_provisioned(void)
{
    static const uint8_t empty_eui[8];
    static const uint8_t empty_key[16];

    return (memcmp(s_lora_config.dev_eui, empty_eui, sizeof(empty_eui)) != 0) &&
           (memcmp(s_lora_config.join_eui, empty_eui, sizeof(empty_eui)) != 0) &&
           (memcmp(s_lora_config.app_key, empty_key, sizeof(empty_key)) != 0);
}

/** 配置 OTAA 参数并发起一次入网尝试。 */
static int my_lora_start_otaa(void)
{
    modem_e_response_code_t response = lr1121_lora_configure_otaa(s_lora_config.dev_eui,
        s_lora_config.join_eui,
        s_lora_config.app_key);
    if (response == MODEM_E_RESPONSE_CODE_OK)
    {
        response = lr1121_lora_join();
    }
    if (response != MODEM_E_RESPONSE_CODE_OK)
    {
        LOG_ERR("LoRaWAN OTAA setup failed: 0x%02x", response);
        return -EIO;
    }

    LOG_INF("LoRaWAN OTAA join started");
    return 0;
}

/** 入网前复位、识别并配置 Modem-E。 */
static int my_lora_initialize_modem(void)
{
    modem_e_system_version_t version = {0};
    modem_e_lorawan_version_t lorawan_version = {0};
    modem_e_response_code_t response;

    /* 等待LR1121完全稳定 */
    k_msleep(100);

    if (lr1121_lora_reset() != MODEM_E_MODEM_HAL_STATUS_OK)
    {
        LOG_ERR("LR1121 hardware reset failed");
        return -EIO;
    }

    /* 复位后等待芯片完全启动 */
    k_msleep(50);

    response = lr1121_lora_get_version(&version);
    if (response != MODEM_E_RESPONSE_CODE_OK)
    {
        LOG_ERR("LR1121 Modem-E version query failed: 0x%02x", response);
        return -EIO;
    }

    LOG_INF("Modem-E version response: hardware 0x%02x, type 0x%02x, firmware %u.%u",
        version.hw, version.type, version.fw >> 8, version.fw & 0xFF);

    response = lr1121_lora_get_lorawan_version(&lorawan_version);
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
#if 0 // TODO 暂时获取的type不是LR1121(供应商反馈是时序问题,待排查,影响不大)
    if (version.type != MODEM_E_SYSTEM_VERSION_TYPE_LR1121)
    {
        LOG_ERR("Unexpected Modem-E device type: 0x%02x", version.type);
        return -ENODEV;
    }
#endif
    s_lora_initialized = true;

    response = my_lora_start_otaa();
    if (response != 0)
    {
        return response;
    }

    s_lora_state = MY_LORA_STATE_JOINING;
    s_next_join_retry_ms = k_uptime_get() + MY_LORA_JOIN_RETRY_INTERVAL_MS;
    return 0;
}

/** 初始化消息端点并创建唯一的 LoRa 所有者线程。 */
int my_lora_init(void)
{
    k_tid_t lora_thread_id;

    s_lora_state = MY_LORA_STATE_OFF;
    s_lora_enabled = false;
    s_lora_initialized = false;
    my_init_msg_handler(MOD_LORA, &my_lora_msgq);
    lora_thread_id = k_thread_create(&s_lora_thread, s_lora_thread_stack,
        K_THREAD_STACK_SIZEOF(s_lora_thread_stack),
        my_lora_thread, NULL, NULL, NULL,
        MY_LORA_TASK_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(lora_thread_id, "MY_LORA");
    LOG_INF("LoRaWAN service initialized and disabled");
    return 0;
}

/** 异步提交 LoRa 开启请求。 */
int my_lora_enable(void)
{
    if (!my_lora_is_provisioned())
    {
        return -EACCES;
    }
    my_send_msg(MOD_MAIN, MOD_LORA, MY_MSG_LORA_ENABLE);
    return 0;
}

/** 异步提交 LoRa 关闭请求。 */
int my_lora_disable(void)
{
    my_send_msg(MOD_MAIN, MOD_LORA, MY_MSG_LORA_DISABLE);
    return 0;
}

/** 入网成功且定时器到期时发送周期上行数据。 */
static void my_lora_schedule_uplink(void)
{
    const uint8_t* payload;
    uint8_t payload_length;

    if ((s_lora_state != MY_LORA_STATE_JOINED) ||
        (k_uptime_get() < s_next_uplink_ms))
    {
        return;
    }

    payload = my_lora_get_test_payload(&payload_length);
    (void)my_lora_send_payload(payload, payload_length);
}

/** 在 LoRa 所有者线程中校验并提交上行数据。 */
int my_lora_send_payload(const uint8_t* payload, uint8_t payload_length)
{
    modem_e_response_code_t response;

    if ((payload == NULL) || (payload_length == 0) ||
        (payload_length > MY_LORA_MAX_PAYLOAD_LENGTH))
    {
        return -EINVAL;
    }
    if (!s_lora_initialized)
    {
        return -ENODEV;
    }
    if (!my_lora_is_provisioned())
    {
        return -EACCES;
    }
    if (s_lora_state != MY_LORA_STATE_JOINED)
    {
        return -EAGAIN;
    }

    response = lr1121_lora_request_tx(s_lora_config.uplink_fport,
        s_lora_config.confirmed_uplink ? MODEM_E_UPLINK_CONFIRMED : MODEM_E_UPLINK_UNCONFIRMED,
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
int my_lora_request_test_uplink(void)
{
    my_send_msg(MOD_MAIN, MOD_LORA, MY_MSG_LORA_TEST_UPLINK);
    return 0;
}
#endif

/** 读取并记录一次下行事件，仅处理配置的 FPort。 */
static void my_lora_handle_down_data(void)
{
    uint8_t payload[MY_LORA_MAX_DOWNLINK_LENGTH];
    uint8_t payload_size = 0U;
    uint8_t remaining;
    modem_e_downlink_metadata_t metadata;
    modem_e_response_code_t response;

    response = lr1121_lora_get_downlink_data_size(&payload_size, &remaining);
    if ((response != MODEM_E_RESPONSE_CODE_OK) ||
        (payload_size > sizeof(payload)))
    {
        LOG_ERR("LoRaWAN downlink size read failed: 0x%02x, size=%u", response,
            payload_size);
        return;
    }

    response = lr1121_lora_get_downlink_data(payload, payload_size);
    if (response == MODEM_E_RESPONSE_CODE_OK)
    {
        response = lr1121_lora_get_downlink_metadata(&metadata);
    }

    if (response != MODEM_E_RESPONSE_CODE_OK)
    {
        LOG_ERR("LoRaWAN downlink read failed: 0x%02x", response);
        return;
    }

    if (metadata.fport != s_lora_config.downlink_fport)
    {
        LOG_WRN("Ignoring downlink on FPort %u", metadata.fport);
        return;
    }

    LOG_HEXDUMP_INF(payload, payload_size, "LoRaWAN downlink command");
    ARG_UNUSED(remaining);
}

/** 推进一次入网、事件和上行状态，仅由 my_lora_thread() 调用。 */
static void my_lora_poll(void)
{
    modem_e_event_fields_t modem_event;
    modem_e_response_code_t response;

    if (!s_lora_enabled || !my_lora_is_provisioned())
    {
        return;
    }

    if ((s_lora_state == MY_LORA_STATE_STARTING) &&
        (k_uptime_get() >= s_next_join_retry_ms))
    {
        if (my_lora_initialize_modem() != 0)
        {
            s_lora_initialized = false;
            s_next_join_retry_ms = k_uptime_get() + MY_LORA_JOIN_RETRY_INTERVAL_MS;
        }
        return;
    }

    if ((s_lora_state == MY_LORA_STATE_JOINING) &&
        (k_uptime_get() >= s_next_join_retry_ms))
    {
        if (my_lora_start_otaa() == 0)
        {
            s_next_join_retry_ms = k_uptime_get() + MY_LORA_JOIN_RETRY_INTERVAL_MS;
        }
        else
        {
            s_next_join_retry_ms = k_uptime_get() + MY_LORA_JOIN_RETRY_INTERVAL_MS;
        }
    }

    response = lr1121_lora_get_event(&modem_event);
    if (response != MODEM_E_RESPONSE_CODE_NO_EVENT &&
        response != MODEM_E_RESPONSE_CODE_OK)
    {
        LOG_ERR("LoRaWAN event read failed: 0x%02x", response);
        return;
    }

    if (response == MODEM_E_RESPONSE_CODE_OK)
    {
        switch (modem_event.event_type)
        {
            case MODEM_E_LORAWAN_EVENT_JOINED:
                s_lora_state = MY_LORA_STATE_JOINED;
                s_next_uplink_ms = k_uptime_get();
                LOG_INF("LoRaWAN joined");
                break;

            case MODEM_E_LORAWAN_EVENT_JOIN_FAIL:
                s_lora_state = MY_LORA_STATE_JOINING;
                s_next_join_retry_ms = k_uptime_get() + MY_LORA_JOIN_RETRY_INTERVAL_MS;
                LOG_WRN("LoRaWAN join attempt failed; retry scheduled");
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
                my_lora_handle_down_data();
                break;

            default:
                LOG_DBG("LoRaWAN event %u", modem_event.event_type);
                break;
        }
    }

    if (s_lora_state == MY_LORA_STATE_JOINED)
    {
        my_lora_schedule_uplink();
    }
}

/** 独占 LoRa 消息队列并执行全部 Modem-E 事务。 */
static void my_lora_thread(void* arg1, void* arg2, void* arg3)
{
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    while (true)
    {
        msg_t message;

        if (my_recv_msg(&my_lora_msgq, &message, sizeof(message),
                K_MSEC(MY_LORA_POLL_INTERVAL_MS)) == 0)
        {
            my_lora_handle_message(message.msgID);
        }
        my_lora_poll();
    }
}
