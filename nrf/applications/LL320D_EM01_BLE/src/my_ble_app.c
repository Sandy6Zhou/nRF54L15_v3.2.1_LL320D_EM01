/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        my_ble_app.c
**文件描述:        设备蓝牙交互模块实现文件
**当前版本:        V1.0
**作    者:        周森达 (zhousenda@jimiiot.com)
**完成日期:        2026-03-05
*********************************************************************
** 功能描述:        1. AES加解密
**                 2. 蓝牙协议数据封包与解包,数据解析
**                 3. DFU文件传输（基于几米自定义 BLE 3.0 DFU 协议解析(Jimi Iot 蓝牙通信协议V3.1.6_2026-3-5 6.4 OTA文件传输)）
*********************************************************************/

/* 必须在包含 my_comm.h 之前定义 BLE_LOG_MODULE_ID，避免与 my_ble_log.h 中的默认定义冲突 */
#define BLE_LOG_MODULE_ID BLE_LOG_MOD_BLE

#include "my_comm.h"

LOG_MODULE_REGISTER(my_ble_app, LOG_LEVEL_INF);

/* 重试的最大次数 */
#define BLE_PACKET_TRANS_MAX_RETRY      3
/* 回复超时时间(ms) */
#define BLE_PACKET_RSP_TIMEOUT_MS       2000
/* 单包回复数据类型（0xFF 0x01） */
#define BLE_PACKET_PROTO_SINGLE         0x01
/* 分包回复数据类型（0xFF 0x02） */
#define BLE_PACKET_PROTO_MULTIPLE       0x02
/* Varint 编码辅助宏：判断值是否需要2字节（>127） */
#define VARINT_IS_2BYTE(val)            ((val) > 127)
/* Varint 编码辅助宏：编码高字节（当值>127时） */
#define VARINT_HIGH_BYTE(val)           (((val) >> 7) & 0x7F)
/* Varint 编码辅助宏：编码低字节（带延续位标记） */
#define VARINT_LOW_BYTE(val)            (((val) & 0x7F) | 0x80)

/* 单/多包传输状态结构 */
typedef struct
{
    uint8_t data[RESP_STRING_LENGTH_MAX];   /* 原始数据 */
    uint16_t total_len;                     /* 原始数据总长度 */
    uint16_t sent_offset;                   /* 已发送数据偏移 */
    uint16_t last_offset;                   /* 最近发送包的数据偏移 */
    uint16_t last_len;                      /* 最近发送包的数据长度 */
    uint8_t next_seq;                       /* 下一个包序号 */
    uint8_t retry_cnt;                      /* 当前重试次数 */
    uint8_t last_seq;                       /* 最近发送包序号 */
    uint8_t last_proto_type;                /* 最近发送协议类型 */
    bool waiting_ack;                       /* 是否正在等待APP回复 */
    bool is_busy;                           /* 传输是否进行中 */
} packet_trans_t;

/* 等待传输的请求（当传输忙时，新请求存入此处） */
typedef struct
{
    uint8_t data[RESP_STRING_LENGTH_MAX];   /* 等待发送的数据 */
    uint16_t len;                           /* 数据长度 */
    bool pending;                           /* 是否有等待的请求 */
} packet_trans_wait_t;

/* 单/多包传输数据集 */
static packet_trans_t s_packet_trans_data = {0};
/* 等待传输请求数据集 */
static packet_trans_wait_t s_packet_trans_wait = {0};
/* BLE单/分包回复超时定时器 */
static struct k_timer s_ble_packet_rsp_timer;

static uint8_t s_rx_ble_buf[BLE_SVC_RX_MAX_LEN];
static uint16_t s_rx_ble_buf_index = 0;

uint16_t g_ble_server_mtu = BLE_SERVER_MIN_MTU;

static uint16_t s_prva_a;
static uint8_t s_aes_base_key[16] = {0x3A, 0x60, 0x43, 0x2A, 0x5C, 0x01, 0x21, 0x1F,
                                 0x29, 0x1E, 0x0F, 0x4E, 0x0C, 0x13, 0x28, 0x25};

/* BLE 数据解密缓冲区（DFU 和普通 BLE 数据共用，不会同时使用，且在单线程中访问，因此线程安全） */
static uint8_t s_ble_decrypt_buffer[BLE_MTU_DATA_LEN];

/* BLE 发送缓冲区（静态全局，避免栈溢出） */
static uint8_t s_ble_tx_buf[BLE_RESP_LENGTH_MAX];
static uint8_t s_ble_encrypt_buf[BLE_RESP_LENGTH_MAX];

/* BLE 响应缓冲区（多个响应函数共用，单线程安全） */
static uint8_t s_ble_rsp_buf[BLE_MTU_DATA_LEN];

/* BLE 日志环形缓冲区 */
#define BLE_LOG_RING_BUF_SIZE   2048
static uint8_t s_ble_log_ring_buf[BLE_LOG_RING_BUF_SIZE];
static uint16_t s_ble_log_write_idx = 0;
static uint16_t s_ble_log_read_idx = 0;
static uint16_t s_ble_log_data_len = 0;

/* 互斥锁保护环形缓冲区，防止 ble_log_send 重入导致数据损坏
 * 原因：k_yield() 后其他任务可能调用 ble_log_send，造成竞态 */
static struct k_mutex s_ble_log_mutex;

/* 蓝牙断开标志，用于通知 ble_log_send 提前退出
 * 注意：在持有锁期间检查此标志，避免强制释放锁导致的未定义行为 */
static volatile bool s_ble_log_disconnect_pending = false;

/********************************************************************
**函数名称:  ble_log_disconnect_cleanup
**入口参数:  无
**出口参数:  无
**函数功能:  蓝牙断开时设置断开标志，通知 ble_log_send 清理资源
**返 回 值:  无
**注意事项:  在蓝牙断开回调中调用，不强制释放互斥锁（Zephyr不允许非持有者释放）
*********************************************************************/
void ble_log_disconnect_cleanup(void)
{
    /* 设置断开标志，通知 ble_log_send 提前退出
     * 注意：不强制释放互斥锁，因为 Zephyr 不允许非持有者释放锁
     * 持有者会在 ble_log_send 中检测到断开标志后自行释放锁 */
    s_ble_log_disconnect_pending = true;

    /* 注意：不清空缓冲区索引和数据，让持有锁的任务自行处理
     * 原因：如果任务A持有锁时清空索引，会导致任务A操作缓冲区时数据不一致
     * 缓冲区会在 ble_log_send 检测到断开标志后，由持有者安全清空
     * 这里的日志输出要用 LOG_DBG 而不是 MY_LOG_DBG 防止递归调用 */

    LOG_DBG("BLE log disconnect pending flag set");
}

/********************************************************************
**函数名称:  ble_log_connect_init
**入口参数:  无
**出口参数:  无
**函数功能:  蓝牙连接建立时清除断开标志，允许日志发送
**返 回 值:  无
**注意事项:  在蓝牙连接回调中调用，清除断开标志使能日志发送
*********************************************************************/
void ble_log_connect_init(void)
{
    s_ble_log_disconnect_pending = false;

    /* 日志输出要用 LOG_DBG 而不是 MY_LOG_DBG 防止递归调用 */
    LOG_DBG("BLE log disconnect pending flag cleared");
}

/********************************************************************
**函数名称:  ble_packet_trans_rsp_timeout_cb
**入口参数:  timer         ---        回复等待定时器对象
**出口参数:  无
**函数功能:  回复等待超时回调，提交工作项到线程上下文处理
**返 回 值:  无
*********************************************************************/
static void ble_packet_trans_rsp_timeout_cb(struct k_timer *timer)
{
    ARG_UNUSED(timer);
    my_send_msg(MOD_BLE, MOD_BLE, MY_MSG_BLE_PACKET_TIMEOUT);
}

/********************************************************************
**函数名称:  ble_packet_trans_stop_rsp_timer
**入口参数:  无
**出口参数:  无
**函数功能:  停止BLE回复等待定时器
**返 回 值:  无
*********************************************************************/
static void ble_packet_trans_stop_rsp_timer(void)
{
    k_timer_stop(&s_ble_packet_rsp_timer);
}

/********************************************************************
**函数名称:  ble_packet_trans_start_rsp_timer
**入口参数:  无
**出口参数:  无
**函数功能:  启动BLE回复等待定时器
**返 回 值:  无
*********************************************************************/
static void ble_packet_trans_start_rsp_timer(void)
{
    k_timer_start(&s_ble_packet_rsp_timer, K_MSEC(BLE_PACKET_RSP_TIMEOUT_MS), K_NO_WAIT);
}

/*********************************************************************
**函数名称:  pkcs7_pad
**入口参数:  buf         --  待填充的数据缓冲区
**           len         --  缓冲区中有效数据长度
**           block_size  --  加密块大小（此处固定为16字节）
**出口参数:  无
**函数功能:  按照PKCS7标准对数据进行填充，使数据长度为block_size的整数倍
**返 回 值:  填充后的数据总长度，参数不合法时返回0
*********************************************************************/
uint32_t pkcs7_pad(uint8_t *buf, uint32_t len, uint32_t block_size)
{
    uint8_t pad_byte;
    uint32_t i;
    uint32_t pad_len = block_size - (len % block_size);

    if (buf == NULL || len == 0 || block_size == 0)
    {
        return 0;
    }

    pad_byte = (uint8_t)pad_len;
    for (i = 0; i < pad_len; i++)
    {
        buf[len + i] = pad_byte;
    }

    return len + pad_len;
}

/*********************************************************************
**函数名称:  pkcs7_unpad
**入口参数:  buf         --  待去填充的数据缓冲区
**           len         --  缓冲区中包含填充的数据总长度
**出口参数:  无
**函数功能:  验证并去除PKCS7填充字节，恢复原始数据长度
**返 回 值:  去填充后的原始数据长度，填充无效时返回原长度
*********************************************************************/
uint32_t pkcs7_unpad(uint8_t *buf, uint32_t len)
{
    uint8_t pad_byte;
    uint32_t i;

    if (buf == NULL || len == 0)
    {
        return 0;
    }

    pad_byte = buf[len - 1];
    if (pad_byte > len)
    {
        return len;  // 无效填充
    }

    // 验证填充字节
    for (i = 0; i < pad_byte; i++)
    {
        if (buf[len - 1 - i] != pad_byte)
        {
            return len;  // 无效填充
        }
    }

    return len - pad_byte;
}

/*********************************************************************
**函数名称:  aes_ecb_encrypt
**入口参数:  key         --  AES加密密钥
**           keybits     --  密钥位数（此处固定为128）
**           plaintext   --  待加密的明文数据
**           pt_len      --  明文数据长度
**           ciphertext  --  存储加密后密文的缓冲区
**           ct_len      --  密文缓冲区长度
**出口参数:  无
**函数功能:  导入AES密钥，对明文做PKCS7填充后，采用ECB模式加密
**返 回 值:  成功返回PSA_SUCCESS(0)，失败返回对应错误码（负数）
*********************************************************************/
int aes_ecb_encrypt(const uint8_t *key, int keybits,
                    const uint8_t *plaintext, uint32_t pt_len,
                    uint8_t *ciphertext, uint32_t ct_len)
{
    psa_status_t status;
    psa_key_id_t key_id = 0;
    psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;
    uint32_t padded_len;
    uint8_t *padded_input = NULL;
    uint32_t i;
    size_t out_len = 0;

    if (key == NULL || plaintext == NULL || ciphertext == NULL)
    {
        return -1;
    }

    /* 1. 导入 AES 密钥 */
    psa_set_key_type(&attr, PSA_KEY_TYPE_AES);
    psa_set_key_bits(&attr, (size_t)keybits);
    psa_set_key_usage_flags(&attr, PSA_KEY_USAGE_ENCRYPT);
    psa_set_key_algorithm(&attr, PSA_ALG_ECB_NO_PADDING);

    status = psa_import_key(&attr, key, (size_t)(keybits / 8), &key_id);
    psa_reset_key_attributes(&attr);
    if (status != PSA_SUCCESS)
    {
        LOG_INF("psa_import_key fail: %d", status);
        goto cleanup;
    }

    /* 2. PKCS7 填充 */
    MY_MALLOC_BUFFER(padded_input, pt_len + 16);
    if (!padded_input)
    {
        status = PSA_ERROR_INSUFFICIENT_MEMORY;
        goto cleanup;
    }

    memcpy(padded_input, plaintext, pt_len);
    padded_len = pkcs7_pad(padded_input, pt_len, 16);

    if (ct_len < padded_len || padded_len == 0)
    {
        LOG_INF("ciphertext buf too small or padded_len is 0");
        status = PSA_ERROR_BUFFER_TOO_SMALL;
        goto cleanup;
    }

    /* 3. 使用 PSA ECB 对每个 16 字节块加密 */
    for (i = 0; i < padded_len; i += 16)
    {
        out_len = 0;

        status = psa_cipher_encrypt(key_id,
                                    PSA_ALG_ECB_NO_PADDING,
                                    padded_input + i,
                                    16,
                                    ciphertext + i,
                                    16,
                                    &out_len);
        if (status != PSA_SUCCESS || out_len != 16)
        {
            LOG_INF("psa_cipher_encrypt fail: %d, out_len=%u",
                          status, (unsigned int)out_len);
            break;
        }
    }

    if (status == PSA_SUCCESS)
    {
        LOG_INF("ECB Encryption success, padded_len:%d", padded_len);

        /* 加密相关日志仅在调试时输出 */
        LOG_HEXDUMP_DBG(key, keybits / 8, "AES key:");
        LOG_HEXDUMP_DBG(plaintext, pt_len, "AES plaintext:");
    }

cleanup:
    if (padded_input)
    {
        MY_FREE_BUFFER(padded_input);
    }

    if (key_id != 0)
    {
        psa_destroy_key(key_id);
    }

    return (int)status;
}

/*********************************************************************
**函数名称:  aes_ecb_decrypt
**入口参数:  key         --  AES解密密钥
**           keybits     --  密钥位数（此处固定为128）
**           ciphertext  --  待解密的密文数据
**           ct_len      --  密文数据长度（必须为16的倍数）
**           plaintext   --  存储解密后明文的缓冲区
**           pt_len      --  明文缓冲区长度
**出口参数:  无
**函数功能:  导入AES密钥，对密文采用ECB模式解密后，去除PKCS7填充
**返 回 值:  成功返回PSA_SUCCESS(0)，失败返回对应错误码
*********************************************************************/
int aes_ecb_decrypt(const uint8_t *key, int keybits,
                    const uint8_t *ciphertext, uint32_t ct_len,
                    uint8_t *plaintext, uint32_t pt_len)
{
    psa_status_t status;
    psa_key_id_t key_id = 0;
    psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;
    uint32_t i;
    uint32_t unpad_len = 0;
    size_t out_len = 0;

    if (key == NULL || ciphertext == NULL || plaintext == NULL)
    {
        return -1;
    }

    /* 1. 导入 AES 密钥，用于解密 */
    psa_set_key_type(&attr, PSA_KEY_TYPE_AES);
    psa_set_key_bits(&attr, (size_t)keybits);
    psa_set_key_usage_flags(&attr, PSA_KEY_USAGE_DECRYPT);
    psa_set_key_algorithm(&attr, PSA_ALG_ECB_NO_PADDING);

    status = psa_import_key(&attr, key, (size_t)(keybits / 8), &key_id);
    psa_reset_key_attributes(&attr);
    if (status != PSA_SUCCESS)
    {
        LOG_INF("psa_import_key fail: %d", status);
        goto cleanup;
    }

    /* 2. 检查缓冲区大小和长度是否为 16 的倍数 */
    if (ct_len == 0 || (ct_len % 16) != 0)
    {
        LOG_INF("ciphertext length not multiple of block size");
        status = PSA_ERROR_INVALID_ARGUMENT;
        goto cleanup;
    }

    if (pt_len < ct_len)
    {
        LOG_INF("plaintext buffer too small");
        status = PSA_ERROR_BUFFER_TOO_SMALL;
        goto cleanup;
    }

    /* 3. 对每个 16 字节块做 ECB 解密（无填充） */
    for (i = 0; i < ct_len; i += 16)
    {
        out_len = 0;

        status = psa_cipher_decrypt(key_id,
                                    PSA_ALG_ECB_NO_PADDING,
                                    ciphertext + i,
                                    16,
                                    plaintext + i,
                                    16,
                                    &out_len);
        if (status != PSA_SUCCESS || out_len != 16)
        {
            LOG_INF("psa_cipher_decrypt fail: %d, out_len=%u",
                    status, (unsigned int)out_len);
            goto cleanup;
        }
    }

    /* 4. 去除 PKCS7 填充 */
    unpad_len = pkcs7_unpad(plaintext, ct_len);
    LOG_INF("Decryption successful, unpad_len:%d", unpad_len);

    /* 解密相关日志仅在调试时输出 */
    LOG_HEXDUMP_DBG(key, keybits / 8, "AES key:");
    LOG_HEXDUMP_DBG(plaintext, unpad_len, "AES plaintext:");

cleanup:
    if (key_id != 0)
    {
        psa_destroy_key(key_id);
    }

    return (int)status;
}

/*********************************************************************
**函数名称:  BLE_DataTransOverBle
**入口参数:  data        --  待发送的蓝牙数据指针
**           len         --  待发送的数据长度
**出口参数:  无
**函数功能:  打印发送数据日志，并调用蓝牙服务端接口发送通知数据
**返 回 值:  无
*********************************************************************/
void BLE_DataTransOverBle(uint8_t *data, uint16_t len)
{
    ble_server_send_notification(data, len);
}

/*********************************************************************
**函数名称:  ble_comu_generate_pkey
**入口参数:  无
**出口参数:  无
**函数功能:  生成随机数，结合G值计算出蓝牙通信的公钥
**返 回 值:  生成的公钥值
*********************************************************************/
static uint32_t ble_comu_generate_pkey(void)
{
    uint32_t prva_key;
    uint32_t val = 0;

    my_generate_random(&val);

    s_prva_a = val & 0xff;
    s_prva_a <<= 8;
    s_prva_a |= val & 0xff;

    prva_key = s_prva_a;
    prva_key *= my_param_get_Gvalue();

    return prva_key;
}

/*********************************************************************
**函数名称:  ble_comu_send_packet
**入口参数:  type        --  发送数据类型（如PKEY/CMD/CID等）
**           data        --  待发送的原始数据指针
**           len         --  待发送的原始数据长度
**出口参数:  无
**函数功能:  封装蓝牙数据包头，对非PKEY类型数据做AES加密，最终发送数据包
**返 回 值:  无
*********************************************************************/
static void ble_comu_send_packet(uint16_t type, uint8_t *data, uint16_t len)
{
    int ret;

    if (data == NULL || (len > BLE_RESP_LENGTH_MAX) || (len % BLE_CMD_DATA_LEN_UNIT))
    {
        LOG_INF("ble packet data or len(%d) error!", len);
        return;
    }

    // 封装包头、包类型
    s_ble_tx_buf[0] = (BLE_DATA_PACKET_HEAD >> 8) & 0xFF;
    s_ble_tx_buf[1] = BLE_DATA_PACKET_HEAD & 0xFF;
    s_ble_tx_buf[2] = (type >> 8) & 0xFF;
    s_ble_tx_buf[3] = type & 0xFF;

    memcpy(&s_ble_tx_buf[4], data, len);
#if 0
    LOG_INF("ble_comu_send_packet:%d", len);
    LOG_HEXDUMP_INF(s_ble_tx_buf, len + 4, "BLE TX (before encrypt):");
#endif
    // 密钥交换和蓝牙日志不需要加密
    if (type == BLE_DATA_TYPE_PKEY || type == BLE_DATA_TYPE_BT_LOG)
    {
        memcpy(s_ble_encrypt_buf, data, len);
    }
    else
    {
        /* AES-128-ECB 加密 */
        ret = aes_ecb_encrypt(s_aes_base_key, 128, data, len,
                             s_ble_encrypt_buf, sizeof(s_ble_encrypt_buf));
        if (ret != 0)
        {
            LOG_INF("aes_ecb_encrypt! ret=%d", ret);
            return;
        }
    }

    memcpy(&s_ble_tx_buf[4], s_ble_encrypt_buf, len);

    /* 日志输出：发送数据打印加密前的原始数据 */
    LOG_HEXDUMP_DBG(s_ble_tx_buf, len + 4, "BLE TX (before encrypt):");

    BLE_DataTransOverBle(s_ble_tx_buf, len + 4);
}

/*********************************************************************
**函数名称:  ble_comu_key_data_handle
**入口参数:  data        --  收到的公钥相关数据指针
**           len         --  收到的公钥数据长度
**出口参数:  无
**函数功能:  解析公钥指令，生成本地公钥，计算并更新AES基础密钥，回复公钥响应
**返 回 值:  无
*********************************************************************/
static void ble_comu_key_data_handle(const uint8_t *data, uint16_t len)
{
    uint64_t temp = 0;
    uint32_t prva_h, rx_key = 0;
    uint32_t rx_pkey_cmd = 0;
    uint8_t  i, out_data[16];
    uint32_t val = 0;

    for (i = 0; i < 4; i++)
    {
        rx_pkey_cmd <<= 8;
        rx_pkey_cmd |= *(data + i);
    }

    if (rx_pkey_cmd != BLE_PKEY_RX_CMD)
    {
        LOG_INF("ble_rx_pkey_cmd error cmd=%08x!", rx_pkey_cmd);
        return ;
    }

    for (i = 4; i < 8; i++)
    {
        rx_key <<= 8;
        rx_key |= *(data + i);
    }

    prva_h = ble_comu_generate_pkey();      // 获取随机数并产生公钥
    temp = rx_key;
    temp *= s_prva_a;                         // 拿到对方的公钥*自己的随机数

    s_aes_base_key[15] = (temp & 0xFF);
    s_aes_base_key[14] = (temp >> 8) & 0xFF;
    s_aes_base_key[13] = (temp >> 16) & 0xFF;
    s_aes_base_key[12] = (temp >> 24) & 0xFF;
    s_aes_base_key[11] = (temp >> 32) & 0xFF;
    s_aes_base_key[10] = (temp >> 40) & 0xFF;

    out_data[0] = BLE_COMU_CMD_START;
    out_data[1] = BLE_PKEY_RSP_DATA1;
    out_data[2] = BLE_COMU_DEV_CODE;
    out_data[3] = BLE_PKEY_RSP_DATA3;

    out_data[4] = (prva_h >> 24) & 0xFF;
    out_data[5] = (prva_h >> 16) & 0xFF;
    out_data[6] = (prva_h >> 8) & 0xFF;
    out_data[7] = prva_h & 0xFF;

    for (i = 0; i < 8; i++)
    {
        my_generate_random(&val);
        out_data[8+i] = val & 0xff;
    }

    ble_comu_send_packet(BLE_DATA_TYPE_PKEY, out_data, 16);

    LOG_HEXDUMP_INF(s_aes_base_key, 16, "ble_aes_base_key:");
}

/*********************************************************************
**函数名称:  ble_comu_response_cmd
**入口参数:  cmd         --  响应指令类型
**           param       --  响应指令参数（成功/失败等）
**出口参数:  无
**函数功能:  封装蓝牙响应指令数据包，填充随机数后发送指令响应
**返 回 值:  无
*********************************************************************/
void ble_comu_response_cmd(uint8_t cmd, uint8_t param)
{
    uint8_t i;
    uint32_t val = 0;

    memset(s_ble_rsp_buf, 0, BLE_CMD_DATA_LEN_UNIT);

    s_ble_rsp_buf[0] = BLE_COMU_CMD_START;
    s_ble_rsp_buf[1] = 0x03;                 // len = device code + cmd + param
    s_ble_rsp_buf[2] = BLE_COMU_DEV_CODE;
    s_ble_rsp_buf[3] = cmd;
    s_ble_rsp_buf[4] = param;

    for(i = 5; i < BLE_CMD_DATA_LEN_UNIT; i++)
    {
        my_generate_random(&val);
        s_ble_rsp_buf[i] = val & 0xff;
    }

    ble_comu_send_packet(BLE_DATA_TYPE_CMD, s_ble_rsp_buf, BLE_CMD_DATA_LEN_UNIT);
}

/*********************************************************************
**函数名称:  ble_comu_cid_data_handle
**入口参数:  data        --  收到的CID(串号)数据指针
**           len         --  收到的CID数据长度
**出口参数:  无
**函数功能:  对比收到的CID与设备本地IMEI，回复对比结果响应
**返 回 值:  无
*********************************************************************/
static void ble_comu_cid_data_handle(const uint8_t *data, uint16_t len)
{
    const gsm_imei_t *gsmImei = my_param_get_imei();

    if (gsmImei->flag == FLAG_VALID)
    {
        if (memcmp(data, gsmImei->hex, GSM_IMEI_LENGTH) == 0)
        {
            ble_comu_response_cmd(BLE_RSP_CMD_CID, BLE_RSP_PARAM_SUCCESS);
            /* CID验证成功，确认是自己的APP，开启蓝牙日志（延迟1秒后允许发送） */
            ble_log_set_ready(true);
        }
        else
        {
            ble_comu_response_cmd(BLE_RSP_CMD_CID, BLE_RSP_PARAM_FAIL);
            LOG_INF("ble_cid_compare_fail!");
        }
    }
    else
    {
        ble_comu_response_cmd(BLE_RSP_CMD_CID, BLE_RSP_PARAM_FAIL);
        LOG_INF("invalid imei param!");
    }
}

/********************************************************************
**函数名称:  ble_comu_response_or_expansion_cmd
**入口参数:  type          ---        命令类型
**         :  str_data      ---        输入数据指针
**         :  len           ---        数据长度
**出口参数:  无
**函数功能:  发送BLE通信响应或扩展命令数据
**返回值:    无
**注意事项:  数据长度会被扩展到16字节的倍数进行发送
*********************************************************************/
void ble_comu_response_or_expansion_cmd(uint16_t type, uint8_t *str_data, uint8_t len)
{
    uint8_t send_len = 0;

    memset(s_ble_rsp_buf, 0, BLE_SVC_RX_MAX_LEN);
    memcpy(s_ble_rsp_buf, str_data, len);
    send_len = len/16;
    send_len *= 16;

    if (len % 16)
        send_len += 16;

    ble_comu_send_packet(type, s_ble_rsp_buf, send_len);
}

/********************************************************************
**函数名称:  ble_packet_trans_record_last_packet
**入口参数:  proto_type    ---        当前协议类型（FF01/FF02）
**         :  seq           ---        当前包序号
**         :  offset        ---        当前包数据偏移
**         :  len           ---        当前包数据长度
**出口参数:  无
**函数功能:  记录最近一次发送包信息，用于超时或错误回复时重发
**返 回 值:  无
*********************************************************************/
static void ble_packet_trans_record_last_packet(uint8_t proto_type, uint8_t seq, uint16_t offset, uint16_t len)
{
    s_packet_trans_data.last_proto_type = proto_type;
    s_packet_trans_data.last_seq = seq;
    s_packet_trans_data.last_offset = offset;
    s_packet_trans_data.last_len = len;
    s_packet_trans_data.waiting_ack = true;
}

/********************************************************************
**函数名称:  ble_packet_trans_reset_state
**入口参数:  b_clear_pending ---        是否同时清空等待队列
**出口参数:  无
**函数功能:  复位当前回复传输状态
**返 回 值:  无
*********************************************************************/
static void ble_packet_trans_reset_state(bool b_clear_pending)
{
    ble_packet_trans_stop_rsp_timer();
    memset(&s_packet_trans_data, 0, sizeof(s_packet_trans_data));
    if (b_clear_pending)
    {
        memset(&s_packet_trans_wait, 0, sizeof(s_packet_trans_wait));
    }
}

/********************************************************************
**函数名称:  ble_packet_trans_try_start_pending
**入口参数:  无
**出口参数:  无
**函数功能:  尝试启动等待中的最新一条回复请求
**返 回 值:  无
*********************************************************************/
static void ble_packet_trans_try_start_pending(void)
{
    if (s_packet_trans_wait.pending)
    {
        s_packet_trans_wait.pending = false;
        ble_packet_trans_send(s_packet_trans_wait.data, s_packet_trans_wait.len);
    }
}

/********************************************************************
**函数名称:  ble_packet_trans_complete_current
**入口参数:  无
**出口参数:  无
**函数功能:  完成当前回复传输并拉起等待请求
**返 回 值:  无
*********************************************************************/
static void ble_packet_trans_complete_current(void)
{
    ble_packet_trans_reset_state(false);
    ble_packet_trans_try_start_pending();
}

/********************************************************************
**函数名称:  ble_packet_trans_abort_current
**入口参数:  无
**出口参数:  无
**函数功能:  中止当前回复传输并拉起等待请求
**返 回 值:  无
*********************************************************************/
static void ble_packet_trans_abort_current(void)
{
    ble_packet_trans_reset_state(false);
    ble_packet_trans_try_start_pending();
}

/********************************************************************
**函数名称:  ble_single_packet_can_send
**入口参数:  len           ---        原始数据长度
**出口参数:  无
**函数功能:  判断当前数据是否可以使用FF01单包协议发送
**返 回 值:  true=可单包发送，false=需要分包
*********************************************************************/
static bool ble_single_packet_can_send(uint16_t len)
{
    uint16_t module_len_bytes;
    uint16_t content_len;
    uint16_t content_len_bytes;
    uint16_t payload_len;
    uint16_t encrypt_len;
    uint16_t max_encrypt_len;

    // data长度>127时Module len占2字节，否则1字节
    module_len_bytes = VARINT_IS_2BYTE(len) ? 2 : 1;

    // content_len = Module id + Module len + data
    content_len = 1 + module_len_bytes + len;

    // len字段本身占用字节数（根据content_len决定，>127时需要2字节）
    content_len_bytes = VARINT_IS_2BYTE(content_len) ? 2 : 1;

    // payload部分长度 = len字段 + content_len（即ble_comu_send_packet传入的data长度）
    payload_len = content_len_bytes + content_len;

    // 按16字节对齐计算加密后长度（只对payload对齐，与ble_comu_response_or_expansion_cmd一致）
    encrypt_len = (payload_len / BLE_CMD_DATA_LEN_UNIT) * BLE_CMD_DATA_LEN_UNIT;
    if (payload_len % BLE_CMD_DATA_LEN_UNIT)
    {
        encrypt_len += BLE_CMD_DATA_LEN_UNIT;
    }

    // 最大允许的加密payload长度（扣除外层4字节head+type、预留16padding填充）
    max_encrypt_len = ((BLE_SERVER_MAX_DATA_LEN - 4) / BLE_CMD_DATA_LEN_UNIT) * BLE_CMD_DATA_LEN_UNIT - 16;

    // 加密后payload长度不超过MTU限制
    return (encrypt_len <= max_encrypt_len);
}

/********************************************************************
**函数名称:  ble_single_packet_send
**入口参数:  data          ---        待发送数据指针
**         :  len           ---        待发送数据长度
**出口参数:  无
**函数功能:  使用FF01协议发送单包回复并进入等待回复状态
**返 回 值:  无
**注意事项:  FF01发送payload格式为 len(Varint, 0-512) + Module id(Varint) + Module len(Varint) + data
*********************************************************************/
static void ble_single_packet_send(uint8_t *data, uint16_t len)
{
    uint16_t encrypt_len;
    uint8_t *p_buf;
    uint8_t module_len_bytes;
    uint16_t content_len;

    if (data == NULL || len == 0)
    {
        return;
    }

    memset(s_ble_rsp_buf, 0, sizeof(s_ble_rsp_buf));
    p_buf = s_ble_rsp_buf;
    module_len_bytes = VARINT_IS_2BYTE(len) ? 2 : 1;
    content_len = 1 + module_len_bytes + len;

    /* len字段动态Varint编码（0-127用1字节，128-512用2字节） */
    if (VARINT_IS_2BYTE(content_len))
    {
        *p_buf++ = VARINT_LOW_BYTE(content_len);
        *p_buf++ = VARINT_HIGH_BYTE(content_len);
    }
    else
    {
        *p_buf++ = (uint8_t)content_len;
    }
    *p_buf++ = BLE_PACKET_TRANS_MODULE_ID;

    if (module_len_bytes == 2)
    {
        *p_buf++ = VARINT_LOW_BYTE(len);
        *p_buf++ = VARINT_HIGH_BYTE(len);
    }
    else
    {
        *p_buf++ = (uint8_t)len;
    }

    memcpy(p_buf, data, len);
    encrypt_len = (uint16_t)(p_buf - s_ble_rsp_buf) + len;
    if (encrypt_len % BLE_CMD_DATA_LEN_UNIT)
    {
        encrypt_len += BLE_CMD_DATA_LEN_UNIT - (encrypt_len % BLE_CMD_DATA_LEN_UNIT);
    }

    // 记录本次发送的包信息，用于超时重发时恢复状态
    ble_packet_trans_record_last_packet(BLE_PACKET_PROTO_SINGLE, 0, 0, len);
    // 发送加密后的单包数据（含包头0xFF01）
    ble_comu_send_packet(BLE_DATA_TYPE_PACKET_SINGLE, s_ble_rsp_buf, encrypt_len);
    // 启动应答超时定时器，等待APP回复确认（超时后自动重发）
    ble_packet_trans_start_rsp_timer();
}

/********************************************************************
**函数名称:  ble_multiple_packet_send
**入口参数:  seq           ---        包序号(0表示首包,0xFF表示最后一包)
**         :  data          ---        本包数据指针
**         :  len           ---        本包数据长度
**         :  is_first      ---        是否为第一包(true表示第一包,包含Module字段)
**出口参数:  无
**函数功能:  封装并发送单个分包，同时进入等待ACK状态
**返 回 值:  无
**注意事项:  FF02发送payload格式为 Len(Varint, 0-512) + Seq(1B) + [首包附带Module id + Module len] + data
*********************************************************************/
static void ble_multiple_packet_send(uint8_t seq, uint8_t *data, uint16_t len, bool is_first)
{
    uint8_t *p_buf;
    uint16_t encrypt_len;
    uint16_t content_len;
    uint16_t offset;

    if (data == NULL || len == 0)
    {
        LOG_WRN("Invalid param: data=%p, len=%d", data, len);
        return;
    }

    content_len = 1 + len;
    if (is_first)
    {
        content_len += 1;
        content_len += VARINT_IS_2BYTE(s_packet_trans_data.total_len) ? 2 : 1;
    }

    memset(s_ble_rsp_buf, 0, sizeof(s_ble_rsp_buf));
    p_buf = s_ble_rsp_buf;

    if (VARINT_IS_2BYTE(content_len))
    {
        *p_buf++ = VARINT_LOW_BYTE(content_len);
        *p_buf++ = VARINT_HIGH_BYTE(content_len);
    }
    else
    {
        *p_buf++ = (uint8_t)content_len;
    }

    *p_buf++ = seq;

    if (is_first)
    {
        *p_buf++ = BLE_PACKET_TRANS_MODULE_ID;
        if (VARINT_IS_2BYTE(s_packet_trans_data.total_len))
        {
            *p_buf++ = VARINT_LOW_BYTE(s_packet_trans_data.total_len);
            *p_buf++ = VARINT_HIGH_BYTE(s_packet_trans_data.total_len);
        }
        else
        {
            *p_buf++ = (uint8_t)s_packet_trans_data.total_len;
        }
    }

    memcpy(p_buf, data, len);
    encrypt_len = (uint16_t)(p_buf - s_ble_rsp_buf) + len;
    if (encrypt_len % BLE_CMD_DATA_LEN_UNIT)
    {
        encrypt_len += BLE_CMD_DATA_LEN_UNIT - (encrypt_len % BLE_CMD_DATA_LEN_UNIT);
    }

    if (encrypt_len > sizeof(s_ble_rsp_buf))
    {
        LOG_ERR("Packet too large: encrypt_len=%d", encrypt_len);
        return;
    }

    // 计算当前数据在原始数据缓冲区中的偏移，用于超时重发定位
    offset = (uint16_t)(data - s_packet_trans_data.data);
    // 记录本次分包发送信息（协议类型、序号、偏移、长度），用于超时重发
    ble_packet_trans_record_last_packet(BLE_PACKET_PROTO_MULTIPLE, seq, offset, len);
    // 发送加密后的分包数据（含包头0xFF02）
    ble_comu_send_packet(BLE_DATA_TYPE_PACKET_MULTIPLE, s_ble_rsp_buf, encrypt_len);
    // 启动应答超时定时器，等待APP回复ACK（超时后按记录的包信息重发）
    ble_packet_trans_start_rsp_timer();
}

/********************************************************************
**函数名称:  ble_multiple_packet_continue
**入口参数:  ack_seq       ---        APP确认的包序号
**出口参数:  无
**函数功能:  ACK匹配后继续发送下一包
**返 回 值:  无
*********************************************************************/
static void ble_multiple_packet_continue(uint8_t ack_seq)
{
    uint16_t remaining;
    uint16_t pkt_len;
    uint16_t max_encrypt_len;
    uint16_t header_len;
    uint8_t seq_to_send;

    if (!s_packet_trans_data.is_busy || !s_packet_trans_data.waiting_ack)
    {
        return;
    }

    if (ack_seq != s_packet_trans_data.last_seq)
    {
        LOG_WRN("Ack mismatch: expect=%d, got=%d", s_packet_trans_data.last_seq, ack_seq);
        return;
    }

    ble_packet_trans_stop_rsp_timer();
    s_packet_trans_data.waiting_ack = false;
    s_packet_trans_data.retry_cnt = 0;

    remaining = s_packet_trans_data.total_len - s_packet_trans_data.sent_offset;
    if (remaining == 0)
    {
        LOG_INF("Packet trans complete");
        ble_packet_trans_complete_current();
        return;
    }

    max_encrypt_len = ((BLE_SERVER_MAX_DATA_LEN - 4) / BLE_CMD_DATA_LEN_UNIT) * BLE_CMD_DATA_LEN_UNIT;

    /* FF02后续包header = Len(Varint) + Seq(1B)
     * content_len = Seq(1B) + data_len
     * 根据content_len决定Len字段字节数 */
    header_len = VARINT_IS_2BYTE(1 + remaining) ? 2 : 1;
    header_len += 1;  // + Seq(1B)

    /* 后续包最大数据长度，预留16字节padding（PKCS7最坏情况） */
    pkt_len = max_encrypt_len - header_len - 16;

    if (pkt_len > remaining)
    {
        pkt_len = remaining;
    }

    if (s_packet_trans_data.sent_offset + pkt_len >= s_packet_trans_data.total_len)
    {
        seq_to_send = BLE_PACKET_TRANS_SEQ_LAST;
        s_packet_trans_data.next_seq = 0;
    }
    else
    {
        seq_to_send = s_packet_trans_data.next_seq;
        s_packet_trans_data.next_seq++;
    }

    ble_multiple_packet_send(seq_to_send,
                             s_packet_trans_data.data + s_packet_trans_data.sent_offset,
                             pkt_len,
                             false);
    s_packet_trans_data.sent_offset += pkt_len;
}

/********************************************************************
**函数名称:  ble_single_packet_rsp_handle
**入口参数:  data          ---        APP回复数据指针（解密后payload）
**         :  len           ---        数据长度
**出口参数:  无
**函数功能:  处理FF01单包回复确认，payload 固定为 0x00
**返 回 值:  无
*********************************************************************/
static void ble_single_packet_rsp_handle(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len < 1)
    {
        return;
    }

    if (!s_packet_trans_data.is_busy || !s_packet_trans_data.waiting_ack ||
        s_packet_trans_data.last_proto_type != BLE_PACKET_PROTO_SINGLE)
    {
        return;
    }

    ble_packet_trans_stop_rsp_timer();
    s_packet_trans_data.waiting_ack = false;
    s_packet_trans_data.retry_cnt = 0;
    ble_packet_trans_complete_current();
}

/********************************************************************
**函数名称:  ble_multiple_packet_ack_handle
**入口参数:  data          ---        APP回复数据指针（解密后payload）
**         :  len           ---        数据长度
**出口参数:  无
**函数功能:  处理FF02分包确认回复，payload 格式为 len(Varint, 0-512) + 包序号
**返 回 值:  无
*********************************************************************/
static void ble_multiple_packet_ack_handle(const uint8_t *data, uint16_t len)
{
    uint8_t ack_seq;
    uint16_t ack_len;
    uint8_t seq_pos;

    if (data == NULL || len < 2)
    {
        return;
    }

    if (!s_packet_trans_data.is_busy || !s_packet_trans_data.waiting_ack ||
        s_packet_trans_data.last_proto_type != BLE_PACKET_PROTO_MULTIPLE)
    {
        return;
    }

    if (data[0] & 0x80)
    {
        if (len < 3)
        {
            LOG_WRN("FF02 ack len invalid");
            return;
        }

        ack_len = (uint16_t)(data[0] & 0x7F);
        ack_len |= ((uint16_t)data[1] << 7);
        seq_pos = 2;
    }
    else
    {
        ack_len = data[0];
        seq_pos = 1;
    }

    if (len < (uint16_t)(seq_pos + ack_len))
    {
        LOG_WRN("FF02 ack payload invalid");
        return;
    }

    ack_seq = data[seq_pos];

    LOG_INF("Ack received: seq=%d", ack_seq);
    ble_multiple_packet_continue(ack_seq);
}

/********************************************************************
**函数名称:  ble_packet_trans_timeout_handler
**入口参数:  无
**出口参数:  无
**函数功能:  BLE 分包传输应答超时处理，在 BLE 线程上下文执行
**返 回 值:  无
*********************************************************************/
void ble_packet_trans_timeout_handler(void)
{
    if (!s_packet_trans_data.is_busy || !s_packet_trans_data.waiting_ack)
    {
        return;
    }

    if (!ble_is_data_channel_ready())
    {
        ble_packet_trans_abort_current();
        return;
    }

    if (s_packet_trans_data.retry_cnt >= BLE_PACKET_TRANS_MAX_RETRY)
    {
        LOG_ERR("BLE packet rsp timeout, abort current trans");
        ble_packet_trans_abort_current();
        return;
    }

    s_packet_trans_data.retry_cnt++;

    if (s_packet_trans_data.last_proto_type == BLE_PACKET_PROTO_SINGLE)
    {
        ble_single_packet_send(s_packet_trans_data.data, s_packet_trans_data.total_len);
    }
    else if (s_packet_trans_data.last_proto_type == BLE_PACKET_PROTO_MULTIPLE)
    {
        ble_multiple_packet_send(s_packet_trans_data.last_seq,
                                 s_packet_trans_data.data + s_packet_trans_data.last_offset,
                                 s_packet_trans_data.last_len,
                                 (s_packet_trans_data.last_offset == 0));
    }
}

/********************************************************************
**函数名称:  ble_packet_trans_send
**入口参数:  data          ---        待发送的实际数据指针
**         :  len           ---        待发送的实际数据长度
**出口参数:  无
**函数功能:  根据数据长度自动选择FF01或FF02协议发送回复
**返 回 值:  无
*********************************************************************/
void ble_packet_trans_send(uint8_t *data, uint16_t len)
{
    uint16_t max_encrypt_len;
    uint16_t first_pkt_max;
    uint16_t pkt_len;
    uint16_t header_len;
    uint16_t module_len_bytes;
    uint16_t content_len;
    uint16_t len_field_bytes;
    uint8_t first_seq;

    if (data == NULL || len == 0)
    {
        LOG_WRN("Invalid param: data=%p, len=%d", (void *)data, len);
        return;
    }

    if (len > RESP_STRING_LENGTH_MAX)
    {
        LOG_WRN("Wait data too large: %d, truncate to %d", len, RESP_STRING_LENGTH_MAX);
        len = RESP_STRING_LENGTH_MAX;
    }

    if (s_packet_trans_data.is_busy)
    {
        memcpy(s_packet_trans_wait.data, data, len);
        s_packet_trans_wait.len = len;
        s_packet_trans_wait.pending = true;
        LOG_INF("Packet trans busy, queued new request: len=%d", len);
        return;
    }

    memset(&s_packet_trans_data, 0, sizeof(s_packet_trans_data));
    memcpy(s_packet_trans_data.data, data, len);
    s_packet_trans_data.total_len = len;
    s_packet_trans_data.is_busy = true;

    if (ble_single_packet_can_send(len))
    {
        ble_single_packet_send(s_packet_trans_data.data, len);
        return;
    }

    max_encrypt_len = ((BLE_SERVER_MAX_DATA_LEN - 4) / BLE_CMD_DATA_LEN_UNIT) * BLE_CMD_DATA_LEN_UNIT;

    /* FF02首包header = Len(Varint) + Seq(1B) + Module_id(1B) + Module_len(Varint)
     * content_len = Seq + Module_id + Module_len + data_len */
    module_len_bytes = VARINT_IS_2BYTE(len) ? 2 : 1;
    content_len = 1 + 1 + module_len_bytes + len;  // content_len 包括: Seq + Module_id + Module_len + data
    len_field_bytes = VARINT_IS_2BYTE(content_len) ? 2 : 1;
    header_len = len_field_bytes + 1 + 1 + module_len_bytes;

    /* 首包最大数据长度，预留16字节padding（PKCS7最坏情况） */
    first_pkt_max = max_encrypt_len - header_len - 16;

    pkt_len = (len > first_pkt_max) ? first_pkt_max : len;

    first_seq = 0;
    s_packet_trans_data.next_seq = 1;

    ble_multiple_packet_send(first_seq, s_packet_trans_data.data, pkt_len, true);
    s_packet_trans_data.sent_offset = pkt_len;
}

/********************************************************************
**函数名称:  ble_packet_trans_init
**入口参数:  无
**出口参数:  无
**函数功能:  初始化BLE单包/分包回复超时与重发模块
**返 回 值:  无
*********************************************************************/
void ble_packet_trans_init(void)
{
    k_timer_init(&s_ble_packet_rsp_timer, ble_packet_trans_rsp_timeout_cb, NULL);
    ble_packet_trans_reset_state(true);
}

/********************************************************************
**函数名称:  ble_packet_trans_disconnect_cleanup
**入口参数:  无
**出口参数:  无
**函数功能:  蓝牙断开时清理BLE单包/分包回复状态
**返 回 值:  无
*********************************************************************/
void ble_packet_trans_disconnect_cleanup(void)
{
    ble_packet_trans_reset_state(true);
}

/********************************************************************
**函数名称:  ble_log_send_packet
**入口参数:  type     ---        命令类型（固定为 BLE_DATA_TYPE_BT_LOG）
**           data     ---        日志数据指针
**           len      ---        日志数据长度
**出口参数:  无
**函数功能:  发送蓝牙日志数据包，无需16字节对齐，明文传输
**返 回 值:  无
*********************************************************************/
static void ble_log_send_packet(uint16_t type, uint8_t *data, uint16_t len)
{
    uint16_t send_len;

    if (data == NULL || (len > BLE_RESP_LENGTH_MAX))
    {
        LOG_INF("ble log packet data or len(%d) error!", len);
        return;
    }

    /* 确保最小发送长度为16字节（APP协议要求）
     * 如果数据不足16字节，填充0 */
    send_len = (len < 16) ? 16 : len;

    // 封装包头、包类型
    s_ble_tx_buf[0] = (BLE_DATA_PACKET_HEAD >> 8) & 0xFF;
    s_ble_tx_buf[1] = BLE_DATA_PACKET_HEAD & 0xFF;
    s_ble_tx_buf[2] = (type >> 8) & 0xFF;
    s_ble_tx_buf[3] = type & 0xFF;

    /* 复制数据，不足16字节的部分填充0 */
    memcpy(&s_ble_tx_buf[4], data, len);
    if (send_len > len)
    {
        memset(&s_ble_tx_buf[4 + len], 0, send_len - len);
    }

    /* 日志输出 */
    LOG_HEXDUMP_DBG(s_ble_tx_buf, send_len + 4, "BLE LOG TX:");

    BLE_DataTransOverBle(s_ble_tx_buf, send_len + 4);
}

/********************************************************************
**函数名称:  ble_log_ring_buf_write
**入口参数:  data     ---        日志数据指针
**           len      ---        日志数据长度
**出口参数:  无
**函数功能:  写入日志数据到环形缓冲区
**返 回 值:  实际写入的字节数
*********************************************************************/
static uint16_t ble_log_ring_buf_write(uint8_t *data, uint16_t len)
{
    uint16_t i;
    uint16_t write_len = len;

    /* 如果缓冲区空间不足，覆盖旧数据 */
    if (len > BLE_LOG_RING_BUF_SIZE - s_ble_log_data_len)
    {
        write_len = BLE_LOG_RING_BUF_SIZE - s_ble_log_data_len;
        LOG_WRN("BLE log ring buffer full, drop %d bytes", len - write_len);
    }

    for (i = 0; i < write_len; i++)
    {
        s_ble_log_ring_buf[s_ble_log_write_idx] = data[i];
        s_ble_log_write_idx = (s_ble_log_write_idx + 1) % BLE_LOG_RING_BUF_SIZE;
    }
    s_ble_log_data_len += write_len;

    return write_len;
}

/********************************************************************
**函数名称:  ble_log_ring_buf_read
**入口参数:  buf      ---        读取缓冲区指针
**           len      ---        期望读取的最大长度
**出口参数:  无
**函数功能:  从环形缓冲区读取日志数据
**返 回 值:  实际读取的字节数
*********************************************************************/
static uint16_t ble_log_ring_buf_read(uint8_t *buf, uint16_t len)
{
    uint16_t i;
    uint16_t read_len = (len < s_ble_log_data_len) ? len : s_ble_log_data_len;

    for (i = 0; i < read_len; i++)
    {
        buf[i] = s_ble_log_ring_buf[s_ble_log_read_idx];
        s_ble_log_read_idx = (s_ble_log_read_idx + 1) % BLE_LOG_RING_BUF_SIZE;
    }
    s_ble_log_data_len -= read_len;

    return read_len;
}

/********************************************************************
**函数名称:  ble_log_send
**入口参数:  data     ---        日志数据指针
**           len      ---        日志数据长度
**出口参数:  无
**函数功能:  发送蓝牙日志到APP，通过指令通道（0x5901）明文传输
**           超长日志自动缓存，分批发送
**返 回 值:  无
**注意事项:  1. 使用 BLE_DATA_TYPE_BT_LOG（0x5901）数据类型
**           2. 数据自动对齐到16字节（AES加密需要）
**           3. 单包日志内容最大240字节，超过会导致蓝牙自动断开
*********************************************************************/
void ble_log_send(uint8_t *data, uint8_t len)
{
    uint16_t max_data_len;
    uint8_t chunk[BLE_LOG_MAX_CHUNK_SIZE];  /* 单包最大240字节，避免栈溢出 */
    uint8_t chunk_len;
    uint16_t read_len;
    uint8_t send_cnt = 0;

    /* MTU 为默认值时不处理日志，避免影响正常通信，这里很重要，不然可能出现一直占用CPU资源 */
    if (g_ble_server_mtu <= BLE_SERVER_MIN_MTU)
    {
        return;
    }

    /* 蓝牙数据通道未就绪时不处理日志，避免无效操作
     * 注意：必须等APP使能CCC通知后才能发送，否则可能导致栈溢出
     */
    if (!ble_is_data_channel_ready())
    {
        return;
    }

    /* 计算最大数据长度
     * 限制为 BLE_LOG_MAX_CHUNK_SIZE(240)，超过会导致蓝牙自动断开
     */
    max_data_len = (g_ble_server_mtu > BLE_LOG_MTU_OVERHEAD) ? (g_ble_server_mtu - BLE_LOG_MTU_OVERHEAD) : 0;
    if (max_data_len > BLE_LOG_MAX_CHUNK_SIZE)
    {
        max_data_len = BLE_LOG_MAX_CHUNK_SIZE;
    }

    /* 获取互斥锁，保护环形缓冲区操作
     * 原因：k_yield() 可能触发任务切换，其他任务调用 ble_log_send 会造成竞态 */
    k_mutex_lock(&s_ble_log_mutex, K_FOREVER);

    /* 检查断开标志，如果蓝牙已断开，清空缓冲区后释放锁并返回
     * 注意：在持有锁期间检查，确保不会发送数据到已断开的连接 */
    if (s_ble_log_disconnect_pending)
    {
        /* 安全清空缓冲区（持有锁期间操作，避免竞态） */
        s_ble_log_write_idx = 0;
        s_ble_log_read_idx = 0;
        s_ble_log_data_len = 0;
        k_mutex_unlock(&s_ble_log_mutex);
        return;
    }

    /* 写入环形缓冲区 */
    ble_log_ring_buf_write(data, len);

    /* 立即尝试发送缓存中的数据
     * 注意事项：
     * 1. 每发送一包后执行 k_yield（主动让出 CPU），避免长时间占用 CPU，这里很重要，如果日志多，可能导致CPU一直被占
     * 2. 最大连续发送 10 包后退出，防止阻塞其他任务
     * 3. 剩余数据下次调用时继续发送
     * 4. 蓝牙断开后立即停止发送，避免无效操作
     * 5. 持有互斥锁期间调用 k_yield()，其他任务阻塞在锁上，不会重入
     * 6. 每次循环检查断开标志，确保能及时响应蓝牙断开事件
     */
    while (s_ble_log_data_len > 0 && send_cnt < 10 && ble_is_connected() &&
           !s_ble_log_disconnect_pending)
    {
        read_len = (max_data_len < sizeof(chunk)) ? max_data_len : sizeof(chunk);
        chunk_len = ble_log_ring_buf_read(chunk, read_len);
        if (chunk_len == 0)
        {
            break;
        }
        ble_log_send_packet(BLE_DATA_TYPE_BT_LOG, chunk, chunk_len);
        send_cnt++;
        k_yield();  /* 主动让出 CPU，避免阻塞 -- 这里很重要，如果日志多，可能导致CPU一直被占 */
    }

    /* 如果是因为断开标志退出循环，清空缓冲区（持有锁期间操作，避免竞态） */
    if (s_ble_log_disconnect_pending)
    {
        s_ble_log_write_idx = 0;
        s_ble_log_read_idx = 0;
        s_ble_log_data_len = 0;
    }

    /* 释放互斥锁 */
    k_mutex_unlock(&s_ble_log_mutex);
}

/********************************************************************
**函数名称:  ble_comu_at_cmd_handle
**入口参数:  data          ---        AT命令数据输入指针
**         :  len           ---        数据长度
**出口参数:  无
**函数功能:  处理BLE接收的AT命令
**返回值:    无
**注意事项:  根据AT命令处理结果决定是否发送响应数据
*********************************************************************/
void ble_comu_at_cmd_handle(const uint8_t *data, uint16_t len)
{
    at_cmd_t ble_at_msg = {0};

#if 0
    LOG_INF("ble_comu_at_cmd_handle:%s, len=%d", data, len);
    LOG_HEXDUMP_INF(data, len, "hex data:");
#endif

    /* 使用静态缓冲区存储响应数据，避免栈溢出 */
    ble_at_msg.resp_msg = g_resp_buf;

    ble_at_msg.rcv_length = len;
    memcpy(ble_at_msg.rcv_msg, data, len);

    /* 调用命令处理函数，返回类型不再使用 */
    (void)at_recv_cmd_handler(&ble_at_msg);

    /* 响应数据根据大小自动选择单包(FF 01)还是分包(FF 02) */
    if (ble_at_msg.resp_length > 0)
    {
        ble_packet_trans_send((uint8_t *)ble_at_msg.resp_msg, ble_at_msg.resp_length);
    }
}

/*********************************************************************
**函数名称:  ble_comu_app_handle
**入口参数:  type        --  收到的蓝牙数据类型
**           data        --  收到的蓝牙数据指针（去除包头后）
**           len         --  收到的蓝牙数据长度（去除包头后）
**出口参数:  无
**函数功能:  根据数据类型分发处理：PKEY类型直接处理，其他类型解密后处理
**返 回 值:  无
*********************************************************************/
static void ble_comu_app_handle(uint32_t type, const uint8_t *data, uint16_t len)
{
    int ret;

    if(type == BLE_DATA_TYPE_PKEY)
    {
        // 公钥数据
        ble_comu_key_data_handle(data, len);
    }
    else
    {
        if ((len % 16) != 0 || len > (BLE_SVC_RX_MAX_LEN - 4))
        {
            LOG_INF("ble data length error(%d)!",len);
        }
        else
        {
            /* 清空解密缓冲区 */
            memset(s_ble_decrypt_buffer, 0, sizeof(s_ble_decrypt_buffer));

            /* AES-128-ECB 解密 */
            ret = aes_ecb_decrypt(s_aes_base_key, 128,
                                data, len,
                                s_ble_decrypt_buffer, sizeof(s_ble_decrypt_buffer));
            if (ret != 0)
            {
                LOG_INF("aes_ecb_decrypt error!");
                return;
            }

            LOG_INF("rx_ble:%x", type);

            /* 日志输出：接收数据打印解密后的内容 */
            LOG_HEXDUMP_DBG(s_ble_decrypt_buffer, len, "BLE RX (after decrypt):");

            switch(type)
            {
                case BLE_DATA_TYPE_CID:         //串号数据
                    ble_comu_cid_data_handle(s_ble_decrypt_buffer, len);
                    break;

                case BLE_DATA_TYPE_AT_CMD:      //用户指令
                    ble_comu_at_cmd_handle(s_ble_decrypt_buffer, len);
                    break;

                case BLE_DATA_TYPE_PACKET_SINGLE:  //单包回复确认
                    ble_single_packet_rsp_handle(s_ble_decrypt_buffer, len);
                    break;

                case BLE_DATA_TYPE_PACKET_MULTIPLE:  //分包传输确认
                    ble_multiple_packet_ack_handle(s_ble_decrypt_buffer, len);
                    break;

                default:
                    LOG_INF("ble packet type error!");
                    break;
            }
        }
    }
}
/********************************************************************
**函数名称:  ble_log_init
**入口参数:  无
**出口参数:  无
**函数功能:  初始化蓝牙日志模块，包括互斥锁初始化
**返 回 值:  0 表示成功
*********************************************************************/
int ble_log_init(void)
{
    k_mutex_init(&s_ble_log_mutex);
    LOG_INF("BLE log module initialized");
    return 0;
}

/*********************************************************************
**函数名称:  ble_app_comm_data_proc
**入口参数:  data        --  收到的完整蓝牙数据包指针
**           len         --  收到的完整蓝牙数据包长度
**出口参数:  无
**函数功能:  解析蓝牙数据包头和数据类型，验证包头合法性后分发处理
**返 回 值:  无
*********************************************************************/
static void ble_app_comm_data_proc(const uint8_t *data, uint16_t len)
{
    uint16_t pkt_head, pkt_type;

    pkt_head = *(data + 0);
    pkt_head <<= 8;
    pkt_head |= *(data + 1);

    pkt_type = *(data + 2);
    pkt_type <<= 8;
    pkt_type |= *(data + 3);

    if (pkt_head != BLE_DATA_PACKET_HEAD)
    {
        LOG_INF("ble packet head(0x%04x) error!", pkt_head);
    }
    else
    {
        LOG_INF("ble packet type:0x%04x,len=%d", pkt_type, len);
        ble_comu_app_handle(pkt_type, data + 4, len - 4);
    }
}

/********************************************************************
**函数名称:  my_ble_dfu_send_response
**入口参数:  cmd   ---   命令码
**           data  ---   响应数据
**           len   ---   数据长度
**出口参数:  无
**函数功能:  发送 DFU 响应数据
**返 回 值:  无
********************************************************************
**蓝牙协议参考文档：Jimi Iot 蓝牙通信协议V3.1.6_2026-3-5 6.4 OTA文件传输
*********************************************************************/
void my_ble_dfu_send_response(uint8_t cmd, uint8_t *data, uint16_t len)
{
    uint16_t send_len;

    if (len > (BLE_MTU_DATA_LEN - 6))
    {
        LOG_ERR("DFU response data too long!");
        return;
    }

    memset(s_ble_rsp_buf, 0, BLE_MTU_DATA_LEN);

    send_len = 0;
    s_ble_rsp_buf[send_len++] = 0x78;
    s_ble_rsp_buf[send_len++] = 0x79;
    s_ble_rsp_buf[send_len++] = BLE_COMU_DEV_CODE;
    s_ble_rsp_buf[send_len++] = cmd;
    s_ble_rsp_buf[send_len++] = len & 0xFF;
    s_ble_rsp_buf[send_len++] = (len >> 8) & 0xFF;
    memcpy(&s_ble_rsp_buf[send_len], data, len);
    send_len += len;

    send_len = (send_len - send_len % BLE_CMD_DATA_LEN_UNIT);   // 对齐处理，确保数据长度为16的倍数
    ble_comu_send_packet(BLE_DATA_TYPE_FILE_TRANS, s_ble_rsp_buf, send_len);
}

/********************************************************************
**函数名称:  my_ble_dfu_file_trans_handle
**入口参数:  data  ---   数据缓冲区（解密后）
**           len   ---   数据长度
**出口参数:  无
**函数功能:  处理 BLE DFU 文件传输数据
**返 回 值:  处理的数据长度
*********************************************************************/
static uint16_t my_ble_dfu_file_trans_handle(const uint8_t *data, uint16_t len)
{
    uint8_t dev_code, cmd;
    uint16_t pkt_const;
    uint16_t content_len;
    uint16_t data_len = len;
    int ret;
    uint8_t i;

    /* 数据长度对齐处理（必须是 16 字节倍数） */
    if (len % BLE_CMD_DATA_LEN_UNIT)
    {
        data_len = len - (len % BLE_CMD_DATA_LEN_UNIT);
    }

    /* 检查是否超过缓冲区限制 */
    if (data_len > BLE_MTU_DATA_LEN)
    {
        LOG_ERR("DFU data too large: %d > %d", data_len, BLE_MTU_DATA_LEN);
        return len;  /* 返回原长度，表示已处理（丢弃） */
    }

    /* AES-ECB 解密数据（16字节对齐） */
    ret = aes_ecb_decrypt(s_aes_base_key, 128, data, data_len, s_ble_decrypt_buffer, sizeof(s_ble_decrypt_buffer));
    if (ret < 0)
    {
        LOG_ERR("DFU data decrypt failed: %d", ret);
        return len;
    }

    /* 解析包头 */
    pkt_const = ((uint16_t)s_ble_decrypt_buffer[0] << 8) | s_ble_decrypt_buffer[1]; // little-endian
    dev_code = s_ble_decrypt_buffer[2];
    cmd = s_ble_decrypt_buffer[3];
    content_len = ((uint16_t)s_ble_decrypt_buffer[5] << 8) | s_ble_decrypt_buffer[4]; // little-endian

    /* 日志输出：接收数据打印解密后的内容（仅打印有效数据，不含填充） */
    LOG_HEXDUMP_DBG(s_ble_decrypt_buffer, (content_len + 6 > data_len) ? data_len : (content_len + 6), "BLE RX (after decrypt):");

    LOG_INF("DFU RX: pkt=0x%04x, dev=0x%02x, cmd=0x%02x, len=%d",
            pkt_const, dev_code, cmd, content_len);

    /* 验证包头 */
    if (pkt_const != 0x7879 || dev_code != BLE_COMU_APP_CODE)
    {
        LOG_ERR("DFU packet error: pkt=0x%04x, dev=0x%02x", pkt_const, dev_code);
        return (data_len + BLE_FRAME_HEAD_LEN + BLE_CMD_HEAD_LEN);
    }

    /* 检查内容长度 (6字节包头 + content_len) */
    if ((content_len + 6) > data_len)
    {
        LOG_WRN("DFU content length error: %d + 6 > %d", content_len, data_len);
        return len;
    }

    /* 分发到 DFU 处理 */
    LOG_INF("DFU cmd: 0x%02x, content=%d", cmd, content_len);
    jimi_dfu_cmd_handler(cmd, (uint8_t *)&s_ble_decrypt_buffer[6], content_len);

    return (data_len + BLE_FRAME_HEAD_LEN + BLE_CMD_HEAD_LEN);
}

/*********************************************************************
**函数名称:  BLE_DataInputBuffer
**入口参数:  data        --  蓝牙接收的原始数据指针
**           len         --  蓝牙接收的原始数据长度
**出口参数:  无
**函数功能:  验证数据长度合法性后缓存数据，发送消息给蓝牙线程处理
**返 回 值:  无
*********************************************************************/
void BLE_DataInputBuffer(const uint8_t *data, uint16_t len)
{
    if (s_rx_ble_buf_index == 0 && len <= BLE_SVC_RX_MAX_LEN && len >= BLE_GATT_FRAME_LEN_MIN)     // 长度检查
    {
        memcpy(&s_rx_ble_buf[0], data, len);
        s_rx_ble_buf_index = len;

        // LOG_HEXDUMP_INF(s_rx_ble_buf, BLE_GATT_FRAME_LEN_MIN, "BLE_DataInput:");
        my_send_msg(MOD_BLE, MOD_BLE, MY_MSG_BLE_RX);
    }
    else
    {
        LOG_WRN("BLE_DataInputBuffer skip: index=%d, len=%d, max=%d, min=%d",
                s_rx_ble_buf_index, len, BLE_SVC_RX_MAX_LEN, BLE_GATT_FRAME_LEN_MIN);
    }
}

/*********************************************************************
**函数名称:  ble_rx_proc_handle
**入口参数:  无
**出口参数:  无
**函数功能:  蓝牙接收消息处理函数，验证包头后分发处理数据包
**返 回 值:  无
*********************************************************************/
void ble_rx_proc_handle(void)
{
    uint16_t read_packet_head = (s_rx_ble_buf[0] << 8) | s_rx_ble_buf[1];
    uint16_t read_cmd_head = (s_rx_ble_buf[2] << 8) | s_rx_ble_buf[3];

    if(read_packet_head == BLE_DATA_PACKET_HEAD)
    {
        if(read_cmd_head == BLE_DATA_TYPE_FILE_TRANS)
        {
            my_ble_dfu_file_trans_handle(s_rx_ble_buf + 4, s_rx_ble_buf_index - 4); // DFU文件传输处理
        }
        else
        {
            ble_app_comm_data_proc(s_rx_ble_buf, s_rx_ble_buf_index);   // 指令处理
        }
    }
    else
    {
        LOG_INF("ble packet head(0x%04x) error!", read_packet_head);
    }

    s_rx_ble_buf_index = 0;
}
