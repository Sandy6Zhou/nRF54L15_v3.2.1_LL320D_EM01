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
*********************************************************************/

#include "my_comm.h"

LOG_MODULE_REGISTER(my_ble_app, LOG_LEVEL_INF);

static uint8_t rx_ble_buf[BLE_SVC_RX_MAX_LEN];
static uint16_t rx_ble_buf_index = 0;

uint16_t ble_server_mtu = BLE_SERVER_MTU_DFT;

static uint16_t prva_a;
static uint8_t aes_base_key[16] = {0x3A, 0x60, 0x43, 0x2A, 0x5C, 0x01, 0x21, 0x1F,
                                 0x29, 0x1E, 0x0F, 0x4E, 0x0C, 0x13, 0x28, 0x25};

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
    padded_input = malloc(pt_len + 16);
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

        LOG_HEXDUMP_INF(key, keybits / 8, "key:");
        LOG_HEXDUMP_INF(plaintext, pt_len, "plaintext:");
        LOG_HEXDUMP_INF(ciphertext, padded_len, "ciphertext:");
    }

cleanup:
    if (padded_input)
    {
        free(padded_input);
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

    LOG_HEXDUMP_INF(key, keybits / 8, "key:");
    LOG_HEXDUMP_INF(ciphertext, ct_len, "ciphertext:");
    LOG_HEXDUMP_INF(plaintext, ct_len, "plaintext (with padding):");

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
    LOG_INF("%s len=%d", __func__, len);
    LOG_HEXDUMP_INF(data, len, "uart_trans_ble:");

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

    prva_a = val & 0xff;
    prva_a <<= 8;
    prva_a |= val & 0xff;

    prva_key = prva_a;
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
    uint8_t tx_ble_buf[BLE_RESP_LENGTH_MAX];
    uint8_t encrypt_out[BLE_RESP_LENGTH_MAX];
    int ret;

    if (data == NULL || (len > BLE_RESP_LENGTH_MAX) || (len % BLE_CMD_DATA_LEN_UNIT))
    {
        LOG_INF("ble packet data or len(%d) error!", len);
        return;
    }

    // 封装包头、包类型
    tx_ble_buf[0] = (BLE_DATA_PACKET_HEAD >> 8) & 0xFF;
    tx_ble_buf[1] = BLE_DATA_PACKET_HEAD & 0xFF;
    tx_ble_buf[2] = (type >> 8) & 0xFF;
    tx_ble_buf[3] = type & 0xFF;

    memcpy(&tx_ble_buf[4], data, len);

    LOG_HEXDUMP_INF(tx_ble_buf, len + 4, "tx_ble:");

    // 密钥交换不需要加密
    if (type == BLE_DATA_TYPE_PKEY)
    {
        memcpy(encrypt_out, data, len);
    }
    else
    {
        /* AES-128-ECB 加密 */
        ret = aes_ecb_encrypt(aes_base_key, 128, data, len,
                             encrypt_out, sizeof(encrypt_out));
        if (ret != 0) 
        {
            LOG_INF("aes_ecb_encrypt! ret=%d", ret);
            return;
        }
    }

    memcpy(&tx_ble_buf[4], encrypt_out, len);
    BLE_DataTransOverBle(tx_ble_buf, len + 4);
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
    temp *= prva_a;                         // 拿到对方的公钥*自己的随机数

    aes_base_key[15] = (temp & 0xFF);
    aes_base_key[14] = (temp >> 8) & 0xFF;
    aes_base_key[13] = (temp >> 16) & 0xFF;
    aes_base_key[12] = (temp >> 24) & 0xFF;
    aes_base_key[11] = (temp >> 32) & 0xFF;
    aes_base_key[10] = (temp >> 40) & 0xFF;

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

    LOG_HEXDUMP_INF(aes_base_key, 16, "ble_aes_base_key:");
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
    uint8_t out_data[BLE_CMD_DATA_LEN_UNIT] = {0};
    uint8_t i;
    uint32_t val = 0;

    out_data[0] = BLE_COMU_CMD_START;
    out_data[1] = 0x03;                 // len = device code + cmd + param
    out_data[2] = BLE_COMU_DEV_CODE;
    out_data[3] = cmd;
    out_data[4] = param;

    for(i = 5; i < BLE_CMD_DATA_LEN_UNIT; i++)
    {
        my_generate_random(&val);
        out_data[i] = val & 0xff;
    }

    ble_comu_send_packet(BLE_DATA_TYPE_CMD, out_data, BLE_CMD_DATA_LEN_UNIT);
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
    const GsmImei_t *gsmImei = my_param_get_imei();

    if (gsmImei->flag == FLAG_VALID)
    {
        if (memcmp(data, gsmImei->hex, GSM_IMEI_LENGTH) == 0)
        {
            ble_comu_response_cmd(BLE_RSP_CMD_CID, BLE_RSP_PARAM_SUCCESS);
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

/*********************************************************************
**函数名称:  ble_comu_app_handle
**入口参数:  type        --  收到的蓝牙数据类型
**           data        --  收到的蓝牙数据指针（去除包头后）
**           len         --  收到的蓝牙数据长度（去除包头后）
**出口参数:  无
**函数功能:  根据数据类型分发处理：PKEY类型直接处理，其他类型解密后处理
**返 回 值:  无
*********************************************************************/
void ble_comu_app_handle(uint32_t type, const uint8_t *data, uint16_t len)
{
    int ret;
    uint8_t dec_buf[BLE_SVC_RX_MAX_LEN - 4] = {0};

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
            /* AES-128-ECB 解密 */
            ret = aes_ecb_decrypt(aes_base_key, 128,
                                data, len,
                                dec_buf, sizeof(dec_buf));
            if (ret != 0) 
            {
                LOG_INF("aes_ecb_decrypt error!");
                return;
            }

            LOG_INF("rx_ble:%x", type);

            LOG_HEXDUMP_INF(data, len, "original data:");
            LOG_HEXDUMP_INF(aes_base_key, 16, "ble_aes_base_key:");
            LOG_HEXDUMP_INF(dec_buf, len, "ble_decrypt:");

            switch(type)
            {
                case BLE_DATA_TYPE_CID:         //串号数据
                    ble_comu_cid_data_handle(dec_buf, len);
                    break;

                default:
                    LOG_INF("ble packet type error!");
                    break;
            }
        }
    }
}

/*********************************************************************
**函数名称:  ble_app_comm_data_proc
**入口参数:  data        --  收到的完整蓝牙数据包指针
**           len         --  收到的完整蓝牙数据包长度
**出口参数:  无
**函数功能:  解析蓝牙数据包头和数据类型，验证包头合法性后分发处理
**返 回 值:  无
*********************************************************************/
void ble_app_comm_data_proc(const uint8_t *data, uint16_t len)
{
    uint16_t pkt_head, pkt_type;

    pkt_head = *(data + 0);
    pkt_head <<= 8;
    pkt_head |= *(data + 1);

    pkt_type = *(data + 2);
    pkt_type <<= 8;
    pkt_type |= *(data + 3);

    if(pkt_head != BLE_DATA_PACKET_HEAD)
    {
        LOG_INF("ble packet head(0x%04x) error!", pkt_head);
    }
    else
    {
        LOG_INF("ble packet type:0x%04x,len=%d", pkt_type, len);
        ble_comu_app_handle(pkt_type, data + 4, len - 4);
    }
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
    if (rx_ble_buf_index == 0 && len <= BLE_SERVER_MAX_DATA_LEN && len >= BLE_GATT_FRAME_LEN_MIN)     // 长度检查
    {
        memcpy(&rx_ble_buf[0], data, len);
        rx_ble_buf_index = len;

        LOG_HEXDUMP_INF(rx_ble_buf, BLE_GATT_FRAME_LEN_MIN, "BLE_DataInput:");
        my_send_msg(MOD_MAIN, MOD_BLE, MY_MSG_BLE_RX);
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
    uint16_t read_packet_head = (rx_ble_buf[0] << 8) | rx_ble_buf[1];
    uint16_t read_cmd_head = (rx_ble_buf[2] << 8) | rx_ble_buf[3];

    if(read_packet_head == BLE_DATA_PACKET_HEAD)
    {
        if(read_cmd_head == BLE_DATA_TYPE_FILE_TRANS)
        {
            // TODO
        }
        else
        {
            ble_app_comm_data_proc(rx_ble_buf, rx_ble_buf_index);   // 指令处理
        }
    }
    else
    {
        LOG_INF("ble packet head(0x%04x) error!", read_packet_head);
    }

    rx_ble_buf_index = 0;
}