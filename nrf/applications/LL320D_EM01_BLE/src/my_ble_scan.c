/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        my_ble_scan.c
**文件描述:        设备扫描模块实现
**当前版本:        V1.0
**作    者:       周森达 (zhousenda@jimiiot.com)
**完成日期:        2026.04.14
*********************************************************************
** 功能描述:        设备扫描模块实现
**                 1. 实现主动扫描，分离处理ADV_IND和SCAN_RSP数据
**                 2. ADV_IND解析FF数据并缓存，SCAN_RSP解析名称/UUID/电量
**                 3. 名称前缀过滤通过后，合并ADV缓存与SCAN_RSP数据
**                 4. 支持四种工作模式：关闭/唤醒扫描/周期缓存/周期上报
**                 5. 按MAC地址聚合结果，表满时替换最小RSSI设备
**                 6. 使用消息队列实现无锁设计，所有操作在BLE线程中串行执行
*********************************************************************/
#include "my_comm.h"

LOG_MODULE_REGISTER(my_ble_scan, LOG_LEVEL_INF);

static tag_prefix_table_t s_prefix_table;       // 前缀表
static tag_scan_result_table_t s_result_table;  // 结果表
static scan_config_t s_scan_config;             // 扫描配置

/* ADV数据缓存表（用于等待SCAN_RSP名称过滤后再合并） */
static adv_cache_item_t s_adv_cache_table[ADV_CACHE_MAX_NUM];
/* 全局序号计数器，用于标记缓存项新旧 */
static uint32_t s_cache_seq_counter;

K_MSGQ_DEFINE(s_tag_scan_process_msgq, sizeof(tag_scan_process_msg_t), 16, 4);

/* 透传MAC扫描结果表和消息队列 */
static tran_mac_result_table_t s_tran_mac_result_table;
K_MSGQ_DEFINE(s_tran_mac_process_msgq, sizeof(tran_mac_process_msg_t), 16, 4);

/* 扫描参数（主动扫描模式）
 * 将 interval 设大、window 设小，可以大幅降低功耗
 */
static struct bt_le_scan_param s_scan_param = {
    .type       = BT_LE_SCAN_TYPE_ACTIVE,           // 主动扫描
    .options    = BT_LE_SCAN_OPT_FILTER_DUPLICATE,
    .interval   = 0x0640,                           // 1000 ms间隔
    .window     = BT_GAP_SCAN_FAST_WINDOW,          // 30 ms时长
    .timeout    = 0,
};

/* 扫描回调结构 */
static struct bt_le_scan_cb s_scan_cb;

// 定义上传mac和tag扫描结果状态枚举
typedef enum {
    UPLOAD_STATE_IDLE = 0,             // 初始空闲状态
    UPLOAD_STATE_TAG_START,            // 发送 TAG START
    UPLOAD_STATE_TAG_END,              // 发送 TAG END
    UPLOAD_STATE_MAC_START,            // 发送 MACINFO START
    UPLOAD_STATE_MAC_END               // 发送 MACINFO END
} upload_state_t;

//tag扫描结果上报和macinfo的序列号
static uint16_t s_tag_macinfo_seq = 0;
static uint8_t s_tag_macinfo_upload_state = UPLOAD_STATE_IDLE;
/* 本轮已从FLASH读出并发送的记录数；上报先发FLASH再发实时表，二者共用连续序号，
 * 发实时表时用"已发序号 - 这个数"把序号换算回实时表自己从0起的下标
 */
static uint16_t s_flash_consumed = 0;
//本轮上报开始时记下实时表有几条，本轮最多发这么多，发送途中表里新增的下轮再发
static uint16_t s_rt_count_snap = 0;

/********************************************************************
**函数名称:  parse_scan_rsp_name
**入口参数:  buf_ptr         ---        接收数据缓冲区
**           name      ---        输出设备名称缓冲区
**           max_len     ---        名称缓冲区最大长度
**出口参数:  name      ---        存储解析出的设备名称
**函数功能:  从SCAN_RSP数据中解析设备名称
**返 回 值:  true表示解析成功，false表示解析失败
*********************************************************************/
static bool parse_scan_rsp_name(struct net_buf_simple *buf_ptr, char *name, uint8_t max_len)
{
    struct net_buf_simple data_buf;
    uint8_t len;
    uint8_t type;
    uint8_t data_len;
    uint8_t copy_len;

    // 克隆缓冲区结构体，以便独立移动读取指针，不影响原始缓冲区
    net_buf_simple_clone(buf_ptr, &data_buf);

    // 遍历SCAN_RSP数据，解析AD结构
    while (data_buf.len > 1)
    {
        // 读取AD结构长度字段
        len = net_buf_simple_pull_u8(&data_buf);
        if (len == 0)
        {
            break;
        }

        if (data_buf.len < len)
        {
            return false;
        }

        // 读取AD结构类型字段
        type = net_buf_simple_pull_u8(&data_buf);
        // 计算数据字段长度 = 总长度 - 类型字段(1字节)
        data_len = len - 1;

        // 检查是否为设备名称类型
        if (type == BT_DATA_NAME_SHORTENED || type == BT_DATA_NAME_COMPLETE)
        {
            // 复制设备名称，预留1字节给字符串结束符
            copy_len = MIN(data_len, (uint8_t)(max_len - 1));
            memcpy(name, net_buf_simple_pull_mem(&data_buf, copy_len), copy_len);
            name[copy_len] = '\0';
            if (data_len > copy_len)
            {
                net_buf_simple_pull_mem(&data_buf, data_len - copy_len);
            }
            return true;
        }

        // 跳过非名称类型的数据字段
        net_buf_simple_pull_mem(&data_buf, data_len);
    }

    return false;
}

/********************************************************************
**函数名称:  parse_ff_data
**入口参数:  buf_ptr         ---        接收数据缓冲区
**           ff_data   ---        输出FF数据缓冲区
**           max_len     ---        FF数据缓冲区最大长度
**出口参数:  ff_data   ---        存储解析出的FF数据
**           out_len_ptr     ---        输出FF数据实际长度
**函数功能:  从广播数据中解析FF数据（Manufacturer Specific Data）
**返 回 值:  true表示解析成功，false表示解析失败
*********************************************************************/
static bool parse_ff_data(struct net_buf_simple *buf_ptr, uint8_t *ff_data,
                          uint8_t max_len, uint8_t *out_len_ptr)
{
    struct net_buf_simple data_buf;
    uint8_t len;
    uint8_t type;
    uint8_t data_len;
    uint8_t copy_len;

    // 克隆缓冲区结构体，以便独立移动读取指针
    net_buf_simple_clone(buf_ptr, &data_buf);

    // 遍历数据，解析AD结构
    while (data_buf.len > 1)
    {
        // 读取AD结构长度字段
        len = net_buf_simple_pull_u8(&data_buf);
        if (len == 0)
        {
            break;
        }

        if (data_buf.len < len)
        {
            return false;
        }

        // 读取AD结构类型字段
        type = net_buf_simple_pull_u8(&data_buf);
        // 计算数据字段长度 = 总长度 - 类型字段(1字节)
        data_len = len - 1;

        // 检查是否为Manufacturer Specific Data类型(0xFF)
        if (type == BT_DATA_MANUFACTURER_DATA && data_len == TAG_FF_DATA_MAX_LEN)
        {
            // 复制FF数据
            copy_len = MIN(data_len, max_len);
            memcpy(ff_data, net_buf_simple_pull_mem(&data_buf, copy_len), copy_len);
            if (data_len > copy_len)
            {
                net_buf_simple_pull_mem(&data_buf, data_len - copy_len);
            }
            *out_len_ptr = copy_len;
            return true;
        }

        // 跳过非FF类型的数据字段
        net_buf_simple_pull_mem(&data_buf, data_len);
    }

    return false;
}

/********************************************************************
**函数名称:  parse_scan_rsp_payload
**入口参数:  buf_ptr         ---        接收数据缓冲区
**           result_ptr      ---        扫描结果结构体指针
**出口参数:  result_ptr      ---        更新UUID和电量字段
**函数功能:  从扫描响应包中解析UUID和电量百分比
**           UUID在0x02类型中
**           电量在0x04类型中，格式为：第七字节=0x02(类型)，第八字节=电量值
**返 回 值:  true表示解析到至少一个字段，false表示全部解析失败
*********************************************************************/
static bool parse_scan_rsp_payload(struct net_buf_simple *buf_ptr, tag_scan_result_t *result_ptr)
{
    struct net_buf_simple data_buf;
    uint8_t len;
    uint8_t type;
    uint8_t data_len;
    uint8_t copy_len;
    const uint8_t *data_ptr;
    bool uuid_found = false;
    bool battery_found = false;

    // 克隆缓冲区结构体
    net_buf_simple_clone(buf_ptr, &data_buf);

    // 遍历数据，解析AD结构
    while (data_buf.len > 1)
    {
        // 读取AD结构长度字段
        len = net_buf_simple_pull_u8(&data_buf);
        if (len == 0)
        {
            break;
        }

        if (data_buf.len < len)
        {
            return (uuid_found || battery_found);
        }

        // 读取AD结构类型字段
        type = net_buf_simple_pull_u8(&data_buf);
        // 计算数据字段长度
        data_len = len - 1;
        data_ptr = net_buf_simple_pull_mem(&data_buf, data_len);

        // 检查是否为16-bit UUID类型(0x02)
        if ((type == BT_DATA_UUID16_SOME) && (!uuid_found))
        {
            // 复制UUID数据
            copy_len = MIN(data_len, UUID_MAX_LEN);
            memcpy(result_ptr->uuid, data_ptr, copy_len);
            result_ptr->uuid_len = copy_len;
            uuid_found = true;
        }
        // 检查是否包含电量字段(类型0x04)
        else if ((type == BT_DATA_UUID32_SOME) && (!battery_found) && (data_len >= 8))
        {
            // 检查第七字节是否为0x02（电量数据类型）
            if (data_ptr[6] == 0x02)
            {
                result_ptr->battery_percent = data_ptr[7];
                battery_found = true;
            }
        }
    }

    return (uuid_found || battery_found);
}

/********************************************************************
**函数名称:  tag_name_match
**入口参数:  name      ---        设备名称
**           len         ---        名称长度
**出口参数:  无
**函数功能:  检查设备名称是否匹配前缀表中的任一前缀
**返 回 值:  true表示匹配成功，false表示不匹配
*********************************************************************/
static bool tag_name_match(const char *name, uint8_t len)
{
    uint8_t i;
    uint8_t prefix_len;

    // 参数校验
    if (len == 0 || name == NULL)
    {
        return false;
    }

    // 遍历前缀表，匹配任一前缀
    for (i = 0; i < s_prefix_table.count; i++)
    {
        // 跳过无效前缀
        if (!s_prefix_table.items[i].valid)
        {
            continue;
        }

        // 获取前缀长度
        prefix_len = s_prefix_table.items[i].len;
        // 比较前缀（名称长度需大于等于前缀长度）
        if (len >= prefix_len &&
            strncmp(name, s_prefix_table.items[i].prefix, prefix_len) == 0)
        {
            return true;
        }
    }

    return false;
}

/********************************************************************
**函数名称:  tag_table_flush_to_flash
**入口参数:  无
**出口参数:  无
**函数功能:  将本轮实时TAG表按时间戳从小到大排序后逐条落入FLASH循环存储，
**           落盘后清空实时表；用于周期扫描模式下扫描时长结束但未立即上报时，
**           缓存本轮历史数据
**返 回 值:  无
**注意事项:  仅BLE线程调用；本接口在每轮扫描结束或实时表满时调用，落盘后实时表归零
*********************************************************************/
static void tag_table_flush_to_flash(void)
{
    uint16_t i;
    uint16_t j;
    uint16_t min_idx;
    tag_scan_result_t tmp;
    int ret;

    // 实时表为空无需落盘
    if (s_result_table.count == 0)
    {
        return;
    }

    // 落盘前按时间戳从小到大排序(选择排序)，确保FLASH里按采集先后顺序存储
    for (i = 0; i < s_result_table.count - 1; i++)
    {
        min_idx = i;
        for (j = i + 1; j < s_result_table.count; j++)
        {
            if (s_result_table.items[j].timestamp < s_result_table.items[min_idx].timestamp)
            {
                min_idx = j;
            }
        }

        if (min_idx != i)
        {
            tmp = s_result_table.items[i];
            s_result_table.items[i] = s_result_table.items[min_idx];
            s_result_table.items[min_idx] = tmp;
        }
    }

    // 逐条把本轮实时表中的TAG记录推入FLASH循环存储暂存缓冲
    for (i = 0; i < s_result_table.count; i++)
    {
        ret = my_flash_store_push_tag(&s_result_table.items[i]);
        if (ret != 0)
        {
            LOG_ERR("flush tag to flash failed at %d (ret %d)", i, ret);
        }
    }

    LOG_INF("flush %d tag records to flash", s_result_table.count);

    // 本轮数据已落盘，清空实时表，下一轮重新扫描收集
    memset(&s_result_table, 0, sizeof(s_result_table));
}

/********************************************************************
**函数名称:  tag_scan_result_save
**入口参数:  result_ptr      ---        扫描结果指针
**出口参数:  无
**函数功能:  保存扫描结果到结果表（按地址聚合最终有效数据）
**返 回 值:  无
*********************************************************************/
static void tag_scan_result_save(tag_scan_result_t *result_ptr)
{
    uint8_t i;
    tag_scan_result_t *item_ptr;

    // 检查是否已存在相同MAC地址的设备
    for (i = 0; i < s_result_table.count; i++)
    {
        if (bt_addr_le_cmp(&s_result_table.items[i].addr, &result_ptr->addr) == 0)
        {
            item_ptr = &s_result_table.items[i];

            // 更新设备名称和RSSI
            strncpy(item_ptr->name, result_ptr->name, sizeof(item_ptr->name) - 1);
            item_ptr->rssi = result_ptr->rssi;

            // 更新FF数据
            if (result_ptr->ff_data_len > 0)
            {
                memcpy(item_ptr->ff_data, result_ptr->ff_data, result_ptr->ff_data_len);
                item_ptr->ff_data_len = result_ptr->ff_data_len;
            }

            // 更新UUID数据
            if (result_ptr->uuid_len > 0)
            {
                memcpy(item_ptr->uuid, result_ptr->uuid, result_ptr->uuid_len);
                item_ptr->uuid_len = result_ptr->uuid_len;
            }

            // 更新电量百分比
            item_ptr->battery_percent = result_ptr->battery_percent;

            // 更新采集时间戳为本次扫描到该设备的时刻
            item_ptr->timestamp = my_get_system_time_sec();

            LOG_INF("TAG updated: %s, RSSI: %d", item_ptr->name, item_ptr->rssi);

#if 0
            if (item_ptr->ff_data_len > 0)
            {
                LOG_HEXDUMP_INF(item_ptr->ff_data, item_ptr->ff_data_len, "TAG FF Data:");
            }

            if (item_ptr->uuid_len > 0)
            {
                LOG_HEXDUMP_INF(item_ptr->uuid, item_ptr->uuid_len, "TAG UUID Data:");
            }
#endif

            return;
        }
    }

    // 添加新设备
    if (s_result_table.count < TAG_RESULT_MAX_NUM)
    {
        item_ptr = &s_result_table.items[s_result_table.count];
        memcpy(item_ptr, result_ptr, sizeof(tag_scan_result_t));
        // 记录采集到该设备的时刻
        item_ptr->timestamp = my_get_system_time_sec();
        s_result_table.count++;

        LOG_INF("TAG found: %s, RSSI: %d, count: %d",
                result_ptr->name, result_ptr->rssi, s_result_table.count);

#if 0
        if (result_ptr->ff_data_len > 0)
        {
            LOG_HEXDUMP_INF(result_ptr->ff_data, result_ptr->ff_data_len, "TAG FF Data:");
        }

        if (result_ptr->uuid_len > 0)
        {
            LOG_HEXDUMP_INF(result_ptr->uuid, result_ptr->uuid_len, "TAG UUID Data:");
        }
#endif

    }
    // 实时表已满，先把当前整表按时间戳排序后落入FLASH并清空，再把本条新记录存入清空后的新表
    else
    {
        LOG_WRN("TAG result table full, flush %d records to flash, rssi=%d",
                s_result_table.count, result_ptr->rssi);

        // 排序+整表落盘+清空实时表(count归零)
        tag_table_flush_to_flash();

        // 触发溢出的本条新记录存入清空后的新表，实时表从头重新累积
        item_ptr = &s_result_table.items[s_result_table.count];
        memcpy(item_ptr, result_ptr, sizeof(tag_scan_result_t));
        // 记录采集到该设备的时刻
        item_ptr->timestamp = my_get_system_time_sec();
        s_result_table.count++;
    }
}

/********************************************************************
**函数名称:  scan_recv_cb
**入口参数:  info_ptr        ---        扫描接收信息
**           buf_ptr         ---        接收数据缓冲区
**出口参数:  无
**函数功能:  扫描接收回调函数，接收ADV和SCAN_RSP数据并发送消息到BLE线程
**返 回 值:  无
*********************************************************************/
static void scan_recv_cb(const struct bt_le_scan_recv_info *info_ptr,
                              struct net_buf_simple *buf_ptr)
{
    char name[ADV_NAME_MAX_LEN] = {0};
    uint8_t ff_temp[TAG_FF_DATA_MAX_LEN];
    uint8_t ff_len;
    tag_scan_process_msg_t process_msg;
    tran_mac_process_msg_t tmac_msg;
    int err;

    // 只处理广播包和扫描响应包
    if (info_ptr->adv_type != BT_GAP_ADV_TYPE_ADV_IND &&
        info_ptr->adv_type != BT_GAP_ADV_TYPE_SCAN_RSP)
    {
        return;
    }

    memset(&process_msg, 0, sizeof(tag_scan_process_msg_t));

    process_msg.adv_type = info_ptr->adv_type;
    bt_addr_le_copy(&process_msg.result.addr, info_ptr->addr);
    process_msg.result.rssi = info_ptr->rssi;

    if (info_ptr->adv_type == BT_GAP_ADV_TYPE_ADV_IND)
    {
        // 处理广播包(ADV_IND)，解析厂商自定义FF数据
        ff_len = 0;
        if (parse_ff_data(buf_ptr, ff_temp, sizeof(ff_temp), &ff_len))
        {
            memcpy(process_msg.result.ff_data, ff_temp, ff_len);
            process_msg.result.ff_data_len = ff_len;

            // 将处理后的消息放入消息队列，使用非阻塞方式
            err = k_msgq_put(&s_tag_scan_process_msgq, &process_msg, K_NO_WAIT);
            if (err)
            {
                // 消息队列已满或其他错误，直接返回
                return;
            }

            // 发送消息到BLE线程处理
            my_send_msg(MOD_BLE, MOD_BLE, MY_MSG_TAG_SCAN_PROCESS);
        }

        // 透传MAC功能：检查MAC是否匹配，匹配则复制广播数据入队
        if (my_tran_mac_check(info_ptr->addr))
        {
            memset(&tmac_msg, 0, sizeof(tran_mac_process_msg_t));
            bt_addr_le_copy(&tmac_msg.addr, info_ptr->addr);
            // 复制广播数据（不含字节序翻转，按原始buf_ptr拷贝）
            tmac_msg.adv_data_len = MIN(buf_ptr->len, TRAN_MAC_ADV_DATA_MAX_LEN);
            memcpy(tmac_msg.adv_data, buf_ptr->data, tmac_msg.adv_data_len);

            err = k_msgq_put(&s_tran_mac_process_msgq, &tmac_msg, K_NO_WAIT);
            if (!err)
            {
                my_send_msg(MOD_BLE, MOD_BLE, MY_MSG_TRAN_MAC_PROCESS);
            }
        }
    }
    else
    {
        // 处理扫描响应包(SCAN_RSP)，解析设备名称和载荷数据
        if (!parse_scan_rsp_name(buf_ptr, name, sizeof(name)))
        {
            return;
        }

        // 拷贝设备名称到消息结构体，预留一个字节存放结束符
        strncpy(process_msg.result.name, name,
                sizeof(process_msg.result.name) - 1);
        // 解析扫描响应载荷数据
        parse_scan_rsp_payload(buf_ptr, &process_msg.result);

        // 将处理后的消息放入消息队列，使用非阻塞方式
        err = k_msgq_put(&s_tag_scan_process_msgq, &process_msg, K_NO_WAIT);
        if (err)
        {
            return;
        }

        // 发送消息到BLE线程处理
        my_send_msg(MOD_BLE, MOD_BLE, MY_MSG_TAG_SCAN_PROCESS);
    }
}

/********************************************************************
**函数名称:  tag_scan_data_handle
**入口参数:  process_msg_ptr  ---        扫描处理消息指针
**出口参数:  无
**函数功能:  在BLE线程中处理扫描数据，完成过滤、缓存和结果保存
**返 回 值:  无
*********************************************************************/
static void tag_scan_data_handle(tag_scan_process_msg_t *process_msg_ptr)
{
    uint8_t i, j;
    uint8_t oldest_idx;
    uint32_t min_seq;
    bool found;
    tag_scan_result_t *result_ptr;

    if (process_msg_ptr == NULL)
    {
        return;
    }

    result_ptr = &process_msg_ptr->result;

    // 处理ADV_IND广播包：缓存FF数据，等待后续SCAN_RSP合并
    if (process_msg_ptr->adv_type == BT_GAP_ADV_TYPE_ADV_IND)
    {
        found = false;
        // 步骤1: 查找已存在的缓存并更新
        for (i = 0; i < ADV_CACHE_MAX_NUM; i++)
        {
            if (s_adv_cache_table[i].valid &&
                bt_addr_le_cmp(&s_adv_cache_table[i].addr, &result_ptr->addr) == 0)
            {
                memcpy(s_adv_cache_table[i].ff_data, result_ptr->ff_data, result_ptr->ff_data_len);
                s_adv_cache_table[i].ff_data_len = result_ptr->ff_data_len;
                s_adv_cache_table[i].seq_num = s_cache_seq_counter++;
                found = true;
                break;
            }
        }

        // 步骤2: 未找到则查找空闲位置新增缓存
        if (!found)
        {
            for (i = 0; i < ADV_CACHE_MAX_NUM; i++)
            {
                if (!s_adv_cache_table[i].valid)
                {
                    bt_addr_le_copy(&s_adv_cache_table[i].addr, &result_ptr->addr);
                    memcpy(s_adv_cache_table[i].ff_data, result_ptr->ff_data, result_ptr->ff_data_len);
                    s_adv_cache_table[i].ff_data_len = result_ptr->ff_data_len;
                    s_adv_cache_table[i].valid = true;
                    s_adv_cache_table[i].seq_num = s_cache_seq_counter++;
                    found = true;
                    break;
                }
            }
        }

        // 步骤3: 缓存已满则替换最旧条目
        if (!found)
        {
            min_seq = s_adv_cache_table[0].seq_num;
            oldest_idx = 0;
            for (j = 1; j < ADV_CACHE_MAX_NUM; j++)
            {
                if (s_adv_cache_table[j].seq_num < min_seq)
                {
                    min_seq = s_adv_cache_table[j].seq_num;
                    oldest_idx = j;
                }
            }

            bt_addr_le_copy(&s_adv_cache_table[oldest_idx].addr, &result_ptr->addr);
            memcpy(s_adv_cache_table[oldest_idx].ff_data, result_ptr->ff_data, result_ptr->ff_data_len);
            s_adv_cache_table[oldest_idx].ff_data_len = result_ptr->ff_data_len;
            s_adv_cache_table[oldest_idx].valid = true;
            s_adv_cache_table[oldest_idx].seq_num = s_cache_seq_counter++;
            LOG_WRN("ADV cache full, overwrite oldest[%d], FF len: %d", oldest_idx, result_ptr->ff_data_len);
        }

#if 0
        LOG_HEXDUMP_INF(result_ptr->ff_data, result_ptr->ff_data_len, "Cached FF Data:");
#endif

        return;
    }

    // 处理SCAN_RSP响应包：名称过滤、FF数据合并、结果保存
    // 名称匹配过滤，不符合则丢弃，同时释放对应ADV缓存
    if (!tag_name_match(result_ptr->name, strlen(result_ptr->name)))
    {
        // 清理该MAC对应的ADV缓存，避免无效数据长期占用缓存槽
        for (i = 0; i < ADV_CACHE_MAX_NUM; i++)
        {
            if (s_adv_cache_table[i].valid &&
                bt_addr_le_cmp(&s_adv_cache_table[i].addr, &result_ptr->addr) == 0)
            {
                s_adv_cache_table[i].valid = false;
                break;
            }
        }
        return;
    }

    // 从缓存中查找并合并FF数据
    for (i = 0; i < ADV_CACHE_MAX_NUM; i++)
    {
        if (s_adv_cache_table[i].valid &&
            bt_addr_le_cmp(&s_adv_cache_table[i].addr, &result_ptr->addr) == 0)
        {
            memcpy(result_ptr->ff_data, s_adv_cache_table[i].ff_data,
                   s_adv_cache_table[i].ff_data_len);
            result_ptr->ff_data_len = s_adv_cache_table[i].ff_data_len;
            s_adv_cache_table[i].valid = false;
            LOG_INF("FF data merged from cache[%d], len: %d", i, result_ptr->ff_data_len);
            break;
        }
    }

#if 0
    if (result_ptr->uuid_len > 0)
    {
        LOG_INF("TAG UUID found, len: %d", result_ptr->uuid_len);
        LOG_HEXDUMP_INF(result_ptr->uuid, result_ptr->uuid_len, "UUID Data:");
    }

    LOG_INF("TAG battery: %d%%", result_ptr->battery_percent);
#endif

    // 保存TAG扫描结果
    tag_scan_result_save(result_ptr);
}

/********************************************************************
**函数名称:  tag_prefix_table_init
**入口参数:  无
**出口参数:  无
**函数功能:  初始化前缀表，添加默认前缀
**返 回 值:  无
*********************************************************************/
static void tag_prefix_table_init(void)
{
    memset(&s_prefix_table, 0, sizeof(s_prefix_table));

    // 添加默认前缀
    my_tag_prefix_add("L780");
    my_tag_prefix_add("PB7");
    my_tag_prefix_add("LL311");
    my_tag_prefix_add("PET");
    // 未来可继续添加...
}

/********************************************************************
**函数名称:  scan_interval_timer_cb
**入口参数:  param         ---        定时器参数
**出口参数:  无
**函数功能:  周期扫描定时器回调函数，发送消息到BLE线程
**返 回 值:  无
*********************************************************************/
static void scan_interval_timer_cb(void *param)
{
    // 发送消息到BLE线程处理
    my_send_msg(MOD_BLE, MOD_BLE, MY_MSG_SCAN_INTERVAL);
}

/********************************************************************
**函数名称:  scan_length_timer_cb
**入口参数:  param         ---        定时器参数
**出口参数:  无
**函数功能:  单次扫描时长定时器回调函数，发送消息到BLE线程
**返 回 值:  无
*********************************************************************/
static void scan_length_timer_cb(void *param)
{
    // 发送消息到BLE线程处理
    my_send_msg(MOD_BLE, MOD_BLE, MY_MSG_SCAN_LENGTH);
}

/********************************************************************
**函数名称:  upload_interval_timer_cb
**入口参数:  param         ---        定时器参数
**出口参数:  无
**函数功能:  上报间隔定时器回调函数，发送消息到BLE线程
**返 回 值:  无
*********************************************************************/
static void upload_interval_timer_cb(void *param)
{
    // 发送消息到BLE线程处理
    my_send_msg(MOD_BLE, MOD_BLE, MY_MSG_SCAN_UPLOAD);
}

/********************************************************************
**函数名称:  scan_start_internal
**入口参数:  无
**出口参数:  无
**函数功能:  内部函数：启动主动扫描，在BLE线程中调用
**返 回 值:  0表示成功，负值表示失败
*********************************************************************/
static int scan_start_internal(void)
{
    int err;

    if (s_scan_config.mode == SCAN_MODE_OFF)
    {
        return -EINVAL;
    }

    if (s_scan_config.state == SCAN_STATE_SCANNING)
    {
        LOG_WRN("ble scan already in progress");
        return 0;
    }

    // 启动扫描，使用扫描参数
    err = bt_le_scan_start(&s_scan_param, NULL);
    if (err)
    {
        LOG_ERR("Failed to start scan (err %d)", err);
        return err;
    }

    s_scan_config.state = SCAN_STATE_SCANNING;
    LOG_INF("ble scan started");

    // 启动单次扫描时长定时器
    if (s_scan_config.scan_length > 0)
    {
        my_start_timer(MY_TIMER_SCAN_LENGTH, s_scan_config.scan_length * 1000,
                       false, scan_length_timer_cb);
    }

    return 0;
}

/********************************************************************
**函数名称:  scan_stop_internal
**入口参数:  无
**出口参数:  无
**函数功能:  内部函数：停止扫描，在BLE线程中调用
**返 回 值:  0表示成功，负值表示失败
*********************************************************************/
static int scan_stop_internal(void)
{
    int err;

    if (s_scan_config.state != SCAN_STATE_SCANNING)
    {
        return 0;
    }

    err = bt_le_scan_stop();
    if (err)
    {
        LOG_ERR("Failed to stop scan (err %d)", err);
        return err;
    }

    s_scan_config.state = SCAN_STATE_IDLE;
    LOG_INF("ble scan stopped");

    // 清空ADV缓存表和缓存序号计数器，避免跨扫描周期残留
    memset(&s_adv_cache_table, 0, sizeof(s_adv_cache_table));
    s_cache_seq_counter = 0;

    // 停止单次扫描时长定时器
    my_stop_timer(MY_TIMER_SCAN_LENGTH);

    return 0;
}

/********************************************************************
**函数名称:  scan_set_config_internal
**入口参数:  mode        ---        工作模式（0-2）
**           scan_interval ---      扫描间隔（秒）
**           scan_length ---        单次扫描时长（秒）
**           upload_interval ---    上报间隔（秒）
**出口参数:  无
**函数功能:  内部函数：设置扫描配置，在BLE线程中调用
**返 回 值:  无
*********************************************************************/
static void scan_set_config_internal(uint8_t mode, uint32_t scan_interval,
                                  uint32_t scan_length, uint32_t upload_interval)
{
    // 停止所有定时器
    my_stop_timer(MY_TIMER_SCAN_INTERVAL);
    my_stop_timer(MY_TIMER_SCAN_LENGTH);
    my_stop_timer(MY_TIMER_UPLOAD_INTERVAL);

    // 停止当前扫描
    scan_stop_internal();

    // 清空数据
    memset(&s_result_table, 0, sizeof(s_result_table));
    memset(&s_tran_mac_result_table, 0, sizeof(s_tran_mac_result_table));
    s_tag_macinfo_seq = 0;
    s_tag_macinfo_upload_state = 0;
    s_flash_consumed = 0;
    s_rt_count_snap = 0;
    // 把FLASH读取位置退回上次已确认处(只退读位置，FLASH里的历史数据保留，下次重新上报)
    my_flash_store_rewind(FS_TYPE_TAG);
    my_flash_store_rewind(FS_TYPE_MAC);

    // 更新配置
    s_scan_config.mode = (scan_mode_t)mode;
    s_scan_config.scan_interval = scan_interval;
    s_scan_config.scan_length = scan_length;
    s_scan_config.upload_interval = upload_interval;
    s_scan_config.state = SCAN_STATE_IDLE;

    LOG_INF("scan config: Mode=%d, ScanInterval=%us, ScanLength=%us, UploadInterval=%us",
            mode, scan_interval, scan_length, upload_interval);

    // 根据模式启动相应功能
    switch (s_scan_config.mode)
    {
        case SCAN_MODE_OFF:
            // Mode 0：关闭所有功能
            LOG_INF("scan disabled");
            break;

        case SCAN_MODE_PERIOD_CACHE:
            // Mode 1：周期扫描，等待LTE唤醒时上报
            LOG_INF("scan Mode 1: Periodic scan started");

            my_start_timer(MY_TIMER_SCAN_INTERVAL, s_scan_config.scan_interval * 1000,
                            true, scan_interval_timer_cb);

            scan_start_internal();
            break;

        case SCAN_MODE_PERIOD_UPLOAD:
            // Mode 2：周期扫描 + 定时上报
            LOG_INF("scan Mode 2: Periodic scan and upload started");

            my_start_timer(MY_TIMER_SCAN_INTERVAL, s_scan_config.scan_interval * 1000,
                            true, scan_interval_timer_cb);

            my_start_timer(MY_TIMER_UPLOAD_INTERVAL, s_scan_config.upload_interval * 1000,
                            true, upload_interval_timer_cb);

            scan_start_internal();
            break;

        default:
            LOG_ERR("Invalid scan mode: %d", mode);
            break;
    }
}

/********************************************************************
**函数名称:  tran_mac_upload_one
**入口参数:  item_ptr --- 透传MAC地址结果项指针
**           seq     ---  序号
**出口参数:  无
**函数功能:  上传透传MAC地址结果项(单条)
**返 回 值:  无
*********************************************************************/
void tran_mac_upload_one(tran_mac_result_item_t *item_ptr, uint16_t seq)
{
    int i = 0;
    int offset;
    char upload_msg[256] = {0};
    char mac_str[13] = {0};

    offset = 0;
    memset(upload_msg, 0, sizeof(upload_msg));

    // 将MAC地址转换为字符串格式
    snprintf(mac_str, sizeof(mac_str), "%02X%02X%02X%02X%02X%02X",
                item_ptr->addr.a.val[5], item_ptr->addr.a.val[4],
                item_ptr->addr.a.val[3], item_ptr->addr.a.val[2],
                item_ptr->addr.a.val[1], item_ptr->addr.a.val[0]);

    // 先拼接seq,时间戳,MAC和广播数据长度字段（时间戳取采集时刻，非上报时刻）
    offset += snprintf(upload_msg + offset, sizeof(upload_msg) - offset, "%d,%u,%s,%d,",
                        seq, item_ptr->timestamp, mac_str, item_ptr->adv_data_len);

    // 将广播数据逐字节转换为HEX字符串并拼接到消息尾部
    for (i = 0; i < item_ptr->adv_data_len; i++)
    {
        offset += snprintf(upload_msg + offset, sizeof(upload_msg) - offset, "%02X",
                            item_ptr->adv_data[i]);
    }

#if RETRANSMIT_CHECK_ENABLED
    lte_send_cmd_with_retry("MACINFO", upload_msg);
#else
    lte_send_command("MACINFO", upload_msg);
#endif

}

/********************************************************************
**函数名称:  tag_scan_upload_one
**入口参数:  item_ptr --- TAG扫描结果项指针
**           seq     ---  序号
**出口参数:  无
**函数功能:  上传TAG扫描结果项（单条）
**返 回 值:  无
*********************************************************************/
void tag_scan_upload_one(tag_scan_result_t *item_ptr, uint16_t seq)
{
    int i = 0;
    char upload_msg[256] = {0};
    char mac_str[13] = {0};
    char uuid_str[UUID_MAX_LEN * 2 + 1] = {0};
    uint8_t rssi_upload;
    char ff_str[TAG_FF_DATA_MAX_LEN * 2 + 1] = {0};

    // 将MAC地址转换为字符串格式
    snprintf(mac_str, sizeof(mac_str), "%02X%02X%02X%02X%02X%02X",
                item_ptr->addr.a.val[5], item_ptr->addr.a.val[4],
                item_ptr->addr.a.val[3], item_ptr->addr.a.val[2],
                item_ptr->addr.a.val[1], item_ptr->addr.a.val[0]);

    // RSSI转换：实际值为负数，上传值为补码形式（按位取反+1）
    // 例如：实际-52(0xCC)，上传值0xCC
    rssi_upload = (uint8_t)item_ptr->rssi;

    // 构建UUID字符串（十六进制）
    if (item_ptr->uuid_len > 0)
    {
        memset(uuid_str, 0, sizeof(uuid_str));
        for (i = 0; i < item_ptr->uuid_len && i < UUID_MAX_LEN; i++)
        {
            snprintf(&uuid_str[i * 2], 3, "%02X", item_ptr->uuid[i]);
        }
    }
    else
    {
        strncpy(uuid_str, "N/A", sizeof(uuid_str) - 1);
    }

    // 构建FF数据字符串（十六进制）
    if (item_ptr->ff_data_len > 0)
    {
        memset(ff_str, 0, sizeof(ff_str));
        for (i = 0; i < item_ptr->ff_data_len && i < TAG_FF_DATA_MAX_LEN; i++)
        {
            snprintf(&ff_str[i * 2], 3, "%02X", item_ptr->ff_data[i]);
        }
        // 构建上报消息：seq,时间戳,MAC(6字节),电量,RSSI,名称长度,名称,UUID长度,UUID,FF长度,FF数据
        snprintf(upload_msg, sizeof(upload_msg), "%d,%u,%s,%d,%02X,%d,%s,%d,%s,%d,%s",
                    seq, item_ptr->timestamp, mac_str, item_ptr->battery_percent, rssi_upload,
                    strlen(item_ptr->name), item_ptr->name,
                    item_ptr->uuid_len, uuid_str,
                    item_ptr->ff_data_len, ff_str);
    }
    else
    {
        // 构建上报消息：seq,时间戳,MAC(6字节),电量,RSSI,名称长度,名称,UUID长度,UUID,FF长度(0),FF数据(无)
        snprintf(upload_msg, sizeof(upload_msg), "%d,%u,%s,%d,%02X,%d,%s,%d,%s,0,N/A",
                    seq, item_ptr->timestamp, mac_str, item_ptr->battery_percent, rssi_upload,
                    strlen(item_ptr->name), item_ptr->name,
                    item_ptr->uuid_len, uuid_str);
    }

#if RETRANSMIT_CHECK_ENABLED
    lte_send_cmd_with_retry("TAG", upload_msg);
#else
    // 发送TAG数据到LTE模块
    lte_send_command("TAG", upload_msg);
#endif

    LOG_INF("TAG uploaded: MAC:%s, Bat:%d%%, RSSI:0x%02X, NameLen:%d, Name:%s, UUIDLen:%d, UUID:%s, FFLen:%d",
            mac_str, item_ptr->battery_percent, rssi_upload,
            strlen(item_ptr->name), item_ptr->name,
            item_ptr->uuid_len, uuid_str, item_ptr->ff_data_len);

#if 0
// 打印聚合后的关键原始数据，方便抓包对比
if (item_ptr->ff_data_len > 0)
        {
            LOG_HEXDUMP_INF(item_ptr->ff_data, item_ptr->ff_data_len, "Upload FF Data:");
        }
#endif

}

/********************************************************************
**函数名称:  clear_tag_macinfo
**入口参数:  无
**出口参数:  无
**函数功能:  重置扫描与上传相关的状态机、序列号，并清空TAG及MAC地址缓存表
**返 回 值:  无
*********************************************************************/
void clear_tag_macinfo(void)
{
    s_tag_macinfo_seq = 0;
    s_tag_macinfo_upload_state = 0;
    s_flash_consumed = 0;
    s_rt_count_snap = 0;
    s_scan_config.state = SCAN_STATE_IDLE;

    memset(&s_result_table, 0, sizeof(s_result_table));
    memset(&s_tran_mac_result_table, 0, sizeof(s_tran_mac_result_table));

    // 把FLASH读取位置退回上次已确认处，没确认的数据下次重发(不清空FLASH里的存储)
    my_flash_store_rewind(FS_TYPE_TAG);
    my_flash_store_rewind(FS_TYPE_MAC);
}

/********************************************************************
**函数名称:  tag_mac_scan_upload_data
**入口参数:  无
**出口参数:  无
**函数功能:  状态机调度函数，负责按（START->DATA->DATA->...->END）分步上传
**             TAG数据和透传MAC地址数据。该函数需配合LTE模块的应答机制
**             多次调用以推进状态流转。
**返 回 值:  无
*********************************************************************/
void tag_mac_scan_upload_data(void)
{
    tag_scan_result_t *item_ptr;
    tran_mac_result_item_t *mac_item_ptr;
    tag_scan_result_t flash_tag;
    tran_mac_result_item_t flash_mac;
    uint32_t tag_total;
    uint32_t mac_total;
    uint16_t rt_idx;
    char buf[32];

    if (s_scan_config.state == SCAN_STATE_SCANNING)
    {
        LOG_WRN("scan is in progress, skip upload");
        return;
    }

    // 各类型待上报总数 = FLASH循环存储待上报数 + 实时表数量
    tag_total = my_flash_store_pending_count(FS_TYPE_TAG) + s_result_table.count;
    mac_total = my_flash_store_pending_count(FS_TYPE_MAC) + s_tran_mac_result_table.count;

    //1发2不发
    //1不发2发
    //1发2发
    //1不发2不发
    if (s_tag_macinfo_upload_state == UPLOAD_STATE_IDLE)
    {
        // 检查是否有TAG数据需要上报
        if (tag_total == 0)
        {
            LOG_INF("No TAG data to upload");
            // 检查macinfo
            if (mac_total == 0) //12不发
            {
                return ;
            }
            else
            {
                s_tag_macinfo_upload_state = UPLOAD_STATE_MAC_START;
                s_tag_macinfo_seq = 0;
                s_flash_consumed = 0;
                // 记下当前实时表有几条，锁定本轮可发数量，发送途中表里新增的不算进本轮
                s_rt_count_snap = s_tran_mac_result_table.count;
                // 开启上报会话：把读取位置退回已确认起点，并暂停落盘，保证实际发送数与START声明的总数一致
                my_flash_store_upload_begin(FS_TYPE_MAC);
                //先发送开始
                snprintf(buf, sizeof(buf), "START,%u", mac_total);
                buf[sizeof(buf) - 1] = '\0';
                #if RETRANSMIT_CHECK_ENABLED
                    lte_send_cmd_with_retry("MACINFO", buf);
                #else
                    lte_send_command("MACINFO", buf);
                #endif

                s_scan_config.state = SCAN_STATE_WAITING_UPLOAD;

                return;
            }
        }

        s_tag_macinfo_upload_state = UPLOAD_STATE_TAG_START;
        s_tag_macinfo_seq = 0;
        s_flash_consumed = 0;
        // 记下当前实时表有几条，锁定本轮可发数量，发送途中表里新增的不算进本轮
        s_rt_count_snap = s_result_table.count;
        // 开启上报会话：把读取位置退回已确认起点，并暂停落盘，保证实际发送数与START声明的总数一致
        my_flash_store_upload_begin(FS_TYPE_TAG);

        snprintf(buf, sizeof(buf), "START,%u", tag_total);
        buf[sizeof(buf) - 1] = '\0';
        //先发送START
        #if RETRANSMIT_CHECK_ENABLED
            lte_send_cmd_with_retry("TAG", buf);
        #else
            lte_send_command("TAG", buf);
        #endif

        //开始上传
        s_scan_config.state = SCAN_STATE_WAITING_UPLOAD;
    }
    else if (s_tag_macinfo_upload_state == UPLOAD_STATE_TAG_START)//应答start之后会发消息到蓝牙线程再次调用这个函数，开始上报数据体
    {
        // 优先上报FLASH循环存储中的历史TAG数据
        if (my_flash_store_read_next(FS_TYPE_TAG, &flash_tag) == 1)
        {
            s_flash_consumed++;
            s_tag_macinfo_seq++;
            tag_scan_upload_one(&flash_tag, s_tag_macinfo_seq);
            return;
        }

        // FLASH历史数据读完后，再发实时表里的数据；实时表下标 = 当前总序号 - 已从FLASH读出的条数
        rt_idx = s_tag_macinfo_seq - s_flash_consumed;
        if (rt_idx < s_rt_count_snap)
        {
            item_ptr = &s_result_table.items[rt_idx];
            s_tag_macinfo_seq++;
            tag_scan_upload_one(item_ptr, s_tag_macinfo_seq);
            return;
        }

        // TAG全部发完：先确认这批FLASH数据(把读取位置正式存盘，已发完的扇区腾空)，再清空实时表
        LOG_INF("TAG upload complete, total: %d, flash: %d", s_tag_macinfo_seq, s_flash_consumed);
        my_flash_store_commit(FS_TYPE_TAG);
        memset(&s_result_table, 0, sizeof(s_result_table));
        s_tag_macinfo_seq = 0;
        s_flash_consumed = 0;
        s_tag_macinfo_upload_state = UPLOAD_STATE_TAG_END;
        #if RETRANSMIT_CHECK_ENABLED
            lte_send_cmd_with_retry("TAG", "END");
        #else
            lte_send_command("TAG", "END");
        #endif
        return;
    }
    else if (s_tag_macinfo_upload_state == UPLOAD_STATE_TAG_END) //传完tag开始传mac（tag，end的应答完再调一次）
    {
        if (mac_total == 0)
        {
            s_tag_macinfo_seq = 0;
            s_flash_consumed = 0;
            s_tag_macinfo_upload_state = UPLOAD_STATE_IDLE;
            s_scan_config.state = SCAN_STATE_IDLE;
            //清空对应的变量
            return;
        }
        s_tag_macinfo_upload_state = UPLOAD_STATE_MAC_START;
        s_tag_macinfo_seq = 0;
        s_flash_consumed = 0;
        // 记下当前实时表有几条，锁定本轮可发数量，发送途中表里新增的不算进本轮
        s_rt_count_snap = s_tran_mac_result_table.count;
        // 开启上报会话：把读取位置退回已确认起点，并暂停落盘，保证实际发送数与START声明的总数一致
        my_flash_store_upload_begin(FS_TYPE_MAC);
        //先发送开始
        snprintf(buf, sizeof(buf), "START,%u", mac_total);
        buf[sizeof(buf) - 1] = '\0';
        #if RETRANSMIT_CHECK_ENABLED
            lte_send_cmd_with_retry("MACINFO", buf);
        #else
            lte_send_command("MACINFO", buf);
        #endif
    }
    else if (s_tag_macinfo_upload_state == UPLOAD_STATE_MAC_START) //应答开始，开始上报mac
    {
        // 优先上报FLASH循环存储中的历史MAC数据
        if (my_flash_store_read_next(FS_TYPE_MAC, &flash_mac) == 1)
        {
            s_flash_consumed++;
            s_tag_macinfo_seq++;
            tran_mac_upload_one(&flash_mac, s_tag_macinfo_seq);
            return;
        }

        // FLASH历史数据读完后，再发实时表里的数据；实时表下标 = 当前总序号 - 已从FLASH读出的条数
        rt_idx = s_tag_macinfo_seq - s_flash_consumed;
        if (rt_idx < s_rt_count_snap)
        {
            mac_item_ptr = &s_tran_mac_result_table.items[rt_idx];
            s_tag_macinfo_seq++;
            tran_mac_upload_one(mac_item_ptr, s_tag_macinfo_seq);
            return;
        }

        // MAC全部发完：先确认这批FLASH数据(把读取位置正式存盘，已发完的扇区腾空)，再清空实时表
        LOG_INF("MAC upload complete, total: %d, flash: %d", s_tag_macinfo_seq, s_flash_consumed);
        my_flash_store_commit(FS_TYPE_MAC);
        s_tag_macinfo_seq = 0;
        s_flash_consumed = 0;
        s_tag_macinfo_upload_state = UPLOAD_STATE_MAC_END;
        #if RETRANSMIT_CHECK_ENABLED
            lte_send_cmd_with_retry("MACINFO", "END");
        #else
            lte_send_command("MACINFO", "END");
        #endif
        // 上报完成后清空结果表
        memset(&s_tran_mac_result_table, 0, sizeof(s_tran_mac_result_table));
        return;
    }
    else //MACINFO结束应答UPLOAD_STATE_MAC_END
    {
        s_tag_macinfo_upload_state = UPLOAD_STATE_IDLE;
        s_tag_macinfo_seq = 0;
        s_flash_consumed = 0;
        s_scan_config.state = SCAN_STATE_IDLE;
    }

}

/********************************************************************
**函数名称:  tran_mac_table_flush_to_flash
**入口参数:  无
**出口参数:  无
**函数功能:  将本轮实时透传MAC表按时间戳从小到大排序后逐条落入FLASH循环存储，
**           落盘后清空实时表；用于周期扫描模式下扫描时长结束但未立即上报时，
**           缓存本轮历史数据
**返 回 值:  无
**注意事项:  仅BLE线程调用；透传MAC实时表恒不溢出，本接口仅在每轮扫描结束时调用
*********************************************************************/
static void tran_mac_table_flush_to_flash(void)
{
    uint8_t i;
    uint8_t j;
    uint8_t min_idx;
    tran_mac_result_item_t tmp;
    int ret;

    // 实时表为空无需落盘
    if (s_tran_mac_result_table.count == 0)
    {
        return;
    }

    // 落盘前按时间戳从小到大排序(选择排序)，确保FLASH里按采集先后顺序存储
    for (i = 0; i < s_tran_mac_result_table.count - 1; i++)
    {
        min_idx = i;
        for (j = i + 1; j < s_tran_mac_result_table.count; j++)
        {
            if (s_tran_mac_result_table.items[j].timestamp < s_tran_mac_result_table.items[min_idx].timestamp)
            {
                min_idx = j;
            }
        }

        if (min_idx != i)
        {
            tmp = s_tran_mac_result_table.items[i];
            s_tran_mac_result_table.items[i] = s_tran_mac_result_table.items[min_idx];
            s_tran_mac_result_table.items[min_idx] = tmp;
        }
    }

    // 逐条把本轮实时表中的MAC记录推入FLASH循环存储暂存缓冲
    for (i = 0; i < s_tran_mac_result_table.count; i++)
    {
        ret = my_flash_store_push_mac(&s_tran_mac_result_table.items[i]);
        if (ret != 0)
        {
            LOG_ERR("flush tran_mac to flash failed at %d (ret %d)", i, ret);
        }
    }

    LOG_INF("flush %d tran_mac records to flash", s_tran_mac_result_table.count);

    // 本轮数据已落盘，清空实时表，下一轮重新扫描收集
    memset(&s_tran_mac_result_table, 0, sizeof(s_tran_mac_result_table));
}

/********************************************************************
**函数名称:  tran_mac_data_handle
**入口参数:  msg_ptr       ---        透传MAC处理消息指针
**出口参数:  无
**函数功能:  处理透传MAC扫描数据，更新或添加到结果表
**返 回 值:  无
*********************************************************************/
static void tran_mac_data_handle(tran_mac_process_msg_t *msg_ptr)
{
    uint8_t i;
    tran_mac_result_item_t *item_ptr;

    if (msg_ptr == NULL || msg_ptr->adv_data_len == 0)
    {
        return;
    }

    // 查找已存在的相同MAC，更新数据（取最新广播包）
    for (i = 0; i < s_tran_mac_result_table.count; i++)
    {
        if (bt_addr_le_cmp(&s_tran_mac_result_table.items[i].addr, &msg_ptr->addr) == 0)
        {
            memcpy(s_tran_mac_result_table.items[i].adv_data, msg_ptr->adv_data, msg_ptr->adv_data_len);
            s_tran_mac_result_table.items[i].adv_data_len = msg_ptr->adv_data_len;
            // 更新采集时间戳为本次扫描到该MAC的时刻
            s_tran_mac_result_table.items[i].timestamp = my_get_system_time_sec();
            return;
        }
    }

    // 新增条目，透传MAC仅扫描指定的TRAN_MAC_MAX_NUM个地址，count恒小于上限，不存在溢出
    if (s_tran_mac_result_table.count < TRAN_MAC_MAX_NUM)
    {
        item_ptr = &s_tran_mac_result_table.items[s_tran_mac_result_table.count];
        bt_addr_le_copy(&item_ptr->addr, &msg_ptr->addr);
        memcpy(item_ptr->adv_data, msg_ptr->adv_data, msg_ptr->adv_data_len);
        item_ptr->adv_data_len = msg_ptr->adv_data_len;
        // 记录采集到该MAC的时刻
        item_ptr->timestamp = my_get_system_time_sec();
        s_tran_mac_result_table.count++;
    }

    MY_LOG_INF("tran_mac found,count: %d", s_tran_mac_result_table.count);
}


/********************************************************************
**函数名称:  scan_trigger_upload
**入口参数:  无
**出口参数:  无
**函数功能:  主动唤醒LTE并上报扫描数据（Mode 3定时上报时调用），在BLE线程中调用
**返 回 值:  无
*********************************************************************/
static void scan_trigger_upload(void)
{
    // 如果正在扫描，立即停止扫描（保证上报的及时性）
    if (s_scan_config.state == SCAN_STATE_SCANNING)
    {
        LOG_INF("scan in progress, stop scan for upload");
        scan_stop_internal();  // 停止扫描
    }

    // 检查是否有数据需要上报
    if (s_result_table.count == 0 &&
        s_tran_mac_result_table.count == 0 &&
        my_flash_store_pending_count(FS_TYPE_TAG) == 0 &&
        my_flash_store_pending_count(FS_TYPE_MAC) == 0)
    {
        LOG_INF("No data to upload");
        s_scan_config.state = SCAN_STATE_IDLE;
        return;
    }

    // 设置LTE开机原因为扫描数据上报
    set_lte_boot_reason(LTE_BOOT_REASON_SCAN);

    LOG_INF("Trigger LTE wakeup and upload %d TAGs", s_result_table.count);
    LOG_INF("Trigger LTE wakeup and upload %d MACs", s_tran_mac_result_table.count);

    // 上报数据（lte_send_command会自动唤醒LTE）
    tag_mac_scan_upload_data();
}

/********************************************************************
**函数名称:  my_scan_init
**入口参数:  无
**出口参数:  无
**函数功能:  初始化扫描模块，注册扫描回调，初始化前缀表
**返 回 值:  0表示成功，负值表示失败
*********************************************************************/
int my_scan_init(void)
{
    int err;

    // 初始化FLASH循环存储模块(须在ZMS挂载完成后)
    err = my_flash_store_init();
    if (err)
    {
        LOG_ERR("flash store init failed (err %d)", err);
        // 存储初始化失败不阻断扫描功能，仅退化为表满丢弃
    }

    // 初始化前缀表
    tag_prefix_table_init();

    // 初始化扫描配置（从全局配置读取）
    s_scan_config.mode = gConfigParam.bt_updata_config.bt_updata_mode;
    s_scan_config.scan_interval = gConfigParam.bt_updata_config.bt_updata_scan_interval;
    s_scan_config.scan_length = gConfigParam.bt_updata_config.bt_updata_scan_length;
    s_scan_config.upload_interval = gConfigParam.bt_updata_config.bt_updata_updata_interval;
    s_scan_config.state = SCAN_STATE_IDLE;

    // 清空结果表
    memset(&s_result_table, 0, sizeof(s_result_table));
    memset(&s_adv_cache_table, 0, sizeof(s_adv_cache_table));
    s_cache_seq_counter = 0;

    // 清空透传MAC结果表
    memset(&s_tran_mac_result_table, 0, sizeof(s_tran_mac_result_table));

    // 注册扫描回调
    s_scan_cb.recv = scan_recv_cb;
    err = bt_le_scan_cb_register(&s_scan_cb);
    if (err)
    {
        LOG_ERR("Failed to register scan callback (err %d)", err);
        return err;
    }

    my_scan_set_config(s_scan_config.mode, s_scan_config.scan_interval,
                       s_scan_config.scan_length, s_scan_config.upload_interval);

    LOG_INF("scan module initialized");
    return 0;
}

/********************************************************************
**函数名称:  my_scan_set_config
**入口参数:  mode        ---        工作模式（0-3）
**           scan_interval ---      扫描间隔（秒）
**           scan_length ---        单次扫描时长（秒）
**           upload_interval ---    上报间隔（秒）
**出口参数:  无
**函数功能:  设置扫描配置参数（直接调用，调用者需在BLE线程中）
**返 回 值:  无
**注意事项:  此函数必须在BLE线程中调用，或确保串行执行安全
*********************************************************************/
void my_scan_set_config(uint8_t mode, uint32_t scan_interval,
                           uint32_t scan_length, uint32_t upload_interval)
{
    // 直接调用内部配置函数（调用者已在BLE线程中，无需消息队列中转）
    scan_set_config_internal(mode, scan_interval, scan_length, upload_interval);
}

/********************************************************************
**函数名称:  my_scan_upload_on_lte_wakeup
**入口参数:  无
**出口参数:  无
**函数功能:  LTE唤醒时处理扫描（告警/工作模式切换时调用）
**           Mode 1：启动扫描
**           Mode 2/3：上报扫描数据
**返 回 值:  无
**注意事项:  此函数在LTE已唤醒或正在唤醒时调用，不主动唤醒LTE
*********************************************************************/
void my_scan_upload_on_lte_wakeup(void)
{
    // 发送消息到BLE线程处理
    my_send_msg(MOD_BLE, MOD_BLE, MY_MSG_UPLOAD_WAKEUP);
}

/********************************************************************
**函数名称:  my_tag_prefix_add
**入口参数:  prefix    ---        要添加的前缀字符串
**出口参数:  无
**函数功能:  添加过滤前缀到前缀表
**返 回 值:  0表示成功，负值表示失败
*********************************************************************/
int my_tag_prefix_add(const char *prefix)
{
    uint8_t index;
    uint8_t len;

    // 检查前缀表是否已满
    if (s_prefix_table.count >= TAG_PREFIX_MAX_NUM)
    {
        LOG_ERR("Prefix table full");
        return -ENOMEM;
    }

    // 获取前缀长度并校验
    len = strlen(prefix);
    if (len == 0 || len >= TAG_PREFIX_MAX_LEN)
    {
        LOG_ERR("Invalid prefix length");
        return -EINVAL;
    }

    // 添加前缀到表中
    index = s_prefix_table.count;
    memcpy(s_prefix_table.items[index].prefix, prefix, len);
    s_prefix_table.items[index].len = len;
    s_prefix_table.items[index].valid = true;
    s_prefix_table.count++;

    LOG_INF("Prefix added: %s", prefix);
    return 0;
}

/********************************************************************
**函数名称:  my_scan_msg_handler
**入口参数:  msg           ---        消息结构体指针
**出口参数:  无
**函数功能:  扫描消息处理函数，在BLE线程中调用
**返 回 值:  无
**注意事项:  此函数必须在BLE线程的消息处理循环中调用
*********************************************************************/
void my_scan_msg_handler(msg_t *msg)
{
    tag_scan_process_msg_t process_msg;
    tran_mac_process_msg_t tmac_msg;
    uint16_t process_count;

    if (msg == NULL)
    {
        return;
    }

    switch (msg->msgID)
    {
        case MY_MSG_TAG_SCAN_PROCESS:
            // 从内部消息队列取出扫描数据，完成过滤、缓存和结果保存
            process_count = 0;
            while (k_msgq_get(&s_tag_scan_process_msgq, &process_msg, K_NO_WAIT) == 0)
            {
                tag_scan_data_handle(&process_msg);
                process_count++;
            }
            if (process_count > 1)
            {
                LOG_INF("TAG scan process drain count: %d", process_count);
            }
            break;

        case MY_MSG_TRAN_MAC_PROCESS:
            // 从内部消息队列取出透传MAC数据，更新到结果表
            process_count = 0;
            while (k_msgq_get(&s_tran_mac_process_msgq, &tmac_msg, K_NO_WAIT) == 0)
            {
                tran_mac_data_handle(&tmac_msg);
                process_count++;
            }
            if (process_count > 1)
            {
                LOG_INF("TRAN_MAC process drain count: %d", process_count);
            }
            break;

        case MY_MSG_SCAN_INTERVAL:
            // 周期扫描定时器消息
            if ((s_scan_config.mode == SCAN_MODE_PERIOD_CACHE ||
                 s_scan_config.mode == SCAN_MODE_PERIOD_UPLOAD) &&
                s_scan_config.state == SCAN_STATE_IDLE)
            {
                scan_start_internal();
            }
            break;

        case MY_MSG_SCAN_LENGTH:
            // 单次扫描时长定时器消息
            scan_stop_internal();
            // Mode 1/2：周期扫描结束后不立即上报，将本轮TAG与透传MAC落入FLASH循环存储缓存
            tag_table_flush_to_flash();
            tran_mac_table_flush_to_flash();
            break;

        case MY_MSG_SCAN_UPLOAD:
            // 上报间隔定时器消息（Mode 2 定时主动唤醒LTE并上报）
            // 透传MAC每轮扫描结束就已存入FLASH、实时表已清空，所以这里要一并检查FLASH里的待上报数
            if (s_scan_config.mode == SCAN_MODE_PERIOD_UPLOAD &&
                (s_result_table.count > 0 || s_tran_mac_result_table.count > 0 ||
                 my_flash_store_pending_count(FS_TYPE_TAG) > 0 ||
                 my_flash_store_pending_count(FS_TYPE_MAC) > 0))
            {
                // Mode 2：主动唤醒LTE并上报数据
                //上报过程中来报警或者切换工作模式，跳过
                if (s_scan_config.state == SCAN_STATE_WAITING_UPLOAD)
                {
                    break;
                }
                scan_trigger_upload();
            }
            break;

        case MY_MSG_UPLOAD_WAKEUP:
            // LTE唤醒时处理扫描数据（告警/工作模式切换时调用）
            switch (s_scan_config.mode)
            {
                case SCAN_MODE_PERIOD_CACHE:
                case SCAN_MODE_PERIOD_UPLOAD:
                    // Mode 1/2：立即停止扫描并上报数据
                    // 透传MAC每轮扫描结束就已存入FLASH、实时表已清空，所以这里要一并检查FLASH里的待上报数
                    if (s_result_table.count > 0 || s_tran_mac_result_table.count > 0 ||
                        my_flash_store_pending_count(FS_TYPE_TAG) > 0 ||
                        my_flash_store_pending_count(FS_TYPE_MAC) > 0)
                    {
                        if (s_scan_config.state == SCAN_STATE_SCANNING)
                        {
                            LOG_INF("Mode %d LTE wakeup: stop scan and upload %d TAGs, %d TRAN_MACs",
                                    s_scan_config.mode, s_result_table.count, s_tran_mac_result_table.count);
                            scan_stop_internal();
                        }
                        //上报过程中来报警或者切换工作模式，跳过
                        if (s_scan_config.state == SCAN_STATE_WAITING_UPLOAD)
                        {
                            break;
                        }
                        tag_mac_scan_upload_data();
                    }
                    break;

                default:
                    break;
            }
            break;

        default:
            break;
    }
}

/********************************************************************
**函数名称:  my_tran_mac_check
**入口参数:  addr          ---        待检查的蓝牙地址指针
**出口参数:  无
**函数功能:  检查指定蓝牙地址是否在透传MAC地址列表中
**返 回 值:  true表示匹配成功，false表示不匹配
*********************************************************************/
bool my_tran_mac_check(const bt_addr_le_t *addr)
{
    uint8_t i;

    for (i = 0; i < gConfigParam.bparmac_config.bt_parmac_mac_count; i++)
    {
        if (bt_addr_le_cmp(&gConfigParam.bparmac_config.bt_parmac_macs[i], addr) == 0)
        {
            return true;
        }
    }
    return false;
}

/********************************************************************
**函数名称:  my_tran_mac_add
**入口参数:  addr          ---        要添加的蓝牙地址指针
**出口参数:  无
**函数功能:  添加透传MAC地址到列表
**返 回 值:  0表示成功，-EEXIST表示已存在,-ENOMEM表示没有空间
*********************************************************************/
int my_tran_mac_add(const bt_addr_le_t *addr)
{
    if (gConfigParam.bparmac_config.bt_parmac_mac_count >= TRAN_MAC_MAX_NUM)
    {
        return -ENOMEM;
    }

    if (my_tran_mac_check(addr))
    {
        return -EEXIST;
    }

    bt_addr_le_copy(&gConfigParam.bparmac_config.bt_parmac_macs[gConfigParam.bparmac_config.bt_parmac_mac_count], addr);
    gConfigParam.bparmac_config.bt_parmac_mac_count++;

    return 0;
}

/********************************************************************
**函数名称:  my_tran_mac_del
**入口参数:  addr          ---        要删除的蓝牙地址指针
**出口参数:  无
**函数功能:  从列表中删除指定透传MAC地址
**返 回 值:  0表示成功，-ENOENT表示未找到
*********************************************************************/
int my_tran_mac_del(const bt_addr_le_t *addr)
{
    uint8_t i, j;

    for (i = 0; i < gConfigParam.bparmac_config.bt_parmac_mac_count; i++)
    {
        if (bt_addr_le_cmp(&gConfigParam.bparmac_config.bt_parmac_macs[i], addr) == 0)
        {
            // 将后续元素前移
            for (j = i; j < gConfigParam.bparmac_config.bt_parmac_mac_count - 1; j++)
            {
                memcpy(&gConfigParam.bparmac_config.bt_parmac_macs[j],
                       &gConfigParam.bparmac_config.bt_parmac_macs[j + 1],
                       sizeof(bt_addr_le_t));
            }
            // 清空最后一个元素
            memset(&gConfigParam.bparmac_config.bt_parmac_macs[gConfigParam.bparmac_config.bt_parmac_mac_count - 1],
                   0, sizeof(bt_addr_le_t));
            gConfigParam.bparmac_config.bt_parmac_mac_count--;
            return 0;
        }
    }

    return -ENOENT;
}

/********************************************************************
**函数名称:  my_tran_mac_del_all
**入口参数:  无
**出口参数:  无
**函数功能:  清空所有透传MAC地址
**返 回 值:  无
*********************************************************************/
void my_tran_mac_del_all(void)
{
    memset(gConfigParam.bparmac_config.bt_parmac_macs, 0, sizeof(gConfigParam.bparmac_config.bt_parmac_macs));
    gConfigParam.bparmac_config.bt_parmac_mac_count = 0;
}

#if FS_STORE_TEST_ENABLE
/********************************************************************
**函数名称:  my_scan_test_flush_sort
**入口参数:  type      ---        数据类型(0=TAG, 1=透传MAC)（输入）
**           count     ---        注入的测试记录条数（输入）
**出口参数:  无
**函数功能:  测试专用：清空对应实时表后注入count条乱序时间戳记录，再调用真实的
**           落盘接口(tag_table_flush_to_flash/tran_mac_table_flush_to_flash)，
**           验证落盘前是否按时间戳从小到大排序。注入时令序号i的记录时间戳=count-i，
**           即序号越小时间戳越大(序号与时间戳反序)，排序正确则回读到的序号应为降序
**返 回 值:  实际注入并落盘的记录条数，负值表示参数非法
**注意事项:  仅FS_STORE_TEST_ENABLE开启时编译；仅供shell测试调用，序号小端编码进
**           MAC地址前4字节，与shell侧解码规则一致
*********************************************************************/
int my_scan_test_flush_sort(uint8_t type, uint16_t count)
{
    uint16_t i;

    if (type == FS_TYPE_TAG)
    {
        // 注入条数不超过实时表容量
        if (count > TAG_RESULT_MAX_NUM)
        {
            count = TAG_RESULT_MAX_NUM;
        }

        memset(&s_result_table, 0, sizeof(s_result_table));

        // 填入序号与时间戳反序的记录，制造乱序待排序数据
        for (i = 0; i < count; i++)
        {
            // 序号i小端编码进MAC地址前4字节
            s_result_table.items[i].addr.a.val[0] = (uint8_t)(i & 0xFF);
            s_result_table.items[i].addr.a.val[1] = (uint8_t)((i >> 8) & 0xFF);
            s_result_table.items[i].addr.a.val[2] = 0;
            s_result_table.items[i].addr.a.val[3] = 0;
            s_result_table.items[i].addr.a.val[4] = 0xAA;
            s_result_table.items[i].addr.a.val[5] = 0x55;
            // 序号越小时间戳越大，排序后顺序应被颠倒
            s_result_table.items[i].timestamp = (uint32_t)(count - i);
            snprintf(s_result_table.items[i].name, ADV_NAME_MAX_LEN, "TAG%u", i);
        }
        s_result_table.count = (uint8_t)count;

        // 调用真实落盘接口(内部排序+落盘+清空实时表)
        tag_table_flush_to_flash();

        return count;
    }
    else if (type == FS_TYPE_MAC)
    {
        if (count > TRAN_MAC_MAX_NUM)
        {
            count = TRAN_MAC_MAX_NUM;
        }

        memset(&s_tran_mac_result_table, 0, sizeof(s_tran_mac_result_table));

        for (i = 0; i < count; i++)
        {
            // 序号i小端编码进MAC地址前4字节
            s_tran_mac_result_table.items[i].addr.a.val[0] = (uint8_t)(i & 0xFF);
            s_tran_mac_result_table.items[i].addr.a.val[1] = (uint8_t)((i >> 8) & 0xFF);
            s_tran_mac_result_table.items[i].addr.a.val[2] = 0;
            s_tran_mac_result_table.items[i].addr.a.val[3] = 0;
            s_tran_mac_result_table.items[i].addr.a.val[4] = 0xBB;
            s_tran_mac_result_table.items[i].addr.a.val[5] = 0x66;
            // 序号越小时间戳越大，排序后顺序应被颠倒
            s_tran_mac_result_table.items[i].timestamp = (uint32_t)(count - i);
            s_tran_mac_result_table.items[i].adv_data_len = 2;
            s_tran_mac_result_table.items[i].adv_data[0] = (uint8_t)(i & 0xFF);
            s_tran_mac_result_table.items[i].adv_data[1] = (uint8_t)((i >> 8) & 0xFF);
        }
        s_tran_mac_result_table.count = (uint8_t)count;

        // 调用真实落盘接口(内部排序+落盘+清空实时表)
        tran_mac_table_flush_to_flash();

        return count;
    }

    return -EINVAL;
}
#endif /* FS_STORE_TEST_ENABLE */
