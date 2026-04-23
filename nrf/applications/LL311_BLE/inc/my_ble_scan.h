/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        my_ble_scan.h
**文件描述:        设备扫描模块头文件
**当前版本:        V1.0
**作    者:       周森达 (zhousenda@jimiiot.com)
**完成日期:        2026.04.14
*********************************************************************
** 功能描述:        设备扫描模块接口定义
**                 1. 定义扫描结果/配置/缓存等核心数据结构
**                 2. 定义四种工作模式：关闭/唤醒扫描/周期缓存/周期上报
**                 3. 提供初始化、配置设置、前缀添加等对外接口
**                 4. 提供LTE唤醒时的扫描触发和数据上报接口
**                 5. 提供BLE线程消息处理接口，支持无锁串行处理
*********************************************************************/

#ifndef _MY_BLE_SCAN_H_
#define _MY_BLE_SCAN_H_

/* 宏定义 */
#define TAG_PREFIX_MAX_NUM          10      // 最大支持TAG前缀数量
#define TAG_PREFIX_MAX_LEN          16      // 前缀最大长度
#define ADV_NAME_MAX_LEN            32      // 设备名称最大长度
#define TAG_FF_DATA_MAX_LEN         29      // FF数据固定长度
#define UUID_MAX_LEN                16      // UUID最大存储长度
#define ADV_CACHE_MAX_NUM           10      // ADV数据缓存最大数量
#define TAG_RESULT_MAX_NUM          50      // 最大存储结果数量,后续根据实际情况调整
#define TRAN_MAC_ADV_DATA_MAX_LEN   31      // 蓝牙广播数据最大长度（Legacy ADV）
#define TRAN_MAC_MAX_NUM            20      // 透传MAC地址最大数量

/* 扫描工作模式枚举 */
typedef enum
{
    SCAN_MODE_OFF = 0,          // Mode 0：关闭蓝牙扫描收集功能
    SCAN_MODE_WAKEUP_SCAN,      // Mode 1：仅在4G唤醒时扫描并上传
    SCAN_MODE_PERIOD_CACHE,     // Mode 2：周期扫描收集，仅4G唤醒时上传
    SCAN_MODE_PERIOD_UPLOAD,    // Mode 3：周期扫描收集，定时主动唤醒上传
} scan_mode_t;

/* 扫描状态枚举 */
typedef enum
{
    SCAN_STATE_IDLE = 0,        // 空闲状态
    SCAN_STATE_SCANNING,        // 扫描中
    SCAN_STATE_WAITING_UPLOAD,  // 等待上报
} scan_state_t;

/* TAG设备名称前缀结构 */
typedef struct
{
    char prefix[TAG_PREFIX_MAX_LEN];    // 前缀字符串
    uint8_t len;                        // 前缀长度
    bool valid;                         // 有效标志
} tag_prefix_t;

/* TAG设备名称前缀表 */
typedef struct
{
    tag_prefix_t items[TAG_PREFIX_MAX_NUM];
    uint8_t count;                      // 当前有效前缀数量
} tag_prefix_table_t;

/* TAG设备扫描结果结构 */
typedef struct
{
    bt_addr_le_t addr;                          // 设备MAC地址
    char name[ADV_NAME_MAX_LEN];                // 设备名称
    uint8_t ff_data[TAG_FF_DATA_MAX_LEN];       // FF数据
    uint8_t ff_data_len;                        // FF数据长度
    uint8_t uuid[UUID_MAX_LEN];                 // UUID数据
    uint8_t uuid_len;                           // UUID数据长度
    uint8_t battery_percent;                    // 电量百分比
    int8_t rssi;                                // 信号强度
} tag_scan_result_t;

/* TAG扫描结果表 */
typedef struct
{
    tag_scan_result_t items[TAG_RESULT_MAX_NUM];
    uint8_t count;                       // 当前结果数量
} tag_scan_result_table_t;

/* 扫描模块配置结构 */
typedef struct
{
    scan_mode_t mode;                 // 工作模式
    uint32_t scan_interval;           // 扫描间隔（秒）
    uint32_t scan_length;             // 单次扫描时长（秒）
    uint32_t upload_interval;         // 上报间隔（秒）
    scan_state_t state;               // 当前状态
} scan_config_t;

/* ADV数据缓存项结构 */
typedef struct
{
    bt_addr_le_t addr;
    uint8_t ff_data[TAG_FF_DATA_MAX_LEN];
    uint8_t ff_data_len;
    bool valid;
    uint32_t seq_num;     // 序号，越大表示越新
} adv_cache_item_t;

/* TAG扫描处理消息结构 */
typedef struct
{
    uint8_t adv_type;
    tag_scan_result_t result;
} tag_scan_process_msg_t;

/* 透传MAC扫描结果项 */
typedef struct
{
    bt_addr_le_t addr;
    uint8_t adv_data[TRAN_MAC_ADV_DATA_MAX_LEN];
    uint8_t adv_data_len;
} tran_mac_result_item_t;

/* 透传MAC扫描结果表 */
typedef struct
{
    tran_mac_result_item_t items[TRAN_MAC_MAX_NUM];
    uint8_t count;
} tran_mac_result_table_t;

/* 透传MAC处理消息结构 */
typedef struct
{
    bt_addr_le_t addr;
    uint8_t adv_data[TRAN_MAC_ADV_DATA_MAX_LEN];
    uint8_t adv_data_len;
} tran_mac_process_msg_t;

/********************************************************************
**函数名称:  my_scan_init
**入口参数:  无
**出口参数:  无
**函数功能:  初始化扫描模块，注册扫描回调，初始化前缀表
**返 回 值:  0表示成功，负值表示失败
*********************************************************************/
int my_scan_init(void);

/********************************************************************
**函数名称:  my_scan_set_config
**入口参数:  mode        ---        工作模式（0-3）
**           scan_interval ---      扫描间隔（秒）
**           scan_length ---        单次扫描时长（秒）
**           upload_interval ---    上报间隔（秒）
**出口参数:  无
**函数功能:  设置扫描配置参数
**返 回 值:  无
*********************************************************************/
void my_scan_set_config(uint8_t mode, uint32_t scan_interval,
                           uint32_t scan_length, uint32_t upload_interval);

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
void my_scan_upload_on_lte_wakeup(void);

/********************************************************************
**函数名称:  my_tag_prefix_add
**入口参数:  prefix    ---        要添加的前缀字符串
**出口参数:  无
**函数功能:  添加过滤前缀到前缀表
**返 回 值:  0表示成功，负值表示失败
*********************************************************************/
int my_tag_prefix_add(const char *prefix);

/********************************************************************
**函数名称:  my_scan_msg_handler
**入口参数:  msg_ptr           ---        消息结构体指针
**出口参数:  无
**函数功能:  扫描消息处理函数，在BLE线程中调用
**返 回 值:  无
**注意事项:  此函数必须在BLE线程的消息处理循环中调用
*********************************************************************/
void my_scan_msg_handler(MSG_S *msg_ptr);

/********************************************************************
**函数名称:  my_tran_mac_check
**入口参数:  addr          ---        待检查的蓝牙地址指针
**出口参数:  无
**函数功能:  检查指定蓝牙地址是否在透传MAC地址列表中
**返 回 值:  true表示匹配成功，false表示不匹配
*********************************************************************/
bool my_tran_mac_check(const bt_addr_le_t *addr);

/********************************************************************
**函数名称:  my_tran_mac_add
**入口参数:  addr          ---        要添加的蓝牙地址指针
**出口参数:  无
**函数功能:  添加透传MAC地址到列表
**返 回 值:  0表示成功，-EEXIST表示已存在,-ENOMEM表示没有空间
*********************************************************************/
int my_tran_mac_add(const bt_addr_le_t *addr);

/********************************************************************
**函数名称:  my_tran_mac_del
**入口参数:  addr          ---        要删除的蓝牙地址指针
**出口参数:  无
**函数功能:  从列表中删除指定透传MAC地址
**返 回 值:  0表示成功，-ENOENT表示未找到
*********************************************************************/
int my_tran_mac_del(const bt_addr_le_t *addr);

/********************************************************************
**函数名称:  my_tran_mac_del_all
**入口参数:  无
**出口参数:  无
**函数功能:  清空所有透传MAC地址
**返 回 值:  无
*********************************************************************/
void my_tran_mac_del_all(void);

/********************************************************************
**函数名称:  clear_tag_macinfo
**入口参数:  无
**出口参数:  无
**函数功能:  重置扫描与上传相关的状态机、序列号，并清空TAG及MAC地址缓存表
**返 回 值:  无
*********************************************************************/
void clear_tag_macinfo(void);

/********************************************************************
**函数名称:  tag_mac_scan_upload_data
**入口参数:  无
**出口参数:  无
**函数功能:  状态机调度函数，负责按（START->DATA->DATA->...->END）分步上传
**             TAG数据和透传MAC地址数据。该函数需配合LTE模块的应答机制
**             多次调用以推进状态流转。
**返 回 值:  无
*********************************************************************/
void tag_mac_scan_upload_data(void);

#endif /* _MY_BLE_SCAN_H_ */
