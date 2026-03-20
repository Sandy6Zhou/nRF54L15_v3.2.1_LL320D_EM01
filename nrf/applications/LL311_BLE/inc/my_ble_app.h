/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        my_ble_app.h
**文件描述:        设备蓝牙交互模块实现头文件
**当前版本:        V1.0
**作    者:        周森达 (zhousenda@jimiiot.com)
**完成日期:        2026-03-05
*********************************************************************
** 功能描述:        1. 提供交互协议相关宏定义
**                 2. 为外部提供数据接收和解析相关接口
** 协议文档:        Jimi Iot 蓝牙通信协议V3.1.6_2026-3-5
** OTA 效率优化:    为提升 OTA 传输效率，采用双 PDU 模式，MTU 扩大至 498 字节
**                 （默认 MTU 247 仅支持单 PDU，498 支持双 PDU，理论吞吐提升约 2 倍）
** 配置位置:        prj.conf 中 CONFIG_BT_L2CAP_TX_MTU=498 和 CONFIG_BT_BUF_ACL_RX_SIZE=502
*********************************************************************/
#ifndef _MY_BLE_APP_H_
#define _MY_BLE_APP_H_

// 交互协议相关宏定义
#define BLE_DATA_FRAME_LEN_MAX                  16

#define BLE_DATA_PACKET_HEAD                    0X4254          //数据包头
#define BLE_DATA_TYPE_PKEY                      0x1101          //公钥数据
#define BLE_DATA_TYPE_CID                       0x2202          //串号数据
#define BLE_DATA_TYPE_MTU_LEN                   0x2203          //MTU交换数据
#define BLE_DATA_TYPE_HID_VALID                 0x2303          //HID有效期
#define BLE_DATA_TYPE_ALARM                     0x3303          //防盗器指令数据
#define BLE_DATA_TYPE_QUERY                     0x3304          //查询仪表数据及应答
#define BLE_DATA_TYPE_REPORT                    0x3305          //仪表数据上行汇报及应答
#define BLE_DATA_TYPE_CMD                       0x5505          //其它指令数据
#define BLE_DATA_TYPE_FILE_TRANS                0x4605          //DFU文件指令,参考Jimi Iot 蓝牙通信协议V3.1.6_2026-3-5 6.4 OTA文件传输
#define BLE_DATA_TYPE_QEURY_VER                 0x5601          //app下行查询版本号
#define BLE_DATA_TYPE_RSP_VER1                  0x5602          //设备类型
#define BLE_DATA_TYPE_RSP_VER2                  0x5603          //设备版本号

#define BLE_DATA_TYPE_SET_ADV                   0x5606          //设置广播名称
#define BLE_DATA_TYPE_SET_IMEI                  0x5707          //设置IMEI号
#define BLE_DATA_TYPE_AT_CMD                    0x5808          //用户指令

/* 日志指令透传数据（LOG_DATA）
 * 指令格式：
 * +--------+--------+--------+--------+
 * |  字节  |   0~1  |   2~3  |  4~N   |
 * +--------+--------+--------+--------+
 * |  字段  |  head  |  cmd   |  data  |
 * +--------+--------+--------+--------+
 * |  值    | 0x4254 | 0x5901 |  日志  |
 * +--------+--------+--------+--------+
 * 说明：
 *   - head (2B): 帧头 0x4254 ("BT")
 *   - cmd  (2B): 指令类型 0x5901 (LOG_DATA)
 *   - data (NB): 透传日志内容，明文传输，无需16字节对齐，测试发现小于16字节的需要填充0，补足16字节
 *   - 长度计算: MTU - BLE_LOG_MTU_OVERHEAD
 *   - MTU开销: ATT头(3B) + head(2B) + cmd(2B) = 7B
 */
#define BLE_DATA_TYPE_BT_LOG                    0x5901          //日志指令透传数据（LOG_DATA）
/* 日志数据MTU开销：ATT头(3B) + head(2B) + cmd(2B) */
#define BLE_LOG_MTU_OVERHEAD                    7

/* 蓝牙日志单包最大内容长度
 * 原因：测试发现超过240字节会导致蓝牙自动断开
 * 注意：此限制与MTU无关，是协议栈稳定性要求
 */
#define BLE_LOG_MAX_CHUNK_SIZE                  240

#define BLE_DFU_HEAD1                           0x78            // DFU包头1
#define BLE_DFU_HEAD2                           0x79            // DFU包头2

#define BLE_COMU_DEV_CODE                       0xB3            // 设备码
#define BLE_COMU_APP_CODE                       0xC2            // 应用码
#define BLE_COMU_CMD_START                      0x89            // 命令码
#define BLE_COMU_ALARM_START                    0x78            // 告警码

#define BLE_PKEY_RX_CMD                         0x8900C200      // 公钥接收命令
#define BLE_PKEY_RSP_DATA1                      0x00            // 公钥接收响应数据1
#define BLE_PKEY_RSP_DATA3                      0x00 

/* APP下行控制指令    */
#define BLE_APP_CMD_APP_LINK                    0x00
#define BLE_APP_CMD_PAIR_ALLOW                  0x02
#define BLE_APP_CMD_PAIR_ALIOS                  0x03            // IOS PAIR ALLOW
#define BLE_APP_CMD_HID_READ                    0x05
#define BLE_APP_CMD_HID_ONOFF                   0x06
#define BLE_APP_CMD_HID_GEAR                    0x08
#define BLE_APP_CMD_QUERY_SENSOR                0x09
#define BLE_APP_CMD_SETUP_SENSOR                0x0a
#define BLE_APP_CMD_QUERY_BUZZER                0x0b
#define BLE_APP_CMD_SETUP_BUZZER                0x0c
#define BLE_APP_CMD_QUERY_CTRLPWR               0x0d
#define BLE_APP_CMD_SETUP_CTRLPWR               0x0e
#define BLE_APP_CMD_QUERY_ALL                   0x0f
#define BLE_APP_CMD_FIND_CAR                    0x10
#define BLE_APP_CMD_USER_MODE                   0x12
#define BLE_APP_CMD_PAIR_MODE                   0x15
#define BLE_APP_CMD_PAIR_CODE                   0x16

/* 设备上行应答APP控制指令       */
#define BLE_RSP_CMD_APP_LINK                    0x00
#define BLE_RSP_CMD_CID                         0x01
#define BLE_RSP_CMD_PAIR_ALLOW                  0x03
#define BLE_RSP_CMD_PAIR_RESULT                 0x04
#define BLE_RSP_CMD_HID_ONOFF                   0x06
#define BLE_RSP_CMD_HID_GEAR                    0x08
#define BLE_RSP_CMD_SENSOR_PARAM                0x09
#define BLE_RSP_CMD_SENSOR_SETUP                0x0a
#define BLE_RSP_CMD_BUZZER_PARAM                0x0b
#define BLE_RSP_CMD_BUZZER_SETUP                0x0c
#define BLE_RSP_CMD_CTRLPWR_PARAM               0x0d
#define BLE_RSP_CMD_CTRLPWR_SETUP               0x0e
#define BLE_RSP_CMD_QUERY_ALL                   0x0f
#define BLE_RSP_CMD_FIND_CAR                    0x10
#define BLE_RSP_CMD_USER_MODE                   0x12
#define BLE_RSP_CMD_PAIR_MODE                   0x15
#define BLE_RSP_CMD_PAIR_CODE                   0x16

#define BLE_RSP_PARAM_SUCCESS                   0x00
#define BLE_RSP_PARAM_FAIL                      0x01

extern uint16_t ble_server_mtu;

#define BLE_SERVER_MAX_MTU CONFIG_BT_L2CAP_TX_MTU // MTU最大长度，系统用掉了3个字节,可用数据最大244字节
#define BLE_SERVER_MIN_MTU 23                     // MTU默认最小长度

#define BLE_SERVER_MAX_DATA_LEN    (ble_server_mtu - 3)
#define BLE_SVC_RX_MAX_LEN         (BLE_SERVER_MAX_MTU - 3)
#define BLE_SVC_TX_MAX_LEN         (BLE_SERVER_MAX_MTU - 3)

#define BLE_RESP_LENGTH_MAX         BLE_SVC_RX_MAX_LEN

#define BLE_FRAME_HEAD_LEN          2               // 蓝牙数据帧头长度2字节
#define BLE_CMD_HEAD_LEN            2               // 蓝牙命令头长度2字节
#define BLE_CMD_DATA_LEN_UNIT       16              // 蓝牙命令内容长度基数16字节，因为要加密，必须是16字节的倍数
#define BLE_GATT_FRAME_LEN_MIN      (BLE_FRAME_HEAD_LEN + BLE_CMD_HEAD_LEN + BLE_CMD_DATA_LEN_UNIT)

#define BLE_MTU_DATA_LEN            (512 + 16)  /* MTU 498 支持，最大解密后约 512 字节，对齐 16 字节 */

/*********************************************************************
**函数名称:  BLE_DataInputBuffer
**入口参数:  data        --  蓝牙接收的原始数据指针
**           len         --  蓝牙接收的原始数据长度
**出口参数:  无
**函数功能:  验证数据长度合法性后缓存数据，发送消息给蓝牙线程处理
**返 回 值:  无
*********************************************************************/
void BLE_DataInputBuffer(const uint8_t *data, uint16_t len);
/*********************************************************************
**函数名称:  ble_rx_proc_handle
**入口参数:  无
**出口参数:  无
**函数功能:  蓝牙接收消息处理函数，验证包头后分发处理数据包
**返 回 值:  无
*********************************************************************/
void ble_rx_proc_handle(void);

/*********************************************************************
**函数名称:  my_ble_dfu_send_response
**入口参数:  cmd   ---   命令码
**           data  ---   响应数据
**           len   ---   数据长度
**出口参数:  无
**函数功能:  发送 DFU 响应数据
**返 回 值:  无
*********************************************************************/
void my_ble_dfu_send_response(uint8_t cmd, uint8_t *data, uint16_t len);

/*********************************************************************
**函数名称:  ble_log_send
**入口参数:  data     ---        日志数据指针
**           len      ---        日志数据长度
**出口参数:  无
**函数功能:  发送蓝牙日志到APP，通过指令通道（0x5901）明文传输
**返 回 值:  无
*********************************************************************/
void ble_log_send(uint8_t *data, uint8_t len);

/*********************************************************************
**函数名称:  ble_log_init
**入口参数:  无
**出口参数:  无
**函数功能:  初始化蓝牙日志模块，包括互斥锁初始化
**返 回 值:  0 表示成功
*********************************************************************/
int ble_log_init(void);

/*********************************************************************
**函数名称:  ble_log_disconnect_cleanup
**入口参数:  无
**出口参数:  无
**函数功能:  蓝牙断开时设置断开标志，通知 ble_log_send 清理资源
**返 回 值:  无
*********************************************************************/
void ble_log_disconnect_cleanup(void);

/*********************************************************************
**函数名称:  ble_log_connect_init
**入口参数:  无
**出口参数:  无
**函数功能:  蓝牙连接建立时清除断开标志，允许日志发送
**返 回 值:  无
*********************************************************************/
void ble_log_connect_init(void);

#endif

