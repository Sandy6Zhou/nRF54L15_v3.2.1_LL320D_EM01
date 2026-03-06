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
#define BLE_DATA_TYPE_FILE_TRANS                0x4605          //文件指令
#define BLE_DATA_TYPE_QEURY_VER                 0x5601          //app下行查询版本号
#define BLE_DATA_TYPE_RSP_VER1                  0x5602          //设备类型
#define BLE_DATA_TYPE_RSP_VER2                  0x5603          //设备版本号

#define BLE_DATA_TYPE_SET_ADV                   0x5606          //设置广播名称
#define BLE_DATA_TYPE_SET_IMEI                  0x5707          //设置IMEI号
#define BLE_DATA_TYPE_AT_CMD                    0x5808          //用户指令

#define BLE_COMU_DEV_CODE                       0xB3
#define BLE_COMU_APP_CODE                       0xC2
#define BLE_COMU_CMD_START                      0x89
#define BLE_COMU_ALARM_START                    0x78

#define BLE_PKEY_RX_CMD                         0x8900C200
#define BLE_PKEY_RSP_DATA1                      0x00
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

#define BLE_SERVER_MAX_MTU         517    // MTU最大长度，系统用掉了3个字节,可用数据最大514字节
#define BLE_SERVER_MTU_DFT         23     // MTU默认长度

#define BLE_SERVER_MAX_DATA_LEN    (ble_server_mtu - 3)
#define BLE_SVC_RX_MAX_LEN         (BLE_SERVER_MAX_MTU - 3)
#define BLE_SVC_TX_MAX_LEN         (BLE_SERVER_MAX_MTU - 3)

#define BLE_RESP_LENGTH_MAX         BLE_SVC_RX_MAX_LEN

#define BLE_FRAME_HEAD_LEN          2               // 蓝牙数据帧头长度2字节
#define BLE_CMD_HEAD_LEN            2               // 蓝牙命令头长度2字节
#define BLE_CMD_DATA_LEN_UNIT       16              // 蓝牙命令内容长度基数16字节，因为要加密，必须是16字节的倍数
#define BLE_GATT_FRAME_LEN_MIN      (BLE_FRAME_HEAD_LEN + BLE_CMD_HEAD_LEN + BLE_CMD_DATA_LEN_UNIT)

#define BLE_MTU_DATA_LEN            (256 + 16)

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

#endif

