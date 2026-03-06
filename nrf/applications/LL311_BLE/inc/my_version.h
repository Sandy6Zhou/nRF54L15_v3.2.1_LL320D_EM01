/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        my_verson.h
**文件描述:        版本信息头文件
**当前版本:        V1.0
**作    者:        Harrison Wu (wuyujiao@jimiiot.com)
**完成日期:        2026.01.15
*********************************************************************
** 功能描述:        记录版本信息
*********************************************************************/

#ifndef _MY_VERSION_H_
#define _MY_VERSION_H_

#define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260305"
/* 软件版本:        V1.0
** 完成日期:        2026.03.05
** 作    者:        周森达 (zhousenda@jimiiot.com)
** 修改内容:        1. 增加BLE与APP鉴权连接与加解密功能
*/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_0303"
/* 软件版本:        V1.0
** 完成日期:        2026.02.28
** 作    者:        Harrison Wu (wuyujiao@jimiiot.com)
** 修改内容:        1. 完成NFC读卡功能并对NFC功能进行优化 - 修复 fm175xx_driver.c 中 FIFO 连续读时序（使用 i2c_write_read），实现动态轮询时长配置，添加卡片事件上报到主任务并调整日志输出；
**                 2. 目前支持的卡片类型为 TYPE-A (Mifare Classic 和 NTag) 卡片的UUID读取；
**                 3. FUN_KEY 按键功能实现 - 在 my_ctrl.c 中实现短按/长按检测（下降沿中断 + 50ms 轮询定时器），短按启动 NFC 轮询，长按发送事件到主任务；
**                 4. 设备树配置更新 - nrf54l15dk_nrf54l15_cpuapp.overlay 中修改 FUN_KEY 为 gpio-keys 兼容类型，配置内部上拉和下降沿触发；
**                 5. 部分代码规范化 - 统一所有文件编码风格（大括号换行），为多个函数添加标准格式注释（含功能描述）；
**                 6. 消息处理扩展 - main.c 中增加按键短按/长按事件处理，短按触发 NFC 轮询启动，长按发送事件到主任务；
**                 7. 头文件同步更新 - my_ctrl.h、my_nfc.h、nfc_api.h 等头文件补充新接口声明和详细注释；
**
**                 注：NFC卡目前只支持TYPEA协议的读（包含Mifare Classic 和 NTag）卡片的UUID读取
**                 FM17550模块事实上可以支持两种卡的读写的包含TYPEA和TYPEB协议的卡片，但是原厂提供的DEMO目前只支持TYPEA协议的读写
**                 未来看产品经理的要求再做处理（支持TYPEB协议的读写），我们软件上可以请原厂提供可以兼容TYPEA及TYPEB协议的DEMO后再做优化
*/

//#define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260228"
/* 软件版本:        V1.0
** 完成日期:        2026.02.28
** 作    者:        Harrison Wu (wuyujiao@jimiiot.com)
** 修改内容:        1. 增加ext_module文件夹，为未来使用外部来源的代码引入目录；
**                 2. 完成NFC模块功能移植，并形成模块化放入ext_module\nfc目录下；
**                 3. 将原P1.14引脚改为NFC RST控制引脚，删除原电池NTC功能（已和硬件确认，NTC会由充电IC自动管理）；
**                 4. 删除my_battery.c中与NTC相关代码；
**                 5. 修改MY_BLE_TASK_STACK_SIZE定义，因没有使用系统自带的NUS功能，直接用数字代替。
*/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260228"
/* 软件版本:        V1.0
** 完成日期:        2026.02.28
** 作    者:        周森达 (zhousenda@jimiiot.com)
** 修改内容:        1.增加双路广播TAG(默认只开启IOS,预留GOOGLE)
**                 2.增加自定义SHELL指令并解析(RTT串口发送格式为:app AT_TEST "AT^GT_CM=PCBA,BT,xxxx")
**                 3.增加自定义数据读写接口并增加部分指令(FF、GG、IMEI、MODIFYGV、JATAG、JGTAG、MAC)
**                 4.暂时更改蓝牙设备名字为LL311-xxxxx(IMEI后五位)
                   注：后续可再优化一版,同时开三路广播(一路可连接、两路不可连接),两路不可连接是常广播状态(独立不受可连接广播的影响)
*/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260207"
/* 软件版本:        V1.0
** 完成日期:        2026.02.06
** 作    者:        Harrison Wu (wuyujiao@jimiiot.com)
** 修改内容:        1. 建立doc文件夹，将项目需要参考的文件统一放到该文件夹中；
*/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260206"
/* 软件版本:        V1.0
** 完成日期:        2026.02.06
** 作    者:        Harrison Wu (wuyujiao@jimiiot.com)
** 修改内容:        1. 适配了LSM6DSV16X传感器并启用my_gsensor模块；
**                 2. 删除了DA215S传感器支持; 
**                 3. 在my_shell中添加了六轴传感器数据读取功能;
**                 4. 添加了my_wdt模块，但未启用;
**                 5. 在prj.conf中启用电源管理功能;
*/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260123"
/* 软件版本:        V1.0
** 完成日期:        2026.01.23
** 作    者:        Harrison Wu (wuyujiao@jimiiot.com)
** 修改内容:        1. 修改my_shell为纯RTT输入和输出，支持人机页面，并做了几个简单交互应用；
**                  2. 取消透传功能，增加自定义GATT服务功能，将收到的蓝牙数据发消息到main模块进行日志输出；
**                  3. 更新board overlay文件，匹配我们使用的硬件平台；
**                  4. 增加内部flash分区管理文件pm_static.yml；
**                  5. 增加内部flash分区管理说明文件partitions.xlsx；
*/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260117"
/* 软件版本:        V1.0
** 完成日期:        2026.01.17
** 作    者:        Harrison Wu (wuyujiao@jimiiot.com)
** 修改内容:        头文件集中管理
*/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260115"
/*
** 软件版本:        V1.0
** 完成日期:        2026.01.15
** 作    者:        Harrison Wu (wuyujiao@jimiiot.com)
** 修改内容:        完成初台框架
*/

#endif /* _MY_VERSION_H_ */
