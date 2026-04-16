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

#define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260416"
/* 软件版本:        V1.0
** 完成日期:        2026.04.16
** 作    者:        吴楚庆 (wuchuqing@jimiiot.com)
** 修改内容:        1.新增网络开锁/上锁指令，通过数据透传LTE+CMD指令
**                  2.LTE+CMD处理程序移动到与蓝牙指令处理程序同个线程
**                  3.开锁限位改为边沿触发
***/

//#define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260416"
/* 软件版本:        V1.0
** 完成日期:        2026.04.16
** 作    者:        曹阳 (caoyang@jimiiot.com)
** 修改内容:        替换所有的g_device_cmd_config为gConfigParam.
***/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260415"
/* 软件版本:        V1.0
** 完成日期:        2026.04.15
** 作    者:        曹阳 (caoyang@jimiiot.com)
** 修改内容:        增加ZMS ID持久化存储参数及读取.
***/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260414"
/* 软件版本:        V1.0
** 完成日期:        2026.04.14
** 作    者:        曹阳 (caoyang@jimiiot.com)
** 修改内容:        1.优化TAG广播，将四路广播修改为三路广播，广播方式改为可连接广播常开，不可连接广播按设置的指令打开，不可连接广播至少打开一个。
**                  2.增加两个小开关，JATAG和JGTAG，用来单独控制两个不可连接广播的打开和关闭，初始化时默认JATAG打开，另一个关闭。
**                  3.TAG,JATAG,JGTAG指令的实现。
***/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260413"
/* 软件版本:        V1.0
** 完成日期:        2026.04.13
** 作    者:        曹阳 (caoyang@jimiiot.com)
** 修改内容:        增加蓝牙功率设置指令的功能逻辑实现
***/

//#define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260410"
/* 软件版本:        V1.0
** 完成日期:        2026.04.10
** 作    者:        吴楚庆 (wuchuqing@jimiiot.com)
** 修改内容:        1.增加锁销状态告警类型,补充锁销告警上报和NFC刷卡上报
*/

//#define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260410"
/* 软件版本:        V1.0
** 完成日期:        2026.04.10
** 作    者:        吴楚庆 (wuchuqing@jimiiot.com)
** 修改内容:        1.增加接收4G应答（BLE+command=OK,参数,参数）统一接口
**                 2.新增处理应答BLE+LOCATION,OK,纬度，经度 执行函数，处理NFC刷卡解锁需要判断位置，获取经纬度
**                 3.增加LTE+LOCATION=<纬度>,<经度>用于更新储存点位置
*/

//#define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260410"
/* 软件版本:        V1.0
** 完成日期:        2026.04.10
** 作    者:        曹阳 (caoyang@jimiiot.com)
** 修改内容:        1.增加PWRSAVE,ON#指令的实现，发送该指令设备关机，系统进入深度睡眠模式。
**                  2.添加按键唤醒功能，按键按下后，系统退出深度睡眠模式。
***/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260408"
/* 软件版本:        V1.0
** 完成日期:        2026.04.08
** 作    者:        曹阳 (caoyang@jimiiot.com)
** 修改内容:        1.初步实现G-Sensor功耗管理功能，后续确认G-Sensor工作逻辑后再做调整。
***/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260407"
/* 软件版本:        V1.0
** 完成日期:        2026.04.07
** 作    者:        曹阳 (caoyang@jimiiot.com)
** 修改内容:        1.增加切换工作模式MODESET指令的实现。
**                  2.将工作模式参数由单独的变量放入到统一的全局变量结构体中，以便后续存入flash。
**                  3.去掉工作模式初始化函数和宏，改为变量定义时直接初始化赋值。
***/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260404"
/* 软件版本:        V1.0
** 完成日期:        2026.04.04
** 作    者:        周森达 (zhousenda@jimiiot.com)
** 修改内容:        1.增加开机指令应答功能(开机原因、版本号、UTC)
**                 2.实现BLE唤醒4G流程(唤醒前导帧)
*/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260403"
/* 软件版本:        V1.0
** 完成日期:        2026.04.03
** 作    者:        吴楚庆 (wuchuqing@jimiiot.com)
** 修改内容:        1.增加LTE+CMD指令透传功能
**                  2.根据产品需求，修改对应NFC卡号设置相关异常信息返回
*/

//#define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260403"
/* 软件版本:        V1.0
** 完成日期:        2026.04.03
** 作    者:        曹阳 (caoyang@jimiiot.com)
** 修改内容:        1.完善电量和充电状态上报和状态变化通知
**                  2.优化send_alarm_message_to_lte接口函数
*/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260402"
/* 软件版本:        V1.0
** 完成日期:        2026.04.02
** 作    者:        曹阳 (caoyang@jimiiot.com)
** 修改内容:        增加与4G进行时间同步，更改系统时间和日志时间戳
*/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260401"
/* 软件版本:        V1.0
** 完成日期:        2026.04.02
** 作    者:        吴楚庆 (wuchuqing@jimiiot.com)
** 修改内容:        1.增加NFC联动指令（增删查）,定义NFCTRG相关结构体
**                 2.修改my_cmd_setting.h指令解析接口，兼容" "内容为一个参数
*/

//#define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260401"
/* 软件版本:        V1.0
** 完成日期:        2026.04.01
** 作    者:        Harrison Wu (wuyujiao@jimiiot.com)
** 修改内容:        1.修正NFC BCC校验问题；
**                 2.修正NFC 7字节及10字节多级联将级联标志处理为UUID的问题
**                 3.在main中将NFC的UUID打印改为LOG_HEXDUMP_INF.
*/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260401"
/* 软件版本:        V1.0
** 完成日期:        2026.04.01
** 作    者:        Harrison Wu (wuyujiao@jimiiot.com)
** 修改内容:        1.修改overlay+prj.conf文件，禁用MX25R64 QSPI Flash，nfc引脚重新设计，匹配最新硬件；
**                 2.电源管理框架升级，电源管理api更改为pm_device_runtime_get/put,初始化时默认为suspend模式；
**                 3.NFC模块电源管理重构，引用my_pm模块管理NFC电源；
*/

//#define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260401"
/* 软件版本:        V1.0
** 完成日期:        2026.04.01
** 作    者:        曹阳 (caoyang@jimiiot.com)
** 修改内容:        1.增加类似BLE+[命令]=[参数]的统一接口
**                  2.增加指令透传统一接口。
*/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260330"
/* 软件版本:        V1.0
** 完成日期:        2026.03.30
** 作    者:        周森达 (zhousenda@jimiiot.com)
** 修改内容:        1.新增告警类型枚举、统一告警上报接口
**                 2.新增UART发送完成信号量，保证串口发送串行不冲突
**                 3.实现LTE缓存消息循环队列(最多缓存10条)
*/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260327"
/* 软件版本:        V1.0
** 完成日期:        2026.03.27
** 作    者:        曹阳 (caoyang@jimiiot.com)
** 修改内容:        增加CBMT#和VERSION#查询命令
*/

//#define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260327"
/* 软件版本:        V1.0
** 完成日期:        2026.03.27
** 作    者:        吴楚庆 (wuchuqing@jimiiot.com)
** 修改内容:        1.补充509版本日志信息：完善拆壳检测、剪线检测的处理逻辑，待上报接口实现，按接口形式处理如何上报
**                 2.增加命令触发方式，上报方式等枚举
*/

//#define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260327"
/* 软件版本:        V1.0
** 完成日期:        2026.03.27
** 作    者:        吴楚庆 (wuchuqing@jimiiot.com)
** 修改内容:        1.完善拆壳检测、剪线检测的处理逻辑，待上报接口实现，按接口形式处理如何上报
*/

//#define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260327"
/* 软件版本:        V1.0
** 完成日期:        2026.03.27
** 作    者:        曹阳 (caoyang@jimiiot.com)
** 修改内容:        1.增加隐藏指示灯的功能
**                  2.将之前代码先关闭灯再关闭定时器的操作改为先关闭定时器再关闭灯
*/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260326"
/* 软件版本:        V1.0
** 完成日期:        2026.03.26
** 作    者:        吴楚庆 (wuchuqing@jimiiot.com)
** 修改内容:        1.适配蜂鸣器逻辑，再相关处增加提示音
*/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260326"
/* 软件版本:        V1.0
** 完成日期:        2026.03.26
** 作    者:        周森达 (zhousenda@jimiiot.com)
** 修改内容:        1.增加蓝牙指令上锁/解锁相关功能
*/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260326"
/* 软件版本:        V1.0
** 完成日期:        2026.03.26
** 作    者:        周森达 (zhousenda@jimiiot.com)
** 修改内容:        1.删除多余的空格
*/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260326"
/* 软件版本:        V1.0
** 完成日期:        2026.03.26
** 作    者:        曹阳 (caoyang@jimiiot.com)
** 修改内容:        增加nfc启动，上锁中，解锁中，解锁成功后LED闪烁功能
*/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260326"
/* 软件版本:        V1.0
** 完成日期:        2026.03.26
** 作    者:        吴楚庆 (wuchuqing@jimiiot.com)
** 修改内容:        1.增加封装蜂鸣器报警类型接口
*/

//#define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260326"
/* 软件版本:        V1.0
** 完成日期:        2026.03.26
** 作    者:        周森达 (zhousenda@jimiiot.com)
** 修改内容:        1.增加非法操作的相关功能(锁销非法拔出、非法解锁)
**                 2.增加蓝牙解锁密钥管理
*/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260325"
/* 软件版本:        V1.0
** 完成日期:        2026.03.25
** 作    者:        曹阳 (caoyang@jimiiot.com)
** 修改内容:        增加两个锁LED的统一接口，通过传入模式参数来控制LED的闪烁模式
*/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260325"
/* 软件版本:        V1.0
** 完成日期:        2026.03.25
** 作    者:        曹阳 (caoyang@jimiiot.com)
** 修改内容:        1. 增加电池电压读取，转换为电量百分比，更新电池状态
                    2. 修改正常状态下的电量指示灯闪烁逻辑，改为在回调中执行
                    3. 删掉shell命令调试代码，改为直接改变外部电压测试
*/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260324.1100"
/* 软件版本:        V1.0
** 完成日期:        2026.03.23
** 作    者:        周森达 (zhousenda@jimiiot.com)
** 修改内容:        1.增加电源管理框架(my_pm.c/my_pm.h),提供统一的电源管理接口
**                 2. 添加“低功耗实施方案.md”文档
*/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260323"
/* 软件版本:        V1.0
** 完成日期:        2026.03.23
** 作    者:        周森达 (zhousenda@jimiiot.com)
** 修改内容:        1.增加NFC卡开锁规则判断(卡ID、经纬度、半径、起止时间、可用次数)
**                 2.增加NFC刷卡记录缓存机制
**                 3.增加锁销自动上锁检测
**                 4.增加开/关锁超时失败检测机制
**                 5.增加shell指令(ble_test)，用于ble的用户指令快速测试验证
*/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260323"
/* 软件版本:        V1.0
** 完成日期:        2026.03.23
** 作    者:        曹阳 (caoyang@jimiiot.com)
** 修改内容:        1. 增加充电状态电量指示灯根据电量指示灯闪烁功能
**                  2. 增加充电状态检测功能
*/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260320"
/* 软件版本:        V1.0
** 完成日期:        2026.03.20
** 作    者:        Harrison Wu (wuyujiao@jimiiot.com)
** 修改内容:        配置功能模块日志输出，采用MY_LOG_XX替换LOG_XX
*/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260319"
/* 软件版本:        V1.0
** 完成日期:        2026.03.13
** 作    者:        Harrison Wu (wuyujiao@jimiiot.com)
** 修改内容:        增加蓝牙日志模块功能
*/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260319"
/* 软件版本:        V1.0
** 完成日期:        2026.03.19
** 作    者:        曹阳 (caoyang@jimiiot.com)
** 修改内容:        1.实现按键短按电量指示灯根据电量状态闪烁功能
*/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260313"
/* 软件版本:        V1.0
** 完成日期:        2026.03.13
** 作    者:        Harrison Wu (wuyujiao@jimiiot.com)
** 修改内容:        1. 基于几米蓝牙通信协议 V3.1.6 的 6.4 OTA 文件传输协议实现DFU功能(my_jimi_dfu.c/my_jimi_dfu.h/my_bie_app.c/my_bie_app.h)；
**                 2. 支持MCUmgr OTA功能，并进行状态监听(my_ble_core.c/my_ble_core.h/main.c)；
**                 3. 增加BLE SMP配对权限，配对密钥自动使用IMEI号后6位，在在 my_ble_core_start 启动时从参数中提取并设置;
**                 4. 增加关机模式功能（my_main.h/mai.c/my_shell.c）;
**                 4. 长按键在关机模式下唤醒，自动切换到智能模式;
**                 5. 添加系统启动时系统信息打印；
**                 6. OTA效率优化，MTU扩展至498（prj.conf/my_ble_app.h),采用双PDU模式;
**                 7. 优化栈变量转为静态全局缓冲区(my_ble_app.c -ble_tx_buf\ble_encrypt_buf\ble_rsp_buf);
**                 8. 内存管理规范化：malloc/free替换为MY_MALLOC_BUFFER/MY_FREE_BUFFER；
**                 9. my_ble_app.c日志级别调整，AES密钥、明文、密文的HEX DUMP由LOG_HEXDUMP_INF除为LOG_HEXDUMP_DBG；
**                 10. my_tool新增CRC校验函数主要用于DFU CRC的验证.
*/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260313"
/* 软件版本:        V1.0
** 完成日期:        2026.03.13
** 作    者:        周森达 (zhousenda@jimiiot.com)
** 修改内容:        1.实现NFC开关锁与权限设置(包含卡号管理及权限设置相关指令)
*/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260311"
/* 软件版本:        V1.0
** 完成日期:        2026.03.11
** 作    者:        周森达 (zhousenda@jimiiot.com)
** 修改内容:        1.整理APP相关指令集并进行数据存储(其中一些比较复杂的指令暂未增加)
*/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260309"
/* 软件版本:        V1.0
** 完成日期:        2026.03.09
** 作    者:        周森达 (zhousenda@jimiiot.com)
** 修改内容:        1.增加监测MTU变化回调
**                 2.增加APP用户指令与设备交互的链路
*/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260306"
/* 软件版本:        V1.0
** 完成日期:        2026.03.06
** 作    者:        Harrison Wu (wuyujiao@jimiiot.com)
** 修改内容:       1. 在overlay中规范光感与锁销检测引脚定义；
**                 2. 增加光感检测功能，状态发生变化时发送事件到主任务；
**                 3. 增加锁销检测功能，状态发生变化时发送事件到主任务；
**                 4. 删除my_ctrl_push_msg，已有统一的接口my_send_msg/my_send_msg_data;
*/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260305"
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
