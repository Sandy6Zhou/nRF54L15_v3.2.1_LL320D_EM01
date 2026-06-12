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

#define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260609"
/* 软件版本:        V1.0
** 完成日期:        2026.06.09
** 作    者:        曹阳 (caoyang@jimiiot.com)
** 修改内容:       1. 增加NFC卡上锁权限设置并增加上锁逻辑
***/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260608"
/* 软件版本:        V1.0
** 完成日期:        2026.06.08
** 作    者:        曹阳 (caoyang@jimiiot.com)
** 修改内容:       1. 增加撞击检测指令和相关逻辑及报警功能
                  2. 为降低功耗，在撞击检测关闭，设备进入低功耗模式时关闭G-SENSOR电源
***/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260602"
/* 软件版本:        V1.0
** 完成日期:        2026.06.02
** 作    者:        曹阳 (caoyang@jimiiot.com)
** 修改内容:       1. 删除非法解锁相关逻辑
**                2. 重复刷卡检测从1s改为60s
***/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260529"
/* 软件版本:        V1.0
** 完成日期:        2026.05.29
** 作    者:        曹阳 (caoyang@jimiiot.com)
** 修改内容:       1.系统异常重启后不知道重启原因,添加重启原因记录
**                 2.修改MOTDET#指令的实现
**                 3.LED灯隐藏去掉，LED#指令走指令透传接口
**                 4.修改蓝牙开锁密码重置指令只能由网络触发
**                 5.当蓝牙发送不在指令表的指令时，必须回复一个cmd error
**                 6.G-SENSOR醒来后切换为高性能模式
**                 7.增加海陆运状态上报告警
**                 8.修改指令集某些指令上报方式数量，缺少一个不上报
***/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260529"
/* 软件版本:        V1.0
** 完成日期:        2026.05.29
** 作    者:       周森达 (zhousenda@jimiiot.com)
** 修改内容:        1.解决GSENSOR关闭电源的情况下，GSENSOR还一直有电
**                 2.增加宏控，支持按需开启/关闭开关锁的中断检测功能(兼容硬件电路外部没有上拉电阻的情况)
***/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260526"
/* 软件版本:        V1.0
** 完成日期:        2026.05.26
** 作    者:        曹阳 (caoyang@jimiiot.com)
** 修改内容:       1. 修复蓝牙发送指令即使不带‘#’结尾也能响应的问题
**                 2. 统一蓝牙回复格式
**                 3. 修复蓝牙发送查询指令时回复乱码的问题
***/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260525"
/* 软件版本:        V1.0
** 完成日期:        2026.05.24
** 作    者:       周森达 (zhousenda@jimiiot.com)
** 修改内容:        1.优化中断IO的触发方式(GPIOTE IN->PORT sense)
**                 2.配置uart30 rx在default状态下内部上拉
**                 3.取消开关锁限位检测配置的内部上拉(改用外部上拉)
**                 4.增加PWM的PM管理
**                 5.优化GSENSOR suspend后真正进休眠模式(LPM2)
**                 说明：功耗保持在32uA,功耗测试是GPS的RTC供电未打开、RTT串口调试关闭的情况下测试的。
***/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260524"
/* 软件版本:        V1.0
** 完成日期:        2026.05.24
** 作    者:       周森达 (zhousenda@jimiiot.com)
** 修改内容:        1.统一命名规范(静态变量：s_xxx、结构体/枚举：xxx_t)
**                 2.删除未使用到的函数及类型(my_delete_timer、my_lang_type)
***/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260522"
/* 软件版本:        V1.0
** 完成日期:        2026.05.22
** 作    者:        曹阳 (caoyang@jimiiot.com)
** 修改内容:       1. TAG配置参数上电后丢失。
**                 2.初步实现STATUS#查询设备状态指令，具体参数得和产品经理确认
***/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260521"
/* 软件版本:        V1.0
** 完成日期:        2026.05.21
** 作    者:        曹阳 (caoyang@jimiiot.com)
** 修改内容:       1. 修复拆壳检测的上报方式以及不告警的情况
**                 2.修复进入超低功耗模式后按键不能唤醒的问题
**                 3.修复蓝牙设置功率配置错误的问题
**                 4.修复NFC刷卡解锁扫描时间不对的问题
**                 5.修复设置NFC卡为管理员卡，但是还是不能用卡解锁的问题
**                 6.修复BT_UPDATA#指令收集TAG数据的参数配置重新上电后丢失的问题
**                 7.修改了一些注释有歧义的地方
***/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260518"
/* 软件版本:        V1.0
** 完成日期:        2026.05.18
** 作    者:        曹阳 (caoyang@jimiiot.com)
** 修改内容:       1.在采样定时器执行期间将G-Sensor设置成睡眠模式
**                2.增加测试命令用来关闭外设电源，比如4G模块，方便查看功耗
***/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260518"
/* 软件版本:        V1.0
** 完成日期:        2026.05.18
** 作    者:        曹阳 (caoyang@jimiiot.com)
** 修改内容:       1.增加当采集完一次数据进行模式判定后得等1min（宏定义可设置）才进行下一次数据采集进行模式判定的功能
**                2.修复当切换模式后，G-Sensor不进行数据采集的问题
***/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260511"
/* 软件版本:        V1.0
** 完成日期:        2026.05.11
** 作    者:        曹阳 (caoyang@jimiiot.com)
** 修改内容:       1.修复系统启动后没有按预设模式运行的bug
**                2.将软件轮询读取G-Sensor数据改为FIFO中断触发读取
***/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260508"
/* 软件版本:        V1.0
** 完成日期:        2026.05.08
** 作    者:        周森达 (zhousenda@jimiiot.com)
** 修改内容:         1.LTE UART空闲3秒自动挂起以降低功耗，发送/接收时动态恢复
**                  2.新增P0.04唤醒引脚中断，4G模块拉低时自动恢复UART
**                  3.LTE断电流程重构，增加发送等待超时保护与状态清理
**                  4.Battery ADC仅在读取时恢复、读取后立即挂起，避免持续耗电
***/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260508"
/* 软件版本:        V1.0
** 完成日期:        2026.05.08
** 作    者:        曹阳 (caoyang@jimiiot.com)
** 修改内容:       1.将告警逻辑拆分开，新增枚举类型,方便4G按类型识别告警
**                2.移除指令透传重传机制，4G回复直接发送给蓝牙APP
***/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260506"
/* 软件版本:        V1.0
** 完成日期:        2026.05.06
** 作    者:        曹阳 (caoyang@jimiiot.com)
** 修改内容:       1.增加6轴IMU的运输模式自动识别算法，区分静止、陆运、海运三种工作状态。
**                2.实现8维特征工程：时域统计+频域FFT分析+自相关周期性检测。
*                 3.实现贝叶斯分类器，训练每类mean/std参数，计算后验概率。
*                 4.实现状态机平滑机制：连续3次确认才切换，EMA概率平滑抗闪烁。
*                 5.实现转移约束：禁止Still到Sea直接跳转，动态先验概率加权。
***/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260506"
/* 软件版本:        V1.0
** 完成日期:        2026.05.06
** 作    者:        吴楚庆 (wuchuqing@jimiiot.com)
** 修改内容:        1.修复蜂鸣器全局变量竞态问题
**                 2. 修复在充电状态下开机，LED灯会按正常显示，但是拔掉充电，LED还是按插着充电的情况显示和
                    充电状态下开机，偶尔出现指示灯不显示（读取电量的线程跑不起来）
**                 3.更改上报告警类型从字符串改为发枚举类型
**                 4.在电机启动加上开启电机电源，停止关闭，初始化默认关闭
***/

//#define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260429"
/* 软件版本:        V1.0
** 完成日期:        2026.04.29
** 作    者:        周森达 (zhousenda@jimiiot.com)
** 修改内容:        1.根据LL311实际硬件设计，统一修正按键/gsensor/电机引脚的有效电平和触发沿方向
**                 2.修正电池采样分压计算公式，电机初始化后两控制IO置低，电机驱动输出恢复默认高阻态
***/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260428"
/* 软件版本:        V1.0
** 完成日期:        2026.04.28
** 作    者:        吴楚庆 (wuchuqing@jimiiot.com)
** 修改内容:       1.将NFC联动指令执行移到蓝牙线程
**                 2.MODESET指令处理函数加入当前模式下更新LTE唤醒定时器时间参数
**                 3.部分回复4G的代码从动态申请空间改为直接回复
**                 4.并根据需求，调整了BLE+LOCATION应答处理(增加提示音)
***/

//#define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260428"
/* 软件版本:        V1.0
** 完成日期:        2026.04.28
** 作    者:        曹阳 (caoyang@jimiiot.com)
** 修改内容:       1.增加接收LTE+LOCATION指令后面携带GPS速度参数，用来区分静止和运动状态。
**                 2.增加接收LTE+NET指令，保存网络状态，判断是否在海中。
***/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260427"
/* 软件版本:        V1.0
** 完成日期:        2026.04.27
** 作    者:        曹阳 (caoyang@jimiiot.com)
** 修改内容:       1.更新串口协议指令(工作模式切换及设置统一走BLE+MODESET)、开机发送工作模式
**                 2.优化进入超低功耗模式后，通知4G先关机
**                 3.增加蓝牙1分钟定时给4G发送心跳指令
***/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260424"
/* 软件版本:        V1.0
** 完成日期:        2026.04.24
** 作    者:        周森达 (zhousenda@jimiiot.com)
** 修改内容:        1.重构运动状态检测：加速度方差+陀螺仪均值/方差二维矩阵，初步区分静止/陆运/海运三种状态
**                 2.新增智能模式低功耗唤醒：挂起时进入15Hz唤醒待机，INT1中断唤醒主控，配合50ms消抖与350ms保护窗口过滤假唤醒
**                 3.新增状态切换滞后机制：连续3次同状态才确认，防止边界抖动
**                 4.定时器驱动采样：周期采样+burst高速填充，窗口满后判定
**                 5.新增陀螺仪零偏校准，提升检测精度
***/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260423"
/* 软件版本:        V1.0
** 完成日期:        2026.04.23
** 作    者:        周森达 (zhousenda@jimiiot.com)
** 修改内容:        1.新增单包/分包传输协议(0xFF01/0xFF02),根据MTU自动选择单包/分包
**                 2.支持APP逐包ACK确认、超时重传(2s超时,最多3次重试)
**                 3.更新UUID为0xFEE5(支持单包/分包)
***/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260423"
/* 软件版本:        V1.0
** 完成日期:        2026.04.23
** 作    者:        吴楚庆 (wuchuqing@jimiiot.com)
** 修改内容:        1.串口重传增加特殊指令处理
**                 2.根据新的指令形式，改进TAG、MACINFO的串口指令发送流程
***/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260423"
/* 软件版本:        V1.0
** 完成日期:        2026.04.23
** 作    者:        曹阳 (caoyang@jimiiot.com)
** 修改内容:       完善拆壳检测、非法解锁、锁状态变化事件触发上报
***/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260423"
/* 软件版本:        V1.0
** 完成日期:        2026.04.23
** 作    者:        曹阳 (caoyang@jimiiot.com)
** 修改内容:       增加产测指令
**                 AT^GT_CM=PCBA,MCU,LED,ON(RETURN_LED_ON)、
**                 AT^GT_CM=PCBA,MCU,LED,OFF(RETURN_MCULED_OFF)
**                 AT^GT_CM=PCBA,MCU,MAC(RETURN_MCU_MAC:)、
**                 AT^GT_CM=PCBA,MCU,GSENSOR(RETURN_MCU_GSENSOR:)
**                 AT^GT_CM=PCBA,MCU,NFC(RETURN_MCU_NFC:)
***/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260423"
/* 软件版本:        V1.0
** 完成日期:        2026.04.23
** 作    者:        吴楚庆 (wuchuqing@jimiiot.com)
** 修改内容:       1.对可设置的指令增加不带参数查询功能
***/

//#define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260420"
/* 软件版本:        V1.0
** 完成日期:        2026.04.20
** 作    者:        曹阳 (caoyang@jimiiot.com)
** 修改内容:       1.增加TAG扫描及透传mac的配置加载及数据存储
**                 2.BLE+TIME=1时间同步返回数据解析
**                 3.补充LTE+TIME时间同步指令应答
**                 4.补充LTE+FOTA重启发送应答数据
***/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260420"
/* 软件版本:        V1.0
** 完成日期:        2026.04.20
** 作    者:       吴楚庆 (wuchuqing@jimiiot.com)
** 修改内容:        1.增加4G透传需要异步回复的队列（处理异步回复)
**                 2.增加4G透传指令发送方号码后面需要对应的命令字段
**                 3.增加4G发送过来的进入产测模式指令解析
***/

//#define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260417"
/* 软件版本:        V1.0
** 完成日期:        2026.04.17
** 作    者:       周森达 (zhousenda@jimiiot.com)
** 修改内容:        1.新增BT_PARMAC指令，支持透传MAC的添加、删除、清空和查询
**                 2.新增BLE透传MAC功能，支持按配置的MAC地址筛选目标设备广播数据
**                 3.在扫描回调中增加透传MAC匹配逻辑，将匹配设备的原始ADV数据缓存并通过BLE线程异步处理
**                 4.新增透传结果表管理与MACINFO上报机制
***/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260417"
/* 软件版本:        V1.0
** 完成日期:        2026.04.17
** 作    者:        曹阳 (caoyang@jimiiot.com)
** 修改内容:        1.增加BLE启动向4G拿UTC时间串口指令
**                  2.增加4G通过串口发送进入升级的指令给蓝牙
**                  3.因为lte时间同步指令修改了参数，只有一个utc秒数参数，直接改成atoll转换写入系统时间。
**                  4.增加蓝牙OTA串口协议指令，ota升级需要等重启后才算成功，重启后再发送ble+ota=exit命令
***/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260417"
/* 软件版本:        V1.0
** 完成日期:        2026.04.17
** 作    者:        吴楚庆 (wuchuqing@jimiiot.com)
** 修改内容:        1.增加串口数据重传机制功能（用宏RETRANSMIT_CHECK_ENABLED控制）
***/

//#define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260417"
/* 软件版本:        V1.0
** 完成日期:        2026.04.17
** 作    者:        曹阳 (caoyang@jimiiot.com)
** 修改内容:        增加蓝牙OTA串口协议指令
***/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260416"
/* 软件版本:        V1.0
** 完成日期:        2026.04.16
** 作    者:        曹阳 (caoyang@jimiiot.com)
** 修改内容:        增加指令集的数据持久化存储
***/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260416"
/* 软件版本:        V1.0
** 完成日期:        2026.04.16
** 作    者:       周森达 (zhousenda@jimiiot.com)
** 修改内容:        1.新增my_ble_scan 模块，支持主动扫描、ADV/SCAN_RSP分离解析、名称前缀过滤、MAC地址聚合及按RSSI替换
**                 2.支持4种工作模式：关闭 / LTE 唤醒扫描 / 周期缓存 / 周期主动上报
**                 3.集成LTE唤醒上报链路：工作模式切换、告警触发时顺带上报
**                 4.新增BTUPDATA AT 指令与 app tagscan Shell 命令用于参数配置
***/

// #define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260416"
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
