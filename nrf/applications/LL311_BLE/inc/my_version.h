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

#define SOFTWARE_VERSION "LL311_NRF54L15_V1.0_260207"
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
