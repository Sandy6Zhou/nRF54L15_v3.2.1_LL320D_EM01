/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        my_shell.h
**文件描述:        Shell 命令行交互模块头文件
**当前版本:        V1.0
**作    者:        Harrison Wu (wuyujiao@jimiiot.com)
**完成日期:        2026.01.15
*********************************************************************
** 功能描述:        Shell 人机交互与蓝牙透传模块接口
**                 1. 通过 UART 接收 ASCII 码指令
**                 2. 蓝牙连接时将 UART 数据透传到蓝牙
**                 3. 蓝牙断开时丢弃 UART 数据
**                 4. 将蓝牙收到的数据全部透传到 UART 输出
*********************************************************************/

#ifndef _MY_SHELL_H_
#define _MY_SHELL_H_

#include <stdbool.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>

/* UART 缓冲区及超时配置，沿用 NUS 示例的配置项 */
#define SHELL_UART_BUF_SIZE           CONFIG_BT_NUS_UART_BUFFER_SIZE
#define SHELL_UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
#define SHELL_UART_WAIT_FOR_RX        CONFIG_BT_NUS_UART_RX_WAIT_TIME

/* UART 发送/接收数据缓冲结构体 */
struct shell_uart_data_t
{
    void *fifo_reserved;
    uint8_t data[SHELL_UART_BUF_SIZE];
    uint16_t len;
};

/*
 * Shell 模块初始化参数结构体：
 * - uart_dev:  对应的底层 UART 设备
 * - uart_rx_to_ble_fifo: UART RX 收到的数据，放入此 FIFO 等待 BLE 读取并发送（仅连接时）
 * - ble_tx_to_uart_fifo: BLE 收到的数据，放入此 FIFO 等待 UART 发送
 */
struct my_shell_init_param
{
    const struct device *uart_dev;

    /* UART -> BLE: 串口接收到的数据，放入此 FIFO 等待 BLE 读取并发送 */
    struct k_fifo *uart_rx_to_ble_fifo;

    /* BLE -> UART: BLE 收到的数据，放入此 FIFO 等待 UART 发送 */
    struct k_fifo *ble_tx_to_uart_fifo;
};

/********************************************************************
**函数名称:  my_shell_init
**入口参数:  param    ---        Shell 模块初始化参数（设备句柄 + 透传 FIFO）
**           tid      ---        指向线程 ID 变量的指针
**出口参数:  tid      ---        存储启动后的线程 ID
**函数功能:  初始化 Shell 模块，配置 UART 异步回调并建立与 BLE 透传 FIFO 的关联，并启动 Shell 线程
**返 回 值:  0 表示成功，负值表示失败（如设备未就绪等错误）
*********************************************************************/
int my_shell_init(const struct my_shell_init_param *param, k_tid_t *tid);

/********************************************************************
**函数名称:  my_shell_send_from_ble
**入口参数:  data     ---        待发送的数据缓存指针
**            len      ---        待发送数据长度
**出口参数:  无
**函数功能:  由 BLE 模块发起的 UART 发送接口，当 UART 忙时缓存到 FIFO 等待发送
**返 回 值:  0 表示成功，负值表示失败（如内存不足或参数非法）
*********************************************************************/
int my_shell_send_from_ble(const uint8_t *data, uint16_t len);

/********************************************************************
**函数名称:  my_shell_set_ble_connected
**入口参数:  connected ---       蓝牙连接状态（true: 已连接，false: 未连接）
**出口参数:  无
**函数功能:  设置蓝牙连接状态，控制 UART RX 数据是否透传到蓝牙
**返 回 值:  无
*********************************************************************/
void my_shell_set_ble_connected(bool connected);

#endif /* _MY_SHELL_H_ */
