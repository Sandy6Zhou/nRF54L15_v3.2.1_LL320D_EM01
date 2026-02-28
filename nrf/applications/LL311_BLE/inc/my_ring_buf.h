/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        my_ring_buf.h
**文件描述:        my_ring_buf.c头文件声明
**当前版本:        V1.0
**作    者:        张庆灿 (Nestor Zhang) zhangqingcan@jimiiot.com
**完成日期:        2026年2月26日
*********************************************************************/
#ifndef __MY_RING_BUFFER_H__
#define __MY_RING_BUFFER_H__

// 标准 Ring Buffer 结构体（单生产单消费无锁版）
typedef struct {
    volatile uint32_t head;  // 写指针（volatile 保证内存可见性）
    volatile uint32_t tail;  // 读指针
    uint8_t *buf;            // 缓冲区
    uint32_t size;           // 缓冲区总大小（实际可用 size-1 字节，预留1字节判满）
} ring_buffer_t;


/**
 * @brief 初始化环形缓冲区
 * @param rb: 缓冲区指针
 * @param buf: 外部传入的缓冲区数组
 * @param size: 缓冲区数组长度（建议为2的幂，取模效率更高）
 * @return 0:成功, -1:入参非法
 */
int my_rb_init(ring_buffer_t *rb, uint8_t *buf, uint32_t size);

/**
 * @brief 清空环形缓冲区
 * @param rb: 缓冲区指针
 */
void my_rb_clear(ring_buffer_t *rb);

/**
 * @brief 写入数据到缓冲区（单生产者）
 * @param rb: 缓冲区指针
 * @param data: 待写入数据指针
 * @param len: 待写入长度
 * @return 实际写入的字节数
 */
int my_rb_write(ring_buffer_t *rb, const uint8_t *data, uint32_t len);

/**
 * @brief 从缓冲区读取数据（单消费者）
 * @param rb: 缓冲区指针
 * @param data: 存储读取数据的指针
 * @param len: 期望读取的长度
 * @return 实际读取的字节数
 */
int my_rb_read(ring_buffer_t *rb, uint8_t *data, uint32_t len);

/**
 * @brief 获取缓冲区中有效数据长度
 * @param rb: 缓冲区指针
 * @return 有效数据字节数
 */
uint32_t my_rb_get_used_size(ring_buffer_t *rb);

/**
 * @brief 获取缓冲区剩余可用空间
 * @param rb: 缓冲区指针
 * @return 剩余可用字节数
 */
uint32_t my_rb_get_free_size(ring_buffer_t *rb);

#endif

