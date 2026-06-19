/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        my_ring_buf.c
**文件描述:        设备平台uart数据收发处理模块
**当前版本:        V1.0
**作    者:        张庆灿(Nestor Zhang) zhangqingcan@jimiiot.com
**完成日期:        2026.02.26
*********************************************************************/
#include "my_comm.h"

/**
 * @brief 初始化环形缓冲区
 * @param rb: 缓冲区指针
 * @param buf: 外部传入的缓冲区数组
 * @param size: 缓冲区数组长度（建议为2的幂，取模效率更高）
 * @return 0:成功, -1:入参非法
 */
int my_rb_init(ring_buffer_t *rb, uint8_t *buf, uint32_t size)
{
    if (rb == NULL || buf == NULL || size < 2) { // 至少预留1字节，size≥2才有效
        return -1;
    }

    rb->head = 0;
    rb->tail = 0;
    rb->buf = buf;
    rb->size = size;

    return 0;
}

/**
 * @brief 清空环形缓冲区
 * @param rb: 缓冲区指针
 */
void my_rb_clear(ring_buffer_t *rb)
{
    if (rb == NULL) return;

    // 单生产单消费下，直接重置指针即可（无锁安全）
    rb->head = 0;
    rb->tail = 0;
}

/**
 * @brief 写入数据到缓冲区（单生产者）
 * @param rb: 缓冲区指针
 * @param data: 待写入数据指针
 * @param len: 待写入长度
 * @return 实际写入的字节数
 */
int my_rb_write(ring_buffer_t *rb, const uint8_t *data, uint32_t len)
{
    uint32_t written = 0;
    uint32_t next_head;

#if 0
    if (rb == NULL || data == NULL || len == 0 || rb->buf == NULL) {
        return 0;
    }
#endif

    while (written < len) {
        // 计算下一个写指针位置，判断队列是否已满（预留1字节）
        next_head = (rb->head + 1) % rb->size;
        if (next_head == rb->tail) {
            break; // 队列满，停止写入
        }

        // 写入单个字节，更新写指针
        rb->buf[rb->head] = data[written];
        rb->head = next_head;
        written++;
    }

    return written;
}

/**
 * @brief 从缓冲区读取数据（单消费者）
 * @param rb: 缓冲区指针
 * @param data: 存储读取数据的指针
 * @param len: 期望读取的长度
 * @return 实际读取的字节数
 */
int my_rb_read(ring_buffer_t *rb, uint8_t *data, uint32_t len)
{
    uint32_t read = 0;
    uint32_t next_tail;

#if 0
    if (rb == NULL || data == NULL || len == 0 || rb->buf == NULL) {
        return 0;
    }
#endif

    while (read < len) {
        // 判断队列是否为空
        if (rb->head == rb->tail) {
            break; // 队列空，停止读取
        }

        // 读取单个字节，更新读指针
        data[read] = rb->buf[rb->tail];
        next_tail = (rb->tail + 1) % rb->size;
        rb->tail = next_tail;
        read++;
    }

    return read;
}

/**
 * @brief 获取缓冲区中有效数据长度
 * @param rb: 缓冲区指针
 * @return 有效数据字节数
 */
uint32_t my_rb_get_used_size(ring_buffer_t *rb)
{
    if (rb == NULL) return 0;

    return (rb->head - rb->tail + rb->size) % rb->size;
}

/**
 * @brief 获取缓冲区剩余可用空间
 * @param rb: 缓冲区指针
 * @return 剩余可用字节数
 */
uint32_t my_rb_get_free_size(ring_buffer_t *rb)
{
    if (rb == NULL) return 0;

    // 预留1字节，所以可用空间是 size-1 - 已用空间
    return (rb->size - 1) - my_rb_get_used_size(rb);
}
