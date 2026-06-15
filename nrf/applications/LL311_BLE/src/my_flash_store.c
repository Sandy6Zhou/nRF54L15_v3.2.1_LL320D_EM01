/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        my_flash_store.c
**文件描述:        BLE扫描数据FLASH循环存储模块实现
**当前版本:        V1.0
**作    者:        周森达 (zhousenda@jimiiot.com)
**完成日期:        2026.06.12
*********************************************************************
** 功能描述:        BLE扫描数据FLASH循环存储模块实现
**                 1. TAG/MAC两个独立环形区，定长记录，整扇区批量写入
**                 2. RAM暂存缓冲攒满一整扇区(4096字节)才批量写入FLASH，不满一扇区的剩余记录不主动写
**                 3. FLASH写满后环形覆盖最旧的扇区
**                 4. 读取只移动内存里的读取位置，整批应答确认后commit才正式推进读指针并写ZMS
**                 5. 环形指针元数据存ZMS，带magic和crc校验，掉电可恢复
**注意事项:        所有接口仅在BLE线程串行调用，无锁设计
*********************************************************************/
#include "my_comm.h"

LOG_MODULE_REGISTER(my_flash_store, LOG_LEVEL_INF);

/* ========== 分区与扇区布局常量 ========== */
#define FS_SECTOR_SIZE          4096U                   // FLASH扇区大小(RRAM擦除块)
#define FS_TOTAL_SECTORS        50U                     // 数据区总扇区数(200KB/4KB)
#define FS_REGION_SECTORS       25U                     // 每个环形区扇区数(TAG/MAC各占一半)
#define FS_SECTOR_HEADER_SIZE   16U                     // 扇区头大小
#define FS_SECTOR_MAGIC         0x424C4453U             // 扇区头魔数 'BLDS':BLE Data Store（BLE数据存储）的缩写
#define FS_REC_MAGIC            0xA5U                   // 记录有效标记
#define FS_META_MAGIC           0x46535431U             // 元数据魔数+版本 'FST1':Flash STore（FLASH存储），最后的'1'是版本号

/* TAG区扇区基址=0，MAC区扇区基址=25(分区内相对扇区索引) */
#define FS_TAG_SECTOR_BASE      0U
#define FS_MAC_SECTOR_BASE      FS_REGION_SECTORS

/* 记录槽大小(对齐到16字节写块)：TAG=92+4=96，MAC=1+3+44=48 */
#define FS_TAG_REC_SIZE         96U
#define FS_MAC_REC_SIZE         48U

/* 每扇区记录条数：(4096-16)/96=42，(4096-16)/48=85 */
#define FS_TAG_REC_PER_SECTOR   ((FS_SECTOR_SIZE - FS_SECTOR_HEADER_SIZE) / FS_TAG_REC_SIZE)
#define FS_MAC_REC_PER_SECTOR   ((FS_SECTOR_SIZE - FS_SECTOR_HEADER_SIZE) / FS_MAC_REC_SIZE)

/* ========== 磁盘结构定义 ========== */
/* 16字节扇区头，用于断电后校验与重建 */
typedef struct
{
    uint32_t magic;             // 扇区魔数 FS_SECTOR_MAGIC
    uint32_t seq;               // 扇区级递增写序号(预留+诊断用途)
    uint16_t rec_count;         // 本扇区有效记录数
    uint16_t rec_size;          // 单记录槽大小(自校验)
    uint32_t reserved;          // 预留对齐
} fs_sector_header_t;

/* TAG落盘记录槽：1字节有效标记 + 3字节填充 + 92字节数据 = 96字节 */
typedef struct
{
    uint8_t magic;              // 记录有效标记 FS_REC_MAGIC
    uint8_t pad[3];             // 填充对齐(使data落在4字节边界，满足timestamp等4字节成员对齐)
    tag_scan_result_t data;     // TAG扫描结果(92字节)
} fs_tag_record_t;

/* MAC落盘记录槽：1字节有效标记 + 3字节填充 + 44字节数据 = 48字节 */
typedef struct
{
    uint8_t magic;              // 记录有效标记 FS_REC_MAGIC
    uint8_t pad[3];             // 填充对齐(使data落在4字节边界，满足timestamp等4字节成员对齐)
    tran_mac_result_item_t data;// 透传MAC结果(44字节)
} fs_mac_record_t;

/* ZMS持久化元数据结构 */
typedef struct
{
    uint32_t magic;             // 魔数+版本 FS_META_MAGIC
    uint16_t wr_sector;         // 下一个待写扇区索引(区内0..24)
    uint16_t rd_sector;         // 最旧待上报扇区索引(区内0..24)
    uint16_t valid_sectors;     // 当前已落盘的有效扇区数(0..25)
    uint16_t rd_rec_idx;        // rd_sector内已确认上报的记录数(开头跳过这么多条)
    uint32_t seq_counter;       // 扇区级全局写序号(预留+诊断用途)
    uint32_t crc;               // 对前面字段的crc32
} fs_meta_t;

/* ========== 运行时区上下文 ========== */
typedef struct
{
    /* 持久化部分(与fs_meta_t对应) */
    uint16_t wr_sector;           // 写扇区索引
    uint16_t rd_sector;           // 已确认读扇区索引
    uint16_t valid_sectors;       // 有效扇区数
    uint16_t rd_rec_idx;          // 最旧扇区里开头已确认发过的条数，下次从这条之后接着发
    uint32_t seq_counter;         // 扇区写序号(预留+诊断用途)

    /* 内存里的本轮读取位置(上报时往后推进，commit确认后才同步到rd_*) */
    uint32_t read_offset;         // 本轮已从FLASH读出的记录数(相对rd起点)
    uint16_t staging_read_idx;    // 本轮已从暂存缓冲读出的记录数
    uint16_t upload_staging_snap; // 上报开始时记下暂存有几条，本轮只发这么多，发送途中新来的下轮再发
    bool upload_active;           // 上报进行中标志，期间不落盘也不动读写指针，防止打乱正在发送的数据

    /* 类型相关常量(初始化时按类型填充) */
    uint16_t sector_base;         // 区扇区基址(0或25)
    uint16_t rec_size;            // 记录槽大小
    uint16_t rec_per_sector;      // 每扇区记录数
    uint32_t zms_id;              // 元数据ZMS ID

    /* RAM暂存缓冲 */
    uint8_t *staging_buf;         // 暂存缓冲指针(指向静态数组)
    uint16_t staging_count;       // 暂存缓冲当前记录数
} fs_region_t;

/* ========== 模块静态数据 ========== */
static const struct flash_area *s_fa;           // 数据分区句柄
static bool s_inited;                           // 初始化标志

/* 两区RAM暂存缓冲(各容纳一扇区可装的记录数，不含扇区头) */
static fs_tag_record_t s_tag_staging[FS_TAG_REC_PER_SECTOR];
static fs_mac_record_t s_mac_staging[FS_MAC_REC_PER_SECTOR];

/* 整扇区落盘组装缓冲(BLE线程串行调用，静态共用以避免BLE线程栈占用) */
static uint8_t s_sector_buf[FS_SECTOR_SIZE];

/* 两区运行时上下文 */
static fs_region_t s_region[FS_TYPE_MAX];

/********************************************************************
**函数名称:  fs_meta_calc_crc
**入口参数:  meta_ptr  ---        元数据指针（输入）
**出口参数:  无
**函数功能:  计算元数据crc字段之前所有字段的crc32校验值
**返 回 值:  crc32校验值
*********************************************************************/
static uint32_t fs_meta_calc_crc(const fs_meta_t *meta_ptr)
{
    // crc字段位于结构体末尾，对其前面的字节做crc32
    return crc32_ieee((const uint8_t *)meta_ptr, sizeof(fs_meta_t) - sizeof(uint32_t));
}

/********************************************************************
**函数名称:  fs_meta_save
**入口参数:  rg_ptr    ---        区上下文指针（输入）
**出口参数:  无
**函数功能:  将区上下文的持久部分组装为元数据并写入ZMS
**返 回 值:  0表示成功，负值表示失败
*********************************************************************/
static int fs_meta_save(fs_region_t *rg_ptr)
{
    fs_meta_t meta;
    int ret;

    memset(&meta, 0, sizeof(meta));
    meta.magic = FS_META_MAGIC;
    meta.wr_sector = rg_ptr->wr_sector;
    meta.rd_sector = rg_ptr->rd_sector;
    meta.valid_sectors = rg_ptr->valid_sectors;
    meta.rd_rec_idx = rg_ptr->rd_rec_idx;
    meta.seq_counter = rg_ptr->seq_counter;
    meta.crc = fs_meta_calc_crc(&meta);

    ret = my_user_data_write(rg_ptr->zms_id, &meta, sizeof(meta));
    if (ret != (int)sizeof(meta))
    {
        LOG_ERR("meta save fail: id=0x%08x ret=%d", rg_ptr->zms_id, ret);
        return -EIO;
    }

    return 0;
}

/********************************************************************
**函数名称:  fs_region_reset
**入口参数:  rg_ptr    ---        区上下文指针（输入/输出）
**出口参数:  rg_ptr    ---        读写位置和计数全部清零
**函数功能:  把存储区清空成初始状态(读写指针、记录计数归零，保留扇区大小等固定属性)
**返 回 值:  无
*********************************************************************/
static void fs_region_reset(fs_region_t *rg_ptr)
{
    rg_ptr->wr_sector = 0;
    rg_ptr->rd_sector = 0;
    rg_ptr->valid_sectors = 0;
    rg_ptr->rd_rec_idx = 0;
    rg_ptr->seq_counter = 0;
    rg_ptr->read_offset = 0;
    rg_ptr->staging_read_idx = 0;
    rg_ptr->upload_staging_snap = 0;
    rg_ptr->upload_active = false;
    rg_ptr->staging_count = 0;
}

/********************************************************************
**函数名称:  fs_region_load
**入口参数:  rg_ptr    ---        区上下文指针（输入/输出）
**出口参数:  rg_ptr    ---        从ZMS恢复出来的读写指针
**函数功能:  从ZMS读取并校验区元数据，恢复读写指针；元数据缺失或校验失败则当作空区处理
**返 回 值:  无
*********************************************************************/
static void fs_region_load(fs_region_t *rg_ptr)
{
    fs_meta_t meta;
    int ret;

    ret = my_user_data_read(rg_ptr->zms_id, &meta, sizeof(meta));
    // 元数据不存在、长度不符、魔数或crc校验失败，均当作空区处理
    if (ret != (int)sizeof(meta) ||
        meta.magic != FS_META_MAGIC ||
        meta.crc != fs_meta_calc_crc(&meta))
    {
        LOG_WRN("meta invalid(id=0x%08x ret=%d), reset region", rg_ptr->zms_id, ret);
        fs_region_reset(rg_ptr);
        return;
    }

    // 边界校验，防止元数据异常导致越界
    if (meta.wr_sector >= FS_REGION_SECTORS ||
        meta.rd_sector >= FS_REGION_SECTORS ||
        meta.valid_sectors > FS_REGION_SECTORS ||
        meta.rd_rec_idx > rg_ptr->rec_per_sector)
    {
        LOG_WRN("meta out of range(id=0x%08x), reset region", rg_ptr->zms_id);
        fs_region_reset(rg_ptr);
        return;
    }

    rg_ptr->wr_sector = meta.wr_sector;
    rg_ptr->rd_sector = meta.rd_sector;
    rg_ptr->valid_sectors = meta.valid_sectors;
    rg_ptr->rd_rec_idx = meta.rd_rec_idx;
    rg_ptr->seq_counter = meta.seq_counter;
    rg_ptr->staging_count = 0;

    LOG_INF("region loaded(id=0x%08x): wr=%d rd=%d valid=%d rec_idx=%d seq=%u",
            rg_ptr->zms_id, rg_ptr->wr_sector, rg_ptr->rd_sector,
            rg_ptr->valid_sectors, rg_ptr->rd_rec_idx, rg_ptr->seq_counter);
}

/********************************************************************
**函数名称:  fs_sector_offset
**入口参数:  rg_ptr      ---      区上下文指针（输入）
**           sector_idx  ---      区内扇区索引(0..24)（输入）
**出口参数:  无
**函数功能:  计算指定区内扇区在分区中的字节偏移
**返 回 值:  分区内字节偏移
*********************************************************************/
static off_t fs_sector_offset(const fs_region_t *rg_ptr, uint16_t sector_idx)
{
    return (off_t)(rg_ptr->sector_base + sector_idx) * FS_SECTOR_SIZE;
}

/********************************************************************
**函数名称:  fs_flush_staging
**入口参数:  rg_ptr    ---        区上下文指针（输入/输出）
**出口参数:  rg_ptr    ---        更新写指针/有效扇区/读指针(覆盖时)
**函数功能:  将暂存缓冲组装扇区头后整扇区写入FLASH的wr_sector，前移写指针；
**           FLASH写满时覆盖最旧扇区(前移rd指针)，最后把元数据存入ZMS
**返 回 值:  0表示成功，负值表示失败
**注意事项:  RRAM无独立擦除，直接write覆盖旧扇区即可，不调用erase
*********************************************************************/
static int fs_flush_staging(fs_region_t *rg_ptr)
{
    fs_sector_header_t *hdr_ptr;
    off_t offset;
    uint32_t data_len;
    int ret;

    // 暂存为空无需落盘
    if (rg_ptr->staging_count == 0)
    {
        return 0;
    }

    // 组装扇区缓冲：头部 + 暂存记录区(使用静态缓冲，避免BLE线程栈占用)
    memset(s_sector_buf, 0xFF, sizeof(s_sector_buf));
    hdr_ptr = (fs_sector_header_t *)s_sector_buf;
    hdr_ptr->magic = FS_SECTOR_MAGIC;
    hdr_ptr->seq = rg_ptr->seq_counter++;
    hdr_ptr->rec_count = rg_ptr->staging_count;
    hdr_ptr->rec_size = rg_ptr->rec_size;
    hdr_ptr->reserved = 0;

    // 拷贝暂存记录到扇区头之后
    data_len = (uint32_t)rg_ptr->staging_count * rg_ptr->rec_size;
    memcpy(s_sector_buf + FS_SECTOR_HEADER_SIZE, rg_ptr->staging_buf, data_len);

    // 整扇区写入wr_sector(RRAM直接覆盖，不擦除)
    offset = fs_sector_offset(rg_ptr, rg_ptr->wr_sector);
    ret = flash_area_write(s_fa, offset, s_sector_buf, FS_SECTOR_SIZE);
    if (ret != 0)
    {
        LOG_ERR("flash_area_write fail: off=%ld ret=%d", (long)offset, ret);
        rg_ptr->seq_counter--;  // 写失败回滚序号
        return ret;
    }

    // 更新有效扇区数 / 写满后覆盖最旧扇区
    if (rg_ptr->valid_sectors < FS_REGION_SECTORS)
    {
        rg_ptr->valid_sectors++;
    }
    else
    {
        // 已写满：丢弃最旧的整个扇区，读指针往后移一个
        LOG_WRN("region full(base=%d), overwrite oldest sector %d",
                rg_ptr->sector_base, rg_ptr->rd_sector);
        rg_ptr->rd_sector = (rg_ptr->rd_sector + 1) % FS_REGION_SECTORS;
        rg_ptr->rd_rec_idx = 0;
    }

    // 写指针往后移一个
    rg_ptr->wr_sector = (rg_ptr->wr_sector + 1) % FS_REGION_SECTORS;
    rg_ptr->staging_count = 0;

    // 新扇区写入后立即把元数据存入ZMS，保证掉电不丢已写入的数据
    return fs_meta_save(rg_ptr);
}

/********************************************************************
**函数名称:  fs_region_push
**入口参数:  rg_ptr    ---        区上下文指针（输入/输出）
**           data_ptr  ---        记录数据指针(TAG或MAC原始数据)（输入）
**           data_len  ---        记录数据长度（输入）
**出口参数:  rg_ptr    ---        更新暂存缓冲
**函数功能:  把一条记录写入暂存缓冲槽并置有效标记，暂存攒满一整扇区就批量写入FLASH
**返 回 值:  0表示成功，负值表示失败
*********************************************************************/
static int fs_region_push(fs_region_t *rg_ptr, const void *data_ptr, uint16_t data_len)
{
    uint8_t *slot_ptr;

    /* 正在上报时不能落盘：落盘会移动读写指针，打乱正在发送的数据。
     * 此时若暂存已满，本条只能丢弃(上报期间扫描已停，几乎不会走到这里)
     */
    if (rg_ptr->upload_active && rg_ptr->staging_count >= rg_ptr->rec_per_sector)
    {
        LOG_WRN("upload active and staging full(base=%d), drop record", rg_ptr->sector_base);
        return -EBUSY;
    }

    // 定位暂存缓冲中的目标槽，先清零再写有效标记和数据
    slot_ptr = rg_ptr->staging_buf + (uint32_t)rg_ptr->staging_count * rg_ptr->rec_size;
    memset(slot_ptr, 0, rg_ptr->rec_size);
    slot_ptr[0] = FS_REC_MAGIC;
    // 数据存放在槽内偏移：TAG/MAC槽数据均偏移4(magic 1字节+pad 3字节)，
    // 统一由记录槽结构保证，这里按记录槽的data成员实际偏移拷贝
    memcpy(slot_ptr + (rg_ptr->rec_size - data_len), data_ptr, data_len);

    rg_ptr->staging_count++;

    // 暂存攒满一整扇区就批量写入FLASH(但上报期间一律不写，保持指针不动)
    if (rg_ptr->staging_count >= rg_ptr->rec_per_sector && !rg_ptr->upload_active)
    {
        return fs_flush_staging(rg_ptr);
    }

    return 0;
}

/********************************************************************
**函数名称:  fs_read_flash_record
**入口参数:  rg_ptr      ---      区上下文指针（输入）
**           sector_idx  ---      区内扇区索引（输入）
**           rec_idx     ---      扇区内记录索引（输入）
**           out_data    ---      接收记录数据的缓冲（输出）
**           data_len    ---      期望读取的数据长度（输入）
**出口参数:  out_data    ---      存储读取到的记录原始数据
**函数功能:  从FLASH指定扇区指定记录槽读取一条记录的数据部分，并校验记录魔数
**返 回 值:  1表示读到有效记录，0表示记录无效，负值表示读失败
*********************************************************************/
static int fs_read_flash_record(const fs_region_t *rg_ptr, uint16_t sector_idx,
                                uint16_t rec_idx, void *out_data, uint16_t data_len)
{
    uint8_t slot[FS_TAG_REC_SIZE];   // 取最大槽尺寸做缓冲
    off_t offset;
    int ret;

    // 计算记录槽在分区内的字节偏移
    offset = fs_sector_offset(rg_ptr, sector_idx) + FS_SECTOR_HEADER_SIZE +
             (off_t)rec_idx * rg_ptr->rec_size;

    ret = flash_area_read(s_fa, offset, slot, rg_ptr->rec_size);
    if (ret != 0)
    {
        LOG_ERR("flash_area_read fail: off=%ld ret=%d", (long)offset, ret);
        return ret;
    }

    // 校验记录有效标记
    if (slot[0] != FS_REC_MAGIC)
    {
        return 0;
    }

    // 拷贝数据部分(槽尾部data_len字节)
    memcpy(out_data, slot + (rg_ptr->rec_size - data_len), data_len);
    return 1;
}

/********************************************************************
**函数名称:  fs_get_region
**入口参数:  type      ---        数据类型（输入）
**出口参数:  无
**函数功能:  根据类型返回区上下文指针，类型非法返回NULL
**返 回 值:  区上下文指针或NULL
*********************************************************************/
static fs_region_t *fs_get_region(fs_data_type_t type)
{
    if (type >= FS_TYPE_MAX)
    {
        return NULL;
    }

    return &s_region[type];
}

/********************************************************************
**函数名称:  my_flash_store_init
**入口参数:  无
**出口参数:  无
**函数功能:  初始化FLASH循环存储模块：打开分区、填充两区类型常量、加载元数据
**返 回 值:  0表示成功，负值表示失败
*********************************************************************/
int my_flash_store_init(void)
{
    fs_region_t *tag_rg;
    fs_region_t *mac_rg;
    int err;

    if (s_inited)
    {
        return 0;
    }

    // 编译期保证记录槽大小与每扇区记录数符合预期
    BUILD_ASSERT(sizeof(fs_tag_record_t) == FS_TAG_REC_SIZE, "TAG record size mismatch");
    BUILD_ASSERT(sizeof(fs_mac_record_t) == FS_MAC_REC_SIZE, "MAC record size mismatch");
    BUILD_ASSERT(sizeof(fs_sector_header_t) == FS_SECTOR_HEADER_SIZE, "sector header size mismatch");
    BUILD_ASSERT(FS_TAG_SECTOR_BASE + FS_REGION_SECTORS == FS_MAC_SECTOR_BASE, "TAG region overlaps MAC");
    BUILD_ASSERT(FS_MAC_SECTOR_BASE + FS_REGION_SECTORS == FS_TOTAL_SECTORS, "region layout overflow");

    // 打开数据分区
    err = flash_area_open(FLASH_AREA_ID(ble_data_storage), &s_fa);
    if (err)
    {
        LOG_ERR("flash_area_open ble_data_storage fail: %d", err);
        return err;
    }

    if (!device_is_ready(s_fa->fa_dev))
    {
        LOG_ERR("ble_data_storage flash device not ready");
        flash_area_close(s_fa);
        s_fa = NULL;
        return -ENODEV;
    }

    // 填充TAG区类型常量
    tag_rg = &s_region[FS_TYPE_TAG];
    memset(tag_rg, 0, sizeof(fs_region_t));
    tag_rg->sector_base = FS_TAG_SECTOR_BASE;
    tag_rg->rec_size = FS_TAG_REC_SIZE;
    tag_rg->rec_per_sector = FS_TAG_REC_PER_SECTOR;
    tag_rg->zms_id = ZMS_ID_BLE_TAG_STORE_META;
    tag_rg->staging_buf = (uint8_t *)s_tag_staging;

    // 填充MAC区类型常量
    mac_rg = &s_region[FS_TYPE_MAC];
    memset(mac_rg, 0, sizeof(fs_region_t));
    mac_rg->sector_base = FS_MAC_SECTOR_BASE;
    mac_rg->rec_size = FS_MAC_REC_SIZE;
    mac_rg->rec_per_sector = FS_MAC_REC_PER_SECTOR;
    mac_rg->zms_id = ZMS_ID_BLE_MAC_STORE_META;
    mac_rg->staging_buf = (uint8_t *)s_mac_staging;

    // 从ZMS加载并恢复两区的读写指针
    fs_region_load(tag_rg);
    fs_region_load(mac_rg);

    s_inited = true;
    LOG_INF("flash store init ok: TAG %d rec/sector, MAC %d rec/sector",
            FS_TAG_REC_PER_SECTOR, FS_MAC_REC_PER_SECTOR);
    return 0;
}

/********************************************************************
**函数名称:  my_flash_store_push_tag
**入口参数:  rec_ptr   ---        待存储的TAG扫描结果指针（输入）
**出口参数:  无
**函数功能:  追加一条TAG记录到TAG区暂存缓冲，攒满一整扇区就写入FLASH
**返 回 值:  0表示成功，负值表示失败
*********************************************************************/
int my_flash_store_push_tag(const tag_scan_result_t *rec_ptr)
{
    if (!s_inited || rec_ptr == NULL)
    {
        return -EINVAL;
    }

    return fs_region_push(&s_region[FS_TYPE_TAG], rec_ptr, sizeof(tag_scan_result_t));
}

/********************************************************************
**函数名称:  my_flash_store_push_mac
**入口参数:  rec_ptr   ---        待存储的透传MAC结果项指针（输入）
**出口参数:  无
**函数功能:  追加一条透传MAC记录到MAC区暂存缓冲，攒满一整扇区就写入FLASH
**返 回 值:  0表示成功，负值表示失败
*********************************************************************/
int my_flash_store_push_mac(const tran_mac_result_item_t *rec_ptr)
{
    if (!s_inited || rec_ptr == NULL)
    {
        return -EINVAL;
    }

    return fs_region_push(&s_region[FS_TYPE_MAC], rec_ptr, sizeof(tran_mac_result_item_t));
}

/********************************************************************
**函数名称:  my_flash_store_pending_count
**入口参数:  type      ---        数据类型（输入）
**出口参数:  无
**函数功能:  返回待上报记录总数 = FLASH有效扇区记录数 + 暂存中尚未写入FLASH的记录数
**返 回 值:  待上报记录数量
*********************************************************************/
uint32_t my_flash_store_pending_count(fs_data_type_t type)
{
    fs_region_t *rg_ptr;
    uint32_t flash_recs;

    rg_ptr = fs_get_region(type);
    if (!s_inited || rg_ptr == NULL)
    {
        return 0;
    }

    // FLASH中记录总数 = 有效扇区数 × 每扇区记录数
    flash_recs = (uint32_t)rg_ptr->valid_sectors * rg_ptr->rec_per_sector;
    // 扣除已确认读扇区内已上报的记录偏移(rd_rec_idx)，得到FLASH中真正待上报的记录数
    if (flash_recs >= rg_ptr->rd_rec_idx)
    {
        flash_recs -= rg_ptr->rd_rec_idx;
    }
    else
    {
        // 异常防御：flash_recs为无符号数，若元数据损坏致rd_rec_idx大于总数，
        // 直接相减会下溢成超大正数使上报逻辑跑飞，此处钳位到0
        flash_recs = 0;
    }

    return flash_recs + rg_ptr->staging_count;
}

/********************************************************************
**函数名称:  my_flash_store_read_next
**入口参数:  type        ---      数据类型（输入）
**           out_rec_ptr ---      接收记录的缓冲区（输出）
**出口参数:  out_rec_ptr ---      存储读取到的一条记录
**函数功能:  从当前读取位置读下一条待上报记录(先读FLASH再读暂存)，只往后移动读取位置
**返 回 值:  1表示读到，0表示无数据，负值表示失败
*********************************************************************/
int my_flash_store_read_next(fs_data_type_t type, void *out_rec_ptr)
{
    fs_region_t *rg_ptr;
    uint8_t *slot_ptr;
    uint16_t data_len;
    uint32_t flash_total;
    uint32_t abs_rec;
    uint16_t sector_idx;
    uint16_t rec_idx;
    int ret;

    rg_ptr = fs_get_region(type);
    if (!s_inited || rg_ptr == NULL || out_rec_ptr == NULL)
    {
        return -EINVAL;
    }

    data_len = (type == FS_TYPE_TAG) ? sizeof(tag_scan_result_t)
                                     : sizeof(tran_mac_result_item_t);

    /* 兜底：正常应先调用my_flash_store_upload_begin开启上报。
     * 若漏调了，这里在第一次读取时补做两件事：打开上报标志、记下当前暂存记录数，
     * 避免读取过程中数据被改动而读错
     */
    if (!rg_ptr->upload_active)
    {
        rg_ptr->upload_active = true;
        rg_ptr->upload_staging_snap = rg_ptr->staging_count;
    }

    // FLASH中待上报记录总数 = 有效扇区记录数 - 已确认读偏移
    flash_total = (uint32_t)rg_ptr->valid_sectors * rg_ptr->rec_per_sector;
    if (flash_total >= rg_ptr->rd_rec_idx)
    {
        flash_total -= rg_ptr->rd_rec_idx;
    }
    else
    {
        flash_total = 0;
    }

    // 阶段一：读取FLASH中尚未读出的记录(因落盘均为满扇区，正常不存在无效记录)
    // 用循环跳过校验失败的异常记录，避免递归调用导致BLE线程栈溢出
    while (rg_ptr->read_offset < flash_total)
    {
        // 绝对记录序号 = rd起点偏移 + 本轮已读偏移
        abs_rec = rg_ptr->rd_rec_idx + rg_ptr->read_offset;
        // 由绝对序号反算位置：序号÷每扇区记录数=跨过的扇区数，从rd_sector往后数这么多个，
        // 再对区扇区总数取模实现环形回绕，得到该记录所在的扇区索引
        sector_idx = (rg_ptr->rd_sector + abs_rec / rg_ptr->rec_per_sector) % FS_REGION_SECTORS;
        // 序号对每扇区记录数取余，得到该记录在所在扇区内的第几条(0起)
        rec_idx = abs_rec % rg_ptr->rec_per_sector;

        ret = fs_read_flash_record(rg_ptr, sector_idx, rec_idx, out_rec_ptr, data_len);
        if (ret < 0)
        {
            return ret;
        }
        rg_ptr->read_offset++;
        // 读到有效记录立即返回；校验失败(数据异常)则继续循环
        if (ret == 1)
        {
            return 1;
        }
    }

    // 阶段二：FLASH读完后，读取暂存缓冲中还没写入FLASH的剩余记录(只发本轮开始时记下的那些条)
    if (rg_ptr->staging_read_idx < rg_ptr->upload_staging_snap)
    {
        slot_ptr = rg_ptr->staging_buf +
                   (uint32_t)rg_ptr->staging_read_idx * rg_ptr->rec_size;
        memcpy(out_rec_ptr, slot_ptr + (rg_ptr->rec_size - data_len), data_len);
        rg_ptr->staging_read_idx++;
        return 1;
    }

    return 0;
}

/********************************************************************
**函数名称:  my_flash_store_rewind
**入口参数:  type      ---        数据类型（输入）
**出口参数:  无
**函数功能:  上报中途失败时调用，把读取位置退回到上次成功确认的地方，
**           让这一轮还没确认的数据下次从头重新发送
**返 回 值:  无
*********************************************************************/
void my_flash_store_rewind(fs_data_type_t type)
{
    fs_region_t *rg_ptr;

    rg_ptr = fs_get_region(type);
    if (!s_inited || rg_ptr == NULL)
    {
        return;
    }

    // 把本轮已读出的计数清零，读取位置退回到上次commit确认的起点
    rg_ptr->read_offset = 0;
    rg_ptr->staging_read_idx = 0;
    // 关闭上报标志，恢复正常落盘(这一轮被打断，下次重新开启上报会话从头再发)
    rg_ptr->upload_active = false;
    rg_ptr->upload_staging_snap = 0;
}

/********************************************************************
**函数名称:  my_flash_store_upload_begin
**入口参数:  type      ---        数据类型（输入）
**出口参数:  无
**函数功能:  开启一次上报会话：把读取位置退回到上次已确认处，立即打开上报标志
**           并记下当前暂存记录数。此后push不再写FLASH、读写指针保持不动，确保
**           整轮实际发送数与START声明的总数严格一致
**返 回 值:  无
**注意事项:  须在发送START之后、首条read_next之前调用；由commit或rewind关闭上报
**           标志。本接口已包含读取位置复位，调用前无需再调rewind
*********************************************************************/
void my_flash_store_upload_begin(fs_data_type_t type)
{
    fs_region_t *rg_ptr;

    rg_ptr = fs_get_region(type);
    if (!s_inited || rg_ptr == NULL)
    {
        return;
    }

    // 读取位置退回到上次已确认的rd处(本轮读取计数清零)
    rg_ptr->read_offset = 0;
    rg_ptr->staging_read_idx = 0;
    // 打开上报标志并记下当前暂存记录数，锁定本轮最多能发的暂存条数，
    // 防止串口协议START发出后到首条read_next之间新存入的记录被多发(超出START声明的总数)
    rg_ptr->upload_active = true;
    rg_ptr->upload_staging_snap = rg_ptr->staging_count;
}

/********************************************************************
**函数名称:  my_flash_store_commit
**入口参数:  type      ---        数据类型（输入）
**出口参数:  无
**函数功能:  对端确认整批收到后调用：把本轮读到的位置正式固定为已确认位置，
**           已发完的整扇区腾空回收，暂存里已发的记录清掉，最后存一次元数据
**返 回 值:  0表示成功，负值表示失败
*********************************************************************/
int my_flash_store_commit(fs_data_type_t type)
{
    fs_region_t *rg_ptr;
    uint32_t abs_consumed;
    uint16_t consumed_sectors;
    uint16_t remain;
    int ret;

    rg_ptr = fs_get_region(type);
    if (!s_inited || rg_ptr == NULL)
    {
        return -EINVAL;
    }

    // 从读起点算起本轮总共确认了多少条(上次残留偏移 + 本轮新读出的条数)
    abs_consumed = rg_ptr->rd_rec_idx + rg_ptr->read_offset;
    // 确认条数 ÷ 每扇区记录数 = 已经完整发完、可以腾空回收的扇区数
    consumed_sectors = (uint16_t)(abs_consumed / rg_ptr->rec_per_sector);

    // 把读起点往后挪过这些发完的扇区，不满一扇区的剩余条数留在新的起点扇区里
    if (consumed_sectors > 0)
    {
        // 读扇区起点往后移consumed_sectors个，对区扇区总数取模实现环形回绕
        rg_ptr->rd_sector = (rg_ptr->rd_sector + consumed_sectors) % FS_REGION_SECTORS;
        // 发完的整扇区被腾空，有效扇区数相应减少
        if (rg_ptr->valid_sectors >= consumed_sectors)
        {
            rg_ptr->valid_sectors -= consumed_sectors;
        }
        else
        {
            // 异常防御：valid_sectors为无符号数，若状态异常致回收数大于有效数，
            // 直接相减会下溢成超大正数，此处钳位到0
            rg_ptr->valid_sectors = 0;
        }
    }
    // 不足一扇区的剩余条数，记为新起点扇区内已确认的偏移
    rg_ptr->rd_rec_idx = (uint16_t)(abs_consumed % rg_ptr->rec_per_sector);

    // 整理暂存缓冲：把已发出确认的记录删掉，剩下没发的(含本轮上报期间新存入的)往前挪到开头
    if (rg_ptr->staging_read_idx > 0)
    {
        // staging_read_idx 受 upload_staging_snap 上限约束，且上报期间 staging_count 只增不减，
        // 故 staging_count >= staging_read_idx 恒成立；此处仍做防御性钳位避免下溢
        if (rg_ptr->staging_count >= rg_ptr->staging_read_idx)
        {
            remain = rg_ptr->staging_count - rg_ptr->staging_read_idx;
        }
        else
        {
            remain = 0;
        }

        if (remain > 0)
        {
            memmove(rg_ptr->staging_buf,
                    rg_ptr->staging_buf +
                        (uint32_t)rg_ptr->staging_read_idx * rg_ptr->rec_size,
                    (uint32_t)remain * rg_ptr->rec_size);
        }
        rg_ptr->staging_count = remain;
    }

    // 本轮读取计数清零(下次从新的已确认起点开始)，并关闭上报标志恢复正常落盘
    rg_ptr->read_offset = 0;
    rg_ptr->staging_read_idx = 0;
    rg_ptr->upload_staging_snap = 0;
    rg_ptr->upload_active = false;

    // 统一持久化一次元数据
    ret = fs_meta_save(rg_ptr);
    LOG_INF("commit(id=0x%08x): rd=%d valid=%d rec_idx=%d staging=%d",
            rg_ptr->zms_id, rg_ptr->rd_sector, rg_ptr->valid_sectors,
            rg_ptr->rd_rec_idx, rg_ptr->staging_count);
    return ret;
}

/********************************************************************
**函数名称:  my_flash_store_clear
**入口参数:  type      ---        数据类型（输入）
**出口参数:  无
**函数功能:  清空指定区(读写指针归零、有效扇区清零、暂存清空)并持久化元数据
**返 回 值:  无
*********************************************************************/
void my_flash_store_clear(fs_data_type_t type)
{
    fs_region_t *rg_ptr;

    rg_ptr = fs_get_region(type);
    if (!s_inited || rg_ptr == NULL)
    {
        return;
    }

    fs_region_reset(rg_ptr);
    fs_meta_save(rg_ptr);
    LOG_INF("region cleared(id=0x%08x)", rg_ptr->zms_id);
}

#if FS_STORE_TEST_ENABLE
/********************************************************************
**函数名称:  my_flash_store_get_debug_info
**入口参数:  type      ---        数据类型（输入）
**           info_ptr  ---        接收调试状态的结构体指针（输出）
**出口参数:  info_ptr  ---        存储指定区的内部环形指针等运行状态
**函数功能:  导出指定类型存储区的内部运行状态，仅供测试或诊断查看
**返 回 值:  0表示成功，负值表示失败
*********************************************************************/
int my_flash_store_get_debug_info(fs_data_type_t type, fs_debug_info_t *info_ptr)
{
    fs_region_t *rg_ptr;

    rg_ptr = fs_get_region(type);
    if (!s_inited || rg_ptr == NULL || info_ptr == NULL)
    {
        return -EINVAL;
    }

    // 逐项拷贝内部运行状态到外部结构(只读，不改变任何指针)
    info_ptr->wr_sector = rg_ptr->wr_sector;
    info_ptr->rd_sector = rg_ptr->rd_sector;
    info_ptr->valid_sectors = rg_ptr->valid_sectors;
    info_ptr->rd_rec_idx = rg_ptr->rd_rec_idx;
    info_ptr->seq_counter = rg_ptr->seq_counter;
    info_ptr->staging_count = rg_ptr->staging_count;
    info_ptr->rec_per_sector = rg_ptr->rec_per_sector;
    info_ptr->region_sectors = FS_REGION_SECTORS;
    info_ptr->upload_staging_snap = rg_ptr->upload_staging_snap;
    info_ptr->read_offset = rg_ptr->read_offset;
    info_ptr->staging_read_idx = rg_ptr->staging_read_idx;
    info_ptr->upload_active = rg_ptr->upload_active;
    info_ptr->pending_count = my_flash_store_pending_count(type);

    return 0;
}
#endif /* FS_STORE_TEST_ENABLE */
