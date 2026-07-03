# IMU 模块使用示例

模块路径：`ext_module/imu`
适用器件：BMI325 六轴惯性测量单元（3 轴加速度计 + 3 轴陀螺仪 + 温度）
头文件：`#include "imu_api.h"`

---

## 一、模块结构

```
imu/
├── api/
│   ├── imu_api.h        统一对外接口（应用层只需包含此头文件）
│   └── imu_api.c
├── port/
│   ├── bmi325_port.h    Zephyr 端口层（I2C/SPI 总线、INT GPIO，应用层无需直接调用）
│   └── bmi325_port.c
└── vendor/              BMI325 原厂驱动（应用层无需直接调用）
    ├── bmi3.c / bmi3.h
    ├── bmi325.c / bmi325.h
    ├── bmi325_defs.h
    └── bmi3_defs.h
```

应用层只与 `imu_api.h` 交互，总线访问、寄存器细节、feature engine 加载等差异全部由 API 层与端口层屏蔽。

> 硬件约定：当前板级仅将 BMI325 的 **INT1** 接入 MCU GPIO，**INT2** 仅做寄存器映射预留，未接出物理中断线。

---

## 二、数据类型速查

### 返回码 `imu_result_t`

| 值 | 宏 | 含义 |
|----|-----|------|
| 0 | IMU_SUCCESS | 成功 |
| 1 | IMU_ERROR_INIT | 未初始化 / 初始化失败 |
| 2 | IMU_ERROR_COMM | 总线通信错误 |
| 3 | IMU_ERROR_CHIP_ID | 芯片 ID 不匹配（期望 0x0045） |
| 4 | IMU_ERROR_PARAM | 参数错误（空指针 / 越界枚举） |
| 5 | IMU_ERROR_TIMEOUT | 操作超时 |

### 加速度量程 `imu_acc_range_t`

| 宏 | 量程 | 备注 |
|-----|------|------|
| IMU_ACC_RANGE_2G | ±2g | 灵敏度最高 |
| IMU_ACC_RANGE_4G | ±4g | |
| IMU_ACC_RANGE_8G | ±8g | 默认值 |
| IMU_ACC_RANGE_16G | ±16g | 量程最大 |
| IMU_ACC_RANGE_MAX | 枚举边界 | 仅用于校验，不可作为入参 |

### 陀螺仪量程 `imu_gyr_range_t`

| 宏 | 量程 |
|-----|------|
| IMU_GYR_RANGE_125DPS | ±125 °/s |
| IMU_GYR_RANGE_250DPS | ±250 °/s |
| IMU_GYR_RANGE_500DPS | ±500 °/s |
| IMU_GYR_RANGE_1000DPS | ±1000 °/s |
| IMU_GYR_RANGE_2000DPS | ±2000 °/s（默认值） |
| IMU_GYR_RANGE_MAX | 枚举边界，仅用于校验 |

### 输出数据率 `imu_odr_t`

`IMU_ODR_12_5HZ` / `25HZ` / `50HZ` / `100HZ`（默认） / `200HZ` / `400HZ` / `800HZ` / `1600HZ`，
边界值 `IMU_ODR_MAX` 仅用于校验。ODR 越高响应越快、功耗越大。

### 电源模式 `imu_power_mode_t`

| 宏 | 说明 |
|-----|------|
| IMU_POWER_SUSPEND | 挂起（加速度/陀螺仪均关闭，最低功耗） |
| IMU_POWER_LOW_POWER | 低功耗模式 |
| IMU_POWER_NORMAL | 普通模式 |
| IMU_POWER_HIGH_PERF | 高性能模式（默认） |
| IMU_POWER_MAX | 枚举边界，仅用于校验 |

### 中断引脚 `imu_int_pin_t`

| 宏 | 说明 |
|-----|------|
| IMU_INT_NONE | 不映射（用于解除映射） |
| IMU_INT_PIN1 | INT1，已接入 MCU GPIO |
| IMU_INT_PIN2 | INT2，仅寄存器映射预留，物理上未接出 |
| IMU_INT_PIN_MAX | 枚举边界，仅用于校验 |

### 中断源 `imu_int_src_t`

数据就绪类：`IMU_INT_SRC_ACC_DRDY` / `GYR_DRDY` / `TEMP_DRDY`
FIFO 类：`IMU_INT_SRC_FIFO_WATERMARK` / `FIFO_FULL`
特征类：`IMU_INT_SRC_ANY_MOTION` / `NO_MOTION` / `FLAT` / `ORIENTATION` / `STEP_DETECTOR` / `STEP_COUNTER` / `SIG_MOTION` / `TILT` / `TAP` / `FEATURE_STATUS`
边界值 `IMU_INT_SRC_MAX` 仅用于校验。

### 特征类型 `imu_feature_t`

`IMU_FEATURE_NO_MOTION` / `ANY_MOTION` / `FLAT` / `ORIENTATION` / `STEP_DETECTOR` / `STEP_COUNTER` / `SIG_MOTION` / `TILT` / `TAP_SINGLE` / `TAP_DOUBLE` / `TAP_TRIPLE`，
边界值 `IMU_FEATURE_MAX` 仅用于校验。

### 数据结构

```c
/* 换算后数据（工程单位） */
struct imu_data
{
    int32_t acc_x;          /* 加速度 X，单位 mg（毫重力加速度），1000 = 1g */
    int32_t acc_y;
    int32_t acc_z;
    int32_t gyr_x;          /* 角速度 X，单位 mdps（毫度每秒），1000 = 1°/s */
    int32_t gyr_y;
    int32_t gyr_z;
    int16_t temperature;    /* 温度，单位 0.01°C，如 2550 = 25.50°C */
};

/* 原始寄存器数据（未换算的 16-bit 有符号 LSB） */
struct imu_raw_data
{
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t gyr_x;
    int16_t gyr_y;
    int16_t gyr_z;
    int16_t temperature;
};

/* 加速度计 + 陀螺仪基础配置 */
struct imu_config
{
    imu_acc_range_t acc_range;      /* 加速度量程 */
    imu_odr_t       acc_odr;        /* 加速度输出数据率 */
    imu_gyr_range_t gyr_range;      /* 陀螺仪量程 */
    imu_odr_t       gyr_odr;        /* 陀螺仪输出数据率 */
    imu_power_mode_t power_mode;    /* 电源模式（同时作用于加速度计和陀螺仪） */
};

/* FIFO 配置 */
struct imu_fifo_config
{
    bool acc_en;            /* 加速度入 FIFO */
    bool gyr_en;            /* 陀螺仪入 FIFO */
    bool temp_en;           /* 温度入 FIFO（半速率采样） */
    bool time_en;           /* sensortime 入 FIFO */
    bool stop_on_full;      /* true=满即停，false=满则覆盖最旧数据 */
    uint16_t watermark;     /* FIFO 水印阈值（字为单位），用于触发水印中断 */
};

/* 中断引脚电气配置 */
struct imu_int_config
{
    bool active_high;       /* true=高有效，false=低有效 */
    bool open_drain;        /* true=开漏，false=推挽 */
    bool latch;             /* true=锁存（需读 INT 状态清除），false=脉冲 */
};

/* 轴映射配置 */
struct imu_axis_map
{
    uint8_t axis_map;       /* 轴重映射编码，取值 0~5（原厂 remap 定义） */
    bool invert_x;          /* X 轴取反 */
    bool invert_y;          /* Y 轴取反 */
    bool invert_z;          /* Z 轴取反 */
};

/* INT GPIO 中断回调（在 GPIO 中断上下文执行，仅做轻量处理） */
typedef void (*imu_int_callback_t)(void);
```

> 数值换算：g = `acc / 1000.0`；°/s = `gyr / 1000.0`；摄氏度 = `temperature / 100.0`。

---

## 三、接口列表

| 接口 | 功能 |
|------|------|
| `imu_init(const struct imu_config *config)` | 初始化模块；传 NULL 使用默认配置 |
| `imu_get_chip_id(uint8_t *id)` | 读取芯片 ID（期望 0x45） |
| `imu_read_raw(struct imu_raw_data *raw)` | 读取一次原始 6 轴 + 温度数据 |
| `imu_read(struct imu_data *data)` | 读取并换算为工程单位 |
| `imu_set_config(const struct imu_config *config)` | 运行期重新配置量程/ODR/电源模式 |
| `imu_set_power_mode(imu_power_mode_t mode)` | 仅切换电源模式，保留其他配置 |
| `imu_read_status(uint16_t *status)` | 读 STATUS 寄存器（数据就绪标志等） |
| `imu_read_err_reg(uint16_t *err_reg)` | 读错误寄存器 |
| `imu_read_sensor_time(uint32_t *sensor_time)` | 读传感器时间戳 |
| `imu_int_pin_config(pin, config)` | 配置中断引脚电气特性 |
| `imu_int_map(src, pin)` | 将中断源映射到指定引脚 |
| `imu_register_int_callback(callback)` | 注册 INT GPIO 中断回调 |
| `imu_read_int_status(pin, status)` | 读取并清除指定引脚中断状态 |
| `imu_fifo_config(const struct imu_fifo_config *config)` | 配置并清空 FIFO |
| `imu_fifo_read(frames, max_frames, frame_count)` | 批量读取 FIFO 帧 |
| `imu_fifo_flush(void)` | 清空 FIFO |
| `imu_feature_enable(feature, enable)` | 使能/禁用 feature engine 特征 |
| `imu_step_counter_enable(bool enable)` | 显式使能/禁用计步器 |
| `imu_get_step_count(uint32_t *count)` | 读取累计步数 |
| `imu_set_axis_map(const struct imu_axis_map *axis_map)` | 设置轴映射 |
| `imu_get_axis_map(struct imu_axis_map *axis_map)` | 读取当前轴映射 |

---

## 四、使用示例

### Demo 1：初始化模块

初始化是使用任何功能前的必要步骤，内部会完成端口层总线初始化、芯片软复位、芯片 ID 校验，并加载可穿戴场景的 feature engine 参数。

```c
#include "imu_api.h"

struct imu_config imu_cfg;
imu_result_t ret;

/* 加速度 ±8g / 100Hz，陀螺仪 ±2000dps / 100Hz，高性能模式 */
imu_cfg.acc_range  = IMU_ACC_RANGE_8G;
imu_cfg.acc_odr    = IMU_ODR_100HZ;
imu_cfg.gyr_range  = IMU_GYR_RANGE_2000DPS;
imu_cfg.gyr_odr    = IMU_ODR_100HZ;
imu_cfg.power_mode = IMU_POWER_HIGH_PERF;

ret = imu_init(&imu_cfg);
if (ret != IMU_SUCCESS)
{
    /* ret=3 芯片ID不匹配，ret=2 通信失败，ret=1 端口层初始化失败 */
    MY_LOG_ERR("imu init failed: %d", ret);
    return;
}
```

也可以传入 `NULL` 使用模块内置默认配置（±8g / ±2000dps / 100Hz / 高性能）：

```c
if (imu_init(NULL) != IMU_SUCCESS)
{
    MY_LOG_ERR("imu init failed");
    return;
}
```

> 量程、ODR、电源模式任一越界（>= 对应 `_MAX`）都会直接返回 `IMU_ERROR_PARAM`，不会触碰硬件。

### Demo 2：读取芯片 ID（自检）

上电自检，确认器件在位且总线通信正常。

```c
uint8_t chip_id = 0;

if (imu_get_chip_id(&chip_id) == IMU_SUCCESS)
{
    MY_LOG_INF("imu chip id: 0x%02X", chip_id);  /* 正常应为 0x45 */
}
```

> `imu_get_chip_id` 返回的是初始化阶段缓存的芯片 ID，调用前必须先 `imu_init` 成功，否则返回 `IMU_ERROR_INIT`。

### Demo 3：读取换算后的 6 轴数据

最常用的数据读取方式，直接拿到工程单位（mg / mdps / 0.01°C）。

```c
struct imu_data data;

if (imu_read(&data) == IMU_SUCCESS)
{
    MY_LOG_INF("acc: %d, %d, %d mg", data.acc_x, data.acc_y, data.acc_z);
    MY_LOG_INF("gyr: %d, %d, %d mdps", data.gyr_x, data.gyr_y, data.gyr_z);
    MY_LOG_INF("temp: %d.%02d C", data.temperature / 100, data.temperature % 100);
}
```

### Demo 4：读取原始数据 + 数据就绪判断

需要自行处理换算，或对数据时序敏感时，先轮询 STATUS 确认数据就绪再读原始值。

```c
#include "bmi3_defs.h"     /* 提供 BMI3_DRDY_xxx_MASK */

struct imu_raw_data raw;
uint16_t status;
uint16_t ready_mask;
int retry;

ready_mask = (uint16_t)(BMI3_DRDY_ACC_MASK | BMI3_DRDY_GYR_MASK | BMI3_DRDY_TEMP_MASK);

/* 轮询等待 acc/gyr/temp 全部就绪，最多约 1 秒 */
for (retry = 0; retry < 50; retry++)
{
    if (imu_read_status(&status) != IMU_SUCCESS)
    {
        break;
    }

    if ((status & ready_mask) == ready_mask)
    {
        if (imu_read_raw(&raw) == IMU_SUCCESS)
        {
            MY_LOG_INF("raw acc: %d, %d, %d", raw.acc_x, raw.acc_y, raw.acc_z);
            MY_LOG_INF("raw gyr: %d, %d, %d", raw.gyr_x, raw.gyr_y, raw.gyr_z);
        }
        break;
    }

    k_msleep(20);
}
```

### Demo 5：运行期重配置 / 切换电源模式

无需重新初始化即可在运行期调整量程、ODR 或电源模式。

```c
struct imu_config cfg;

/* 切换到 ±2g / 200Hz / 普通模式，提升加速度灵敏度 */
cfg.acc_range  = IMU_ACC_RANGE_2G;
cfg.acc_odr    = IMU_ODR_200HZ;
cfg.gyr_range  = IMU_GYR_RANGE_500DPS;
cfg.gyr_odr    = IMU_ODR_200HZ;
cfg.power_mode = IMU_POWER_NORMAL;

if (imu_set_config(&cfg) != IMU_SUCCESS)
{
    MY_LOG_ERR("imu set config failed");
}

/* 仅切换电源模式（保留当前量程/ODR），例如进入低功耗待机 */
imu_set_power_mode(IMU_POWER_LOW_POWER);

/* 完全挂起加速度计与陀螺仪，降至最低功耗 */
imu_set_power_mode(IMU_POWER_SUSPEND);
```

> `imu_set_power_mode` 内部会沿用模块缓存的量程/ODR，只替换电源模式后整体下发，所以无需重复填写其他字段。

### Demo 6：计步器

计步器参数在 `imu_init` 阶段已按可穿戴场景加载，应用层只需使能并周期读取。

```c
uint32_t steps = 0;

/* 1. 使能计步器（会触发 feature engine 重载，累计步数从 0 开始） */
if (imu_step_counter_enable(true) != IMU_SUCCESS)
{
    MY_LOG_ERR("enable step counter failed");
    return;
}

/* 2. 运行期周期读取累计步数 */
if (imu_get_step_count(&steps) == IMU_SUCCESS)
{
    MY_LOG_INF("step count: %u", steps);
}

/* 3. 不再需要时禁用 */
imu_step_counter_enable(false);
```

> 重要：`imu_step_counter_enable(true)` / `imu_feature_enable()` 都会触发 feature engine 重载并**清零累计步数**，因此使能动作应只在初始化阶段执行一次，运行期间只调用 `imu_get_step_count` 读取，不要反复使能。

### Demo 7：特征事件 + 中断（以 ANY_MOTION 唤醒为例）

将“任意运动”事件映射到 INT1，运动发生时通过 GPIO 中断回调通知 MCU。

```c
/* 中断标志，回调里只置位，主循环再处理 */
static volatile bool s_imu_motion_flag = false;

/********************************************************************
**函数名称:  imu_int_cb
**入口参数:  无
**出口参数:  无
**函数功能:  IMU INT1 GPIO 中断回调，仅置位标志
**返回值:    无
*********************************************************************/
static void imu_int_cb(void)
{
    s_imu_motion_flag = true;       // 中断上下文只做轻量置位
}

void imu_any_motion_setup(void)
{
    struct imu_int_config int_cfg;

    /* 1. 使能 any-motion 特征 */
    imu_feature_enable(IMU_FEATURE_ANY_MOTION, true);

    /* 2. 配置 INT1 电气特性：高有效、推挽、锁存 */
    int_cfg.active_high = true;
    int_cfg.open_drain  = false;
    int_cfg.latch       = true;
    imu_int_pin_config(IMU_INT_PIN1, &int_cfg);

    /* 3. 将 any-motion 中断源映射到 INT1 */
    imu_int_map(IMU_INT_SRC_ANY_MOTION, IMU_INT_PIN1);

    /* 4. 注册 GPIO 中断回调 */
    imu_register_int_callback(imu_int_cb);
}

/* 主循环中检测并处理 */
void imu_any_motion_poll(void)
{
    uint16_t int_status;

    if (s_imu_motion_flag == true)
    {
        s_imu_motion_flag = false;

        /* 读取中断状态会同时清除锁存标志 */
        if (imu_read_int_status(IMU_INT_PIN1, &int_status) == IMU_SUCCESS)
        {
            MY_LOG_INF("imu int1 status: 0x%04X", int_status);
        }
    }
}
```

> 当前硬件只接出 INT1，映射到 `IMU_INT_PIN2` 仅写寄存器、不会产生实际 GPIO 中断。
> `latch=true` 时务必在事件后调用 `imu_read_int_status` 清除，否则中断会一直保持。

### Demo 8：FIFO 批量采集

高 ODR 下逐帧轮询开销大，可用 FIFO 累积数据后批量取出。

```c
#define IMU_FIFO_BATCH   16

struct imu_fifo_config fifo_cfg;
struct imu_raw_data frames[IMU_FIFO_BATCH];
uint16_t frame_count = 0;
uint16_t i;

/* 1. 配置 FIFO：仅缓存 acc + gyr，满则覆盖，水印 32 字 */
fifo_cfg.acc_en       = true;
fifo_cfg.gyr_en       = true;
fifo_cfg.temp_en      = false;
fifo_cfg.time_en      = false;
fifo_cfg.stop_on_full = false;
fifo_cfg.watermark    = 32;

if (imu_fifo_config(&fifo_cfg) != IMU_SUCCESS)   /* 配置后会自动 flush 一次 */
{
    MY_LOG_ERR("imu fifo config failed");
    return;
}

/* 2. 等积累一段时间后批量读取 */
k_msleep(200);

if (imu_fifo_read(frames, IMU_FIFO_BATCH, &frame_count) == IMU_SUCCESS)
{
    MY_LOG_INF("fifo got %u frames", frame_count);
    for (i = 0; i < frame_count; i++)
    {
        MY_LOG_INF("[%u] acc:%d,%d,%d gyr:%d,%d,%d",
                   i, frames[i].acc_x, frames[i].acc_y, frames[i].acc_z,
                   frames[i].gyr_x, frames[i].gyr_y, frames[i].gyr_z);
    }
}

/* 3. 需要时手动清空 */
imu_fifo_flush();
```

> `imu_fifo_config` 要求 acc/gyr/temp/time 至少使能一项，否则返回 `IMU_ERROR_PARAM`。
> FIFO 中温度为半速率采样，`imu_fifo_read` 已按比例映射到对应帧，温度帧数约为运动帧的一半。

### Demo 9：轴映射

当传感器在 PCB 上的安装方向与产品坐标系不一致时，用轴映射在硬件侧纠正方向。

```c
struct imu_axis_map axis_map;

/* 读取当前映射 */
if (imu_get_axis_map(&axis_map) == IMU_SUCCESS)
{
    MY_LOG_INF("axis_map=%u, inv=%d,%d,%d",
               axis_map.axis_map, axis_map.invert_x, axis_map.invert_y, axis_map.invert_z);
}

/* 设置映射：axis_map 取值 0~5，并把 Z 轴取反 */
axis_map.axis_map = 0;
axis_map.invert_x = false;
axis_map.invert_y = false;
axis_map.invert_z = true;

if (imu_set_axis_map(&axis_map) != IMU_SUCCESS)
{
    MY_LOG_ERR("imu set axis map failed");
}
```

> `axis_map` 取值范围为 0~5，超出会返回 `IMU_ERROR_PARAM`。

### Demo 10：完整流程串联

```c
void imu_demo(void)
{
    struct imu_config cfg;
    struct imu_data   data;
    uint8_t  id = 0;
    uint32_t steps = 0;

    /* 1. 初始化 */
    cfg.acc_range  = IMU_ACC_RANGE_8G;
    cfg.acc_odr    = IMU_ODR_100HZ;
    cfg.gyr_range  = IMU_GYR_RANGE_2000DPS;
    cfg.gyr_odr    = IMU_ODR_100HZ;
    cfg.power_mode = IMU_POWER_HIGH_PERF;
    if (imu_init(&cfg) != IMU_SUCCESS)
    {
        return;
    }

    /* 2. 自检 */
    imu_get_chip_id(&id);
    MY_LOG_INF("imu chip id: 0x%02X", id);

    /* 3. 使能计步器（仅一次） */
    imu_step_counter_enable(true);

    /* 4. 读取一次运动数据 */
    if (imu_read(&data) == IMU_SUCCESS)
    {
        MY_LOG_INF("acc:%d,%d,%d mg  gyr:%d,%d,%d mdps  temp:%d.%02d C",
                   data.acc_x, data.acc_y, data.acc_z,
                   data.gyr_x, data.gyr_y, data.gyr_z,
                   data.temperature / 100, data.temperature % 100);
    }

    /* 5. 读取步数 */
    imu_get_step_count(&steps);
    MY_LOG_INF("steps: %u", steps);

    /* 6. 进入低功耗 */
    imu_set_power_mode(IMU_POWER_LOW_POWER);
}
```

---

## 五、使用注意事项

1. **必须先初始化**：未调用 `imu_init` 直接读取或配置，相关接口会返回 `IMU_ERROR_INIT`。
2. **芯片 ID 校验**：`imu_init` 内部会校验芯片 ID（期望 `0x0045`），不匹配返回 `IMU_ERROR_CHIP_ID`，通常意味着器件未在位或总线异常。
3. **参数校验前置**：量程/ODR/电源模式等枚举越界、传入空指针，接口在触碰硬件前即返回 `IMU_ERROR_PARAM`。
4. **计步器/特征只在初始化阶段使能一次**：`imu_step_counter_enable(true)` 与 `imu_feature_enable()` 都会触发 feature engine 重载并清零累计步数，运行期间只读取、不要反复使能。
5. **中断硬件限制**：当前板级仅 INT1 接入 MCU GPIO，映射到 `IMU_INT_PIN2` 仅写寄存器、不产生物理中断。
6. **锁存中断需手动清除**：`imu_int_config.latch = true` 时，事件后必须调用 `imu_read_int_status` 读取并清除，否则中断电平会持续保持。
7. **FIFO 至少使能一类数据源**：`imu_fifo_config` 中 acc/gyr/temp/time 全为 false 时返回 `IMU_ERROR_PARAM`；配置成功后会自动 flush 一次。
8. **FIFO 温度为半速率**：FIFO 内温度帧数约为运动帧的一半，`imu_fifo_read` 已自动按比例映射，无需应用层干预。
9. **线程安全**：本模块约定所有 IMU API 在同一线程内串行调用，内部未加锁；如需跨线程访问请自行加锁保护。
10. **数据单位**：`imu_read` 输出加速度为 mg、角速度为 mdps、温度为 0.01°C；`imu_read_raw` 输出未换算的 16-bit 原始 LSB。
