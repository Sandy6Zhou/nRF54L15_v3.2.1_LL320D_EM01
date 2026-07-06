# Accelerometer 三轴加速度传感器使用示例

模块路径：`ext_module/accelerometer`
适用器件：DA213 三轴加速度传感器
头文件：`#include "accelerometer_api.h"`

---

## 一、模块结构

```
accelerometer/
├── api/
│   ├── accelerometer_api.h      统一对外接口（应用层只需包含此头文件）
│   └── accelerometer_api.c
└── drivers/
    └── DA213/                   DA213 底层驱动（应用层无需直接调用）
        ├── inc/da213_driver.h
        └── src/da213_driver.c
```

应用层只与 `accelerometer_api.h` 交互，底层驱动差异由 API 层屏蔽。

---

## 二、数据类型速查

### 返回码 `accel_result_t`

| 值 | 宏 | 含义 |
|----|-----|------|
| 0 | ACCEL_SUCCESS | 成功 |
| 1 | ACCEL_ERROR_INIT | 初始化失败或模块未完成初始化 |
| 2 | ACCEL_ERROR_COMM | 寄存器访问或总线通信失败 |
| 3 | ACCEL_ERROR_CHIP_ID | 芯片 ID 校验失败 |
| 4 | ACCEL_ERROR_PARAM | 输入参数无效 |
| 5 | ACCEL_ERROR_NOT_READY | 预留结果码：模块未就绪 |

### 量程 `accel_range_t`

| 宏 | 说明 | 灵敏度 |
|-----|------|--------|
| ACCEL_RANGE_2G | ±2g | 4096 LSB/g |
| ACCEL_RANGE_4G | ±4g | 2048 LSB/g |
| ACCEL_RANGE_8G | ±8g | 1024 LSB/g |
| ACCEL_RANGE_16G | ±16g | 512 LSB/g |

### 输出数据率 `accel_odr_t`

| 宏 | 频率 | 备注 |
|-----|------|------|
| ACCEL_ODR_1HZ | 1 Hz | 仅低功耗模式 |
| ACCEL_ODR_1_95HZ | 1.95 Hz | 仅低功耗模式 |
| ACCEL_ODR_3_9HZ | 3.9 Hz | |
| ACCEL_ODR_7_81HZ | 7.81 Hz | |
| ACCEL_ODR_15_63HZ | 15.63 Hz | |
| ACCEL_ODR_31_25HZ | 31.25 Hz | |
| ACCEL_ODR_62_5HZ | 62.5 Hz | |
| ACCEL_ODR_125HZ | 125 Hz | |
| ACCEL_ODR_250HZ | 250 Hz | |
| ACCEL_ODR_500HZ | 500 Hz | 仅正常模式 |
| ACCEL_ODR_1000HZ | 1000 Hz | 仅正常模式 |

### 电源模式 `accel_power_mode_t`

| 宏 | 说明 | 典型功耗 |
|-----|------|----------|
| ACCEL_POWER_NORMAL | 正常模式 | ~180 uA |
| ACCEL_POWER_LOW_POWER | 低功耗模式 | ~40 uA |
| ACCEL_POWER_SUSPEND | 挂起模式 | ~1 uA |

### 数据结构

```c
struct accel_data
{
    int32_t x_mg;       /* X轴加速度，单位 mg */
    int32_t y_mg;       /* Y轴加速度，单位 mg */
    int32_t z_mg;       /* Z轴加速度，单位 mg */
};

struct accel_raw_data
{
    int16_t x;          /* X轴原始值 */
    int16_t y;          /* Y轴原始值 */
    int16_t z;          /* Z轴原始值 */
};

struct accel_config
{
    accel_range_t range;            /* 量程 */
    accel_odr_t odr;                /* 输出数据率 */
    accel_power_mode_t power_mode;  /* 电源模式 */
};
```

---

## 三、接口列表

| 接口 | 功能 |
|------|------|
| `accelerometer_init(config)` | 初始化模块并设定量程/ODR/电源模式 |
| `accelerometer_get_chip_id(id)` | 读取芯片 ID（期望 0x13） |
| `accelerometer_set_power_mode(mode)` | 切换电源模式 |
| `accelerometer_set_range(range)` | 运行时修改量程 |
| `accelerometer_set_odr(odr)` | 运行时修改数据率 |
| `accelerometer_read(data)` | 读取三轴加速度（mg 单位） |
| `accelerometer_read_raw(data)` | 读取三轴原始数据 |
| `accelerometer_config_active_int(config)` | 配置运动检测中断 |
| `accelerometer_config_tap_int(config)` | 配置敲击检测中断 |
| `accelerometer_config_freefall_int(config)` | 配置自由落体中断 |
| `accelerometer_config_orient_int(config)` | 配置方向识别中断 |
| `accelerometer_disable_int(type)` | 禁用指定类型中断 |
| `accelerometer_read_int_status(status)` | 读取中断触发状态 |
| `accelerometer_read_orient_status(status)` | 读取方向识别状态 |
| `accelerometer_reset_int()` | 复位所有锁存中断 |
| `accelerometer_register_int_callback(cb)` | 注册加速度计中断回调 |

---

## 四、使用示例

### Demo 1：初始化模块

初始化是使用任何功能前的必要步骤，量程、ODR、电源模式在此一次性设定。

```c
#include "accelerometer_api.h"

struct accel_config accel_cfg;
accel_result_t ret;

accel_cfg.range = ACCEL_RANGE_4G;           /* ±4g */
accel_cfg.odr = ACCEL_ODR_125HZ;            /* 125Hz */
accel_cfg.power_mode = ACCEL_POWER_NORMAL;  /* 正常模式 */

ret = accelerometer_init(&accel_cfg);
if (ret != ACCEL_SUCCESS)
{
    /* ret=3 芯片ID不匹配, ret=2 通信失败, ret=1 其他初始化失败 */
    MY_LOG_ERR("accelerometer init failed: %d", ret);
    return;
}
```

### Demo 2：读取芯片 ID

常用于上电检查，确认器件在位且 I2C 通信正常。

```c
uint8_t chip_id = 0;

if (accelerometer_get_chip_id(&chip_id) == ACCEL_SUCCESS)
{
    MY_LOG_INF("chip id: 0x%02X", chip_id);  /* 正常应为 0x13 */
}
```

### Demo 3：读取三轴加速度（mg 单位）

API 层自动将原始数据按当前量程转换为 mg 单位。

```c
struct accel_data data;

if (accelerometer_read(&data) == ACCEL_SUCCESS)
{
    MY_LOG_INF("X: %d mg, Y: %d mg, Z: %d mg", data.x_mg, data.y_mg, data.z_mg);
}
```

### Demo 4：读取原始数据

适合需要自行处理原始 ADC 值的场景。

```c
struct accel_raw_data raw;

if (accelerometer_read_raw(&raw) == ACCEL_SUCCESS)
{
    MY_LOG_INF("raw X:%d, Y:%d, Z:%d", raw.x, raw.y, raw.z);
}
```

### Demo 5：配置运动检测（Active）中断

设备静止时触发中断，用于检测移动。

```c
struct accel_active_int_config active_cfg;

active_cfg.threshold = 0x14;    /* ±4g下 LSB=7.81mg, 0x14*7.81=156mg */
active_cfg.duration = 0x02;     /* 持续时间=(2+1)=3ms */
active_cfg.enable_x = true;
active_cfg.enable_y = true;
active_cfg.enable_z = true;

if (accelerometer_config_active_int(&active_cfg) == ACCEL_SUCCESS)
{
    MY_LOG_INF("Active interrupt configured");
}
```

### Demo 6：配置敲击检测（Tap）中断

检测单击/双击事件。

```c
struct accel_tap_int_config tap_cfg;

tap_cfg.threshold = 0x0E;       /* ±4g下 LSB=125mg, 0x0E*125=1750mg */
tap_cfg.quiet = 0;              /* 静默时间 30ms */
tap_cfg.shock = 0;              /* 冲击时间 50ms */
tap_cfg.duration = 4;           /* 双击窗口 250ms */
tap_cfg.enable_single = true;
tap_cfg.enable_double = true;

if (accelerometer_config_tap_int(&tap_cfg) == ACCEL_SUCCESS)
{
    MY_LOG_INF("Tap interrupt configured");
}
```

### Demo 7：配置自由落体中断

检测设备自由落体状态。

```c
struct accel_freefall_int_config ff_cfg;

ff_cfg.threshold = 0x30;        /* 阈值=0x30*7.81mg=375mg */
ff_cfg.duration = 0x09;         /* 持续时间=(9+1)*2ms=20ms */
ff_cfg.hysteresis = 0x01;       /* 迟滞=1*125mg=125mg */
ff_cfg.sum_mode = false;        /* 单轴模式 */

if (accelerometer_config_freefall_int(&ff_cfg) == ACCEL_SUCCESS)
{
    MY_LOG_INF("Freefall interrupt configured");
}
```

### Demo 8：配置方向识别中断

检测设备横竖屏翻转。

```c
struct accel_orient_int_config orient_cfg;

orient_cfg.mode = 0;            /* 对称模式 */
orient_cfg.blocking = 1;        /* Z轴阻塞 */
orient_cfg.hysteresis = 3;      /* 迟滞=3*62.5mg=187.5mg */
orient_cfg.z_blocking = 8;      /* Z阻塞=8*62.5mg=500mg */

if (accelerometer_config_orient_int(&orient_cfg) == ACCEL_SUCCESS)
{
    MY_LOG_INF("Orient interrupt configured");
}
```

### Demo 9：读取中断状态并清除

中断触发后，读取状态判断来源，然后复位锁存。

```c
struct accel_int_status status;

if (accelerometer_read_int_status(&status) == ACCEL_SUCCESS)
{
    if (status.active)      MY_LOG_INF("Active!");
    if (status.single_tap)  MY_LOG_INF("Single tap!");
    if (status.double_tap)  MY_LOG_INF("Double tap!");
    if (status.freefall)    MY_LOG_INF("Freefall!");
    if (status.orient)      MY_LOG_INF("Orientation changed!");
}

accelerometer_reset_int();
```

### Demo 10：注册中断回调

中断回调运行在 GPIO 中断上下文中，建议仅做事件投递，不要直接在回调里访问 I2C。

```c
#include <zephyr/kernel.h>

static struct k_work s_accel_int_work;

static void accel_int_work_handler(struct k_work *work)
{
    struct accel_int_status status;

    ARG_UNUSED(work);

    if (accelerometer_read_int_status(&status) != ACCEL_SUCCESS)
    {
        return;
    }

    if (status.active)
    {
        MY_LOG_INF("Active interrupt triggered");
    }

    if (status.single_tap)
    {
        MY_LOG_INF("Single tap detected");
    }

    if (status.double_tap)
    {
        MY_LOG_INF("Double tap detected");
    }

    if (status.freefall)
    {
        MY_LOG_INF("Freefall detected");
    }

    if (status.orient)
    {
        MY_LOG_INF("Orientation changed");
    }
}

static void accel_int_callback(void)
{
    k_work_submit(&s_accel_int_work);
}

void accelerometer_int_demo_init(void)
{
    k_work_init(&s_accel_int_work, accel_int_work_handler);
    accelerometer_register_int_callback(accel_int_callback);
}
```

### Demo 11：方向中断回调中读取朝向状态

若当前只关心方向识别中断，可在工作队列中进一步读取 `orient_xy` 和 `orient_z`。

```c
struct accel_orient_status orient;
const char *xy_str[] = {"Portrait Up", "Portrait Down", "Landscape Left", "Landscape Right"};
const char *z_str[] = {"Face Up", "Face Down"};

if (accelerometer_read_orient_status(&orient) == ACCEL_SUCCESS)
{
    MY_LOG_INF("Orientation: %s, %s", xy_str[orient.orient_xy], z_str[orient.orient_z]);
}
```

### Demo 12：电源模式切换

```c
/* 进入挂起模式（约1uA功耗） */
accelerometer_set_power_mode(ACCEL_POWER_SUSPEND);

/* 恢复正常模式 */
accelerometer_set_power_mode(ACCEL_POWER_NORMAL);
```

### Demo 13：完整流程串联

```c
void accelerometer_demo(void)
{
    struct accel_config cfg = { ACCEL_RANGE_4G, ACCEL_ODR_125HZ, ACCEL_POWER_NORMAL };
    struct accel_data data;
    uint8_t id = 0;
    int i;

    /* 1. 初始化 */
    if (accelerometer_init(&cfg) != ACCEL_SUCCESS)
    {
        return;
    }

    /* 2. 读取芯片 ID */
    accelerometer_get_chip_id(&id);
    MY_LOG_INF("chip id: 0x%02X", id);

    /* 3. 连续读取10次 */
    for (i = 0; i < 10; i++)
    {
        if (accelerometer_read(&data) == ACCEL_SUCCESS)
        {
            MY_LOG_INF("[%d] X:%d Y:%d Z:%d mg", i, data.x_mg, data.y_mg, data.z_mg);
        }
        k_msleep(100);
    }

    /* 4. 配置运动检测中断后进入低功耗 */
    struct accel_active_int_config active = { .threshold = 0x14, .duration = 2,
                                              .enable_x = true, .enable_y = true, .enable_z = true };
    k_work_init(&s_accel_int_work, accel_int_work_handler);
    accelerometer_register_int_callback(accel_int_callback);
    accelerometer_config_active_int(&active);
    accelerometer_set_power_mode(ACCEL_POWER_LOW_POWER);
}
```

---

## 五、CMakeLists.txt 集成

在顶层 `CMakeLists.txt` 中添加：

```cmake
target_include_directories(app PRIVATE
  ./ext_module/accelerometer/api
  ./ext_module/accelerometer/drivers/DA213/inc
)

target_sources(app PRIVATE
  ext_module/accelerometer/api/accelerometer_api.c
  ext_module/accelerometer/drivers/DA213/src/da213_driver.c
)
```

---

## 六、使用注意事项

1. **必须先初始化**：未调用 `accelerometer_init` 直接读取或配置中断，会返回 `ACCEL_ERROR_INIT`。
2. **重复初始化**：模块已初始化时再次调用会直接返回成功并打印警告，不会重复执行。
3. **量程与阈值关系**：Active/Tap 中断的阈值 LSB 取决于当前量程，切换量程后需重新配置中断参数。
4. **ODR 与电源模式**：1Hz/1.95Hz 仅低功耗模式可用；500Hz/1000Hz 仅正常模式可用。
5. **中断引脚**：所有中断默认映射到 INT1（对应 overlay 中 `gsensor-int` P1.2），需在 GPIO 中断回调中读取中断状态。
6. **中断回调上下文**：回调函数内只提交 `k_work`，真正的寄存器读取在线程上下文完成，避免在 GPIO 中断中直接访问 I2C。
7. **挂起模式**：挂起模式下无法采集数据，仅支持寄存器读写，需先切回 NORMAL 或 LOW_POWER 才能读取。
