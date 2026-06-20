# Barometer 气压模块使用示例

模块路径：`ext_module/barometer`
适用器件：SPA06-003 气压传感器
头文件：`#include "barometer_api.h"`

---

## 一、模块结构

```
barometer/
├── api/
│   ├── barometer_api.h      统一对外接口（应用层只需包含此头文件）
│   └── barometer_api.c
└── drivers/
    └── SPA06/               SPA06-003 底层驱动（应用层无需直接调用）
        ├── inc/spa06_driver.h
        └── src/spa06_driver.c
```

应用层只与 `barometer_api.h` 交互，底层驱动差异由 API 层屏蔽。

---

## 二、数据类型速查

### 返回码 `barometer_result_t`

| 值 | 宏 | 含义 |
|----|-----|------|
| 0 | BARO_SUCCESS | 成功 |
| 1 | BARO_ERROR_INIT | 初始化错误 |
| 2 | BARO_ERROR_COMM | 通信错误 |
| 3 | BARO_ERROR_CHIP_ID | 芯片 ID 不匹配 |
| 4 | BARO_ERROR_PARAM | 参数错误 |
| 5 | BARO_ERROR_TIMEOUT | 测量超时 |

### 工作模式 `barometer_work_mode_t`

| 宏 | 说明 |
|-----|------|
| BARO_MODE_STOP | 停止测量 |
| BARO_MODE_SINGLE_TEMPERATURE | 单次温度 |
| BARO_MODE_SINGLE_PRESSURE | 单次气压 |
| BARO_MODE_SINGLE_BOTH | 单次气压+温度 |
| BARO_MODE_CONTINUOUS_TEMPERATURE | 连续温度 |
| BARO_MODE_CONTINUOUS_PRESSURE | 连续气压 |
| BARO_MODE_CONTINUOUS_BOTH | 连续气压+温度 |
| BARO_MODE_MAX | 枚举边界，仅用于校验，不可作为工作模式 |

### 数据结构

```c
struct barometer_data
{
    int32_t pressure_pa;    /* 气压值，单位 Pa，如 101325 = 标准大气压 */
    int16_t temperature;    /* 温度值，单位 0.01°C，如 2550 = 25.50°C */
};

struct barometer_config
{
    uint8_t pressure_sample_rate_hz;    /* 气压采样率，Hz */
    uint8_t pressure_oversampling;      /* 气压过采样倍率 */
    uint8_t temperature_sample_rate_hz; /* 温度采样率，Hz */
    uint8_t temperature_oversampling;   /* 温度过采样倍率 */
};
```

> 数值换算：摄氏度 = `temperature / 100.0`，百帕(hPa) = `pressure_pa / 100.0`。

---

## 三、接口列表

| 接口 | 功能 |
|------|------|
| `barometer_init(const struct barometer_config *config)` | 初始化模块并设定采样配置 |
| `barometer_get_chip_id(uint8_t *id)` | 读取芯片 ID（期望 0x11） |
| `barometer_set_work_mode(barometer_work_mode_t work_mode)` | 设置工作模式 |
| `barometer_read(struct barometer_data *data)` | 按当前模式读取一次测量数据 |

---

## 四、使用示例

### Demo 1：初始化模块

初始化是使用任何功能前的必要步骤，采样率与过采样在此一次性设定，运行期不再变更。

```c
#include "barometer_api.h"

struct barometer_config baro_config;
barometer_result_t ret;

/* 气压/温度均设为 32Hz 采样率、8 倍过采样（精度与功耗的折中） */
baro_config.pressure_sample_rate_hz    = 32;
baro_config.pressure_oversampling      = 8;
baro_config.temperature_sample_rate_hz = 32;
baro_config.temperature_oversampling   = 8;

ret = barometer_init(&baro_config);
if (ret != BARO_SUCCESS)
{
    /* ret=3 芯片ID不匹配，ret=5 超时，ret=1 其他初始化失败 */
    MY_LOG_ERR("barometer init failed: %d", ret);
    return;
}
```

> 过采样倍率可选 1/2/4/8/16/32/64/128，倍率越高精度越高、单次测量耗时越长。

### Demo 2：读取芯片 ID（自检）

常用于上电自检，确认器件在位且 I2C 通信正常。

```c
uint8_t chip_id = 0;

if (barometer_get_chip_id(&chip_id) == BARO_SUCCESS)
{
    MY_LOG_INF("chip id: 0x%02X", chip_id);  /* 正常应为 0x11 */
}
```

### Demo 3：单次气压+温度测量

适合低功耗场景：需要数据时触发一次，测完自动回到待机。

```c
struct barometer_data data;

if (barometer_set_work_mode(BARO_MODE_SINGLE_BOTH) == BARO_SUCCESS)
{
    if (barometer_read(&data) == BARO_SUCCESS)
    {
        MY_LOG_INF("pressure: %d Pa (%d.%02d hPa)",
                   data.pressure_pa, data.pressure_pa / 100, data.pressure_pa % 100);
        MY_LOG_INF("temperature: %d.%02d C",
                   data.temperature / 100, data.temperature % 100);
    }
}
```

### Demo 4：只测温度 / 只测气压

按需选择单测模式，省去不需要的测量耗时。

```c
struct barometer_data data;

/* 只测温度 */
if (barometer_set_work_mode(BARO_MODE_SINGLE_TEMPERATURE) == BARO_SUCCESS)
{
    barometer_read(&data);
    MY_LOG_INF("temp: %d.%02d C", data.temperature / 100, data.temperature % 100);
}

/* 只测气压 */
if (barometer_set_work_mode(BARO_MODE_SINGLE_PRESSURE) == BARO_SUCCESS)
{
    barometer_read(&data);
    MY_LOG_INF("pres: %d Pa", data.pressure_pa);
}
```

### Demo 5：连续模式周期采集

适合需要持续监测的场景：设置一次连续模式后，反复调用 `barometer_read` 即可。

```c
struct barometer_data data;
int i;

if (barometer_set_work_mode(BARO_MODE_CONTINUOUS_BOTH) == BARO_SUCCESS)
{
    for (i = 0; i < 10; i++)
    {
        if (barometer_read(&data) == BARO_SUCCESS)
        {
            MY_LOG_INF("[%d] p:%d Pa, t:%d.%02d C",
                       i, data.pressure_pa, data.temperature / 100, data.temperature % 100);
        }
        k_msleep(100);  /* 按需调整采集间隔 */
    }

    /* 不再需要时停止，降低功耗 */
    barometer_set_work_mode(BARO_MODE_STOP);
}
```

> 连续气压模式下，驱动会按固定间隔自动重测温度以补偿温漂，无需应用层干预。

### Demo 6：完整流程串联

```c
void barometer_demo(void)
{
    struct barometer_config cfg = { 32, 8, 32, 8 };
    struct barometer_data   data;
    uint8_t id = 0;

    /* 1. 初始化 */
    if (barometer_init(&cfg) != BARO_SUCCESS)
    {
        return;
    }

    /* 2. 自检 */
    barometer_get_chip_id(&id);
    MY_LOG_INF("chip id: 0x%02X", id);

    /* 3. 单次采集 */
    if (barometer_set_work_mode(BARO_MODE_SINGLE_BOTH) == BARO_SUCCESS)
    {
        if (barometer_read(&data) == BARO_SUCCESS)
        {
            MY_LOG_INF("p:%d Pa, t:%d.%02d C",
                       data.pressure_pa, data.temperature / 100, data.temperature % 100);
        }
    }

    /* 4. 停止 */
    barometer_set_work_mode(BARO_MODE_STOP);
}
```

---

## 五、使用注意事项

1. **必须先初始化**：未调用 `barometer_init` 直接设置模式或读取，会返回 `BARO_ERROR_INIT`。
2. **重复初始化**：模块已初始化时再次调用 `barometer_init` 会直接返回成功并打印警告，不会重复执行。
3. **先设模式再读取**：`barometer_read` 依据当前工作模式决定测量内容；STOP 模式下读取无意义。
4. **非法模式拦截**：传入 `>= BARO_MODE_MAX` 的越界值会返回 `BARO_ERROR_PARAM`。
5. **单次 vs 连续**：单次模式省电、每次按需触发；连续模式吞吐高、适合持续监测，用完记得切回 STOP。
6. **采样配置时机**：采样率/过采样仅在 `barometer_init` 时设定，运行期不支持动态修改。
