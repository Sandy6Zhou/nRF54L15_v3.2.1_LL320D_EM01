# Battery Gauge 模块使用示例

模块路径：`ext_module/battery_gauge`

适用器件：OM70201WV 电流型电量计

头文件：`#include "battery_gauge_api.h"`

---

## 一、模块结构

```text
battery_gauge/
├── api/
│   ├── battery_gauge_api.h    统一对外接口
│   ├── battery_gauge_api.c
│   └── battery_gauge_reg.h    API 层私有寄存器定义
├── port/
│   ├── om70201wv_port.h       Zephyr I2C、GPIO 和延时适配
│   ├── om70201wv_port.c
│   └── omg_impl.h             厂家驱动接口适配
└── vendor/
    ├── omg_battery.h          厂家驱动配置和接口
    └── omg_battery.c          厂家驱动及电池 Profile
```

应用层只包含 `battery_gauge_api.h`，不要直接访问 `port`、`vendor` 或私有寄存器。

当前工程硬件和驱动约定：

- I2C 控制器：`i2c21`；
- OM70201WV 7 位地址：`0x38`；
- INTN：`P0.02`，开漏、低电平有效；
- 电流采样：低边采样，采样电阻 `10 mΩ`；
- 当前温度来源：OM70201WV 内部温度传感器；
- 芯片 ID：`0xB1`。

电池 Profile、采样电阻、温度来源和 SOH 策略由 `vendor/omg_battery.c` 与
`vendor/omg_battery.h` 决定。更换电芯、采样电阻或温度方案前，必须重新确认这些配置。

---

## 二、返回码

| 返回码 | 含义 |
|--------|------|
| `BATTERY_GAUGE_SUCCESS` | 操作成功 |
| `BATTERY_GAUGE_ERROR_INIT` | 未初始化、端口初始化失败或缺少 Profile |
| `BATTERY_GAUGE_ERROR_COMM` | I2C 或端口通信失败 |
| `BATTERY_GAUGE_ERROR_CHIP_ID` | 芯片 ID 不匹配 |
| `BATTERY_GAUGE_ERROR_PARAM` | 空指针、枚举越界或阈值非法 |
| `BATTERY_GAUGE_ERROR_TIMEOUT` | 操作超时 |
| `BATTERY_GAUGE_ERROR_UNKNOWN` | 未归类错误 |

---

## 三、数据类型

### 3.1 完整采样数据

```c
typedef struct
{
    uint16_t voltage_mv;
    int16_t current_ma;
    int8_t temperature_c;
    uint8_t soc_percent;
    uint8_t soh_percent;
    uint16_t cycle_count;
} battery_gauge_data_t;
```

- `voltage_mv`：电池电压，单位 `mV`；
- `current_ma`：有符号电流，单位 `mA`；
- `temperature_c`：电池温度，单位 `℃`；
- `soc_percent`：剩余电量，范围 `0~100%`；
- `soh_percent`：健康度，范围 `0~100%`；
- `cycle_count`：电池循环次数。

> 电流符号保持芯片原始方向，必须结合当前板级 CSP/CSN 连接和充放电实测统一业务定义。

### 3.2 中断配置

```c
typedef struct
{
    bool soc_enable;
    bool high_temperature_enable;
    bool low_temperature_enable;
    uint8_t soc_threshold_percent;
    int8_t high_temperature_c;
    int8_t low_temperature_c;
} battery_gauge_interrupt_config_t;
```

- SOC 阈值范围：`0~100`；
- 温度阈值范围：`-40~85 ℃`；
- 必须满足 `low_temperature_c <= high_temperature_c`；
- SOC 阈值设为 `100` 时，SOC 每变化 `1%` 可产生一次中断。

---

## 四、接口列表

| 接口 | 功能与限制 |
|------|------------|
| `battery_gauge_init(const uint16_t *cycle_count_raw)` | 初始化端口和厂家驱动；参数为 `1/32` 次单位的原始循环计数，`NULL` 表示不恢复 |
| `battery_gauge_get_chip_id(uint8_t *chip_id)` | 读取并校验芯片 ID |
| `battery_gauge_read(battery_gauge_data_t *data)` | 按推荐顺序读取完整数据，并根据循环次数同步当前 Host SOH |
| `battery_gauge_get_voltage()`、`battery_gauge_get_current()`、<br />`battery_gauge_get_temperature()`、`battery_gauge_get_soc()` | 分别读取单项量测数据 |
| `battery_gauge_get_soh(uint8_t *soh_percent)` | 读取当前 SOH；调用前会读取循环次数以更新 Host SOH |
| `battery_gauge_get_cycle_count(uint16_t *cycle_count)` | 读取完整循环次数，单位为次；会同步 Host SOH |
| `battery_gauge_get_cycle_count_raw(uint16_t *cycle_count_raw)` | 读取原始循环计数，单位为 `1/32` 次；用于无损持久化 |
| `battery_gauge_set_cycle_count(uint16_t cycle_count)` | 初始化完成后设置完整循环次数（`0~1000`），触发循环计数初始化并更新 SOH；不访问 ZMS |
| `battery_gauge_set_work_mode(battery_gauge_work_mode_t work_mode)` | 设置睡眠或正常工作模式 |
| `battery_gauge_interrupt_config(const battery_gauge_interrupt_config_t *config)` | 配置 SOC、高温和低温中断阈值及使能 |
| `battery_gauge_interrupt_get_status(battery_gauge_interrupt_status_t *status)` | 读取中断状态位 |
| `battery_gauge_register_interrupt_callback(battery_gauge_interrupt_callback_t callback)` | 注册 INTN GPIO 中断回调；回调运行在中断上下文 |

---

## 五、使用示例

### Demo 1：初始化和芯片自检

初始化应在调用其他接口前完成。业务层先读取持久化数据，读取成功时将原始循环计数地址传给 `battery_gauge_init()`；无有效数据时传 `NULL`。重复调用会直接返回成功。

```c
#include "battery_gauge_api.h"
#include "my_zms_param.h"

/********************************************************************
**函数名称:  app_battery_gauge_init
**入口参数:  无
**出口参数:  无
**函数功能:  初始化电量计并检查 OM70201WV 芯片 ID
**返回值:    0 表示成功，负值表示失败
*********************************************************************/
int app_battery_gauge_init(void)
{
    int ret;
    battery_gauge_result_t result;
    uint8_t chip_id;
    uint16_t cycle_count_raw;

    chip_id = 0U;
    cycle_count_raw = 0U;
    ret = my_battery_gauge_state_load(&cycle_count_raw);
    if (ret == 0)
    {
        result = battery_gauge_init(&cycle_count_raw);
    }
    else
    {
        result = battery_gauge_init(NULL);
    }

    if (result != BATTERY_GAUGE_SUCCESS)
    {
        return -1;
    }

    result = battery_gauge_get_chip_id(&chip_id);
    if ((result != BATTERY_GAUGE_SUCCESS) || (chip_id != 0xB1U))
    {
        return -1;
    }

    return 0;
}
```

### Demo 2：读取全部电池数据

优先使用 `battery_gauge_read()`。该接口先读取电压、电流、温度和 SOC，再读取循环次数、同步 host SOH，最后返回更新后的 SOH。

```c
#include "battery_gauge_api.h"

/********************************************************************
**函数名称:  app_battery_gauge_sample
**入口参数:  无
**出口参数:  无
**函数功能:  读取并处理一次完整电池数据
**返回值:    0 表示成功，负值表示读取失败
*********************************************************************/
int app_battery_gauge_sample(void)
{
    battery_gauge_result_t result;
    battery_gauge_data_t data;

    memset(&data, 0, sizeof(data));
    result = battery_gauge_read(&data);
    if (result != BATTERY_GAUGE_SUCCESS)
    {
        return -1;
    }

    MY_LOG_INF("battery: %u mV, %d mA, %d C, SOC=%u%%, SOH=%u%%, cycle=%u",
               data.voltage_mv,
               data.current_ma,
               data.temperature_c,
               data.soc_percent,
               data.soh_percent,
               data.cycle_count);

    return 0;
}
```

正常模式下量测值约每 `1.3 s` 更新一次。若业务需要观察动态变化，建议采样周期不小于 `1500 ms`。

电量计 API 不访问 ZMS，所有读取接口也不会保存参数。业务层应根据掉电策略调用 `battery_gauge_get_cycle_count_raw()` 获取原始值，并在确实发生变化时调用 `my_battery_gauge_state_save()`。重新初始化时，由业务层加载原始计数并传给 `battery_gauge_init()`。

`battery_gauge_set_cycle_count()` 的参数单位为完整循环次数，且只能在
`battery_gauge_init()` 成功后调用；它会修改芯片循环次数和 SOH，但是否持久化仍由调用方决定。

### Demo 3：读取单项数据

仅需要某个数据时，可调用对应单项接口。

```c
#include "battery_gauge_api.h"

/********************************************************************
**函数名称:  app_battery_gauge_get_soc
**入口参数:  无
**出口参数:  无
**函数功能:  读取并上报电池 SOC
**返回值:    0 表示成功，负值表示读取失败
*********************************************************************/
int app_battery_gauge_get_soc(void)
{
    battery_gauge_result_t result;
    uint8_t soc_percent;

    soc_percent = 0U;
    result = battery_gauge_get_soc(&soc_percent);
    if (result != BATTERY_GAUGE_SUCCESS)
    {
        return -1;
    }

    MY_LOG_INF("battery soc=%u%%", soc_percent);

    return 0;
}
```

如果业务在同一周期内需要 SOC 和其他数据，仍建议使用 `battery_gauge_read()`，避免打乱推荐读取顺序。

### Demo 4：循环计数恢复、保存和生产设置

OM70201WV 掉电后会丢失循环计数和 SOH。保存时必须使用原始循环计数，以保留 `1/32` 次精度；初始化时将该原始值传给 `battery_gauge_init()`。

```c
#include "battery_gauge_api.h"
#include "my_zms_param.h"

/********************************************************************
**函数名称:  app_battery_gauge_save_cycle_count
**入口参数:  无
**出口参数:  无
**函数功能:  读取并保存电池循环计数原始值
**返回值:    0 表示成功，负值表示失败
*********************************************************************/
int app_battery_gauge_save_cycle_count(void)
{
    battery_gauge_result_t result;
    uint16_t cycle_count_raw;

    cycle_count_raw = 0U;
    result = battery_gauge_get_cycle_count_raw(&cycle_count_raw);
    if (result != BATTERY_GAUGE_SUCCESS)
    {
        return -1;
    }

    return my_battery_gauge_state_save(cycle_count_raw);
}

/********************************************************************
**函数名称:  app_battery_gauge_set_cycle_count
**入口参数:  cycle_count ---        需要设置的完整循环次数（输入）
**出口参数:  无
**函数功能:  设置芯片循环次数并保存对应原始值
**返回值:    0 表示成功，负值表示失败
*********************************************************************/
int app_battery_gauge_set_cycle_count(uint16_t cycle_count)
{
    battery_gauge_result_t result;
    uint16_t cycle_count_raw;

    result = battery_gauge_set_cycle_count(cycle_count);
    if (result != BATTERY_GAUGE_SUCCESS)
    {
        return -1;
    }

    cycle_count_raw = cycle_count * 32U;

    return my_battery_gauge_state_save(cycle_count_raw);
}
```

`battery_gauge_set_cycle_count()` 仅接受完整循环次数，会丢失原始值中不足一次的部分；因此它适合生产设置、测试或完整次数恢复。上电恢复应优先使用 `battery_gauge_init(&cycle_count_raw)`。

#### 预留功能：ZMS 循环计数持久化实现

本节代码为 OM70201WV 循环计数掉电保存的**预留实现**，不属于
`battery_gauge` 模块的必选功能。当前工程可通过 `my_zms_param` 使用该实现；后续若从
业务代码移除 `om70201wv_state_t`、`ZMS_ID_BATTERY_GAUGE_STATE`、
`my_battery_gauge_state_load()` 和 `my_battery_gauge_state_save()`，可保留本节作为恢复该
存储功能的参考。

该实现依赖已初始化的 ZMS 存储，以及现有的 `FLAG_VALID`、
`my_user_data_read()` 和 `my_user_data_write()` 通用接口。状态中的
`cycle_count_raw` 单位为 `1/32` 次，必须使用原始值保存，避免损失循环次数的小数精度。

在 `my_zms_id_t` 的末尾预留一个独立的 ZMS ID：

```c
    ZMS_ID_BATTERY_GAUGE_STATE,        // OM70201WV 循环次数状态ID
```

在实现文件中定义保存状态结构体：

```c
typedef struct
{
    uint8_t flag;
    uint16_t cycle_count_raw;
} om70201wv_state_t;
```

读取和保存函数如下：

```c
/********************************************************************
**函数名称:  my_battery_gauge_state_load
**入口参数:  cycle_count_raw ---        原始循环计数缓冲区（输入）
**出口参数:  cycle_count_raw ---        从 ZMS 恢复的 1/32 次原始计数（输出）
**函数功能:  加载并检查 OM70201WV 掉电保存状态有效标志
**返回值:    0 表示成功，负值表示无有效数据或读取失败
*********************************************************************/
int my_battery_gauge_state_load(uint16_t *cycle_count_raw)
{
    int ret;
    om70201wv_state_t state;

    if (cycle_count_raw == NULL)
    {
        return -EINVAL;
    }

    memset(&state, 0, sizeof(state));
    ret = my_user_data_read(ZMS_ID_BATTERY_GAUGE_STATE, &state, sizeof(state));
    if (ret != (int)sizeof(state))
    {
        return (ret < 0) ? ret : -ENOENT;
    }

    if (state.flag != FLAG_VALID)
    {
        return -EINVAL;
    }

    *cycle_count_raw = state.cycle_count_raw;

    return 0;
}

/********************************************************************
**函数名称:  my_battery_gauge_state_save
**入口参数:  cycle_count_raw ---        需要保存的 1/32 次原始计数（输入）
**出口参数:  无
**函数功能:  保存带有效标志的 OM70201WV 掉电状态
**返回值:    0 表示成功，负值表示写入失败
*********************************************************************/
int my_battery_gauge_state_save(uint16_t cycle_count_raw)
{
    int ret;
    om70201wv_state_t state;

    memset(&state, 0, sizeof(state));
    state.flag = FLAG_VALID;
    state.cycle_count_raw = cycle_count_raw;

    ret = my_user_data_write(ZMS_ID_BATTERY_GAUGE_STATE, &state, sizeof(state));
    if (ret != (int)sizeof(state))
    {
        return (ret < 0) ? ret : -EIO;
    }

    return 0;
}
```

对应头文件仅需保留以下声明：

```c
int my_battery_gauge_state_load(uint16_t *cycle_count_raw);
int my_battery_gauge_state_save(uint16_t cycle_count_raw);
```

### Demo 5：睡眠和唤醒

睡眠模式下芯片进入低功耗状态，所有量测值停止更新。恢复正常模式后，应等待至少一个量测更新周期再使用新数据。

```c
#include "battery_gauge_api.h"
#include <zephyr/kernel.h>

/********************************************************************
**函数名称:  app_battery_gauge_sleep
**入口参数:  无
**出口参数:  无
**函数功能:  设置电量计进入睡眠模式
**返回值:    0 表示成功，负值表示设置失败
*********************************************************************/
int app_battery_gauge_sleep(void)
{
    battery_gauge_result_t result;

    result = battery_gauge_set_work_mode(BATTERY_GAUGE_WORK_MODE_SLEEP);

    return (result == BATTERY_GAUGE_SUCCESS) ? 0 : -1;
}

/********************************************************************
**函数名称:  app_battery_gauge_wakeup
**入口参数:  无
**出口参数:  无
**函数功能:  唤醒电量计并等待量测数据更新
**返回值:    0 表示成功，负值表示设置失败
*********************************************************************/
int app_battery_gauge_wakeup(void)
{
    battery_gauge_result_t result;

    result = battery_gauge_set_work_mode(BATTERY_GAUGE_WORK_MODE_NORMAL);
    if (result != BATTERY_GAUGE_SUCCESS)
    {
        return -1;
    }

    k_msleep(1500U);

    return 0;
}
```

芯片睡眠不会丢失寄存器配置。若 OM70201WV 掉电且 MCU 也重启，启动流程会重新调用 `battery_gauge_init()` 并加载 Profile。若只有 OM70201WV 掉电、MCU 未复位，当前 API 没有反初始化接口，不能依赖重复调用 `battery_gauge_init()` 恢复芯片状态，应在系统设计中保证二者同步复位或重启 MCU。

### Demo 6：配置 SOC 和温度中断

```c
#include "battery_gauge_api.h"

/********************************************************************
**函数名称:  app_battery_gauge_interrupt_init
**入口参数:  无
**出口参数:  无
**函数功能:  配置电量、过温和低温告警阈值
**返回值:    0 表示成功，负值表示配置失败
*********************************************************************/
int app_battery_gauge_interrupt_init(void)
{
    battery_gauge_result_t result;
    battery_gauge_interrupt_config_t config;

    memset(&config, 0, sizeof(config));
    config.soc_enable = true;
    config.soc_threshold_percent = 20U;
    config.high_temperature_enable = true;
    config.high_temperature_c = 60;
    config.low_temperature_enable = true;
    config.low_temperature_c = -10;

    result = battery_gauge_interrupt_config(&config);

    return (result == BATTERY_GAUGE_SUCCESS) ? 0 : -1;
}
```

### Demo 7：注册中断回调并通过工作队列处理

GPIO 回调运行在中断上下文，只设置标志或提交工作项，禁止直接调用 I2C 接口。

```c
#include "battery_gauge_api.h"
#include <zephyr/kernel.h>

static struct k_work s_battery_gauge_interrupt_work;

/********************************************************************
**函数名称:  app_battery_gauge_interrupt_work_handler
**入口参数:  work ---        中断工作项指针（输入）
**出口参数:  无
**函数功能:  在工作队列上下文读取并处理电量计中断状态
**返回值:    无
*********************************************************************/
static void app_battery_gauge_interrupt_work_handler(struct k_work *work)
{
    battery_gauge_result_t result;
    battery_gauge_interrupt_status_t status;

    status = BATTERY_GAUGE_INTERRUPT_NONE;
    result = battery_gauge_interrupt_get_status(&status);
    if (result != BATTERY_GAUGE_SUCCESS)
    {
        return;
    }

    if ((status & BATTERY_GAUGE_INTERRUPT_SOC) != 0U)
    {
        MY_LOG_WRN("battery soc interrupt");
    }

    if ((status & BATTERY_GAUGE_INTERRUPT_HIGH_TEMPERATURE) != 0U)
    {
        MY_LOG_WRN("battery high temperature interrupt");
    }

    if ((status & BATTERY_GAUGE_INTERRUPT_LOW_TEMPERATURE) != 0U)
    {
        MY_LOG_WRN("battery low temperature interrupt");
    }
}

/********************************************************************
**函数名称:  app_battery_gauge_interrupt_callback
**入口参数:  无
**出口参数:  无
**函数功能:  在 GPIO 中断上下文提交电量计中断工作项
**返回值:    无
*********************************************************************/
static void app_battery_gauge_interrupt_callback(void)
{
    k_work_submit(&s_battery_gauge_interrupt_work);
}

/********************************************************************
**函数名称:  app_battery_gauge_callback_init
**入口参数:  无
**出口参数:  无
**函数功能:  初始化工作项并注册电量计 INTN 回调
**返回值:    0 表示成功，负值表示注册失败
*********************************************************************/
int app_battery_gauge_callback_init(void)
{
    battery_gauge_result_t result;

    k_work_init(&s_battery_gauge_interrupt_work,
                app_battery_gauge_interrupt_work_handler);
    result = battery_gauge_register_interrupt_callback(
        app_battery_gauge_interrupt_callback);

    return (result == BATTERY_GAUGE_SUCCESS) ? 0 : -1;
}
```

> 当前 API 只保存一个中断回调。再次注册会覆盖此前注册的回调，业务层应统一管理回调所有权。

---

## 六、推荐初始化流程

```text
系统外设和 I2C 就绪
        ↓
my_battery_gauge_state_load()
        ↓
battery_gauge_init(&cycle_count_raw) 或 battery_gauge_init(NULL)
        ↓
battery_gauge_get_chip_id()
        ↓
battery_gauge_interrupt_config()
        ↓
battery_gauge_register_interrupt_callback()
        ↓
周期调用 battery_gauge_read()
```

建议在正常工作模式下以不小于 `1500 ms` 的周期调用 `battery_gauge_read()`。需要保存循环次数时，业务层按掉电策略保存 `battery_gauge_get_cycle_count_raw()` 的结果。

---

## 七、注意事项

1. 不要在 GPIO 中断回调中直接访问 I2C；
2. `battery_gauge_read()` 已按厂家推荐顺序读取数据，业务层优先使用该接口；
3. 正常模式数据约每 `1.3 s` 更新一次，过快读取可能得到重复数据；
4. 睡眠模式下所有量测停止更新，读取到的可能是进入睡眠前的旧值；
5. 芯片掉电后循环次数和 SOH 会丢失；应持久化 `cycle_count_raw`，并在下一次初始化时恢复；
6. 温度阈值转换精度为 `0.5 ℃/LSB`，API 入参使用整数摄氏度；
7. 电流正负方向必须通过充电和放电实测确认后，再由业务层统一定义；
8. 电池 Profile 位于厂家驱动中，更换电池型号后必须重新标定和验证；
9. I2C 频率不得超过手册规定的 `400 kHz`；
10. 应用层不要直接修改芯片地址、写保护或 Profile 寄存器。
