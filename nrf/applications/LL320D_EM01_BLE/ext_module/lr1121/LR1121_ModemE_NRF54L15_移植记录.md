# LR1121 Modem-E 移植记录（NRF54L15）

**日期**：2026-08-16  
**目标**：在 `LL320D_EM01_BLE` 工程中接入 LR1121 Modem-E，实现 LoRaWAN OTAA 入网、周期上行和下行数据接收，用于 Tracker 终端验证。

## 1. 驱动来源和版本选择

使用 Semtech 官方开源驱动：

- 仓库：<https://github.com/Lora-net/lr1121_modemE_driver>
- 本地目录：`ext_module/lr1121/modemE_driver`
- 使用版本：`v2.0.0`
- Modem-E 固件兼容性：LR1121 Modem-E `v2.1.0`

选择 Modem-E 驱动而非直接 Radio 驱动的原因：Tracker 的目标流程是 LoRaWAN OTAA 入网、上行和下行；Modem-E 固件已在 LR1121 内部实现 LoRaWAN 协议栈，主控只需调用 Modem-E API，不需要在 NRF54L15 上移植完整 LoRaWAN MAC。

## 2. 硬件分析结论

### 2.1 必需连接

| LR1121 信号 | NRF54L15 引脚 | 用途 |
| --- | --- | --- |
| SCK | P2.01 | SPI 时钟 |
| SDO/MOSI | P2.02 | 主控到 LR1121 数据 |
| SDI/MISO | P2.04 | LR1121 到主控数据 |
| NSS/CS | P2.05 | SPI 片选 |
| LR_NRESET | P2.03 | 模块硬复位 |
| BUSY | P2.00 | 命令执行忙状态握手 |

使用 `spi20`。普通 SPI 传输由 Zephyr SPI 的 `cs-gpios` 管理片选；HAL 仅在 Modem-E 规定的唤醒流程中手动控制 NSS。

### 2.2 NRF54L15 与 LR1121 评估板实际接线

以下连接依据 `LR1121MB2xxxAS_e742v01a_schematic_layout.pdf` 中的网络名称和 mbed 兼容连接器标号。NRF54L15 与 LR1121 评估板共需连接 8 根线：6 根数字信号线、1 根地线和 1 根 3.3 V 电源线。

| 序号 | NRF54L15 侧 | LR1121 评估板侧 | 原理图连接器/标号 | 说明 |
| --- | --- | --- | --- | --- |
| 1 | P2.01 | SCK | J2 / D10（pin 3） | SPI 时钟 |
| 2 | P2.02 | MOSI | J2 / D11（pin 4） | 主控输出，接 LR1121 MOSI |
| 3 | P2.04 | MISO | J2 / D12（pin 5） | 主控输入，接 LR1121 MISO |
| 4 | P2.05 | NSS | J1 / D7（pin 8） | 低有效 SPI 片选 |
| 5 | P2.03 | LR_NRESET | J4 / A0（pin 1） | 低有效硬复位 |
| 6 | P2.00 | BUSY | J1 / D3（pin 4） | LR1121 输出，主控输入 |
| 7 | GND | GND | J3 / GND（pin 6 或 pin 7） | 必须共地 |
| 8 | 3.3 V 电源 | +3V3 / VDD_3V3 | J3 / +3V3（pin 4） | 为未独立供电的评估板供电 |

接线方向要点：`MOSI` 是 NRF54L15 输出到 LR1121，`MISO` 是 LR1121 输出到 NRF54L15；BUSY 同样是 LR1121 输出到 NRF54L15。数字信号与电源均以 3.3 V 逻辑电平使用。

供电注意事项：

1. 若 LR1121 评估板已经通过 USB 或其他途径独立供电，只连接两板 GND 和 6 根数字信号线；不要把 NRF54L15 的 3.3 V 与来源不明的外部电源并联。
2. 若由 NRF54L15 板给 LR1121 评估板供电，接 J3 的 `+3V3`，不要接 J3 的 `+5V` 或 `VIN`。
3. 天线应接在与目标 LoRaWAN Region 对应的 LoRa SMA 口；2.4 GHz SMA 口不用于本次 LoRaWAN 验证。

### 2.3 不使用 DIO9/IRQ

本次使用 Modem-E host driver 的轮询事件接口 `modem_e_get_event()`。该方案所需硬件握手是 SPI、BUSY 和 RESET，当前不依赖 `DIO9` 中断线。不能将 `BUSY` 误认为 DIO0；它是 LR1121 的独立 BUSY 信号。

### 2.4 资源冲突处理

`spi20` 与原 `uart20` 冲突，因此在 overlay 中禁用 `uart20`。原磁吸 UART 模块改为不在启动时初始化，并将其设备引用改为 `DEVICE_DT_GET_OR_NULL()`，使 disabled 的 `uart20` 不再引发 Devicetree 设备符号编译错误。环境传感器电源脚 P2.05 也暂时停用，以释放给 LR1121 NSS。

## 3. 软件适配步骤

### 3.1 Devicetree 和 Kconfig

修改 `boards/nrf54l15dk_nrf54l15_cpuapp.overlay`：

1. 配置 `spi20` 的 SCK/MOSI/MISO pinctrl。
2. 设置 `cs-gpios = <&gpio2 5 GPIO_ACTIVE_LOW>`。
3. 创建 `lr1121@0` SPI 子节点，最大频率为 8 MHz。
4. 在 `lr1121@0` 节点中配置 BUSY 和 RESET GPIO 属性。
5. 禁用冲突的 `uart20`。

新增 `dts/bindings/lora/semtech,lr1121-modem-e.yaml`，继承 `spi-device.yaml`。这是必要项：若 SPI 从设备没有匹配 binding，`SPI_DT_SPEC_GET()` 所需的 `spi-max-frequency` 等生成属性不会存在，构建会失败。

在 `prj.conf` 启用 `CONFIG_SPI=y`。

### 3.2 NRF54L15 HAL

新增 `ext_module/lr1121/port/modem_e_hal_nrf54l15.c`，实现 Semtech 驱动要求的 HAL：

- 通过 Zephyr `spi_transceive_dt()` 读写 LR1121。
- 每次命令前后读取 BUSY，避免模块尚未完成前一命令时继续传输。
- 通过 RESET GPIO 进行硬复位。
- 在 Modem-E 唤醒时序中产生 NSS 脉冲。
- 使用 Zephyr 睡眠 API 实现延时。

### 3.3 Semtech 源文件选择

本需求只需 LoRaWAN Modem-E API，编译以下官方源文件：

- `modem_e_modem.c`
- `modem_e_lorawan.c`
- `modem_e_system.c`

不能只加入 `modem_e_system.c`：它只提供版本、复位等系统命令；LoRaWAN 入网、发包、取事件和读下行分别依赖 `modem_e_modem.c` 与 `modem_e_lorawan.c`。相应头文件通过 `CMakeLists.txt` 中的 `ext_module/lr1121/modemE_driver/src` 包含路径提供。

### 3.4 应用层框架

新增文件：

- `inc/my_lora.h`、`src/my_lora.c`：上层初始化、LoRa 开关状态、OTAA 业务、Join 重试、事件处理、上行调度、下行处理和 poll 线程；`my_lora.c` 顶部集中保留 Region、DevEUI、JoinEUI、AppKey、FPort、发送周期和临时 Tracker Payload 定义。
- `ext_module/lr1121/api/lr1121_lora_api.h/.c`：LR1121 Modem-E 底层适配接口，封装复位、版本、OTAA 配置、事件、上下行等驱动调用。

启动时，`my_lora_init()` 只创建常驻 LoRa 线程并将状态置为 `OFF`，不复位 Modem-E、不发起 Join。预留的 4G 开关消息分别调用 `my_lora_enable()`/`my_lora_disable()`；启用后线程执行 RESET -> 等待 50 ms -> 读取 Modem-E 版本 -> 配置 Region 和 OTAA 参数 -> 发起 Join。LoRa 线程每 100 ms 调用内部 `my_lora_poll()`，在 `JOINING` 状态按 30 秒间隔重试，在 `JOINED` 状态处理 `TX_DONE`、`DOWN_DATA` 和周期上行。

4G 开关协议为 `LTE+LORA=1` 和 `LTE+LORA=0`：`1` 调用 `my_lora_enable()` 并回复 `LTE+LORA=OK,1`，`0` 调用 `my_lora_disable()` 并回复 `LTE+LORA=OK,0`；参数非法时回复 `LTE+LORA=FAIL,PARAM`。

Modem-E 的唯一线程所有者是 LoRa poll 线程。LoRa 注册为公共模块 `MOD_LORA`，使用 `MY_MSG_LORA_ENABLE`、`MY_MSG_LORA_DISABLE` 和 `MY_MSG_LORA_TEST_UPLINK` 消息 ID；LTE、Shell 等调用方只通过 `my_send_msg()` 投递请求，LoRa 线程通过 `my_recv_msg()` 串行处理，不直接在业务调用方使用 RTOS 队列 API。

入网成功后，框架按配置周期调用 `modem_e_request_tx()` 发送上行；下行数据按 FPort 过滤后输出，业务命令解析待 Orange/KKS 协议明确后补充。

## 4. 编译问题分析与修复

### 4.1 SPI 子节点缺少 binding

现象：`SPI_DT_SPEC_GET()` 报 `spi_max_frequency`、`duplex`、`frame_format` 等属性缺失。

原因：新增的 `lr1121@0` 没有 compatible binding，Zephyr 未按 SPI device 生成属性。

修复：新增 `semtech,lr1121-modem-e.yaml`，并在 overlay 使用 `compatible = "semtech,lr1121-modem-e"`。

### 4.2 Modem-E 事件 API 未定义

现象：`modem_e_event_fields_t`、`modem_e_get_event()` 和 LoRaWAN 事件枚举未定义，随后出现 `event_type` 不是结构体成员的连锁错误。

原因：`my_lora.c` 只包含 HAL 和 LoRaWAN 头文件，遗漏了声明事件 API 和事件类型的 `modem_e_modem.h`。

修复：在 `ext_module/lr1121/api/lr1121_lora_api.c` 集中包含并调用 Modem-E 驱动接口，`src/my_lora.c` 只依赖 `lr1121_lora_api.h`。

同时，`my_lte.h` 单独包含时依赖 `my_comm.h` 中先定义的 `lte_boot_reason_t`。`my_lora.c` 改为包含 `my_comm.h`，使用工程既有的头文件依赖顺序。

### 4.3 禁用 UART20 后磁吸 UART 构建失败

现象：`DEVICE_DT_GET(MAGNETIC_UART_NODE)` 引用已 disabled 的 `uart20`，报 `__device_dts_ord_xxx` 未定义。

修复：改用 `DEVICE_DT_GET_OR_NULL()`，并在 `main.c` 暂停 `my_magnetic_uart_init()` 调用。

## 5. 当前验证结果

已使用 NCS v3.2.1 工具链完成构建：

```powershell
$env:Path = 'D:\ncs\toolchains\66cdf9b75e\opt\bin;' + $env:Path
& 'D:\ncs\toolchains\66cdf9b75e\opt\bin\ninja.exe' -C build
```

结果：构建成功，生成 `build/merged.hex`。

- FLASH：40848 B / 54 KB（73.87%）
- RAM：22368 B / 188 KB（11.62%）

工程仍存在若干既有编译警告，但 LoRa 集成没有剩余的编译或链接错误。

## 6. 下一步实机验证前的必填项

当前 `src/my_lora.c` 中的 OTAA 凭证为全零占位值。填入以下真实信息前，程序会只检测 Modem-E 版本，不会入网或发射：

1. LoRaWAN Region，例如 EU868、AS923、US915 等。
2. DevEUI、JoinEUI、AppKey。
3. 上行 FPort、下行 FPort、确认/非确认上行和上报周期。
4. Orange/KKS 定义的上行 Payload 编码及下行命令格式。

填入后需执行板级验证：确认 RESET/BUSY 时序、读取 Modem-E 版本、OTAA Join 成功、网关收到上行、终端收到指定 FPort 的下行数据。

## 7. Shell 测试命令

测试命令由 `inc/my_lora.h` 中的 `MY_LORA_SHELL_TEST_ENABLE` 控制。默认值为 `1`；后续量产时设为 `0`，测试 Shell 命令和仅用于测试的接口不会参与编译。初始化和事件轮询业务位于 `src/my_lora.c`。

| 命令 | 用途 | 预期结果 |
| --- | --- | --- |
| `app lora status` | 读取当前 LR1121、凭证和 Join 状态 | 初次上电且未填凭证时，`detected=yes`、`not configured`、`joined=no` |
| `LTE+LORA=1` | 异步启动 LoRa 服务，RESET 后读取 Modem-E 版本并发起 OTAA Join | 返回 `LTE+LORA=OK,1`；有效凭证时随后出现 Join 事件 |
| `app lora tx` | 已入网后立即发送固定 Payload `TEST` | 返回 `0`，网络服务器在配置的上行 FPort 收到 4 字节 ASCII `TEST` |

`app lora tx` 的错误码：`-ENODEV` 表示尚未识别 LR1121，`-EACCES` 表示 OTAA 凭证仍为全零，`-EAGAIN` 表示尚未 Join，`-EBUSY` 表示 Modem-E 当前不接受发送请求。LoRa 服务关闭时会先请求 Modem-E 离网，再停止轮询。
