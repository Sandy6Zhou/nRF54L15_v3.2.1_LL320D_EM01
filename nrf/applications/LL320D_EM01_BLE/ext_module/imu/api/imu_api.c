/********************************************************************
**版权所有         深圳市几米物联有限公司
**文件名称:        imu_api.c
**文件描述:        BMI325 IMU 模块统一接口实现文件
**当前版本:        V1.0
*********************************************************************/

#include "imu_api.h"
#include "../port/bmi325_port.h"

#include <errno.h>
#include <string.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(imu_api, LOG_LEVEL_INF);

#define IMU_FIFO_READ_BUF_SIZE          256
#define IMU_FIFO_FRAME_LIMIT            32

/* 当前项目约定所有 IMU API 均在同一线程串行调用，此处不额外增加锁保护。 */
static bool s_imu_initialized = false;
static imu_acc_range_t s_acc_range = IMU_ACC_RANGE_8G;
static imu_gyr_range_t s_gyr_range = IMU_GYR_RANGE_2000DPS;
static imu_odr_t s_acc_odr = IMU_ODR_100HZ;
static imu_odr_t s_gyr_odr = IMU_ODR_100HZ;

/********************************************************************
**函数名称:  imu_convert_result
**入口参数:  ret      ---        底层返回值（输入）
**出口参数:  无
**函数功能:  将底层错误码转换为 IMU 模块错误码
**返回值:    对应 IMU 模块错误码
*********************************************************************/
static imu_result_t imu_convert_result(int ret)
{
    if (ret == 0)
    {
        return IMU_SUCCESS;
    }

    if (ret == -EINVAL)
    {
        return IMU_ERROR_PARAM;
    }

    if (ret == -EIO)
    {
        return IMU_ERROR_COMM;
    }

    if (ret == -ETIMEDOUT)
    {
        return IMU_ERROR_TIMEOUT;
    }

    if (ret == -ENODEV)
    {
        return IMU_ERROR_CHIP_ID;
    }

    return IMU_ERROR_COMM;
}

/********************************************************************
**函数名称:  imu_map_acc_odr
**入口参数:  odr      ---        IMU ODR 枚举（输入）
**出口参数:  无
**函数功能:  将 IMU ODR 枚举映射到原厂加速度 ODR 枚举
**返回值:    原厂加速度 ODR 枚举值
*********************************************************************/
static uint8_t imu_map_acc_odr(imu_odr_t odr)
{
    switch (odr)
    {
        case IMU_ODR_12_5HZ:
            return BMI3_ACC_ODR_12_5HZ;

        case IMU_ODR_25HZ:
            return BMI3_ACC_ODR_25HZ;

        case IMU_ODR_50HZ:
            return BMI3_ACC_ODR_50HZ;

        case IMU_ODR_100HZ:
            return BMI3_ACC_ODR_100HZ;

        case IMU_ODR_200HZ:
            return BMI3_ACC_ODR_200HZ;

        case IMU_ODR_400HZ:
            return BMI3_ACC_ODR_400HZ;

        case IMU_ODR_800HZ:
            return BMI3_ACC_ODR_800HZ;

        case IMU_ODR_1600HZ:
            return BMI3_ACC_ODR_1600HZ;

        default:
            return BMI3_ACC_ODR_100HZ;
    }
}

/********************************************************************
**函数名称:  imu_map_gyr_odr
**入口参数:  odr      ---        IMU ODR 枚举（输入）
**出口参数:  无
**函数功能:  将 IMU ODR 枚举映射到原厂陀螺仪 ODR 枚举
**返回值:    原厂陀螺仪 ODR 枚举值
*********************************************************************/
static uint8_t imu_map_gyr_odr(imu_odr_t odr)
{
    switch (odr)
    {
        case IMU_ODR_12_5HZ:
            return BMI3_GYR_ODR_12_5HZ;

        case IMU_ODR_25HZ:
            return BMI3_GYR_ODR_25HZ;

        case IMU_ODR_50HZ:
            return BMI3_GYR_ODR_50HZ;

        case IMU_ODR_100HZ:
            return BMI3_GYR_ODR_100HZ;

        case IMU_ODR_200HZ:
            return BMI3_GYR_ODR_200HZ;

        case IMU_ODR_400HZ:
            return BMI3_GYR_ODR_400HZ;

        case IMU_ODR_800HZ:
            return BMI3_GYR_ODR_800HZ;

        case IMU_ODR_1600HZ:
            return BMI3_GYR_ODR_1600HZ;

        default:
            return BMI3_GYR_ODR_100HZ;
    }
}

/********************************************************************
**函数名称:  imu_map_acc_range
**入口参数:  range    ---        IMU 加速度量程枚举（输入）
**出口参数:  无
**函数功能:  将 IMU 加速度量程枚举映射到原厂量程枚举
**返回值:    原厂量程枚举值
*********************************************************************/
static uint8_t imu_map_acc_range(imu_acc_range_t range)
{
    switch (range)
    {
        case IMU_ACC_RANGE_2G:
            return BMI3_ACC_RANGE_2G;

        case IMU_ACC_RANGE_4G:
            return BMI3_ACC_RANGE_4G;

        case IMU_ACC_RANGE_8G:
            return BMI3_ACC_RANGE_8G;

        case IMU_ACC_RANGE_16G:
            return BMI3_ACC_RANGE_16G;

        default:
            return BMI3_ACC_RANGE_8G;
    }
}

/********************************************************************
**函数名称:  imu_map_gyr_range
**入口参数:  range    ---        IMU 陀螺仪量程枚举（输入）
**出口参数:  无
**函数功能:  将 IMU 陀螺仪量程枚举映射到原厂量程枚举
**返回值:    原厂量程枚举值
*********************************************************************/
static uint8_t imu_map_gyr_range(imu_gyr_range_t range)
{
    switch (range)
    {
        case IMU_GYR_RANGE_125DPS:
            return BMI3_GYR_RANGE_125DPS;

        case IMU_GYR_RANGE_250DPS:
            return BMI3_GYR_RANGE_250DPS;

        case IMU_GYR_RANGE_500DPS:
            return BMI3_GYR_RANGE_500DPS;

        case IMU_GYR_RANGE_1000DPS:
            return BMI3_GYR_RANGE_1000DPS;

        case IMU_GYR_RANGE_2000DPS:
            return BMI3_GYR_RANGE_2000DPS;

        default:
            return BMI3_GYR_RANGE_2000DPS;
    }
}

/********************************************************************
**函数名称:  imu_map_acc_mode
**入口参数:  mode     ---        IMU 电源模式枚举（输入）
**出口参数:  无
**函数功能:  将 IMU 电源模式枚举映射到原厂加速度工作模式
**返回值:    原厂加速度工作模式值
*********************************************************************/
static uint8_t imu_map_acc_mode(imu_power_mode_t mode)
{
    switch (mode)
    {
        case IMU_POWER_SUSPEND:
            return BMI3_ACC_MODE_DISABLE;

        case IMU_POWER_LOW_POWER:
            return BMI3_ACC_MODE_LOW_PWR;

        case IMU_POWER_NORMAL:
            return BMI3_ACC_MODE_NORMAL;

        case IMU_POWER_HIGH_PERF:
            return BMI3_ACC_MODE_HIGH_PERF;

        default:
            return BMI3_ACC_MODE_HIGH_PERF;
    }
}

/********************************************************************
**函数名称:  imu_map_gyr_mode
**入口参数:  mode     ---        IMU 电源模式枚举（输入）
**出口参数:  无
**函数功能:  将 IMU 电源模式枚举映射到原厂陀螺仪工作模式
**返回值:    原厂陀螺仪工作模式值
*********************************************************************/
static uint8_t imu_map_gyr_mode(imu_power_mode_t mode)
{
    switch (mode)
    {
        case IMU_POWER_SUSPEND:
            return BMI3_GYR_MODE_DISABLE;

        case IMU_POWER_LOW_POWER:
            return BMI3_GYR_MODE_LOW_PWR;

        case IMU_POWER_NORMAL:
            return BMI3_GYR_MODE_NORMAL;

        case IMU_POWER_HIGH_PERF:
            return BMI3_GYR_MODE_HIGH_PERF;

        default:
            return BMI3_GYR_MODE_HIGH_PERF;
    }
}

/********************************************************************
**函数名称:  imu_fill_default_config
**入口参数:  config   ---        配置结构体指针（输出）
**出口参数:  config   ---        默认配置内容
**函数功能:  填充 IMU 默认初始化配置
**返回值:    无
*********************************************************************/
static void imu_fill_default_config(struct imu_config *config)
{
    config->acc_range = IMU_ACC_RANGE_8G;
    config->acc_odr = IMU_ODR_100HZ;
    config->gyr_range = IMU_GYR_RANGE_2000DPS;
    config->gyr_odr = IMU_ODR_100HZ;
    config->power_mode = IMU_POWER_HIGH_PERF;
}

/********************************************************************
**函数名称:  imu_get_dev_checked
**入口参数:  dev      ---        原厂设备上下文二级指针（输出）
**出口参数:  dev      ---        原厂设备上下文指针
**函数功能:  检查 IMU 初始化状态并获取原厂设备上下文
**返回值:    IMU_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
static imu_result_t imu_get_dev_checked(struct bmi3_dev **dev)
{
    if (s_imu_initialized != true)
    {
        return IMU_ERROR_INIT;
    }

    if (dev == NULL)
    {
        return IMU_ERROR_PARAM;
    }

    *dev = bmi325_port_get_dev();
    if (*dev == NULL)
    {
        return IMU_ERROR_INIT;
    }

    return IMU_SUCCESS;
}

/********************************************************************
**函数名称:  imu_read_reg_u16
**入口参数:  reg_addr ---        寄存器地址（输入）
**出口参数:  reg_val  ---        寄存器值
**函数功能:  读取 BMI325 16-bit 小端寄存器
**返回值:    IMU_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
static imu_result_t imu_read_reg_u16(uint8_t reg_addr, uint16_t *reg_val)
{
    struct bmi3_dev *dev;
    uint8_t data[2];
    imu_result_t ret;
    int8_t rslt;

    if (reg_val == NULL)
    {
        return IMU_ERROR_PARAM;
    }

    ret = imu_get_dev_checked(&dev);
    if (ret != IMU_SUCCESS)
    {
        return ret;
    }

    rslt = bmi325_get_regs(reg_addr, data, 2, dev);
    if (rslt != BMI325_OK)
    {
        return IMU_ERROR_COMM;
    }

    *reg_val = (uint16_t)data[0] | ((uint16_t)data[1] << 8);

    return IMU_SUCCESS;
}

/********************************************************************
**函数名称:  imu_write_reg_u16
**入口参数:  reg_addr ---        寄存器地址（输入）
**           reg_val  ---        寄存器值（输入）
**出口参数:  无
**函数功能:  写入 BMI325 16-bit 小端寄存器
**返回值:    IMU_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
static imu_result_t imu_write_reg_u16(uint8_t reg_addr, uint16_t reg_val)
{
    struct bmi3_dev *dev;
    uint8_t data[2];
    imu_result_t ret;
    int8_t rslt;

    ret = imu_get_dev_checked(&dev);
    if (ret != IMU_SUCCESS)
    {
        return ret;
    }

    data[0] = (uint8_t)(reg_val & 0x00FFU);
    data[1] = (uint8_t)((reg_val >> 8) & 0x00FFU);

    rslt = bmi325_set_regs(reg_addr, data, 2, dev);
    if (rslt != BMI325_OK)
    {
        return IMU_ERROR_COMM;
    }

    return IMU_SUCCESS;
}

/********************************************************************
**函数名称:  imu_convert_accel
**入口参数:  raw      ---        加速度原始值（输入）
**出口参数:  无
**函数功能:  将加速度原始值换算为 mg
**返回值:    换算后的 mg 值
*********************************************************************/
static int32_t imu_convert_accel(int16_t raw)
{
    int32_t range_g;

    switch (s_acc_range)
    {
        case IMU_ACC_RANGE_2G:
            range_g = 2;
            break;

        case IMU_ACC_RANGE_4G:
            range_g = 4;
            break;

        case IMU_ACC_RANGE_8G:
            range_g = 8;
            break;

        case IMU_ACC_RANGE_16G:
            range_g = 16;
            break;

        default:
            range_g = 8;
            break;
    }

    return (int32_t)(((int64_t)raw * range_g * 1000) / 32768);
}

/********************************************************************
**函数名称:  imu_convert_gyro
**入口参数:  raw      ---        角速度原始值（输入）
**出口参数:  无
**函数功能:  将角速度原始值换算为 mdps
**返回值:    换算后的 mdps 值
*********************************************************************/
static int32_t imu_convert_gyro(int16_t raw)
{
    int32_t range_dps;

    switch (s_gyr_range)
    {
        case IMU_GYR_RANGE_125DPS:
            range_dps = 125;
            break;

        case IMU_GYR_RANGE_250DPS:
            range_dps = 250;
            break;

        case IMU_GYR_RANGE_500DPS:
            range_dps = 500;
            break;

        case IMU_GYR_RANGE_1000DPS:
            range_dps = 1000;
            break;

        case IMU_GYR_RANGE_2000DPS:
            range_dps = 2000;
            break;

        default:
            range_dps = 2000;
            break;
    }

    return (int32_t)(((int64_t)raw * range_dps * 1000) / 32768);
}

/********************************************************************
**函数名称:  imu_convert_temp
**入口参数:  raw      ---        温度原始值（输入）
**出口参数:  无
**函数功能:  将温度原始值换算为 0.01 摄氏度
**返回值:    温度值，单位 0.01 摄氏度
*********************************************************************/
static int16_t imu_convert_temp(int16_t raw)
{
    return (int16_t)(((int32_t)raw * 100) / 512 + 2300);
}

/********************************************************************
**函数名称:  imu_apply_basic_config
**入口参数:  dev      ---        原厂设备上下文（输入）
**           config   ---        IMU 初始化配置（输入）
**出口参数:  无
**函数功能:  配置 BMI325 加速度计与陀螺仪基础参数
**返回值:    0 表示成功，负值表示失败
*********************************************************************/
static int imu_apply_basic_config(struct bmi3_dev *dev, const struct imu_config *config)
{
    struct bmi3_sens_config sens_cfg[2];
    int8_t ret;

    memset(sens_cfg, 0, sizeof(sens_cfg));

    sens_cfg[0].type = BMI3_ACCEL;
    sens_cfg[0].cfg.acc.odr = imu_map_acc_odr(config->acc_odr);
    sens_cfg[0].cfg.acc.range = imu_map_acc_range(config->acc_range);
    sens_cfg[0].cfg.acc.acc_mode = imu_map_acc_mode(config->power_mode);
    sens_cfg[0].cfg.acc.bwp = BMI3_ACC_BW_ODR_HALF;
    sens_cfg[0].cfg.acc.avg_num = BMI3_ACC_AVG1;

    sens_cfg[1].type = BMI3_GYRO;
    sens_cfg[1].cfg.gyr.odr = imu_map_gyr_odr(config->gyr_odr);
    sens_cfg[1].cfg.gyr.range = imu_map_gyr_range(config->gyr_range);
    sens_cfg[1].cfg.gyr.gyr_mode = imu_map_gyr_mode(config->power_mode);
    sens_cfg[1].cfg.gyr.bwp = BMI3_GYR_BW_ODR_HALF;
    sens_cfg[1].cfg.gyr.avg_num = BMI3_GYR_AVG1;

    ret = bmi325_set_sensor_config(sens_cfg, 2, dev);
    if (ret != BMI325_OK)
    {
        return -EIO;
    }

    s_acc_range = config->acc_range;
    s_gyr_range = config->gyr_range;
    s_acc_odr = config->acc_odr;
    s_gyr_odr = config->gyr_odr;

    return 0;
}

/********************************************************************
**函数名称:  imu_step_counter_init
**入口参数:  dev      ---        原厂设备上下文（输入）
**出口参数:  无
**函数功能:  使能 BMI325 计步器特征（仅初始化时调用一次）
**返回值:    0 表示成功，负值表示失败
**注意事项:  计步器参数已由 bmi325_init 中的 context_switch_selection(WEARABLE) 加载，
**           为原厂针对可穿戴场景调优的参数，此处不再覆盖，仅负责使能；
**           通过 select_sensor 重新使能或重写 FEATURE_IO_STATUS 触发 feature engine 重载
**           均会复位累计步数，因此使能只能在初始化阶段执行一次，运行期间只读取步数
*********************************************************************/
static int imu_step_counter_init(struct bmi3_dev *dev)
{
    struct bmi3_feature_enable feature;
    int8_t ret;

    memset(&feature, 0, sizeof(feature));

    // 使能计步器特征（仅在初始化阶段执行一次）
    // 注意：bmi325_select_sensor 为读改写，传入结构体中为 0 的特征位会被关闭，
    // 此处仅在初始化阶段（其他特征尚未使能）调用，故只置 step_counter_en 是安全的
    feature.step_counter_en = BMI3_ENABLE;
    ret = bmi325_select_sensor(&feature, dev);
    if (ret != BMI325_OK)
    {
        return -EIO;
    }

    return 0;
}

/********************************************************************
**函数名称:  imu_int_src_to_reg_field
**入口参数:  src      ---        中断源（输入）
**出口参数:  reg_addr ---        映射寄存器地址
**           bit_pos  ---        位偏移
**函数功能:  将 IMU 中断源映射为 BMI325 中断映射寄存器字段
**返回值:    0 表示成功，负值表示失败
*********************************************************************/
static int imu_int_src_to_reg_field(imu_int_src_t src, uint8_t *reg_addr, uint8_t *bit_pos)
{
    if ((reg_addr == NULL) || (bit_pos == NULL))
    {
        return -EINVAL;
    }

    switch (src)
    {
        case IMU_INT_SRC_NO_MOTION:
            *reg_addr = BMI3_REG_INT_MAP1;
            *bit_pos = 0;
            break;

        case IMU_INT_SRC_ANY_MOTION:
            *reg_addr = BMI3_REG_INT_MAP1;
            *bit_pos = 2;
            break;

        case IMU_INT_SRC_FLAT:
            *reg_addr = BMI3_REG_INT_MAP1;
            *bit_pos = 4;
            break;

        case IMU_INT_SRC_ORIENTATION:
            *reg_addr = BMI3_REG_INT_MAP1;
            *bit_pos = 6;
            break;

        case IMU_INT_SRC_STEP_DETECTOR:
            *reg_addr = BMI3_REG_INT_MAP1;
            *bit_pos = 8;
            break;

        case IMU_INT_SRC_STEP_COUNTER:
            *reg_addr = BMI3_REG_INT_MAP1;
            *bit_pos = 10;
            break;

        case IMU_INT_SRC_SIG_MOTION:
            *reg_addr = BMI3_REG_INT_MAP1;
            *bit_pos = 12;
            break;

        case IMU_INT_SRC_TILT:
            *reg_addr = BMI3_REG_INT_MAP1;
            *bit_pos = 14;
            break;

        case IMU_INT_SRC_TAP:
            *reg_addr = BMI3_REG_INT_MAP2;
            *bit_pos = 0;
            break;

        case IMU_INT_SRC_FEATURE_STATUS:
            *reg_addr = BMI3_REG_INT_MAP2;
            *bit_pos = 4;
            break;

        case IMU_INT_SRC_TEMP_DRDY:
            *reg_addr = BMI3_REG_INT_MAP2;
            *bit_pos = 6;
            break;

        case IMU_INT_SRC_GYR_DRDY:
            *reg_addr = BMI3_REG_INT_MAP2;
            *bit_pos = 8;
            break;

        case IMU_INT_SRC_ACC_DRDY:
            *reg_addr = BMI3_REG_INT_MAP2;
            *bit_pos = 10;
            break;

        case IMU_INT_SRC_FIFO_WATERMARK:
            *reg_addr = BMI3_REG_INT_MAP2;
            *bit_pos = 12;
            break;

        case IMU_INT_SRC_FIFO_FULL:
            *reg_addr = BMI3_REG_INT_MAP2;
            *bit_pos = 14;
            break;

        default:
            return -EINVAL;
    }

    return 0;
}

/********************************************************************
**函数名称:  imu_feature_to_mask
**入口参数:  feature  ---        特征类型（输入）
**出口参数:  无
**函数功能:  将 IMU 特征类型转换为 FEATURE_IO0 位掩码
**返回值:    特征位掩码，0 表示无效
*********************************************************************/
static uint16_t imu_feature_to_mask(imu_feature_t feature)
{
    switch (feature)
    {
        case IMU_FEATURE_NO_MOTION:
            return BMI3_NO_MOTION_X_EN_MASK | BMI3_NO_MOTION_Y_EN_MASK | BMI3_NO_MOTION_Z_EN_MASK;

        case IMU_FEATURE_ANY_MOTION:
            return BMI3_ANY_MOTION_X_EN_MASK | BMI3_ANY_MOTION_Y_EN_MASK | BMI3_ANY_MOTION_Z_EN_MASK;

        case IMU_FEATURE_FLAT:
            return BMI3_FLAT_EN_MASK;

        case IMU_FEATURE_ORIENTATION:
            return BMI3_ORIENTATION_EN_MASK;

        case IMU_FEATURE_STEP_DETECTOR:
            return BMI3_STEP_DETECTOR_EN_MASK;

        case IMU_FEATURE_STEP_COUNTER:
            return BMI3_STEP_COUNTER_EN_MASK;

        case IMU_FEATURE_SIG_MOTION:
            return BMI3_SIG_MOTION_EN_MASK;

        case IMU_FEATURE_TILT:
            return BMI3_TILT_EN_MASK;

        case IMU_FEATURE_TAP_SINGLE:
            return BMI3_TAP_DETECTOR_S_TAP_EN_MASK;

        case IMU_FEATURE_TAP_DOUBLE:
            return BMI3_TAP_DETECTOR_D_TAP_EN_MASK;

        case IMU_FEATURE_TAP_TRIPLE:
            return BMI3_TAP_DETECTOR_T_TAP_EN_MASK;

        default:
            return 0;
    }
}

/********************************************************************
**函数名称:  imu_init
**入口参数:  config   ---        初始化配置参数（输入）
**出口参数:  无
**函数功能:  初始化 BMI325 IMU 模块并配置加速度计与陀螺仪
**返回值:    IMU_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
imu_result_t imu_init(const struct imu_config *config)
{
    struct imu_config default_config;
    const struct imu_config *real_config;
    struct bmi3_dev *dev;
    int ret;
    int8_t rslt;

    if (config == NULL)
    {
        imu_fill_default_config(&default_config);
        real_config = &default_config;
    }
    else
    {
        if ((config->acc_range >= IMU_ACC_RANGE_MAX) ||
            (config->gyr_range >= IMU_GYR_RANGE_MAX) ||
            (config->acc_odr >= IMU_ODR_MAX) ||
            (config->gyr_odr >= IMU_ODR_MAX) ||
            (config->power_mode >= IMU_POWER_MAX))
        {
            return IMU_ERROR_PARAM;
        }

        real_config = config;
    }

    ret = bmi325_port_init_dev();
    if (ret < 0)
    {
        return imu_convert_result(ret);
    }

    dev = bmi325_port_get_dev();
    if (dev == NULL)
    {
        return IMU_ERROR_INIT;
    }

    rslt = bmi325_init(dev);
    if (rslt != BMI325_OK)
    {
        return IMU_ERROR_CHIP_ID;
    }

    ret = imu_apply_basic_config(dev, real_config);
    if (ret < 0)
    {
        return imu_convert_result(ret);
    }

    s_imu_initialized = true;

    return IMU_SUCCESS;
}

/********************************************************************
**函数名称:  imu_get_chip_id
**入口参数:  id       ---        芯片 ID 存储指针（输出）
**出口参数:  id       ---        读取到的芯片 ID
**函数功能:  获取 BMI325 芯片 ID
**返回值:    IMU_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
imu_result_t imu_get_chip_id(uint8_t *id)
{
    struct bmi3_dev *dev;
    imu_result_t ret;

    if (id == NULL)
    {
        return IMU_ERROR_PARAM;
    }

    ret = imu_get_dev_checked(&dev);
    if (ret != IMU_SUCCESS)
    {
        return ret;
    }

    *id = dev->chip_id;

    return IMU_SUCCESS;
}

/********************************************************************
**函数名称:  imu_read_raw
**入口参数:  raw      ---        原始数据存储指针（输出）
**出口参数:  raw      ---        原始 6 轴数据和温度
**函数功能:  读取 BMI325 原始数据
**返回值:    IMU_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
imu_result_t imu_read_raw(struct imu_raw_data *raw)
{
    struct bmi3_dev *dev;
    struct bmi3_sensor_data sensor_data[3];
    imu_result_t ret;
    int8_t rslt;

    if (raw == NULL)
    {
        return IMU_ERROR_PARAM;
    }

    ret = imu_get_dev_checked(&dev);
    if (ret != IMU_SUCCESS)
    {
        return ret;
    }

    memset(sensor_data, 0, sizeof(sensor_data));
    sensor_data[0].type = BMI3_ACCEL;
    sensor_data[1].type = BMI3_GYRO;
    sensor_data[2].type = BMI3_TEMP;

    rslt = bmi325_get_sensor_data(sensor_data, 3, dev);
    if (rslt != BMI3_OK)
    {
        return IMU_ERROR_COMM;
    }

    raw->acc_x = sensor_data[0].sens_data.acc.x;
    raw->acc_y = sensor_data[0].sens_data.acc.y;
    raw->acc_z = sensor_data[0].sens_data.acc.z;
    raw->gyr_x = sensor_data[1].sens_data.gyr.x;
    raw->gyr_y = sensor_data[1].sens_data.gyr.y;
    raw->gyr_z = sensor_data[1].sens_data.gyr.z;
    raw->temperature = (int16_t)sensor_data[2].sens_data.temp.temp_data;

    return IMU_SUCCESS;
}

/********************************************************************
**函数名称:  imu_read
**入口参数:  data     ---        换算数据存储指针（输出）
**出口参数:  data     ---        换算后的 6 轴数据和温度
**函数功能:  读取并换算 BMI325 数据
**返回值:    IMU_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
imu_result_t imu_read(struct imu_data *data)
{
    struct imu_raw_data raw;
    imu_result_t ret;

    if (data == NULL)
    {
        return IMU_ERROR_PARAM;
    }

    ret = imu_read_raw(&raw);
    if (ret != IMU_SUCCESS)
    {
        return ret;
    }

    data->acc_x = imu_convert_accel(raw.acc_x);
    data->acc_y = imu_convert_accel(raw.acc_y);
    data->acc_z = imu_convert_accel(raw.acc_z);
    data->gyr_x = imu_convert_gyro(raw.gyr_x);
    data->gyr_y = imu_convert_gyro(raw.gyr_y);
    data->gyr_z = imu_convert_gyro(raw.gyr_z);
    data->temperature = imu_convert_temp(raw.temperature);

    return IMU_SUCCESS;
}

/********************************************************************
**函数名称:  imu_set_config
**入口参数:  config   ---        运行期配置参数（输入）
**出口参数:  无
**函数功能:  运行期配置 BMI325 加速度计与陀螺仪
**返回值:    IMU_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
imu_result_t imu_set_config(const struct imu_config *config)
{
    struct bmi3_dev *dev;
    imu_result_t ret;
    int result;

    if (config == NULL)
    {
        return IMU_ERROR_PARAM;
    }

    if ((config->acc_range >= IMU_ACC_RANGE_MAX) ||
        (config->gyr_range >= IMU_GYR_RANGE_MAX) ||
        (config->acc_odr >= IMU_ODR_MAX) ||
        (config->gyr_odr >= IMU_ODR_MAX) ||
        (config->power_mode >= IMU_POWER_MAX))
    {
        return IMU_ERROR_PARAM;
    }

    ret = imu_get_dev_checked(&dev);
    if (ret != IMU_SUCCESS)
    {
        return ret;
    }

    result = imu_apply_basic_config(dev, config);
    if (result < 0)
    {
        return imu_convert_result(result);
    }

    return IMU_SUCCESS;
}

/********************************************************************
**函数名称:  imu_set_power_mode
**入口参数:  mode     ---        电源模式（输入）
**出口参数:  无
**函数功能:  设置 BMI325 电源模式
**返回值:    IMU_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
imu_result_t imu_set_power_mode(imu_power_mode_t mode)
{
    struct imu_config config;

    if (mode >= IMU_POWER_MAX)
    {
        return IMU_ERROR_PARAM;
    }

    config.acc_range = s_acc_range;
    config.acc_odr = s_acc_odr;
    config.gyr_range = s_gyr_range;
    config.gyr_odr = s_gyr_odr;
    config.power_mode = mode;
    return imu_set_config(&config);
}

/********************************************************************
**函数名称:  imu_read_status
**入口参数:  status   ---        状态寄存器存储指针（输出）
**出口参数:  status   ---        状态寄存器值
**函数功能:  读取 BMI325 状态寄存器
**返回值:    IMU_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
imu_result_t imu_read_status(uint16_t *status)
{
    return imu_read_reg_u16(BMI3_REG_STATUS, status);
}

/********************************************************************
**函数名称:  imu_read_err_reg
**入口参数:  err_reg  ---        错误寄存器存储指针（输出）
**出口参数:  err_reg  ---        错误寄存器值
**函数功能:  读取 BMI325 错误寄存器
**返回值:    IMU_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
imu_result_t imu_read_err_reg(uint16_t *err_reg)
{
    return imu_read_reg_u16(BMI3_REG_ERR_REG, err_reg);
}

/********************************************************************
**函数名称:  imu_read_sensor_time
**入口参数:  sensor_time ---     传感器时间存储指针（输出）
**出口参数:  sensor_time ---     传感器时间
**函数功能:  读取 BMI325 传感器时间
**返回值:    IMU_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
imu_result_t imu_read_sensor_time(uint32_t *sensor_time)
{
    struct bmi3_dev *dev;
    imu_result_t ret;
    int8_t rslt;

    if (sensor_time == NULL)
    {
        return IMU_ERROR_PARAM;
    }

    ret = imu_get_dev_checked(&dev);
    if (ret != IMU_SUCCESS)
    {
        return ret;
    }

    rslt = bmi325_get_sensor_time(sensor_time, dev);
    if (rslt != BMI325_OK)
    {
        return IMU_ERROR_COMM;
    }

    return IMU_SUCCESS;
}

/********************************************************************
**函数名称:  imu_int_pin_config
**入口参数:  pin      ---        中断引脚（输入）
**           config   ---        中断引脚配置（输入）
**出口参数:  无
**函数功能:  配置 BMI325 中断引脚
**返回值:    IMU_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
imu_result_t imu_int_pin_config(imu_int_pin_t pin, const struct imu_int_config *config)
{
    struct bmi3_dev *dev;
    struct bmi3_int_pin_config int_cfg;
    imu_result_t ret;
    uint8_t index;
    int8_t rslt;

    if ((config == NULL) || (pin == IMU_INT_NONE) || (pin >= IMU_INT_PIN_MAX))
    {
        return IMU_ERROR_PARAM;
    }

    ret = imu_get_dev_checked(&dev);
    if (ret != IMU_SUCCESS)
    {
        return ret;
    }

    memset(&int_cfg, 0, sizeof(int_cfg));
    int_cfg.pin_type = (pin == IMU_INT_PIN1) ? BMI3_INT1 : BMI3_INT2;
    int_cfg.int_latch = (config->latch == true) ? BMI3_INT_LATCH_EN : BMI3_INT_LATCH_DISABLE;
    index = (pin == IMU_INT_PIN1) ? 0U : 1U;
    int_cfg.pin_cfg[index].lvl = (config->active_high == true) ? BMI3_INT_ACTIVE_HIGH : BMI3_INT_ACTIVE_LOW;
    int_cfg.pin_cfg[index].od = (config->open_drain == true) ? BMI3_INT_OPEN_DRAIN : BMI3_INT_PUSH_PULL;
    int_cfg.pin_cfg[index].output_en = BMI3_INT_OUTPUT_ENABLE;

    rslt = bmi325_set_int_pin_config(&int_cfg, dev);
    if (rslt != BMI325_OK)
    {
        return IMU_ERROR_COMM;
    }

    return IMU_SUCCESS;
}

/********************************************************************
**函数名称:  imu_int_map
**入口参数:  src      ---        中断源（输入）
**           pin      ---        目标中断引脚（输入）
**出口参数:  无
**函数功能:  映射 BMI325 中断源到指定中断引脚
**返回值:    IMU_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
imu_result_t imu_int_map(imu_int_src_t src, imu_int_pin_t pin)
{
    uint16_t reg_val;
    uint8_t reg_addr;
    uint8_t bit_pos;
    int ret;
    imu_result_t result;

    if ((src >= IMU_INT_SRC_MAX) || (pin >= IMU_INT_PIN_MAX))
    {
        return IMU_ERROR_PARAM;
    }

    ret = imu_int_src_to_reg_field(src, &reg_addr, &bit_pos);
    if (ret < 0)
    {
        return IMU_ERROR_PARAM;
    }

    result = imu_read_reg_u16(reg_addr, &reg_val);
    if (result != IMU_SUCCESS)
    {
        return result;
    }

    reg_val &= ~(uint16_t)(0x0003U << bit_pos);
    reg_val |= (uint16_t)(((uint16_t)pin & 0x0003U) << bit_pos);

    return imu_write_reg_u16(reg_addr, reg_val);
}

/********************************************************************
**函数名称:  imu_register_int_callback
**入口参数:  callback ---        中断回调函数（输入）
**出口参数:  无
**函数功能:  注册 BMI325 INT GPIO 中断回调
**返回值:    IMU_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
imu_result_t imu_register_int_callback(imu_int_callback_t callback)
{
    int ret;

    if (callback == NULL)
    {
        return IMU_ERROR_PARAM;
    }

    ret = bmi325_port_register_int_callback(callback);

    return imu_convert_result(ret);
}

/********************************************************************
**函数名称:  imu_read_int_status
**入口参数:  pin      ---        中断引脚（输入）
**           status   ---        中断状态存储指针（输出）
**出口参数:  status   ---        中断状态
**函数功能:  读取 BMI325 中断状态
**返回值:    IMU_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
imu_result_t imu_read_int_status(imu_int_pin_t pin, uint16_t *status)
{
    struct bmi3_dev *dev;
    imu_result_t ret;
    int8_t rslt;

    if ((status == NULL) || (pin == IMU_INT_NONE) || (pin >= IMU_INT_PIN_MAX))
    {
        return IMU_ERROR_PARAM;
    }

    ret = imu_get_dev_checked(&dev);
    if (ret != IMU_SUCCESS)
    {
        return ret;
    }

    if (pin == IMU_INT_PIN1)
    {
        rslt = bmi325_get_int1_status(status, dev);
    }
    else
    {
        rslt = bmi325_get_int2_status(status, dev);
    }

    if (rslt != BMI325_OK)
    {
        return IMU_ERROR_COMM;
    }

    return IMU_SUCCESS;
}

/********************************************************************
**函数名称:  imu_fifo_config
**入口参数:  config   ---        FIFO 配置参数（输入）
**出口参数:  无
**函数功能:  配置 BMI325 FIFO
**返回值:    IMU_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
imu_result_t imu_fifo_config(const struct imu_fifo_config *config)
{
    struct bmi3_dev *dev;
    imu_result_t ret;
    uint16_t fifo_cfg;
    int8_t rslt;

    if (config == NULL)
    {
        return IMU_ERROR_PARAM;
    }

    if ((config->acc_en != true) && (config->gyr_en != true) &&
        (config->temp_en != true) && (config->time_en != true))
    {
        return IMU_ERROR_PARAM;
    }

    ret = imu_get_dev_checked(&dev);
    if (ret != IMU_SUCCESS)
    {
        return ret;
    }

    fifo_cfg = 0;
    fifo_cfg |= (config->stop_on_full == true) ? BMI3_FIFO_STOP_ON_FULL : 0U;
    fifo_cfg |= (config->time_en == true) ? BMI3_FIFO_TIME_EN : 0U;
    fifo_cfg |= (config->acc_en == true) ? BMI3_FIFO_ACC_EN : 0U;
    fifo_cfg |= (config->gyr_en == true) ? BMI3_FIFO_GYR_EN : 0U;
    fifo_cfg |= (config->temp_en == true) ? BMI3_FIFO_TEMP_EN : 0U;

    rslt = bmi325_set_fifo_wm(config->watermark, dev);
    if (rslt != BMI325_OK)
    {
        return IMU_ERROR_COMM;
    }

    rslt = bmi325_set_fifo_config(BMI3_FIFO_CONFIG_MASK, BMI325_DISABLE, dev);
    if (rslt != BMI325_OK)
    {
        return IMU_ERROR_COMM;
    }

    rslt = bmi325_set_fifo_config(fifo_cfg, BMI325_ENABLE, dev);
    if (rslt != BMI325_OK)
    {
        return IMU_ERROR_COMM;
    }

    return imu_fifo_flush();
}

/********************************************************************
**函数名称:  imu_fifo_read
**入口参数:  frames      ---     FIFO 帧存储指针（输出）
**           max_frames  ---     最大读取帧数（输入）
**出口参数:  frames      ---     FIFO 帧数据
**           frame_count ---     实际读取帧数
**函数功能:  读取 BMI325 FIFO 数据
**返回值:    IMU_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
imu_result_t imu_fifo_read(struct imu_raw_data *frames, uint16_t max_frames, uint16_t *frame_count)
{
    struct bmi3_dev *dev;
    struct bmi3_fifo_frame fifo;
    static struct bmi3_fifo_sens_axes_data s_acc_data[IMU_FIFO_FRAME_LIMIT];
    static struct bmi3_fifo_sens_axes_data s_gyr_data[IMU_FIFO_FRAME_LIMIT];
    static struct bmi3_fifo_temperature_data s_temp_data[IMU_FIFO_FRAME_LIMIT];
    static uint8_t fifo_buf[IMU_FIFO_READ_BUF_SIZE];
    uint16_t fifo_cfg;
    uint16_t frame_bytes;
    uint16_t frame_words;
    uint16_t frames_per_pass;
    uint16_t fifo_words;
    uint16_t frames_avail;
    uint16_t frames_this;
    uint16_t parse_len;
    uint16_t read_frames;
    uint16_t read_bytes;
    uint16_t copy_cnt;
    uint16_t total;
    uint16_t index;
    uint16_t tidx;
    imu_result_t ret;
    int8_t rslt;

    if ((frames == NULL) || (frame_count == NULL) || (max_frames == 0U))
    {
        return IMU_ERROR_PARAM;
    }

    *frame_count = 0;

    ret = imu_get_dev_checked(&dev);
    if (ret != IMU_SUCCESS)
    {
        return ret;
    }

    /* 读取 FIFO 配置，按当前使能的数据源计算单帧字节数（acc/gyr 各 6B，temp/time 各 2B） */
    rslt = bmi325_get_fifo_config(&fifo_cfg, dev);
    if (rslt != BMI325_OK)
    {
        return IMU_ERROR_COMM;
    }

    frame_bytes = 0;
    frame_bytes += ((fifo_cfg & BMI3_FIFO_ACC_EN) != 0U) ? BMI3_LENGTH_FIFO_ACC : 0U;
    frame_bytes += ((fifo_cfg & BMI3_FIFO_GYR_EN) != 0U) ? BMI3_LENGTH_FIFO_GYR : 0U;
    frame_bytes += ((fifo_cfg & BMI3_FIFO_TEMP_EN) != 0U) ? BMI3_LENGTH_TEMPERATURE : 0U;
    frame_bytes += ((fifo_cfg & BMI3_FIFO_TIME_EN) != 0U) ? BMI3_LENGTH_SENSOR_TIME : 0U;

    if (frame_bytes == 0U)
    {
        /* 未使能任何数据源，无数据可读 */
        return IMU_SUCCESS;
    }

    frame_words = (uint16_t)(frame_bytes / 2U);

    /* 单次读取能容纳的整帧数：扣除 dummy 字节后按帧对齐，受静态数组上限约束。 */
    frames_per_pass = (uint16_t)((IMU_FIFO_READ_BUF_SIZE - dev->dummy_byte) / frame_bytes);
    if (frames_per_pass > IMU_FIFO_FRAME_LIMIT)
    {
        frames_per_pass = IMU_FIFO_FRAME_LIMIT;
    }

    if (frames_per_pass == 0U)
    {
        /* 缓冲区不足以容纳 1 帧数据，配置异常 */
        return IMU_ERROR_PARAM;
    }

    /* 循环分块读取，直到读满 max_frames 或 FIFO 中已无完整帧 */
    total = 0;
    while (total < max_frames)
    {
        rslt = bmi325_get_fifo_length(&fifo_words, dev);
        if (rslt == BMI3_W_FIFO_EMPTY)
        {
            break;
        }

        if (rslt != BMI325_OK)
        {
            return IMU_ERROR_COMM;
        }

        /* 仅处理已落入 FIFO 的完整帧，残缺帧留待下次 */
        frames_avail = (uint16_t)(fifo_words / frame_words);
        if (frames_avail == 0U)
        {
            break;
        }

        /* 本次采纳帧数 = min(可用帧, 单次容量, 剩余需求) */
        frames_this = frames_avail;
        if (frames_this > frames_per_pass)
        {
            frames_this = frames_per_pass;
        }

        if (frames_this > (uint16_t)(max_frames - total))
        {
            frames_this = (uint16_t)(max_frames - total);
        }

        if (frames_this == 0U)
        {
            break;
        }

        /* 仅按本次计划输出帧数读取真实 FIFO 数据；
         * 解析阶段通过虚拟扩展 fifo.length 兜住最后 1 帧边界，
         * 避免为构造哨兵而把下一帧真实数据提前读走 */
        read_frames = frames_this;
        read_bytes = (uint16_t)((read_frames * frame_bytes) + dev->dummy_byte);
        /* 仅扩展解析边界，不增加真实 I2C 读取长度；
         * 最后一帧数据虽然已经读进来了，但原厂解析会把“刚好到末尾”的那一帧当成无效，
         * 所以这里人为多留出一点空间让它能被正常解析 */
        parse_len = (uint16_t)(read_bytes + frame_bytes);

        memset(&fifo, 0, sizeof(fifo));
        memset(s_acc_data, 0, sizeof(s_acc_data));
        memset(s_gyr_data, 0, sizeof(s_gyr_data));
        memset(s_temp_data, 0, sizeof(s_temp_data));

        fifo.data = fifo_buf;
        fifo.length = parse_len;
        /* available_fifo_len 为实读字宽，向上取整覆盖 dummy 偏移 */
        fifo.available_fifo_len = (uint16_t)((read_bytes + 1U) / 2U);

        rslt = bmi325_read_fifo_data(&fifo, dev);
        if (rslt != BMI325_OK)
        {
            return IMU_ERROR_COMM;
        }

        if ((fifo.available_fifo_sens & BMI3_FIFO_ACC_EN) != 0U)
        {
            (void)bmi325_extract_accel(s_acc_data, &fifo, dev);
        }

        if ((fifo.available_fifo_sens & BMI3_FIFO_GYR_EN) != 0U)
        {
            (void)bmi325_extract_gyro(s_gyr_data, &fifo, dev);
        }

        if ((fifo.available_fifo_sens & BMI3_FIFO_TEMP_EN) != 0U)
        {
            (void)bmi325_extract_temperature(s_temp_data, &fifo, dev);
        }

        /* 运动数据按 acc/gyr 的完整帧数输出；
         * 温度在 FIFO 里本来就是半速率采样，不能拿它去卡总帧数，
         * 否则会把正常的运动数据一起少读掉 */
        copy_cnt = read_frames;
        if (((fifo.available_fifo_sens & BMI3_FIFO_ACC_EN) != 0U) && (copy_cnt > fifo.avail_fifo_accel_frames))
        {
            copy_cnt = fifo.avail_fifo_accel_frames;
        }

        if (((fifo.available_fifo_sens & BMI3_FIFO_GYR_EN) != 0U) && (copy_cnt > fifo.avail_fifo_gyro_frames))
        {
            copy_cnt = fifo.avail_fifo_gyro_frames;
        }

        if (copy_cnt > frames_this)
        {
            copy_cnt = frames_this;     // 虚拟扩展边界仅用于解析尾帧，不计入输出
        }

        for (index = 0; index < copy_cnt; index++)
        {
            frames[total + index].acc_x = s_acc_data[index].x;
            frames[total + index].acc_y = s_acc_data[index].y;
            frames[total + index].acc_z = s_acc_data[index].z;
            frames[total + index].gyr_x = s_gyr_data[index].x;
            frames[total + index].gyr_y = s_gyr_data[index].y;
            frames[total + index].gyr_z = s_gyr_data[index].z;

            /* 温度为半速率数据，厂商已剔除 dummy 帧并压缩存放，其数量约为运动帧的一半。
             * 按比例映射到对应温度槽，缺失时退化到最后一个有效温度，避免错位 */
            if (fifo.avail_fifo_temp_frames > 0U)
            {
                tidx = (uint16_t)(index >> 1);              // 每 2 个运动帧对应 1 个温度帧
                if (tidx >= fifo.avail_fifo_temp_frames)
                {
                    tidx = (uint16_t)(fifo.avail_fifo_temp_frames - 1U);
                }

                frames[total + index].temperature = (int16_t)s_temp_data[tidx].temp_data;
            }
            else
            {
                frames[total + index].temperature = 0;
            }
        }

        total = (uint16_t)(total + copy_cnt);

        /* 实际采纳帧数少于预期，说明可解析数据已耗尽 */
        if (copy_cnt < frames_this)
        {
            break;
        }
    }

    *frame_count = total;

    return IMU_SUCCESS;
}

/********************************************************************
**函数名称:  imu_fifo_flush
**入口参数:  无
**出口参数:  无
**函数功能:  清空 BMI325 FIFO
**返回值:    IMU_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
imu_result_t imu_fifo_flush(void)
{
    return imu_write_reg_u16(BMI3_REG_FIFO_CTRL, BMI3_FIFO_FLUSH_MASK);
}

/********************************************************************
**函数名称:  imu_feature_enable
**入口参数:  feature  ---        特征类型（输入）
**           enable   ---        true 使能，false 禁用（输入）
**出口参数:  无
**函数功能:  使能或禁用 BMI325 feature engine 特征
**返回值:    IMU_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
imu_result_t imu_feature_enable(imu_feature_t feature, bool enable)
{
    uint16_t feature_val;
    uint16_t mask;
    imu_result_t ret;

    if (feature >= IMU_FEATURE_MAX)
    {
        return IMU_ERROR_PARAM;
    }

    mask = imu_feature_to_mask(feature);
    if (mask == 0U)
    {
        return IMU_ERROR_PARAM;
    }

    LOG_WRN("imu_feature_enable will reload feature engine and may clear the current step count");

    ret = imu_read_reg_u16(BMI3_REG_FEATURE_IO0, &feature_val);
    if (ret != IMU_SUCCESS)
    {
        return ret;
    }

    if (enable == true)
    {
        feature_val |= mask;
    }
    else
    {
        feature_val &= ~mask;
    }

    ret = imu_write_reg_u16(BMI3_REG_FEATURE_IO0, feature_val);
    if (ret != IMU_SUCCESS)
    {
        return ret;
    }

    return imu_write_reg_u16(BMI3_REG_FEATURE_IO_STATUS, BMI3_ENABLE);
}

/********************************************************************
**函数名称:  imu_step_counter_enable
**入口参数:  enable   ---        true 使能，false 禁用（输入）
**出口参数:  无
**函数功能:  显式使能或禁用 BMI325 计步器
**返回值:    IMU_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
imu_result_t imu_step_counter_enable(bool enable)
{
    struct bmi3_dev *dev;
    struct bmi3_feature_enable feature;
    imu_result_t ret;
    int8_t rslt;

    ret = imu_get_dev_checked(&dev);
    if (ret != IMU_SUCCESS)
    {
        return ret;
    }

    memset(&feature, 0, sizeof(feature));
    feature.step_counter_en = (enable == true) ? BMI3_ENABLE : BMI3_DISABLE;

    rslt = bmi325_select_sensor(&feature, dev);
    if (rslt != BMI325_OK)
    {
        return IMU_ERROR_COMM;
    }

    return IMU_SUCCESS;
}

/********************************************************************
**函数名称:  imu_get_step_count
**入口参数:  count    ---        步数存储指针（输出）
**出口参数:  count    ---        当前步数
**函数功能:  读取 BMI325 计步值
**返回值:    IMU_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
imu_result_t imu_get_step_count(uint32_t *count)
{
    struct bmi3_dev *dev;
    struct bmi3_sensor_data sensor_data;
    imu_result_t ret;
    int8_t rslt;

    if (count == NULL)
    {
        return IMU_ERROR_PARAM;
    }

    ret = imu_get_dev_checked(&dev);
    if (ret != IMU_SUCCESS)
    {
        return ret;
    }

    memset(&sensor_data, 0, sizeof(sensor_data));
    sensor_data.type = BMI3_STEP_COUNTER;

    rslt = bmi325_get_sensor_data(&sensor_data, 1, dev);
    if (rslt != BMI325_OK)
    {
        return IMU_ERROR_COMM;
    }

    *count = sensor_data.sens_data.step_counter_output;

    return IMU_SUCCESS;
}

/********************************************************************
**函数名称:  imu_set_axis_map
**入口参数:  axis_map ---        轴映射配置（输入）
**出口参数:  无
**函数功能:  设置 BMI325 轴映射
**返回值:    IMU_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
imu_result_t imu_set_axis_map(const struct imu_axis_map *axis_map)
{
    struct bmi3_dev *dev;
    struct bmi3_axes_remap remap_axis;
    imu_result_t ret;
    int8_t rslt;

    if ((axis_map == NULL) || (axis_map->axis_map > 5U))
    {
        return IMU_ERROR_PARAM;
    }

    ret = imu_get_dev_checked(&dev);
    if (ret != IMU_SUCCESS)
    {
        return ret;
    }

    remap_axis.axis_map = axis_map->axis_map;
    remap_axis.invert_x = (axis_map->invert_x == true) ? BMI3_ENABLE : BMI3_DISABLE;
    remap_axis.invert_y = (axis_map->invert_y == true) ? BMI3_ENABLE : BMI3_DISABLE;
    remap_axis.invert_z = (axis_map->invert_z == true) ? BMI3_ENABLE : BMI3_DISABLE;

    rslt = bmi325_set_remap_axes(remap_axis, dev);
    if (rslt != BMI325_OK)
    {
        return IMU_ERROR_COMM;
    }

    return IMU_SUCCESS;
}

/********************************************************************
**函数名称:  imu_get_axis_map
**入口参数:  axis_map ---        轴映射配置存储指针（输出）
**出口参数:  axis_map ---        当前轴映射配置
**函数功能:  读取 BMI325 轴映射
**返回值:    IMU_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
imu_result_t imu_get_axis_map(struct imu_axis_map *axis_map)
{
    struct bmi3_dev *dev;
    struct bmi3_axes_remap remap_axis;
    imu_result_t ret;
    int8_t rslt;

    if (axis_map == NULL)
    {
        return IMU_ERROR_PARAM;
    }

    ret = imu_get_dev_checked(&dev);
    if (ret != IMU_SUCCESS)
    {
        return ret;
    }

    memset(&remap_axis, 0, sizeof(remap_axis));
    rslt = bmi325_get_remap_axes(&remap_axis, dev);
    if (rslt != BMI325_OK)
    {
        return IMU_ERROR_COMM;
    }

    axis_map->axis_map = remap_axis.axis_map;
    axis_map->invert_x = (remap_axis.invert_x != 0U);
    axis_map->invert_y = (remap_axis.invert_y != 0U);
    axis_map->invert_z = (remap_axis.invert_z != 0U);

    return IMU_SUCCESS;
}

/********************************************************************
**函数名称:  imu_set_any_motion_config
**入口参数:  config   ---        ANY_MOTION 配置参数（输入）
**出口参数:  无
**函数功能:  配置 BMI325 ANY_MOTION 检测参数
**返回值:    IMU_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
imu_result_t imu_set_any_motion_config(const struct imu_any_motion_config *config)
{
    struct bmi3_dev *dev;
    struct bmi3_sens_config sens_cfg;
    imu_result_t ret;
    int8_t rslt;

    if (config == NULL)
    {
        return IMU_ERROR_PARAM;
    }

    /* 参数范围检查 */
    if (config->slope_thres > 4095U)
    {
        LOG_ERR("slope_thres %u out of range (0-4095)", config->slope_thres);
        return IMU_ERROR_PARAM;
    }

    if (config->duration > 8191U)
    {
        LOG_ERR("duration %u out of range (0-8191)", config->duration);
        return IMU_ERROR_PARAM;
    }

    if (config->hysteresis > 1023U)
    {
        LOG_ERR("hysteresis %u out of range (0-1023)", config->hysteresis);
        return IMU_ERROR_PARAM;
    }

    if (config->wait_time > 7U)
    {
        LOG_ERR("wait_time %u out of range (0-7)", config->wait_time);
        return IMU_ERROR_PARAM;
    }

    if (config->acc_ref_up > 1U)
    {
        LOG_ERR("acc_ref_up %u out of range (0-1)", config->acc_ref_up);
        return IMU_ERROR_PARAM;
    }

    ret = imu_get_dev_checked(&dev);
    if (ret != IMU_SUCCESS)
    {
        return ret;
    }

    /* 配置 ANY_MOTION 参数 */
    sens_cfg.type = BMI3_ANY_MOTION;
    sens_cfg.cfg.any_motion.slope_thres = config->slope_thres;
    sens_cfg.cfg.any_motion.duration = config->duration;
    sens_cfg.cfg.any_motion.hysteresis = config->hysteresis;
    sens_cfg.cfg.any_motion.wait_time = config->wait_time;
    sens_cfg.cfg.any_motion.acc_ref_up = config->acc_ref_up;

    /* 写入配置到 BMI325 */
    rslt = bmi325_set_sensor_config(&sens_cfg, 1, dev);
    if (rslt != BMI325_OK)
    {
        LOG_ERR("bmi325_set_sensor_config failed: %d", rslt);
        return IMU_ERROR_COMM;
    }

    LOG_INF("ANY_MOTION configured: slope_thres=%u, duration=%u, hysteresis=%u, wait_time=%u, acc_ref_up=%u",
            config->slope_thres, config->duration, config->hysteresis, config->wait_time, config->acc_ref_up);

    return IMU_SUCCESS;
}
