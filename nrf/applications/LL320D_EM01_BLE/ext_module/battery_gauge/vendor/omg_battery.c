#include "omg_battery.h"
#include "omg_impl.h"

/********************************************************************************************/
/*  					          register definitions 						        		*/
/********************************************************************************************/
#define REG_PRODUCT_ID                          0x00
#define REG_EN_CONF				                0x01
#define REG_VCELL_H                             0x02
#define REG_VCELL_L                             0x03
#define REG_SOC_H             	                0x04
#define REG_SOC_L		                        0x05
#define REG_SOC_VOLT_H                          0xAA
#define REG_SOC_VOLT_L		                    0xAB

#define REG_TEMP_EXTERNAL                       0x06
#define REG_TEMP_INTERNAL                       0x07
#define REG_T_HOST            	                0xA0
#if OMG_USER_USE_TEMP_SOURCE_TYPE == OMG_EXTERNAL_NTC_TEMP
#define REG_TEMP				                REG_TEMP_EXTERNAL
#elif OMG_USER_USE_TEMP_SOURCE_TYPE == OMG_INTERNAL_TEMP
#define REG_TEMP				                REG_TEMP_INTERNAL
#else
#define REG_TEMP				                REG_T_HOST
#endif
#define REG_CONFIG         		                0x08
#define REG_I2C_CONFIG			                0x09
#define REG_INT_CONFIG                          0x0A
#define REG_SOC_ALERT                           0x0B
#define REG_TEMP_MAX                            0x0C
#define REG_TEMP_MIN                            0x0D
#define REG_CURRENT_H                           0x0E
#define REG_CURRENT_L                           0x0F
#define REG_USER_CONF                           0xA1
#define REG_CYCLE_H                             0xA4
#define REG_CYCLE_L                             0xA5
#define REG_SOH                                 0xA6

#define REG_CONFIG_CYCLE_CNT_INIT_MASK	        0x10
#define REG_CONFIG_SOH_INIT_MASK		        0x08
#define REG_CONFIG_SOC_INIT_MASK		        0x04
#define REG_CONFIG_ACTIVE_MODE_MASK		        0x02

#define REG_EN_CONF_DEFAULT_VALUE	            0x41
#define REG_EN_CONF_INT_TEMP_MASK	            0x01
#define REG_EN_CONF_EXT_TEMP_MASK	            0x02
#define REG_EN_CONF_T_HOST_EN_MASK	            0x08
#define REG_EN_CONF_SOH_HOST_EN_MASK            0x10   /* bit4 */
#define REG_EN_CONF_SOH_ADJUST_EN_MASK          0x20   /* bit5 */

#if OMG_EN_EXTEMP_ENABLE
#  if   OMG_NTC_RESISTANCE_AT_25C == 10
#    define REG_EN_CONF_TS_MODE_VALUE           0x40   /* 2'b01, R25=10k  */
#  elif OMG_NTC_RESISTANCE_AT_25C == 47
#    define REG_EN_CONF_TS_MODE_VALUE           0x80   /* 2'b10, R25=47k  */
#  elif OMG_NTC_RESISTANCE_AT_25C == 100
#    define REG_EN_CONF_TS_MODE_VALUE           0xC0   /* 2'b11, R25=100k */
#  else
#    error "Unsupported OMG_NTC_RESISTANCE_AT_25C value for TS_MODE"
#  endif
#else
#  define REG_EN_CONF_TS_MODE_VALUE             0x00   /* 2'b00, OUTNTC disabled */
#endif

#define REG_EN_CONF_VALUE \
	( REG_EN_CONF_TS_MODE_VALUE \
	| (OMG_USE_SOH_HOST ? REG_EN_CONF_SOH_HOST_EN_MASK : 0) \
	| (OMG_SOH_ADJUST_ENABLE ? REG_EN_CONF_SOH_ADJUST_EN_MASK : 0) \
	| ((OMG_USER_USE_TEMP_SOURCE_TYPE == OMG_USE_HOST_TEMP) ? REG_EN_CONF_T_HOST_EN_MASK : 0) \
	| (OMG_EN_EXTEMP_ENABLE ? REG_EN_CONF_EXT_TEMP_MASK : 0) \
	| (OMG_EN_INTEMP_ENABLE ? REG_EN_CONF_INT_TEMP_MASK : 0) )

#define REG_I2C_CONFIG_POR_ANALOG_READY_MASK	0x04


/********************************************************************************************/
/*  					          global feature definitions 						        */
/********************************************************************************************/
#define MAX_T_HOST				                85
#define MIN_T_HOST				                -40

#define MAX_SOH					                100
#define MIN_SOH					                60

#define MAX_CYCLE_CNT			                1000

#define TEMP_EXPANSION		                    10
#define SOC_EXPANSION		                    100
#define CUR_EXPANSION		                    10

#if OMG_USE_VERSION_TYPE == OMG_7020XXX
static omg_uint8_t cache_soh = 0;		    //soh value want to write back, only om7020X support
static omg_uint16_t cache_cycle_cnt = 0;	//cycle cnt value want to write back, only om7020X support
#endif

#if OMG_ENABLE_SOC_COMPENSATION
#define OMG_MAX(x, y)               (((x) > (y)) ? (x) : (y))
#define OMG_MIN(x, y)               (((x) < (y)) ? (x) : (y))
#define TIMER_COUNT_MAX		        3

static omg_uint16_t g_cache_voltage = 0;
static omg_bool_t g_used_voltage = omg_true;
static omg_int16_t g_cache_current = 0;
static omg_bool_t g_used_current = omg_true;
#endif

#if (OMG_ENABLE_SOC_COMPENSATION) || \
    (OMG_ENABLE_SOC_COMPENSATION_RELAX_ANTI)
#define SOC_MAX				        100
#define SOC_MIN				        0
#define SOC_DIFF_MAX		        15
#define REG_SAVE_DSOC				REG_T_HOST
#define REG_SAVE_DSOC_DEFAULT_VALUE	0x82
static omg_uint8_t g_cache_dsoc = REG_SAVE_DSOC_DEFAULT_VALUE;
#endif

/********************************************************************************************/
/*  					          global feature calculations 						        */
/********************************************************************************************/
#define CALC_VOL(v)			    ((v) * OMG_USER_VOLTAGE_MULTIPLE * 5 / 16)	//mV
#define CALC_CUR(i)			    ((i) * 16 / 10 / OMG_USER_SENSING)	//mA
#if OMG_ENABLE_FIXED_POINT
/* reg t (0..255) -> Q16.16( t/2 - 40 ) = (t<<15) - (40<<16) */
#define CALC_TEMP_FROM_REG(t)   ((omg_float_t)(((omg_int32_t)(t) << (OMG_FLOAT_FRACTIONAL_BITS-1)) - OMG_FLOAT_FROM_INT(40)))
/* reg soc16=percent*256 -> Q16.16(percent) = soc16<<8 */
#define CALC_SOC_FROM_REG(s)    ((omg_float_t)((omg_int32_t)(s) << (OMG_FLOAT_FRACTIONAL_BITS-8)))
#else
#define CALC_TEMP(t)		    ((t) / 2 - 40)	// degree C
#define CALC_SOC(soc)		    ((soc) / 256)
#endif

#define CALC_CYCLE_CNT(cnt)	    ((cnt) / 32)
#define CALC_CYCLE_CNT_INVERSE(cnt)	(cnt * 32)
#define CALC_TEMP_INVERSE(t)	((t) * 2 + 80)

#define CALC_SOH_I(cycle)       (MAX_SOH - (cycle) / 25)	// 500 cycle <-> soh 80%
#define CALC_SOH(cycle)	        (((cycle) > MAX_CYCLE_CNT) ? MIN_SOH : CALC_SOH_I(cycle))	

/********************************************************************************************/
/*  					          global driver definitions 						        */
/********************************************************************************************/
/* Profile write verify control
 * 1: enable write-after-read verify with up to 3 retries
 * 0: disable
 */
#if OMG_USE_VERSION_TYPE > OMG_70101DB
#define OMG_ENABLE_PROFILE_WRITE_VERIFY 1
#define OMG_PROFILE_VERIFY_RETRY_MAX  	3
#endif

/********************************************************************************************/
/*  					          battery profile definitions 						        */
/********************************************************************************************/
const OMG_FuelgaugeParamType g_OMGFuelgauge_param = {
	.profileData = {
    // Config_10m_ohm_10kNTC
	.ver = 0x01,
    .pairData = {
			{0x20, 0x00},
			{0x21, 0x01},
			{0x22, 0x03},
			{0x23, 0x04},
			{0x24, 0x07},
			{0x25, 0x0B},
			{0x26, 0x11},
			{0x27, 0x17},
			{0x28, 0x1D},
			{0x29, 0x24},
			{0x2A, 0x2C},
			{0x2B, 0x35},
			{0x2C, 0x3F},
			{0x2D, 0x4B},
			{0x2E, 0x59},
			{0x2F, 0x66},
			{0x30, 0x70},
			{0x31, 0x79},
			{0x32, 0x80},
			{0x33, 0x87},
			{0x34, 0x8E},
			{0x35, 0x97},
			{0x36, 0xA0},
			{0x37, 0xAB},
			{0x38, 0xB6},
			{0x39, 0xC0},
			{0x3A, 0xC8},
			{0x3B, 0xD1},
			{0x3C, 0xDE},
			{0x3D, 0xF1},
			{0x3E, 0xF9},
			{0x3F, 0xFD},
			{0x40, 0xFF},
			{0x41, 0x3E},
			{0x42, 0x0B},
			{0x43, 0x07},
			{0x44, 0x07},
			{0x45, 0x0C},
			{0x46, 0x0C},
			{0x47, 0x0C},
			{0x48, 0x06},
			{0x49, 0x0F},
			{0x4E, 0x64},
			{0x4F, 0xDC},
			{0x50, 0x08},
			{0x51, 0x77},
			{0x52, 0x0A},
			{0x53, 0x6E},
			{0x54, 0x0E},
			{0x55, 0x02},
			{0x5A, 0x00},
			{0x5B, 0x03},
			{0x5C, 0x50},
			{0x5D, 0x0B},
			{0x5E, 0x0A},
			{0x5F, 0x80},
			{0x60, 0x6E},
			{0x61, 0xFF},
			{0x62, 0xFB},
			{0x63, 0x50},
			{0x64, 0x1E},
			{0x65, 0x6C},
			{0x66, 0xDC},
			{0x67, 0x10},
			{0x68, 0x32},
			{0x69, 0xDC},
			{0x6A, 0x96},
			{0x80, 0x61},
			{0x01, REG_EN_CONF_VALUE},
#if OMG_USER_USE_TEMP_SOURCE_TYPE == OMG_INTERNAL_TEMP
			{0x0A, 0x80 & 0x7F},
#else
			{0x0A, 0x80},
#endif
			{0x96, 0x01},
#if OMG_USER_USE_TEMP_SOURCE_TYPE == OMG_INTERNAL_TEMP
			{0x87, 0x0D},
#else
			{0x87, 0x4D},
#endif
			{0x10, 0x93},
#if defined(OMG_ENABLE_SHUTDOWN_VOL_LO) && (OMG_ENABLE_SHUTDOWN_VOL_LO == 1)
			{0x08, 0x00},
			{0x9B, 0x40},
#endif
		},
		.pairDataNum = 73 + OMG_ENABLE_SHUTDOWN_VOL_LO * 2,
	},
#if OMG_ENABLE_CURRENT_OFFSET
	.curOffsetLsb = -2,
#endif

#if OMG_ENABLE_CUR_ADJUST_PROFILE
	.curAdjustParam = {
		.threshold = (omg_int16_t[]){-500},
		.threshold_num = 1,
		.addr = (omg_uint8_t[]){0x55, 0x5B, 0x5C, 0x62},
		.addr_num = 4,
		.value = (omg_uint8_t*[]){
			(omg_uint8_t[]){0xF4, 0x03, 0x9E, 0xE2},	// 大电流 小容量
			(omg_uint8_t[]){0x03, 0x03, 0x64, 0xF3},	// 小电流 大容量
		},
	},
#endif

#if OMG_ENABLE_SOC_COMPENSATION
	.chargeEndPointParam = {
		.charge_cutoff_cur = 60,
		.charge_soc_init_cur_hi = 58,
		.charge_soc_init_cur_lo = 50,
		.charge_soc_init_vol = 4200,    	// cv charge voltage
		.resting_cur = 10,
		.charge_step = (const omg_float_t)OMG_FLOAT(0.84f),	// deltaSoc = cur * time / 标称容量 * 100%
		.discharge_step = (const omg_float_t)OMG_FLOAT(1),
		.soc_threshold_start = (const omg_float_t)OMG_FLOAT(90),
		.soc = {100, 99, 98, 97, 96, 95, 94, 93, 92, 91},
		.coeff = {100, 90, 80, 70, 60, 50, 40, 30, 19, 8},
	},
#endif

#if defined(OMG_ENABLE_SOC_COMPENSATION_RELAX_ANTI) && (OMG_ENABLE_SOC_COMPENSATION_RELAX_ANTI == 1)
	.socRelaxParam = {
		.charge_step = 0.2,
		.discharge_step = 0.4,
	},
#endif

#if defined(OMG_ENABLE_CHARGE_ADJUST_PROFILE) && (OMG_ENABLE_CHARGE_ADJUST_PROFILE == 1)
	.chargeAdjustProfileParam = {
		.addr = (omg_uint8_t[]){0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x55},
		.addr_num = 10,
		.value = (omg_uint8_t*[]){	
			(omg_uint8_t[]){0x19, 0x02, 0x01, 0x01, 0x02, 0x03, 0x04, 0x03, 0x05, 0x07},  //discharge
			(omg_uint8_t[]){0x64, 0x04, 0x02, 0x01, 0x03, 0x13, 0x04, 0x07, 0x09, 0x07},  //charge	
		},
	},
#endif
};

#if OMG_ENABLE_PROFILE_WRITE_VERIFY
static inline omg_uint8_t _omg_profile_expected_read(const omg_uint8_t addr, const omg_uint8_t written)
{
	switch(addr)
	{
		case 0x53: return 0x00;
		case 0x60: return 0x00;
		case 0x87: return 0x0D;
		case 0x9B: return 0x61;
		default:   return written;
	}
}
#endif

static int _omg_set_battery_profile(const OMG_BatteryProfileDataType* profile_data)
{
	if(!profile_data)
	{
		return OMG_ERROR_NO_PROFILE;
	}
	
	for(int i = 0; i < profile_data->pairDataNum; i++)
	{
		const OMG_PairDataType* one = &profile_data->pairData[i];

		int ret = _omg_write_byte(one->addr, one->value);
		if(ret < 0)
		{
			return OMG_ERROR_IIC;
		}

#if OMG_ENABLE_PROFILE_WRITE_VERIFY
		// Read-back verify with up to OMG_PROFILE_VERIFY_RETRY_MAX retries
		omg_uint8_t read_val = 0;
		int verify_ok = 0;
		const omg_uint8_t expect_val = _omg_profile_expected_read(one->addr, one->value);
		for(int t = 0; t <= OMG_PROFILE_VERIFY_RETRY_MAX; t++)
		{
			int r = _omg_read_byte(one->addr, &read_val);
			if(r >= 0 && read_val == expect_val)
			{
				verify_ok = 1;
				break;
			}
			if(t < OMG_PROFILE_VERIFY_RETRY_MAX)
			{
				_omg_write_byte(one->addr, one->value);
			}
		}
		if(!verify_ok)
		{
			return OMG_ERROR_IIC;
		}
#endif
	}
	
	return OMG_ERROR_NONE;
}

#if OMG_ENABLE_CURRENT_OFFSET
#define SOC_FIX_EFUSE_CTRL_0 0xC0
#define SOC_FIX_EFUSE_CTRL_1 0xC1
#define SOC_FIX_EFUSE_READ_DONE_MASK 0x01
#define SOC_FIX_EFUSE_PRELOAD_EN_MASK 0x80
#define SOC_FIX_OFFSET_H 0xC6
#define SOC_FIX_OFFSET_L 0xC5
#define SOC_FIX_OFFSET_L_LSB_MASK  ((0x10 + g_OMGFuelgauge_param.curOffsetLsb) << 4)    // (16-2) << 4
#define SOC_FIX_OFFSET_L_OTHER_MASK  0x0F
#define SOC_FIX_OFFSET_L_SHIFT_BIT 4
#define SOC_FIX_OFFSET_BYTE_MAX    0xFF
static int _omg_set_cur_offset()
{
	omg_uint8_t ctrl_value;
	
    //1. wait until EFUSE_READ_DONE(register 0xC0 bit0) is 1, about 22ms
    /*do
    {
        _omg_read_byte(SOC_FIX_EFUSE_CTRL_0, &ctrl_value);
    } while ((ctrl_value & SOC_FIX_EFUSE_READ_DONE_MASK) == 0);*/
	
	const omg_uint8_t max_try_times = 3;
	const omg_uint32_t ms = 25;
	omg_uint8_t finish_upload = 0;
	for(omg_uint8_t try_times = 0; try_times < max_try_times; try_times ++)
	{
		_omg_delay_ms(ms);
		
		_omg_read_byte(SOC_FIX_EFUSE_CTRL_0, &ctrl_value);
		if(ctrl_value & SOC_FIX_EFUSE_READ_DONE_MASK)
		{
			finish_upload = 1;
			break;
		}
	}
	if(finish_upload == 0)
	{
		return OMG_ERROR_OTHER;
	}

    //2. read offset value in register 0xC5 and set -2(0xE0 | other, actually)
    //0xC5{[7:4](offset), [3:0](other)}
    omg_uint8_t offset;
    _omg_read_byte(SOC_FIX_OFFSET_L, &offset);
    offset &= SOC_FIX_OFFSET_L_OTHER_MASK;
    offset |= SOC_FIX_OFFSET_L_LSB_MASK;

    //3. set EFUSE_PRELOAD_EN(register 0xC1 bit7) to 1
    ctrl_value = 0;
    ctrl_value |= SOC_FIX_EFUSE_PRELOAD_EN_MASK;
    _omg_write_byte(SOC_FIX_EFUSE_CTRL_1, ctrl_value);

    //4. write new offset value in register 0xC6(0xFF) and 0xC5
    _omg_write_byte(SOC_FIX_OFFSET_H, SOC_FIX_OFFSET_BYTE_MAX);
    _omg_write_byte(SOC_FIX_OFFSET_L, offset);

    //5. set EFUSE_PRELOAD_EN(register 0xC1 bit7) to 0
    ctrl_value &= ~SOC_FIX_EFUSE_PRELOAD_EN_MASK;
    _omg_write_byte(SOC_FIX_EFUSE_CTRL_1, ctrl_value);
	
	return OMG_ERROR_NONE;
}
#endif

#if OMG_USER_USE_TEMP_SOURCE_TYPE == OMG_USE_HOST_TEMP
int omg_set_temp(const omg_int8_t temp)
{
	if(temp < MIN_T_HOST || temp > MAX_T_HOST)
	{
		return OMG_ERROR_OTHER;
	}
	
	const omg_int16_t tmp = (const omg_int16_t)temp;
	omg_uint8_t value = (omg_uint8_t)CALC_TEMP_INVERSE(tmp);
	return _omg_write_byte(REG_T_HOST, value);
}
#endif

#if defined(OMG_ENABLE_SHUTDOWN_VOL_LO) && (OMG_ENABLE_SHUTDOWN_VOL_LO == 1)
static int _omg_check_sleep()
{
	omg_uint8_t i2c_config;
	int ret = _omg_read_byte(REG_I2C_CONFIG, &i2c_config);
	if(ret < 0)
	{
		return OMG_IS_SLEEP;
	}
	
	return (i2c_config & REG_I2C_CONFIG_POR_ANALOG_READY_MASK) ? OMG_IS_NOT_SLEEP : OMG_IS_SLEEP;
}

static int _omg_wait_analog_ready()
{
#define TRY_TIMES 3
#define DELAY_MS 35
	for(int i = 0; i < TRY_TIMES; i ++) 
	{
		_omg_delay_ms(DELAY_MS);
		
		int sleep = _omg_check_sleep();
		if(sleep == OMG_IS_NOT_SLEEP) 
		{
			return OMG_ERROR_NONE;
		}
	}
	
	return OMG_ERROR_OTHER;
}
#endif

int omg_init()
{
	omg_uint8_t id;
	int ret = omg_get_id(&id);
	if(ret < 0)
	{
		return ret;
	}
	
	omg_uint8_t user_config;
	ret = _omg_read_byte(REG_USER_CONF, &user_config);
	if(ret < 0)
	{
		return OMG_ERROR_IIC;
	}
	
	const OMG_BatteryProfileDataType *pParam = &g_OMGFuelgauge_param.profileData;
	if(user_config == pParam->ver)	//write profile already && ver not change, only change to active mode
	{
#if (OMG_ENABLE_SOC_COMPENSATION) || \
    (OMG_ENABLE_SOC_COMPENSATION_RELAX_ANTI)
		ret = _omg_read_byte(REG_SAVE_DSOC, &g_cache_dsoc);
		if(ret < 0)
		{
			return OMG_ERROR_IIC;
		}
#endif
		return omg_active();
	}
	
	ret = _omg_set_battery_profile(pParam);
	if(ret < 0)
	{
		return ret;
	}

	// set OMG_USE_SOH_HOST

	// set OMG_USER_USE_TEMP_SOURCE_TYPE
	
	omg_uint8_t configValue = REG_CONFIG_ACTIVE_MODE_MASK | REG_CONFIG_SOC_INIT_MASK;
	
#if OMG_USE_VERSION_TYPE == OMG_7020XXX
#if OMG_USE_SOH_HOST == 1
	// cache_cycle_cnt = xxx();	// get saved cycle cnt from flash 
#else
	// cache_soh = xxx();	// get saved soh from flash
#endif
	
	if(cache_soh > 0)
	{
		ret = omg_set_soh(cache_soh);
		if(ret < 0)
		{
			return OMG_ERROR_IIC;
		}
		configValue |= REG_CONFIG_SOH_INIT_MASK;
	}
	
	if(cache_cycle_cnt > 0)
	{
		ret = omg_set_cycle_cnt(cache_cycle_cnt);
		if(ret < 0)
		{
			return OMG_ERROR_IIC;
		}
		configValue |= REG_CONFIG_CYCLE_CNT_INIT_MASK;
	}
#endif
	
	ret = _omg_write_byte(REG_CONFIG, configValue);
	if(ret < 0)
	{
		return OMG_ERROR_IIC;
	}

#if OMG_ENABLE_CURRENT_OFFSET
	ret = _omg_set_cur_offset();
	if(ret < 0)
	{
		return ret;
	}
#endif
	
#if defined(OMG_ENABLE_SHUTDOWN_VOL_LO) && (OMG_ENABLE_SHUTDOWN_VOL_LO == 1)
	ret = _omg_wait_analog_ready();
	if(ret < 0)
	{
		return OMG_ERROR_OTHER;
	}
#endif

	ret = _omg_write_byte(REG_USER_CONF, pParam->ver);
	if(ret < 0)
	{
		return OMG_ERROR_IIC;
	}
	
	_omg_delay_ms(100);	//OM fuel gauge need 70~100ms to prepare the 1st right data(vol/cur/soc etc...)
	
	return OMG_ERROR_NONE;
}

int omg_get_id(omg_uint8_t* id)
{
	int ret = _omg_read_byte(REG_PRODUCT_ID, id);
	if(ret < 0)
	{
		return OMG_ERROR_IIC;
	}

#if OMG_USE_VERSION_TYPE == OMG_70101DB
	if(*id >= 0xA0 && *id <= 0xA3)
	{
		return OMG_ERROR_NONE;
	}
	else
	{
		return OMG_ERROR_CHIP_ID;
	}
#elif OMG_USE_VERSION_TYPE == OMG_70101DC || OMG_USE_VERSION_TYPE == OMG_7020XXX
	if(*id != OMG_PRODUCT_ID)
	{
		return OMG_ERROR_CHIP_ID;
	}

#endif
	
	return OMG_ERROR_NONE;
}

int omg_get_vol(omg_uint16_t* vol)
{
	omg_uint16_t reg_vol;
	int ret = _omg_read_word(REG_VCELL_H, &reg_vol);
	if(ret < 0)
	{
		return OMG_ERROR_IIC;
	}
	
	*vol = (omg_uint16_t)CALC_VOL((omg_uint32_t)(reg_vol));

#if defined(OMG_ENABLE_SHUTDOWN_VOL_LO_CACHE) && (OMG_ENABLE_SHUTDOWN_VOL_LO_CACHE == 1)
	static omg_uint16_t last_vol = 0;
    if(*vol == 0 && _omg_check_sleep() == OMG_IS_SLEEP)
    {
        *vol = last_vol;
    }
    else
    {
        last_vol = *vol;
    }
#endif

#if OMG_ENABLE_SOC_COMPENSATION
	if(g_used_voltage) {
		g_cache_voltage = *vol;
		g_used_voltage = omg_false;
	}
#endif
	return OMG_ERROR_NONE;
}

#if OMG_ENABLE_NTC_TEMP_COMPENSATION_100K
//temperature compensation parameter(100K NTC)
#define TEMP_FIXED_KEY0			((const omg_float_t)OMG_FLOAT(-6.5f))
#define TEMP_FIXED_KEY1 		((const omg_float_t)OMG_FLOAT(3.0f))
#define TEMP_FIXED_KEY2 		((const omg_float_t)OMG_FLOAT(38.0f))
#define TEMP_FIXED_KEY3 		((const omg_float_t)OMG_FLOAT(51.5f))
#define TEMP_FIXED_GAIN0    	((const omg_float_t)OMG_FLOAT(0.6222f))
#define TEMP_FIXED_GAIN1    	((const omg_float_t)OMG_FLOAT(0.0f))
#define TEMP_FIXED_GAIN2    	((const omg_float_t)OMG_FLOAT(-0.1457f))
#define TEMP_FIXED_GAIN3    	((const omg_float_t)OMG_FLOAT(0.0f))
#define TEMP_FIXED_GAIN4    	((const omg_float_t)OMG_FLOAT(0.4764f))
#define TEMP_FIXED_OFFSET0   	((const omg_float_t)OMG_FLOAT(6.0444f))
#define TEMP_FIXED_OFFSET1   	((const omg_float_t)OMG_FLOAT(2.0f))
#define TEMP_FIXED_OFFSET2   	((const omg_float_t)OMG_FLOAT(2.4371f))
#define TEMP_FIXED_OFFSET3   	((const omg_float_t)OMG_FLOAT(-3.1f))
#define TEMP_FIXED_OFFSET4   	((const omg_float_t)OMG_FLOAT(-27.633f))
static omg_float_t _omg_temp_fixed(const omg_float_t temp)
{
    omg_float_t diff;
    if (temp <= TEMP_FIXED_KEY0) {
        diff = OMG_FLOAT_MUL(TEMP_FIXED_GAIN0, temp) + TEMP_FIXED_OFFSET0; 
    } else if (temp <= TEMP_FIXED_KEY1) {
        diff = OMG_FLOAT_MUL(TEMP_FIXED_GAIN1, temp) + TEMP_FIXED_OFFSET1;
    } else if (temp <= TEMP_FIXED_KEY2) {
        diff = OMG_FLOAT_MUL(TEMP_FIXED_GAIN2, temp) + TEMP_FIXED_OFFSET2;
    } else if (temp <= TEMP_FIXED_KEY3) {
        diff = OMG_FLOAT_MUL(TEMP_FIXED_GAIN3, temp) + TEMP_FIXED_OFFSET3;
    } else {
        diff = OMG_FLOAT_MUL(TEMP_FIXED_GAIN4, temp) + TEMP_FIXED_OFFSET4;
    }

    return temp + diff;
}
#endif

int omg_get_temp(omg_int8_t* temp)
{
	omg_uint8_t register_value;
	int ret = _omg_read_byte(REG_TEMP, &register_value);
	if(ret < 0)
	{
		return OMG_ERROR_IIC;
	}
#if OMG_ENABLE_FIXED_POINT
	omg_float_t temp_omg_float = CALC_TEMP_FROM_REG(register_value);
#else
	omg_float_t register_value_omg_float = (omg_float_t)OMG_FLOAT(register_value);
	omg_float_t temp_omg_float = CALC_TEMP(register_value_omg_float);
#endif
#if OMG_ENABLE_NTC_TEMP_COMPENSATION_100K
	temp_omg_float = _omg_temp_fixed(temp_omg_float);
#endif

	*temp = (omg_int8_t)OMG_FLOAT_ROUND_TO_INT(temp_omg_float);

#if defined(OMG_ENABLE_SHUTDOWN_VOL_LO_CACHE) && (OMG_ENABLE_SHUTDOWN_VOL_LO_CACHE == 1)
#define TEMP_RESET_VALUE				25
#define TEMP_REGISTER_RESET_VALUE		0x82
	static omg_int8_t last_temp = TEMP_RESET_VALUE;
    if(register_value == TEMP_REGISTER_RESET_VALUE && _omg_check_sleep() == OMG_IS_SLEEP)
    {
        *temp = last_temp;
    }
    else
    {
        last_temp = *temp;
    }
#endif

	return OMG_ERROR_NONE;
}

#if OMG_ENABLE_CUR_ADJUST_PROFILE
static int _omg_adjust_cap(const omg_int16_t cur)
{
	const OMG_CurAdjustParamType *pParam = &g_OMGFuelgauge_param.curAdjustParam;
	static omg_uint8_t using_index = 0;
	omg_uint8_t index = 0;
	for(; index < pParam->threshold_num; index ++)
	{
		if(cur < pParam->threshold[index])
		{
			break;
		}
	}
	
	int ret;
	if(using_index != index)
	{
		for(omg_uint8_t i = 0; i < pParam->addr_num; i ++)
		{
			ret = _omg_write_byte(pParam->addr[i], pParam->value[index][i]);
			if(ret < 0)
			{
				return OMG_ERROR_IIC;
			}
		}
		
		using_index = index;
	}
	
	return OMG_ERROR_NONE;
}
#endif

#if defined(OMG_ENABLE_CHARGE_ADJUST_PROFILE) && (OMG_ENABLE_CHARGE_ADJUST_PROFILE == 1)
#define REG_FVOL 0xA2
#define FVOL_TREND_THRESHOLD_RAW  8   /* raw reg units – tuneable */
static omg_bool_t _omg_get_charge_status_from_fvol()
{
	static omg_uint16_t ref_reg_fvol = 0;
	static omg_bool_t  charge_state = omg_false;   /* false = discharge, true = charge */
	static omg_uint8_t  inited = 0;

	omg_uint16_t reg_fvol;
	int ret = _omg_read_word(REG_FVOL, &reg_fvol);
	if(ret < 0)
	{
		return charge_state;        /* keep last known state on error */
	}

	if(!inited)
	{
		ref_reg_fvol = reg_fvol;
		inited       = 1;
		return charge_state;        /* default: discharge on first call */
	}

	omg_int16_t delta = (omg_int16_t)reg_fvol - (omg_int16_t)ref_reg_fvol;

	if(delta >= (omg_int16_t)FVOL_TREND_THRESHOLD_RAW)
	{
		charge_state = omg_true;             /* charging  */
		ref_reg_fvol = reg_fvol;             /* update reference */
	}
	else if(delta <= -(omg_int16_t)FVOL_TREND_THRESHOLD_RAW)
	{
		charge_state = omg_false;            /* discharging */
		ref_reg_fvol = reg_fvol;             /* update reference */
	}

	return charge_state;
}

static int _omg_adjust_charge_profile()
{
	const OMG_ChargeAdjustProfileParamType *pParam = &g_OMGFuelgauge_param.chargeAdjustProfileParam;
	static omg_uint8_t using_index = 0;
	omg_uint8_t index = 0;
	omg_bool_t is_charging = _omg_get_charge_status_from_fvol();
	index = is_charging ? 1 : 0;

	int ret;
	if(using_index != index)
	{
		for(omg_uint8_t i = 0; i < pParam->addr_num; i++)
		{
			ret = _omg_write_byte(pParam->addr[i], pParam->value[index][i]);
			if(ret < 0)
			{
				return OMG_ERROR_IIC;
			}
		}
		using_index = index;
	}
	return OMG_ERROR_NONE;
}
#endif

#if OMG_ENABLE_SOC_COMPENSATION
static omg_float_t _omg_soc_compensation(const omg_float_t rsoc, const omg_int16_t cur, const omg_uint16_t vol)
{
	static omg_uint8_t soc_init_flag = 0;

	static omg_int16_t min_cur_step;
	static omg_int16_t SOC_cur = 0;

	omg_float_t dsoc = OMG_FLOAT(-1);
	static omg_float_t last_dsoc = OMG_FLOAT(-1);

	if(last_dsoc < OMG_FLOAT(0))
	{
		omg_float_t cache_dsoc = OMG_FLOAT(g_cache_dsoc);
		omg_float_t soc_diff = (cache_dsoc > rsoc) ? (cache_dsoc - rsoc) : (rsoc - cache_dsoc);
		if(cache_dsoc <= OMG_FLOAT(SOC_MAX) && soc_diff <= OMG_FLOAT(SOC_DIFF_MAX))
		{
			last_dsoc = cache_dsoc;
		}
		else
		{
			last_dsoc = rsoc;
		}
		
		return last_dsoc;
	}
	
	const OMG_ChargeEndPointParamType *pParam = &g_OMGFuelgauge_param.chargeEndPointParam;
	if(cur > pParam->resting_cur)   // charge
	{		
		if(rsoc >= pParam->soc_threshold_start) // 90 
		{
			if(SOC_cur == 0)
			{
				SOC_cur = cur;
				min_cur_step = (SOC_cur - pParam->charge_cutoff_cur) / 100;
			}
			
			for(int i = 0; i < OMG_CHARGE_EP_THRESHOLD_NUMBER; i++)
			{
				if(cur <= SOC_cur - min_cur_step * pParam->coeff[i])
				{
					dsoc = OMG_FLOAT(pParam->soc[i]);
					break;
				}
			}
			
			if(dsoc < 0)
			{
				dsoc = pParam->soc_threshold_start;
			}
		}
		else
		{
			dsoc = rsoc;
		}
		
		dsoc = dsoc > last_dsoc ? (last_dsoc + pParam->charge_step) : last_dsoc;
		dsoc = OMG_MIN(dsoc, OMG_FLOAT(SOC_MAX));

		if(vol > pParam->charge_soc_init_vol 
			&& cur < pParam->charge_soc_init_cur_hi 
			&& cur > pParam->charge_soc_init_cur_lo 
			&& soc_init_flag == 0 
			&& (dsoc < OMG_FLOAT(SOC_MAX) || rsoc < OMG_FLOAT(SOC_MAX)))
		{
			int ret = _omg_write_byte(REG_CONFIG, REG_CONFIG_ACTIVE_MODE_MASK | REG_CONFIG_SOC_INIT_MASK);
			if(ret < 0)
			{

				//return OMG_ERROR_IIC;
			}
			else
			{
				soc_init_flag = 1;
				dsoc = OMG_FLOAT(SOC_MAX);
			}
		}
	}
	else
	{	
		dsoc = rsoc < last_dsoc ? (last_dsoc - pParam->discharge_step) : last_dsoc;
		dsoc = OMG_MAX(dsoc, OMG_FLOAT(SOC_MIN));

		if(dsoc < pParam->soc_threshold_start)
		{
			SOC_cur = 0;
			soc_init_flag = 0;
		}
	}
	
	static omg_uint8_t last_dsoc_save_value = REG_SAVE_DSOC_DEFAULT_VALUE;
	omg_uint8_t dsoc_save_value = (omg_uint8_t)OMG_SOC_FLOAT_TO_INT(dsoc);
	if(dsoc_save_value != last_dsoc_save_value)
	{
		int ret = _omg_write_byte(REG_SAVE_DSOC, dsoc_save_value);
		if(ret < 0)
		{
			//return OMG_ERROR_IIC;
		}
		else
		{
			last_dsoc_save_value = dsoc_save_value;
		}
	}

	last_dsoc = dsoc;
	return dsoc;
}
#endif

#if OMG_ENABLE_FULL_SET_CHARGE_DETECTION
#define REG_SOC_CONTROL      0x66
#define REG_SOC_CONTROL_CURR_CHARGE_DET_EN_MASK  0x10
static int _omg_full_set_charge_detection(const omg_uint8_t soc)
{
	static omg_uint8_t last_soc_100_status = 255; 
	omg_uint8_t current_soc_100_status = (soc == 100) ? 1 : 0;
	
	if(last_soc_100_status != current_soc_100_status)
	{
		omg_uint8_t reg_value;
		int ret = _omg_read_byte(REG_SOC_CONTROL, &reg_value);
		if(ret < 0)
		{
			return OMG_ERROR_IIC;
		}
		
		if(current_soc_100_status)
		{
			reg_value &= ~REG_SOC_CONTROL_CURR_CHARGE_DET_EN_MASK;
		}
		else
		{
			reg_value |= REG_SOC_CONTROL_CURR_CHARGE_DET_EN_MASK;
		}
		
		ret = _omg_write_byte(REG_SOC_CONTROL, reg_value);
		if(ret < 0)
		{
			return OMG_ERROR_IIC;
		}
		
		last_soc_100_status = current_soc_100_status;
	}
	
	return OMG_ERROR_NONE;
}
#endif

#if defined(OMG_ENABLE_SOC_COMPENSATION_RELAX_ANTI) && (OMG_ENABLE_SOC_COMPENSATION_RELAX_ANTI == 1)
static omg_float_t _OMG_soc_compensation_relax(const omg_float_t rsoc)
{
	omg_float_t dsoc = OMG_FLOAT(-1);
	static omg_float_t last_dsoc = OMG_FLOAT(-1);
	if(last_dsoc < OMG_FLOAT(0))
	{
		omg_float_t cache_dsoc = OMG_FLOAT(g_cache_dsoc);
		omg_float_t soc_diff = (cache_dsoc > rsoc) ? (cache_dsoc - rsoc) : (rsoc - cache_dsoc);
		if(cache_dsoc <= OMG_FLOAT(SOC_MAX) && soc_diff <= OMG_FLOAT(SOC_DIFF_MAX))
		{
			last_dsoc = cache_dsoc;
		}
		else
		{
			last_dsoc = rsoc;
		}
		
		return last_dsoc;
	}
	
	dsoc = last_dsoc;
	const OMG_SocRelaxParamType *pParam = &g_OMGFuelgauge_param.socRelaxParam;
	const omg_bool_t is_charging = _omg_get_charge_status();
	if(!is_charging)
	{
		if(rsoc < dsoc)
		{
			dsoc -= pParam->discharge_step;
			if(dsoc < rsoc)
			{
				dsoc = rsoc;
			}
		}
	}
	else
	{
		if(rsoc >= dsoc)
		{
			dsoc += pParam->charge_step;
			if(dsoc >= rsoc)
			{
				dsoc = rsoc;
			}
		}
	}

	static omg_uint8_t last_dsoc_save_value = REG_SAVE_DSOC_DEFAULT_VALUE;
	omg_uint8_t dsoc_save_value = (omg_uint8_t)OMG_SOC_FLOAT_TO_INT(dsoc);
	if(dsoc_save_value != last_dsoc_save_value)
	{
		int ret = _omg_write_byte(REG_SAVE_DSOC, dsoc_save_value);
		if(ret < 0)
		{
			//return OMG_ERROR_IIC;
		}
		else
		{
			last_dsoc_save_value = dsoc_save_value;
		}
	}

	last_dsoc = dsoc;
	return dsoc;
}
#endif

int omg_get_soc(omg_uint8_t* soc)
{
	int ret;
	omg_uint16_t register_value;
	ret = _omg_read_word(REG_SOC_H, &register_value);
	if(ret < 0)
	{
		return OMG_ERROR_IIC;
	}

#if defined(OMG_ENABLE_SHUTDOWN_VOL_LO_CACHE) && (OMG_ENABLE_SHUTDOWN_VOL_LO_CACHE == 1)
	static omg_uint8_t last_soc = 0;
    if(register_value == 0 && _omg_check_sleep() == OMG_IS_SLEEP)
    {
        *soc = last_soc;
		return OMG_ERROR_NONE;
    }
#endif

#if OMG_ENABLE_FIXED_POINT
	omg_float_t soc_float = CALC_SOC_FROM_REG(register_value);
#else
	omg_float_t register_value_float = (omg_float_t)OMG_FLOAT(register_value);
	omg_float_t soc_float = CALC_SOC(register_value_float);
#endif

#if OMG_ENABLE_SOC_COMPENSATION
	omg_int16_t cur; omg_uint16_t vol;
	if(g_used_current)
	{
		ret = omg_get_cur(&cur);
		if(ret < 0)
		{
			return OMG_ERROR_IIC;
		}
		g_used_current = omg_true;
	}
	else
	{
		cur = g_cache_current;
		g_used_current = omg_true;
	}
	if(g_used_voltage)
	{
		ret = omg_get_vol(&vol);
		if(ret < 0)
		{
			return OMG_ERROR_IIC;
		}
		g_used_voltage = omg_true;
	}
	else
	{
		vol = g_cache_voltage;
		g_used_voltage = omg_true;
	}
	soc_float = _omg_soc_compensation(soc_float, cur, vol);
#endif

#if defined(OMG_ENABLE_SOC_COMPENSATION_RELAX_ANTI) && (OMG_ENABLE_SOC_COMPENSATION_RELAX_ANTI == 1)
	soc_float = _OMG_soc_compensation_relax(soc_float);
#endif

	*soc = (omg_uint8_t)OMG_SOC_FLOAT_TO_INT(soc_float);

#if defined(OMG_ENABLE_SHUTDOWN_VOL_LO_CACHE) && (OMG_ENABLE_SHUTDOWN_VOL_LO_CACHE == 1)
	last_soc = *soc;
#endif

#if OMG_ENABLE_FULL_SET_CHARGE_DETECTION
	_omg_full_set_charge_detection(*soc);
#endif

#if defined(OMG_ENABLE_CHARGE_ADJUST_PROFILE) && (OMG_ENABLE_CHARGE_ADJUST_PROFILE == 1)
	_omg_adjust_charge_profile();
#endif

	return OMG_ERROR_NONE;
}

int omg_sleep()
{
	int ret;
	ret = _omg_write_byte(REG_CONFIG, 0);
	if(ret < 0)
	{
		return ret;
	}

	omg_uint8_t reg_val;
	ret = _omg_read_byte(0x09, &reg_val);
	if(ret < 0)
	{
		return ret;
	}
	if(reg_val & (1 << 2))
	{
		return OMG_ERROR_OTHER;		/* bit2=1: not yet in sleep */
	}
	return OMG_ERROR_NONE;			/* bit2=0: sleep confirmed */
}

int omg_active()
{
	return _omg_write_byte(REG_CONFIG, REG_CONFIG_ACTIVE_MODE_MASK);
}

#if OMG_USE_VERSION_TYPE == OMG_7020XXX
int omg_get_cur(omg_int16_t* cur)
{
	omg_uint16_t register_value_u16;
	int ret = _omg_read_word(REG_CURRENT_H, &register_value_u16);
	if(ret < 0)
	{
		return OMG_ERROR_IIC;
	}
	
	omg_int16_t register_value_s16 = (omg_int16_t)(register_value_u16);//0xFFFF
	omg_int32_t register_value_s32 = (omg_int32_t)(register_value_s16);
#if defined(OMG_ENABLE_CURRENT_OFFSET) && (OMG_ENABLE_CURRENT_OFFSET == 1) && (OMG_ENABLE_GET_CUR_SUB_OFFSET == 1)
	omg_int32_t cur32;
	omg_int32_t offset = g_OMGFuelgauge_param.curOffsetLsb;
	cur32 = CALC_CUR(register_value_s32 * 100 + offset * 141) / 100;
#else
	omg_int32_t cur32 = CALC_CUR(register_value_s32);
#endif

	*cur = (omg_int16_t)cur32;

#if OMG_ENABLE_CUR_ADJUST_PROFILE
	_omg_adjust_cap(*cur);
#endif

#if defined(OMG_ENABLE_SHUTDOWN_VOL_LO_CACHE) && (OMG_ENABLE_SHUTDOWN_VOL_LO_CACHE == 1)
	static omg_int16_t last_cur = 0;
    if(register_value_u16 == 0 && _omg_check_sleep() == OMG_IS_SLEEP)
    {
        *cur = last_cur;
    }
    else
    {
        last_cur = *cur;
    }
#endif

#if OMG_ENABLE_SOC_COMPENSATION
	if(g_used_current) {
		g_cache_current = *cur;
		g_used_current = omg_false;
	}
#endif
	return OMG_ERROR_NONE;
}

int omg_get_soh(omg_uint8_t* soh)
{
	int ret = _omg_read_byte(REG_SOH, soh);
	if(ret < 0)
	{
		return OMG_ERROR_IIC;
	}
	
#if OMG_USE_SOH_HOST == 0
	//save soh to flash at proper time
#endif
	return OMG_ERROR_NONE;
}

int omg_get_cycle_cnt(omg_uint16_t* cycle_cnt)
{
	int ret = _omg_read_word(REG_CYCLE_H, cycle_cnt);
	if(ret < 0)
	{
		return OMG_ERROR_IIC;
	}
	
	*cycle_cnt = CALC_CYCLE_CNT(*cycle_cnt);
	return OMG_ERROR_NONE;
}

int omg_set_soh(const omg_uint8_t soh)
{
	return _omg_write_byte(REG_SOH, soh);
}

int omg_set_cycle_cnt(const omg_uint16_t cycle_cnt)
{
	const omg_uint16_t cycle_cnt_register_value = CALC_CYCLE_CNT_INVERSE(cycle_cnt);
	return _omg_write_word(REG_CYCLE_H, cycle_cnt_register_value);
}

#if OMG_USE_SOH_HOST
int omg_set_soh_by_cycle_cnt(const omg_uint16_t cycle_cnt)
{	
	static omg_uint8_t last_soh = 100;
	omg_uint8_t soh = CALC_SOH(cycle_cnt);
	if(soh == last_soh)
	{
		return OMG_ERROR_NONE;
	}
	
	int ret = omg_set_soh(soh);
	if(ret < 0)
	{
		return OMG_ERROR_IIC;
	}
	
	//save cycle cnt to flash
	
	last_soh = soh;
	return OMG_ERROR_NONE;
}
#endif
#endif  // OMG_USE_VERSION_TYPE == OMG_7020XXX

/********************************************************************************************/
/*  					          update or log debug api 						        	*/
/********************************************************************************************/
#if OMG_ENABLE_LOG_FEATURE

#define OMG_DUMP_LINE_BUF_SIZE  64

static inline void _omg_hex2_u8(const omg_uint8_t v, char out2[2])
{
	static const char hex[] = "0123456789ABCDEF";
	out2[0] = hex[(v >> 4) & 0x0Fu];
	out2[1] = hex[v & 0x0Fu];
}

static inline void _omg_buf_append_char(char* buf, omg_uint16_t* idx, const omg_uint16_t max, const char c)
{
	if(!buf || !idx || max == 0)
	{
		return;
	}
	if(*idx + 1 >= max)
	{
		return;
	}
	buf[*idx] = c;
	(*idx)++;
	buf[*idx] = '\0';
}

static inline void _omg_buf_append_str(char* buf, omg_uint16_t* idx, const omg_uint16_t max, const char* s)
{
	if(!buf || !idx || !s)
	{
		return;
	}
	while(*s)
	{
		_omg_buf_append_char(buf, idx, max, *s++);
	}
}

static inline void _omg_buf_append_hex2(char* buf, omg_uint16_t* idx, const omg_uint16_t max, const omg_uint8_t v)
{
	char h[2];
	_omg_hex2_u8(v, h);
	_omg_buf_append_char(buf, idx, max, h[0]);
	_omg_buf_append_char(buf, idx, max, h[1]);
}

/* trigger dump after OMG_DUMP_TRIGGER_COUNT calls */
#define OMG_DUMP_TRIGGER_COUNT  1000
int omg_dump(void)
{
	static omg_uint32_t s_log_call_count = OMG_DUMP_TRIGGER_COUNT;
	/* trigger dump after OMG_DUMP_TRIGGER_COUNT calls */
	s_log_call_count++;
	if (s_log_call_count < OMG_DUMP_TRIGGER_COUNT)
	{
		return OMG_ERROR_NONE;
	}
#if !OMG_ENABLE_DUMP
    int ret = 0;
    omg_uint8_t reg0x08 = 0xFF;
    omg_uint8_t reg0x09 = 0xFF;
    omg_uint8_t reg0x5B = 0xFF;
    omg_uint8_t reg0x5C = 0xFF;
    omg_uint8_t reg0xA1 = 0xFF;

    ret |= _omg_read_byte(0x08, &reg0x08);
    ret |= _omg_read_byte(0x09, &reg0x09);
    ret |= _omg_read_byte(0x5B, &reg0x5B);
    ret |= _omg_read_byte(0x5C, &reg0x5C);
    ret |= _omg_read_byte(0xA1, &reg0xA1);
    if (ret < 0)
    {
        return OMG_ERROR_IIC;
    }
    _omg_log_debug("Dump Regs: 0x08=0x%02X, 0x09=0x%02X, 0x5B=0x%02X, 0x5C=0x%02X, 0xA1=0x%02X\n",
                  reg0x08, reg0x09, reg0x5B, reg0x5C, reg0xA1);
	s_log_call_count = 0;
    return OMG_ERROR_NONE;
#else // OMG_ENABLE_DUMP
#define OMG_DUMP_START_ADDR     0x00
#define OMG_DUMP_END_ADDR       0xEF
	int ret = OMG_ERROR_NONE;
    omg_uint8_t row;
	omg_uint8_t col;
	omg_uint8_t data = 0;
	const omg_uint8_t start = (omg_uint8_t)OMG_DUMP_START_ADDR;
	const omg_uint8_t end   = (omg_uint8_t)OMG_DUMP_END_ADDR;

	if (end < start)
	{
		return OMG_ERROR_OTHER;
	}

	char line[OMG_DUMP_LINE_BUF_SIZE];
	omg_uint16_t idx = 0;
	line[0] = '\0';
	_omg_buf_append_str(line, &idx, (omg_uint16_t)sizeof(line), "reg");
	for (col = 0; col < 16; col++)
	{
		_omg_buf_append_char(line, &idx, (omg_uint16_t)sizeof(line), ' ');
		_omg_buf_append_hex2(line, &idx, (omg_uint16_t)sizeof(line), col);
	}
	_omg_buf_append_char(line, &idx, (omg_uint16_t)sizeof(line), '\n');
	_omg_log_debug("%s", line);

	for (row = (start & 0xF0u); row <= end; row += 0x10u)
	{
		idx = 0;
		line[0] = '\0';
		_omg_buf_append_hex2(line, &idx, (omg_uint16_t)sizeof(line), (omg_uint8_t)(row & 0xF0u));
		_omg_buf_append_char(line, &idx, (omg_uint16_t)sizeof(line), ':');
		for (col = 0; col < 16; col++)
		{
			omg_uint8_t addr = (omg_uint8_t)(row + col);
			if (addr < start || addr > end)
			{
				_omg_buf_append_str(line, &idx, (omg_uint16_t)sizeof(line), "   ");
				continue;
			}

			if (_omg_read_byte(addr, &data) < 0)
			{
				/* mark read error as XX */
				_omg_buf_append_str(line, &idx, (omg_uint16_t)sizeof(line), " XX");
				ret = OMG_ERROR_IIC;
			}
			else
			{
				_omg_buf_append_char(line, &idx, (omg_uint16_t)sizeof(line), ' ');
				_omg_buf_append_hex2(line, &idx, (omg_uint16_t)sizeof(line), data);
			}
		}
		_omg_buf_append_char(line, &idx, (omg_uint16_t)sizeof(line), '\n');
		_omg_log_debug("%s", line);
		if (end - row < 0x10u)
		{
			break;
		}
	}

	s_log_call_count = 0;
	return ret;
#endif // OMG_ENABLE_DUMP
}

#endif  // OMG_ENABLE_LOG_FEATURE

static int omg_get_vsoc(omg_uint16_t* vsoc)
{
	int ret;
	omg_uint16_t register_value;
	ret = _omg_read_word(REG_SOC_VOLT_H, &register_value);
	if(ret < 0)
	{
		return OMG_ERROR_IIC;
	}
	*vsoc = (omg_uint16_t)((omg_uint32_t)register_value * 100 / 256);
	return OMG_ERROR_NONE;
}

static int omg_get_rsoc(omg_uint16_t* rsoc)
{
	int ret;
	omg_uint16_t register_value;
	ret = _omg_read_word(REG_SOC_H, &register_value);
	if(ret < 0)
	{
		return OMG_ERROR_IIC;
	}
	*rsoc = (omg_uint16_t)((omg_uint32_t)register_value * 100 / 256);
	return OMG_ERROR_NONE;
}

static OMG_LogDataType g_omg_log_data;

POMG_LogDataType omg_update_log(void) 
{
	int ret = 0;
	g_omg_log_data.vol = 0xFFFF;
	g_omg_log_data.temp = 0x7F;
	g_omg_log_data.soc = 0xFF;
	g_omg_log_data.vsoc = 0xFF;
	g_omg_log_data.rsoc = 0xFF;

	ret = omg_get_vol(&g_omg_log_data.vol);
	if(ret != OMG_ERROR_NONE) {
		_omg_log_error("omg_get_vol error\n");
		return OMG_NULL;
	}

	ret = omg_get_temp(&g_omg_log_data.temp);
	if(ret != OMG_ERROR_NONE) {
		_omg_log_error("omg_get_temp error\n");
		return OMG_NULL;
	}

	ret = omg_get_soc(&g_omg_log_data.soc);
	if(ret != OMG_ERROR_NONE) {
		_omg_log_error("omg_get_soc error\n");
		return OMG_NULL;
	}

	ret = omg_get_vsoc(&g_omg_log_data.vsoc);
	if(ret != OMG_ERROR_NONE) {
		_omg_log_error("omg_get_vsoc error\n");
		return OMG_NULL;
	}

	ret = omg_get_rsoc(&g_omg_log_data.rsoc);
	if(ret != OMG_ERROR_NONE) {
		_omg_log_error("omg_get_rsoc error\n");
		return OMG_NULL;
	}

#if OMG_USE_VERSION_TYPE == OMG_7020XXX
	g_omg_log_data.cur = 0x7FFF;
	g_omg_log_data.soh = 0xFF;
	g_omg_log_data.cycle_cnt = 0xFFFF;

	ret = omg_get_cur(&g_omg_log_data.cur);
	if(ret != OMG_ERROR_NONE) {
		_omg_log_error("omg_get_cur error\n");
		return OMG_NULL;
	}

	ret = omg_get_soh(&g_omg_log_data.soh);
	if(ret != OMG_ERROR_NONE) {
		_omg_log_error("omg_get_soh error\n");
		return OMG_NULL;
	}

	ret = omg_get_cycle_cnt(&g_omg_log_data.cycle_cnt);
	if (ret != OMG_ERROR_NONE) {
		_omg_log_error("omg_get_cycle_cnt error\n");
		return OMG_NULL;
	}

#if OMG_USE_SOH_HOST
	ret = omg_set_soh_by_cycle_cnt(g_omg_log_data.cycle_cnt);
	if(ret != OMG_ERROR_NONE) {
		_omg_log_error("omg_set_soh_by_cycle_cnt error\n");
		return OMG_NULL;
	}
#endif

	_omg_log_info("vol=%d, cur=%d, temp=%d, dsoc=%d, vsoc=%d, rsoc=%d, soh=%d, cycle=%d\n",
		g_omg_log_data.vol, g_omg_log_data.cur, g_omg_log_data.temp, g_omg_log_data.soc, g_omg_log_data.vsoc, 
        g_omg_log_data.rsoc, g_omg_log_data.soh, g_omg_log_data.cycle_cnt);
#else
	_omg_log_info("vol=%d, temp=%d, dsoc=%d, vsoc=%d, rsoc=%d\n",
		g_omg_log_data.vol, g_omg_log_data.temp, g_omg_log_data.soc, g_omg_log_data.vsoc, g_omg_log_data.rsoc);
#endif


#if defined(OMG_LOG_LEVEL) && (OMG_LOG_LEVEL >= OMG_LOG_LEVEL_DEBUG)
	omg_dump();
#endif

	return &g_omg_log_data;
}
