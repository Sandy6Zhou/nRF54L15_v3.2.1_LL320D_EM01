#ifndef __OMG_BATTERY_H__
#define __OMG_BATTERY_H__

#include <stdint.h>
#include <stdbool.h>

/********************************************************************************/
/*  					      common definitions 							 	*/
/********************************************************************************/
// -------- omg version type definitions ------
#define OMG_70101DB  							0
#define OMG_70101DC  							1
#define OMG_7020XXX  							2

// ---------- error code definitions ----------
#define OMG_ERROR_NONE							0
#define OMG_ERROR_IIC							-1
#define OMG_ERROR_CHIP_ID						-2
#define OMG_ERROR_NO_PROFILE					-3
#define OMG_ERROR_OTHER							-4


// -------- sleep status code definitions -----
#define OMG_IS_SLEEP							1
#define OMG_IS_NOT_SLEEP						0

// ------------type definitions ---------------
typedef uint8_t     							omg_uint8_t;
typedef int8_t       							omg_int8_t;
typedef uint16_t     							omg_uint16_t;
typedef int16_t      							omg_int16_t;
typedef uint32_t      							omg_uint32_t;
typedef int32_t       							omg_int32_t;
typedef uint64_t     							omg_uint64_t;
typedef int64_t       							omg_int64_t;

#define omg_true	   							true
#define omg_false	   							false
typedef bool 	 								omg_bool_t;

#define OMG_NULL								((void *)0)

#define OMG_ENABLE_FIXED_POINT               	1               // 0 disable; 1 enable fixed point math
#if OMG_ENABLE_FIXED_POINT
typedef omg_int32_t                           	omg_float_t;   // fixed point container (Q16.16 style when FRACTIONAL_BITS=16)
#define OMG_FLOAT_FRACTIONAL_BITS             	16u            // fractional precision bits
#define OMG_FLOAT_SCALE                       	((omg_int32_t)1 << OMG_FLOAT_FRACTIONAL_BITS)
#define OMG_FLOAT(x)                          	((omg_float_t)((x) * OMG_FLOAT_SCALE))
#define OMG_FLOAT_FROM_INT(x)                 	((omg_float_t)((omg_int32_t)(x) * OMG_FLOAT_SCALE))
#define OMG_FLOAT_TO_FLOAT(x)                 	(((float)(x) / OMG_FLOAT_SCALE))
#define OMG_FLOAT_TO_INT(x)                   	((omg_int32_t)((x) >> OMG_FLOAT_FRACTIONAL_BITS))
#define OMG_FLOAT_ROUND_TO_INT(x)             	((omg_int32_t)(((x) >= 0) ? \
												(((x) + (OMG_FLOAT_SCALE >> 1)) / OMG_FLOAT_SCALE) : \
												(((x) - (OMG_FLOAT_SCALE >> 1)) / OMG_FLOAT_SCALE)))
#define OMG_FLOAT_MUL(x, y)                   	((omg_float_t)(((omg_int64_t)(x) * (omg_int64_t)(y)) >> OMG_FLOAT_FRACTIONAL_BITS))
#define OMG_FLOAT_MUL_INT(x, y)               	((omg_float_t)((omg_int64_t)(x) * (omg_int64_t)(y)))
#define OMG_FLOAT_MUL_INT_TO_INT(x, y)        	((omg_int32_t)(((omg_int64_t)(x) * (omg_int64_t)(y)) >> OMG_FLOAT_FRACTIONAL_BITS))
#define OMG_FLOAT_DIV(x, y)                   	((omg_float_t)((((omg_int64_t)(x)) << OMG_FLOAT_FRACTIONAL_BITS) / (omg_int64_t)(y)))

#else
typedef float       							omg_float_t;   	// floating point type definition
#define OMG_FLOAT(x) 							(x)				// float to float
#define OMG_FLOAT_FROM_INT(x)       			((omg_float_t)(x))         			// int to float
#define OMG_FLOAT_TO_FLOAT(x)       			((float)(x))         				// float to float
#define OMG_FLOAT_TO_INT(x)       				((omg_int32_t)(x))		  			// float to int
#define OMG_FLOAT_ROUND_TO_INT(x)   			((omg_int32_t)(((x) >= 0.0f) ? ((x) + 0.5f) : ((x) - 0.5f)))
#define OMG_FLOAT_MUL(x, y)       				((x) * (y))         				// float multiply
#define OMG_FLOAT_MUL_INT(x, y)       			((x) * (y))         				// float multiply with int
#define OMG_FLOAT_MUL_INT_TO_INT(x, y)  		((omg_int32_t)((x) * (y)))         	// float multiply with int to int
#define OMG_FLOAT_DIV(x, y)       				((x) / (y))         				// float divide
#endif		

/* soc conversion */
#define OMG_SOC_UPPER_LIMIT   					((const omg_float_t)OMG_FLOAT(99.01f))
#define OMG_SOC_LOWER_LIMIT   					((const omg_float_t)OMG_FLOAT(0.6f))
#define OMG_SOC_FLOAT_TO_INT(omg_soc_float)     (((omg_soc_float) > OMG_SOC_UPPER_LIMIT) ? 100 : \
	 											((omg_soc_float) < OMG_SOC_LOWER_LIMIT) ? 0 :    \
	 											(OMG_FLOAT_ROUND_TO_INT((omg_soc_float))))

/********************************************************************************/
/*  					      driver version definitions                     	*/
/********************************************************************************/
#define OMG_USE_VERSION_TYPE					OMG_7020XXX
#define OMG_DRIVER_VERSION_STRING				"XX_OM70X0X_XX_v1.0"

#if 	OMG_USE_VERSION_TYPE == OMG_70101DB
#define OMG_PRODUCT_ID		0xA3				// OM70101DB:0xA3(not active)/0xA0(active)
#elif 	(OMG_USE_VERSION_TYPE == OMG_7020XXX) \
		|| (OMG_USE_VERSION_TYPE == OMG_70101DC)
#define OMG_PRODUCT_ID		0xB1		  		// OM70101DC:0xB1; om70201:0xB1
#else
#error "Please define a correct OMG_USE_VERSION_TYPE"
#endif

/********************************************************************************/
/*  					     user setting definitions 							*/
/********************************************************************************/
#if 	OMG_USE_VERSION_TYPE == OMG_7020XXX
#define OMG_USER_SENSING				10						// mohm
#define OMG_USE_SOH_HOST				1						// 0 use default soh calc; 1 use soh host + cycle cnt
#define OMG_SOH_ADJUST_ENABLE			1						// enable soh_gain in current_soc calculation
#endif

#ifndef OMG_SOH_ADJUST_ENABLE
#define OMG_SOH_ADJUST_ENABLE			0						
#endif

#ifndef OMG_USE_SOH_HOST
#define OMG_USE_SOH_HOST				0
#endif

/**
 * @brief VCELL voltage divider circuit configuration parameters
 * 
 * Battery voltage = VCELL * OMG_USER_VOLTAGE_MULTIPLE
 */
#define OMG_USER_VOLTAGE_MULTIPLE		1

// --------------temp definitions & select--------------
#define OMG_INTERNAL_TEMP		    			0   				// omg use the internal temp sensor
#define OMG_EXTERNAL_NTC_TEMP					1   				// omg use the external NTC temp sensor
#define OMG_USE_HOST_TEMP						2   				// omg use the temp returned by the host

#define OMG_USER_USE_TEMP_SOURCE_TYPE 			OMG_INTERNAL_TEMP		// which temp source to select
#define OMG_EN_INTEMP_ENABLE					1						// enable internal temperature sampling
#define OMG_EN_EXTEMP_ENABLE					0						// enable external NTC temperature sampling
#if OMG_EN_EXTEMP_ENABLE
#define OMG_NTC_RESISTANCE_AT_25C				100					// kohm (10 / 47 / 100)
#endif

#if defined(OMG_NTC_RESISTANCE_AT_25C) && (OMG_NTC_RESISTANCE_AT_25C == 100)
#define OMG_ENABLE_NTC_TEMP_COMPENSATION_100K		1
#else
#define OMG_ENABLE_NTC_TEMP_COMPENSATION_100K		0
#endif

/********************************************************************************/
/*  					     driver feature setting 							*/
/********************************************************************************/
#if OMG_USE_VERSION_TYPE == OMG_7020XXX

#define OMG_ENABLE_CURRENT_OFFSET 				0   		// 0 disable; 1 enable
#if 	OMG_ENABLE_CURRENT_OFFSET && (OMG_USER_SENSING < 10)
#define OMG_ENABLE_GET_CUR_SUB_OFFSET  			1
#else
#define OMG_ENABLE_GET_CUR_SUB_OFFSET  			0
#endif

#define OMG_ENABLE_CUR_ADJUST_PROFILE			0			// 0 disable; 1 enable
#define OMG_ENABLE_SOC_COMPENSATION				0			// 0 disable; 1 enable
#define OMG_ENABLE_FULL_SET_CHARGE_DETECTION 	0			// 0 disable; 1 enable

#else
#define OMG_ENABLE_SOC_COMPENSATION_RELAX_ANTI	0 		 	// 0 disable; 1 enable	
#define OMG_ENABLE_CHARGE_ADJUST_PROFILE    	0			// 0 disable; 1 enable
#endif

#define OMG_ENABLE_SHUTDOWN_VOL_LO				0			 // 0 disable; 1 enable
#if		OMG_ENABLE_SHUTDOWN_VOL_LO
#define OMG_ENABLE_SHUTDOWN_VOL_LO_CACHE		1			 // 0 disable; 1 enable
#endif

/**
 * @brief Enable or disable the logging feature within the OMG driver.
 * When enabled, the driver will provide detailed log information useful for debugging and development.
 * This feature should be disabled in production builds to optimize performance and reduce code size.
 */
#define OMG_ENABLE_LOG_FEATURE 					1
#define OMG_ENABLE_DUMP         				1
/********************************************************************************/
/*  					     battery profile definitions 						*/
/********************************************************************************/
#define OMG_MAX_BATTERY_PROFILE_PAIR_DATA_NUM 75
typedef struct 
{
	omg_uint8_t addr;
	omg_uint8_t value;
} OMG_PairDataType;

typedef struct
{
	omg_uint8_t ver;
	OMG_PairDataType pairData[OMG_MAX_BATTERY_PROFILE_PAIR_DATA_NUM];
	omg_uint8_t pairDataNum;
} OMG_BatteryProfileDataType;

#if OMG_ENABLE_CUR_ADJUST_PROFILE
typedef struct
{
	omg_int16_t *threshold;
	omg_uint8_t threshold_num;
	omg_uint8_t *addr;
	omg_uint8_t addr_num;
	omg_uint8_t **value;
} OMG_CurAdjustParamType;
#endif

#if OMG_ENABLE_SOC_COMPENSATION
#define OMG_CHARGE_EP_THRESHOLD_NUMBER	10
typedef struct
{
	omg_int16_t charge_cutoff_cur;
	omg_int16_t charge_soc_init_cur_hi;
	omg_int16_t charge_soc_init_cur_lo;
	omg_uint16_t charge_soc_init_vol;
	omg_int16_t resting_cur;
	omg_float_t charge_step;
	omg_float_t discharge_step;

	omg_float_t soc_threshold_start;
	omg_uint8_t soc[OMG_CHARGE_EP_THRESHOLD_NUMBER];
	omg_uint8_t coeff[OMG_CHARGE_EP_THRESHOLD_NUMBER];
} OMG_ChargeEndPointParamType;
#endif

#if defined(OMG_ENABLE_SOC_COMPENSATION_RELAX_ANTI) && (OMG_ENABLE_SOC_COMPENSATION_RELAX_ANTI == 1)
	typedef struct 
	{
		omg_float_t charge_step;
		omg_float_t discharge_step;
	} OMG_SocRelaxParamType;
#endif

#if defined(OMG_ENABLE_CHARGE_ADJUST_PROFILE) && (OMG_ENABLE_CHARGE_ADJUST_PROFILE == 1)
typedef struct {
	omg_uint8_t *addr;
	omg_uint8_t addr_num;
	omg_uint8_t **value;
} OMG_ChargeAdjustProfileParamType;
#endif

typedef struct
{
	OMG_BatteryProfileDataType profileData;

#if OMG_ENABLE_CURRENT_OFFSET
	omg_int8_t curOffsetLsb;
#endif

#if OMG_ENABLE_CUR_ADJUST_PROFILE
	OMG_CurAdjustParamType curAdjustParam;
#endif

#if OMG_ENABLE_SOC_COMPENSATION
	OMG_ChargeEndPointParamType chargeEndPointParam;
#endif

#if defined(OMG_ENABLE_SOC_COMPENSATION_RELAX_ANTI) && (OMG_ENABLE_SOC_COMPENSATION_RELAX_ANTI == 1)
	OMG_SocRelaxParamType socRelaxParam;
#endif

#if defined(OMG_ENABLE_CHARGE_ADJUST_PROFILE) && (OMG_ENABLE_CHARGE_ADJUST_PROFILE == 1)
	OMG_ChargeAdjustProfileParamType chargeAdjustProfileParam;
#endif

} OMG_FuelgaugeParamType;

/********************************************************************************/
/*  					       omg driver functions                     		*/
/********************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

int omg_init();
int omg_get_id(omg_uint8_t* id);
int omg_get_vol(omg_uint16_t* vol);	// mV
int omg_get_temp(omg_int8_t* temp);	// degree C
int omg_get_soc(omg_uint8_t* soc);	// 1%
int omg_sleep();
int omg_active();

#if OMG_USE_VERSION_TYPE == OMG_7020XXX
int omg_get_cur(omg_int16_t* cur);	// mA
int omg_get_soh(omg_uint8_t* soh);	// 1%
int omg_get_cycle_cnt(omg_uint16_t* cycle_cnt);	//1
int omg_set_soh(const omg_uint8_t soh);
int omg_set_cycle_cnt(const omg_uint16_t cycle_cnt);
#if OMG_USE_SOH_HOST
int omg_set_soh_by_cycle_cnt(const omg_uint16_t cycle_cnt);
#endif
#endif

#if OMG_USER_USE_TEMP_SOURCE_TYPE == OMG_USE_HOST_TEMP
int omg_set_temp(const omg_int8_t temp); // -45 <= temp <= 85 degree C
#endif


typedef struct {
	omg_uint16_t vol;          /**< Voltage in mV */
	omg_int8_t temp;          /**< Temperature in degree C */
	omg_uint8_t soc;          /**< Displayed SOC (%) */
	omg_uint16_t vsoc;		  /**< VSOC derived from internal voltage (debug) */
	omg_uint16_t rsoc;		  /**< Raw SOC read from register 0x04 (debug) */
#if OMG_USE_VERSION_TYPE == OMG_7020XXX
	omg_int16_t cur;          /**< Current in mA */
	omg_uint8_t soh;          /**< State of Health in percentage */
	omg_uint16_t cycle_cnt;   /**< Cycle Count */
#endif
} OMG_LogDataType, *POMG_LogDataType;
/**
 * @brief Preferred function for updating debug log information during development.
 * if OMG_ENABLE_LOG_FEATURE is enabled, this function prints the current log data.
 */
POMG_LogDataType omg_update_log(void);

#ifdef __cplusplus
}
#endif

#endif  // __OMG_BATTERY_H__
