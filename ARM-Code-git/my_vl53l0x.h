/*
This library is from pololu/vl53l0x-arduino
license applied by pololu and STMICROELECTRONICS INTERNATIONAL N.V.
(https://github.com/pololu/vl53l0x-arduino)

Modified by Tomertomism, to be able to use in C environment
*/
/*

https://github.com/tomertomism
Copyright 2024 Tomertomism

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#ifndef my_vl53l0x_h
#define my_vl53l0x_h
#include <stm32f10x.h>
#include <stdbool.h>

#define SYSRANGE_START																0x00
#define SYSTEM_THRESH_HIGH  													0x0C
#define SYSTEM_THRESH_LOW   													0x0E
#define SYSTEM_SEQUENCE_CONFIG												0x01
#define SYSTEM_RANGE_CONFIG														0x09
#define SYSTEM_INTERMEASUREMENT_PERIOD  							0x04
#define SYSTEM_INTERRUPT_CONFIG_GPIO									0x0A
#define GPIO_HV_MUX_ACTIVE_HIGH 											0x84
#define SYSTEM_INTERRUPT_CLEAR  											0x0B
#define RESULT_INTERRUPT_STATUS 											0x13
#define RESULT_RANGE_STATUS 													0x14
#define RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN					0xBC
#define RESULT_CORE_RANGING_TOTAL_EVENTS_RTN					0xC0
#define RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF					0xD0
#define RESULT_CORE_RANGING_TOTAL_EVENTS_REF					0xD4
#define RESULT_PEAK_SIGNAL_RATE_REF 									0xB6
#define ALGO_PART_TO_PART_RANGE_OFFSET_MM   					0x28
#define I2C_SLAVE_DEVICE_ADDRESS											0x8A
#define MSRC_CONFIG_CONTROL 													0x60
#define PRE_RANGE_CONFIG_MIN_SNR											0x27
#define PRE_RANGE_CONFIG_VALID_PHASE_LOW							0x56
#define PRE_RANGE_CONFIG_VALID_PHASE_HIGH   					0x57
#define PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT  					0x64
#define FINAL_RANGE_CONFIG_MIN_SNR  									0x67
#define FINAL_RANGE_CONFIG_VALID_PHASE_LOW  					0x47
#define FINAL_RANGE_CONFIG_VALID_PHASE_HIGH 					0x48
#define FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT 	0x44
#define PRE_RANGE_CONFIG_SIGMA_THRESH_HI							0x61
#define PRE_RANGE_CONFIG_SIGMA_THRESH_LO							0x62
#define PRE_RANGE_CONFIG_VCSEL_PERIOD   							0x50
#define PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI  					0x51
#define PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO  					0x52
#define SYSTEM_HISTOGRAM_BIN													0x81
#define HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT   			0x33
#define HISTOGRAM_CONFIG_READOUT_CTRL   							0x55
#define FINAL_RANGE_CONFIG_VCSEL_PERIOD 							0x70
#define FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI					0x71
#define FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO					0x72
#define CROSSTALK_COMPENSATION_PEAK_RATE_MCPS   			0x20
#define MSRC_CONFIG_TIMEOUT_MACROP  									0x46
#define SOFT_RESET_GO2_SOFT_RESET_N 									0xBF
#define IDENTIFICATION_MODEL_ID 											0xC0
#define IDENTIFICATION_REVISION_ID  									0xC2
#define OSC_CALIBRATE_VAL   													0xF8
#define GLOBAL_CONFIG_VCSEL_WIDTH   									0x32
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_0							0xB0
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_1							0xB1
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_2							0xB2
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_3							0xB3
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_4							0xB4
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_5							0xB5
#define GLOBAL_CONFIG_REF_EN_START_SELECT   					0xB6
#define DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD 					0x4E
#define DYNAMIC_SPAD_REF_EN_START_OFFSET							0x4F
#define POWER_MANAGEMENT_GO1_POWER_FORCE							0x80
#define VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV   						0x89
#define ALGO_PHASECAL_LIM   													0x30
#define ALGO_PHASECAL_CONFIG_TIMEOUT									0x30

#define VL53L0X_Mode_Normal 													(u8)0
#define VL53L0X_Mode_High_Speed 											(u8)1
#define VL53L0X_Mode_Long_Range 											(u8)2
#define VL53L0X_Mode_High_Accuracy								 		(u8)3

typedef enum{
	VcselPeriodPreRange,
	VcselPeriodFinalRange
}VL53L0X_vcselPeriodType;

typedef struct{
	u8 address;					//Address of sensor
	u16 io_timeout;				//timeout time in ms
	bool io_2V8;				//2.8v io
	bool Use_GPIO;				//gpio1 on sensor
	GPIO_TypeDef *GPIO_PORT;	//which port does mcu receive 
	u16 GPIO_PIN;				//which pin does mcu receive 
	bool Use_TIM;				//use timer input?

	u8 VL53L0X_Mode;
}VL53L0X_InitTypeDef;

typedef struct{
	bool tcc;
	bool msrc;
	bool dss;
	bool pre_range;
	bool final_range;
}VL53L0X_SequenceStepEnables;

typedef struct{
	u16 pre_range_vcsel_period_pclks;
	u16 final_range_vcsel_period_pclks;
	u16 msrc_dss_tcc_mclks;
	u16 pre_range_mclks;
	u16 final_range_mclks;
	u32 msrc_dss_tcc_us;
	u32 pre_range_us;
	u32 final_range_us;
}VL53L0X_SequenceStepTimeouts;

bool VL53L0X_Init(VL53L0X_InitTypeDef*);

void VL53L0X_writeReg(u8, u8, u8);
void VL53L0X_writeReg16Bit(u8, u8, u16);
void VL53L0X_writeReg32Bit(u8, u8, u32);
u8 VL53L0X_readReg(u8, u8);
u16 VL53L0X_readReg16Bit(u8, u8);
u32 VL53L0X_readReg32Bit(u8, u8);

void VL53L0X_writeMulti(u8, u8, u8 const*, u8);
void VL53L0X_readMulti(u8, u8, u8*, u8);

bool VL53L0X_setSignalRateLimit(VL53L0X_InitTypeDef*, float);
float VL53L0X_getSignalRateLimit(VL53L0X_InitTypeDef*);

bool VL53L0X_setMeasurementTimingBudget(VL53L0X_InitTypeDef*, u32);
u32 VL53L0X_getMeasurementTimingBudget(VL53L0X_InitTypeDef*);

bool VL53L0X_setVcselPulsePeriod(VL53L0X_InitTypeDef*, VL53L0X_vcselPeriodType, u8);
u8 VL53L0X_getVcselPulsePeriod(VL53L0X_InitTypeDef*, VL53L0X_vcselPeriodType);

void VL53L0X_startContinuous(VL53L0X_InitTypeDef*, u32);
void VL53L0X_stopContinuous(VL53L0X_InitTypeDef*);
u16 VL53L0X_readRangeContinuousMillimeters(VL53L0X_InitTypeDef*);
u16 VL53L0X_readRangeSingleMillimeters(VL53L0X_InitTypeDef*);

bool VL53L0X_timeoutOccurred(void);
bool VL53L0X_getSpadInfo(VL53L0X_InitTypeDef*, u8*, bool*);

void VL53L0X_getSequenceStepEnables(VL53L0X_InitTypeDef*, VL53L0X_SequenceStepEnables*);
void VL53L0X_getSequenceStepTimeouts(VL53L0X_InitTypeDef*, VL53L0X_SequenceStepEnables const*, VL53L0X_SequenceStepTimeouts*);

bool VL53L0X_performSingleRefCalibration(VL53L0X_InitTypeDef*, u8);

static u16 VL53L0X_decodeTimeout(u16);
static u16 VL53L0X_encodeTimeout(u32);
static u32 VL53L0X_timeoutMclksToMicroseconds(u16, u16);
static u32 VL53L0X_timeoutMicrosecondsToMclks(u32, u16);

void VL53L0X_setLongRange(VL53L0X_InitTypeDef*);
void VL53L0X_setHighAccuracy(VL53L0X_InitTypeDef*);
void VL53L0X_setHighSpeed(VL53L0X_InitTypeDef*);
u16 VL53L0X_IRQHandler(VL53L0X_InitTypeDef*);
#endif