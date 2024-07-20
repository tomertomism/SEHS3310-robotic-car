#include "my_vl53l0x.h"
#include "serial.h"
#include "main.h"
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
static u32 timeout_start_ms;
static u32 measurement_timing_budget_us;
static u8 stop_variable;
static bool did_timeout;
// Record the current time to check an upcoming timeout against
#define VL53L0X_startTimeout() (timeout_start_ms = get_msticks())
// Check if timeout is enabled (set to nonzero value) and has expired
#define VL53L0X_checkTimeoutExpired() (sensor->io_timeout > 0 && ((get_msticks() - timeout_start_ms) > sensor->io_timeout))
// Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs
// from register value
// based on VL53L0X_decode_vcsel_period()
#define VL53L0X_decodeVcselPeriod(reg_val) ((uint8_t)((reg_val) + 1) << 1)
// Encode VCSEL pulse period register value from period in PCLKs
// based on VL53L0X_encode_vcsel_period()
#define VL53L0X_encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)
// Calculate macro period in nanoseconds from VCSEL period in PCLKs
// based on VL53L0X_calc_macro_period_ps()
// PLL_period_ps = 1655; macro_period_vclks = 2304
#define VL53L0X_calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)

bool VL53L0X_Init(VL53L0X_InitTypeDef* sensor){
	if(VL53L0X_readReg(sensor->address, IDENTIFICATION_MODEL_ID) != 0xEE){//check model id register
		return false;
	}
	//VL53L0X_DataInit() begin
	if(sensor->io_2V8){//enable 2.8v io
		VL53L0X_writeReg(sensor->address, VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, (VL53L0X_readReg(sensor->address, VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV) | 0x01));
	}
	//VL53L0X_writeReg(sensor->address, 0x88, 0x00);//set i2c standard mode
	VL53L0X_writeReg(sensor->address, 0x80, 0x01);
	VL53L0X_writeReg(sensor->address, 0xFF, 0x01);
	VL53L0X_writeReg(sensor->address, 0x00, 0x00);
	stop_variable = VL53L0X_readReg(sensor->address, 0x91);
	VL53L0X_writeReg(sensor->address, 0x00, 0x01);
	VL53L0X_writeReg(sensor->address, 0xFF, 0x00);
	VL53L0X_writeReg(sensor->address, 0x80, 0x00);
	//disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
	VL53L0X_writeReg(sensor->address, MSRC_CONFIG_CONTROL, VL53L0X_readReg(sensor->address, MSRC_CONFIG_CONTROL) | 0x12);
	//set final range signal rate limit to 0.25 MCPS (million counts per second)
	VL53L0X_setSignalRateLimit(sensor, 0.25);
	VL53L0X_writeReg(sensor->address, SYSTEM_SEQUENCE_CONFIG, 0xFF);
	//VL53L0X_DataInit() end
	
	//VL53L0X_StaticInit() begin
	uint8_t spad_count;
	bool spad_type_is_aperture;
	if(!VL53L0X_getSpadInfo(sensor, &spad_count, &spad_type_is_aperture)){
		return false; 
	}
	//The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
	//the API, but the same data seems to be more easily readable from
	//GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
	uint8_t ref_spad_map[6];
	VL53L0X_readMulti(sensor->address, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);
	
	//-- VL53L0X_set_reference_spads() begin (assume NVM values are valid)
	VL53L0X_writeReg(sensor->address, 0xFF, 0x01);
	VL53L0X_writeReg(sensor->address, DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
	VL53L0X_writeReg(sensor->address, DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
	VL53L0X_writeReg(sensor->address, 0xFF, 0x00);
	VL53L0X_writeReg(sensor->address, GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);
	uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0; //12 is the first aperture spad
	uint8_t spads_enabled = 0;
	for(uint8_t i = 0; i < 48; i++){
		if(i < first_spad_to_enable || spads_enabled == spad_count){
			//This bit is lower than the first one that should be enabled, or
			//(reference_spad_count) bits have already been enabled, so zero this bit
			ref_spad_map[i / 8] &= ~(1 << (i % 8));
		}else if((ref_spad_map[i / 8] >> (i % 8)) & 0x1){
			spads_enabled++;
		}
	}
	// -- VL53L0X_load_tuning_settings() end
	
	VL53L0X_writeMulti(sensor->address, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);
	//-- VL53L0X_set_reference_spads() end

	//-- VL53L0X_load_tuning_settings() begin
	//DefaultTuningSettings from VL53L0X_tuning.h
	VL53L0X_writeReg(sensor->address, 0xFF, 0x01);
	VL53L0X_writeReg(sensor->address, 0x00, 0x00);
	VL53L0X_writeReg(sensor->address, 0xFF, 0x00);
	VL53L0X_writeReg(sensor->address, 0x09, 0x00);
	VL53L0X_writeReg(sensor->address, 0x10, 0x00);
	VL53L0X_writeReg(sensor->address, 0x11, 0x00);
	VL53L0X_writeReg(sensor->address, 0x24, 0x01);
	VL53L0X_writeReg(sensor->address, 0x25, 0xFF);
	VL53L0X_writeReg(sensor->address, 0x75, 0x00);
	VL53L0X_writeReg(sensor->address, 0xFF, 0x01);
	VL53L0X_writeReg(sensor->address, 0x4E, 0x2C);
	VL53L0X_writeReg(sensor->address, 0x48, 0x00);
	VL53L0X_writeReg(sensor->address, 0x30, 0x20);
	VL53L0X_writeReg(sensor->address, 0xFF, 0x00);
	VL53L0X_writeReg(sensor->address, 0x30, 0x09);
	VL53L0X_writeReg(sensor->address, 0x54, 0x00);
	VL53L0X_writeReg(sensor->address, 0x31, 0x04);
	VL53L0X_writeReg(sensor->address, 0x32, 0x03);
	VL53L0X_writeReg(sensor->address, 0x40, 0x83);
	VL53L0X_writeReg(sensor->address, 0x46, 0x25);
	VL53L0X_writeReg(sensor->address, 0x60, 0x00);
	VL53L0X_writeReg(sensor->address, 0x27, 0x00);
	VL53L0X_writeReg(sensor->address, 0x50, 0x06);
	VL53L0X_writeReg(sensor->address, 0x51, 0x00);
	VL53L0X_writeReg(sensor->address, 0x52, 0x96);
	VL53L0X_writeReg(sensor->address, 0x56, 0x08);
	VL53L0X_writeReg(sensor->address, 0x57, 0x30);
	VL53L0X_writeReg(sensor->address, 0x61, 0x00);
	VL53L0X_writeReg(sensor->address, 0x62, 0x00);
	VL53L0X_writeReg(sensor->address, 0x64, 0x00);
	VL53L0X_writeReg(sensor->address, 0x65, 0x00);
	VL53L0X_writeReg(sensor->address, 0x66, 0xA0);
	VL53L0X_writeReg(sensor->address, 0xFF, 0x01);
	VL53L0X_writeReg(sensor->address, 0x22, 0x32);
	VL53L0X_writeReg(sensor->address, 0x47, 0x14);
	VL53L0X_writeReg(sensor->address, 0x49, 0xFF);
	VL53L0X_writeReg(sensor->address, 0x4A, 0x00);
	VL53L0X_writeReg(sensor->address, 0xFF, 0x00);
	VL53L0X_writeReg(sensor->address, 0x7A, 0x0A);
	VL53L0X_writeReg(sensor->address, 0x7B, 0x00);
	VL53L0X_writeReg(sensor->address, 0x78, 0x21);
	VL53L0X_writeReg(sensor->address, 0xFF, 0x01);
	VL53L0X_writeReg(sensor->address, 0x23, 0x34);
	VL53L0X_writeReg(sensor->address, 0x42, 0x00);
	VL53L0X_writeReg(sensor->address, 0x44, 0xFF);
	VL53L0X_writeReg(sensor->address, 0x45, 0x26);
	VL53L0X_writeReg(sensor->address, 0x46, 0x05);
	VL53L0X_writeReg(sensor->address, 0x40, 0x40);
	VL53L0X_writeReg(sensor->address, 0x0E, 0x06);
	VL53L0X_writeReg(sensor->address, 0x20, 0x1A);
	VL53L0X_writeReg(sensor->address, 0x43, 0x40);
	VL53L0X_writeReg(sensor->address, 0xFF, 0x00);
	VL53L0X_writeReg(sensor->address, 0x34, 0x03);
	VL53L0X_writeReg(sensor->address, 0x35, 0x44);
	VL53L0X_writeReg(sensor->address, 0xFF, 0x01);
	VL53L0X_writeReg(sensor->address, 0x31, 0x04);
	VL53L0X_writeReg(sensor->address, 0x4B, 0x09);
	VL53L0X_writeReg(sensor->address, 0x4C, 0x05);
	VL53L0X_writeReg(sensor->address, 0x4D, 0x04);
	VL53L0X_writeReg(sensor->address, 0xFF, 0x00);
	VL53L0X_writeReg(sensor->address, 0x44, 0x00);
	VL53L0X_writeReg(sensor->address, 0x45, 0x20);
	VL53L0X_writeReg(sensor->address, 0x47, 0x08);
	VL53L0X_writeReg(sensor->address, 0x48, 0x28);
	VL53L0X_writeReg(sensor->address, 0x67, 0x00);
	VL53L0X_writeReg(sensor->address, 0x70, 0x04);
	VL53L0X_writeReg(sensor->address, 0x71, 0x01);
	VL53L0X_writeReg(sensor->address, 0x72, 0xFE);
	VL53L0X_writeReg(sensor->address, 0x76, 0x00);
	VL53L0X_writeReg(sensor->address, 0x77, 0x00);
	VL53L0X_writeReg(sensor->address, 0xFF, 0x01);
	VL53L0X_writeReg(sensor->address, 0x0D, 0x01);
	VL53L0X_writeReg(sensor->address, 0xFF, 0x00);
	VL53L0X_writeReg(sensor->address, 0x80, 0x01);
	VL53L0X_writeReg(sensor->address, 0x01, 0xF8);
	VL53L0X_writeReg(sensor->address, 0xFF, 0x01);
	VL53L0X_writeReg(sensor->address, 0x8E, 0x01);
	VL53L0X_writeReg(sensor->address, 0x00, 0x01);
	VL53L0X_writeReg(sensor->address, 0xFF, 0x00);
	VL53L0X_writeReg(sensor->address, 0x80, 0x00);
	
	//"Set interrupt config to new sample ready"
	//-- VL53L0X_SetGpioConfig() begin
	VL53L0X_writeReg(sensor->address, SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
	VL53L0X_writeReg(sensor->address, GPIO_HV_MUX_ACTIVE_HIGH, VL53L0X_readReg(sensor->address, GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); // active low
	VL53L0X_writeReg(sensor->address, SYSTEM_INTERRUPT_CLEAR, 0x01);
	//-- VL53L0X_SetGpioConfig() end
	
	measurement_timing_budget_us = VL53L0X_getMeasurementTimingBudget(sensor);
	//"Disable MSRC and TCC by default"
	//MSRC = Minimum Signal Rate Check
	//TCC = Target CentreCheck
	//this code is written by (Github) tomertomism
	//-- VL53L0X_SetSequenceStepEnable() begin
	VL53L0X_writeReg(sensor->address, SYSTEM_SEQUENCE_CONFIG, 0xE8);
	//-- VL53L0X_SetSequenceStepEnable() end
	
	//"Recalculate timing budget"
	VL53L0X_setMeasurementTimingBudget(sensor, measurement_timing_budget_us);
	//VL53L0X_StaticInit() end
	
	//VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())
	//-- VL53L0X_perform_vhv_calibration() begin
	VL53L0X_writeReg(sensor->address, SYSTEM_SEQUENCE_CONFIG, 0x01);
	if(!VL53L0X_performSingleRefCalibration(sensor, 0x40)){
		return false;
	}
	//-- VL53L0X_perform_vhv_calibration() end
	
	//-- VL53L0X_perform_phase_calibration() begin
	VL53L0X_writeReg(sensor->address, SYSTEM_SEQUENCE_CONFIG, 0x02);
	if(!VL53L0X_performSingleRefCalibration(sensor, 0x00)){
		return false;
	}
	//-- VL53L0X_perform_phase_calibration() end
	
	//restore the previous Sequence Config
	VL53L0X_writeReg(sensor->address, SYSTEM_SEQUENCE_CONFIG, 0xE8);
	//VL53L0X_PerformRefCalibration() end
	return true;
}


void VL53L0X_writeReg(u8 addr, u8 reg, u8 value){
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
	I2C_GenerateSTART(I2C1, ENABLE);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress(I2C1, addr, I2C_Direction_Transmitter);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	I2C_SendData(I2C1, reg);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING));
	I2C_SendData(I2C1, value);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING));
	I2C_GenerateSTOP(I2C1, ENABLE);
}

void VL53L0X_writeReg16Bit(u8 addr, u8 reg, u16 value){
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
	I2C_GenerateSTART(I2C1, ENABLE);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress(I2C1, addr, I2C_Direction_Transmitter);
	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == RESET);
	I2C_SendData(I2C1, reg);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING));
	I2C_SendData(I2C1, value >> 8);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING));
	I2C_SendData(I2C1, (u8)value);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C_GenerateSTOP(I2C1, ENABLE);
}

void VL53L0X_writeReg32Bit(u8 addr, u8 reg, u32 value){
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
	I2C_GenerateSTART(I2C1, ENABLE);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress(I2C1, addr, I2C_Direction_Transmitter);
	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == RESET);
	I2C_SendData(I2C1, reg);
	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == RESET);
	I2C_SendData(I2C1, value >> 24);
	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == RESET);
	I2C_SendData(I2C1, (u8)value >> 16);
	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == RESET);
	I2C_SendData(I2C1, (u8)value >> 8);
	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == RESET);
	I2C_SendData(I2C1, (u8)value);
	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == RESET);
	I2C_GenerateSTOP(I2C1, ENABLE);
}

u8 VL53L0X_readReg(u8 addr, u8 reg){
	u8 value;
	while(I2C_GetFlagStatus(I2C1,I2C_FLAG_BUSY));
	I2C_GenerateSTART(I2C1, ENABLE);
	while(!I2C_GetFlagStatus(I2C1,I2C_FLAG_SB));
	I2C_Send7bitAddress(I2C1, addr, I2C_Direction_Transmitter);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	I2C_SendData(I2C1, reg);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C_GenerateSTART(I2C1, ENABLE);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress(I2C1, addr, I2C_Direction_Receiver);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	I2C_AcknowledgeConfig(I2C1, DISABLE);
	I2C_GenerateSTOP(I2C1, ENABLE);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
	I2C_NACKPositionConfig(I2C1, I2C_NACKPosition_Current);
	I2C_AcknowledgeConfig(I2C1, ENABLE);
	value = I2C_ReceiveData(I2C1);
	return value;
}

u16 VL53L0X_readReg16Bit(u8 addr, u8 reg){
	u16 value;
	while(I2C_GetFlagStatus(I2C1,I2C_FLAG_BUSY));
	I2C_GenerateSTART(I2C1, ENABLE);
	while(!I2C_GetFlagStatus(I2C1,I2C_FLAG_SB));
	I2C_Send7bitAddress(I2C1, addr, I2C_Direction_Transmitter);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	I2C_SendData(I2C1, reg);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C_GenerateSTART(I2C1, ENABLE);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress(I2C1, addr, I2C_Direction_Receiver);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
	value = (u16)(I2C_ReceiveData(I2C1) << 8);
	I2C_AcknowledgeConfig(I2C1, DISABLE);
	I2C_GenerateSTOP(I2C1, ENABLE);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
	I2C_NACKPositionConfig(I2C1, I2C_NACKPosition_Current);
	I2C_AcknowledgeConfig(I2C1, ENABLE);
	value |= I2C_ReceiveData(I2C1);
	return value;
}

u32 VL53L0X_readReg32Bit(u8 addr, u8 reg){
	u32 value;
	while(I2C_GetFlagStatus(I2C1,I2C_FLAG_BUSY));
	I2C_GenerateSTART(I2C1, ENABLE);
	while(!I2C_GetFlagStatus(I2C1,I2C_FLAG_SB));
	I2C_Send7bitAddress(I2C1, addr, I2C_Direction_Transmitter);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	I2C_SendData(I2C1, reg);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C_GenerateSTART(I2C1, ENABLE);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress(I2C1, addr, I2C_Direction_Receiver);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
	value = (u32)I2C_ReceiveData(I2C1) << 24;
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
	value |= (u32)I2C_ReceiveData(I2C1) << 16;
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
	value |= (u32)I2C_ReceiveData(I2C1) << 8;
	I2C_AcknowledgeConfig(I2C1, DISABLE);
	I2C_GenerateSTOP(I2C1, ENABLE);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
	I2C_NACKPositionConfig(I2C1, I2C_NACKPosition_Current);
	I2C_AcknowledgeConfig(I2C1, ENABLE);
	value |= I2C_ReceiveData(I2C1);
	return value;
}

void VL53L0X_writeMulti(u8 addr, u8 reg, u8 const* src, u8 count){
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
	I2C_GenerateSTART(I2C1, ENABLE);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress(I2C1, addr, I2C_Direction_Transmitter);
	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == RESET);
	I2C_SendData(I2C1, reg);
	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == RESET);
	while(count-- > 0){
		I2C_SendData(I2C1, *(src++));
		while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == RESET);
	}
	I2C_GenerateSTOP(I2C1, ENABLE);
}

void VL53L0X_readMulti(u8 addr, u8 reg, u8* dst, u8 count){
	while(I2C_GetFlagStatus(I2C1,I2C_FLAG_BUSY));
	I2C_GenerateSTART(I2C1, ENABLE);
	while(!I2C_GetFlagStatus(I2C1,I2C_FLAG_SB));
	I2C_Send7bitAddress(I2C1, addr, I2C_Direction_Transmitter);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	I2C_SendData(I2C1, reg);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C_GenerateSTART(I2C1, ENABLE);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress(I2C1, addr, I2C_Direction_Receiver);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	while(count-- > 0){
		if(count == 0){
			I2C_AcknowledgeConfig(I2C1, DISABLE);
			I2C_GenerateSTOP(I2C1, ENABLE);
			while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
			I2C_NACKPositionConfig(I2C1, I2C_NACKPosition_Current);
			I2C_AcknowledgeConfig(I2C1, ENABLE);
			*(dst++) = I2C_ReceiveData(I2C1);
		}else{
			while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
			*(dst++) = I2C_ReceiveData(I2C1);
		}
	}
}

bool VL53L0X_setSignalRateLimit(VL53L0X_InitTypeDef* sensor, float limit_Mcps){
	if(limit_Mcps < 0 || limit_Mcps >= 512){
		return false;
	}
	//Q9.7 fixed point format (9 integer bits, 7 fractional bits)
	VL53L0X_writeReg16Bit(sensor->address, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, (u16)limit_Mcps * (1 << 7));
	return true;
}

float VL53L0X_getSignalRateLimit(VL53L0X_InitTypeDef* sensor){
	return (float)(VL53L0X_readReg16Bit(sensor->address, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT) / (1 << 7));
}

bool VL53L0X_setMeasurementTimingBudget(VL53L0X_InitTypeDef* sensor, u32 budget_us){
	VL53L0X_SequenceStepEnables enables;
	VL53L0X_SequenceStepTimeouts timeouts;
	
	u16 const StartOverhead     = 1910;
	u16 const EndOverhead        = 960;
	u16 const MsrcOverhead       = 660;
	u16 const TccOverhead        = 590;
	u16 const DssOverhead        = 690;
	u16 const PreRangeOverhead   = 660;
	u16 const FinalRangeOverhead = 550;
	
	u32 used_budget_us = StartOverhead + EndOverhead;
	VL53L0X_getSequenceStepEnables(sensor, &enables);
	VL53L0X_getSequenceStepTimeouts(sensor, &enables, &timeouts);
	if(enables.tcc){
		used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
	}
	if(enables.dss){
		used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
	}else if(enables.msrc){
		used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
	}
	if(enables.pre_range){
		used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
	}
	if(enables.final_range){
		used_budget_us += FinalRangeOverhead;
		//Note that the final range timeout is determined by the timing
		//budget and the sum of all other timeouts within the sequence.
		//If there is no room for the final range timeout, then an error
		//will be set. Otherwise the remaining time will be applied to
		//the final range.
		if(used_budget_us > budget_us){
			//Requested timeout too big.
			return false;
		}
		u32 final_range_timeout_us = budget_us - used_budget_us;
		//set_sequence_step_timeout() begin
		//(SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)
		//For the final range timeout, the pre-range timeout
		//must be added. To do this both final and pre-range
		//timeouts must be expressed in macro periods MClks
		//because they have different vcsel periods.
		u32 final_range_timeout_mclks = VL53L0X_timeoutMicrosecondsToMclks(final_range_timeout_us, timeouts.final_range_vcsel_period_pclks);
		if(enables.pre_range){
			final_range_timeout_mclks += timeouts.pre_range_mclks;
		}
		VL53L0X_writeReg16Bit(sensor->address, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, VL53L0X_encodeTimeout(final_range_timeout_mclks));
		//set_sequence_step_timeout() end
		
		measurement_timing_budget_us = budget_us; // store for internal reuse
	}
	return true;
}

u32 VL53L0X_getMeasurementTimingBudget(VL53L0X_InitTypeDef* sensor){
	VL53L0X_SequenceStepEnables enables;
	VL53L0X_SequenceStepTimeouts timeouts;
	
	u16 const StartOverhead = 1910;
	u16 const EndOverhead = 960;
	u16 const MsrcOverhead = 660;
	u16 const TccOverhead = 590;
	u16 const DssOverhead = 690;
	u16 const PreRangeOverhead = 660;
	u16 const FinalRangeOverhead = 550;
	
	// "Start and end overhead times always present"
	u32 budget_us = StartOverhead + EndOverhead;
	
	VL53L0X_getSequenceStepEnables(sensor, &enables);
	VL53L0X_getSequenceStepTimeouts(sensor, &enables, &timeouts);
	
	if(enables.tcc){
		budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
	}
	if(enables.dss){
		budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
	}else if(enables.msrc){
		budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
	}
	if(enables.pre_range){
		budget_us += (timeouts.pre_range_us + PreRangeOverhead);
	}
	if(enables.final_range){
		budget_us += (timeouts.final_range_us + FinalRangeOverhead);
	}
	
	measurement_timing_budget_us = budget_us; // store for internal reuse
	return budget_us;
}

bool VL53L0X_setVcselPulsePeriod(VL53L0X_InitTypeDef* sensor, VL53L0X_vcselPeriodType type, u8 period_pclks){
	u8 vcsel_period_reg = VL53L0X_encodeVcselPeriod(period_pclks);
	VL53L0X_SequenceStepEnables enables;
	VL53L0X_SequenceStepTimeouts timeouts;
	VL53L0X_getSequenceStepEnables(sensor, &enables);
	VL53L0X_getSequenceStepTimeouts(sensor, &enables, &timeouts);
	// "Apply specific settings for the requested clock period"
	// "Re-calculate and apply timeouts, in macro periods"

	// "When the VCSEL period for the pre or final range is changed,
	// the corresponding timeout must be read from the device using
	// the current VCSEL period, then the new VCSEL period can be
	// applied. The timeout then must be written back to the device
	// using the new VCSEL period.
	//
	// For the MSRC timeout, the same applies - this timeout being
	// dependant on the pre-range vcsel period."
	if(type == VcselPeriodPreRange){
		// "Set phase check limits"
		switch(period_pclks){
			case 12:
				VL53L0X_writeReg(sensor->address, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
				break;
			case 14:
				VL53L0X_writeReg(sensor->address, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
				break;
			case 16:
				VL53L0X_writeReg(sensor->address, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
				break;
			case 18:
				VL53L0X_writeReg(sensor->address, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
				break;
			default:
        // invalid period
				return false;
		}
		VL53L0X_writeReg(sensor->address, PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
		// apply new VCSEL period
		VL53L0X_writeReg(sensor->address, PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);
		// update timeouts
		// set_sequence_step_timeout() begin
		// (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE)
		u32 new_pre_range_timeout_mclks = VL53L0X_timeoutMicrosecondsToMclks(timeouts.pre_range_us, period_pclks);
		VL53L0X_writeReg16Bit(sensor->address, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, VL53L0X_encodeTimeout(new_pre_range_timeout_mclks));
		// set_sequence_step_timeout() end
		// set_sequence_step_timeout() begin
		// (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)
		u32 new_msrc_timeout_mclks = VL53L0X_timeoutMicrosecondsToMclks(timeouts.msrc_dss_tcc_us, period_pclks);
		VL53L0X_writeReg(sensor->address, MSRC_CONFIG_TIMEOUT_MACROP, (new_msrc_timeout_mclks > 256) ? 255 : (u8)(new_msrc_timeout_mclks - 1));
    // set_sequence_step_timeout() end
	}else if(type == VcselPeriodFinalRange){
		switch(period_pclks){
			case 8:
				VL53L0X_writeReg(sensor->address, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
				VL53L0X_writeReg(sensor->address, FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
				VL53L0X_writeReg(sensor->address, GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
				VL53L0X_writeReg(sensor->address, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
				VL53L0X_writeReg(sensor->address, 0xFF, 0x01);
				VL53L0X_writeReg(sensor->address, ALGO_PHASECAL_LIM, 0x30);
				VL53L0X_writeReg(sensor->address, 0xFF, 0x00);
				break;
			case 10:
				VL53L0X_writeReg(sensor->address, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
				VL53L0X_writeReg(sensor->address, FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
				VL53L0X_writeReg(sensor->address, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
				VL53L0X_writeReg(sensor->address, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
				VL53L0X_writeReg(sensor->address, 0xFF, 0x01);
				VL53L0X_writeReg(sensor->address, ALGO_PHASECAL_LIM, 0x20);
				VL53L0X_writeReg(sensor->address, 0xFF, 0x00);
				break;
			case 12:
				VL53L0X_writeReg(sensor->address, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
				VL53L0X_writeReg(sensor->address, FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
				VL53L0X_writeReg(sensor->address, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
				VL53L0X_writeReg(sensor->address, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
				VL53L0X_writeReg(sensor->address, 0xFF, 0x01);
				VL53L0X_writeReg(sensor->address, ALGO_PHASECAL_LIM, 0x20);
				VL53L0X_writeReg(sensor->address, 0xFF, 0x00);
				break;
			case 14:
				VL53L0X_writeReg(sensor->address, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
				VL53L0X_writeReg(sensor->address, FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
				VL53L0X_writeReg(sensor->address, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
				VL53L0X_writeReg(sensor->address, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
				VL53L0X_writeReg(sensor->address, 0xFF, 0x01);
				VL53L0X_writeReg(sensor->address, ALGO_PHASECAL_LIM, 0x20);
				VL53L0X_writeReg(sensor->address, 0xFF, 0x00);
				break;
			default:
				// invalid period
				return false;
		}
		// apply new VCSEL period
		VL53L0X_writeReg(sensor->address, FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);
		// update timeouts

		// set_sequence_step_timeout() begin
		// (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

		// "For the final range timeout, the pre-range timeout
		//  must be added. To do this both final and pre-range
		//  timeouts must be expressed in macro periods MClks
		//  because they have different vcsel periods."
		u32 new_final_range_timeout_mclks = VL53L0X_timeoutMicrosecondsToMclks(timeouts.final_range_us, period_pclks);
		if(enables.pre_range){
			new_final_range_timeout_mclks += timeouts.pre_range_mclks;
		}
			VL53L0X_writeReg16Bit(sensor->address, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, VL53L0X_encodeTimeout(new_final_range_timeout_mclks));
			// set_sequence_step_timeout end
	}else{
		// invalid type
		return false;
	}
	// "Finally, the timing budget must be re-applied"
	VL53L0X_setMeasurementTimingBudget(sensor, measurement_timing_budget_us);
	// "Perform the phase calibration. This is needed after changing on vcsel period."
	// VL53L0X_perform_phase_calibration() begin
	u8 sequence_config = VL53L0X_readReg(sensor->address, SYSTEM_SEQUENCE_CONFIG);
	VL53L0X_writeReg(sensor->address, SYSTEM_SEQUENCE_CONFIG, 0x02);
	VL53L0X_performSingleRefCalibration(sensor, 0x0);
	VL53L0X_writeReg(sensor->address, SYSTEM_SEQUENCE_CONFIG, sequence_config);
	// VL53L0X_perform_phase_calibration() end
	return true;
}

u8 VL53L0X_getVcselPulsePeriod(VL53L0X_InitTypeDef* sensor, VL53L0X_vcselPeriodType type){
	if(type == VcselPeriodPreRange){
		return (u8)VL53L0X_decodeVcselPeriod(VL53L0X_readReg(sensor->address, PRE_RANGE_CONFIG_VCSEL_PERIOD));
	}else if(type == VcselPeriodFinalRange){
		return (u8)VL53L0X_decodeVcselPeriod(VL53L0X_readReg(sensor->address, FINAL_RANGE_CONFIG_VCSEL_PERIOD));
	}else{
		return 255;
	}
}


void VL53L0X_startContinuous(VL53L0X_InitTypeDef* sensor, u32 period_ms){
	VL53L0X_writeReg(sensor->address, 0x80, 0x01);
	VL53L0X_writeReg(sensor->address, 0xFF, 0x01);
	VL53L0X_writeReg(sensor->address,  0x00, 0x00);
	VL53L0X_writeReg(sensor->address, 0x91, stop_variable);
	VL53L0X_writeReg(sensor->address, 0x00, 0x01);
	VL53L0X_writeReg(sensor->address, 0xFF, 0x00);
	VL53L0X_writeReg(sensor->address, 0x80, 0x00);
	if(period_ms != 0){
		// continuous timed mode
		// VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin
		u16 osc_calibrate_val = VL53L0X_readReg16Bit(sensor->address, OSC_CALIBRATE_VAL);
		if(osc_calibrate_val != 0){
			period_ms *= osc_calibrate_val;
		}
		VL53L0X_writeReg32Bit(sensor->address, SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);
		// VL53L0X_SetInterMeasurementPeriodMilliSeconds() end
		VL53L0X_writeReg(sensor->address, SYSRANGE_START, 0x04); // VL53L0X_REG_SYSRANGE_MODE_TIMED
	}else{
		// continuous back-to-back mode
		VL53L0X_writeReg(sensor->address, SYSRANGE_START, 0x02); // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
	}
}

void VL53L0X_stopContinuous(VL53L0X_InitTypeDef* sensor){
	VL53L0X_writeReg(sensor->address, SYSRANGE_START, 0x01); // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT
	VL53L0X_writeReg(sensor->address, 0xFF, 0x01);
	VL53L0X_writeReg(sensor->address, 0x00, 0x00);
	VL53L0X_writeReg(sensor->address, 0x91, 0x00);
	VL53L0X_writeReg(sensor->address, 0x00, 0x01);
	VL53L0X_writeReg(sensor->address, 0xFF, 0x00);
}

u16 VL53L0X_readRangeContinuousMillimeters(VL53L0X_InitTypeDef* sensor){
	VL53L0X_startTimeout();
	if(sensor->Use_GPIO && !sensor->Use_TIM){
		while(GPIO_ReadInputDataBit(sensor->GPIO_PORT, sensor->GPIO_PIN)){
			if(VL53L0X_checkTimeoutExpired()){
				did_timeout = true;
				return 65535;
			}
		}
	}else{
		while((VL53L0X_readReg(sensor->address, RESULT_INTERRUPT_STATUS) & 0x07) == 0){
			if(VL53L0X_checkTimeoutExpired()){
				did_timeout = true;
				return 65535;
			}
		}
	}
	// assumptions: Linearity Corrective Gain is 1000 (default);
	// fractional ranging is not enabled
	u16 range = VL53L0X_readReg16Bit(sensor->address, RESULT_RANGE_STATUS + 10);
	VL53L0X_writeReg(sensor->address, SYSTEM_INTERRUPT_CLEAR, 0x01);
	return range;
}

u16 VL53L0X_readRangeSingleMillimeters(VL53L0X_InitTypeDef* sensor){
	VL53L0X_writeReg(sensor->address, 0x80, 0x01);
	VL53L0X_writeReg(sensor->address, 0xFF, 0x01);
	VL53L0X_writeReg(sensor->address, 0x00, 0x00);
	VL53L0X_writeReg(sensor->address, 0x91, stop_variable);
	VL53L0X_writeReg(sensor->address, 0x00, 0x01);
	VL53L0X_writeReg(sensor->address, 0xFF, 0x00);
	VL53L0X_writeReg(sensor->address, 0x80, 0x00);
	VL53L0X_writeReg(sensor->address, SYSRANGE_START, 0x01);
	// "Wait until start bit has been cleared"
	VL53L0X_startTimeout();
	while(VL53L0X_readReg(sensor->address, SYSRANGE_START) & 0x01){
		if(VL53L0X_checkTimeoutExpired()){
				GPIO_SetBits(GPIOC, GPIO_Pin_0);
			did_timeout = true;
			return 65535;
		}
	}
	return VL53L0X_readRangeContinuousMillimeters(sensor);
}

bool VL53L0X_timeoutOccurred(){
	bool tmp = did_timeout;
	did_timeout = false;
	return tmp;
}

bool VL53L0X_getSpadInfo(VL53L0X_InitTypeDef* sensor, u8* count, bool* type_is_aperture){
	u8 tmp;
	
	VL53L0X_writeReg(sensor->address, 0x80, 0x01);
	VL53L0X_writeReg(sensor->address, 0xFF, 0x01);
	VL53L0X_writeReg(sensor->address, 0x00, 0x00);
	
	VL53L0X_writeReg(sensor->address, 0xFF, 0x06);
	VL53L0X_writeReg(sensor->address, 0x83, VL53L0X_readReg(sensor->address, 0x83) | 0x04);
	VL53L0X_writeReg(sensor->address, 0xFF, 0x07);
	VL53L0X_writeReg(sensor->address, 0x81, 0x01);
	
	VL53L0X_writeReg(sensor->address, 0x80, 0x01);
	
	VL53L0X_writeReg(sensor->address, 0x94, 0x6b);
	VL53L0X_writeReg(sensor->address, 0x83, 0x00);
	VL53L0X_startTimeout();
	while(VL53L0X_readReg(sensor->address, 0x83) == 0){
		if(VL53L0X_checkTimeoutExpired()){
			return false;
		}
	}
	VL53L0X_writeReg(sensor->address, 0x83, 0x01);
	tmp = VL53L0X_readReg(sensor->address, 0x92);
	
	*count = tmp & 0x7f;
	*type_is_aperture = (tmp >> 7) & 0x01;
	
	VL53L0X_writeReg(sensor->address, 0x81, 0x00);
	VL53L0X_writeReg(sensor->address, 0xFF, 0x06);
	VL53L0X_writeReg(sensor->address, 0x83, VL53L0X_readReg(sensor->address, 0x83) & ~0x04);
	VL53L0X_writeReg(sensor->address, 0xFF, 0x01);
	VL53L0X_writeReg(sensor->address, 0x00, 0x01);
	
	VL53L0X_writeReg(sensor->address, 0xFF, 0x00);
	VL53L0X_writeReg(sensor->address, 0x80, 0x00);
	
	return true;
}

void VL53L0X_getSequenceStepEnables(VL53L0X_InitTypeDef* sensor, VL53L0X_SequenceStepEnables* enables){
	u8 sequence_config = VL53L0X_readReg(sensor->address, SYSTEM_SEQUENCE_CONFIG);
	enables->tcc = (sequence_config >> 4) & 0x1;
	enables->dss = (sequence_config >> 3) & 0x1;
	enables->msrc = (sequence_config >> 2) & 0x1;
	enables->pre_range = (sequence_config >> 6) & 0x1;
	enables->final_range = (sequence_config >> 7) & 0x1;
}

void VL53L0X_getSequenceStepTimeouts(VL53L0X_InitTypeDef* sensor, VL53L0X_SequenceStepEnables const* enables, VL53L0X_SequenceStepTimeouts* timeouts){
	timeouts->pre_range_vcsel_period_pclks = VL53L0X_getVcselPulsePeriod(sensor, VcselPeriodPreRange);
	timeouts->msrc_dss_tcc_mclks = VL53L0X_readReg(sensor->address, MSRC_CONFIG_TIMEOUT_MACROP) + 1;
	timeouts->msrc_dss_tcc_us = VL53L0X_timeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks, timeouts->pre_range_vcsel_period_pclks);
	timeouts->pre_range_mclks = VL53L0X_decodeTimeout(VL53L0X_readReg16Bit(sensor->address, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
	timeouts->pre_range_us = VL53L0X_timeoutMclksToMicroseconds(timeouts->pre_range_mclks, timeouts->pre_range_vcsel_period_pclks);
	timeouts->final_range_vcsel_period_pclks = VL53L0X_getVcselPulsePeriod(sensor, VcselPeriodFinalRange);
	timeouts->final_range_mclks = VL53L0X_decodeTimeout(VL53L0X_readReg16Bit(sensor->address, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));
	if(enables->pre_range){
		timeouts->final_range_mclks -= timeouts->pre_range_mclks;
	}
	timeouts->final_range_us = VL53L0X_timeoutMclksToMicroseconds(timeouts->final_range_mclks, timeouts->final_range_vcsel_period_pclks);
}

bool VL53L0X_performSingleRefCalibration(VL53L0X_InitTypeDef* sensor, u8 vhv_init_byte){
	VL53L0X_writeReg(sensor->address, SYSRANGE_START, 0x01 | vhv_init_byte); // VL53L0X_REG_SYSRANGE_MODE_START_STOP
	VL53L0X_startTimeout();
	while((VL53L0X_readReg(sensor->address, RESULT_INTERRUPT_STATUS) & 0x07) == 0){
		if(VL53L0X_checkTimeoutExpired()){
			return false;
		}
	}
	VL53L0X_writeReg(sensor->address, SYSTEM_INTERRUPT_CLEAR, 0x01);
	VL53L0X_writeReg(sensor->address, SYSRANGE_START, 0x00);
	return true;
}

u16 VL53L0X_decodeTimeout(u16 reg_val){
	// format: "(LSByte * 2^MSByte) + 1"
	return (u16)((reg_val & 0x00FF) << (u16)((reg_val & 0xFF00) >> 8)) + 1;
}

u16 VL53L0X_encodeTimeout(u32 timeout_mclks){
	//format: "(LSByte * 2^MSByte) + 1"
	u32 ls_byte = 0;
	u16 ms_byte = 0;
	if(timeout_mclks > 0){
		ls_byte = timeout_mclks - 1;
		while((ls_byte & 0xFFFFFF00) > 0){
			ls_byte >>= 1;
			ms_byte++;
		}
		return (u16)(ms_byte << 8) | (ls_byte & 0xFF);
	}else{
		return 0;
	}
}

static u32 VL53L0X_timeoutMclksToMicroseconds(u16 timeout_period_mclks, u16 vcsel_period_pclks){
	u32 macro_period_ns = VL53L0X_calcMacroPeriod(vcsel_period_pclks);
	return ((timeout_period_mclks * macro_period_ns) + 500) / 1000;
}

u32 VL53L0X_timeoutMicrosecondsToMclks(u32 timeout_period_us, u16 vcsel_period_pclks){
	uint32_t macro_period_ns = VL53L0X_calcMacroPeriod(vcsel_period_pclks);
	return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

void VL53L0X_setLongRange(VL53L0X_InitTypeDef* sensor){
	// lower the return signal rate limit (default is 0.25 MCPS)
	VL53L0X_setSignalRateLimit(sensor, (float)0.1);
	// increase laser pulse periods (defaults are 14 and 10 PCLKs)
	VL53L0X_setVcselPulsePeriod(sensor, VcselPeriodPreRange, 18);
	VL53L0X_setVcselPulsePeriod(sensor, VcselPeriodFinalRange, 14);
}

void VL53L0X_setHighAccuracy(VL53L0X_InitTypeDef* sensor){
	VL53L0X_setSignalRateLimit(sensor, 0.25);
	VL53L0X_setMeasurementTimingBudget(sensor, 200000);
	VL53L0X_setVcselPulsePeriod(sensor, VcselPeriodPreRange, 14);
	VL53L0X_setVcselPulsePeriod(sensor, VcselPeriodFinalRange, 10);
}

void VL53L0X_setHighSpeed(VL53L0X_InitTypeDef* sensor){
	VL53L0X_setSignalRateLimit(sensor, 0.25);
	VL53L0X_setMeasurementTimingBudget(sensor, 20000);
	VL53L0X_setVcselPulsePeriod(sensor, VcselPeriodPreRange, 14);
	VL53L0X_setVcselPulsePeriod(sensor, VcselPeriodFinalRange, 10);
}

u16 VL53L0X_IRQHandler(VL53L0X_InitTypeDef* sensor){
	u16 range = 0;
	if(TIM_GetITStatus(TIM1, TIM_IT_CC4)){
		range = VL53L0X_readReg16Bit(sensor->address, RESULT_RANGE_STATUS + 10);
		VL53L0X_writeReg(sensor->address, SYSTEM_INTERRUPT_CLEAR, 0x01);
		if(range > 8100){
			u2_tx("out of range...\n\r", 18);
		}else{
			//sprintf(pbuffer, "ToF distance: %d mm\n\r", range);
			//u2_tx(pbuffer, 30);
		}
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC4);
	}
	return range;
}
