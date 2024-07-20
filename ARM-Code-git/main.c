#include "main.h"
#include "motor.h"
#include "serial.h"
#include "servo.h"
#include "mvcam.h"
#include "i2c.h"
#include "my_vl53l0x.h"
#include "ultrasound.h"
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
/*
Total RAM Size = RW Data + ZI Data
Total ROM Size = Code + RO Data + RW Data
//////////////////////////////////////////////////////////////////////
//  TIM1_CH1	(LMF)	TIM2_CH1	(*TRIG)	TIM3_CH1	(SV1)	TIM4_CH1	()	//
//  TIM1_CH1N	(LMB)	TIM2_CH2	()			TIM3_CH2	(SV2)	TIM4_CH2	()	//
//  TIM1_CH2	(RMF)	TIM2_CH3	()			TIM3_CH3	(SV3)	TIM4_CH3	()	//
//  TIM1_CH2N	(RMB)	TIM2_CH4	()			TIM3_CH4	(SV4)	TIM4_CH4	()	//
//  TIM1_CH3	(*ECHO)																								//
//  TIM1_CH3N	()																										//
//	TIM1_CH4	(Tof_trig)																						//
//////////////////////////////////////////////////////////////////////
//	PA8							PA1(op)						PA6							I2C						//
//	PB13															PA7							I2C						//
//	PA9																PB0														//
//	PB14															PB1														//
//	PA10																														//
//	
//	PA11
//////////////////////////////////////////////////////////////////////
//	USART2_TX		PA2		USART3_TX		PC10															//
//	USART2_RX		PA3		USART3_RX		PC11															//
*/
//edit below to enable function
int motor_en = 1;
int serial_en = 1;
int servo_en = 1;
int ults_en = 0;
int tof_en = 0;// vl53l0x only
int debug_en = 0;
//-----------------------------------seperate line----------------------------------------//

static char pbuf[50];
static volatile u16 L_Output = 10;
static volatile u16 R_Output = 10;
static VL53L0X_InitTypeDef sensor1;
static volatile u8 current_state = 0;
static u16 grip_counter = 0;
static u32 sc2_counter = 0;

int main(){
	
	if(debug_en) debug_init();
	
	if(serial_en) usart_init(921600, 921600);
	
	if(motor_en) JGB520_Init(40, 40);
	
	if(servo_en) servo_init(arm_down, sv_pwm_min, sv_pwm_min, gripper_open);
	
	if(ults_en) ults_init();
	
	if(extern_timer_config) timer_init();
	
	if(tof_en){
		char state[7];
		i2c1_init(300000);
		sensor1.address = 0x52;
		sensor1.VL53L0X_Mode = VL53L0X_Mode_Normal;
		sensor1.io_2V8 = true;
		sensor1.io_timeout = 0;
		sensor1.Use_GPIO = true;
		sensor1.GPIO_PORT = GPIOA;
		sensor1.GPIO_PIN = GPIO_Pin_11;
		sensor1.Use_TIM = true;
		sprintf(state, "TOF %s\n\r", VL53L0X_Init(&sensor1)? "SUC" : "FAI");
		u2_tx(state, 10);
	}
	
	char state;
	do{
		do{
			u3_tx("SC0", 3);
			state = u3_rx();
		}while(state == 'Z');
	}while(state != 'K');
	
	while(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13));
	
	while(true){
		current_state = from_openmv(0);
		if(current_state == 0){
			do{
				do{
					u3_tx("SC1", 3);
					state = u3_rx();
				}while(state == 'Z');
			}while(state != 'K');
			
		}else if(current_state == 1){
			u8 motor_act = from_openmv(1);
			if(motor_act == 11){
				L_Output = 10;
				R_Output = 2000;
			}else if(motor_act == 12){
				L_Output = 2000;
				R_Output = 10;
			}else if(motor_act == 13){
				L_Output = 3500;
				R_Output = 3500;
				if(!grip_counter) SV4O(gripper_close);
				grip_counter++;
			}else if(motor_act == 14){
				L_Output = 3500;
				R_Output = 3500;
			}else if(motor_act == 15){
				L_Output = 10;
				R_Output = 10;
			}
			if(grip_counter > 10){
				SV1O(arm_up);
				grip_counter = 0;
				do{
					do{
						u3_tx("SC2", 3);
						state = u3_rx();
					}while(state == 'Z');
				}while(state != 'K');
				sc2_counter = get_msticks();
			}
			
		}else if(current_state == 2){
			u8 motor_act = from_openmv(2);
			if(motor_act == 21){
				L_Output = 3000;
				R_Output = 10000;
			}else if(motor_act == 22){
				L_Output = 10000;
				R_Output = 3000;
			}else if(motor_act == 23){
				L_Output = 10000;
				R_Output = 10000;
			}
			if(get_msticks() - sc2_counter > 2850){
				do{
					do{
						u3_tx("SC3", 3);
						state = u3_rx();
					}while(state == 'Z');
				}while(state != 'K');
			}
			
		}else if(current_state == 3){
			u8 motor_act = from_openmv(3);
			if(motor_act == 31){
				L_Output = 4000;
				R_Output = 10000;
			}else if(motor_act == 32){
				L_Output = 10000;
				R_Output = 4000;
			}else if(motor_act == 33){
				L_Output = 10000;
				R_Output = 10000;
			}else if(motor_act == 34){
				L_Output = 10;
				R_Output = 10000;
				GPIO_SetBits(GPIOA, GPIO_Pin_5);
				sc2_counter = get_msticks();
				while(get_msticks() - sc2_counter < 400);
				do{
					do{
						u3_tx("SC4", 3);
						state = u3_rx();
					}while(state == 'Z');
				}while(state != 'K');
			}
			
		}else if(current_state == 4){
			u8 motor_act = from_openmv(4);
			if(motor_act == 41){
				L_Output = 4000;
				R_Output = 10000;
			}else if(motor_act == 42){
				L_Output = 10000;
				R_Output = 4000;
			}else if(motor_act == 43){
				L_Output = 10000;
				R_Output = 10000;
			}else if(motor_act == 44){
				grip_counter++;
				if(grip_counter >= 20 && grip_counter < 24){
					L_Output = 10;
					R_Output = 10;
					SV1O(arm_down);
				}else if(grip_counter >= 24 && grip_counter < 30) SV4O(gripper_open);
				else if(grip_counter >= 30	) SV1O(arm_up);
			}
		}
		if(debug_en){
			sprintf(pbuf, "%d %d\n\r", L_Output, R_Output);
			u2_tx(pbuf, 13);
		}
	}
}

void timer_init(){
	TIM_TimeBaseInitTypeDef tim;
	NVIC_InitTypeDef nvic;
	
	tim.TIM_ClockDivision = TIM_CKD_DIV1;//T = 10ms
	tim.TIM_CounterMode = TIM_CounterMode_Up;
	tim.TIM_Period = 100 - 1;
	tim.TIM_Prescaler = 7200 - 1;
	tim.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &tim);
	TIM_Cmd(TIM1, ENABLE);
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
	
	nvic.NVIC_IRQChannel = TIM1_UP_IRQn;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&nvic);
	
	tim.TIM_ClockDivision = TIM_CKD_DIV1;//T = 20ms
	tim.TIM_CounterMode = TIM_CounterMode_Up;
	tim.TIM_Period = 10000 - 1;
	tim.TIM_Prescaler = 144 - 1;
	tim.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &tim);
	TIM_Cmd(TIM3, ENABLE);
}

void TIM1_UP_IRQHandler(){
	if(TIM_GetITStatus(TIM1, TIM_IT_Update)){
		msticks++;
		if(msticks % 20 == 0){
			SV2O(L_Output);
			SV3O(R_Output);
		}
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
}

void TIM1_CC_IRQHandler(){
	if(ults_en) ults_IRQHandler();
}

vu32 get_msticks(){
	return msticks;
}
