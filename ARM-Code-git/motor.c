#include "main.h"
#include "motor.h"
#include "serial.h"
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
bool JGB520_Init(volatile u16 out_pwm_L, volatile u16 out_pwm_R){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	
	GPIO_InitTypeDef gpio;
	NVIC_InitTypeDef nvic;
	
	//Pin config for motor move forward
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &gpio);
	
	//Pin config for motor move backward
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &gpio);
	
	if(!extern_timer_config){
		TIM_TimeBaseInitTypeDef tim;
		
		tim.TIM_ClockDivision = TIM_CKD_DIV1;//T = 10ms
		tim.TIM_CounterMode = TIM_CounterMode_Up;
		tim.TIM_Period = 100 - 1;
		tim.TIM_Prescaler = 7200 - 1;
		tim.TIM_RepetitionCounter = 0;
		TIM_TimeBaseInit(TIM1, &tim);
		TIM_Cmd(TIM1, ENABLE);
	}
	motor_Output(out_pwm_L, out_pwm_R);
	
	GPIO_PinRemapConfig(GPIO_PartialRemap2_TIM2, ENABLE);
	
	gpio.GPIO_Mode = GPIO_Mode_IPD;
	gpio.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_9;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &gpio);
	
	if(!extern_timer_config){
		TIM_ICInitTypeDef ic;
		ic.TIM_Channel = TIM_Channel_4;
		ic.TIM_ICFilter = 0;
		ic.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
		ic.TIM_ICPrescaler = TIM_ICPSC_DIV1;
		ic.TIM_ICSelection = TIM_ICSelection_DirectTI;
		TIM_ICInit(TIM2, &ic);
		TIM_ITConfig(TIM2, TIM_IT_CC4, ENABLE);
		
		nvic.NVIC_IRQChannel = TIM2_IRQn;
		nvic.NVIC_IRQChannelCmd = ENABLE;
		nvic.NVIC_IRQChannelPreemptionPriority = 1;
		nvic.NVIC_IRQChannelSubPriority = 0;
		NVIC_Init(&nvic);
		
		ic.TIM_Channel = TIM_Channel_3;
		ic.TIM_ICFilter = 0;
		ic.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
		ic.TIM_ICPrescaler = TIM_ICPSC_DIV1;
		ic.TIM_ICSelection = TIM_ICSelection_DirectTI;
		TIM_ICInit(TIM4, &ic);
		TIM_ITConfig(TIM4, TIM_IT_CC3, ENABLE);
		
		nvic.NVIC_IRQChannel = TIM4_IRQn;
		nvic.NVIC_IRQChannelCmd = ENABLE;
		nvic.NVIC_IRQChannelPreemptionPriority = 1;
		nvic.NVIC_IRQChannelSubPriority = 1;
		NVIC_Init(&nvic);
	}
	
	return true;
}

void motor_Output(volatile u16 out_pwm_L, volatile u16 out_pwm_R){
	if(out_pwm_L > motor_pwm_max) out_pwm_L = motor_pwm_max;
	if(out_pwm_R > motor_pwm_max) out_pwm_R = motor_pwm_max;
	
	for(int i = 0; i < 1000; i++){
		for(int j = 0; j < 150; j++);
	}
	L_motor_pos_out(out_pwm_L);
	for(int i = 0; i < 1000; i++){
		for(int j = 0; j < 150; j++);
	}
	R_motor_pos_out(out_pwm_R);
	
	
}

void L_motor_pos_out(vu16 value){
	TIM_OCInitTypeDef oc;
	oc.TIM_OCMode = TIM_OCMode_PWM1;
	oc.TIM_OCPolarity = TIM_OCPolarity_High;
	oc.TIM_OutputState = TIM_OutputState_Enable;
	oc.TIM_OutputNState = TIM_OutputNState_Disable;
	oc.TIM_Pulse = value;
	TIM_OC1Init(TIM1, &oc);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
}

void R_motor_pos_out(vu16 value){
	TIM_OCInitTypeDef oc;
	oc.TIM_OCMode = TIM_OCMode_PWM1;
	oc.TIM_OCPolarity = TIM_OCPolarity_High;
	oc.TIM_OutputState = TIM_OutputState_Enable;
	oc.TIM_OutputNState = TIM_OutputNState_Disable;
	oc.TIM_Pulse = value;
	TIM_OC2Init(TIM1, &oc);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
}
