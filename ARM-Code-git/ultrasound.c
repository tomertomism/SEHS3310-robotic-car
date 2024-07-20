#include <stm32f10x.h>
#include "ultrasound.h"
#include "serial.h"
#include "main.h"
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
u16 pulseEdge = 0;
u32 width = 0;
volatile unsigned char pbuffer[100];

bool ults_init(){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef gpio;
	// Pin config for Ultrasound sensor Trig, PA0, PWM2/1
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio.GPIO_Pin = GPIO_Pin_0;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &gpio);
	
	// Pin config for Ultrasound sensor Echo, PA10, TIM1_CH3
	gpio.GPIO_Mode = GPIO_Mode_IPD;
	gpio.GPIO_Pin = GPIO_Pin_10;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &gpio);
	
	if(!extern_timer_config){
		TIM_TimeBaseInitTypeDef tim;
		
		tim.TIM_ClockDivision = TIM_CKD_DIV1;//T = 60.41667ms
		tim.TIM_CounterMode = TIM_CounterMode_Up;
		tim.TIM_Period = 10002 - 1;//period*psc > 4320720!
		tim.TIM_Prescaler = 432 - 1;
		tim.TIM_RepetitionCounter = 0;
		TIM_TimeBaseInit(TIM2, &tim);
		TIM_Cmd(TIM2, ENABLE);
	}
	
	TIM_OCInitTypeDef oc;
	oc.TIM_OCMode = TIM_OCMode_PWM1;
	oc.TIM_OCPolarity = TIM_OCPolarity_High;
	oc.TIM_OutputState = TIM_OutputState_Enable;
	oc.TIM_Pulse = 2;
	TIM_OC1Init(TIM2, &oc);
	TIM_OC1FastConfig(TIM2, TIM_OCFast_Enable);
	
	TIM_ICInitTypeDef ic;
	ic.TIM_Channel = TIM_Channel_3;
	ic.TIM_ICFilter = 0;
	ic.TIM_ICPolarity = TIM_ICPolarity_Rising;
	ic.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	ic.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInit(TIM1, &ic);
	TIM_ITConfig(TIM1, TIM_IT_CC3, ENABLE);
	
	NVIC_InitTypeDef nvic;
	nvic.NVIC_IRQChannel = TIM1_CC_IRQn;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	nvic.NVIC_IRQChannelPreemptionPriority = 1;
	nvic.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&nvic);
	return 1;
}

void ults_IRQHandler(){
	if(TIM_GetITStatus(TIM1, TIM_IT_CC3)){
		if(pulseEdge == 0){
			width = get_msticks();
			pulseEdge = 1;
			TIM_OC3PolarityConfig(TIM1, TIM_ICPolarity_Falling);
		}else{
			width = get_msticks() - width;
			pulseEdge = 0;
			TIM_OC3PolarityConfig(TIM1, TIM_ICPolarity_Rising);
			sprintf(pbuffer, "ultsdistance: %ld mm\n\r", width);
			u2_tx(pbuffer, 30);
			u3_tx(pbuffer, 30);
		}
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC3);
	}
}
