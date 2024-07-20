#include <math.h>
#include "servo.h"
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

vu16 SV1 = 0;
vu16 SV2 = 0;
vu16 SV3 = 0;
vu16 SV4 = 0;

void servo_init(vu16 a, vu16 b, vu16 c, vu16 d){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef gpio;
	//Pin config for Servo 1, PA6, PWM3/1
	//Pin config for Servo 2, PA7, PWM3/2
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &gpio);
	
	//Pin config for Servo 3, PB0, PWM3/3
	//Pin config for Servo 4, PB1, PWM3/4
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &gpio);
	
	if(!extern_timer_config){
		TIM_TimeBaseInitTypeDef tim;
		
		tim.TIM_ClockDivision = TIM_CKD_DIV1;//T = 20ms
		tim.TIM_CounterMode = TIM_CounterMode_Up;
		tim.TIM_Period = 10000 - 1;
		tim.TIM_Prescaler = 144 - 1;
		tim.TIM_RepetitionCounter = 0;
		TIM_TimeBaseInit(TIM3, &tim);
		TIM_Cmd(TIM3, ENABLE);
	}
	SV1 = a;
	SV2 = b;
	SV3 = c;
	SV4 = d;
	
	//Output config for first SERVO, preloaded config, initalize 0% pwm
	SV1O(SV1);
	//Output config for second SERVO, preloaded config, initalize 0% pwm
	SV2O(SV2);
	//Output config for third SERVO, preloaded config, initalize 0% pwm
	SV3O(SV3);
	//Output config for fourth SERVO, preloaded config, initalize 0% pwm
	SV4O(SV4);
}

void SV1O(vu16 value){
	TIM_OCInitTypeDef SV1OP;
	SV1OP.TIM_OCMode = TIM_OCMode_PWM1;
	SV1OP.TIM_OCPolarity = TIM_OCPolarity_High;
	SV1OP.TIM_OutputState = TIM_OutputState_Enable;
	SV1OP.TIM_Pulse = value;
	TIM_OC1Init(TIM3, &SV1OP);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
}

void SV2O(vu16 value){
	TIM_OCInitTypeDef SV2OP;
	SV2OP.TIM_OCMode = TIM_OCMode_PWM1;
	SV2OP.TIM_OCPolarity = TIM_OCPolarity_High;
	SV2OP.TIM_OutputState = TIM_OutputState_Enable;
	SV2OP.TIM_Pulse = value;
	TIM_OC2Init(TIM3, &SV2OP);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
}

void SV3O(vu16 value){
	TIM_OCInitTypeDef SV3OP;
	SV3OP.TIM_OCMode = TIM_OCMode_PWM1;
	SV3OP.TIM_OCPolarity = TIM_OCPolarity_High;
	SV3OP.TIM_OutputState = TIM_OutputState_Enable;
	SV3OP.TIM_Pulse = value;
	TIM_OC3Init(TIM3, &SV3OP);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
}

void SV4O(vu16 value){
	TIM_OCInitTypeDef SV4OP;
	SV4OP.TIM_OCMode = TIM_OCMode_PWM1;
	SV4OP.TIM_OCPolarity = TIM_OCPolarity_High;
	SV4OP.TIM_OutputState = TIM_OutputState_Enable;
	SV4OP.TIM_Pulse = value;
	TIM_OC4Init(TIM3, &SV4OP);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
}
