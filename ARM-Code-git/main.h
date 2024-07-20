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

#ifndef main_h
#define main_h
#include <stm32f10x.h>
#include <stdbool.h>

#define extern_timer_config true
static vu32 msticks = 0;

extern int motor_en;
extern int serial_en;
extern int servo_en;
extern int ults_en;
extern int tof_en;
extern int debug_en;

void timer_init(void);
void TIM1_UP_IRQHandler(void);
void TIM1_CC_IRQHandler(void);
void TIM2_IRQHandler(void);
void TIM4_IRQHandler(void);
vu32 get_msticks(void);
static void debug_init(void);

void debug_init(){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitTypeDef gpio;
	//Pin config for debug LED, PA5
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio.GPIO_Pin = GPIO_Pin_5;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &gpio);

	gpio.GPIO_Mode = GPIO_Mode_Out_OD;
	gpio.GPIO_Pin = GPIO_Pin_0;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &gpio);
	GPIO_SetBits(GPIOC, GPIO_Pin_0);
	
	gpio.GPIO_Mode = GPIO_Mode_Out_OD;
	gpio.GPIO_Pin = GPIO_Pin_1;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &gpio);
	GPIO_SetBits(GPIOC, GPIO_Pin_1);
	
	gpio.GPIO_Mode = GPIO_Mode_Out_OD;
	gpio.GPIO_Pin = GPIO_Pin_2;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &gpio);
	GPIO_SetBits(GPIOC, GPIO_Pin_2);
	
	gpio.GPIO_Mode = GPIO_Mode_Out_OD;
	gpio.GPIO_Pin = GPIO_Pin_3;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &gpio);
	GPIO_SetBits(GPIOC, GPIO_Pin_3);
	
	gpio.GPIO_Mode = GPIO_Mode_IPD;
	gpio.GPIO_Pin = GPIO_Pin_13;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &gpio);
	
	//GPIO_SetBits(GPIOC, GPIO_Pin_0);
	/*
	
	GPIO_SetBits(GPIOA, GPIO_Pin_5);
	*/
}

#endif
