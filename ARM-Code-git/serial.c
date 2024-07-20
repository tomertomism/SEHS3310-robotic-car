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

bool usart_init(u32 usart2_ubrr, u32 usart3_ubrr){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	USART_InitTypeDef usart;
	GPIO_InitTypeDef gpio;
	
	if(usart2_ubrr != 0){
		//No Remap available
		//Pin config for USART2 Tx, PA2
		gpio.GPIO_Mode = GPIO_Mode_AF_PP;
		gpio.GPIO_Pin = GPIO_Pin_2;
		gpio.GPIO_Speed = GPIO_Speed_2MHz;
		GPIO_Init(GPIOA, &gpio);
		
		//Pin config for USART2 Rx, PA3
		gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		gpio.GPIO_Pin = GPIO_Pin_3;
		gpio.GPIO_Speed = GPIO_Speed_2MHz;
		GPIO_Init(GPIOA, &gpio);
		
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
		
		usart.USART_BaudRate = usart2_ubrr;
		usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		usart.USART_Parity = USART_Parity_No;
		usart.USART_StopBits = USART_StopBits_1;
		usart.USART_WordLength = USART_WordLength_8b;
		USART_Init(USART2, &usart);
		USART_Cmd(USART2, ENABLE);
	}
	
	if(usart3_ubrr != 0){
		if(IS_GPIO_REMAP(GPIO_PartialRemap_USART3)){
			//Pin config for USART3 Tx, PC10(Remap)
			gpio.GPIO_Mode = GPIO_Mode_AF_PP;
			gpio.GPIO_Pin = GPIO_Pin_10;
			gpio.GPIO_Speed = GPIO_Speed_2MHz;
			GPIO_Init(GPIOC, &gpio);
			
			//Pin config for USART3 Rx, PC11(Remap)
			gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
			gpio.GPIO_Pin = GPIO_Pin_11;
			gpio.GPIO_Speed = GPIO_Speed_2MHz;
			GPIO_Init(GPIOC, &gpio);
		}else{
			//Pin config for USART3 Tx, PB10
			gpio.GPIO_Mode = GPIO_Mode_AF_PP;
			gpio.GPIO_Pin = GPIO_Pin_10;
			gpio.GPIO_Speed = GPIO_Speed_2MHz;
			GPIO_Init(GPIOB, &gpio);
			
			//Pin config for USART3 Rx, PB11
			gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
			gpio.GPIO_Pin = GPIO_Pin_11;
			gpio.GPIO_Speed = GPIO_Speed_2MHz;
			GPIO_Init(GPIOB, &gpio);
		}
		
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
		
		usart.USART_BaudRate = usart3_ubrr;
		usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		usart.USART_Parity = USART_Parity_No;
		usart.USART_StopBits = USART_StopBits_1;
		usart.USART_WordLength = USART_WordLength_8b;
		USART_Init(USART3, &usart);
		USART_Cmd(USART3, ENABLE);
	}
	return 1;
}

void u2_tx(char *buffer, u16 size){
	while(size--){
		while(!USART_GetFlagStatus(USART2, USART_FLAG_TC));
		USART_SendData(USART2, *buffer++);
	}
}

char u2_rx(){
	char ch;
	while(!USART_GetFlagStatus(USART2, USART_FLAG_RXNE));
	ch = USART_ReceiveData(USART2) & 0xFF;
	if(debug_en) u2_tx(&ch, 2);
	return ch;
}

void u3_tx(char *buffer, u16 size){
	while(size--){
		while(!USART_GetFlagStatus(USART3, USART_FLAG_TC));
		USART_SendData(USART3, *buffer++);
	}
}

char u3_rx(){
	char ch = 0;
	while(!USART_GetFlagStatus(USART3, USART_FLAG_RXNE));
	ch = USART_ReceiveData(USART3) & 0xFF;
	if(debug_en) u2_tx(&ch, 1);
	return ch;
}
