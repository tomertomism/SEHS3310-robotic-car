#include "i2c.h"
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
void i2c1_init(u32 clk_spd){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	
	GPIO_InitTypeDef gpio;
	I2C_InitTypeDef i2c;
	
	//Pin config for I2C_1 SCL, PB6
	//Pin config for I2C_1 SDA, PB7
	gpio.GPIO_Mode = GPIO_Mode_AF_OD;
	gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &gpio);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	//GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);
	
	i2c.I2C_Ack = I2C_Ack_Enable;
	i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	i2c.I2C_ClockSpeed = clk_spd;
	i2c.I2C_DutyCycle = I2C_DutyCycle_2;
	i2c.I2C_Mode = I2C_Mode_I2C;
	i2c.I2C_OwnAddress1 = 0;
	I2C_Init(I2C1, &i2c);
	I2C_Cmd(I2C1, ENABLE);
}