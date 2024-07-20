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

#ifndef motor_h
#define motor_h
#define motor_pwm_max 100
#define motor_pwm_min -100
#define encoder_min 20
#define encoder_max 105
#include <stm32f10x.h>
#include <stdio.h>
#include <stdbool.h>

bool JGB520_Init(volatile u16, volatile u16);
void motor_Output(volatile u16, volatile u16);
void L_motor_pos_out(vu16);
void L_motor_neg_out(vu16);
void R_motor_pos_out(vu16);
void R_motor_neg_out(vu16);
#endif
