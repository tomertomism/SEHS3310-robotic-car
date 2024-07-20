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

#ifndef servo_h
#define servo_h
#include <stm32f10x.h>

void servo_init(vu16, vu16, vu16, vu16);//initialize servos outputs
void SV1O(vu16 value);
void SV2O(vu16 value);
void SV3O(vu16 value);
void SV4O(vu16 value);

extern vu16 SV1;//PWM3/1, PA6
extern vu16 SV2;//PWM3/2, PA7
extern vu16 SV3;//PWM3/3, PB0
extern vu16 SV4;//PWM3/4, PB1

#define sv_pwm_min 300
#define sv_pwm_max 1200
#define gripper_close 770
#define gripper_open 550
#define arm_down 550
#define arm_up 350

#endif
