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
#ifndef __ULTRASOUND_H
#define __ULTRASOUND_H
#include <stm32f10x.h>
#include <stdio.h>
#include <stdbool.h>

extern u16 pulseEdge;
extern u32 width;
extern volatile unsigned char pbuffer[100];
bool ults_init(void);
void ults_IRQHandler(void);


#endif
