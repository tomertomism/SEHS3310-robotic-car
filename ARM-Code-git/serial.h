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

#ifndef serial_h
#define serial_h
#include <stm32f10x.h>
#include <stdbool.h>
#include <stdio.h>

bool usart_init(u32, u32);
void u2_tx(char*, u16);
char u2_rx(void);
void u3_tx(char*, u16);
char u3_rx(void);
#endif
