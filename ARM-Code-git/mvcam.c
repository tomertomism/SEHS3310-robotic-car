#include <stdbool.h>
#include <math.h>
#include "mvcam.h"
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

static volatile char msg3_rx[100];//Rx message container for usart3
static char com_err[32] = "Connection Error: Time out!\n\r";
static char haha = 'K';
static char openmv_msg[23] = "Open-MV connected!\n\r";
static volatile char msg3_tx[20];

u8 from_openmv(u8 num){
	char state;
	if(num == 0){
		do{
			u3_tx("STA", 3);
			state = u3_rx();
		}while(state == 'Z');
		if(state == '0') return 0;
		else if(state == '1') return 1;
		else if(state == '2') return 2;
		else if(state == '3') return 3;
		else if(state == '4') return 4;
		else return 0xFF;
	}else if(num == 1){
		do{
			u3_tx("RQ1", 3);
			state = u3_rx();
		}while(state == 'Z');
		if(state == 'L') return 11;
		else if(state == 'R') return 12;
		else if(state == 'G') return 13;
		else if(state == 'S') return 14;
		else if(state == 'N') return 15;
		else return 0xFF;
		
	}else if(num == 2){
		do{
			u3_tx("RQ2", 3);
			state = u3_rx();
		}while(state == 'Z');
		if(state == 'L') return 21;
		else if(state == 'R') return 22;
		else if(state == 'S') return 23;
		else return 0xFF;
	}else if(num == 3){
		do{
			u3_tx("RQ3", 3);
			state = u3_rx();
		}while(state == 'Z');
		if(state == 'L') return 31;
		else if(state == 'R') return 32;
		else if(state == 'S') return 33;
		else if(state == 'T') return 34;
		else return 0xFF;
	}else if(num == 4){
		do{
			u3_tx("RQ4", 3);
			state = u3_rx();
		}while(state == 'Z');
		if(state == 'L') return 41;
		else if(state == 'R') return 42;
		else if(state == 'S') return 43;
		else if(state == 'N') return 44;
		else return 0xFF;
	}
	return 0xFF;
}
