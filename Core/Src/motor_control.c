/*
 * motor_control.c
 *
 *  Created on: Jun 10, 2025
 *      Author: Sergey
 */
#include "motor_control.h"

void MOT_ParseCmd(ControlCommand *rx){
	static uint32_t cnt = 0;
	if(rx->reserved == 0x0F){
		cnt++;
		if(rx->mode == 1){ //Manual
			rx->reserved = 0;
		}else{ //Auto
			rx->reserved = 0;
		}
	}
	else{
		rx->reserved = 0;
	}
}
