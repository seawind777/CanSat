/*
 * motor_control.c
 *
 *  Created on: Jun 10, 2025
 *      Author: Sergey
 */
#include "motor_control.h"
#include "stm32f4xx_hal.h"
#include "pca9685.h"
#include "FSM.h"

extern int32_t encoder_count;

ControlCommand cmd;

void MOT_ParseCmd(ControlCommand *rx){
	if(rx->reserved == 0x0F){
		if(rx->mode == 1){ //Manual
			//TODO: Handle manual CMD
			// Access 1st angle via rx->payload.manual.motor[0]
			if(cmd.payload.manual.motor[0] < rx->payload.manual.motor[0]){
				PCA9685_SetPin(1, 0, 0);
				PCA9685_SetPin(0, 4096, 0);
			}
			if(cmd.payload.manual.motor[0] > rx->payload.manual.motor[0]){
				PCA9685_SetPin(0, 0, 0);
				PCA9685_SetPin(1, 4096, 0);
			}

		}else{ //Auto
			rx->reserved = 0;
			//TODO: Handle auto CMD
			// Access 1st value via rx->payload.auto.pitch
			// Access 2nd value via rx->payload.auto.yawW
		}
		cmd = *rx;
	}
	else{
		rx->reserved = 0; //TODO: Error
	}
}
