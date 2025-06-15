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

void MOT_ParseCmd(ControlCommand *rx){
	static uint32_t cnt = 0;
	static float angle = 0;
	if(rx->reserved == 0x0F){
		cnt++;
		if(rx->mode == 1){ //Manual
			//TODO: Handle manual CMD
			PCA9685_SetPin(0, 4096, 0);
			PCA9685_SetPin(1, 0, 0);
			// Access 1st angle via rx->payload.manual.motor[0]
			angle = ENC_GetAngle();
		}else{ //Auto
			rx->reserved = 0;
			//TODO: Handle auto CMD
			// Access 1st value via rx->payload.auto.pitch
			// Access 2nd value via rx->payload.auto.yawW
		}
	}
	else{
		rx->reserved = 0; //TODO: Error
	}
}
