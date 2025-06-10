/*
 * motor_control.c
 *
 *  Created on: Jun 10, 2025
 *      Author: Sergey
 */
#include "motor_control.h"
#include "telemetry_lora.h"

static ControlCommand cmd;

void MOT_ParseCmdManual(uint8_t *rx){
 cmd = *((ControlCommand*) rx);
 if(cmd.reserved == 0x0F){
	 cmd.reserved = 0x0F;
	 cmd.payload.manual.motor[0]; //Angle for MOT0
	 //TODO: handle new manual command
 }
}

