/*
 * motor_control.c
 *
 *  Created on: Jun 10, 2025
 *      Author: Sergey
 */
#include "motor_control.h"

static ControlCommand cmd;

void MOT_ParseCmd(ControlCommand *rx){
	static uint32_t cnt = 0;
 cmd = *rx;
 if(cmd.mode == 1){ //Manual
	 if(cmd.reserved != 0x0F)
		 cmd.reserved = 0x0F; //Placeholder, Error
	 else{
		 //TODO: handle new manual command
		 //cmd.payload.manual.motor[0]
	 }
 }
 else { //Auto
	 if(cmd.reserved != 0x0F)
		 cmd.reserved = 0x0F; //Placeholder, Error
	 else{
		 //TODO: handle new Auto command
		 //cmd.payload.auto_.pitch;
		 //cmd.payload.auto_.yaw;
	 }
 }
 cnt++;

}
