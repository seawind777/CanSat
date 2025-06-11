/*
 * motor_control.h
 *
 *  Created on: Jun 10, 2025
 *      Author: Sergey
 */

#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_
#include <stdint.h>
#include "telemetry_lora.h"

void MOT_ParseCmd(ControlCommand *rx);

#endif /* INC_MOTOR_CONTROL_H_ */
