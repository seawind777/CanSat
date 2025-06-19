/*
 * motor_control.h
 *
 *  Created on: Jun 10, 2025
 *      Author: Sergey
 */

#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_
#include <stdint.h>
#include <stdlib.h>
#include "telemetry_lora.h"

#define KP 0.5    // Proportional gain
#define KI 0.0   // Integral gain
#define KD 0.1    // Derivative gain

typedef struct {
    int16_t targetEnc;       // Target encoder value
    int16_t prevEnc;         // Previous encoder reading
    int32_t integral;        // Integral term accumulator
    uint8_t active;          // Whether PID control is active for this channel
} PID_State;

void MOT_ParseCmd(ControlCommand *rx);

void MOT_SetSpeed(int8_t Channel, int8_t speed);

void PID_Update(int8_t Channel, int16_t currentEnc);

#endif /* INC_MOTOR_CONTROL_H_ */
