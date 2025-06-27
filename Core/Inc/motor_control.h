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

#define KP 2    // Proportional gain
#define KI 0.5   // Integral gain
#define KD 5    // Derivative gain

#define KP_elevon_wx 0.005
#define KI_elevon_wx 0.00001
#define KD_elevon_wx 0.001

#define KP_elevon_wy 0.005
#define KI_elevon_wy 0.00001
#define KD_elevon_wy 0.001

#define KP_stab_wx 0.005
#define KI_stab_wx 0.00001
#define KD_stab_wx 0.001

#define KP_stab_wy 0.005
#define KI_stab_wy 0.00001
#define KD_stab_wy 0.001

#define KP_fin_wz 0.005
#define KI_fin_wz 0.00001
#define KD_fin_wz 0.001


typedef struct {
    int16_t targetEnc;       // Target encoder value
    int16_t prevEnc;         // Previous encoder reading
    int32_t integral;        // Integral term accumulator
    uint8_t active;          // Whether PID control is active for this channel
} PID_State;

typedef struct {
	float target_wx;       // Target encoder value
	float target_wy;
	float target_wz;
    float prev_wx;
    float prev_wy;
    float prev_wz; // Previous encoder reading
    float integral_wx;
    float integral_wy;
    float integral_wz;// Integral term accumulator
    uint8_t active;
} PID_Angular_State;

void MOT_ParseCmd(ControlCommand *rx);

void MOT_SetSpeed(int8_t Channel, int8_t speed);

void PID_Update(int8_t Channel, int16_t currentEnc);

void PID_Speed_Update(float wx, float wy, float wz);

#endif /* INC_MOTOR_CONTROL_H_ */
