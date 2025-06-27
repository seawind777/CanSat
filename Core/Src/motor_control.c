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

PID_State pidStates[6] = {0};

PID_Angular_State angularState = {0};

extern int32_t encoder_count[6];

ControlCommand cmd;

void MOT_ParseCmd(ControlCommand *rx){
	if(rx->reserved == 0x0F){
		if(rx->mode == 1){ //Manual
			//TODO: Handle manual CMD
			// Access 1st angle via rx->payload.manual.motor[0]
			for (int i = 0; i<4; i++){
				angularState.active = 0;
				PID_SetTarget(i, rx->payload.manual.motor[i]*4);


//				if (encoder_count[i] > pidStates[i].targetEnc){
//					MOT_SetSpeed(i, -5);
//				} else if (encoder_count[i] < pidStates[i
//														  ].targetEnc){
//					MOT_SetSpeed(i, 5);
				}
			PID_SetTarget(5, rx->payload.manual.motor[5]*40);

//			PCA9685_SetPin(6, 0, 0);
//			PCA9685_SetPin(7, 4096, 0);
//			MOT_SetSpeed(5, 100);

//			PCA9685_SetPin(0, 4096, 0);
//			PCA9685_SetPin(1, 0, 0);
//
//			MOT_SetSpeed(0, 100);
//			MOT_SetSpeed(1, 100);
//			MOT_SetSpeed(2, 100);
//			MOT_SetSpeed(3, 100);
//			MOT_SetSpeed(4, 100);



		}else{ //Auto
			rx->reserved = 0;

//			PID_Speed_SetTarget(0.0f, 0.0f, 0.0f);

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

void MOT_SetSpeed(int8_t Channel, int8_t speed){
	switch (Channel){
	case 0:
		if (speed>0){
			PCA9685_SetPin(8, 500 + (int16_t)((float)speed / 100.0f * (3000-500)), 0);
			PCA9685_SetPin(9, 0, 0);

		}else if (speed<0){
			PCA9685_SetPin(8, 0, 0);
			PCA9685_SetPin(9, 500 + (int16_t)((float)speed / 100.0f * (3000-500)), 0);
		}else{
			PCA9685_SetPin(8, 0, 0);
			PCA9685_SetPin(9, 0, 0);
		}
	break;
	case 1:
			if (speed>0){
						PCA9685_SetPin(2, 1000 + (int16_t)((float)speed / 100.0f * (4000-1000)), 0);
						PCA9685_SetPin(3, 0, 0);

					}else if (speed<0){
						PCA9685_SetPin(2, 0, 0);
						PCA9685_SetPin(3, 1000 + (int16_t)((float)speed / 100.0f * (4000-1000)), 0);
					}else{
						PCA9685_SetPin(2, 0, 0);
						PCA9685_SetPin(3, 0, 0);
					}
	break;
	case 2:
			if (speed>0){
						PCA9685_SetPin(10, 1000 + (int16_t)((float)speed / 100.0f * (4000-1000)), 0);
						PCA9685_SetPin(11, 0, 0);

					}else if (speed<0){
						PCA9685_SetPin(10, 0, 0);
						PCA9685_SetPin(11, 1000 + (int16_t)((float)speed / 100.0f * (4000-1000)), 0);
					}else{
						PCA9685_SetPin(10, 0, 0);
						PCA9685_SetPin(11, 0, 0);
					}
	break;
	case 3:
			if (speed>0){
						PCA9685_SetPin(0, 1000 + (int16_t)((float)speed / 100.0f * (4000-1000)), 0);
						PCA9685_SetPin(1, 0, 0);

					}else if (speed<0){
						PCA9685_SetPin(0, 0, 0);
						PCA9685_SetPin(1, 1000 + (int16_t)((float)speed / 100.0f * (4000-1000)), 0);
					}else{
						PCA9685_SetPin(0, 0, 0);
						PCA9685_SetPin(1, 0, 0);
					}
	break;
	case 4:
		if (speed>0){
					PCA9685_SetPin(4, 1000 + (int16_t)((float)speed / 100.0f * (4000-1000)), 0);
					PCA9685_SetPin(5, 0, 0);

				}else if (speed<0){
					PCA9685_SetPin(4, 0, 0);
					PCA9685_SetPin(5, 1000 + (int16_t)((float)speed / 100.0f * (4000-1000)), 0);
				}else{
					PCA9685_SetPin(4, 0, 0);
					PCA9685_SetPin(5, 0, 0);
				}
	break;
	case 5:
			if (speed<0){
						PCA9685_SetPin(6, 500 + (int16_t)((float)speed / 100.0f * (3000-500)), 0);
						PCA9685_SetPin(7, 0, 0);

					}else if (speed>0){
						PCA9685_SetPin(6, 0, 0);
						PCA9685_SetPin(7, 500 + (int16_t)((float)speed / 100.0f * (3000-500)), 0);
					}else{
						PCA9685_SetPin(6, 0, 0);
						PCA9685_SetPin(7, 0, 0);
					}
	break;
	}
}

void PID_SetTarget(int8_t Channel, int16_t target) {
    if (Channel >= 0 && Channel < 6) {
    	pidStates[Channel].targetEnc = target;
        pidStates[Channel].active = 1;
        pidStates[Channel].integral = 0;
        }
}

void PID_Update(int8_t Channel, int16_t currentEnc) {
    if (Channel < 0 || Channel >= 6 || !pidStates[Channel].active) {
        return;
    }

    PID_State* state = &pidStates[Channel];

    // Calculate error (difference between target and current position)
    int16_t error = state->targetEnc - currentEnc;

    // Proportional term
    float p_term = KP * error;

    // Integral term (with anti-windup)
    state->integral += error;
    // Simple anti-windup - limit integral term
    if (state->integral > 1000) state->integral = 1000;
    if (state->integral < -1000) state->integral = -1000;
    float i_term = KI * state->integral;

    // Derivative term (rate of change of error)
    int16_t deriv = (currentEnc - state->prevEnc);
    float d_term = KD * deriv;

    // Store current encoder value for next time
    state->prevEnc = currentEnc;

    // Calculate PID output
    float output = p_term + i_term - d_term;  // Note: subtract d_term since deriv is based on position change

    // Convert to int8_t speed (-100 to 100)
    int8_t speed;
    if (output > 100) speed = 100;
    else if (output < -100) speed = -100;
    else speed = (int8_t)output;

    // Set motor speed
    MOT_SetSpeed(Channel, speed);

    // Optional: disable PID when close to target to prevent oscillation
    if (error = 0) {  // Threshold of 5 encoder ticks
        MOT_SetSpeed(Channel, 0);
        state->active = 0;  // Disable PID until new target is set
    }
}


void PID_Speed_SetTarget(float wx, float wy, float wz) {
	angularState.target_wx = wx;
	angularState.target_wy = wy;
	angularState.target_wz = wz;
	angularState.active = 1;
	angularState.integral_wx = 0;
	angularState.integral_wy = 0;
	angularState.integral_wz = 0;
}


void PID_Speed_Update(float wx, float wy, float wz){

	if (!angularState.active){
		return;
	}

	PID_Angular_State* state = &angularState;

	float error_x = state->target_wx - wx;
	float error_y = state->target_wy - wy;
	float error_z = state->target_wz - wz;

	state->integral_wx += error_x;
	state->integral_wy += error_y;
	state->integral_wz += error_z;

	if (state->integral_wx > 1000000) state->integral_wx = 1000000;
	if (state->integral_wx < -1000000) state->integral_wx = -1000000;
	if (state->integral_wy > 1000000) state->integral_wy = 1000000;
	if (state->integral_wy < -1000000) state->integral_wy = -1000000;
	if (state->integral_wz > 1000000) state->integral_wz = 1000000;
	if (state->integral_wz < -1000000) state->integral_wz = -1000000;

	float output_elevon_wx = KP_elevon_wx*error_x + KI_elevon_wx*state->integral_wx - KD_elevon_wx*(wx - state->prev_wx);
	float output_elevon_wy = KP_elevon_wy*error_y + KI_elevon_wy*state->integral_wy - KD_elevon_wy*(wy - state->prev_wy);

	float output_stab_wx = KP_stab_wx*error_x + KI_stab_wx*state->integral_wx - KD_stab_wx*(wx - state->prev_wx);
	float output_stab_wy = KP_stab_wy*error_y + KI_stab_wy*state->integral_wy - KD_stab_wy*(wy - state->prev_wy);

	float output_fin_wz = KP_fin_wz*error_z + KI_fin_wz*state->integral_wz - KD_fin_wz*(wz - state->prev_wz);


	state->prev_wx = wx;
	state->prev_wy = wy;
	state->prev_wz = wz;

	if (output_elevon_wx > 10)  output_elevon_wx = 10;
	else if (output_elevon_wx < -10) output_elevon_wx = -10;
	if (output_elevon_wy > 5)  output_elevon_wy = 5;
	else if (output_elevon_wy < -5) output_elevon_wy = -5;

	if (output_stab_wx > 10)  output_stab_wx = 10;
	else if (output_stab_wx < -10) output_stab_wx = -10;
	if (output_stab_wy > 10)  output_stab_wy = 10;
	else if (output_stab_wy< -10) output_stab_wx = -10;

	if (output_fin_wz > 10)  output_fin_wz = 10;
	else if (output_fin_wz < -10) output_fin_wz = -10;



	PID_SetTarget(0, (int16_t)(output_elevon_wx+output_elevon_wy)*4);
	PID_SetTarget(1, (int16_t)(-output_elevon_wx+output_elevon_wy)*4);
	PID_SetTarget(2, (int16_t)(output_stab_wx+output_stab_wy)*4);
	PID_SetTarget(3, (int16_t)(-output_stab_wx+output_stab_wy)*4);
	PID_SetTarget(4, (int16_t)(output_fin_wz)*4);

}
