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

extern int32_t encoder_count[6];

ControlCommand cmd;

void MOT_ParseCmd(ControlCommand *rx){
	if(rx->reserved == 0x0F){
		if(rx->mode == 1){ //Manual
			//TODO: Handle manual CMD
			// Access 1st angle via rx->payload.manual.motor[0]
//			for (int i =0; i<7; i++){
//				PID_SetTarget(i, rx->payload.manual.motor[i]*4);
//				if (encoder_count[i] > pidStates[i].targetEnc){
//					MOT_SetSpeed(i, -5);
//				} else if (encoder_count[i] < pidStates[i
//														  ].targetEnc){
//					MOT_SetSpeed(i, 5);
//				}
//			}

			PCA9685_SetPin(0, 4096, 0);
			PCA9685_SetPin(1, 0, 0);

			MOT_SetSpeed(0, 100);
			MOT_SetSpeed(1, 100);
			MOT_SetSpeed(2, 100);
			MOT_SetSpeed(3, 100);
			MOT_SetSpeed(4, 100);



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
						PCA9685_SetPin(2, 500 + (int16_t)((float)speed / 100.0f * (3000-500)), 0);
						PCA9685_SetPin(3, 0, 0);

					}else if (speed<0){
						PCA9685_SetPin(2, 0, 0);
						PCA9685_SetPin(3, 500 + (int16_t)((float)speed / 100.0f * (3000-500)), 0);
					}else{
						PCA9685_SetPin(2, 0, 0);
						PCA9685_SetPin(3, 0, 0);
					}
	break;
	case 2:
			if (speed>0){
						PCA9685_SetPin(10, 500 + (int16_t)((float)speed / 100.0f * (3000-500)), 0);
						PCA9685_SetPin(11, 0, 0);

					}else if (speed<0){
						PCA9685_SetPin(10, 0, 0);
						PCA9685_SetPin(11, 500 + (int16_t)((float)speed / 100.0f * (3000-500)), 0);
					}else{
						PCA9685_SetPin(10, 0, 0);
						PCA9685_SetPin(11, 0, 0);
					}
	break;
	case 3:
			if (speed>0){
						PCA9685_SetPin(0, 500 + (int16_t)((float)speed / 100.0f * (3000-500)), 0);
						PCA9685_SetPin(1, 0, 0);

					}else if (speed<0){
						PCA9685_SetPin(0, 0, 0);
						PCA9685_SetPin(1, 500 + (int16_t)((float)speed / 100.0f * (3000-500)), 0);
					}else{
						PCA9685_SetPin(0, 0, 0);
						PCA9685_SetPin(1, 0, 0);
					}
	break;
	case 4:
		if (speed>0){
					PCA9685_SetPin(4, 500 + (int16_t)((float)speed / 100.0f * (3000-500)), 0);
					PCA9685_SetPin(5, 0, 0);

				}else if (speed<0){
					PCA9685_SetPin(4, 0, 0);
					PCA9685_SetPin(5, 500 + (int16_t)((float)speed / 100.0f * (3000-500)), 0);
				}else{
					PCA9685_SetPin(4, 0, 0);
					PCA9685_SetPin(5, 0, 0);
				}
	break;
	}
}

void PID_SetTarget(int8_t Channel, int16_t target) {
    if (Channel >= 1 && Channel < 7) {
    	pidStates[Channel].targetEnc = target;
        pidStates[Channel].active = 1;
        pidStates[Channel].integral = 0;
        }
}

void PID_Update(int8_t Channel, int16_t currentEnc) {
    if (Channel < 1 || Channel >= 7 || !pidStates[Channel].active) {
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
    if (error == 0) {  // Threshold of 5 encoder ticks
        MOT_SetSpeed(Channel, 0);
        state->active = 0;  // Disable PID until new target is set
    }
}
