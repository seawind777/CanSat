/*
 * MS5611.h
 *
 *  Created on: May 4, 2024
 *      Author: Nate Hunter
 */

#pragma once
#include "main.h"

#define MS56_CMD_RESET 0x1E
#define MS56_CMD_CAL_READ 0xA0
#define MS56_CMD_CONV_D1 0x40
#define MS56_CMD_CONV_D2 0x50
#define MS56_CMD_ADC_READ 0x00

#define MS56_OSR_256 0b000
#define MS56_OSR_512 0b001
#define MS56_OSR_1024 0b010
#define MS56_OSR_2048 0b011
#define MS56_OSR_4096 0b100
struct MS5611_CalData {
	float SENS_T1, OFF_T1, TCS, TCO, T_REF, TEMPSENS;
};

uint8_t MS5611_Init(SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *NSS_Port, uint16_t NSS_Pin);
void MS5611_SetOS(uint8_t tempOS, uint8_t pressOS);
void MS5611_Read(int32_t *temp, uint32_t *press);
int32_t MS5611_GetAltitude(uint32_t* pressure, uint32_t* seaLevelPressure);
