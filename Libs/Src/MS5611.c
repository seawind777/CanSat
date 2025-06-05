/*
 * MS5611.c
 *
 *  Created on: May 4, 2024
 *      Author: Nate Hunter
 */
#include "MS5611.h"
#include <math.h>
static SPI_HandleTypeDef *spi;
static uint8_t overSamplimg = 0; //|0|OS_Press[4:6]|OS_temp[1:3]|0|
static uint8_t txBuf[1];
static GPIO_TypeDef *_NSS_Port;
static uint16_t _NSS_Pin;
struct MS5611_CalData calData;
static const uint8_t delayArr[5] = { 1, 2, 3, 5, 9 };

static void MS5611_GetADC(uint8_t cmd, uint32_t *res){
	*res = 0;
	_NSS_Port->ODR &= ~_NSS_Pin;
	txBuf[0] = cmd | ((overSamplimg >> 3) & 0b1110);
	HAL_SPI_Transmit(spi, txBuf, 1, 1000);
	_NSS_Port->ODR |= _NSS_Pin;
	if(cmd & (1 << 4)) HAL_Delay(delayArr[overSamplimg >> 1 & 0b111]);
	else HAL_Delay(delayArr[overSamplimg >> 4 & 0b111]);
	_NSS_Port->ODR &= ~_NSS_Pin;
	txBuf[0] = MS56_CMD_ADC_READ;
	HAL_SPI_Transmit(spi, txBuf, 1, 1000);
	HAL_SPI_Receive(spi, (uint8_t*) res, 3, 1000);
	_NSS_Port->ODR |= _NSS_Pin;
	*res = (*res & 0xFF) << 16 | (*res & 0xFF00) | *res >> 16;
}

uint8_t MS5611_Init(SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *NSS_Port, uint16_t NSS_Pin){
	uint16_t rawCal[6];
	spi = spiHandle;
	_NSS_Port = NSS_Port;
	_NSS_Pin = NSS_Pin;
	for(uint8_t i = 1; i < 7; i++) {
		_NSS_Port->ODR &= ~_NSS_Pin;
		txBuf[0] = MS56_CMD_CAL_READ | (i << 1);
		HAL_SPI_Transmit(spi, txBuf, 1, 1000);
		HAL_SPI_Receive(spi, (uint8_t*) &rawCal[i - 1], 2, 1000);
		rawCal[i - 1] = (rawCal[i - 1] << 8 & 0xFF00) | rawCal[i - 1] >> 8;
		_NSS_Port->ODR |= _NSS_Pin;
	}
	calData.SENS_T1 = rawCal[0] * 32768.0;
	calData.OFF_T1 = rawCal[1] * 65536.0;
	calData.TCS = rawCal[2] / 256.0;
	calData.TCO = rawCal[3] / 128.0;
	calData.T_REF = rawCal[4] * 256.0;
	calData.TEMPSENS = rawCal[5] / 8388608.0;
	return 1;
}

void MS5611_SetOS(uint8_t tempOS, uint8_t pressOS){
	overSamplimg = ((0b111 & tempOS) | ((0b111 & pressOS) << 3)) << 1;
}

void MS5611_Read(int32_t *temp, uint32_t *press){
	uint32_t D1, D2;
	float dT, off, sens, tempComp, dt2 = 0, off2 = 0, sens2 = 0;
	MS5611_GetADC(MS56_CMD_CONV_D1, &D1);
	MS5611_GetADC(MS56_CMD_CONV_D2, &D2);
	dT = D2 - calData.T_REF;
	*temp = 2000 + dT * calData.TEMPSENS;
	if(*temp < 2000) { //LOW temp
		dt2 = dT * dT / 2147483648.0;
		tempComp = (*temp - 2000.0);
		off2 = 5.0 * tempComp * tempComp / 2.0;
		sens2 = 5.0 * tempComp * tempComp / 4.0;
	}
	if(*temp < -1500) { //XTRA LOW temp
		tempComp = (*temp + 1500.0);
		off2 += 7.0 * tempComp * tempComp;
		sens2 += 11.0 * tempComp * tempComp / 2.0;
	}
	off = calData.OFF_T1 + calData.TCO * dT - off2;
	sens = calData.SENS_T1 + calData.TCS * dT - sens2;
	*temp -= dt2;
	*press = (D1 * sens / 2097152.0 - off) / 32768.0;
}

int32_t MS5611_GetAltitude(uint32_t* pressure, uint32_t* seaLevelPressure) {  //Pressure in Pa, height in cm
  return 4433000 * (1.0f - pow((float)*pressure / *seaLevelPressure, 0.1903));
}
