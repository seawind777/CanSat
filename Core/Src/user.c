/**
 * @file user.c
 * @brief User configuration and utility functions for CanSat-Regular-HZ
 */
#include "user.h"
#include "MS5611.h"
#include "LIS3.h"
#include "LSM6.h"
#include "LoRa.h"
#include "MicroSD.h"
#include "W25Qx.h"
#include "CircularBuffer.h"
#include "telemetry_lora.h"

void StoreVectAbs(TelemetryRaw *dat) {
	dat->vectAbs = 0;
	for (uint8_t i = 0; i < 3; i++)
		dat->vectAbs += dat->accelData[i] * dat->accelData[i];
	dat->vectAbs = sqrt(dat->vectAbs);
}

void gyroCalibration(gyrobias *bias, int iterations) {

    float sumX = 0, sumY = 0, sumZ = 0;


    for (int i = 0; i < iterations; i++) {

    	float accelData[3];
    	float gyroData[3];

    	LSM6_Read(accelData, gyroData);



        sumX += gyroData[0];
        sumY +=gyroData[1];
        sumZ += gyroData[2];

    }


    bias->x = sumX / iterations;
    bias->y = sumY / iterations;
    bias->z = sumZ / iterations;
}

void ImuGet(TelemetryRaw *imuData){
	LIS3_Read(imuData->magData);
	LSM6_Read(imuData->accelData, imuData->gyroData);
}


void ImuGetAll(TelemetryRaw *imuData) {
	imuData->time = HAL_GetTick();
	MS5611_Read(&imuData->temp, &imuData->press);
	LIS3_Read(imuData->magData);
	LSM6_Read(imuData->accelData, imuData->gyroData);
	StoreVectAbs(imuData);
	imuData->altitude = MS5611_GetAltitude(&imuData->press, &imuData->press0);
}

void ImuSaveAll(TelemetryRaw *imuData, TelemetryPacket *tx, LoRa_Handle_t *lora, W25Qx_Device *wq) {
	Telemetry_convertRawToPacket(imuData, tx);
	LoRa_Transmit(lora, tx, sizeof(TelemetryPacket));  //Calculated On-Air ~15ms
	FlashLED(LED2_Pin);
	W25Qx_WriteData(wq, imuData->wqAdr, tx, sizeof(TelemetryPacket));
	imuData->wqAdr += sizeof(TelemetryPacket);
	FlashLED(LED1_Pin);
	if (microSD_Write(tx, sizeof(TelemetryPacket), SD_FILENAME) != FR_OK) {
		FlashLED(LED_ERR_Pin);
		MX_FATFS_Init();
		microSD_Init();
	}
	FlashLED(0);
}

void FlashLED(uint16_t led) {
	LED1_GPIO_Port->ODR &= ~LED1_Pin;
	LED2_GPIO_Port->ODR &= ~LED2_Pin;
	LED_ERR_GPIO_Port->ODR &= ~LED_ERR_Pin;
//@formatter:off
	switch (led){
		case LED1_Pin: LED1_GPIO_Port->ODR |= LED1_Pin; break;
		case LED2_Pin: LED2_GPIO_Port->ODR |= LED2_Pin; break;
		case LED_ERR_Pin: LED_ERR_GPIO_Port->ODR |= LED_ERR_Pin; break;
		default: break;
	}}
//@formatter:on

uint32_t compare_uint32(const void *a, const void *b) {
	uint32_t val_a = *(const uint32_t*) a;
	uint32_t val_b = *(const uint32_t*) b;
	return (val_a > val_b) ? (val_a - val_b) : (val_b - val_a);
}

void Error(uint8_t errCode) {
	while (1) {
		for (uint8_t i = 0; i < errCode; i++) {
			FlashLED(LED_ERR_Pin);
			HAL_Delay(200);
			FlashLED(0);
			HAL_Delay(200);
		}
		HAL_Delay(500);
	}
}

void BN220_TryGet(GNGGA_Parser *gps_parser, TelemetryRaw *imuData) {
	// Process the GPS data
	GNGGA_Loop(gps_parser);

	if (gps_parser->data.finish) {
		// Convert latitude from degrees-minutes to decimal degrees
		float lat_degrees = floor(fabs(gps_parser->data.latitude) / 100);
		float lat_minutes = fabs(gps_parser->data.latitude) - (lat_degrees * 100);
		imuData->lat = lat_degrees + (lat_minutes / 60.0f);
		if (gps_parser->data.latitude < 0)
			imuData->lat *= -1;

		// Convert longitude from degrees-minutes to decimal degrees
		float lon_degrees = floor(fabs(gps_parser->data.longitude) / 100);
		float lon_minutes = fabs(gps_parser->data.longitude) - (lon_degrees * 100);
		imuData->lon = lon_degrees + (lon_minutes / 60.0f);
		if (gps_parser->data.longitude < 0)
			imuData->lon *= -1;
	}
}

uint8_t BN220_Init() {
	huart3.Init.BaudRate = 115200;
	HAL_UART_Init(&huart3); // Try to set 115200
	return HAL_OK;
}
