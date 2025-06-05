/**
 * @file FSM.c
 * @brief Implementation of the Finite State Machine for the flight controller
 */

#include "FSM.h"
#include "main.h"
#include "user.h"

#include "MS5611.h"
#include "LIS3.h"
#include "LSM6.h"
#include "LoRa.h"
#include "MicroSD.h"
#include "W25Qx.h"
#include "CircularBuffer.h"

#include <math.h>
/** @brief IMU data structure */
struct ImuData {
	uint32_t time;          ///< Milliseconds from start
	int32_t temp;           ///< MS56 temperature (centigrade*10e2)
	uint32_t press;         ///< MS56 pressure (Pa)
	float magData[3];       ///< LIS3 mag (mG)
	float accelData[3];     ///< LSM6 accel (mG)
	float gyroData[3];      ///< LSM6 gyro (mdps)
	int32_t altitude;       ///< Altitude (zero at start, cm)
	uint32_t flags;         ///< Flags (0|0|0|0|0|Land|Eject|Start)
	uint32_t press0;        ///< MS56 pressure at 0 Alt (Pa)
	float vectAbs;          ///< Absolute value of accel vector
	uint32_t wqAdr;         ///< WQ address
} imuData;

/** @brief System states */
enum states {
	INIT,       ///< Initialization state
	LORA_WAIT,  ///< Waiting for LoRa command
	MAIN,       ///< Main flight state
	LANDING,    ///< Landing state
	DUMP        ///< Memory dump state
};

extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;
extern SD_HandleTypeDef hsd;
extern SPI_HandleTypeDef hspi1;

static enum states lastState = LANDING;
static enum states currentState = INIT;

static CircularBuffer cbPress = { .item_size = sizeof(imuData.press), .size = PRESS_BUFFER_LEN };

//@formatter:off
/** @brief LoRa config*/
static LoRaConfig loraCfg = {
		.frequency = 433,
		.bandwidth = 0x08,
		.spreadingFactor = 7,
		.codingRate = 0b001,
		.headerMode = 0,
		.crcEnabled = 1,
		.lowDataRateOptimize = 0,
		.preambleLength = 6,
		.payloadLength = 56,
		.txAddr = 255,
		.rxAddr = 0,
		.txPower = 0x01
};
//@formatter:on
/** @brief LoRa struct */
static LoRa_HandleTypeDef lora = { .spi = &hspi1, .NSS_Port = LORA_NSS_GPIO_Port, .NSS_Pin = LORA_NSS_Pin, };

/** @brief W25Q128 struct */
static W25Qx_Device wq = { .spi = &hspi1, .cs_port = WQ_NSS_GPIO_Port, .cs_pin = WQ_NSS_Pin, .capacity = 16777216 };

/** @brief Forward declarations */
void Error(uint8_t errCode);
void FlashLED(uint16_t led);
void StoreVectAbs(struct ImuData *dat);
void ImuSaveAll(struct ImuData *imuData);
void ImuGetAll(struct ImuData *imuData);

/**
 * @brief Compare two uint32_t values and return their absolute difference.
 *
 * @param a Pointer to the first value.
 * @param b Pointer to the second value.
 * @return uint32_t Absolute difference between the two values.
 */
uint32_t compare_uint32(const void *a, const void *b) {
    uint32_t val_a = *(const uint32_t*)a;
    uint32_t val_b = *(const uint32_t*)b;
    return (val_a > val_b) ? (val_a - val_b) : (val_b - val_a);
}

/**
 * @brief Initialize all system components
 */
static void init_state(void) {
	static uint8_t errorCode = 0;
	if (currentState != lastState) {
		lastState = currentState;
		if (!MS5611_Init(&hspi1, MS_NSS_GPIO_Port, MS_NSS_Pin))
			errorCode = 1;
		if (!LIS3_Init(&hspi1, LIS_NSS_GPIO_Port, LIS_NSS_Pin))
			errorCode = 2;
		if (!LSM6_Init(&hi2c1, NULL, (0b1101010 << 1)))
			errorCode = 3;
		if (!LoRa_Init(&lora))
			errorCode = 4;
		if (!microSD_Init())
			errorCode = errorCode; // CODE - 5
		if (!W25Qx_Init(&wq))
			errorCode = 6;

		if (!errorCode) {
			MS5611_SetOS(MS56_OSR_4096, MS56_OSR_4096);
			LIS3_Config(LIS_CTRL1, LIS_MODE_HP | LIS_ODR_80);
			LIS3_Config(LIS_CTRL2, LIS_SCALE_4);
			LIS3_Config(LIS_CTRL3, LIS_CYCLIC);
			LSM6_ConfigAG(LSM6_ACCEL_16G | LSM6_CFG_12_5_Hz, LSM6_GYRO_2000DPS | LSM6_CFG_12_5_Hz);
			CB_Init(&cbPress);
			imuData.wqAdr = 0;
			currentState = LORA_WAIT;
		} else {
			Error(errorCode);
		}
	}
}

/**
 * @brief Handle LoRa waiting state
 */
static void lora_wait_state(void) {
	static uint8_t rxbuf[1];
	static uint8_t pingFlag = 0;
	uint8_t rxLen = 0;

	if (currentState != lastState) {
		lastState = currentState;
	}

	if (!pingFlag) {
		if (HAL_GetTick() % 300 > 150)
			FlashLED(LED1_Pin);
		else
			FlashLED(LED2_Pin);
	} else if (HAL_GetTick() % 500 > 250)
		FlashLED(LED1_Pin);
	else
		FlashLED(0);

	if (LoRa_Receive(&lora, rxbuf, &rxLen)) {
		if (rxbuf[0] == '0') {
			pingFlag = 1;
			LoRa_Transmit(&lora, "Ping OK\n", 8);
		}
		if (pingFlag) {
			switch (rxbuf[0]) {
				case '1':
					LoRa_Transmit(&lora, "Starting\n", 9);
					currentState = MAIN;
					break;
				case '2':
					currentState = DUMP;
					LoRa_Transmit(&lora, "Memory dump\n", 12);
					break;
				case '3':
					LoRa_Transmit(&lora, "Erase All\n", 10);
					W25Qx_EraseChip(&wq);
					microSD_RemoveFile(SD_FILENAME);
					microSD_RemoveFile(SD_FILENAME_WQ);
					LoRa_Transmit(&lora, "Done\n", 5);
					break;
				default:
					break;
			}
		}
	}
}

/**
 * @brief Handle main flight state
 */
static void main_state(void) {
	if (currentState != lastState) {
		lastState = currentState;
		for (uint8_t i = 0; i < 10; i++) {
			MS5611_Read(&imuData.temp, &imuData.press0);
			HAL_Delay(50);
		}
		MS5611_Read(&imuData.temp, &imuData.press0);
		StoreVectAbs(&imuData);
		imuData.time = HAL_GetTick();
	}

	if (HAL_GetTick() - imuData.time >= DATA_PERIOD) {
		HAL_ADC_Start(&hadc1);
		ImuGetAll(&imuData);

		if (imuData.altitude > START_TH)
			bitSet(imuData.flags, 0);
		if (bitRead(imuData.flags, 0) && ADC1->DR < EJECT_TH)
			bitSet(imuData.flags, 1);

		if (bitRead(imuData.flags, 1)) {
			CB_Push(&cbPress, &imuData.press);
			if (CB_Diff(&cbPress, compare_uint32) < PRESS_LAND_DELTA) {
				bitSet(imuData.flags, 2);
				currentState = LANDING;
			}
		}
		ImuSaveAll(&imuData);
	}
}

/**
 * @brief Handle landing state
 */
static void landing_state(void) {
	if (currentState != lastState) {
		lastState = currentState;
		imuData.time = HAL_GetTick();
	}
	if (HAL_GetTick() - imuData.time >= DATA_PERIOD_LND) {
		ImuGetAll(&imuData);
		ImuSaveAll(&imuData);
	}
}

/**
 * @brief Handle memory dump state
 */
static void dump_state(void) {
	if (currentState != lastState) {
		lastState = currentState;
	}

	uint8_t buf[FRAME_SIZE];
	for (uint32_t addr = 0; addr < 0xFFFFFF; addr += FRAME_SIZE) {
		FlashLED(LED2_Pin);
		W25Qx_ReadData(&wq, addr, buf, FRAME_SIZE);
		if (buf[0] == 0xFF && buf[1] == 0xFF && buf[2] == 0xFF && buf[3] == 0xFF)
			break;
		LoRa_Transmit(&lora, buf, FRAME_SIZE);
		FlashLED(LED1_Pin);
		if (microSD_Write(buf, FRAME_SIZE, SD_FILENAME_WQ) != FR_OK) {
			FlashLED(LED_ERR_Pin);
			HAL_Delay(1000);
		}
	}
	LoRa_Transmit(&lora, "Done\n", 5);
	currentState = LORA_WAIT;
}

void FSM_Init(void) {
	lora.config = loraCfg;
	lastState = LANDING;
	currentState = INIT;
}

void FSM_Update(void) {
	switch (currentState) {
		case INIT:
			init_state();
			break;
		case LORA_WAIT:
			lora_wait_state();
			break;
		case MAIN:
			main_state();
			break;
		case LANDING:
			landing_state();
			break;
		case DUMP:
			dump_state();
		default:
			break;
	}
}

void StoreVectAbs(struct ImuData *dat) {
	dat->vectAbs = 0;
	for (uint8_t i = 0; i < 3; i++)
		dat->vectAbs += dat->accelData[i] * dat->accelData[i];
	dat->vectAbs = sqrt(dat->vectAbs);
}

void ImuGetAll(struct ImuData *imuData) {
	imuData->time = HAL_GetTick();
	MS5611_Read(&imuData->temp, &imuData->press);
	LIS3_Read(imuData->magData);
	LSM6_Read(imuData->accelData, imuData->gyroData);
	StoreVectAbs(imuData);
	imuData->altitude = MS5611_GetAltitude(&imuData->press, &imuData->press0);
}

void ImuSaveAll(struct ImuData *imuData) {
	FlashLED(LED2_Pin);
	LoRa_Transmit(&lora, imuData, FRAME_SIZE);
	W25Qx_WriteData(&wq, imuData->wqAdr, imuData, FRAME_SIZE);
	imuData->wqAdr += FRAME_SIZE;
	FlashLED(LED1_Pin);
	if (microSD_Write(imuData, FRAME_SIZE, SD_FILENAME) != FR_OK) {
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
