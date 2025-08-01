/**
 * @file FSM.c
 * @brief Implementation of the Finite State Machine for the flight controller
 */

#include "FSM.h"
#include "user.h"
#include "GNGGA_Parser.h"
#include <math.h>
#include "motor_control.h"
#include "pca9685.h"
/** @brief System states */
enum states {
	INIT,       ///< Initialization state
	LORA_WAIT,  ///< Waiting for LoRa command
	MAIN,       ///< Main flight state
	LANDING,    ///< Landing state
	DUMP        ///< Memory dump state
};

static enum states lastState = LANDING;
static enum states currentState = INIT;
TelemetryRaw imuData;
TelemetryPacket txPack;
uint8_t rxbuf[32];
uint8_t rxlen;
gyrobias gbias;

uint8_t flag_plus = 0;
uint8_t flag_minus = 0;

int32_t encoder_count[6];

static CircularBuffer cbPress = { .item_size = sizeof(imuData.press), .size = PRESS_BUFFER_LEN };

GNGGA_Parser gps_parser;

//@formatter:off
/** @brief LoRa config*/
static LoRa_Config_t loraCfg = {
		.frequency = 433,
		.bandwidth = 0x08,
		.spreadingFactor = 7,
		.codingRate = 0b001,
		.headerMode = 0,
		.crcEnabled = 1,
		.lowDataRateOptimize = 0,
		.preambleLength = 6,
		.payloadLength = sizeof (TelemetryPacket),
		.txAddr = 255,
		.rxAddr = 0,
		.txPower = 0x01
};


//@formatter:on

/** @brief LoRa struct */
static LoRa_Handle_t lora = { .spi = &hspi1, .nssPort = LORA_NSS_GPIO_Port, .nssPin = LORA_NSS_Pin, };

/** @brief W25Q128 struct */
static W25Qx_Device wq = { .spi = &hspi1, .cs_port = WQ_NSS_GPIO_Port, .cs_pin = WQ_NSS_Pin, .capacity = 16777216 };

/**
 * @brief Initialize all system components
 */
static void init_state(void) {
	static uint8_t errorCode = 0;
	if (currentState != lastState) {
		lastState = currentState;
		if (!MS5611_Init(&hspi1, MS_NSS_GPIO_Port, MS_NSS_Pin))		//[V]
			errorCode = 1;
		if (!LIS3_Init(&hspi1, LIS_NSS_GPIO_Port, LIS_NSS_Pin))		//TODO: Calibrate me!!!
			errorCode = 2;
		if (!LSM6_Init(&hi2c1, NULL, (0b1101010 << 1)))				//[V]
			errorCode = 3;
		if (!LoRa_Init(&lora))										//[V]
			errorCode = 4;
//		if (!microSD_Init())										//[V]
//			errorCode = 5;
 		if (!W25Qx_Init(&wq))										//[V]
			errorCode = 6;
		if (BN220_Init() != HAL_OK)									//FIXME: Set up bn880 module for GNGGA
			errorCode = 7;
		if (PCA9685_Init(&hi2c1)){
			errorCode = 8;
		}

		if (!errorCode) {
			MS5611_SetOS(MS56_OSR_4096, MS56_OSR_4096);

			LIS3_Config(LIS_CTRL1, LIS_MODE_HP | LIS_ODR_80);
			LIS3_Config(LIS_CTRL2, LIS_SCALE_4);
			LIS3_Config(LIS_CTRL3, LIS_CYCLIC);

			LoRa_EnableDIO0Interrupt(&lora, 0); //Enable RX_Done Interrupt

			LSM6_ConfigAG(LSM6_ACCEL_16G | LSM6_CFG_12_5_Hz, LSM6_GYRO_2000DPS | LSM6_CFG_12_5_Hz);

			CB_Init(&cbPress);
			GNGGA_Init(&gps_parser, &huart3);
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
static void lora_wait_state(void) { // TODO: Unify with rxCmd structure
	static uint8_t pingFlag = 0;

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

	if (rxlen > 0) {
		rxlen=0;
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
				case '4':
					LoRa_Transmit(&lora, "Gyro Calib\n", 11);
					gyroCalibration(&gbias,500);
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
		HAL_TIM_Base_Start_IT(&htim1);
	}

//	BN220_TryGet(&gps_parser, &imuData);

	if (HAL_GetTick() - imuData.time >= DATA_PERIOD) {
//		HAL_ADC_Start(&hadc1);
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

		//TODO: Your job each DATA_PERIOD ms

		/* Save & transmit all */
		ImuSaveAll(&imuData, &txPack, &lora, &wq);
	}

	//TODO: Your job while in main_state

}

/**
 * @brief Handle landing state
 */
static void landing_state(void) {
	if (currentState != lastState) {
		lastState = currentState;
		imuData.time = HAL_GetTick();
	}

	BN220_TryGet(&gps_parser, &imuData);

	if (HAL_GetTick() - imuData.time >= DATA_PERIOD_LND) {
		ImuGetAll(&imuData);
		ImuSaveAll(&imuData, &txPack, &lora, &wq);
	}
}

/**
 * @brief Handle memory dump state
 */
static void dump_state(void) {
	if (currentState != lastState) {
		lastState = currentState;
	}

	uint8_t buf[sizeof(TelemetryPacket)];
	for (uint32_t addr = 0; addr < 0xFFFFFF; addr += sizeof(TelemetryPacket)) {
		FlashLED(LED2_Pin);
		W25Qx_ReadData(&wq, addr, buf, sizeof(TelemetryPacket));
		if (buf[0] == 0xFF && buf[1] == 0xFF && buf[2] == 0xFF && buf[3] == 0xFF)
			break;
		LoRa_Transmit(&lora, buf, sizeof(TelemetryPacket));
		FlashLED(LED1_Pin);
		if (microSD_Write(buf, sizeof(TelemetryPacket), SD_FILENAME_WQ) != FR_OK) {
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

/**
 * @brief override weak UART callback
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart3) {
		GNGGA_UART_IRQHandler(&gps_parser);
	}
}

//float ENC_GetAngle(void) {
//	__disable_irq();
//	int32_t cnt = encoder_count;
//	__enable_irq();
//	return cnt * DEGREES_PER_PULSE;
//}

/**
 * @brief override weak EXTI callback
 *
 * @note handle PC5 (LoRa RXDone) and encoder M1_C1 / M1_C2
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	switch (GPIO_Pin) {
	case GPIO_PIN_5: // DIO0 LoRa (PC5)
		if(LoRa_Receive(&lora, rxbuf, &rxlen)) {
			if(rxlen == sizeof(ControlCommand))
				MOT_ParseCmd((ControlCommand*)(&rxbuf)); // Calculated On-Air ~15ms
		}
		break;

	case M1_C1_Pin: //TODO: Control Logic
		if(HAL_GPIO_ReadPin(M1_C2_GPIO_Port, M1_C2_Pin))
			encoder_count[0]++; else encoder_count[0]--;
//		if(encoder_count[0] > 1440){
//					MOT_SetSpeed(1, -20);
//				}
//
//				if(encoder_count[0] < 1440){
//					MOT_SetSpeed(1, 20);
//				}
//
//				if(encoder_count[0] == 1440){
//					MOT_SetSpeed(1, 0);
//				}
//		PID_Update(0, encoder_count[0]);

		break;
	case M2_C1_Pin:
			if(HAL_GPIO_ReadPin(M2_C2_GPIO_Port, M2_C2_Pin))
				encoder_count[1]--; else encoder_count[1]++;

			break;
	case M3_C1_Pin:
			if(HAL_GPIO_ReadPin(M3_C2_GPIO_Port, M3_C2_Pin))
				encoder_count[2]++; else encoder_count[2]--;

			break;
	case M4_C1_Pin:
		if(HAL_GPIO_ReadPin(M4_C2_GPIO_Port, M4_C2_Pin))
			encoder_count[3]++; else encoder_count[3]--;

		break;
	case M5_C1_Pin:
			if(HAL_GPIO_ReadPin(M5_C2_GPIO_Port, M5_C2_Pin))
				encoder_count[4]++; else encoder_count[4]--;
			break;
	case M6_C1_Pin:
				if(HAL_GPIO_ReadPin(M6_C2_GPIO_Port, M6_C2_Pin))
					encoder_count[5]--; else encoder_count[5]++;
				break;
	default:
			break;
	}
}

/**
 * @brief override weak PeriodElapsed callback
 *
 * @note handle TIM1 interrupt
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM1)
    {
//    	ImuGet(&imuData);
//    	MadgwickAHRSupdate(imuData.gyroData[2]*0.001,imuData.gyroData[0]*0.001, imuData.gyroData[1]*0.001, imuData.accelData[2]*0.00982, imuData.accelData[0]*0.00982, imuData.accelData[1]*0.00982, -1*imuData.magData[2]*0.001, imuData.magData[1]*0.001, imuData.magData[0]*0.001);
//    	computeAngles();
    	PID_Speed_Update(imuData.gyroData[2], imuData.gyroData[0], imuData.gyroData[1]);
    	for (int i = 0; i<6; i++){
        	PID_Update(i, encoder_count[i]);
        }
    	// TODO: Handle TIM1 IT
    }
}

