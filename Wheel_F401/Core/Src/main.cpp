/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FfbReportHandler.h"
#include "FfbWheel.h"
#include "HIDReportType.h"
#include <chrono>
#include "usbd_customhid.h"
#include "helpers.h"
#include "globals.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

/* Definitions for mainTask */
osThreadId_t mainTaskHandle;
const osThreadAttr_t mainTask_attributes = { .name = "mainTask", .stack_size = 128 * 4, .priority = (osPriority_t) osPriorityNormal, };
/* Definitions for task02 */
osThreadId_t task02Handle;
const osThreadAttr_t task02_attributes = { .name = "task02", .stack_size = 128 * 4, .priority = (osPriority_t) osPriorityNormal, };
/* Definitions for task03 */
osThreadId_t task03Handle;
const osThreadAttr_t task03_attributes = { .name = "task03", .stack_size = 128 * 4, .priority = (osPriority_t) osPriorityNormal, };
/* Definitions for task04 */
osThreadId_t task04Handle;
const osThreadAttr_t task04_attributes = { .name = "task04", .stack_size = 128 * 4, .priority = (osPriority_t) osPriorityNormal, };
/* Definitions for task05 */
osThreadId_t task05Handle;
const osThreadAttr_t task05_attributes = { .name = "task05", .stack_size = 128 * 4, .priority = (osPriority_t) osPriorityNormal, };
/* USER CODE BEGIN PV */

reportHID_t reportHID = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

volatile uint16_t adcResultsDMA[3];
const int adcChannelCount = sizeof(adcResultsDMA) / sizeof(adcResultsDMA[0]);
volatile int adcConversionComplete = 0; // set by callback

uint8_t fx_ratio_i = 80; // Reduce effects to a certain ratio of the total power to have a margin for the endstop
int32_t torque = 0; // last torque
int32_t effectTorque = 0; // last torque
int32_t lastEnc = 0;
uint8_t endstop_gain_i = 4; // Sets how much extra torque per count above endstop is added. High = stiff endstop. Low = softer

void *handler;

int16_t debug;
int32_t torqueTest;
int32_t addtorqueTest;
int32_t endstopTorqueTest;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);
void StartTask05(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int16_t updateEndstop() {
	int8_t clipdir = cliptest<int32_t, int32_t>(lastEnc, -6144, 6144);
	if (clipdir == 0) {
		return 0;
	}
	int32_t addtorque = 0;

	addtorque += clip<int32_t, int32_t>(abs(lastEnc) - 6144, -6144, 6144);
	addtorque *= (float) endstop_gain_i; // Apply endstop gain for stiffness
	addtorque *= -clipdir;
	addtorqueTest = addtorque;

	return clip<int32_t, int32_t>(addtorque, -700, 700);
}

void printEffectState(TEffectState *state) {
	printf("state: %u\n", state->state);
	printf("effectType: %u\n", state->effectType);
	printf("offset: %d\n", state->offset);
	printf("gain: %u\n", state->gain);
	printf("attackLevel: %d, fadeLevel: %d\n", state->attackLevel, state->fadeLevel);
	printf("magnitude: %d\n", state->magnitude);
	printf("enableAxis: %#x\n", state->enableAxis);
	printf("directionX: %u, directionY: %u\n", state->directionX, state->directionY);
	printf("cpOffset: %d\n", state->cpOffset);
	printf("positiveCoefficient: %d\n", state->positiveCoefficient);
	printf("negativeCoefficient: %d\n", state->negativeCoefficient);
	printf("positiveSaturation: %u, negativeSaturation: %u\n", state->positiveSaturation, state->negativeSaturation);
	printf("deadBand: %u\n", state->deadBand);
	printf("last_value: %ld\n", state->last_value);
	printf("phase: %u\n", state->phase);
	printf("startMagnitude: %d, endMagnitude: %d\n", state->startMagnitude, state->endMagnitude);
	printf("axis: %u\n", state->axis);
	printf("period: %u\n", state->period);
	printf("duration: %u, fadeTime: %u, attackTime: %u, counter: %u\n", state->duration, state->fadeTime, state->attackTime, state->counter);
	printf("startTime: %llu\n", state->startTime);
	printf("--------------------------------\n");
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_TIM3_Init();
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */
	handler = createFfbReportHandler();
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	printf("Hello!\n");
	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of mainTask */
	mainTaskHandle = osThreadNew(StartDefaultTask, NULL, &mainTask_attributes);

	/* creation of task02 */
	task02Handle = osThreadNew(StartTask02, NULL, &task02_attributes);

	/* creation of task03 */
	task03Handle = osThreadNew(StartTask03, NULL, &task03_attributes);

	/* creation of task04 */
	task04Handle = osThreadNew(StartTask04, NULL, &task04_attributes);

	/* creation of task05 */
	task05Handle = osThreadNew(StartTask05, NULL, &task05_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 15;
	RCC_OscInitStruct.PLL.PLLN = 144;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 5;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 1919;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA2_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);

	/*Configure GPIO pins : PB8 PB9 */
	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	adcConversionComplete = 1;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the mainTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument) {
	/* init code for USB_DEVICE */
	MX_USB_DEVICE_Init();
	/* USER CODE BEGIN 5 */
	int32_t lastTorque;

	/* Infinite loop */
	for (;;) {
		lastTorque = torque;
		torque = 0;
		lastEnc = reportHID.X;

		effectTorque = callCalculateEffects(handler, reportHID.X, 1);

		if (abs(effectTorque) >= MAX_TORQUE) {
			//clipped
		}
		// Scale for power and endstop margin
		float effect_margin_scaler = ((float) fx_ratio_i / 255.0);
		effectTorque *= effect_margin_scaler;

		// Always check if endstop reached
//		int32_t endstopTorque = updateEndstop();
//		endstopTorqueTest = endstopTorque;

		// Calculate total torque
		torque += effectTorque;

		// Torque changed
		if (torque != lastTorque) {
			// Update torque and clip
			torque = clip<int32_t, int32_t>(torque, -MAX_TORQUE, MAX_TORQUE);
			if (abs(torque) == MAX_TORQUE) {
				//clipped
			}
			// Send to motor driver
			if (torque < 0) {
				HAL_GPIO_WritePin(R_EN_GPIO_Port, R_EN_Pin, GPIO_PIN_RESET); // Установка R_EN на низкий уровень
				HAL_GPIO_WritePin(L_EN_GPIO_Port, L_EN_Pin, GPIO_PIN_SET); // Установка L_EN на высокий уровень
			}
			else {
				HAL_GPIO_WritePin(R_EN_GPIO_Port, R_EN_Pin, GPIO_PIN_SET); // Установка R_EN на высокий уровень
				HAL_GPIO_WritePin(L_EN_GPIO_Port, L_EN_Pin, GPIO_PIN_RESET); // Установка L_EN на низкий уровень
			}
			int32_t val = (uint32_t) abs(torque) + 600;
			torqueTest = torque;
			printf("\tTorque = %ld", torque);

			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, val);
		}

//		TEffectState* effect;
//		effect = (TEffectState*) callGetEffectData(handler, 0);
//		printEffectState(effect);

		osDelay(1);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
 * @brief Function implementing the task02 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument) {
	/* USER CODE BEGIN StartTask02 */
	HAL_StatusTypeDef ret;
	uint8_t raw[2] = { 0, 0 };
	int16_t prevPos = 0;
	int16_t temp;
	int rdCnt = 0;

	static const uint8_t H_ADDR = 0x36 << 1;
	static uint8_t H_DATA = 0x0C;

	ret = HAL_I2C_Master_Transmit(&hi2c1, H_ADDR, &H_DATA, 1, HAL_MAX_DELAY);
	if (ret != HAL_OK) {
	}
	ret = HAL_I2C_Master_Receive(&hi2c1, H_ADDR, raw, 2, HAL_MAX_DELAY);
	if (ret != HAL_OK) {
	}
	prevPos = raw[1] | (raw[0] << 8);

	/* Infinite loop */
	for (;;) {
		ret = HAL_I2C_Master_Transmit(&hi2c1, H_ADDR, &H_DATA, 1, HAL_MAX_DELAY);
		if (ret != HAL_OK) {
			continue;
		}
		ret = HAL_I2C_Master_Receive(&hi2c1, H_ADDR, raw, 2, HAL_MAX_DELAY);
		if (ret != HAL_OK) {
			continue;
		}
		temp = raw[1] | (raw[0] << 8);

		if (prevPos > 2596 && prevPos <= 4096 && temp >= 0 && temp <= 1596) {
			rdCnt++;
		}
		else if (temp > 2596 && temp <= 4096 && prevPos >= 0 && prevPos <= 1596) {
			rdCnt--;
		}
		if (temp + 4096 * rdCnt < reportHID.X + 2 && temp + 4096 * rdCnt > reportHID.X - 2) {
			// Do nothing
		}
		else {
			reportHID.X = -(temp + rdCnt * 4096);
		}
		prevPos = temp;
		debug = temp;

		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*) &reportHID, sizeof(reportHID_t));
		osDelay(1);
	}
	/* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
 * @brief Function implementing the task03 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument) {
	/* USER CODE BEGIN StartTask03 */
	/* Infinite loop */
	for (;;) {
		osDelay(1);
	}
	/* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
 * @brief Function implementing the task04 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument) {
	/* USER CODE BEGIN StartTask04 */
	/* Infinite loop */
	for (;;) {

		HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adcResultsDMA, adcChannelCount);
		while (adcConversionComplete == 0) {

		}
		adcConversionComplete = 0;

//		if (adcResultsDMA[0] < reportHID.Z + 20
//				&& adcResultsDMA[0] > reportHID.Z - 20) {
//			// Do nothing
//		} else {
//			reportHID.Z = adcResultsDMA[0];
//		}
//		if (adcResultsDMA[1] < reportHID.RX + 20
//				&& adcResultsDMA[1] > reportHID.RX - 20) {
//			// Do nothing
//		} else {
//			reportHID.RX = adcResultsDMA[1];
//		}
//		if (adcResultsDMA[2] < reportHID.RY + 20
//				&& adcResultsDMA[2] > reportHID.RY - 20) {
//			// Do nothing
//		} else {
//			reportHID.RY = adcResultsDMA[2];
//		}

		osDelay(1);
	}
	/* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
 * @brief Function implementing the task05 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask05 */
void StartTask05(void *argument) {
	/* USER CODE BEGIN StartTask05 */
	/* Infinite loop */
	for (;;) {
		osDelay(1);
	}
	/* USER CODE END StartTask05 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
