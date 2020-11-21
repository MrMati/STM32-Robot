/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
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
#include "usbd_cdc_if.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define HAL_Delay() NULL// it's FreeRTOS so don't use blocking delays
#define RX_BUF_LEN 100
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
osThreadAttr_t defaultTask_attributes = { };

osThreadId_t ESPTaskHandle;
osThreadAttr_t ESPTask_attributes = { };

/* USER CODE BEGIN PV */
char DataToSend[60]; // Tablica zawierajaca dane do wyslania
uint8_t MessageLength = 0; // Zawiera dlugosc wysylanej wiadomosci

uint8_t received[RX_BUF_LEN];
uint8_t bufp;
uint8_t datain;
uint8_t received_flag;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);
void ESPTask(void *argument);

void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef *huart) {
	MessageLength = snprintf(DataToSend, sizeof(DataToSend),
			"AbortReceiveCpltCallback\r\n");
	CDC_Transmit_FS((uint8_t*) DataToSend, MessageLength);
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) {
	MessageLength = snprintf(DataToSend, sizeof(DataToSend),
			"RxHalfCpltCallback\r\n");
	CDC_Transmit_FS((uint8_t*) DataToSend, MessageLength);
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	MessageLength = snprintf(DataToSend, sizeof(DataToSend),
			"ErrorCallback: %lu;state: %u\r\n", HAL_UART_GetError(&huart2),
			HAL_UART_GetState(&huart2));
	CDC_Transmit_FS((uint8_t*) DataToSend, MessageLength);
	__HAL_UART_CLEAR_OREFLAG(huart);
	__HAL_UART_CLEAR_NEFLAG(huart);
	__HAL_UART_CLEAR_FEFLAG(huart);
	/* Disable the UART Error Interrupt: (Frame error, noise error, overrun error) */
	__HAL_UART_DISABLE_IT(huart, UART_IT_ERR);
	HAL_UART_Receive_DMA(&huart2, &datain, 1);

}
void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *huart) {
	MessageLength = snprintf(DataToSend, sizeof(DataToSend),
			"AbortCpltCallback\r\n");
	CDC_Transmit_FS((uint8_t*) DataToSend, MessageLength);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	if (bufp > RX_BUF_LEN - 1) {
		bufp = 0;
		received[0] = '\0';
	}
	received[bufp] = datain;
	bufp++;
	if (datain == '\n') {
		received_flag = 1;

	}
}
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
	MX_USART2_UART_Init();
	MX_USB_DEVICE_Init();
	MX_TIM3_Init();
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

	/* USER CODE BEGIN 2 */
	HAL_UART_Receive_DMA(&huart2, &datain, 1);
	BSP_LED_Init(LED4);
	BSP_LED_Init(LED5);
	BSP_LED_Off(LED4);
	BSP_LED_Off(LED5);
	/*if (BSP_ACCELERO_Init() != HAL_OK) {
	 Error_Handler();
	 }*/
	BSP_COMPASS_Init(0);
	//BSP_GYRO_Init();

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
	/* creation of defaultTask */
	defaultTask_attributes.name = "defaultTask";
	defaultTask_attributes.priority = (osPriority_t) osPriorityNormal;
	defaultTask_attributes.stack_size = 128 * 4;
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL,
			&defaultTask_attributes);

	ESPTask_attributes.name = "ESPTask";
	ESPTask_attributes.priority = (osPriority_t) osPriorityLow;
	ESPTask_attributes.stack_size = 128 * 4;
	ESPTaskHandle = osThreadNew(ESPTask, NULL, &ESPTask_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		//Vector acc = read_accel();
		//MessageLength = sprintf(DataToSend, "%f,%f,%f\n\r", acc.x, acc.y,
		//acc.z);
		//int16_t temp = L3GD20_ReadTemperature();
		//int16_t temp_a = LSM303DLHC_TempRead();
		//MessageLength = sprintf(DataToSend, "Gyro: %i, accell: %i\r\n", 40 - temp, temp_a + 20);
		//float out = read_gyro();
		//MessageLength = sprintf(DataToSend, "%i,%i,%i\n\r", (int)out.x, (int)out.y,
		//(int)out.z);
		//MessageLength = sprintf(DataToSend, "Heading: %i\n\r",
		//(int) read_heading());
		//CDC_Transmit_FS(DataToSend, MessageLength);
		//osDelay(200);
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
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 7;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 8;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
		Error_Handler();
	}
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
	htim3.Init.Prescaler = 20;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 99;
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
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 38400;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_2;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
	/* DMA1_Stream6_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE, A1_Pin | A2_Pin | B1_Pin | B2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : A1_Pin A2_Pin B1_Pin B2_Pin */
	GPIO_InitStruct.Pin = A1_Pin | A2_Pin | B1_Pin | B2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument) {
	/* USER CODE BEGIN 5 */
	//int direc = 180;
	//int defSpeed = 80;
	/* Infinite loop */
	/*for (;;) {
	 int val = diff(read_heading(), direc);
	 if (val > 0) {
	 setMotorsSpeed(defSpeed - 30, defSpeed);
	 MessageLength = sprintf(DataToSend, "Val: %i A: %i B: %i\n\r", val,
	 defSpeed - 20, defSpeed);
	 } else {
	 setMotorsSpeed(defSpeed, defSpeed - 30);
	 MessageLength = sprintf(DataToSend, "Val: %i A: %i B: %i\n\r", val,
	 defSpeed, defSpeed + 20);
	 }

	 CDC_Transmit_FS(DataToSend, MessageLength);

	 }*/
	uint8_t sensor[] = "[RobotOUT/acc][0.01;0.98;0.09]\r\n";
	while (1) {
		BSP_LED_On(LED5);
		vTaskDelay(50);
		BSP_LED_Off(LED5);
		vTaskDelay(50);
		HAL_UART_Transmit_DMA(&huart2, sensor, 32);
	}

	int angle = 180; // user input
	int target = angle;
	uint8_t speed = 80;
	target = target % 360;
	if (target < 0) {
		target += 360;
	}
	int direction = angle;
	while (1) {

		int currentAngle = read_heading();
		int diff = target - currentAngle;
		if (abs(diff) < 5) {
			setMotors(0, 0);
			return;
		}
		direction = 180 - (diff + 360) % 360;
		MessageLength = snprintf(DataToSend, sizeof(DataToSend),
				"Dir: %i Diff: %i Abs: %i\n\r", direction, diff, currentAngle);
		CDC_Transmit_FS((uint8_t*) DataToSend, MessageLength);
		if (diff > 0) {
			setMotors(speed, -speed); //right
			MessageLength = snprintf(DataToSend, sizeof(DataToSend),
					"RIGHT\n\r");
			osDelay(10);
		} else {
			setMotors(-speed, speed); //left
			MessageLength = snprintf(DataToSend, sizeof(DataToSend),
					"LEFT\n\r");
			osDelay(10);
		}
		CDC_Transmit_FS((uint8_t*) DataToSend, MessageLength);
		//if(diff<-180)
		//	diff += 360;
		//else if(diff> 180)
		//	diff -= 360;
		//direction=-diff;

	}
	osDelay(2000);
	/* USER CODE END 5 */
}

void ESPTask(void *argument) {

	for (;;) {

	if (received_flag) {
		if (prefix((char*) received, "[RobotIN/command][TOGGLE]")) {
			BSP_LED_Toggle(LED4);

			//remove_char_from_string((char*) received, '\r');
			//remove_char_from_string((char*) received, '\n');
			MessageLength = snprintf(DataToSend, sizeof(DataToSend), "%s\r\n",
					received);

			//HAL_UART_Transmit_DMA(&huart2, (uint8_t*) DataToSend,
			//MessageLength);
			CDC_Transmit_FS((uint8_t*) DataToSend, MessageLength);

		}
		received_flag = 0;
		bufp = 0;
		memset(received, 0, sizeof(received));

	}
	vTaskDelay(3);

	}

}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM1) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
