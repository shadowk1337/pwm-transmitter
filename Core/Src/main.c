/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define NEXT_LINE "\r\n"

#define ACK 	0x00000004U
#define SYN 	0x00000026U

#define FALSE	0x00000000U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define ARRAY_LEN(x) 			(sizeof(x) / sizeof((x)[0]))
#define IS_NULL(x) 				((x) == NULL)

#define READ_BIT(REG, BIT)    	((REG) & (BIT))

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

int pulse = 0, dir = 100;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

void uart_rx_check(void);
void uart_process_data(const void*, size_t);
void uart_send_string(const char*);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int __io_putchar(int ch) {
	HAL_UART_Transmit(&huart1, (uint8_t*) &ch, 1, 0xFFFF);
	return 0;
}

/** @defgroup reading console through DMA
 * @{
 */
/**
 * @brief ring buffer structure definition with default value
 */
typedef struct {
	uint8_t buff[1024];
	size_t pos;
} RingBuffer;

RingBuffer ring_buf = { .pos = 0 };

uint8_t* get_ring_begin(RingBuffer *instance) {
	if (IS_NULL(instance->buff) || !ARRAY_LEN(instance->buff)) {
		return NULL;
	}
	return &instance->buff[0];
}

uint8_t* get_ring_end(RingBuffer *instance) {
	if (IS_NULL(instance->buff) || !ARRAY_LEN(instance->buff)) {
		return NULL;
	}
	size_t len = ARRAY_LEN(instance->buff);
	return &instance->buff[len - 1];
}

uint8_t* get_ring_start(RingBuffer *instance) {
	if (IS_NULL(instance->buff) || !ARRAY_LEN(instance->buff)) {
		return NULL;
	}
	return &instance->buff[instance->pos];
}

/**
 * @}
 */

/** @defgroup reading console through DMA
 * @{
 */
void uart_rx_check(void) {
	size_t pos = ARRAY_LEN(ring_buf.buff)
			- LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_3);
	if (pos == ring_buf.pos) {
		return;
	}
	if (pos > ring_buf.pos) {
		uart_process_data(get_ring_start(&ring_buf), pos - ring_buf.pos);
	} else {
		uart_process_data(get_ring_start(&ring_buf),
		ARRAY_LEN(ring_buf.buff) - ring_buf.pos);
		if (pos > 0) {
			uart_process_data(&ring_buf.buff[0], pos);
		}
	}
	ring_buf.pos = pos;
}

void uart_process_data(const void *data, size_t len) {
	const uint8_t *d = data;
	while (len > 0) {
		LL_USART_TransmitData8(USART1, *d);
		while (!LL_USART_IsActiveFlag_TXE(USART1)) {
		}
		len--, d++;
	}
	HAL_UART_Transmit(&huart1, NEXT_LINE, 2, -1);

	while (!LL_USART_IsActiveFlag_TC(USART1)) {
	}
}

void uart_send_string(const char *str) {
	uart_process_data(str, strlen(str));
}

/**
 * @}
 */

//uint8_t rx_buf[32];
//uint8_t main_buf[64];
//
//uint16_t old_pos = 0, new_pos = 0;
//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
//	if (huart->Instance == USART1) {
//		old_pos = new_pos;
//
//		if (old_pos + Size > ARRAY_LEN(main_buf)) {
//			uint16_t data_to_copy = ARRAY_LEN(main_buf) - old_pos;
//			memcpy((uint8_t*) main_buf + old_pos, rx_buf, data_to_copy);
//
//			old_pos = 0;
//			memcpy((uint8_t*) main_buf, (uint8_t*) rx_buf + data_to_copy,
//					(Size - data_to_copy));
//			new_pos = (Size - data_to_copy);
//		} else {
//			memcpy((uint8_t*) main_buf + old_pos, rx_buf, Size);
//			new_pos = Size + old_pos;
//		}
//		uart_process_data(main_buf, strlen(main_buf));
//
//		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t*) rx_buf,
//				ARRAY_LEN(rx_buf));
//		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
//	}
//}
uint32_t get_rising_edge(TIM_HandleTypeDef *htim, uint32_t Channel) {
	if (htim != &htim3) {
		return 0;
	}
	if (htim->State != (HAL_TIM_StateTypeDef) HAL_TIM_CHANNEL_STATE_READY) {
		return 0;
	}
	if ((htim->Instance->CCER >> (Channel + 1)) & 1U) {
		return 0;
	}
//	uint32_t polarity = LL_TIM_IC_GetPolarity(TIMx, Channel)
	return HAL_TIM_ReadCapturedValue(htim, Channel);
}

uint32_t get_falling_edge(TIM_HandleTypeDef *htim, uint32_t Channel) {
	if (htim != &htim3) {
		return 0;
	}
	if (htim->State != (HAL_TIM_StateTypeDef) HAL_TIM_CHANNEL_STATE_READY) {
		return 0;
	}
	if (READ_BIT(htim->Instance->CCER >> (Channel + 1), 1) != 1) {
		return 0;
	}
	return HAL_TIM_ReadCapturedValue(htim, Channel);
}

void toggle_bits(uint32_t ascii_char) {
}

/**
 * @brief Function for converting message from dec to bin
 * @arg msg - message given by user from host
 */
void parse_message(const char *msg) {

}

//void DMA1_Channel3_IRQHandler(void) {
//	printf("DMA sent IRQ\r\n");
//	/* Check half-transfer complete interrupt */
//	if (LL_DMA_IsEnabledIT_HT(DMA1, LL_DMA_CHANNEL_3)
//			&& LL_DMA_IsActiveFlag_HT5(DMA1)) {
//		LL_DMA_ClearFlag_HT5(DMA1); /* Clear half-transfer complete flag */
//		uart_rx_check(); /* Check for data to process */
//	}
//
//	/* Check transfer-complete interrupt */
//	if (LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_CHANNEL_3)
//			&& LL_DMA_IsActiveFlag_TC5(DMA1)) {
//		LL_DMA_ClearFlag_TC5(DMA1); /* Clear transfer complete flag */
//		uart_rx_check(); /* Check for data to process */
//	}
//
//	/* Implement other events when needed */
//}

//void HAL_USART_IRQHandler(void) {
//	printf("Hello");
//}

//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
//	printf("%d\r\n", __HAL_TIM_GET_COUNTER(htim));
//}

//void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
//	printf("%d\r\n", __HAL_TIM_GET_COUNTER(htim));
//	if (htim == &htim2) {
//		pulse += dir;
//		if (pulse == 1000) {
//			dir = -100;
//		} else if (pulse == 0) {
//			dir = 100;
//		}
//		TIM2->CCR2 = pulse;
//	}
//}

/*
 void process_buff_half(uint8_t *buff, size_t begin_idx) {
 if (begin_idx < 0 || begin_idx >= 8) {
 return;
 }
 uint8_t transmitted_buff[4];
 memcpy(transmitted_buff, &buff[begin_idx], 4 * sizeof(uint8_t));
 HAL_UART_Transmit(&huart1, transmitted_buff, 4, -1);
 }
 */

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) {
// передана половина данных
//	uart_rx_check();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
// завершена передача всех данных
//	HAL_UART_Receive_DMA(&huart1, buff, 8);
//	uart_rx_check();
}

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//	if (htim == &htim16) {
//		printf("from IT\r\n");
//	}
//}
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
	MX_USART1_UART_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */
	TIM2->CCR2 = 200;
	TIM2->ARR = 1000;
//	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_2);
//	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
//	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);
//	HAL_TIM_Base_Start_IT(&htim16);
//	HAL_UART_Receive_DMA(&huart1, uart_rx_dma_buffer,
//			ARRAY_LEN(uart_rx_dma_buffer));

	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t*) ring_buf.buff,
			ARRAY_LEN(ring_buf.buff));

//	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
//		HAL_UART_Transmit(&huart1, buff, 8, -1);
//		uint32_t rising_edge = get_rising_edge(&htim3, TIM_CHANNEL_2);
//		uint32_t falling_edge = get_falling_edge(&htim3, TIM_CHANNEL_3);

//		read_input(&huart1, buf, (uint16_t)strlen(buf));

//		HAL_UART_Receive(&huart1, string, strlen(string), -1);
//		printf("%s", string);
//		printf("rising = %d, falling = %d\r\n", rising_edge, falling_edge);
//		int count = __HAL_TIM_GET_COUNTER(&htim16);

//		printf("ms = %d\r\n", count);
//		HAL_UART_Transmit(&huart1, buf, strlen(buf), -1);

//		char ch = 0;
//		HAL_UART_Receive(&huart1, &ch, 1, -1);
//		HAL_UART_Transmit(&huart1, &ch, 1, -1);

//		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
//		HAL_Delay(100);
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
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 48000 - 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 0;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_SlaveConfigTypeDef sSlaveConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_IC_InitTypeDef sConfigIC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 48000 - 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_IC_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
	sSlaveConfig.InputTrigger = TIM_TS_TI2FP2;
	sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sSlaveConfig.TriggerFilter = 0;
	if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel2_3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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

