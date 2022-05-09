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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
	uint8_t buff[512];
	size_t pos;
} RingBuffer;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define NEXT_LINE "\r\n"

#define ACK     0x6U
#define NAK     0x25U
#define SYN     0x26U

/** @defgroup PWM duty cycle modes in percent
 * @{
 */
#define PWM_LOW_PERCENT 25
#define PWM_HIGH_PERCENT 75
#define PWM_IDLE_PERCENT 101
/*
 * }
 */

/** @defgroup boolean operators
 * @{
 */
#define TRUE    0x00000001U
#define FALSE   0x00000000U
/*
 * }
 */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define ARRAY_LEN(x) 						(sizeof(x) / sizeof((x)[0]))
#define IS_NULL(x)							((x) == NULL)
#define GET_PERCENT_VALUE(value, percent) 	((value) * ((percent) / 100.0))
#define READ_BIT(REG, BIT)					((REG) & (BIT))

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */

void PWM_DecodeChar(char, uint8_t*);
void PWM_SendDecodedChar(char);
void PWM_ProcessString(const char*, size_t);
void PWM_SetIdle(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int __io_putchar(int ch) {
	HAL_UART_Transmit(&huart1, (uint8_t*) &ch, 1, 0xFFFF);
	return 0;
}

volatile uint16_t PWM_data_sent_flag = 0;

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
		PWM_data_sent_flag = 1;
		char cha = PWM_data_sent_flag == 1 ? '1' : '0';
		HAL_UART_Transmit(&huart1, &cha, 1, -1);
//
	}

//	if (HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_2) != HAL_OK) {
//		Error_Handler();
//	}
//	HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_2);
//	*data = 1;
//	LL_USART_TransmitData8(USART1, 's');
//	if (htim->Instance == TIM2) {
//		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
//
//		}
//	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	if (huart == &huart1) {
		__HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
		UART_RxCheck();
	}
}

void HAL_UART_IDLE_Callback(UART_HandleTypeDef *huart) {
	if (huart == &huart1) {
		char ch = 's';
		HAL_UART_Transmit(huart, &ch, 1, -1);
		__HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
		UART_RxCheck();
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
}

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//	if (htim->Instance == TIM2) {
////		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
//		char cha = PWM_data_sent_flag == 1 ? '1' : '0';
//		HAL_UART_Transmit(&huart1, &cha, 1, -1);
//		PWM_data_sent_flag = 1;
//	}
//}

//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
//	if (htim->Instance == TIM3) {
//		int falling_edge = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);
//		int rising_edge = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_2);
//		printf("r %d f %d\r\n", rising_edge, falling_edge);
//	}
//}

/** @defgroup ring buffer implementation
 * @{
 */

RingBuffer ring_buf = { .pos = 0 };

uint8_t _RING_CheckInstance(RingBuffer *instance) {
	if (IS_NULL(instance->buff) || !ARRAY_LEN(instance->buff)) {
		return FALSE;
	}
	return TRUE;
}

uint8_t* RING_Begin(RingBuffer *instance) {
	if (!_RING_CheckInstance(instance)) {
		return NULL;
	}
	return &instance->buff[0];
}

uint8_t* RING_End(RingBuffer *instance) {
	if (!_RING_CheckInstance(instance)) {
		return NULL;
	}
	size_t len = ARRAY_LEN(instance->buff);
	return &instance->buff[len - 1];
}

uint8_t* RING_Start(RingBuffer *instance) {
	if (!_RING_CheckInstance(instance)) {
		return NULL;
	}
	return &instance->buff[instance->pos];
}

size_t RING_BufferSize(RingBuffer *instance) {
	if (!_RING_CheckInstance(instance)) {
		return 0;
	}
	return ARRAY_LEN(instance->buff);
}

/**
 * @}
 */

/** @defgroup reading console through DMA
 * @{
 */
void UART_Stop() {
	HAL_UART_AbortReceive(&huart1);
}

void UART_Start() {
	HAL_UART_AbortReceive(&huart1);
	memset(ring_buf.buff, 0, ARRAY_LEN(ring_buf.buff));
	while (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, ring_buf.buff,
			ARRAY_LEN(ring_buf.buff)) != HAL_OK)
		;
}

void UART_RxCheck(void) {
	size_t pos = RING_BufferSize(&ring_buf)
			- LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_5);

	if (pos == ring_buf.pos) {
		return;
	}
	if (pos > ring_buf.pos) {
		UART_ProcessData(RING_Start(&ring_buf), pos - ring_buf.pos);
	} else {
		UART_ProcessData(RING_Start(&ring_buf),
				RING_BufferSize(&ring_buf) - ring_buf.pos);
		if (pos > 0) {
			UART_ProcessData(RING_Begin(&ring_buf), pos);
		}
	}
	ring_buf.pos = pos;
}

void UART_ProcessData(const void *data, size_t len) {
//	HAL_UART_DMAPause(&huart1);
//	HAL_UART_DeInit(&huart1);
//	HAL_UARTEx_EnableStopMode(&huart1);
//	HAL_UART_AbortReceive(&huart1);

//	HAL_UART_DMAStop(&huart1);
//	UART_Stop();

	UART_Stop();
	PWM_ProcessString((const char*) data, len);
	while (!LL_USART_IsActiveFlag_TC(USART1)) {
	}
//	PWM_SetIDLE();
//	HAL_UART_DMAResume(&huart1);
}

void UART_SendString(const char *str) {
	UART_ProcessData(str, strlen(str));
}

/**
 * @}
 */

/** @defgroup PWM string and char processing
 * @{
 */
void PWM_DecodeChar(char ch, uint8_t *decoded_char) {
	if (IS_NULL(decoded_char)) {
		return;
	}
	for (int i = 0, mv = 7; i <= 7; ++i, --mv) {
		decoded_char[i] = (ch & (1 << mv)) ? 1 : 0;
	}
}

void PWM_SendHello(void) {
//	uint8_t decoded_ACK[8];
//	PWM_DecodeChar(ACK, decoded_ACK);
	PWM_SendDecodedChar(ACK);
}

void PWM_SendDecodedChar(char ch) {
	for (int i = 0, mv = 7; i <= 7; ++i, --mv) {
		uint8_t bit = (ch & (1 << mv)) ? 1 : 0;

		uint16_t duty_cycle = bit == 0 ? PWM_LOW_PERCENT : PWM_HIGH_PERCENT;
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,
				GET_PERCENT_VALUE(TIM2->ARR, duty_cycle));

		HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_2);

//		__HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC2);
//		__HAL_TIM_DISABLE_IT(&htim2, TIM_IT_COM);
//		__HAL_TIM_DISABLE_IT(&htim2, TIM_IT_TRIGGER);
//		__HAL_TIM_DISABLE_IT(&htim2, TIM_IT_BREAK);

		while (!PWM_data_sent_flag)
			;
		PWM_data_sent_flag = 0;
	}

// !

//	if (HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_2) != HAL_OK) {
//		Error_Handler();
//	}
//	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_2);
//	while (!*data) {
//	}
//	*data = 0;

//	uint32_t tr_ar[8];
//	for (int i = 0, mv = 7; i <= 7; ++i, --mv) {
//		uint8_t bit = (ch & (1 << mv)) ? 1 : 0;
//		if (bit == 0) {
//			tr_ar[i] = 0;
//		} else if (bit == 1) {
//			tr_ar[i] = 100;
//		}
//	}
//
//	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
//	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_2, tr_ar, 8);
//
//	while (HAL_TIM_GetChannelState(&htim2, TIM_CHANNEL_2)
//			!= HAL_TIM_CHANNEL_STATE_READY) {
//		char c;
//		int status = HAL_TIM_GetChannelState(&htim2, TIM_CHANNEL_2);
//		if (status == HAL_TIM_CHANNEL_STATE_RESET) {
//			c = '0';
//		} else if (status == HAL_TIM_CHANNEL_STATE_READY) {
//			c = '1';
//		} else if (status == HAL_TIM_CHANNEL_STATE_BUSY) {
//			c = '2';
//		}
//		HAL_UART_Transmit(&huart1, &c, 1, -1);
//	}
//	PWM_data_sent_flag = 0;

//	while (HAL_DMA_GetState(&hdma_tim2_ch2) != HAL_DMA_STATE_READY) {
//		char ch;
//		int status = HAL_DMA_GetState(&hdma_tim2_ch2);
//		if (status == HAL_DMA_STATE_RESET) {
//			ch = '0';
//		} else if (status == HAL_DMA_STATE_READY) {
//			ch = '1';
//		} else if (status == HAL_DMA_STATE_BUSY) {
//			ch = '2';
//		} else if (status == HAL_DMA_STATE_TIMEOUT) {
//			ch = '3';
//		}
//		HAL_UART_Transmit(&huart1, &ch, 1, -1);
//	}

//	while (!DMA_GetFlagStatus(DMA2_FLAG_TC3))
//		;
//	DMA_ClearFlag(DMA2_FLAG_TC3);

//	while (!PWM_data_sent_flag) {
//	}
//	PWM_data_sent_flag = 0;
}

void PWM_SetIDLE(void) {
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,
			GET_PERCENT_VALUE(TIM2->ARR, PWM_IDLE_PERCENT));
//	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_2);
}

void PWM_ProcessString(const char *str, size_t len) {
	volatile const char *s = str;
	HAL_UART_Transmit(&huart1, str, strlen(str), -1);
	while (len > 0) {
//		uint8_t bin[8];
//		PWM_DecodeChar(*s, bin);
		PWM_SendDecodedChar(*s);
//		PWM_SetIDLE();
		len--, s++;
	}
}

/**
 * @}
 */

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
	MX_TIM16_Init();
	/* USER CODE BEGIN 2 */

	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);

//	HAL_TIM_Base_Start_IT(&htim2);

//	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
//	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
//	HAL_TIM_Base_Start_IT(&htim2);
//	HAL_TIM_Base_Start_IT(&htim3);
	if (HAL_TIM_Base_Start_IT(&htim16) != HAL_OK) {
		Error_Handler();
	}

//	if (HAL_UARTEx_ReceiveToIdle_IT(&huart1, (uint8_t*) ring_buf.buff,
//			ARRAY_LEN(ring_buf.buff)) != HAL_OK) {
//		Error_Handler();
//	}

//	if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t*) ring_buf.buff,
//			ARRAY_LEN(ring_buf.buff)) != HAL_OK) {
//		Error_Handler();
//	}
//	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

	HAL_UART_Receive_DMA(&huart1, (uint8_t*) ring_buf.buff,
			ARRAY_LEN(ring_buf.buff));
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
//		char ch;
//		HAL_UART_Receive(&huart1, (uint8_t*) &ch, 1, -1);
//		PWM_SendDecodedChar(ch);
//
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

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 48000 - 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 100 - 1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
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

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_SlaveConfigTypeDef sSlaveConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_IC_InitTypeDef sConfigIC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 48000 - 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 100 - 1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
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
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
	sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief TIM16 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM16_Init(void) {

	/* USER CODE BEGIN TIM16_Init 0 */

	/* USER CODE END TIM16_Init 0 */

	/* USER CODE BEGIN TIM16_Init 1 */

	/* USER CODE END TIM16_Init 1 */
	htim16.Instance = TIM16;
	htim16.Init.Prescaler = 48000 - 1;
	htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim16.Init.Period = 1000;
	htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim16.Init.RepetitionCounter = 0;
	htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim16) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM16_Init 2 */

	/* USER CODE END TIM16_Init 2 */

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
	/* DMA1_Channel4_5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

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
	printf("Error\r\n");
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

