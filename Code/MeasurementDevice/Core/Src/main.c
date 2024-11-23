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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum TubeState {
	TUBE_FILL_ONLY, // Water flows in from top, held at the bottom
	TUBE_DRAIN_ONLY, // Water flows out from bottom, no inflow from top
	TUBE_HOLD, // Water held in tube
	TUBE_FLOW_THROUGH, // Water freely flows thru tube
	TUBE_UNKNOWN // Initial state
} TubeState;

typedef struct GPIOPin
{
    GPIO_TypeDef *port;
    uint16_t pin;
} GPIOPin;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 0xFF was chosen since given conditions of system, we will probably
// never send this byte, and if we do, it's in an place where we can simply change it
// to one lower and it won't mater.
#define UART_START_BYTE 0xFF
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
const GPIOPin L293D_IN_1 = {GPIOB, GPIO_PIN_5};
const GPIOPin L293D_IN_2 = {GPIOB, GPIO_PIN_4};
const GPIOPin L293D_IN_3 = {GPIOA, GPIO_PIN_7};
const GPIOPin L293D_IN_4 = {GPIOB, GPIO_PIN_6};
const GPIOPin BLUE_PUSHBTN = {GPIOC, GPIO_PIN_13};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void prepare_uint16_for_uart(uint16_t number, uint8_t startIdx);
HAL_StatusTypeDef send_data_to_uart(float number, uint16_t timeSinceReading);

// Functions to control solenoid state
TubeState currentSolenoidState = TUBE_UNKNOWN;
void Solenoids_SetState(TubeState newState);
void Solenoids_SetRawState(GPIOPin in1, GPIOPin in2, int state);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t transmissionData[4];
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  GPIO_PinState btnStatus = 0;
  GPIO_PinState prevBtnStatus = 0;
  HAL_StatusTypeDef status;
  uint32_t lastBtnPressTimestamp = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    btnStatus = !HAL_GPIO_ReadPin(BLUE_PUSHBTN.port, BLUE_PUSHBTN.pin);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, btnStatus);
    if (prevBtnStatus != btnStatus) {
    	if (btnStatus) {
    		Solenoids_SetState(TUBE_HOLD);
    	} else {
    		Solenoids_SetState(TUBE_FLOW_THROUGH);
    	}
    }
//    if (prevBtnStatus != btnStatus) {
//    	lastBtnPressTimestamp = HAL_GetTick();
//    }
//
//    uint16_t timeSinceLastBtnPress = (uint16_t) ((HAL_GetTick() - lastBtnPressTimestamp) / 1000);
//    if (btnStatus) {
//    	status = send_data_to_uart(25.5, timeSinceLastBtnPress);
//    } else {
//    	status = send_data_to_uart(0.5, timeSinceLastBtnPress);
//    }
//
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, status != HAL_OK);
    prevBtnStatus = btnStatus;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|L293D_IN3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, L293D_IN2_Pin|L293D_IN1_Pin|L293D_IN4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin L293D_IN3_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|L293D_IN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : L293D_IN2_Pin L293D_IN1_Pin L293D_IN4_Pin */
  GPIO_InitStruct.Pin = L293D_IN2_Pin|L293D_IN1_Pin|L293D_IN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void prepare_uint16_for_uart(uint16_t number, uint8_t startIdx) {
	// follows little endian
	transmissionData[startIdx + 0] = (uint8_t)(number & 0xff); // Lower byte first
	transmissionData[startIdx + 1] = (uint8_t)((number >> 8) & 0xff); // Higher byte second

	// Make sure we never include the start byte in our messages
	if (transmissionData[startIdx + 0] == UART_START_BYTE) {
		transmissionData[startIdx + 0] = UART_START_BYTE - 1;
	}
	if (transmissionData[startIdx + 1] == 0xff) {
		transmissionData[startIdx + 1] = UART_START_BYTE - 1;
	}
}

HAL_StatusTypeDef send_data_to_uart(float number, uint16_t timeSinceLastReading) {
	if ((number * 100) > 65535 || timeSinceLastReading > 65535) {
		return HAL_ERROR; // Out of range
	}

    // Start byte for synchronization
    uint8_t startByte = UART_START_BYTE;

    // Prepare the data
    prepare_uint16_for_uart((uint16_t)(number * 100), 0);
    prepare_uint16_for_uart(timeSinceLastReading, 2);

    // Transmit start byte followed by the data
    HAL_UART_Transmit(&huart1, &startByte, 1, 500);
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart1, transmissionData, 4, 500);

    HAL_Delay(10); // Delay to ensure stable UART transmission
    return status;
}

void Solenoids_SetState(TubeState newState) {
	// Already at new state
	if (currentSolenoidState == newState) return;
	currentSolenoidState = newState;

	// 1 -> open, -1 -> close
	// 1st -> top, 2nd -> bot
	int rawSolenoidStates[2] = {0, 0};
	switch (newState) {
		case TUBE_FILL_ONLY: {
			rawSolenoidStates[0] = 1;
			rawSolenoidStates[1] = -1;
			break;
		}
		case TUBE_DRAIN_ONLY: {
			rawSolenoidStates[0] = -1;
			rawSolenoidStates[1] = 1;
			break;
		}
		case TUBE_HOLD: {
			rawSolenoidStates[0] = -1;
			rawSolenoidStates[1] = -1;
			break;
		}
		case TUBE_FLOW_THROUGH: {
			rawSolenoidStates[0] = 1;
			rawSolenoidStates[1] = 1;
			break;
		}
		default: {
			break;
		}
	}

	// TODO: make sure that the order of l293d ins actually match what would make it open/close
	Solenoids_SetRawState(L293D_IN_1, L293D_IN_2, rawSolenoidStates[0]);
	Solenoids_SetRawState(L293D_IN_3, L293D_IN_4, rawSolenoidStates[1]);
}

// 1 -> open, -1 -> close
void Solenoids_SetRawState(GPIOPin in1, GPIOPin in2, int state) {
	int individualInPinStates[2] = {GPIO_PIN_RESET, GPIO_PIN_RESET};
	switch (state) {
		case 1: {
			individualInPinStates[0] = GPIO_PIN_SET;
			individualInPinStates[1] = GPIO_PIN_RESET;

			break;
		}
		case -1: {
			individualInPinStates[0] = GPIO_PIN_RESET;
			individualInPinStates[1] = GPIO_PIN_SET;

			break;
		}
	}

	// Pulse pins for 40ms
	HAL_GPIO_WritePin(in1.port, in1.pin, individualInPinStates[0]);
	HAL_GPIO_WritePin(in2.port, in2.pin, individualInPinStates[1]);
	HAL_Delay(40); // Almost all sources say >=30ms, just wait a tiny extra bit
	HAL_GPIO_WritePin(in1.port, in1.pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(in2.port, in2.pin, GPIO_PIN_RESET);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
