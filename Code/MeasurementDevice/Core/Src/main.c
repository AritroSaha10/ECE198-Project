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
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
bool led_enabled = true;
uint32_t diode_1_1;
uint32_t diode_1_2;
uint32_t diode_2_1;
uint32_t diode_2_2;

uint32_t diode_average = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

void toggle_led();
uint32_t read_diode();
void prepare_uint16_for_uart(uint16_t number, uint8_t startIdx);
HAL_StatusTypeDef send_data_to_uart(float number, uint16_t timeSinceReading);
float ntu(uint32_t raw_value);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t transmissionData[4];

void UART_Print(const char *str) {
  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}
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
  uint32_t lastMeasurementTimestamp = 0;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  GPIO_PinState btnStatus = 0;
  GPIO_PinState prevBtnStatus = 0;
  HAL_StatusTypeDef status;
  uint32_t lastBtnPressTimestamp = HAL_GetTick();

  HAL_ADC_Start(&hadc1);

	toggle_led();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//    btnStatus = !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
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
//    prevBtnStatus = btnStatus;

    // LED Code
	uint16_t timeSinceLastMeasurement = (uint16_t) ((HAL_GetTick() - lastMeasurementTimestamp) / 1000);
	if(timeSinceLastMeasurement >= 1) {
		lastMeasurementTimestamp = HAL_GetTick();

		diode_average = read_diode();
		printf("Diodes \n\r(1, 1): %lu \r\n(1, 2): %lu\r\n(2, 1): %lu\r\n(2, 2): %lu\r\nAverage: %lu\r\n", diode_1_1, diode_1_2, diode_2_1, diode_2_2, diode_average);
		printf("NTU: %lu\r\n\n", (uint32_t) (ntu(diode_average) * 100));
	}
	send_data_to_uart(ntu(diode_average), timeSinceLastMeasurement);



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
HAL_ADC_Stop(&hadc1);

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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|L293D_IN2_Pin|L293D_IN1_Pin|L293D_IN4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : PB10 L293D_IN2_Pin L293D_IN1_Pin L293D_IN4_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_10|L293D_IN2_Pin|L293D_IN1_Pin|L293D_IN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

void toggle_led() {
//	led_enabled = !led_enabled;
	led_enabled = true;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, led_enabled);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, led_enabled);
}

//void read_diode() {
//	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
//	diode_1_1 = HAL_ADC_GetValue(&hadc1);
//
////	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
//	diode_1_2 = HAL_ADC_GetValue(&hadc1);
//
////	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
//	diode_2_1 = HAL_ADC_GetValue(&hadc1);
//
////	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
//	diode_2_2 = HAL_ADC_GetValue(&hadc1);
//}

uint32_t read_diode() {
    uint32_t sample_sum = 0;

    // Take 5 samples of the average of all photodiodes
    for (int i = 0; i < 5; i++) {
        // Wait for the full sequence conversion to complete
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);

        // Retrieve the values for each channel in the sequence
        diode_1_1 = HAL_ADC_GetValue(&hadc1);
        diode_1_2 = HAL_ADC_GetValue(&hadc1);
        diode_2_1 = HAL_ADC_GetValue(&hadc1);
        diode_2_2 = HAL_ADC_GetValue(&hadc1);

        // Compute the average of the four photodiodes for this sample
        uint32_t current_sample_average = (diode_1_1 + diode_1_2 + diode_2_1 + diode_2_2) / 4;

        // Add this sample's average to the total sum
        sample_sum += current_sample_average;

        // Wait 50 ms before the next sample
        if(i != 3) {
			HAL_Delay(50);
        }
    }

    // Compute the final filtered value as the average of the 4 samples
    return sample_sum / 5;
}

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

float ntu(uint32_t raw_value) {
	int32_t value1 = 3390;
	int32_t value2 = 3370;
	float ntu1 = 20.0f;

//	printf("%ld\r\n", (int32_t)(((int32_t)raw_value)-value1));
	return ((float) ((-ntu1/(float)(value1-value2))*(float)((int32_t)((int32_t) raw_value)-value1)));
}

void moving_average() {

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
