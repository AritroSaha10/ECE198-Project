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
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// All the pins of the display mapped to their corresponding
// pin number that it's connected to on the PCF8574.
typedef enum DisplayPin {
	DISPLAY_REGISTER_SELECT = 0,
	DISPLAY_READ_WRITE = 1,
	DISPLAY_ENABLE = 2,
	DISPLAY_DB4 = 4, // DB -> Data bus line
	DISPLAY_DB5 = 5,
	DISPLAY_DB6 = 6,
	DISPLAY_DB7 = 7
} DisplayPin;

DisplayPin displayPins[] = {
	DISPLAY_REGISTER_SELECT,
	DISPLAY_READ_WRITE,
	DISPLAY_ENABLE,
	DISPLAY_DB4,
	DISPLAY_DB5,
	DISPLAY_DB6,
	DISPLAY_DB7
};

typedef enum LCDMessageType {
	LCD_MESSAGE_COMMAND,
	LCD_MESSAGE_DATA
} LCDMessageType;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SLAVE_LCD_ADDRESS (0x38 << 1) // shifted 1 bit to the right since we're using 7 bit addrs
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
uint8_t lcd_curr_pin_state = 0; // Current state of the pins for LCD

// Directly change all pins for LCD to current value of lcd_curr_pin state,
// essentially just sending this message directly to display via I2C and PCF8574
void LCD_Write();

// These functions will set/toggle/reset the provided display pin
void LCD_SetPin(DisplayPin pin);
void LCD_TogglePin(DisplayPin pin);
void LCD_ResetPin(DisplayPin pin);

// This function serve as slightly higher level abstraction that allow us
// to send commands & data to the LCD
void LCD_SendMessage(uint8_t msg, LCDMessageType type);

// Initialization functions for LCD
void LCD_WaitForReady();
void LCD_Init();

// Managing text on display
void LCD_SendString(char *str);
void LCD_SetCursor(uint8_t row, uint8_t col);
void LCD_ClearScreen();

// Buzzer control functions
static void Tone(uint32_t Frequency);
static void noTone();

//
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  LCD_Init();
//
  LCD_SetCursor(0, 0);
  LCD_SendString("hello world");
  LCD_SetCursor(1, 0);
  LCD_SendString("wow");

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint16_t counter = 0;
  uint32_t octaveFour[8] = {262, 294, 330, 349, 392, 440, 493, 523};

  while (1)
  {
	char str[15];
	sprintf(str, "Count: %d", counter++);
	LCD_SetCursor(1, 0);
	LCD_SendString(str);
	Tone(octaveFour[counter % 8]);
	HAL_Delay(200);
	noTone();

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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

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
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void LCD_Write() {
	HAL_I2C_Master_Transmit(&hi2c1, SLAVE_LCD_ADDRESS, &lcd_curr_pin_state, 1, HAL_MAX_DELAY);
}

void LCD_SetPin(DisplayPin pin) {
	uint8_t pin_num = (uint8_t) pin;
	lcd_curr_pin_state |= (1 << pin_num);
	LCD_Write();
}

void LCD_TogglePin(DisplayPin pin) {
	uint8_t pin_num = (uint8_t) pin;
	lcd_curr_pin_state ^= (1 << pin_num);
	LCD_Write();
}

void LCD_ResetPin(DisplayPin pin) {
	uint8_t pin_num = (uint8_t) pin;
	lcd_curr_pin_state &= ~(1 << pin_num);
	LCD_Write();
}

void LCD_SendMessage(uint8_t msg, LCDMessageType type) {
	// Need to split the cmd into two, since we're in 4 bit mode
	// and that requires us to send an 8 bit message as two 4 bit messages.
	// Data bits are kept in first 4 bits to target the data bus lines
	uint8_t upper_bits = msg & 0b11110000;
	uint8_t lower_bits = (msg & 0b00001111) << 4;

	// RS pin is the first pin, and controls whether we're sending a command or data
	// If RS = 0, command, if RS = 1, data
	uint8_t message_type_mask = type == LCD_MESSAGE_DATA ? 0b0001 : 0b0000;

	// Start composing the message to send over I2C
	uint8_t i2c_msg[4];

	// Order follows as such: EN (Pin 2), RW (P1), RS (P0) (P4 isn't connected to anything)
	// EN = 1 (LCD will actually store cmd on falling edge of EN pin, so we need to pulse it up and back down)
	// RW = 0 (this will always be 0 since we always want to write)
	// RS = 0 or 1 (sets to command or data mode), controlled by message_type_mask
	i2c_msg[0] = upper_bits | 0b0100 | message_type_mask;

	// Same message, but we bring EN = 0 for the falling edge.
	// The OR mask does nothing, but is there for better readability.
	i2c_msg[1] = upper_bits | 0b0000 | message_type_mask;

	// We do the same thing as before but for the lower bits.
	i2c_msg[2] = lower_bits | 0b0100 | message_type_mask;
	i2c_msg[3] = lower_bits | 0b0000 | message_type_mask;

	HAL_I2C_Master_Transmit(&hi2c1, SLAVE_LCD_ADDRESS, (uint8_t*) i2c_msg, 4, 100);
	HAL_Delay(3); // All commands require a max of 1.64ms to respond, so this gives the PCF8574 and display some time
}

void LCD_WaitForReady() {
	HAL_StatusTypeDef result;
	uint32_t timeout = 60000; // ms
	uint32_t startTime = HAL_GetTick();

	// Keep checking until timeout
	do {
		result = HAL_I2C_IsDeviceReady(&hi2c1, SLAVE_LCD_ADDRESS, 3, 10);
		if (result == HAL_OK) break;
	} while ((HAL_GetTick() - startTime) < timeout);
}

void LCD_Init() {
	LCD_WaitForReady();

	// Follow the initialization sequence as set out by pg. 12 in datasheet
	// https://www.waveshare.com/datasheet/LCD_en_PDF/LCD1602.pdf

	// First, configure 4 bit data transfer
	HAL_Delay(50); // wait >15ms
	LCD_SendMessage(0b0011, LCD_MESSAGE_COMMAND); // Function set (interface is 8 bit length)

	HAL_Delay(5); // wait >4.1ms
	LCD_SendMessage(0b0011, LCD_MESSAGE_COMMAND); // Function set (interface is 8 bit length)

	HAL_Delay(1); // wait >100us
	LCD_SendMessage(0b0011, LCD_MESSAGE_COMMAND); // Function set (interface is 8 bit length)

	HAL_Delay(10); // datasheet doesn't say anything here but just wait 10ms
	LCD_SendMessage(0b0010, LCD_MESSAGE_COMMAND); // Set interface to 4 bit length

	// Second, initialize overall display
	LCD_SendMessage(0b00101000, LCD_MESSAGE_COMMAND); // Function set: DL=0 (4 bit mode), N=1 (2 lines), F=0 (5x8 chars)
	HAL_Delay(1);

	LCD_SendMessage(0b1000, LCD_MESSAGE_COMMAND); // On/off ctrl: Display = 0, Cursor = 0, Blink = 0
	HAL_Delay(1);

	LCD_ClearScreen();

	LCD_SendMessage(0b0110, LCD_MESSAGE_COMMAND); // Entry mode: I=1 & D=1 (increment cursor), S=0 (no shift)
	HAL_Delay(1);

	LCD_SendMessage(0b1100, LCD_MESSAGE_COMMAND); // On/off ctrl: D=1, C=0, B=0 (cursor & blink, last two bits)

	// Some padding time before immediately jumping to sending data
	HAL_Delay(1000);
}

void LCD_SendString(char *str) {
	// Send message for every character
	uint8_t len = 0;
	while (*str && len < 16) {
		LCD_SendMessage(*str++, LCD_MESSAGE_DATA);
		++len;
	}

	for (int i = len; i < 16; ++i) {
		LCD_SendMessage(' ', LCD_MESSAGE_DATA); // Fill rest of line with spaces
	}
}

void LCD_SetCursor(uint8_t row, uint8_t col) {
	assert_param(row < 2);
	assert_param(col < 16);

	// The original datasheet doesn't entirely specify how this works, so I will!
	// To change the position of the cursor, we need to change the address that we want to change
	// in the DDRAM, or Display Data RAM. To do that, we use the DDRAM AD SET command as specified
	// in the Waveshare datasheet (0b1xxxxxxx). However, this still doesn't explain how to set the actual row & col.
	// We can figure that out from the datasheet for the LCD controller: https://www.sparkfun.com/datasheets/LCD/HD44780.pdf
	// From pg. 11, we see that for row 1, addr is simply the row, and for row 2, we add 0x40 to the col.
	// We can just perform a bitwise OR for this since col will only ever be 4 bits.
	// As such, we just implement that below:
	uint8_t ddram_addr = col;
	if (row == 1) ddram_addr |= 0x40;

	uint8_t msg = ddram_addr | 0b10000000;
	LCD_SendMessage(msg, LCD_MESSAGE_COMMAND);
}

void LCD_ClearScreen() {
	LCD_SendMessage(0b0001, LCD_MESSAGE_COMMAND); // Clear display
	HAL_Delay(2);
}

// Buzzer functions were written with aid from the following resource:
// https://deepbluembedded.com/stm32-buzzer-piezo-active-passive-buzzer-example-code-tone/
static void Tone(uint32_t Frequency)
{
    TIM2->ARR = (1000000UL / Frequency) - 1; // set PWM freq
    TIM2->CCR1 = (TIM2->ARR >> 1); // set duty cycle to 50%
}

static void noTone()
{
    TIM2->CCR1 = 0; // set duty cycle to 0%
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
