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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_RX_BUFFER_SIZE 100
#define MAX_DIGITS 8

#define DS1337_ADDRESS (0x68 << 1)

#define DEBOUNCE_CHECKS 5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t error_state;

uint8_t digit_to_segment[] = {
    0b00111111, // 0
    0b00000110, // 1
    0b01011011, // 2
    0b01001111, // 3
    0b01100110, // 4
    0b01101101, // 5
    0b01111101, // 6
    0b00000111, // 7
    0b01111111, // 8
    0b01101111  // 9
};

uint8_t current_indicator = 0;
const char *display_string = "12345678";

uint8_t spi_data[2];

uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
uint8_t uart_index = 0;

uint8_t digit_states[MAX_DIGITS] = {0};

uint8_t rtcData[7];
uint8_t rtcDataWrite[7];

uint8_t seconds;
uint8_t minutes;
uint8_t hours;
uint8_t dayOfWeek;
uint8_t date;
uint8_t month;
uint8_t year;

uint8_t clock_state = 0; // 0 - time; 1 - date

uint8_t debounce_buffer[DEBOUNCE_CHECKS] = {0};
uint8_t debounce_index = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */

void ShiftRegister_Send(uint8_t indicator, uint8_t segments);
uint8_t char_to_digit(char c);

uint8_t DEC2BCD(uint8_t val);
uint8_t BCD2DEC(uint8_t val);
void Read_RTC_TimeDate(void);
void Set_RTC_Time(uint8_t hours, uint8_t minutes, uint8_t seconds);
void Set_RTC_Date(uint8_t date, uint8_t month, uint8_t year);
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
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim8);
  HAL_UART_Receive_IT(&huart2, (uint8_t *)uart_rx_buffer, 1);
  for (int i = 0; i < MAX_DIGITS; i++) {
	  digit_states[i] = digit_to_segment[0];
  }
  if (HAL_I2C_IsDeviceReady(&hi2c1, DS1337_ADDRESS, 1, HAL_MAX_DELAY) != HAL_OK)
  {
      seconds = 99;
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//	Read_RTC_TimeDate();
//	HAL_Delay(1000);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_1LINE;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 179;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 1799;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 49999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_SS_GPIO_Port, SPI2_SS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SPI2_SS_Pin */
  GPIO_InitStruct.Pin = SPI2_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI2_SS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_RED_EXTI_Pin */
  GPIO_InitStruct.Pin = BTN_RED_EXTI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN_RED_EXTI_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint8_t char_to_digit(char c) {
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    return 0;
}


void process_uart_string(char *str) {
    char *input = strchr(str, 't');
    if (input != NULL){
        char hours_str[3] = {input[1], input[2], '\0'};
        char minutes_str[3] = {input[4], input[5], '\0'};
        char seconds_str[3] = {input[7], input[8], '\0'};

        uint8_t temp_hours = (uint8_t)atoi(hours_str);
        uint8_t temp_minutes = (uint8_t)atoi(minutes_str);
        uint8_t temp_seconds = (uint8_t)atoi(seconds_str);

        if (temp_hours <= 23 && temp_minutes <= 59 && temp_seconds <= 59) {
//        	hours = temp_hours;
//			minutes = temp_minutes;
//			seconds = temp_seconds;
			Set_RTC_Time(temp_hours, temp_minutes, temp_seconds);
		}
		return;
    }
    input = strchr(str, 'd');
    if (input != NULL){
		char date_str[3] = {input[1], input[2], '\0'};
		char month_str[3] = {input[4], input[5], '\0'};
		char year_str[3] = {input[7], input[8], '\0'};

		uint8_t temp_date = (uint8_t)atoi(date_str);
		uint8_t temp_month = (uint8_t)atoi(month_str);
		uint8_t temp_year = (uint8_t)atoi(year_str);

		if (temp_date <= 28 && temp_month <= 12 && temp_year <= 99) {
//			date = temp_date;
//			month = temp_month;
//			year = temp_year;
			Set_RTC_Date(temp_date, temp_month, temp_year);
		}
		return;
        }
    return;
}

void Read_RTC_TimeDate(void){
    if (HAL_I2C_Mem_Read_DMA(&hi2c1, DS1337_ADDRESS, 0x00, I2C_MEMADD_SIZE_8BIT, rtcData, 7) != HAL_OK)
    {
        Error_Handler();
    }
}

void Set_RTC_Time(uint8_t hours, uint8_t minutes, uint8_t seconds){
    uint8_t timeData[3];

    timeData[0] = DEC2BCD(seconds);
    timeData[1] = DEC2BCD(minutes);
    timeData[2] = DEC2BCD(hours);
    if (HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_READY)
	{
    	HAL_I2C_Mem_Write_DMA(&hi2c1, DS1337_ADDRESS, 0x00, I2C_MEMADD_SIZE_8BIT, timeData, 3);
	}
    else{
		Error_Handler();
	}

//    if (HAL_I2C_Mem_Write_DMA(&hi2c1, DS1337_ADDRESS, 0x00, I2C_MEMADD_SIZE_8BIT, timeData, 3) != HAL_OK)
//    {
//        Error_Handler();
//    }
}

void Set_RTC_Date(uint8_t date, uint8_t month, uint8_t year){
    uint8_t dateData[3];

    dateData[0] = DEC2BCD(date);
    dateData[1] = DEC2BCD(month);
    dateData[2] = DEC2BCD(year);

    if (HAL_I2C_Mem_Write_DMA(&hi2c1, DS1337_ADDRESS, 0x04, I2C_MEMADD_SIZE_8BIT, dateData, 3) != HAL_OK)
    {
        Error_Handler();
    }
}

uint8_t DEC2BCD(uint8_t val){
    return ((val / 10) << 4) | (val % 10);
}

uint8_t BCD2DEC(uint8_t val){
    return ((val >> 4) * 10) + (val & 0x0F);
}

void update_digit_states() {
	if (!clock_state){
		digit_states[0] = digit_to_segment[hours / 10];
		digit_states[1] = digit_to_segment[hours % 10];
		digit_states[1] |= 0b10000000;
		digit_states[2] = digit_to_segment[minutes / 10];
		digit_states[3] = digit_to_segment[minutes % 10];
		digit_states[3] |= 0b10000000;
		digit_states[4] = digit_to_segment[seconds / 10];
		digit_states[5] = digit_to_segment[seconds % 10];
		digit_states[6] = 0;
		digit_states[7] = 0;
	}
	else{
		digit_states[0] = digit_to_segment[date / 10];
		digit_states[1] = digit_to_segment[date % 10];
		digit_states[1] |= 0b10000000;
		digit_states[2] = digit_to_segment[month / 10];
		digit_states[3] = digit_to_segment[month % 10];
		digit_states[3] |= 0b10000000;
		digit_states[4] = digit_to_segment[2];
		digit_states[5] = digit_to_segment[0];
		digit_states[6] = digit_to_segment[year / 10];;
		digit_states[7] = digit_to_segment[year % 10];;
	}
}



void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
    if (hi2c->Instance == I2C1)
    {
		seconds = BCD2DEC(rtcData[0]);
		minutes = BCD2DEC(rtcData[1]);
		hours = BCD2DEC(rtcData[2]);
		dayOfWeek = BCD2DEC(rtcData[3]);
		date = BCD2DEC(rtcData[4]);
		month = BCD2DEC(rtcData[5]);
		year = BCD2DEC(rtcData[6]);
    }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI2) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        if (uart_rx_buffer[uart_index] == '\r') {
            uart_rx_buffer[uart_index + 1] = '\0';
            process_uart_string((char *)uart_rx_buffer);
            uart_index = 0;
        } else {
            uart_index++;
            if (uart_index >= sizeof(uart_rx_buffer) - 1) {
				uart_index = 0;
			}
        }
        HAL_UART_Receive_IT(&huart2, &uart_rx_buffer[uart_index], 1);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM1) {

    	update_digit_states();

        uint8_t indicator = ~(1 << current_indicator);
        uint8_t segments = digit_states[current_indicator];

        spi_data[0] = segments;
        spi_data[1] = indicator;

        HAL_SPI_Transmit_IT(&hspi2, spi_data, 2);

        current_indicator = (current_indicator + 1) % MAX_DIGITS;
    }
    if (htim->Instance == TIM8) {

		Read_RTC_TimeDate();
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    if (GPIO_Pin == BTN_RED_EXTI_Pin){
    	clock_state = !clock_state;
	}
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
  error_state = 1;
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
