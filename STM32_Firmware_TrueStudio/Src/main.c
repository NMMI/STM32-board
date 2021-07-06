// ----------------------------------------------------------------------------
// BSD 3-Clause License

// Copyright (c) 2016, qbrobotics
// Copyright (c) 2017-2021, Centro "E.Piaggio"
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.

// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.

// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// POSSIBILITY OF SUCH DAMAGE.
// ----------------------------------------------------------------------------

/**
* \file         main.c
*
* \brief        Firmware main file.
* \date         July 6th, 2021
* \author       _Centro "E.Piaggio"_
* \copyright    (C) 2021 Centro "E.Piaggio". All rights reserved.
* \mainpage     Firmware
* \brief        This is the firmware of PSoC5 logic board.
* \version      0.8
*
* \details      This is the firmware of STM32 logic board. Depending on the configuration,
*               it can control IMUs and read encoders. Also can read and
*               convert analog measurements connected to the microcontroller.
*
*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
//#include "User_Functions\Serial_Functions.h"
#include "User_Functions\globals.h" // ALL GLOBAL DEFINITIONS, STRUCTURES AND MACROS HERE
#include "User_Functions\interruptions.h"
#include "User_Functions\command_processing.h"
#include "User_Functions\utils.h"
#include "User_Functions\IMU_functions.h"
#include "User_Functions\Encoder_functions.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
//ADC_HandleTypeDef hadc1;

//SPI_HandleTypeDef hspi1;
//SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart5;
//UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t receive_flag = 0;
_Bool Led_Status = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_UART5_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART1_UART_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
  MX_SPI1_Init();
  MX_SPI2_Init();
  HAL_Delay(100);
  DWT_Init(); // Questa dovrebbe inizialiazzare la libreria che conta i microsecondi

  // Flash Settings
  mem_settings.TypeErase = FLASH_TYPEERASE_SECTORS; // Type of writing
  mem_settings.VoltageRange = FLASH_VOLTAGE_RANGE_2; //2
  mem_settings.Sector = FLASH_SECTOR_7; // Last Sector // 7
  mem_settings.NbSectors = 1; // Number of Sector to write
  // EEPROM
  memRecall();
  HAL_Delay(200);

  // QUI SCRIVO TUTTI I CODICI DI INIZIALIZZAZIONE
  HAL_GPIO_WritePin(CSBB_GPIO_Port, CSBB_Pin, 1);
  HAL_GPIO_WritePin(CS00_GPIO_Port, CS00_Pin, 1);
  HAL_GPIO_WritePin(CS01_GPIO_Port, CS01_Pin, 1);
  HAL_GPIO_WritePin(CS02_GPIO_Port, CS02_Pin, 1);
  HAL_GPIO_WritePin(CS03_GPIO_Port, CS03_Pin, 1);
  HAL_GPIO_WritePin(CS04_GPIO_Port, CS04_Pin, 1);
  HAL_GPIO_WritePin(CS05_GPIO_Port, CS05_Pin, 1); // [GS] Qui sulla millefori c'è attaccato un led che uso per debug
  HAL_GPIO_WritePin(CS06_GPIO_Port, CS06_Pin, 1);
  HAL_GPIO_WritePin(CS07_GPIO_Port, CS07_Pin, 1);
  HAL_GPIO_WritePin(CS08_GPIO_Port, CS08_Pin, 1);
  HAL_GPIO_WritePin(CS09_GPIO_Port, CS09_Pin, 1);
  HAL_GPIO_WritePin(CS10_GPIO_Port, CS10_Pin, 1);
  HAL_GPIO_WritePin(CS11_GPIO_Port, CS11_Pin, 1);
  HAL_GPIO_WritePin(CS12_GPIO_Port, CS12_Pin, 1);
  HAL_GPIO_WritePin(CS13_GPIO_Port, CS13_Pin, 1);
  HAL_GPIO_WritePin(CS14_GPIO_Port, CS14_Pin, 1);
  HAL_GPIO_WritePin(CS15_GPIO_Port, CS15_Pin, 1);
  HAL_GPIO_WritePin(CS16_GPIO_Port, CS16_Pin, 1);
  HAL_GPIO_WritePin(CS17_GPIO_Port, CS17_Pin, 1);

  // Init AS5045 devices
  if (g_mem.read_encoders) {
	  InitEncoderGeneral();
	  HAL_Delay(10);
  }
  // Init MPU9250 devices
  if (g_mem.read_imu) {
	  InitIMUgeneral();
	  HAL_Delay(100);

	  for (int k_quat = 0; k_quat < N_IMU_MAX; k_quat++) {
		Quat[k_quat][0] = 0.999;
		Quat[k_quat][1] = 0.01;
		Quat[k_quat][2] = 0.01;
		Quat[k_quat][3] = 0.01;
	  }
  }

  MX_DMA_Init();
  HAL_Delay(100);
  MX_ADC1_Init();
  HAL_Delay(100);
  HAL_ADC_Start(&hadc1);
  HAL_Delay(1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADCValues, 5); // [GS] Ricontrollare se legge per bene tutti i sensori
  HAL_Delay(100);

  HAL_GPIO_WritePin(FTDI_EN_GPIO_Port, FTDI_EN_Pin, 1);
  HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, 0);
  HAL_GPIO_WritePin(RS485_CTS_GPIO_Port, RS485_CTS_Pin, 0);
  HAL_Delay(100);
  MX_USART1_UART_Init();
  HAL_UART_Receive_IT(&huart1, &Rx_data, 1); // Enables the interrupt when a byte is received on the serial port
  g_rx.length = 0;
  g_rx.ready  = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	function_scheduler();


  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */

  sConfig_global.Channel = ADC_CHANNEL_0;  // Voltage
  sConfig_global.Rank = 3;
  sConfig_global.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig_global) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfig_global.Channel  = ADC_CHANNEL_1; // Current
  sConfig_global.Rank = 4;
  sConfig_global.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig_global) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfig_global.Channel = ADC_CHANNEL_2; // EMG_1
  sConfig_global.Rank = 1;
  sConfig_global.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig_global) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfig_global.Channel = ADC_CHANNEL_13; // EMG_2
  sConfig_global.Rank = 2;
  sConfig_global.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig_global) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfig_global.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig_global.Rank = 5;
  sConfig_global.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig_global) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  // ADCValues[0] = Voltage
  // ADCValues[1] = Current
  // ADCValues[2] = Temperature
  // ADCValues[3] = EMG_1
  // ADCValues[4] = EMG_2


}

/**
  * Enable DMA controller clock
  */
void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __DMA2_CLK_ENABLE();

  /* DMA interrupt init */
 HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);

}


/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE; // 1EDGE
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI3 init function */
static void MX_SPI3_Init(void)
{

  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* UART5 init function */
static void MX_UART5_Init(void)
{

  huart5.Instance = UART5;
  huart5.Init.BaudRate = 2000000;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_8;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 2000000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_8;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CSBB_Pin|CS00_Pin|CS01_Pin|CS02_Pin
                          |CS05_Pin|CS06_Pin|CS12_Pin|CS13_Pin 
                          |RS485_EN_Pin|FTDI_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, Aux_2_Pin|Aux_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CS03_Pin|CS04_Pin|CS14_Pin|RS485_CTS_Pin 
                          |CS15_Pin|CS_WIFI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CS07_Pin|CS08_Pin|CS09_Pin|CS10_Pin 
                          |CS11_Pin|Driver_EN_Pin|CS16_Pin|CS17_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CSBB_Pin CS00_Pin CS01_Pin CS02_Pin 
                           CS05_Pin CS06_Pin CS12_Pin CS13_Pin 
                           RS485_EN_Pin FTDI_EN_Pin */
  GPIO_InitStruct.Pin = CSBB_Pin|CS00_Pin|CS01_Pin|CS02_Pin
                          |CS05_Pin|CS06_Pin|CS12_Pin|CS13_Pin 
                          |RS485_EN_Pin|FTDI_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Aux_2_Pin Aux_1_Pin */
  GPIO_InitStruct.Pin = Aux_2_Pin|Aux_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : CS03_Pin CS04_Pin CS14_Pin RS485_CTS_Pin 
                           CS15_Pin CS_WIFI_Pin */
  GPIO_InitStruct.Pin = CS03_Pin|CS04_Pin|CS14_Pin|RS485_CTS_Pin 
                          |CS15_Pin|CS_WIFI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CS07_Pin CS08_Pin CS09_Pin CS10_Pin 
                           CS11_Pin Driver_EN_Pin CS16_Pin CS17_Pin */
  GPIO_InitStruct.Pin = CS07_Pin|CS08_Pin|CS09_Pin|CS10_Pin 
                          |CS11_Pin|Driver_EN_Pin|CS16_Pin|CS17_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Driver_1_Pin Driver_2_Pin */
  GPIO_InitStruct.Pin = Driver_1_Pin|Driver_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	if (huart->Instance == USART1) {
		interrupt_flag = TRUE;
		Rx_buffer[Rx_counter] = Rx_data;
		Rx_counter ++;
		if (Rx_counter > Rx_Buffer_size)  {Rx_counter = 0;}
		HAL_UART_Receive_IT(&huart1, &Rx_data, 1);
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
