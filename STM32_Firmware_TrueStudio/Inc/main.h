/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define CSBB_Pin GPIO_PIN_13
#define CSBB_GPIO_Port GPIOC
#define Aux_2_Pin GPIO_PIN_0
#define Aux_2_GPIO_Port GPIOH
#define Aux_1_Pin GPIO_PIN_1
#define Aux_1_GPIO_Port GPIOH
#define CS00_Pin GPIO_PIN_0
#define CS00_GPIO_Port GPIOC
#define CS01_Pin GPIO_PIN_1
#define CS01_GPIO_Port GPIOC
#define CS02_Pin GPIO_PIN_2
#define CS02_GPIO_Port GPIOC
#define EMG_1_Pin GPIO_PIN_3
#define EMG_1_GPIO_Port GPIOC
#define Voltage_Sense_Pin GPIO_PIN_0
#define Voltage_Sense_GPIO_Port GPIOA
#define Current_Sense_Pin GPIO_PIN_1
#define Current_Sense_GPIO_Port GPIOA
#define EMG_2_Pin GPIO_PIN_2
#define EMG_2_GPIO_Port GPIOA
#define CS03_Pin GPIO_PIN_3
#define CS03_GPIO_Port GPIOA
#define CS04_Pin GPIO_PIN_4
#define CS04_GPIO_Port GPIOA
#define CS05_Pin GPIO_PIN_4
#define CS05_GPIO_Port GPIOC
#define CS06_Pin GPIO_PIN_5
#define CS06_GPIO_Port GPIOC
#define CS07_Pin GPIO_PIN_0
#define CS07_GPIO_Port GPIOB
#define CS08_Pin GPIO_PIN_1
#define CS08_GPIO_Port GPIOB
#define CS09_Pin GPIO_PIN_2
#define CS09_GPIO_Port GPIOB
#define CS10_Pin GPIO_PIN_10
#define CS10_GPIO_Port GPIOB
#define CS11_Pin GPIO_PIN_12
#define CS11_GPIO_Port GPIOB
#define CS12_Pin GPIO_PIN_6
#define CS12_GPIO_Port GPIOC
#define CS13_Pin GPIO_PIN_7
#define CS13_GPIO_Port GPIOC
#define RS485_EN_Pin GPIO_PIN_8
#define RS485_EN_GPIO_Port GPIOC
#define FTDI_EN_Pin GPIO_PIN_9
#define FTDI_EN_GPIO_Port GPIOC
#define CS14_Pin GPIO_PIN_8
#define CS14_GPIO_Port GPIOA
#define RS485_TX_Pin GPIO_PIN_9
#define RS485_TX_GPIO_Port GPIOA
#define RS485_RX_Pin GPIO_PIN_10
#define RS485_RX_GPIO_Port GPIOA
#define RS485_CTS_Pin GPIO_PIN_11
#define RS485_CTS_GPIO_Port GPIOA
#define CS15_Pin GPIO_PIN_12
#define CS15_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define CS_WIFI_Pin GPIO_PIN_15
#define CS_WIFI_GPIO_Port GPIOA
#define UART_TX_Pin GPIO_PIN_12
#define UART_TX_GPIO_Port GPIOC
#define UART_RX_Pin GPIO_PIN_2
#define UART_RX_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define Driver_EN_Pin GPIO_PIN_4
#define Driver_EN_GPIO_Port GPIOB
#define Driver_1_Pin GPIO_PIN_6
#define Driver_1_GPIO_Port GPIOB
#define Driver_2_Pin GPIO_PIN_7
#define Driver_2_GPIO_Port GPIOB
#define CS16_Pin GPIO_PIN_8
#define CS16_GPIO_Port GPIOB
#define CS17_Pin GPIO_PIN_9
#define CS17_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
