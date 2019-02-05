/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define op_led_Pin GPIO_PIN_13
#define op_led_GPIO_Port GPIOC
#define IP4_Pin GPIO_PIN_14
#define IP4_GPIO_Port GPIOC
#define IP3_Pin GPIO_PIN_15
#define IP3_GPIO_Port GPIOC
#define IP2_Pin GPIO_PIN_0
#define IP2_GPIO_Port GPIOF
#define IP1_Pin GPIO_PIN_1
#define IP1_GPIO_Port GPIOF
#define LED_PWR_Pin GPIO_PIN_0
#define LED_PWR_GPIO_Port GPIOA
#define LED_GSM_Pin GPIO_PIN_1
#define LED_GSM_GPIO_Port GPIOA
#define LDR_FRONT_ADC_Pin GPIO_PIN_2
#define LDR_FRONT_ADC_GPIO_Port GPIOA
#define LDR_BACK_ADC_Pin GPIO_PIN_3
#define LDR_BACK_ADC_GPIO_Port GPIOA
#define EXT_BAT_DET_Pin GPIO_PIN_0
#define EXT_BAT_DET_GPIO_Port GPIOB
#define INT_BAT_ADC_Pin GPIO_PIN_1
#define INT_BAT_ADC_GPIO_Port GPIOB
#define GSM_POWER_Pin GPIO_PIN_12
#define GSM_POWER_GPIO_Port GPIOB
#define GSM_VDD_EXT_Pin GPIO_PIN_13
#define GSM_VDD_EXT_GPIO_Port GPIOB
#define POWER_KEY_GSM_Pin GPIO_PIN_14
#define POWER_KEY_GSM_GPIO_Port GPIOB
#define RI_GSM_Pin GPIO_PIN_15
#define RI_GSM_GPIO_Port GPIOB
#define DTR_GSM_Pin GPIO_PIN_8
#define DTR_GSM_GPIO_Port GPIOA
#define RX_GSM_Pin GPIO_PIN_9
#define RX_GSM_GPIO_Port GPIOA
#define TX_GSM_Pin GPIO_PIN_10
#define TX_GSM_GPIO_Port GPIOA
#define WDT_WDI_Pin GPIO_PIN_8
#define WDT_WDI_GPIO_Port GPIOB
#define WDT_EN_Pin GPIO_PIN_9
#define WDT_EN_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
