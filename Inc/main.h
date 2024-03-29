/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define Key0_Pin GPIO_PIN_1
#define Key0_GPIO_Port GPIOA
#define Key1_Pin GPIO_PIN_2
#define Key1_GPIO_Port GPIOA
#define Key2_Pin GPIO_PIN_3
#define Key2_GPIO_Port GPIOA
#define Key3_Pin GPIO_PIN_4
#define Key3_GPIO_Port GPIOA
#define Key4_Pin GPIO_PIN_5
#define Key4_GPIO_Port GPIOA
#define Key5_Pin GPIO_PIN_6
#define Key5_GPIO_Port GPIOA
#define Key6_Pin GPIO_PIN_7
#define Key6_GPIO_Port GPIOA
#define free_Pin GPIO_PIN_0
#define free_GPIO_Port GPIOB
#define LCD_SCL_Pin GPIO_PIN_10
#define LCD_SCL_GPIO_Port GPIOB
#define LCD_SDA_Pin GPIO_PIN_11
#define LCD_SDA_GPIO_Port GPIOB
#define Freq_Pin GPIO_PIN_3
#define Freq_GPIO_Port GPIOB
#define Tx_OW_Pin GPIO_PIN_6
#define Tx_OW_GPIO_Port GPIOB
#define E_A1_Pin GPIO_PIN_8
#define E_A1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
// �������� ��� ������� �� VCP 
//void CheckRxVCP (uint8_t *pData, uint16_t Size);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
