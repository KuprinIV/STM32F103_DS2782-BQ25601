/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
void Load_66mA_Ctrl(uint8_t is_enabled);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LOAD_EN_Pin GPIO_PIN_3
#define LOAD_EN_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOB
#define PWR_BTN_Pin GPIO_PIN_15
#define PWR_BTN_GPIO_Port GPIOB
#define QON_Pin GPIO_PIN_15
#define QON_GPIO_Port GPIOA
#define GHGEN_Pin GPIO_PIN_3
#define GHGEN_GPIO_Port GPIOB
#define INT_Pin GPIO_PIN_4
#define INT_GPIO_Port GPIOB
#define INT_EXTI_IRQn EXTI4_IRQn
#define PG_Pin GPIO_PIN_5
#define PG_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define COMMANDS_ID							1
#define COMMANDS_RESPONSE_ID				2

#define COMMANDS_SZ							36
#define COMMANDS_RESPONSE_SZ				36

#define READ_DS2782_REGISTERS_DATA			0xC0
#define READ_DS2782_EEPROM_DATA				0xC1
#define WRITE_DS2782_EEPROM_DATA			0xC2
#define LOCK_DS2782_EEPROM_BLOCK			0xC3
#define READ_DS2782_EEPROM_LOCK_STATUS		0xC4
#define READ_BQ25601_STATUS					0xC5
#define READ_BQ25601_FAULTS					0xC6
#define BQ25601_CHG_CTRL					0xC7
#define LOAD_66MA_CTRL						0xC8

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
