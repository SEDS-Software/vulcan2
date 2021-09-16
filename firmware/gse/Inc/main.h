/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define EM_MAIN_GO_Pin GPIO_PIN_2
#define EM_MAIN_GO_GPIO_Port GPIOE
#define EM_MAIN_SENSE_Pin GPIO_PIN_4
#define EM_MAIN_SENSE_GPIO_Port GPIOE
#define KEY_Pin GPIO_PIN_5
#define KEY_GPIO_Port GPIOE
#define V2_PWR_EN_Pin GPIO_PIN_6
#define V2_PWR_EN_GPIO_Port GPIOE
#define PT1_Pin GPIO_PIN_1
#define PT1_GPIO_Port GPIOC
#define PT2_Pin GPIO_PIN_2
#define PT2_GPIO_Port GPIOC
#define BUTTON_Pin GPIO_PIN_0
#define BUTTON_GPIO_Port GPIOA
#define PT3_Pin GPIO_PIN_1
#define PT3_GPIO_Port GPIOA
#define PT4_Pin GPIO_PIN_2
#define PT4_GPIO_Port GPIOA
#define PT5_Pin GPIO_PIN_3
#define PT5_GPIO_Port GPIOA
#define PT6_Pin GPIO_PIN_5
#define PT6_GPIO_Port GPIOA
#define PT7_Pin GPIO_PIN_6
#define PT7_GPIO_Port GPIOA
#define PT8_Pin GPIO_PIN_7
#define PT8_GPIO_Port GPIOA
#define S_CTL_1_Pin GPIO_PIN_7
#define S_CTL_1_GPIO_Port GPIOE
#define S_CTL_2_Pin GPIO_PIN_8
#define S_CTL_2_GPIO_Port GPIOE
#define S_CTL_3_Pin GPIO_PIN_9
#define S_CTL_3_GPIO_Port GPIOE
#define S_CTL_4_Pin GPIO_PIN_10
#define S_CTL_4_GPIO_Port GPIOE
#define S_CTL_5_Pin GPIO_PIN_11
#define S_CTL_5_GPIO_Port GPIOE
#define S_CTL_6_Pin GPIO_PIN_12
#define S_CTL_6_GPIO_Port GPIOE
#define S_CTL_7_Pin GPIO_PIN_13
#define S_CTL_7_GPIO_Port GPIOE
#define S_CTL_8_Pin GPIO_PIN_14
#define S_CTL_8_GPIO_Port GPIOE
#define S_CTL_9_Pin GPIO_PIN_15
#define S_CTL_9_GPIO_Port GPIOE
#define S_CTL_10_Pin GPIO_PIN_11
#define S_CTL_10_GPIO_Port GPIOB
#define S_CTL_11_Pin GPIO_PIN_12
#define S_CTL_11_GPIO_Port GPIOB
#define S_CTL_12_Pin GPIO_PIN_13
#define S_CTL_12_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_12
#define LED1_GPIO_Port GPIOD
#define LED2_Pin GPIO_PIN_13
#define LED2_GPIO_Port GPIOD
#define LED3_Pin GPIO_PIN_14
#define LED3_GPIO_Port GPIOD
#define LED4_Pin GPIO_PIN_15
#define LED4_GPIO_Port GPIOD
#define SG3_DAT_Pin GPIO_PIN_8
#define SG3_DAT_GPIO_Port GPIOC
#define SG3_CLK_Pin GPIO_PIN_9
#define SG3_CLK_GPIO_Port GPIOC
#define SG1_DAT_Pin GPIO_PIN_0
#define SG1_DAT_GPIO_Port GPIOD
#define SG2_CLK_Pin GPIO_PIN_1
#define SG2_CLK_GPIO_Port GPIOD
#define SG1_CLK_Pin GPIO_PIN_2
#define SG1_CLK_GPIO_Port GPIOD
#define SG2_DAT_Pin GPIO_PIN_3
#define SG2_DAT_GPIO_Port GPIOD
#define EM_BACK_GO_Pin GPIO_PIN_0
#define EM_BACK_GO_GPIO_Port GPIOE
#define EM_BACK_SENSE_Pin GPIO_PIN_1
#define EM_BACK_SENSE_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
