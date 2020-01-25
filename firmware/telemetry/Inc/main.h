/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
#define LED4_Pin GPIO_PIN_3
#define LED4_GPIO_Port GPIOE
#define LED3_Pin GPIO_PIN_4
#define LED3_GPIO_Port GPIOE
#define LED2_Pin GPIO_PIN_5
#define LED2_GPIO_Port GPIOE
#define LED1_Pin GPIO_PIN_6
#define LED1_GPIO_Port GPIOE
#define VBAT_Pin GPIO_PIN_3
#define VBAT_GPIO_Port GPIOF
#define V5V_Pin GPIO_PIN_4
#define V5V_GPIO_Port GPIOF
#define V3V3_Pin GPIO_PIN_5
#define V3V3_GPIO_Port GPIOF
#define VGSE_Pin GPIO_PIN_6
#define VGSE_GPIO_Port GPIOF
#define PT4_Pin GPIO_PIN_0
#define PT4_GPIO_Port GPIOC
#define PT3_Pin GPIO_PIN_1
#define PT3_GPIO_Port GPIOC
#define PT2_Pin GPIO_PIN_2
#define PT2_GPIO_Port GPIOC
#define PT1_Pin GPIO_PIN_3
#define PT1_GPIO_Port GPIOC
#define MAIN_SENSE_Pin GPIO_PIN_4
#define MAIN_SENSE_GPIO_Port GPIOA
#define BACKUP_GO_Pin GPIO_PIN_5
#define BACKUP_GO_GPIO_Port GPIOA
#define MAIN_GO_Pin GPIO_PIN_6
#define MAIN_GO_GPIO_Port GPIOA
#define BACKUP_SENSE_Pin GPIO_PIN_7
#define BACKUP_SENSE_GPIO_Port GPIOA
#define IMU_DEN_Pin GPIO_PIN_10
#define IMU_DEN_GPIO_Port GPIOE
#define IMU_INT2_Pin GPIO_PIN_11
#define IMU_INT2_GPIO_Port GPIOE
#define IMU_INT1_Pin GPIO_PIN_12
#define IMU_INT1_GPIO_Port GPIOE
#define IMU_INT_M_Pin GPIO_PIN_13
#define IMU_INT_M_GPIO_Port GPIOE
#define IMU_DRDY_M_Pin GPIO_PIN_14
#define IMU_DRDY_M_GPIO_Port GPIOE
#define S_CTL_6_Pin GPIO_PIN_4
#define S_CTL_6_GPIO_Port GPIOB
#define S_CTL_5_Pin GPIO_PIN_5
#define S_CTL_5_GPIO_Port GPIOB
#define S_CTL_4_Pin GPIO_PIN_6
#define S_CTL_4_GPIO_Port GPIOB
#define S_CTL_1_Pin GPIO_PIN_7
#define S_CTL_1_GPIO_Port GPIOB
#define S_CTL_2_Pin GPIO_PIN_8
#define S_CTL_2_GPIO_Port GPIOB
#define S_CTL_3_Pin GPIO_PIN_9
#define S_CTL_3_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
