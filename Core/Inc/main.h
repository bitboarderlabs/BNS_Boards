/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ETH_nINT_Pin GPIO_PIN_2
#define ETH_nINT_GPIO_Port GPIOE
#define ETH_nRST_Pin GPIO_PIN_3
#define ETH_nRST_GPIO_Port GPIOE
#define NODEID_2_Pin GPIO_PIN_4
#define NODEID_2_GPIO_Port GPIOE
#define NODEID_4_Pin GPIO_PIN_5
#define NODEID_4_GPIO_Port GPIOE
#define NODEID_1_Pin GPIO_PIN_6
#define NODEID_1_GPIO_Port GPIOE
#define NODEID_8_Pin GPIO_PIN_13
#define NODEID_8_GPIO_Port GPIOC
#define AIN_VIO_FUSED_Pin GPIO_PIN_0
#define AIN_VIO_FUSED_GPIO_Port GPIOC
#define AIN_VLOGIC_Pin GPIO_PIN_1
#define AIN_VLOGIC_GPIO_Port GPIOC
#define AIN_VIO_UNFUSED_Pin GPIO_PIN_2
#define AIN_VIO_UNFUSED_GPIO_Port GPIOC
#define AIN3_Pin GPIO_PIN_3
#define AIN3_GPIO_Port GPIOC
#define STATUS_LED_Pin GPIO_PIN_2
#define STATUS_LED_GPIO_Port GPIOA
#define nLCD_BL_Pin GPIO_PIN_3
#define nLCD_BL_GPIO_Port GPIOA
#define AIN2_Pin GPIO_PIN_6
#define AIN2_GPIO_Port GPIOA
#define AIN4_Pin GPIO_PIN_7
#define AIN4_GPIO_Port GPIOA
#define AIN1_Pin GPIO_PIN_4
#define AIN1_GPIO_Port GPIOC
#define AIN0_Pin GPIO_PIN_5
#define AIN0_GPIO_Port GPIOC
#define ADDR_0_Pin GPIO_PIN_8
#define ADDR_0_GPIO_Port GPIOE
#define ADDR_1_Pin GPIO_PIN_9
#define ADDR_1_GPIO_Port GPIOE
#define ADDR_2_Pin GPIO_PIN_10
#define ADDR_2_GPIO_Port GPIOE
#define DIGOUT_nOE_Pin GPIO_PIN_12
#define DIGOUT_nOE_GPIO_Port GPIOE
#define nBUS_ENABLE_Pin GPIO_PIN_13
#define nBUS_ENABLE_GPIO_Port GPIOE
#define LCD_nRST_Pin GPIO_PIN_10
#define LCD_nRST_GPIO_Port GPIOB
#define LCD_AO_Pin GPIO_PIN_11
#define LCD_AO_GPIO_Port GPIOB
#define LCD_nCS_Pin GPIO_PIN_12
#define LCD_nCS_GPIO_Port GPIOB
#define RS422_DE_Pin GPIO_PIN_10
#define RS422_DE_GPIO_Port GPIOD
#define RS422_DE_REV_Pin GPIO_PIN_11
#define RS422_DE_REV_GPIO_Port GPIOD
#define SER_nRE_Pin GPIO_PIN_12
#define SER_nRE_GPIO_Port GPIOD
#define SER_nRE_REV_Pin GPIO_PIN_13
#define SER_nRE_REV_GPIO_Port GPIOD
#define BOARDID_0_Pin GPIO_PIN_8
#define BOARDID_0_GPIO_Port GPIOA
#define BOARDID_1_Pin GPIO_PIN_12
#define BOARDID_1_GPIO_Port GPIOC
#define DATA_7_Pin GPIO_PIN_0
#define DATA_7_GPIO_Port GPIOD
#define DATA_6_Pin GPIO_PIN_1
#define DATA_6_GPIO_Port GPIOD
#define DATA_5_Pin GPIO_PIN_2
#define DATA_5_GPIO_Port GPIOD
#define DATA_4_Pin GPIO_PIN_3
#define DATA_4_GPIO_Port GPIOD
#define DATA_3_Pin GPIO_PIN_4
#define DATA_3_GPIO_Port GPIOD
#define DATA_2_Pin GPIO_PIN_5
#define DATA_2_GPIO_Port GPIOD
#define DATA_1_Pin GPIO_PIN_6
#define DATA_1_GPIO_Port GPIOD
#define DATA_0_Pin GPIO_PIN_7
#define DATA_0_GPIO_Port GPIOD
#define BUS_rW_Pin GPIO_PIN_6
#define BUS_rW_GPIO_Port GPIOB
#define ETH_nCS_Pin GPIO_PIN_7
#define ETH_nCS_GPIO_Port GPIOB
#define BOARDID_2_Pin GPIO_PIN_8
#define BOARDID_2_GPIO_Port GPIOB
#define BOARDID_3_Pin GPIO_PIN_9
#define BOARDID_3_GPIO_Port GPIOB
#define NODEID_1BABY_Pin GPIO_PIN_0
#define NODEID_1BABY_GPIO_Port GPIOE
#define NODEID_4BABY_Pin GPIO_PIN_1
#define NODEID_4BABY_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
