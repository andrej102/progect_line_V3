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
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32h7xx_hal.h"

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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define MUX1_Pin GPIO_PIN_0
#define MUX1_GPIO_Port GPIOF
#define MUX2_Pin GPIO_PIN_1
#define MUX2_GPIO_Port GPIOF
#define MUX3_Pin GPIO_PIN_2
#define MUX3_GPIO_Port GPIOF
#define MUX4_Pin GPIO_PIN_3
#define MUX4_GPIO_Port GPIOF
#define MUX5_Pin GPIO_PIN_4
#define MUX5_GPIO_Port GPIOF
#define MUX6_Pin GPIO_PIN_5
#define MUX6_GPIO_Port GPIOF
#define MUX7_Pin GPIO_PIN_6
#define MUX7_GPIO_Port GPIOF
#define MUX8_Pin GPIO_PIN_7
#define MUX8_GPIO_Port GPIOF
#define S0_Pin GPIO_PIN_8
#define S0_GPIO_Port GPIOF
#define S1_Pin GPIO_PIN_9
#define S1_GPIO_Port GPIOF
#define S2_Pin GPIO_PIN_10
#define S2_GPIO_Port GPIOF
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define RMII_MDC_Pin GPIO_PIN_1
#define RMII_MDC_GPIO_Port GPIOC
#define RMII_REF_CLK_Pin GPIO_PIN_1
#define RMII_REF_CLK_GPIO_Port GPIOA
#define RMII_MDIO_Pin GPIO_PIN_2
#define RMII_MDIO_GPIO_Port GPIOA
#define RMII_CRS_DV_Pin GPIO_PIN_7
#define RMII_CRS_DV_GPIO_Port GPIOA
#define RMII_RXD0_Pin GPIO_PIN_4
#define RMII_RXD0_GPIO_Port GPIOC
#define RMII_RXD1_Pin GPIO_PIN_5
#define RMII_RXD1_GPIO_Port GPIOC
#define LED_GREEN_Pin GPIO_PIN_0
#define LED_GREEN_GPIO_Port GPIOB
#define LIGHT_CONTROL_Pin GPIO_PIN_1
#define LIGHT_CONTROL_GPIO_Port GPIOB
#define SWITCH_BOX_Pin GPIO_PIN_2
#define SWITCH_BOX_GPIO_Port GPIOB
#define S3_Pin GPIO_PIN_11
#define S3_GPIO_Port GPIOF
#define ROW2_Pin GPIO_PIN_10
#define ROW2_GPIO_Port GPIOB
#define ROW3_Pin GPIO_PIN_11
#define ROW3_GPIO_Port GPIOB
#define ROW4_Pin GPIO_PIN_12
#define ROW4_GPIO_Port GPIOB
#define RMII_TXD1_Pin GPIO_PIN_13
#define RMII_TXD1_GPIO_Port GPIOB
#define LED_RED_Pin GPIO_PIN_14
#define LED_RED_GPIO_Port GPIOB
#define STLK_VCP_RX_Pin GPIO_PIN_8
#define STLK_VCP_RX_GPIO_Port GPIOD
#define STLK_VCP_TX_Pin GPIO_PIN_9
#define STLK_VCP_TX_GPIO_Port GPIOD
#define USB_FS_PWR_EN_Pin GPIO_PIN_10
#define USB_FS_PWR_EN_GPIO_Port GPIOD
#define USB_FS_OVCR_Pin GPIO_PIN_7
#define USB_FS_OVCR_GPIO_Port GPIOG
#define USB_FS_VBUS_Pin GPIO_PIN_9
#define USB_FS_VBUS_GPIO_Port GPIOA
#define USB_FS_ID_Pin GPIO_PIN_10
#define USB_FS_ID_GPIO_Port GPIOA
#define USB_FS_DM_Pin GPIO_PIN_11
#define USB_FS_DM_GPIO_Port GPIOA
#define USB_FS_DP_Pin GPIO_PIN_12
#define USB_FS_DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define DATA0_Pin GPIO_PIN_0
#define DATA0_GPIO_Port GPIOD
#define DATA1_Pin GPIO_PIN_1
#define DATA1_GPIO_Port GPIOD
#define DATA2_Pin GPIO_PIN_2
#define DATA2_GPIO_Port GPIOD
#define DATA3_Pin GPIO_PIN_3
#define DATA3_GPIO_Port GPIOD
#define RS_Pin GPIO_PIN_4
#define RS_GPIO_Port GPIOD
#define RW_Pin GPIO_PIN_5
#define RW_GPIO_Port GPIOD
#define RMII_TX_EN_Pin GPIO_PIN_11
#define RMII_TX_EN_GPIO_Port GPIOG
#define RMII_TXD0_Pin GPIO_PIN_13
#define RMII_TXD0_GPIO_Port GPIOG
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define COLUMN1_Pin GPIO_PIN_5
#define COLUMN1_GPIO_Port GPIOB
#define COLUMN2_Pin GPIO_PIN_6
#define COLUMN2_GPIO_Port GPIOB
#define COLUMN3_Pin GPIO_PIN_7
#define COLUMN3_GPIO_Port GPIOB
#define COLUMN4_Pin GPIO_PIN_8
#define COLUMN4_GPIO_Port GPIOB
#define ROW1_Pin GPIO_PIN_9
#define ROW1_GPIO_Port GPIOB
#define LED_YELLOW_Pin GPIO_PIN_1
#define LED_YELLOW_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
