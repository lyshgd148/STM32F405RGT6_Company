/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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
#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_2
#define LED2_GPIO_Port GPIOE
#define DO2_Pin GPIO_PIN_4
#define DO2_GPIO_Port GPIOB
#define DO3_Pin GPIO_PIN_3
#define DO3_GPIO_Port GPIOB
#define DO4_Pin GPIO_PIN_15
#define DO4_GPIO_Port GPIOA
#define DO1_Pin GPIO_PIN_11
#define DO1_GPIO_Port GPIOA
#define DI1_Pin GPIO_PIN_7
#define DI1_GPIO_Port GPIOD
#define DI2_Pin GPIO_PIN_4
#define DI2_GPIO_Port GPIOD
#define DI3_Pin GPIO_PIN_0
#define DI3_GPIO_Port GPIOD
#define DI4_Pin GPIO_PIN_8
#define DI4_GPIO_Port GPIOA
#define VBUS_Pin GPIO_PIN_0
#define VBUS_GPIO_Port GPIOC
#define DO_POW_Pin GPIO_PIN_10
#define DO_POW_GPIO_Port GPIOB
#define DI_POW_Pin GPIO_PIN_13
#define DI_POW_GPIO_Port GPIOB
#define DI_IPC_Pin GPIO_PIN_9
#define DI_IPC_GPIO_Port GPIOD
#define DO_IPC_Pin GPIO_PIN_13
#define DO_IPC_GPIO_Port GPIOD
#define CM4_EN_Pin GPIO_PIN_11
#define CM4_EN_GPIO_Port GPIOB
#define DI_ES_Pin GPIO_PIN_12
#define DI_ES_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

#define LED1_S HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0)
#define LED1_R HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1)
#define LED1_T HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin)
#define LED2_S HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0)
#define LED2_R HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1)
#define LED2_T HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin)

#define RUN_KEY      HAL_GPIO_ReadPin(DI1_GPIO_Port,DI1_Pin)//X1-运行按钮
#define STOP_KEY     HAL_GPIO_ReadPin(DI2_GPIO_Port,DI2_Pin)//X2-停止按钮
#define DI3_BUTTON   HAL_GPIO_ReadPin(DI3_GPIO_Port,DI3_Pin)//X3
#define DI4_BUTTON   HAL_GPIO_ReadPin(DI4_GPIO_Port,DI4_Pin)//X4-CM4复位按钮
#define DI_POW       HAL_GPIO_ReadPin(DI_POW_GPIO_Port,DI_POW_Pin)//POW
#define DI_IPC       HAL_GPIO_ReadPin(DI_IPC_GPIO_Port,DI_IPC_Pin)//IPC
#define DI_ES        HAL_GPIO_ReadPin(DI_ES_GPIO_Port,DI_ES_Pin)  //ES

#define RUN_LED_OPEN   HAL_GPIO_WritePin(DO1_GPIO_Port, DO1_Pin, 0)//Y1-运行绿灯
#define RUN_LED_CLOSE  HAL_GPIO_WritePin(DO1_GPIO_Port, DO1_Pin, 1)
#define RUN_LED_T      HAL_GPIO_TogglePin(DO1_GPIO_Port, DO1_Pin)

#define STOP_LED_OPEN  HAL_GPIO_WritePin(DO2_GPIO_Port, DO2_Pin, 0)//Y2-停止红灯
#define STOP_LED_CLOSE HAL_GPIO_WritePin(DO2_GPIO_Port, DO2_Pin, 1)
#define STOP_LED_T      HAL_GPIO_TogglePin(DO2_GPIO_Port, DO2_Pin)

#define DO3_OPEN   HAL_GPIO_WritePin(DO3_GPIO_Port, DO3_Pin, 0)//Y3
#define DO3_CLOSE  HAL_GPIO_WritePin(DO3_GPIO_Port, DO3_Pin, 1)
#define DO3_T      HAL_GPIO_TogglePin(DO3_GPIO_Port, DO3_Pin)

#define DO4_OPEN   HAL_GPIO_WritePin(DO4_GPIO_Port, DO4_Pin, 0)//Y4-stm32h7复位
#define DO4_CLOSE  HAL_GPIO_WritePin(DO4_GPIO_Port, DO4_Pin, 1)
#define DO4_T      HAL_GPIO_TogglePin(DO4_GPIO_Port, DO4_Pin)

#define CM4_EN_OPEN  HAL_GPIO_WritePin(CM4_EN_GPIO_Port, CM4_EN_Pin, 0)//CM4复位
#define CM4_EN_CLOSE  HAL_GPIO_WritePin(CM4_EN_GPIO_Port, CM4_EN_Pin, 1)

#define POW_OPEN  HAL_GPIO_WritePin(DO_POW_GPIO_Port, DO_POW_Pin, 1)
#define POW_CLOSE HAL_GPIO_WritePin(DO_POW_GPIO_Port, DO_POW_Pin, 0)
#define IPC_OPEN  HAL_GPIO_WritePin(DO_IPC_GPIO_Port, DO_IPC_Pin, 1)
#define IPC_CLOSE HAL_GPIO_WritePin(DO_IPC_GPIO_Port, DO_IPC_Pin, 0)


typedef enum all_state
{
	S0_IDLE=1, //待机状�??
	S1_STOP=2, //停止状�??
	S2_RUN=4,  //运行状�??
	S3_ERROR=8 //错误状�??
}all_state;


void advance(void);
void S0_idle_action(void);
void S1_stop_action(void);
void S2_run_action(void);
void S3_error_action(void);
void green_light(void);//绿灯�?
void red_light(void);//红灯�?
void green_twinkle(void);//绿灯�?
void red_twinkle(void);//红灯�?
void greenRed_twinkle(void);//红绿灯闪
void allLed_close(void);//灯灭

#define single_PRES 1	//单击
#define LONG_PRES	2	//长按
#define DOUBLE_PRES	3	//双击

typedef struct
{
  uint32_t u32time1;            //第一次按下后�????始计时，主要用于判断长按，放在定时器中自�????
  uint32_t u32time2;            //第一次松手后�????始计时，用于判断双击或单击，放在定时器中自加
  uint8_t u8key_flag;           //第一次按下标�????
  uint8_t u8key_double_flag;    //第二次标�????
}Key_Tag;

extern Key_Tag skey;

void key_scan(void);
void clear_allclick_flag(void);
void System_Reset(void);//软件复位函数

//void CAN_MESSAGE_TX(uint8_t *_DataBuf);
//void CAN_MESSAGE_RX(void);
void CAN2_MESSAGE_TX(uint8_t *_DataBuf2);
void CAN2_MESSAGE_RX(void);

//void io_in_data(uint8_t *I_buf);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
