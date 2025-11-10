/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    adc.c
 * @brief   This file provides code for the configuration
 *          of the ADC instances.
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
/* Includes ------------------------------------------------------------------*/
#include "adc.h"

/* USER CODE BEGIN 0 */
#include "stdio.h"
#include <stdbool.h>

#include "tim.h"
#include "motor_runtime_param.h"
#include "foc.h"
#include "filter.h"
#include "global_def.h"
#include "arm_math.h"
#include "usart.h"
struct Frame
{
  float data[2];
  unsigned char tail[4];
};

struct Frame frame = {.tail = {0x00, 0x00, 0x80, 0x7f}};
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC2)
  {
    static int32_t calculate_num = 0;
    static float calculate_u = 0;
    static float calculate_v = 0;
    static bool run_state = false;
    if (calculate_num < 1024)
    {
      calculate_num++;
      calculate_u += (float)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
      calculate_v += (float)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);
    }
    else if (calculate_num == 1024)
    {
      calculate_u = calculate_u / 1024.0f;
      calculate_v = calculate_v / 1024.0f;
      run_state = true;

      calculate_num = 1025;

      HAL_GPIO_WritePin(SD1_GPIO_Port, SD1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(SD2_GPIO_Port, SD2_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(SD3_GPIO_Port, SD3_Pin, GPIO_PIN_SET);
    }

    if (run_state)
    {
      uint32_t raw_u = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
      uint32_t raw_v = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);

      float u_u = ADC_REFERENCE_VOLT * ((float)(raw_u - calculate_u) / ((1 << ADC_BITS) - 1));
      float u_w = ADC_REFERENCE_VOLT * ((float)(raw_v - calculate_v) / ((1 << ADC_BITS) - 1));
      motor_i_u = u_u / R_SHUNT / OP_GAIN;
      motor_i_w = u_w / R_SHUNT / OP_GAIN;

      float i_alpha = 0;
      float i_beta = 0;
      arm_clarke_f32(motor_i_u, -(motor_i_w + motor_i_u), &i_alpha, &i_beta);
      float sin_value = arm_sin_f32(rotor_logic_angle);
      float cos_value = arm_cos_f32(rotor_logic_angle);
      float _motor_i_d = 0;
      float _motor_i_q = 0;
      arm_park_f32(i_alpha, i_beta, &_motor_i_d, &_motor_i_q, sin_value, cos_value);
      float filter_alpha_i_d = 0.1;
      float filter_alpha_i_q = 0.1;
      motor_i_d = LFP(_motor_i_d, motor_i_d, filter_alpha_i_d);
      motor_i_q = LFP(_motor_i_q, motor_i_q, filter_alpha_i_q);

      switch (motor_control_context.type)
      {
      case control_type_position:
        lib_position_control(motor_control_context.position);
        break;
      case control_type_speed:
        lib_speed_control(motor_control_context.speed);
        break;
      case control_type_torque:
        lib_torque_control(motor_control_context.torque_norm_d, motor_control_context.torque_norm_q);
        break;
      case control_type_position_speed_torque:
        lib_position_speed_torque_control(motor_control_context.position, motor_control_context.max_speed, motor_control_context.max_torque_norm);
        break;
      default:
        break;
      }

      // frame.data[0] = motor_i_d;
      // frame.data[1] = motor_i_q / MAX_CURRENT;
      frame.data[0] = encoder_angle;
      frame.data[1] = rotor_logic_angle;
      // frame.data[4] = motor_i_u;
      // frame.data[5] = motor_i_w;
      HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&frame, sizeof(frame));
    }
  }
}
// 规则组ADC转换完成回调
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC2)
  {
    // motor_control_context.torque_norm_q = Knob_buf / 4095.0f * 0.8f;
    // motor_control_context.position = Knob_buf / 4095.0f * deg2rad(90);
  }
}
/* USER CODE END 0 */

ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc2;

/* ADC2 init function */
void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_InjectionConfTypeDef sConfigInjected = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
   */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
   */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_3;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 2;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
   */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_17;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if (adcHandle->Instance == ADC2)
  {
    /* USER CODE BEGIN ADC2_MspInit 0 */

    /* USER CODE END ADC2_MspInit 0 */

    /** Initializes the peripherals clocks
     */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
    PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* ADC2 clock enable */
    __HAL_RCC_ADC12_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**ADC2 GPIO Configuration
    PA4     ------> ADC2_IN17
    PA6     ------> ADC2_IN3
    PB2     ------> ADC2_IN12
    */
    GPIO_InitStruct.Pin = IC_Pin | IA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = Knob_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(Knob_GPIO_Port, &GPIO_InitStruct);

    /* ADC2 DMA Init */
    /* ADC2 Init */
    hdma_adc2.Instance = DMA1_Channel5;
    hdma_adc2.Init.Request = DMA_REQUEST_ADC2;
    hdma_adc2.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc2.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc2.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc2.Init.Mode = DMA_NORMAL;
    hdma_adc2.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_adc2) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(adcHandle, DMA_Handle, hdma_adc2);

    /* ADC2 interrupt Init */
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
    /* USER CODE BEGIN ADC2_MspInit 1 */

    /* USER CODE END ADC2_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef *adcHandle)
{

  if (adcHandle->Instance == ADC2)
  {
    /* USER CODE BEGIN ADC2_MspDeInit 0 */

    /* USER CODE END ADC2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC12_CLK_DISABLE();

    /**ADC2 GPIO Configuration
    PA4     ------> ADC2_IN17
    PA6     ------> ADC2_IN3
    PB2     ------> ADC2_IN12
    */
    HAL_GPIO_DeInit(GPIOA, IC_Pin | IA_Pin);

    HAL_GPIO_DeInit(Knob_GPIO_Port, Knob_Pin);

    /* ADC2 DMA DeInit */
    HAL_DMA_DeInit(adcHandle->DMA_Handle);

    /* ADC2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(ADC1_2_IRQn);
    /* USER CODE BEGIN ADC2_MspDeInit 1 */

    /* USER CODE END ADC2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
