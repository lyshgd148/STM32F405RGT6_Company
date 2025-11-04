/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;

SPI_HandleTypeDef hspi4;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//IWDG_HandleTypeDef hiwdg1;

//串口2
uint8_t U2RX=0;
uint8_t U2RxBuff[20];
uint8_t rx2Cnt=0;
uint8_t txLen=0;

////CANFD1
//FDCAN_TxHeaderTypeDef TxHeader;
//FDCAN_RxHeaderTypeDef RxHeader;
//uint8_t TxData[8];
//uint8_t RxData[8];
//void FDCAN1_Config(void);

//CANFD2
FDCAN_TxHeaderTypeDef TxHeader2;
FDCAN_RxHeaderTypeDef RxHeader2;
uint8_t TxData2[8];
uint8_t RxData2[8];
uint8_t RxData2Len=0;
void FDCAN2_Config(void);

Key_Tag skey;

all_state state = S0_IDLE;
uint8_t single_click_flag=0;
uint8_t double_click_flag=0;
uint8_t long_click_flag=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_SPI4_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//printf
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
PUTCHAR_PROTOTYPE
{
 HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
 return ch;
}
#endif

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_SPI4_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  //FDCAN1_Config();
  FDCAN2_Config();

  HAL_UART_Receive_IT(&huart2,&U2RX,1);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);

  HAL_GPIO_WritePin(CM4_EN_GPIO_Port, CM4_EN_Pin, 1);//0->1:restart

  DO4_OPEN;

  //***delete HAL_UART_Receive_IT(); __HAL_LOCK(huart)
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint16_t cnt = 0;
  FDCAN_ProtocolStatusTypeDef ProtocolStatus;

  while (1)
  {
	  HAL_Delay(10);

	  key_scan();

	  advance();

	  if(DI4_BUTTON==0)//按钮被按下
	  {
		  //System_Reset();//stm32h7软件复位函数
		  CM4_EN_OPEN;//CM4复位
		  HAL_Delay(10);
		  CM4_EN_CLOSE;
	  }

	  cnt++;
	  if(cnt>=10)
	  {
		  cnt=0;
		  HAL_FDCAN_GetProtocolStatus(&hfdcan2,&ProtocolStatus);
		  if(ProtocolStatus.BusOff==1)
		  {
			  MX_FDCAN2_Init();
			  FDCAN2_Config();
		  }
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 4;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_0;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_UART4
                              |RCC_PERIPHCLK_FDCAN|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_UART5|RCC_PERIPHCLK_SPI4;
  PeriphClkInitStruct.Spi45ClockSelection = RCC_SPI45CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  //娉㈢壒鐜囪绠楀叕锟�???????????????
  //BaudRate = CLK / ((DTSEG1+DTSEG2 + 1) * DBPR)
  //CLK=50MHz銆丏TSEG1=5銆丏TSEG2=4銆丏BPR=5銆丅audRate=1M
  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 5;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 5;
  hfdcan1.Init.NominalTimeSeg2 = 4;
  hfdcan1.Init.DataPrescaler = 5;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 5;
  hfdcan1.Init.DataTimeSeg2 = 4;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 10;
  hfdcan1.Init.ExtFiltersNbr = 10;
  hfdcan1.Init.RxFifo0ElmtsNbr = 1;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 1;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */


  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 50;
  hfdcan2.Init.NominalSyncJumpWidth = 1;
  hfdcan2.Init.NominalTimeSeg1 = 5;
  hfdcan2.Init.NominalTimeSeg2 = 4;
  hfdcan2.Init.DataPrescaler = 5;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 5;
  hfdcan2.Init.DataTimeSeg2 = 4;
  hfdcan2.Init.MessageRAMOffset = 0;
  hfdcan2.Init.StdFiltersNbr = 10;
  hfdcan2.Init.ExtFiltersNbr = 10;
  hfdcan2.Init.RxFifo0ElmtsNbr = 1;
  hfdcan2.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxFifo1ElmtsNbr = 0;
  hfdcan2.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxBuffersNbr = 0;
  hfdcan2.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.TxEventsNbr = 0;
  hfdcan2.Init.TxBuffersNbr = 0;
  hfdcan2.Init.TxFifoQueueElmtsNbr = 1;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan2.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_SLAVE;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 0x0;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi4.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi4.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi4.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi4.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi4.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */
  //鍛ㄦ湡锛滱PB1 Timer Clocks/Prescaler/Period
  //APB1 Timer Clocks=100MHz銆丳rescaler=10000銆丳eriod=5000銆佸懆锟�??=1/(100M/10000/100)=1ms
  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 10000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */
  //鍛ㄦ湡锛滱PB1 Timer Clocks/Prescaler/Period
  //APB1 Timer Clocks=100MHz銆丳rescaler=10000銆丳eriod=5000銆佸懆锟�????????????=1/(100M/10000/5000)=500ms
  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 10000-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 5000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart1, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DO2_Pin|DO3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DO4_Pin|DO1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DO_POW_Pin|CM4_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DO_IPC_GPIO_Port, DO_IPC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED2_Pin */
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DO2_Pin DO3_Pin DO_POW_Pin CM4_EN_Pin */
  GPIO_InitStruct.Pin = DO2_Pin|DO3_Pin|DO_POW_Pin|CM4_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DO4_Pin DO1_Pin */
  GPIO_InitStruct.Pin = DO4_Pin|DO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DI1_Pin DI2_Pin DI3_Pin DI_IPC_Pin
                           DI_ES_Pin */
  GPIO_InitStruct.Pin = DI1_Pin|DI2_Pin|DI3_Pin|DI_IPC_Pin
                          |DI_ES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : DI4_Pin */
  GPIO_InitStruct.Pin = DI4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DI4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_Pin */
  GPIO_InitStruct.Pin = VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DI_POW_Pin */
  GPIO_InitStruct.Pin = DI_POW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DI_POW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DO_IPC_Pin */
  GPIO_InitStruct.Pin = DO_IPC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DO_IPC_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
uint8_t control_buf[1];
void advance()
{
	//control_buf[0]=state;
	switch(state){

		case S0_IDLE:
			S0_idle_action();

			break;

		case S1_STOP:
			S1_stop_action();

			break;

		case S2_RUN:
			S2_run_action();

			break;

		case S3_ERROR:
			S3_error_action();

			break;

		default:
			break;
	}
}

uint8_t greenLed_twinkle=0;
uint8_t redLed_twinkle=0;
void S0_idle_action(void)//待机状�??
{
	POW_CLOSE;//48V断电

	allLed_close();//灯灭
	if(single_click_flag==1)
	{
		clear_allclick_flag();
		state=S1_STOP;
	}
//	else if(DI3_BUTTON==0)
//	{
//		//CM4复位
//	}
}

void S1_stop_action(void)//停止状�??
{
	POW_OPEN;//48V上电
	green_light();//绿灯�?
	if(single_click_flag==1)
	{
		clear_allclick_flag();
		state=S2_RUN;
	}
	if(long_click_flag==1)
	{
		clear_allclick_flag();
		state=S0_IDLE;
	}
//	if(double_click_flag==1)
//	{
//		double_click_flag=0;
//		LED2_T;
//		//示教模式
//		greenRed_twinkle();//红绿灯闪
//
//	}
}

void S2_run_action(void)//运行状�??
{
	green_twinkle();//绿灯�?
	if(STOP_KEY==0)//停止按钮按下
	{
		state=S1_STOP;
	}
//	else if()
//	{
//	    red_twinkle();红灯�?
//	    state=S1_STOP;
//	}
}

void S3_error_action(void)//错误状�??
{

}

void clear_allclick_flag()
{
	single_click_flag=0;
	double_click_flag=0;
	long_click_flag=0;
}

void green_light()//绿灯�?
{
	RUN_LED_OPEN;
	//STOP_LED_CLOSE;
	redLed_twinkle=0;
	greenLed_twinkle=0;
}

void red_light()//红灯�?
{
	RUN_LED_CLOSE;
	//STOP_LED_OPEN;
	redLed_twinkle=0;
	greenLed_twinkle=0;
}

void green_twinkle()//绿灯�?
{
	//STOP_LED_CLOSE;
	redLed_twinkle=0;
	greenLed_twinkle=1;
}

void red_twinkle()//红灯�?
{
	RUN_LED_CLOSE;
	redLed_twinkle=1;
	greenLed_twinkle=0;
}

void greenRed_twinkle()//红绿灯闪
{
	redLed_twinkle=1;
	greenLed_twinkle=1;
}

void allLed_close()//灯灭
{
	RUN_LED_CLOSE;
	//STOP_LED_CLOSE;
	redLed_twinkle=0;
	greenLed_twinkle=0;
}


void decode(uint8_t *data,uint8_t len)
{
	if(len!=4)
	{
		return;
	}
	if(data[0]!=0)
	{
		RUN_LED_CLOSE;	//240308 add
		STOP_LED_OPEN;//红灯亮代表错�?
	}
	else
	{
		RUN_LED_OPEN;	//240308 add
		STOP_LED_CLOSE;
	}

	CAN2_MESSAGE_TX(&data[1]);//取地�?�?后发8个字�?
}
//串口2中断回调函数
uint8_t flag80=0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//DO3_T;
	if(huart->Instance == huart2.Instance)
	{
		if(flag80)
		{
			flag80 = 0;
			switch(U2RX)
			{
				case 0x00:
					U2RxBuff[rx2Cnt++] = 0x80;
					break;

				case 0xFF:
					rx2Cnt=0;
					break;

				case 0xFE:
					//DO4_T;
					decode(U2RxBuff,rx2Cnt);
					rx2Cnt=0;
					break;

				default:
					break;
			}
		}
		else if(U2RX==0x80)
		{
			flag80 = 1;
		}
		else// if(wait_end)
		{
			U2RxBuff[rx2Cnt++] = U2RX;
			if(rx2Cnt>15) rx2Cnt=0;
		}
	}
	HAL_UART_Receive_IT(&huart2, &U2RX, 1);
}

////FDCAN1配置函数
//void FDCAN1_Config(void)
//{
//  FDCAN_FilterTypeDef sFilterConfig;
//
//  /* Configure Rx filter 鎺ユ敹璁剧疆锛岃繖閲屾寚瀹氭帴鏀剁殑CANID*/
//  sFilterConfig.IdType = FDCAN_EXTENDED_ID;
//  sFilterConfig.FilterIndex = 0;
//  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
//  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
//  sFilterConfig.FilterID1 = 20000;
//  sFilterConfig.FilterID2 = 0xFF;
//  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* Configure global filter:
//     Filter all remote frames with STD and EXT ID
//     Reject non matching frames with STD ID and EXT ID */
//  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /* Start the FDCAN module */
//  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}

//FDCAN2配置函数
void FDCAN2_Config(void)
{
  FDCAN_FilterTypeDef sFilterConfig;

  /* Configure Rx filter 鎺ユ敹璁剧疆锛岃繖閲屾寚瀹氭帴鏀剁殑CANID*/
  sFilterConfig.IdType = FDCAN_EXTENDED_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 20000;
  sFilterConfig.FilterID2 = 0xFF;
  if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* Configure global filter:
     Filter all remote frames with STD and EXT ID
     Reject non matching frames with STD ID and EXT ID */
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

  /* Start the FDCAN module */
  if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

////CANFD1 send
//void CAN_MESSAGE_TX(uint8_t *_DataBuf)
//{
//	TxHeader.Identifier = 20000;
//	TxHeader.IdType = FDCAN_EXTENDED_ID;
//	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
//	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
//	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
//	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
//	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
//	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
//	TxHeader.MessageMarker = 0;
//
//	if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, _DataBuf) != HAL_OK)
//	{
//		LED2_T;
//		//Error_Handler();
//	}
//}

//CANFD2 send
void CAN2_MESSAGE_TX(uint8_t *_DataBuf2)
{
	TxHeader2.Identifier = 20000;
	TxHeader2.IdType = FDCAN_EXTENDED_ID;
	TxHeader2.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader2.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader2.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader2.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader2.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader2.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader2.MessageMarker = 0;

	if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader2, _DataBuf2) != HAL_OK)
	{
		//LED2_T;
		//Error_Handler();
	}

}
//CAN中断接收回调函数
uint8_t head_buf[2]={0x80,0xFF};
uint8_t end_buf[2]={0x80,0xFE};
uint8_t special_buf[2]={0x80,0x00};
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
//	  if(hfdcan == &hfdcan1)//CANFD1中断
//	  {
//		  HAL_FDCAN_GetRxMessage(&hfdcan1,FDCAN_RX_FIFO0,&RxHeader,RxData);
//
//		  HAL_UART_Transmit(&huart2, (uint8_t *)RxData,2,0xFFFF);
//
//		  HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);
//	  }
	  if(hfdcan == &hfdcan2)//CANFD2中断
	  {
		  HAL_FDCAN_GetRxMessage(&hfdcan2,FDCAN_RX_FIFO0,&RxHeader2,RxData2);

		  HAL_UART_Transmit(&huart2, (uint8_t *)head_buf,2,0xFFFF);//

		  if(RUN_KEY == 0)				//240308 add KEY1
			  control_buf[0] &= 0xFE;
		  else
			  control_buf[0] |= 0x01;

		  if(STOP_KEY == 0)				//240308 add KEY2
			  control_buf[0] &= 0xFD;
		  else
			  control_buf[0] |= 0x02;

		  if(DI3_BUTTON == 0)			//240308 add KEY3
			  control_buf[0] &= 0xFB;
		  else
			  control_buf[0] |= 0x04;

		  if(DI4_BUTTON == 0)			//240308 add KEY4
			  control_buf[0] &= 0xF7;
		  else
			  control_buf[0] |= 0x08;

		  if(DI_POW == 0)				//240308 add POW
			  control_buf[0] &= 0xEF;
		  else
			  control_buf[0] |= 0x10;

		  if(DI_IPC == 0)				//240308 add IPC/POW2
			  control_buf[0] &= 0xDF;
		  else
			  control_buf[0] |= 0x20;

		  if(DI_ES == 0)				//240308 add ES
			  control_buf[0] &= 0xBF;
		  else
			  control_buf[0] |= 0x40;

		  HAL_UART_Transmit(&huart2, (uint8_t *)control_buf,1,0xFFFF);//状�??
		  for(int i=0;i<3;i++)
		  {
			  if(RxData2[i]==0x80)
			  {
				  HAL_UART_Transmit(&huart2, (uint8_t *)special_buf,2,0xFFFF);
			  }
			  else
			  {
				  HAL_UART_Transmit(&huart2, (uint8_t *)&RxData2[i],1,0xFFFF);
			  }
		  }
		  HAL_UART_Transmit(&huart2, (uint8_t *)end_buf,2,0xFFFF);

		  //HAL_UART_Transmit(&huart2, (uint8_t *)RxData2,3,0xFFFF);

		  HAL_FDCAN_ActivateNotification(&hfdcan2,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);
	  }
  }

}


//定时器中断回调函�?
uint32_t tim3cnt=0;
uint32_t tim3cnt_greenLed=0;
uint32_t tim3cnt_redLed=0;
//uint8_t temp_buf[8]={0};
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim4)
	{
		LED1_T;
	}
	if(htim == &htim3)
	{
		//temp_buf[0]++;
		//CAN2_MESSAGE_TX(temp_buf);
		tim3cnt++;
		if(tim3cnt==500)
		{
			//DO4_T;
			tim3cnt=0;
		}

		if(greenLed_twinkle==1)//绿灯�?
		{
			tim3cnt_greenLed++;
			if(tim3cnt_greenLed==300)
			{
				RUN_LED_T;
				tim3cnt_greenLed=0;
			}
		}

		if(redLed_twinkle==1)//红灯�?
		{
			tim3cnt_redLed++;
			if(tim3cnt_redLed==300)
			{
				STOP_LED_T;
				tim3cnt_redLed=0;
			}
		}

		skey.u32time1++;
		skey.u32time2++;
	}
}

void key_scan()
{
    static uint8_t press = 0;

    if(RUN_KEY==0)//按钮按下
    {
    	HAL_Delay(10);
        if(RUN_KEY==0)
        {
            if(skey.u8key_flag==0)
            {
                skey.u8key_flag=1;
                skey.u32time1=0;
            }
            else if(skey.u8key_flag==1)
            {
                if(!press && skey.u32time1 > 3000)
                {
                    press = 1;
                    long_click_flag=1;
                    //DO4_T;
                    //return LONG_PRES;//长按
                }
            }
        }
    }
    else if(RUN_KEY==1)
    {
        if(skey.u8key_flag==1)
        {
            skey.u8key_flag=0;
            if(skey.u32time1>3000)
            {
                press = 0;
                skey.u32time1 = 0;
                skey.u32time2 = 0;
                skey.u8key_flag=0;
                skey.u8key_double_flag=0;
            }
            else if(skey.u8key_double_flag==0)
            {
                skey.u8key_double_flag=1;
                skey.u32time2=0;
            }
            else if(skey.u8key_double_flag==1)
            {
                if(skey.u32time2<500)
                {
                    skey.u8key_double_flag=0;
                    double_click_flag=1;
                    //DO4_T;
                    //return DOUBLE_PRES;//双击
                }
            }
        }
        else if(skey.u8key_double_flag==1)
        {
            if(skey.u32time2>=500)
            {
                skey.u8key_double_flag=0;
                single_click_flag=1;
                //DO4_T;
                //return single_PRES;//单击
            }
        }
    }
    //return 0;//无按�?
}

//软件复位函数
void System_Reset(void)
{
	__set_FAULTMASK(1); //关闭所有中断
	NVIC_SystemReset(); //进行软件复位
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
