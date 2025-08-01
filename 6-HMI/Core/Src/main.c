/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "Screen.h"
#include "hmi_driver.h"
#include "cmd_queue.h"
#include "cmd_process.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#if dbg
	uint8_t debug=1;
	uint8_t firstFlag_dgb=0;
	uint8_t secondFlag_dgb=0;
	uint8_t state_dbg=0;        //0 ״̬�ɱ䶯  1 ��dgb1 2 3 4 5 
	uint8_t goHomeFlag_dbg=0;
	int32_t heigh_dbg;
#endif
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
	queue_reset();

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_TIM3_Init();
	MX_TIM6_Init();
	MX_CAN1_Init();
	MX_USART2_UART_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */
	CAN1_Config();

	HAL_UART_Receive_IT(&huart2, (uint8_t *)cmd_buffer, CMD_MAX_SIZE);
	HAL_TIM_Base_Start_IT(&htim2);

	Motor_Init();
	HAL_Delay(1000); 			//��ʼ�������ʱ�����
	/* USER CODE END 2 */

	/* Infinite loop */

	/* USER CODE BEGIN WHILE */
	SetButtonValue(0, 2, 0);
	MySetIcon(0,3,0);
	HAL_Delay(100);
	MySetIcon(0,20,0);
	MySetIcon(0,4,0);
	motor_AllGoHome();
	HAL_GPIO_WritePin(Y6_GPIO_Port, Y6_Pin, GPIO_PIN_RESET); 
	


	while (1)
	{
	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */
		HAL_Delay(1000);
		while(updateFlag!=1)
		{
			#if dbg
				if(state_dbg==1 || state_dbg==3)
				{
					MoveFifthMotor(1000, 100,heigh_dbg*16384/lead_screw);
					while(motor_statuses[4].is_reach ==0)
					{
							HAL_Delay(100);
					}
					motor_statuses[4].is_reach =0;
					state_dbg=0;
				}
				else if(state_dbg==2)
				{
					motorGoHome(1);
					motorGoHome(2);
					while(motor_statuses[0].is_reach ==0 && motor_statuses[1].is_reach ==0)
					{
						HAL_Delay(100);
					}
					motor_statuses[0].is_reach =0; 
					motor_statuses[1].is_reach =0; 
					
					motorGoHome(3);
					motorGoHome(4);
					motorGoHome(5);
					while(motor_statuses[2].is_reach ==0 && motor_statuses[3].is_reach ==0 && motor_statuses[4].is_reach ==0)
					{
						HAL_Delay(100);
					}
					motor_statuses[2].is_reach =0;
					motor_statuses[3].is_reach =0;
					motor_statuses[4].is_reach =0;
					
					firstFlag_dgb=0;
					secondFlag_dgb=0;
					heigh_dbg=0;
					state_dbg=0;
				}
				else if(state_dbg==4)
				{
					MoveFirstGMotors(firstFlag_dgb%2,500,50);
					while(motor_statuses[0].is_reach ==0 && motor_statuses[1].is_reach ==0)
					{
							HAL_Delay(100);
					}
					motor_statuses[0].is_reach =0;
					motor_statuses[1].is_reach =0;
					firstFlag_dgb++;
					state_dbg=0;
				}
				else if(state_dbg==5)
				{
					MoveSecondGMotors(secondFlag_dgb%2,500,50);					
					while(motor_statuses[2].is_reach ==0 && motor_statuses[3].is_reach ==0)
					{
						HAL_Delay(100);
					}
					motor_statuses[2].is_reach =0; 
					motor_statuses[3].is_reach =0;
					secondFlag_dgb++;
					state_dbg=0;
				}	
			#endif
			HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
			HAL_Delay(100);
		}
		#if dbg
			debug=0;
		#endif
		MyGetTextValue();
		HAL_Delay(500);
		
		Sys_Run(tray_num-1);
		HAL_Delay(2500);
		if(tray_num != 1)
			motor_AllGoHome();
		
		SetButtonValue(0, 2, 0);
		MySetIcon(0,4,0);
		MySetIcon(0,3,0);
		MySetIcon(0,20,0);
		
		updateFlag=0;
		#if dbg
			debug=1;
		#endif
		HAL_Delay(1500);
		
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
#ifdef USE_FULL_ASSERT
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
