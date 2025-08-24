#include <string.h>
#include <stdint.h>
#include "hmi_driver.h"
#include "Screen.h"
#include "motor.h"
#include "can.h"
#include "debug.h"

MotorStatus motor_statuses[motor_num];

uint8_t inputs[motor_num] = {0, 0, 0, 0, 0}; // 按键状态数组

int8_t tray_num = 1;		// 起始层数：5-1=4
int16_t tray_offset = 0;	// 第一个料盘的偏置位置 mm
int16_t PutDown_offset = 0; // 放第一个料盘的偏置位置
int16_t heigh = 0;			// 层高 mm

uint8_t updateFlag = 0; // 一轮开始标志 1开始

int32_t tray_position[Max_tray_num];	// 料盘位置
int32_t PutDown_position[Max_tray_num]; // 摆放料盘的位置

// uint8_t statrt_layer = 3; // 料盘起始层

uint8_t sys_state = 0;	 // 系统状态机
uint8_t stop_state = 1;	 // 急停状态机 0:不停机 1:停机
uint8_t home_button = 0; // 归零按钮 1:按钮归零

int64_t fifthPosition;

void Motor_Init(void)
{
	uint8_t i;

	for (i = 0; i < motor_num; i++)
	{
		motor_statuses[i].id = i + 1;
		motor_statuses[i].is_reach = 0;
	}
}

void motorGoHome(uint8_t slaveAddr)
{
	txBuffer[0] = 0x91;
	CAN1_Send_Msg(txBuffer, 2, slaveAddr);
}

void motor_AllGoHome(void) /*要小改一下*/
{
	uint32_t tick = 0;
#if dbg
	goHomeFlag_dbg = 0;
#endif
	/*----------------------------------------------------------------------------------*/
	LED_Yellow();
	motorGoHome(1);
	motorGoHome(2);
	while (motor_statuses[0].is_reach == 0 || motor_statuses[1].is_reach == 0)
	{
		HAL_Delay(100);
		tick++;
		if (tick >= 1000)
		{
			break;
		}
	}
	if (motor_statuses[0].is_reach == 0 || motor_statuses[1].is_reach == 0)
	{
		/*错误处理*/
	}
	else
	{
		tick = 0;
		motor_statuses[0].is_reach = 0;
		motor_statuses[1].is_reach = 0;
	}
	HAL_Delay(500);
	/*----------------------------------------------------------------------------------*/
	motorGoHome(3);
	motorGoHome(4);
	while (motor_statuses[2].is_reach == 0 || motor_statuses[3].is_reach == 0)
	{
		HAL_Delay(100);
		tick++;
		if (tick >= 1000)
		{
			break;
		}
	}
	if (motor_statuses[2].is_reach == 0 || motor_statuses[3].is_reach == 0)
	{
		/*错误处理*/
	}
	else
	{
		tick = 0;
		motor_statuses[2].is_reach = 0;
		motor_statuses[3].is_reach = 0;
	}
	HAL_Delay(500);
	/*----------------------------------------------------------------------------------*/
	motorGoHome(5);
	while (motor_statuses[4].is_reach == 0)
	{
		HAL_Delay(100);
		tick++;
		if (tick >= 1000)
		{
			break;
		}
	}
	if (motor_statuses[4].is_reach == 0)
	{
		/*错误处理*/
	}
	else
	{
		tick = 0;
		motor_statuses[4].is_reach = 0;
		sys_state = 1;
	}
	/*-----------------------------------------------------*/
	motorGoPosition(5, 1000, 100, 8192);
	while (motor_statuses[4].is_reach == 0)
	{
		HAL_Delay(100);
		tick++;
		if (tick >= 50000)
		{
			break;
		}
	}
	if (motor_statuses[4].is_reach == 0)
	{
		/*错误处理*/
	}
	else
	{
		tick = 0;
		motor_statuses[4].is_reach = 0;
		sys_state = 1;
	}
	LED_Green();
#if dbg
	goHomeFlag_dbg = 1;
	heigh_dbg = 8192 / 16384;
	SetTextFloat(2, 22, (float)(heigh_dbg / 10), 1, 2);
#endif
}

void motorGoPosition(uint8_t slaveAddr, uint16_t speed, uint8_t acc, int32_t pos)
{
	txBuffer[0] = 0xF5; // FD:相对脉冲(16细分*200),FE:绝对脉冲
						// F4:相对坐标,F5:绝对坐标(16384/r)

	txBuffer[1] = (speed >> 8) & 0x00FF; //
	txBuffer[2] = speed & 0x00FF;
	txBuffer[3] = acc;
	txBuffer[4] = (pos >> 16) & 0xFF; // bit23 - bit16
	txBuffer[5] = (pos >> 8) & 0xFF;  // bit15 - bit8
	txBuffer[6] = (pos >> 0) & 0xFF;  // bit7 - bit0
	CAN1_Send_Msg(txBuffer, 8, slaveAddr);
}

void motorReadPosition(uint8_t slaveAddr)
{
	txBuffer[0] = 0x31;
	CAN1_Send_Msg(txBuffer, 2, slaveAddr);
}

void motor_read(void)
{
	if (HAL_GPIO_ReadPin(X1_GPIO_Port, X1_Pin) == 0)
		inputs[0] = 1;
	if (HAL_GPIO_ReadPin(X2_GPIO_Port, X2_Pin) == 0)
		inputs[1] = 1;
}

void Tray_posInit(uint32_t tray_offset, uint32_t heigh) // 料盘位置初始化
{
	uint8_t i;
	int32_t t_off = tray_offset * 16384 / lead_screw;
	int32_t h = heigh * 16384 / lead_screw;

	for (i = tray_num; i > 1; i--)
	{
		tray_position[i - 1] = t_off + ((i - 2) * h);
	}
	tray_position[0] = (Max_Length * 16384 / lead_screw); // 顶层机器人取料点
}

void PutDown_posInit(uint32_t PutDown_offset, uint32_t heigh) // 料盘放置位置初始化
{
	uint8_t i;
	int32_t p_off = PutDown_offset * 16384 / lead_screw;
	int32_t h = heigh * 16384 / lead_screw;

	for (i = tray_num; i > 1; i--)
	{
		PutDown_position[i - 1] = p_off + ((tray_num - i) * h);
	}
	PutDown_position[0] = PutDown_position[1] + h / 4; // 顶层放料带点
}

void Position_Init(uint32_t tray_offset, uint32_t PutDown_offset, uint32_t heigh) // 取料放料位置初始化
{
	memset(tray_position, 0, sizeof(tray_position));
	memset(PutDown_position, 0, sizeof(PutDown_position));

	Tray_posInit(tray_offset, heigh);
	PutDown_posInit(PutDown_offset, heigh);
}

void MoveFirstGMotors(uint8_t state, uint16_t speed, uint8_t acc)
{
	if (state == 0) // 钩子张开
	{
		motorGoPosition(1, 1000, 150, -240000);
		motorGoPosition(2, 1000, 150, -240000);
	}
	else if (state == 1)
	{
		motorGoPosition(1, 1000, 100, 0);
		motorGoPosition(2, 1000, 100, 0);
	}
}

void MoveSecondGMotors(uint8_t state, uint16_t speed, uint8_t acc)
{
	if (state == 0) // 小臂前伸
	{
		motorGoPosition(3, speed, acc, 85500);
		motorGoPosition(4, speed, acc, -85500);
	}
	else if (state == 1)
	{
		motorGoPosition(3, speed, acc, 0);
		motorGoPosition(4, speed, acc, 0);
	}
}

void MoveFifthMotor(uint16_t speed, uint8_t acc, int32_t pos)
{
	motorGoPosition(5, speed, acc, pos);
}

uint8_t GetMaterial(uint8_t num) // 去取num盘料
{
	uint32_t tick = 0;
	uint8_t timeoutFlag = 0; // 超时标志
	while (stop_state != 0)
	{
		HAL_Delay(50);
	};

	/*-----------------------整体升降到料盘处-----------------------------*/
	while (motor_statuses[4].is_reach == 0)
	{
		MoveFifthMotor(2500, 180, tray_position[num]);
		while (motor_statuses[4].is_reach != 1)
		{
			tick += 1;
			if (tick >= 1000)
			{
				timeoutFlag = 1;
				break;
			}
			if (stop_state == 1 || updateFlag == 0)
			{
				// motor_Stop(5);
				motor_Sstop(5, 0);
				while (motor_statuses[4].is_reach == 0)
				{
					HAL_Delay(50);
				}
				motor_statuses[4].is_reach = 0;
				break;
			}
			HAL_Delay(50);
		};
		while (stop_state != 0)
		{
			if (updateFlag == 0)
				return 1;
			HAL_Delay(50);
		};
		tick = 0;
		if (motor_statuses[4].is_reach == 0 && timeoutFlag == 1)
		{
			timeoutFlag = 0;
			LED_Red();
			/*
				错误处理
			*/
		}
	}
	motor_statuses[4].is_reach = 0;

	/*--------------------------钩子钩住--------------------------*/
	tick = 0;
	while (motor_statuses[0].is_reach == 0 || motor_statuses[1].is_reach == 0)
	{
		MoveFirstGMotors(0, 1000, 100);
		while (motor_statuses[0].is_reach != 1 || motor_statuses[1].is_reach != 1)
		{

			tick += 1;
			if (tick >= 1000)
			{
				timeoutFlag = 1;
				break;
			}
			if (stop_state == 1 || updateFlag == 0)
			{
				motor_Stop(1);
				HAL_Delay(50);
				motor_Stop(2);
				HAL_Delay(450);
				break;
			}
			HAL_Delay(50);
		};
		while (stop_state == 1)
		{
			if (updateFlag == 0)
				return 1;
			HAL_Delay(50);
		};
		tick = 0;
		if ((motor_statuses[0].is_reach == 0 || motor_statuses[1].is_reach == 0) && timeoutFlag == 1)
		{
			timeoutFlag = 0;
			LED_Red();
			/*
			  错误处理
			*/
		}
	}
	motor_statuses[0].is_reach = 0;
	motor_statuses[1].is_reach = 0;

	/*---------------------------整体上升到机器人工作处-------------------------*/
	tick = 0;
	while (motor_statuses[4].is_reach == 0)
	{
		MoveFifthMotor(2500, 180, tray_position[0]);
		while (motor_statuses[4].is_reach != 1)
		{

			tick += 1;
			if (tick >= 1000)
			{
				timeoutFlag = 1;
				break;
			}
			if (stop_state == 1 || updateFlag == 0)
			{
				// motor_Stop(5);
				motor_Sstop(5, 0);
				while (motor_statuses[4].is_reach == 0)
				{
					HAL_Delay(50);
				}
				motor_statuses[4].is_reach = 0;
				break;
			}
			HAL_Delay(50);
		};
		while (stop_state == 1)
		{
			if (updateFlag == 0)
				return 1;
			HAL_Delay(50);
		};
		tick = 0;
		if (motor_statuses[4].is_reach == 0 && timeoutFlag == 1)
		{
			timeoutFlag = 0;
			LED_Red();
			/*
			  错误处理
			*/
		}
	}
	motor_statuses[4].is_reach = 0;

	return 0;
}

uint8_t PutDownMaterial(uint8_t num) // 放第num盘料
{
	uint32_t tick = 0;
	uint8_t timeoutFlag = 0;
	/*---------------------------整体降到放料最高点-------------------------*/
	while (motor_statuses[4].is_reach == 0)
	{
		MoveFifthMotor(2500, 180, PutDown_position[0]);
		while (motor_statuses[4].is_reach != 1)
		{
			tick += 1;
			if (tick >= 1000)
			{
				timeoutFlag = 1;
				break;
			}
			if (stop_state == 1 || updateFlag == 0)
			{
				// motor_Stop(5);
				motor_Sstop(5, 0);
				while (motor_statuses[4].is_reach == 0)
				{
					HAL_Delay(50);
				}
				motor_statuses[4].is_reach = 0;
				break;
			}
			HAL_Delay(50);
		};
		while (stop_state == 1)
		{
			if (updateFlag == 0)
				return 1;
			HAL_Delay(50);
		};
		tick = 0;
		if (motor_statuses[4].is_reach == 0 && timeoutFlag == 1)
		{
			timeoutFlag = 0;
			LED_Red();
			/*
				错误处理
			*/
		}
	}
	motor_statuses[4].is_reach = 0;

	/*---------------------------小臂前伸-------------------------*/
	tick = 0;
	while (motor_statuses[2].is_reach == 0 || motor_statuses[3].is_reach == 0)
	{
		MoveSecondGMotors(0, 1000, 50);
		while (motor_statuses[2].is_reach != 1 || motor_statuses[3].is_reach != 1)
		{

			tick += 1;
			if (tick >= 1000)
			{
				timeoutFlag = 1;
				break;
			}
			if (stop_state == 1 || updateFlag == 0)
			{
				motor_Stop(3);
				HAL_Delay(50);
				motor_Stop(4);
				HAL_Delay(450);
				break;
			}
			HAL_Delay(50);
		};
		while (stop_state == 1)
		{
			if (updateFlag == 0)
				return 1;
			HAL_Delay(50);
		};
		tick = 0;
		if ((motor_statuses[2].is_reach == 0 || motor_statuses[3].is_reach == 0) && timeoutFlag == 1)
		{
			timeoutFlag = 0;
			LED_Red();
			/*
			 错误处理
			*/
		}
	}
	motor_statuses[2].is_reach = 0;
	motor_statuses[3].is_reach = 0;

	/*---------------------------整体降到放料点-------------------------*/
	tick = 0;
	while (motor_statuses[4].is_reach == 0)
	{
		MoveFifthMotor(2500, 180, PutDown_position[num]);
		while (motor_statuses[4].is_reach != 1)
		{

			tick += 1;
			if (tick >= 1000)
			{
				timeoutFlag = 1;
				break;
			}
			if (stop_state == 1 || updateFlag == 0)
			{
				// motor_Stop(5);
				motor_Sstop(5, 0);
				while (motor_statuses[4].is_reach == 0)
				{
					HAL_Delay(50);
				}
				motor_statuses[4].is_reach = 0;
				break;
			}
			HAL_Delay(50);
		};
		while (stop_state == 1)
		{
			if (updateFlag == 0)
				return 1;
			HAL_Delay(50);
		};
		tick = 0;
		if (motor_statuses[4].is_reach == 0 && timeoutFlag == 1)
		{
			timeoutFlag = 0;
			LED_Red();
			/*
			  错误处理
			*/
		}
	}
	motor_statuses[4].is_reach = 0;

	/*---------------------------钩子松开-------------------------*/
	tick = 0;
	while (motor_statuses[0].is_reach == 0 || motor_statuses[1].is_reach == 0)
	{
		MoveFirstGMotors(1, 1000, 100);
		while (motor_statuses[0].is_reach != 1 || motor_statuses[1].is_reach != 1)
		{

			tick += 1;
			if (tick >= 1000)
			{
				timeoutFlag = 1;
				break;
			}
			if (stop_state == 1 || updateFlag == 0)
			{
				motor_Stop(1);
				HAL_Delay(50);
				motor_Stop(2);
				HAL_Delay(450);
				break;
			}
			HAL_Delay(50);
		};
		while (stop_state == 1)
		{
			if (updateFlag == 0)
				return 1;
			HAL_Delay(50);
		};
		tick = 0;
		if ((motor_statuses[0].is_reach == 0 || motor_statuses[1].is_reach == 0) && timeoutFlag == 1)
		{
			timeoutFlag = 0;
			LED_Red();
			/*
			  错误处理
			*/
		}
	}
	motor_statuses[0].is_reach = 0;
	motor_statuses[1].is_reach = 0;

	/*---------------------------小臂后缩-------------------------*/
	tick = 0;
	while (motor_statuses[2].is_reach == 0 || motor_statuses[3].is_reach == 0)
	{
		MoveSecondGMotors(1, 1000, 50);
		while (motor_statuses[2].is_reach != 1 || motor_statuses[3].is_reach != 1)
		{

			tick += 1;
			if (tick >= 1000)
			{
				timeoutFlag = 1;
				break;
			}
			if (stop_state == 1 || updateFlag == 0)
			{
				motor_Stop(3);
				HAL_Delay(50);
				motor_Stop(4);
				HAL_Delay(450);
				break;
			}
			HAL_Delay(50);
		};
		tick = 0;
		while (stop_state == 1)
		{
			if (updateFlag == 0)
				return 1;
			HAL_Delay(50);
		};
		if ((motor_statuses[2].is_reach == 0 || motor_statuses[3].is_reach == 0) && timeoutFlag == 1)
		{
			timeoutFlag = 0;
			LED_Red();
			/*
			 错误处理
			*/
		}
	}
	motor_statuses[2].is_reach = 0;
	motor_statuses[3].is_reach = 0;

	return 0;
}

void IO_Tran(uint8_t state) // 输出IO信号 料仓告知机器人料盘到位 state:1 到位 state:0 结束
{
	if (state == 0)
		HAL_GPIO_WritePin(GPIOC, Y1_Pin, GPIO_PIN_SET);
	else if (state == 1)
		HAL_GPIO_WritePin(GPIOC, Y1_Pin, GPIO_PIN_RESET);
}

uint8_t IO_Read(void) // 输入IO 机器人告知料仓已近取完 返回0:取完 返回1:未取完
{
	if (HAL_GPIO_ReadPin(X1_GPIO_Port, X1_Pin) == 0)
	{
		inputs[0] = 1;
		return 0;
	}
	else
		return 1;
}

uint8_t Run(uint8_t num) // 机器人加工一盘 流程:取盘、完成、放盘
{
	while (stop_state != 0)
	{
		if (updateFlag == 0)
			break;
		HAL_Delay(50);
	};

	if (sys_state == 1 && updateFlag != 0)
	{
		LED_Yellow();

		if (GetMaterial(num) != 0)
			return 1;

		IO_Tran(1); // 告知机器人取料到位
		LED_Green();
		sys_state = 2;
	}

	while (IO_Read() != 0) // 不加它就会有隐形的小坑
	{
		if (updateFlag == 0)
			return 1;
		HAL_Delay(10);
	};

	IO_Tran(0); // 结束
	LED_Yellow();

	while (stop_state != 0)
	{
		if (updateFlag == 0)
			return 1;

		HAL_Delay(50);
	};

	if (inputs[0] == 1 && sys_state == 2 && updateFlag != 0)
	{
		inputs[0] = 0;
		sys_state = 3;
		HAL_Delay(1000);
	}

	if (sys_state == 3 && updateFlag != 0)
	{
		if (PutDownMaterial(num) != 0)
			return 1;

		sys_state = 1;
	}

	return 0;
}

uint8_t Sys_Run(uint8_t stratnum)
{
	uint32_t i;
	for (i = stratnum; i >= 1; i--)
	{
		if (updateFlag == 1)
		{
			SetTextInt32(0, 23, tray_num - i, 0, 2);
			if (Run(i) != 0)
			{

				motor_statuses[0].is_reach = 0;
				motor_statuses[1].is_reach = 0;
				motor_statuses[2].is_reach = 0;
				motor_statuses[3].is_reach = 0;
				motor_statuses[4].is_reach = 0;

				SetTextInt32(0, 23, 0, 0, 2);
				LED_Green();
				sys_state = 1;
				return 1;
			}
		}
	}
	stop_state = 1;
	return 0;
}

void motor_Stop(uint16_t id)
{
	uint8_t msg[2];
	msg[0] = 0xf7;
	CAN1_Send_Msg(msg, 2, id);
}

void LED_Green(void) // 绿灯
{
	HAL_GPIO_WritePin(Y2_GPIO_Port, Y2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Y3_GPIO_Port, Y3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Y4_GPIO_Port, Y4_Pin, GPIO_PIN_SET);
}

void LED_Yellow(void) // 黄灯
{
	HAL_GPIO_WritePin(Y2_GPIO_Port, Y2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Y3_GPIO_Port, Y3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Y4_GPIO_Port, Y4_Pin, GPIO_PIN_SET);
}

void LED_Red(void) // 红灯
{
	HAL_GPIO_WritePin(Y2_GPIO_Port, Y2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Y3_GPIO_Port, Y3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Y4_GPIO_Port, Y4_Pin, GPIO_PIN_RESET);
}

void motor_Sstop(uint8_t slaveAddr, uint8_t acc)
{
	txBuffer[0] = 0xF5;
	txBuffer[1] = 0;
	txBuffer[2] = 0;
	txBuffer[3] = acc;
	txBuffer[4] = 0;
	txBuffer[5] = 0;
	txBuffer[6] = 0;
	CAN1_Send_Msg(txBuffer, 8, slaveAddr);
}
