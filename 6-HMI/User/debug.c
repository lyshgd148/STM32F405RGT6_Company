#include "debug.h"
#include "hmi_driver.h"
#include "motor.h"




#if dbg
uint8_t debug = 1;
uint8_t firstFlag_dgb = 0;
uint8_t secondFlag_dgb = 0;
uint8_t state_dbg = 0; // 0 ?????  1 ??dgb1 2 3 4 5
uint8_t goHomeFlag_dbg = 0;
int32_t heigh_dbg;
int32_t heigh_dbg_single = 5;

int32_t hookL_angle;
int32_t hookR_angle;
int32_t arm_length;
uint8_t hook_flag = 0;
uint8_t hookL_flag = 0;
uint8_t hookR_flag = 0;
uint8_t arm_flag = 0;
extern uint8_t flag;



void Mydebug(void)
{
	if (hookR_flag == 1)
	{
		motorGoPosition(1, 1000, 150, -1 * hookR_angle);
		LED_Yellow();
		while (motor_statuses[0].is_reach == 0)
		{
			HAL_Delay(100);
		}
		LED_Green();
		hookR_flag = 0;
		motor_statuses[0].is_reach = 0;
	}
	else if (hookL_flag == 1)
	{
		motorGoPosition(2, 1000, 150, -1 * hookL_angle);
		LED_Yellow();
		while (motor_statuses[1].is_reach == 0)
		{
			HAL_Delay(100);
		}
		LED_Green();
		hookL_flag = 0;
		motor_statuses[1].is_reach = 0;
	}
	else if (hook_flag == 1)
	{
		int32_t value=(hookR_angle-22000)/2533.33;
		motorGoPosition(1, 1000, 150, -1 * hookR_angle);
		motorGoPosition(2, 1000, 150, -1 * hookL_angle);
		SetTextInt32(2, 28, value, 0, 3);
		SetTextInt32(2, 24, value, 0, 3);
		LED_Yellow();
		while (motor_statuses[0].is_reach == 0 || motor_statuses[1].is_reach == 0)
		{
			HAL_Delay(100);
		}
		LED_Green();
		hook_flag = 0;
		motor_statuses[0].is_reach = 0;
		motor_statuses[1].is_reach = 0;
	}
	else if (arm_flag == 1)
	{
		motorGoPosition(3, 1000, 150, arm_length);
		motorGoPosition(4, 1000, 150, -1 * arm_length);
		LED_Yellow();
		while (motor_statuses[2].is_reach == 0 || motor_statuses[3].is_reach == 0)
		{
			HAL_Delay(100);
		}
		LED_Green();
		arm_flag = 0;
		motor_statuses[2].is_reach = 0;
		motor_statuses[3].is_reach = 0;
	}


	if (state_dbg == 1 || state_dbg == 3)
	{
		MoveFifthMotor(1000, 100, heigh_dbg * 16384 / lead_screw);
		LED_Yellow();
		while (motor_statuses[4].is_reach == 0)
		{
			HAL_Delay(100);
		}
		SetTextFloat(2, 22, (float)(heigh_dbg / 10), 1, 2);
		LED_Green();
		motor_statuses[4].is_reach = 0;
		state_dbg = 0;
		flag = 1;
	}
	else if (state_dbg == 2)
	{
//		motorGoHome(1);
//		motorGoHome(2);
//		LED_Yellow();
//		while (motor_statuses[0].is_reach == 0 && motor_statuses[1].is_reach == 0)
//		{
//			HAL_Delay(100);
//		}
//		motor_statuses[0].is_reach = 0;
//		motor_statuses[1].is_reach = 0;

//		motorGoHome(3);
//		motorGoHome(4);
//		motorGoHome(5);
//		while (motor_statuses[2].is_reach == 0 && motor_statuses[3].is_reach == 0 && motor_statuses[4].is_reach == 0)
//		{
//			HAL_Delay(100);
//		}
//		LED_Green();
//		motor_statuses[2].is_reach = 0;
//		motor_statuses[3].is_reach = 0;
//		motor_statuses[4].is_reach = 0;

//		SetTextFloat(2, 22, (float)(0), 1, 2);
		motor_AllGoHome();
		
		firstFlag_dgb = 0;
		secondFlag_dgb = 0;
//		heigh_dbg = 0;
		state_dbg = 0;
		flag = 0;
	}
	else if (state_dbg == 4)
	{
		MoveFirstGMotors(firstFlag_dgb % 2, 500, 50);
		LED_Yellow();
		while (motor_statuses[0].is_reach == 0 && motor_statuses[1].is_reach == 0)
		{
			HAL_Delay(100);
		}
		LED_Green();
		motor_statuses[0].is_reach = 0;
		motor_statuses[1].is_reach = 0;
		firstFlag_dgb++;
		state_dbg = 0;
		flag = 1;
	}
	else if (state_dbg == 5)
	{
		MoveSecondGMotors(secondFlag_dgb % 2, 500, 50);
		LED_Yellow();
		while (motor_statuses[2].is_reach == 0 && motor_statuses[3].is_reach == 0)
		{
			HAL_Delay(100);
		}
		LED_Green();
		motor_statuses[2].is_reach = 0;
		motor_statuses[3].is_reach = 0;
		secondFlag_dgb++;
		state_dbg = 0;
		flag = 1;
	}
}
#endif
