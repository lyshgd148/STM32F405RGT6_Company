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

int8_t x1=0;   //Ã«Å÷ÅÌ²ãÆ«
int8_t x2=0;   //³ÉÆ·ÅÌ²ãÆ«



void Mydebug(void)
{
	uint32_t tick;
	uint8_t err;
	
	if (hookR_flag == 1)
	{
		tick=0;
		err=0;
		motorGoPosition(1, 1000, 150, -1 * hookR_angle);
		LED_Yellow();
		while (motor_statuses[0].is_reach == 0)
		{	
			tick++;
			if(tick>601)
			{
				err=1;
				break;
			}
			HAL_Delay(100);
		}
		if(err==1)
			LED_Red();
		else
			LED_Green();
		
		hookR_flag = 0;
		motor_statuses[0].is_reach = 0;
	}
	else if (hookL_flag == 1)
	{
		tick=0;
		err=0;
		motorGoPosition(2, 1000, 150, -1 * hookL_angle);
		LED_Yellow();
		while (motor_statuses[1].is_reach == 0)
		{
			tick++;
			if(tick>601)
			{
				err=1;
				break;
			}
			HAL_Delay(100);
		}
		if(err==1)
			LED_Red();
		else
			LED_Green();
		hookL_flag = 0;
		motor_statuses[1].is_reach = 0;
	}
	else if (hook_flag == 1)
	{
		tick=0;
		err=0;
		
		int32_t value=(hookR_angle-22000)/2533.33;
		motorGoPosition(1, 1000, 150, -1 * hookR_angle);
		motorGoPosition(2, 1000, 150, -1 * hookL_angle);
		SetTextInt32(2, 28, value, 0, 3);
		SetTextInt32(2, 24, value, 0, 3);
		LED_Yellow();
		while (motor_statuses[0].is_reach == 0 || motor_statuses[1].is_reach == 0)
		{
			tick++;
			if(tick>601)
			{
				err=1;
				break;
			}
			HAL_Delay(100);
		}
		if(err==1)
			LED_Red();
		else
			LED_Green();
		hook_flag = 0;
		motor_statuses[0].is_reach = 0;
		motor_statuses[1].is_reach = 0;
	}
	else if (arm_flag == 1)
	{
		tick=0;
		err=0;
		
		motorGoPosition(3, 1000, 150, arm_length);
		motorGoPosition(4, 1000, 150, -1 * arm_length-364);
		LED_Yellow();
		while (motor_statuses[2].is_reach == 0 || motor_statuses[3].is_reach == 0)
		{
			tick++;
			if(tick>601)
			{
				err=1;
				break;
			}
			HAL_Delay(100);
		}
		if(err==1) 
			LED_Red();
		else
			LED_Green();
		
		arm_flag = 0;
		motor_statuses[2].is_reach = 0;
		motor_statuses[3].is_reach = 0;
	}


	if (state_dbg == 1 || state_dbg == 3)
	{
		tick=0;
		err=0;
		
		MoveFifthMotor(1000, 100, heigh_dbg * 16384 / lead_screw);
		LED_Yellow();
		while (motor_statuses[4].is_reach == 0)
		{
			tick++;
			if(tick>801)
			{
				err=1;
				break;
			}
			HAL_Delay(100);
		}
		if(err==1)
			LED_Red();
		else
			LED_Green();
			
		SetTextInt32(2,22,heigh_dbg,1,3);
		motor_statuses[4].is_reach = 0;
		state_dbg = 0;
		flag = 1;
	}
	else if (state_dbg == 2)
	{

		motor_AllGoHome();
		
		firstFlag_dgb = 0;
		secondFlag_dgb = 0;

		state_dbg = 0;
		flag = 0;
	}
	else if (state_dbg == 4)
	{
		tick=0;
		err=0;
		MoveFirstGMotors(firstFlag_dgb, 500, 50);
		LED_Yellow();
		while (motor_statuses[0].is_reach == 0 && motor_statuses[1].is_reach == 0)
		{
			tick++;
			if(tick>601)
			{
				err=1;
				break;
			}
			HAL_Delay(100);
		}
		if(err==1)
			LED_Red();
		else
			LED_Green();
		
		motor_statuses[0].is_reach = 0;
		motor_statuses[1].is_reach = 0;
		
		firstFlag_dgb++;
		firstFlag_dgb%=2;
		
		state_dbg = 0;
		flag = 1;
	}
	else if (state_dbg == 5)
	{
		tick=0;
		err=0;
		
		MoveSecondGMotors(secondFlag_dgb, 500, 50,0);
		LED_Yellow();
		while (motor_statuses[2].is_reach == 0 && motor_statuses[3].is_reach == 0)
		{
			tick++;
			if(tick>601)
			{
				err=1;
				break;
			}
			HAL_Delay(100);
		}
		if(err==1) 
			LED_Red();
		else
			LED_Green();
		motor_statuses[2].is_reach = 0;
		motor_statuses[3].is_reach = 0;
		
		secondFlag_dgb++;
		secondFlag_dgb%=2;
		
		state_dbg = 0;
		flag = 1;
	}
}
#endif
