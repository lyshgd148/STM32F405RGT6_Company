#include "Screen.h"
#include "hmi_driver.h"
#include "motor.h"
#include "debug.h"

uint8 cmd_buffer[CMD_MAX_SIZE];

void ProcessMessage(PCTRL_MSG msg, uint16 size)
{
	uint8 cmd_type = msg->cmd_type;
	uint8 ctrl_msg = msg->ctrl_msg;
	uint8 control_type = msg->control_type;
	uint16 screen_id = PTR2U16(&msg->screen_id);
	uint16 control_id = PTR2U16(&msg->control_id);
	uint32 value = PTR2U32(msg->param);

	switch (cmd_type)
	{
	case NOTIFY_CONTROL:
	{
		switch (control_type)
		{
		case kCtrlButton:
			NotifyButton(screen_id, control_id, msg->param[1]);
			break;
		case kCtrlText:
			NotifyText(screen_id, control_id, msg->param);
			break;

		default:
			break;
		}
	}
	default:
		break;
	}
}

/*
state: 0 弹起  、 1 按下
*/
void NotifyButton(uint16 screen_id, uint16 control_id, uint8 state)
{
	if (screen_id == 0)
	{
		if (control_id == 2 && state == 0)
		{
			stop_state = 1;
			updateFlag = 0;
		}
		else if (control_id == 2 && state == 1) // 按钮按下
		{
#if dbg
			debug = 0;
#endif
			Position_Init(tray_offset, PutDown_offset, heigh);
			stop_state = 0;
			updateFlag = 1;
		}
	}

#if dbg
	if (debug == 1)
	{
		if (screen_id == 2)
		{
			if (control_id == 2 && state == 1 && state_dbg == 0 && goHomeFlag_dbg == 1) // 1
			{
				heigh_dbg += heigh_dbg_single;
				state_dbg = 1;
			}

			else if (control_id == 3 && state == 1 && state_dbg == 0 && goHomeFlag_dbg == 1) // 2
			{
				heigh_dbg = 0;
				state_dbg = 2;
			}

			else if (control_id == 4 && state == 1 && state_dbg == 0 && goHomeFlag_dbg == 1) // 3
			{
				heigh_dbg -= heigh_dbg_single;
				state_dbg = 3;
			}

			else if (control_id == 5 && state == 1 && state_dbg == 0 && goHomeFlag_dbg == 1) // 4
			{
				state_dbg = 4;
			}

			else if (control_id == 6 && state == 1 && state_dbg == 0 && goHomeFlag_dbg == 1) // 5
			{
				state_dbg = 5;
			}
		}
	}

#endif
}

void NotifyText(uint16 screen_id, uint16 control_id, uint8 *strs)
{
	int32 value = 0;
	sscanf((const char *)strs, "%ld", &value); // 只取整数 输入小数会截断
	if (screen_id == 0)
	{
		if (control_id == 9)
		{
			if ((tray_offset + (value - 2) * heigh) > Max_Length || (PutDown_offset + (value - 2) * heigh + heigh / 4) > Max_Length)
			{
				SetScreen(1);
				tray_num = 1;
				tray_offset = 0;
				PutDown_offset = 0;
				heigh = 0;
			}
			else
				tray_num = value + 1;
		}
		/*-----------------------------------------------------------------*/
		else if (control_id == 10)
		{
			if ((tray_offset + (tray_num - 2) * value) > Max_Length || (PutDown_offset + (tray_num - 2) * value + value / 4) > Max_Length)
			{
				SetScreen(1);
				tray_num = 1;
				tray_offset = 0;
				PutDown_offset = 0;
				heigh = 0;
			}
			else
				heigh = value;
		}
		/*-----------------------------------------------------------------*/
		else if (control_id == 11)
		{
			if ((value + (tray_num - 2) * heigh) > Max_Length || (PutDown_offset + (tray_num - 2) * heigh + heigh / 4) > Max_Length)
			{
				SetScreen(1);
				tray_num = 1;
				tray_offset = 0;
				PutDown_offset = 0;
				heigh = 0;
			}
			else
				tray_offset = value;
		}
		/*-----------------------------------------------------------------*/
		else if (control_id == 12)
		{
			if ((tray_offset + (tray_num - 2) * heigh) > Max_Length || (value + (tray_num - 2) * heigh + heigh / 4) > Max_Length)
			{
				SetScreen(1);
				tray_num = 1;
				tray_offset = 0;
				PutDown_offset = 0;
				heigh = 0;
			}
			else
				PutDown_offset = value;
		}
	}

#if dbg
	if (screen_id == 2)
	{
		if (control_id == 17)
		{
			heigh_dbg_single = value;
		}
		else if (control_id == 24 && hookR_flag == 0 && hookL_flag == 0 && arm_flag == 0 && hook_flag == 0) // 右钩子
		{
			hookR_angle = 22000 + 2533.33 * value;
			hookR_flag = 1;
		}
		else if (control_id == 28 && hookR_flag == 0 && hookL_flag == 0 && arm_flag == 0 && hook_flag == 0) // 左钩子
		{
			hookL_angle = 16000 + 2433.33 * value;
			hookL_flag = 1;
		}
		else if (control_id == 30 && hookR_flag == 0 && hookL_flag == 0 && arm_flag == 0 && hook_flag == 0) // 所有钩子
		{
			hookR_angle = 22000 + 2533.33 * value;
			hookL_angle = 16000 + 2433.33 * value;
			hook_flag = 1;
		}
		else if (control_id == 25 && hookR_flag == 0 && hookL_flag == 0 && arm_flag == 0 && hook_flag == 0) // 手臂
		{
			arm_length = value * 16384 / 9;
			arm_flag = 1;
		}
	}
#endif
}

void MyGetTextValue(void) // 获取4个文本输入框的值
{
	GetControlValue(0, 9);
	HAL_Delay(150);
	GetControlValue(0, 10);
	HAL_Delay(150);
	GetControlValue(0, 11);
	HAL_Delay(150);
	GetControlValue(0, 12);
}
