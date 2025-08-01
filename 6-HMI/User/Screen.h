#ifndef _SCREEN_H_
#define _SCREEN_H_
 
#include "main.h"
#include "stdio.h"
 
//串口屏部分
#include "hmi_driver.h"
#include "cmd_queue.h"
#include "cmd_process.h"

#define dbg		   1											//显示屏debug模式 /1开启                                                             //也是让我用上体条件编译了
#if dbg
	extern uint8_t debug;
	extern int32_t heigh_dbg;
	extern uint8_t firstFlag_dgb;
	extern uint8_t secondFlag_dgb;
	extern uint8_t state_dbg;
	extern uint8_t goHomeFlag_dbg;
#endif

extern uint8 cmd_buffer[CMD_MAX_SIZE]; 

void ProcessMessage( PCTRL_MSG msg, uint16 size );
void MyGetTextValue(void); 							//获取4个文本输入框的值

#endif
