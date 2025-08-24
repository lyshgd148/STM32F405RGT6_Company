#ifndef _SCREEN_H_
#define _SCREEN_H_

#include "main.h"
#include "stdio.h"

// 串口屏部分
#include "hmi_driver.h"
#include "cmd_queue.h"
#include "cmd_process.h"
#include "debug.h"

extern uint8 cmd_buffer[CMD_MAX_SIZE];

void ProcessMessage(PCTRL_MSG msg, uint16 size);
void MyGetTextValue(void); // 获取4个文本输入框的值

#endif
