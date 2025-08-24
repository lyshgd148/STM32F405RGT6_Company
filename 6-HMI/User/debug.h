#ifndef _DEBUG_H_
#define _DEBUG_H_

#include "main.h"

#define dbg 1
#if dbg
extern uint8_t debug;
extern int32_t heigh_dbg;
extern uint8_t firstFlag_dgb;
extern uint8_t secondFlag_dgb;
extern uint8_t state_dbg;
extern uint8_t goHomeFlag_dbg;
extern int32_t heigh_dbg_single;

extern int32_t hookL_angle;
extern int32_t hookR_angle;
extern int32_t arm_length;
extern uint8_t hook_flag;
extern uint8_t hookL_flag;
extern uint8_t hookR_flag;
extern uint8_t arm_flag;

void Mydebug(void);
#endif

#endif
