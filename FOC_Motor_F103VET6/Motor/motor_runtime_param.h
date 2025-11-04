#pragma once

#include "conf.h"

#define rotor_phy_angle (encoder_angle - rotor_zero_angle)
#define rotor_logical_angle motor_logical_angle *POLE_PAIRS

extern float motor_current_i_u;
extern float motor_current_i_v;
extern float motor_i_d;
extern float motor_i_q;
extern float motor_speed;
extern float motor_logical_angle; // 电机多圈角度
extern float encoder_angle;       // 编码器角度
extern float rotor_zero_angle;    // 转子d轴与线圈d轴重合时角度


