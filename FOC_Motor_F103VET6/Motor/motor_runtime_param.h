#pragma once

#include "conf.h"



extern float motor_i_u;
extern float motor_i_v;
extern float motor_i_d;
extern float motor_i_q;
extern float motor_speed;
extern float motor_logical_angle; // 电机多圈角度
extern float encoder_angle;       // 编码器角度
extern float rotor_zero_angle;    // 转子d轴与线圈d轴重合时角度

#define rotor_phy_angle (encoder_angle - rotor_zero_angle)
#define rotor_logical_angle rotor_phy_angle *POLE_PAIRS