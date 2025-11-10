#pragma once

#include "stm32g4xx_hal.h"
#include "conf.h"

extern uint16_t angle;

#define rotor_phy_angle (encoder_angle - rotor_zero_angle) // 转子物理角度
#define rotor_logic_angle rotor_phy_angle *POLE_PAIRS      // 转子多圈角度
volatile extern float motor_i_u;
volatile extern float motor_i_w;
volatile extern float motor_i_d;
volatile extern float motor_i_q;
volatile extern float motor_speed;
volatile extern float motor_logic_angle; // 电机多圈角度
volatile extern float encoder_angle;     // 编码器直接读出的角度
volatile extern float rotor_zero_angle;  // 转子d轴与线圈d轴重合时的编码器角度

extern uint16_t Knob_buf; // 电位器ADC采样值
