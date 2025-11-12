#include "stm32g4xx_hal.h"


#include "motor_runtime_param.h"

float motor_i_u;
float motor_i_w;
float motor_i_d;
float motor_i_q;
float motor_speed;
float motor_logic_angle;
float encoder_angle;
float rotor_zero_angle;

uint16_t angle_raw;