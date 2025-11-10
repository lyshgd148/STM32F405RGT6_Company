#include "stm32g4xx_hal.h"

#include "motor_runtime_param.h"

volatile float motor_i_u;
volatile float motor_i_w;
volatile float motor_i_d;
volatile float motor_i_q;
volatile float motor_speed;
volatile float motor_logic_angle;
volatile float encoder_angle;
volatile float rotor_zero_angle = 0;

uint16_t angle = 0;

uint16_t Knob_buf;
