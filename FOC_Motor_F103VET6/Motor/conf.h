#pragma once

#define POLE_PAIRS 7

#define R_SHUNT 0.02  // 电流采样电阻阻值
#define OP_GAIN 50.0  // 电流采样放大倍数
#define MAX_CURRENT 2 // 最大电流值

#define ADC_REFERENCE_VOLTAGE 3.3 // ADC参考电压
#define ADC_BITS 12               // ADC分辨率

#define motor_speed_calc_freq 930 // 电机速度计算频率 HZ
#define motor_pwm_freq 20000      // 电机PWM频率      HZ

#define position_cycle 6 * 3.14159265359 // 电机多圈周期，等于正半周期+负半周期