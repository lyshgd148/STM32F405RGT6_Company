#pragma once 

#include <math.h>
#include "arm_math.h"
#include <stdbool.h>
#include "stdio.h"

#define SQRT3 1.73205080757f
#ifndef PI
#define PI 3.14159265358979323846
#endif // PI

#define deg2rad(deg) ((deg) * (PI / 180.0))
#define rad60 deg2rad(60.0f)

struct Frame
  {
    float fdata[3];
    uint8_t tail[4];
  };
 
extern struct Frame frame; 

void svpwm(float phi, float d, float q, float *d_u, float *d_v, float *d_w);