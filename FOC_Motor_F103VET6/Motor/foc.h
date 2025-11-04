#pragma once

void foc_forward(float d, float q, float rotor_rad);
void svpwm(float phi, float d, float q, float *d_u, float *d_v, float *d_w);

float cycle_diff(float diff, float cycle);