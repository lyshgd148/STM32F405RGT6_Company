#include "foc.h"
#include "arm_math.h"
#include "motor_runtime_param.h"
#include "global_def.h"

#include <stdbool.h>

#define rad60 deg2rad(60.0f)
#define SQRT3 1.73205080757f

/**
 * @brief 笛卡尔坐标系下的svpwm
 *
 * @param phi 转子角度
 * @param d d轴强度
 * @param q q轴强度
 * @param d_u U相占空比指针
 * @param d_v V相占空比指针
 * @param d_w W相占空比指针
 */
void svpwm(float phi, float d, float q, float *d_u, float *d_v, float *d_w)
{
    d = min(d, 1);
    d = max(d, -1);
    q = min(q, 1);
    q = max(q, -1);

    const int v[6][3] = {{1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {0, 1, 1}, {0, 0, 1}, {1, 0, 1}};
    const int K_to_sector[] = {4, 6, 5, 5, 3, 1, 2, 2};

    float sin_phi = arm_sin_f32(phi);
    float cos_phi = arm_cos_f32(phi);

    float alpha = 0;
    float beta = 0;

    arm_inv_park_f32(d, q, &alpha, &beta, sin_phi, cos_phi);

    bool A = beta > 0;
    bool B = fabs(beta) > SQRT3 * fabs(alpha);
    bool C = alpha > 0;

    int K = 4 * A + 2 * B + 1 * C;
    int sector = K_to_sector[K];

    float t_m = arm_sin_f32(sector * rad60) * alpha - arm_cos_f32(sector * rad60) * beta;
    float t_n = arm_cos_f32(sector * rad60 - rad60) * beta - arm_sin_f32(sector * rad60 - rad60) * alpha;
    float t_0 = 1 - t_m - t_n;

    *d_u = t_m * v[sector - 1][0] + t_n * v[sector % 6][0] + t_0 / 2;
    *d_v = t_m * v[sector - 1][1] + t_n * v[sector % 6][1] + t_0 / 2;
    *d_w = t_m * v[sector - 1][2] + t_n * v[sector % 6][2] + t_0 / 2;
}

__attribute__((weak)) void set_pwm_duty(float d_u, float d_v, float d_w)
{
    while (1)
        ;
}

void foc_forward(float d, float q, float rotor_rad)
{
    float d_u = 0;
    float d_v = 0;
    float d_w = 0;

    svpwm(rotor_rad, d, q, &d_u, &d_v, &d_w);
    set_pwm_duty(d_u, d_v, d_w);
}

float cycle_diff(float diff, float cycle) // 两次检测角度之差要保证不会超过180° 才是使用这个函数的基础
{
    if (diff > (cycle / 2))
    {
        diff -= cycle;
    }
    else if (diff < (-cycle / 2))
    {
        diff += cycle;
    }

    return diff;
}