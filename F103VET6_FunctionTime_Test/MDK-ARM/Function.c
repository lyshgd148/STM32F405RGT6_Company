#include "Function.h"



struct Frame frame = {
    .tail = {0x00, 0x00, 0x80, 0x7f}};

void svpwm(float phi, float d, float q, float *d_u, float *d_v, float *d_w)
{
    // 限幅
    float magnitude = sqrtf(d * d + q * q);
    if (magnitude > 0.85f)
    {
        d /= magnitude;
        q /= magnitude;
    }

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