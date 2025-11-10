#include "foc.h"
#include "arm_math.h"
#include "motor_runtime_param.h"
#include "global_def.h"

#include <stdbool.h>

#define rad60 deg2rad(60.0f)
#define SQRT3 1.73205080757f

motor_control_context_t motor_control_context;
static arm_pid_instance_f32 pid_position;
static arm_pid_instance_f32 pid_speed;
static arm_pid_instance_f32 pid_torque_d;
static arm_pid_instance_f32 pid_torque_q;

void set_motor_pid(float position_p, float position_i, float position_d,
                   float speed_p, float speed_i, float speed_d,
                   float torque_d_p, float torque_d_i, float torque_d_d,
                   float torque_q_p, float torque_q_i, float torque_q_d)
{

    pid_position.Kp = position_p;
    pid_position.Ki = position_i;
    pid_position.Kd = position_d;

    pid_speed.Kp = speed_p;
    pid_speed.Ki = speed_i;
    pid_speed.Kd = speed_d;

    pid_torque_d.Kp = torque_d_p;
    pid_torque_d.Ki = torque_d_i;
    pid_torque_d.Kd = torque_d_d;

    pid_torque_q.Kp = torque_q_p;
    pid_torque_q.Ki = torque_q_i;
    pid_torque_q.Kd = torque_q_d;

    arm_pid_init_f32(&pid_position, false);
    arm_pid_init_f32(&pid_speed, false);
    arm_pid_init_f32(&pid_torque_d, false);
    arm_pid_init_f32(&pid_torque_q, false);
}

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
    // 限幅
    float magnitude = sqrtf(d * d + q * q);
    if (magnitude > 0.85f)
    {
        d = d / magnitude * 0.85f;
        q = q / magnitude * 0.85f;
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

/*----------------------------------------------位置环*/
static float position_loop(float rad)
{
    float diff = cycle_diff(rad - motor_logical_angle, position_cycle);
    return arm_pid_f32(&pid_position, diff);
}
void lib_position_control(float rad)
{
    float d = 0;
    float q = position_loop(rad);
    foc_forward(d, q, rotor_logical_angle);
}

/*----------------------------------------------速度环*/
static float speed_loop(float speed)
{
    float diff = speed - motor_speed;
    return arm_pid_f32(&pid_speed, diff);
}
void lib_speed_control(float speed)
{
    float d = 0;
    float q = speed_loop(speed);
    foc_forward(d, q, rotor_logical_angle);
}

/*----------------------------------------------力矩环*/
static float torque_d_loop(float d)
{
    float diff = d - motor_i_d / MAX_CURRENT;
    return arm_pid_f32(&pid_torque_d, diff);
}
static float torque_q_loop(float q)
{
    float diff = q - motor_i_q / MAX_CURRENT;
    return arm_pid_f32(&pid_torque_q, diff);
}
void lib_torque_control(float torque_norm_d, float torque_norm_q)
{
    float d = torque_d_loop(torque_norm_d);
    float q = torque_q_loop(torque_norm_q);
    foc_forward(d, q, rotor_logical_angle);
}

/*----------------------------------------------位置、速度、力矩*/
void lib_speed_torque_control(float speed_rad, float max_torque_norm)
{
    float torque_norm = speed_loop(speed_rad);
    torque_norm = min(torque_norm, max_torque_norm);
    lib_torque_control(0, torque_norm);
}
void lib_position_speed_torque_control(float position_rad, float max_speed_rad, float max_torque_norm)
{
    float speed_rad = position_loop(position_rad);
    speed_rad = min(fabs(speed_rad), max_speed_rad) * (speed_rad > 0 ? 1 : -1);
    lib_speed_torque_control(speed_rad, max_torque_norm);
}
