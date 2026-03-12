#include "PID.h"
#include <math.h>

/* 简单夹紧函数 */
static inline float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* 对称“绝对值限幅” */
static inline float clamp_abs(float v, float abs_limit) {
    if (abs_limit <= 0.0f) return v;
    if (v >  abs_limit) return  abs_limit;
    if (v < -abs_limit) return -abs_limit;
    return v;
}

void PID_Init(PID_t *pid, float kp, float ki, float kd, float dt,
              float kp_near, float kp_scale_threshold, float i_deadband){
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->dt = (dt > 0.0f) ? dt : 1e-3f;

    pid->kp_near             = kp_near;
    pid->kp_scale_threshold  = kp_scale_threshold;
    pid->i_deadband          = i_deadband;

    pid->out_min = PID_OUT_MIN_DEFAULT;
    pid->out_max = PID_OUT_MAX_DEFAULT;
    pid->delta_out_limit = 100.0f;
    pid->enable_conditional_i = true;

    // 初始化时统一设置 d_alpha
    pid->d_alpha = PID_D_ALPHA_DEFAULT;

    // 调用 Reset 来初始化状态，避免代码重复
    PID_Reset(pid, 0.0f, 0.0f);
}

void PID_SetOutputLimit(PID_t *pid, float out_min, float out_max) {
    pid->out_min = out_min;
    pid->out_max = out_max;
}

void PID_SetDeltaLimit(PID_t *pid, float delta_out_limit) {
    pid->delta_out_limit = (delta_out_limit >= 0.0f) ? delta_out_limit : 0.0f;
}

void PID_SetDFilter(PID_t *pid, float d_alpha) {
    // [修正] 现在这个函数能正确生效了
    pid->d_alpha = clampf(d_alpha, 0.0f, 1.0f);
}

void PID_EnableConditionalI(PID_t *pid, bool enable) {
    pid->enable_conditional_i = enable;
}

void PID_Reset(PID_t *pid, float measurement, float u_init) {
    pid->e_prev       = 0.0f;
    pid->v_filt_prev  = 0.0f;
    pid->meas_prev    = measurement;
    pid->u_prev       = u_init; // u_prev 被强制设置为 u_init (通常是0)
    pid->d_filter_state = 0.0f;

    // [关键修正] 必须将 has_prev 置为 false！
    // 这能确保下一次 Update 时，会进入正确的初始化分支，
    // 避免使用陈旧的 e_prev 和 v_filt_prev，防止启动跳变。
    pid->has_prev     = false;
}

float PID_Update_Pos2Vel(PID_t *pid, float setpoint, float measurement) {
    float e = setpoint - measurement;

    if (!pid->has_prev) {
        pid->meas_prev = measurement;
    }
    float v_meas = (measurement - pid->meas_prev) / pid->dt;

    // [修正] 使用统一的 d_alpha，滤波器现在可以被正确配置
    pid->d_filter_state = pid->d_alpha * v_meas + (1.0f - pid->d_alpha) * pid->d_filter_state;
    float v_filt = pid->d_filter_state;

    if (!pid->has_prev) {
        pid->e_prev      = e;
        pid->v_filt_prev = v_filt;
        // [关键修正] 首次运行时，u_prev 必须从0开始，而不是继承 Reset 时的值
        pid->u_prev      = 0.0f;
        pid->has_prev    = true;
    }

    float active_kp = (fabsf(e) < pid->kp_scale_threshold) ? pid->kp_near : pid->kp;

    float p_inc = active_kp * (e - pid->e_prev);
    float i_inc = pid->ki * e * pid->dt;
    float d_inc = -pid->kd * (v_filt - pid->v_filt_prev);

    float du_pid = p_inc + d_inc;

    bool is_saturated = (pid->u_prev >= pid->out_max && e > 0.0f) ||
                        (pid->u_prev <= pid->out_min && e < 0.0f);

    if (fabsf(e) > pid->i_deadband) {
        if (!pid->enable_conditional_i || !is_saturated) {
            du_pid += i_inc;
        }
    }

    if (pid->delta_out_limit > 0.0f) {
        du_pid = clamp_abs(du_pid, pid->delta_out_limit);
    }

    float u_final = pid->u_prev + du_pid;
    float u_clamped = clampf(u_final, pid->out_min, pid->out_max);

    pid->e_prev      = e;
    pid->v_filt_prev = v_filt;
    pid->meas_prev   = measurement;
    pid->u_prev      = u_clamped; // 下一次的 u_prev 是本次限幅后的输出

    /* [BUG修复] 额外的硬限幅 */
    if(u_clamped >= 20) u_clamped = 20;
    else if(u_clamped <= -20) u_clamped = -20; // <-- 已修复笔误

    return u_clamped;
}
