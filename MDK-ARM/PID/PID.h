#ifndef PID_H
#define PID_H

#include <stdint.h>
#include <stdbool.h>
#include "app_config.h"


/**
 * @brief 增量式 PID 控制器结构体 (已修正)
 */
typedef struct {
    /* === 参数 === */
    float kp, ki, kd;
    float dt;
    float out_min, out_max;
    float delta_out_limit;
    float d_alpha;             // [修正] 统一使用 d_alpha，移除易混淆的 d_filter_alpha
    bool  enable_conditional_i;
    float i_deadband;
    float kp_scale_threshold;
    float kp_near;

    /* === 状态量 === */
    float e_prev;
    float v_filt_prev;
    float meas_prev;
    float u_prev;
    bool  has_prev;
    float d_filter_state;
} PID_t;

#ifdef __cplusplus
extern "C" {
#endif

// 函数声明保持不变
void PID_Init(PID_t *pid, float kp, float ki, float kd, float dt,
              float kp_near, float kp_scale_threshold, float i_deadband);
void PID_SetOutputLimit(PID_t *pid, float out_min, float out_max);
void PID_SetDeltaLimit(PID_t *pid, float delta_out_limit);
void PID_SetDFilter(PID_t *pid, float d_alpha);
void PID_EnableConditionalI(PID_t *pid, bool enable);
void PID_Reset(PID_t *pid, float measurement, float u_init);
float PID_Update_Pos2Vel(PID_t *pid, float setpoint, float measurement);

#ifdef __cplusplus
}
#endif

#endif // PID_H
