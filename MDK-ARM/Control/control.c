#include "control.h"
#include <string.h>
#include <math.h>

/* ===========================================================================================
 * 职责：
 *   - 实现多轴“位置→速度”闭环控制的初始化、对齐、设定目标与周期计算（Tick）。
 *   - 与 PID 内核模块（PID.h/.c）对接，将位置误差转换为速度命令（vel_cmd）。
 * 量纲与约定：
 *   - 位置 pos_*：相对零位的长度/角度（单位：m 或 rad，取决于系统定义），由上层统一。
 *   - 速度 vel_*：m/s 或 rad/s（与执行器协议一致）。
 *   - 控制周期 dt：秒（s），由上层在 Control_Init 设置并保持恒定。
 * 与其他层的关系：
 *   - App 层在每个控制周期调用 Control_TickAll，之后把 vel_out 写入通信层命令（W）。
 *   - 归零/重新对齐时，调用 Control_SyncAxisToMeas / Control_SyncAllToMeas 清理历史状态。
 * 安全与鲁棒性：
 *   - 通过 PID_SetDeltaLimit 将“速度命令变化率”限制为 accel_max*dt，抑制速度阶跃。
 *   - use_cond_i 可抑制远离目标时的积分累积，防止 windup。
 * =========================================================================================== */

/**
 * @brief  初始化所有轴的控制器与 PID 内核，并设置默认参数（运行时可覆盖）
 * @param  c   控制器对象指针（由调用者提供存储）
 * @param  dt  控制周期（秒）；若传入不合理（<=0），退回 APP_CTRL_DT
 * @note
 *  1) PID_Init 顺序：先配置滤波/输出限幅/条件积分等，再初始化增益与采样周期；
 *  2) delta-limit：将“输出增量”限制为 accel_max*dt，避免速度命令阶跃；
 *  3) PID_Reset：把 PID 内部状态（误差积分/微分滤波/上一输出等）重置到“测量=0、u=0”。
 */
void Control_Init(Control_t *c, float dt){
    memset(c, 0, sizeof(*c));
    c->dt = (dt > 0.0f) ? dt : APP_CTRL_DT;

    for (int i=0; i<APP_MOTORS; ++i){
        /* D 项一阶滤波系数：0..1（越小越平滑，延迟越大） */
        PID_SetDFilter(&c->axis[i].pid, PID_D_ALPHA_DEFAULT);

        /* 输出限幅：避免速度命令过大（保护执行器/绳索/电源） */
        PID_SetOutputLimit(&c->axis[i].pid, PID_OUT_MIN_DEFAULT, PID_OUT_MAX_DEFAULT);

        /* 条件积分：仅在接近目标时允许积分，抑制 windup */
        PID_EnableConditionalI(&c->axis[i].pid, PID_USE_COND_I_DEFAULT != 0);

        /* 初始化 PID 增益与采样周期，并传入近区 KP 缩放与积分死区配置
         *  - KP_NEAR / KP_SCALE：接近目标时降低 KP，减小抖动；
         *  - I_DEADBAND：误差小于阈值不积分。 */
        PID_Init(&c->axis[i].pid, PID_KP_DEFAULT, PID_KI_DEFAULT, PID_KD_DEFAULT,
                 c->dt, PID_KP_NEAR, PID_KP_SCALE, PID_I_DEADBAND);

        /* 将“速度命令变化”限幅为 a_max*dt，限制斜率，避免阶跃造成机械冲击 */
        PID_SetDeltaLimit(&c->axis[i].pid,
            PID_ACCEL_MAX_DEFAULT > 0.0f ? PID_ACCEL_MAX_DEFAULT * c->dt : 0.0f);

        /* 轴默认使能；目标/测量/输出均清零 */
        c->axis[i].enabled  = 1;
        c->axis[i].pos_set  = 0.0f;
        c->axis[i].pos_meas = 0.0f;
        c->axis[i].vel_cmd  = 0.0f;

        /* 把 PID 的内部状态重置为“当前测量=0，输出=0”，防止历史状态影响初次控制 */
        PID_Reset(&c->axis[i].pid, /*meas=*/0.0f, /*u_init=*/0.0f);
    }
}

/**
 * @brief  把某一轴的“目标/测量/输出”对齐到同一位置（常用于上电置零或重新置零）
 * @param  c         控制器对象
 * @param  i         轴索引（0..APP_MOTORS-1）
 * @param  pos_meas  当前实测相对位置（单位：m 或 rad，与系统统一）
 * @note
 *  - pos_set←pos_meas，vel_cmd←0；PID_Reset(meas=pos_meas,u_init=0)
 *  - 可避免在零位切换瞬间出现大误差引起的强烈控制动作或积分惯性。
 */
void Control_SyncAxisToMeas(Control_t *c, int i, float pos_meas){
    if (i<0 || i>=APP_MOTORS) return;
    c->axis[i].pos_meas = pos_meas;
    c->axis[i].pos_set  = pos_meas;
    c->axis[i].vel_cmd  = 0.0f;
    PID_Reset(&c->axis[i].pid, pos_meas, 0.0f);
}

/**
 * @brief  所有轴按当前测量进行批量对齐（归零后常用的一步）
 * @param  c         控制器对象
 * @param  pos_meas  各轴实测相对位置数组（长度 APP_MOTORS）
 */
void Control_SyncAllToMeas(Control_t *c, const float pos_meas[APP_MOTORS]){
    if (!pos_meas) return;
    for (int i=0;i<APP_MOTORS;++i){
        Control_SyncAxisToMeas(c, i, pos_meas[i]);
    }
}

/**
 * @brief  批量设定各轴的目标位置（相对零位）
 * @param  c        控制器对象
 * @param  pos_set  目标位置数组（长度 APP_MOTORS）
 * @note   只写入 pos_set，不立即计算；真正计算在 Control_TickAll 中完成。
 */
void Control_SetTargets(Control_t *c, const float pos_set[APP_MOTORS]){
    if (!pos_set) return;
    for (int i=0;i<APP_MOTORS;++i){
        c->axis[i].pos_set = pos_set[i];
    }
}

/**
 * @brief  运行时配置某轴 PID 参数与约束（热更新）
 * @param  c          控制器对象
 * @param  i          轴索引
 * @param  kp,ki,kd   PID 增益
 * @param  d_alpha    D 项滤波系数（0..1，越小越平滑）
 * @param  out_min    输出下限
 * @param  out_max    输出上限
 * @param  accel_max  速度命令的最大斜率（a_max），内部转换为 delta-limit=a_max*dt
 * @param  use_cond_i 条件积分开关（0/1）
 * @param  u_init     输出初值（用于参数切换时的平滑过渡）
 * @param  i_deadband 积分死区（|err|<阈值不积分）
 * @param  kp_scale_threshold  KP 缩放触发阈值（接近目标降增益）
 * @param  kp_near    近区 KP 或缩放系数
 * @note
 *  - 为避免并发写入，建议在控制周期边界调用，或在实现中加互斥。
 *  - PID_Reset(meas=当前测量, u_init) 用于抑制参数突变带来的历史项惯性。
 */
void Control_ConfigAxisPID(Control_t *c, int i,
                           float kp, float ki, float kd,
                           float d_alpha,
                           float out_min, float out_max,
                           float accel_max, int use_cond_i,
                           float u_init, float i_deadband, 
                           float kp_scale_threshold, float kp_near)   {
    if (i<0 || i>=APP_MOTORS) return;

    /* 更新增益/采样与近区策略/积分死区 */
    PID_Init(&c->axis[i].pid, kp, ki, kd, c->dt,  kp_near, kp_scale_threshold, i_deadband);

    /* 更新滤波/限幅/条件积分 */
    PID_SetDFilter(&c->axis[i].pid, d_alpha);
    PID_SetOutputLimit(&c->axis[i].pid, out_min, out_max);
    PID_EnableConditionalI(&c->axis[i].pid, use_cond_i != 0);

    /* 更新斜率限制：a_max * dt，若 a_max<=0 则关闭增量限制 */
    PID_SetDeltaLimit(&c->axis[i].pid, (accel_max>0.0f) ? (accel_max * c->dt) : 0.0f);

    /* 重置内部状态，避免参数瞬时改变导致的“历史项惯性”产生突变 */
    PID_Reset(&c->axis[i].pid, c->axis[i].pos_meas, u_init);
}

/**
 * @brief  在一个控制周期内：更新测量 → 逐轴计算 vel_cmd → 可选返回 vel_out
 * @param  c         控制器对象
 * @param  pos_meas  实测相对位置数组（可为 NULL；为 NULL 时不更新测量）
 * @param  vel_out   输出速度数组指针（可为 NULL；为 NULL 时仅内部更新 vel_cmd）
 * @note
 *  - 对每轴：err = pos_set - pos_meas → PID_Update_Pos2Vel → vel_cmd（含限幅/滤波/斜率限制）
 *  - 若 axis[i].enabled==0，则强制 vel_cmd=0（安全/分轴调试）。
 */
void Control_TickAll(Control_t *c,
                          const float pos_meas[APP_MOTORS],
                          float       vel_out [APP_MOTORS]
                          )
{
    /* 1) 更新测量：若提供了 pos_meas，则同步到各轴的 pos_meas */
    if (pos_meas){
        for (int i=0;i<APP_MOTORS;++i){
            c->axis[i].pos_meas = pos_meas[i];
        }
    }

    /* 2) 逐轴计算速度命令：位置→速度（考虑使能） */
    for (int i=0;i<APP_MOTORS;++i){
        ControlAxis_t *ax = &c->axis[i];
        ax->vel_cmd = ax->enabled ? PID_Update_Pos2Vel(&ax->pid, ax->pos_set, ax->pos_meas) : 0.0f;
				
        /* 3) 可选返回：若调用方提供 vel_out 缓冲，则复制结果 */
        if (vel_out) {
            vel_out[i] = ax->vel_cmd;
        }
    }
}
