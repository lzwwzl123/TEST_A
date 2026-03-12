#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include "app_config.h"
#include "PID.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===========================================================================================
 * 模块说明（仅注释，不影响编译）
 * -------------------------------------------------------------------------------------------
 * 控制层职责：
 *   - 为每个轴提供“位置 → 速度”的闭环控制（Position controller 输出速度命令）。
 *   - 输入：目标位置 pos_set（相对零位）、测量位置 pos_meas（相对零位）。
 *   - 输出：速度命令 vel_cmd（写入通信层命令的 W 通道，下发到执行器）。
 *
 * 量纲/单位（建议统一，具体以项目实现为准）：
 *   - 位置（pos_*）：米 m（绳长/末端等效长度）或弧度 rad（若为电机角度），本文称“相对零位”。
 *   - 速度（speed_set / vel_cmd）：m/s 或 rad/s。
 *   - 控制周期 dt：秒 s（由定时器/采样频率给定）。
 *
 * 与其他层的接口：
 *   - App 层：在 App_Tick 中调用 Control_SetTargets/Control_TickAll；热更新 PID 用 Control_ConfigAxisPID。
 *   - 通信层：App_Tick 之后将 vel_cmd 写入 CommBus_GetCmd(i)->W，由 RS485 下发。
 * ===========================================================================================*/

/* ----------------------------------------------
 * 单轴控制对象：封装一次“位置→速度”闭环的状态
 * ----------------------------------------------*/
typedef struct {
    PID_t pid;          ///< 位置→速度 PID 内核（误差 = pos_set - pos_meas；输出 = vel_cmd）
    float pos_set;      ///< 目标位置（相对零位，单位：m 或 rad；由上层轨迹/插值给出）
    float speed_set;    ///< 目标速度（可选前馈/限速用途，单位：m/s 或 rad/s；如未用可置 0）
    float pos_meas;     ///< 测量位置（相对零位，单位：m 或 rad；由通信层测量并减零偏）
    float vel_cmd;      ///< 输出速度命令（单位：m/s 或 rad/s；写入 CommBus 命令 W）
    uint8_t enabled;    ///< 轴使能（1=闭环生效，0=不输出 vel→强制 0；用于分轴调试/安全）
} ControlAxis_t;

/* ----------------------------------------------
 * 多轴控制器：固定周期 dt，管理 APP_MOTORS 个轴
 * ----------------------------------------------*/
typedef struct {
    float dt;                         ///< 控制周期（秒），由上层初始化并保持恒定
    ControlAxis_t axis[APP_MOTORS];   ///< 各轴控制对象（索引 0..APP_MOTORS-1）
} Control_t;

/**
 * @brief  控制器初始化（设置周期 dt、清零各轴状态并初始化 PID 内核）
 * @param  c   控制器句柄
 * @param  dt  控制周期（秒）
 * @note   - 不修改外部目标；默认 vel_cmd=0、enabled 状态由上层决定。
 *         - PID 内核的限幅/滤波等默认参数可在 Control_ConfigAxisPID 中覆盖。
 */
void  Control_Init(Control_t *c, float dt);

/**
 * @brief  将第 i 轴的“控制器内部状态/位置引用”与实测位置对齐
 * @param  c        控制器
 * @param  i        轴索引（0..APP_MOTORS-1）
 * @param  pos_meas 当前实测相对位置
 * @note   - 用于归零/重定位后消除瞬时大误差/积分累积；通常同时清理 PID 内部状态。
 */
void  Control_SyncAxisToMeas(Control_t *c, int i, float pos_meas);

/**
 * @brief  批量对齐所有轴的测量位置（归零后常用）
 * @param  c         控制器
 * @param  pos_meas  所有轴的实测相对位置数组
 */
void  Control_SyncAllToMeas(Control_t *c, const float pos_meas[APP_MOTORS]);

/**
 * @brief  设置所有轴的目标位置（相对零位）
 * @param  c        控制器
 * @param  pos_set  目标位置数组（长度 APP_MOTORS）
 * @note   - 一般在轨迹插值后每周期更新；内部仅写 pos_set，不立即计算输出。
 */
void  Control_SetTargets(Control_t *c, const float pos_set[APP_MOTORS]);

/**
 * @brief  在一个周期内对所有轴执行控制计算（位置→速度）
 * @param  c         控制器
 * @param  pos_meas  实测位置数组（相对零位）
 * @param  vel_out   输出速度数组（结果指针；函数内写入 vel_cmd）
 * @note   - 对每个轴：err = pos_set - pos_meas → PID → vel_cmd（限幅/滤波/反饱和由 PID 决定）
 *         - 若 axis[i].enabled==0，则 vel_cmd 置 0（保持电机不动作）。
 */
void  Control_TickAll(Control_t *c,
                           const float pos_meas[APP_MOTORS],
                           float       vel_out [APP_MOTORS]);

/**
 * @brief  运行时修改某一轴 PID 参数与约束（不改变控制逻辑）
 * @param  c          控制器
 * @param  i          轴索引（0..APP_MOTORS-1）
 * @param  kp,ki,kd   PID 增益
 * @param  d_alpha    D 项一阶滤波系数（0..1，越小越平滑）
 * @param  out_min    输出下限（速度命令最小值）
 * @param  out_max    输出上限（速度命令最大值）
 * @param  accel_max  输出/速度的加速度限幅（斜率约束；单位依实现）
 * @param  use_cond_i 条件积分开关（0/1；接近目标时才积分，抗风up）
 * @param  u_init     控制器输出初值（用于切换/复位的平滑过渡）
 * @param  i_deadband 积分死区（|err|<阈值不积分，减小抖动）
 * @param  kp_scale_threshold KP 缩放触发阈值（接近目标时降低 KP）
 * @param  kp_near    近区 KP 或缩放后的系数（与实现约定）
 * @note   - 请在控制周期边界调用，或内部做互斥，避免与 Tick 并发写同一轴。
 *         - out_min/out_max 建议对称设置，避免方向饱和偏置。
 */
void Control_ConfigAxisPID(Control_t *c, int i,
                           float kp, float ki, float kd,
                           float d_alpha,
                           float out_min, float out_max,
                           float accel_max, int use_cond_i,
                           float u_init, float i_deadband, 
                           float kp_scale_threshold, float kp_near);

/**
 * @brief  轴使能/失能（内联小函数）
 * @param  c   控制器
 * @param  i   轴索引（0..APP_MOTORS-1）
 * @param  en  0=关（输出强制 0），非 0=开（允许闭环输出）
 * @note   失能时建议上层同时处理执行器侧的使能/抱闸/安全逻辑。
 */
static inline void Control_EnableAxis(Control_t *c, int i, uint8_t en){
    if (i<0 || i>=APP_MOTORS) return;
    c->axis[i].enabled = (en!=0);
}

#ifdef __cplusplus
}
#endif
#endif // CONTROL_H
