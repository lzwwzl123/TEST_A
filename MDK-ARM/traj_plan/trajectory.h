// =================================
// FILE: trajectory.h (Final Version)
// =================================

#ifndef TRAJECTORY_H
#define TRAJECTORY_H
#include "app_config.h"
#ifdef __cplusplus
extern "C" {
#endif

// 几何点和位姿的基本定义
typedef struct { float x, y; } TP2_Point;
typedef struct { float x, y, yaw; } TP2_Pose;


// 这将是轨迹求值函数的最终输出
typedef struct {
    TP2_Pose pos;      // 位置 (x, y, yaw)
    TP2_Pose vel;      // 速度 (vx, vy, vyaw)
    TP2_Pose acc;      // 加速度 (ax, ay, ayaw)
} TP2_MotionState;


// 三次多项式 q(t) = a0 + a1*t + a2*t^2 + a3*t^3
typedef struct { float a0, a1, a2, a3; } TP2_Cubic;

// 函数：创建、求值、求一阶导数、[新增]求二阶导数
TP2_Cubic TP2_CubicMake(float q0, float qf, float v0, float vf, float T);
float     TP2_CubicEval (TP2_Cubic c, float t);
float     TP2_CubicEvalD(TP2_Cubic c, float t);
float     TP2_CubicEvalDD(TP2_Cubic c, float t); // <--- 新增二阶导数求值

/* -------- 直线轨迹对象 -------- */
typedef struct {
    float dt, T;
    TP2_Point p0, dir;
    float     dist;
    TP2_Cubic s_cubic;
    TP2_Cubic yaw_cubic;
} TP2_LinePlan;

void TP2_LinePlanInit(TP2_LinePlan *L, float dt, TP2_Point p0, TP2_Point p1, float T,
                      float v0, float vf, float yaw0, float yawf, float wy0, float wyf);

// [修改] 输出参数类型变为 TP2_MotionState
void TP2_LineEvalAt(const TP2_LinePlan *L, float t, TP2_MotionState *state);


/* -------- 圆弧轨迹对象 -------- */
typedef struct {
    float dt, T;
    TP2_Point c;
    float R;
		float dist;
    TP2_Cubic phi_cubic;
    TP2_Cubic yaw_cubic;
} TP2_ArcPlan;

void TP2_ArcPlanInit(TP2_ArcPlan *C, float dt, TP2_Point center, float radius, float phi0, float phi1, float T,
                     float w0, float wf, float yaw0, float yawf, float wy0, float wyf);

// 输出参数类型变为 TP2_MotionState
void TP2_ArcEvalAt(const TP2_ArcPlan *C, float t, TP2_MotionState *state);


#ifdef __cplusplus
}
#endif
#endif // TRAJECTORY_H
