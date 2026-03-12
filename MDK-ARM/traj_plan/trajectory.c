// =================================
// FILE: trajectory.c (Final Version)
// =================================

#include "trajectory.h"
#include <math.h>
#include <string.h> // For memset

/* ----------- 三次多项式基础 ----------- */

TP2_Cubic TP2_CubicMake(float q0, float qf, float v0, float vf, float T){
    if (T <= 1e-6f) T = 1e-6f;
    TP2_Cubic c;
    c.a0 = q0;
    c.a1 = v0;
    c.a2 = (3.0f*(qf - q0)/(T*T)) - (2.0f*v0/T) - (vf/T);
    c.a3 = (2.0f*(q0 - qf)/(T*T*T)) + ((v0 + vf)/(T*T));
    return c;
}

float TP2_CubicEval (TP2_Cubic c, float t){ return c.a0 + c.a1*t + c.a2*t*t + c.a3*t*t*t; }
float TP2_CubicEvalD(TP2_Cubic c, float t){ return c.a1 + 2.0f*c.a2*t + 3.0f*c.a3*t*t; }

// 三次多项式二阶导数
float TP2_CubicEvalDD(TP2_Cubic c, float t){ return 2.0f*c.a2 + 6.0f*c.a3*t; }


/* ----------- 直线轨迹 ----------- */

static inline float hypot2(float x, float y){ return sqrtf(x*x + y*y); }

void TP2_LinePlanInit(TP2_LinePlan *L, float dt, TP2_Point p0, TP2_Point p1, float T,
                      float v0, float vf, float yaw0, float yawf, float wy0, float wyf)
{
    L->dt = (dt > 0.0f) ? dt : 1e-3f;
    L->T  = (T > 0.0f) ? T  : 1e-3f;
    L->p0 = p0;

    float dx = p1.x - p0.x, dy = p1.y - p0.y;
    float d  = hypot2(dx,dy);
    L->dist = d;
    L->dir = (d > 1e-6f) ? (TP2_Point){dx/d, dy/d} : (TP2_Point){0.0f, 0.0f};

    L->s_cubic   = TP2_CubicMake(0.0f, d, v0, vf, L->T);
    L->yaw_cubic = TP2_CubicMake(yaw0, yawf, wy0, wyf, L->T);
}

// [修改] 计算完整的运动状态
void TP2_LineEvalAt(const TP2_LinePlan *L, float t, TP2_MotionState *state) {
    if (t < 0.0f) t = 0.0f;
    if (t > L->T) t = L->T;
    if (!state) return;
    
    memset(state, 0, sizeof(TP2_MotionState));

    // 1. 计算路径参数 s(t) 及其一阶、二阶导数
    float s      = TP2_CubicEval  (L->s_cubic, t);
    float s_dot  = TP2_CubicEvalD (L->s_cubic, t);
    float s_ddot = TP2_CubicEvalDD(L->s_cubic, t);

    // 2. 计算 yaw(t) 及其一阶、二阶导数
    float yaw      = TP2_CubicEval  (L->yaw_cubic, t);
    float yaw_dot  = TP2_CubicEvalD (L->yaw_cubic, t);
    float yaw_ddot = TP2_CubicEvalDD(L->yaw_cubic, t);

    // 3. 合成最终的运动状态
    // 位置: P = P0 + s * dir
    state->pos.x = L->p0.x + s * L->dir.x;
    state->pos.y = L->p0.y + s * L->dir.y;
    state->pos.yaw = yaw;

    // 速度: V = s_dot * dir
    state->vel.x = s_dot * L->dir.x;
    state->vel.y = s_dot * L->dir.y;
    state->vel.yaw = yaw_dot;

    // 加速度: A = s_ddot * dir
    state->acc.x = s_ddot * L->dir.x;
    state->acc.y = s_ddot * L->dir.y;
    state->acc.yaw = yaw_ddot;
}


/* ----------- 圆弧轨迹 ----------- */

void TP2_ArcPlanInit(TP2_ArcPlan *C, float dt, TP2_Point center, float radius, float phi0, float phi1, float T,
                     float w0, float wf, float yaw0, float yawf, float wy0, float wyf)
{
    C->dt = (dt > 0.0f) ? dt : 1e-3f;
    C->T  = (T > 0.0f) ? T  : 1e-3f;
    C->c  = center;
    C->R  = (radius > 0.0f) ? radius : 1e-6f;
		C->dist = C->R * 2 * M_PI;

    C->phi_cubic = TP2_CubicMake(phi0, phi1, w0,  wf,  C->T);
    C->yaw_cubic = TP2_CubicMake(yaw0, yawf, wy0, wyf, C->T);
}

// [修改] 计算完整的运动状态
void TP2_ArcEvalAt(const TP2_ArcPlan *C, float t, TP2_MotionState *state) {
    if (t < 0.0f) t = 0.0f;
    if (t > C->T) t = C->T;
    if (!state) return;

    memset(state, 0, sizeof(TP2_MotionState));

    // 1. 计算路径参数 phi(t) 及其一阶、二阶导数
    float phi      = TP2_CubicEval  (C->phi_cubic, t);
    float phi_dot  = TP2_CubicEvalD (C->phi_cubic, t);
    float phi_ddot = TP2_CubicEvalDD(C->phi_cubic, t);

    // 2. 计算 yaw(t) 及其一阶、二阶导数
    float yaw      = TP2_CubicEval  (C->yaw_cubic, t);
    float yaw_dot  = TP2_CubicEvalD (C->yaw_cubic, t);
    float yaw_ddot = TP2_CubicEvalDD(C->yaw_cubic, t);

    // 3. 合成最终的运动状态 (应用链式法则)
    float cos_phi = cosf(phi);
    float sin_phi = sinf(phi);

    // 位置: P = Center + R * [cos(phi), sin(phi)]
    state->pos.x = C->c.x + C->R * cos_phi;
    state->pos.y = C->c.y + C->R * sin_phi;
    state->pos.yaw = yaw;

    // 速度: V = R * phi_dot * [-sin(phi), cos(phi)]
    state->vel.x = -C->R * phi_dot * sin_phi;
    state->vel.y =  C->R * phi_dot * cos_phi;
    state->vel.yaw = yaw_dot;

    // 加速度: A = A_tangential + A_centripetal
    // A = R*phi_ddot*[-sin,cos] - R*phi_dot^2*[cos,sin]
    state->acc.x = -C->R * phi_ddot * sin_phi - C->R * phi_dot * phi_dot * cos_phi;
    state->acc.y =  C->R * phi_ddot * cos_phi - C->R * phi_dot * phi_dot * sin_phi;
    state->acc.yaw = yaw_ddot;
}
