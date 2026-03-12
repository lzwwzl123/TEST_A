#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* =========================
 * 工程级别的配置参数
 * ========================= */

/// 电机数量（并联机器人此处为4）
#define APP_MOTORS                 4

/// 控制周期（Hz）——请与 TIM2 的中断频率保持一致
#define APP_CTRL_HZ                4000.0f
#define APP_CTRL_DT                (1.0f/APP_CTRL_HZ)

/// 通信参数：超时与上电预热轮数
#define COMM_SEND_TIMEOUT_MS       50
#define COMM_INIT_WARMUP_CYCLES    (APP_MOTORS * 500)

/// 轨迹采样：共有 freq 个“关键点”，每段跨 points_duration 个1ms周期
#define MAX_FREQ                   1000
#define APP_SAMPLER_FREQ_DEFAULT   150     // 默认关键点个数
#define APP_POINTS_DUR_DEFAULT     150     // 默认每段跨多少个1ms
/// PID 默认参数（可在运行时动态覆盖）
#define PID_KP_DEFAULT             10.0f
#define PID_KI_DEFAULT             1000.0f
#define PID_KD_DEFAULT              0.4f
#define PID_D_ALPHA_DEFAULT         0.001f  // D项一阶滤波系数（0~1，越小越重）
#define PID_OUT_MIN_DEFAULT       (-20.0f)
#define PID_OUT_MAX_DEFAULT         20.0f
#define PID_USE_COND_I_DEFAULT      1      // 条件积分（防风up）
#define PID_ACCEL_MAX_DEFAULT       100.0f   // 速度命令的加速度上限a_max，内部用于限Δu=a_max*dt
#define PID_I_DEADBAND							0.0001f
#define PID_KP_SCALE								4.5f
#define PID_KP_NEAR									6.5f
#define KP_MOTOR										0.25f
#define KD_MOTOR										0.25f


#define M_PI												3.1415926535897f


#endif // APP_CONFIG_H
