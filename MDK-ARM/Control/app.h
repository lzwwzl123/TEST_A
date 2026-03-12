// =================================
// FILE: app.h (Final Version)
// =================================

#ifndef APP_H
#define APP_H

#include <stdint.h>
#include <stdbool.h> // For bool type
#include <math.h>
#include <string.h>
#include <stdint.h>
#include "app_config.h"
#include "trajectory.h"
#include "control.h"
#include "comm_rs485.h"
#include "cdpr.h"
#ifdef __cplusplus
extern "C" {
#endif


/* ===========================================================================================
 * -------------------------------------------------------------------------------------------
 * 本模块封装“应用层”数据结构与接口，负责将运动学/轨迹、控制器以及通信总线（RS485）粘合在一起：
 *   - App_t：应用全局状态（周期时间 dt、控制器、通信、零位/测量/目标、轨迹缓存、运行状态等）
 *   - DebugPanel_t：用于 IDE/上位机在运行时修改的参数集合（轨迹、PID、轴使能、归零等）
 *   - MotionParams：轨迹参数（直线/圆弧），以及采样/持续时间
 *   - 接口函数：初始化、归零、设置轨迹、周期调度（Tick）、主循环（看门狗/重建轨迹等）
 *
 * RS485 要点（与 comm_rs485.h/实现配合）：
 *   - 半双工：发送时拉高 DE（或由硬件自动）；发送完成后按需等待回转时间再进入接收
 *   - 多节点：node_ids 指定 4 个电机从站地址（0..3 仅示例），支持轮询/广播
 *   - 建议在 Tick 内完成“采样→控制→限幅/反饱和→打包帧→下发”，确保固定周期
 * ===========================================================================================*/

/* --------------------------------------
 * 运动模式枚举：与 MotionParams::mode 对应
 * --------------------------------------*/
typedef enum { MODE_NONE=0, MODE_LINE=1, MODE_ARC=2 } MotionMode;

/* ---------------------------------------------------------------------------
 * @brief  轨迹参数（期望）
 *        - LINE：使用 L_p0/L_p1 + L_yaw0/L_yawf
 *        - ARC ：使用 C_center/C_R + C_phi0/C_phi1 + C_yaw0/C_yawf
 *        - freq：轨迹求值频率（Hz），与控制频率协调
 *        - points_duration：采样点“持续时长”参数（具体语义由 trajectory 模块解释）
 * ---------------------------------------------------------------------------*/
typedef struct {
    int   mode;                        ///< 运动模式（MODE_LINE/MODE_ARC）
    TP2_Point L_p0, L_p1;              ///< 直线：起点/终点（单位：米，世界坐标）
    float     L_yaw0, L_yawf;          ///< 直线：起始/终止偏航角（单位：rad）
    TP2_Point C_center; float C_R;     ///< 圆弧：圆心（米）与半径 R（米）
    float     C_phi0, C_phi1;          ///< 圆弧：起始/终止弧度（单位：rad）
    float     C_yaw0, C_yawf;          ///< 圆弧：起始/终止偏航角（单位：rad）
    int   freq;                        ///< 轨迹求值频率（Hz），影响 pose_kp/motor_kp 的采样间隔
    int   points_duration;             ///< 采样/片段持续时长参数（与 MAX_FREQ、kp_len/motor_len 相关）
} MotionParams;

/* -----------------------------
 * 运动状态机（仅运动相关）
 * -----------------------------*/
typedef enum {
    MOTION_IDLE,                       ///< 空闲（未运行轨迹/等待启动）
    MOTION_RUNNING,                    ///< 正在运行（Tick 周期推进）
} MotionState_t;

/* -----------------------------
 * 应用状态机（系统分层）
 * -----------------------------*/
typedef enum {
    APP_STATE_INIT,                    ///< 初始化阶段（外设/参数未就绪）
    APP_STATE_READY,                   ///< 就绪（可接受指令/运行）
} AppState_t;

/* ------------------------------------------------------------------------------------------------
 * @brief  应用全局对象（由 App_Init 构造、在 Tick/主循环中维护）
 * ------------------------------------------------------------------------------------------------*/
typedef struct {
    float        dt;                                   ///< 控制周期（秒），由定时器配置推导
    Control_t    ctl;                                  ///< 轴级控制器对象（PID/限幅等）
    CommBus_t    bus;                                  ///< 通信总线（RS485）对象
    float        zero_off[APP_MOTORS];                 ///< 零偏（米），归零后 pos_rel = pos_abs - zero_off
    float        pos_meas_abs[APP_MOTORS];             ///< 实测绝对位置/长度（米）
    float        pos_meas_rel[APP_MOTORS];             ///< 实测相对位置/长度（米）
    float        pos_set_rel[APP_MOTORS];              ///< 目标相对位置/长度（米）
    float        speed_set_rel[APP_MOTORS];            ///< 目标相对速度（量纲依实现，常为 m/s 或 rad/s）
    float        vel_out[APP_MOTORS];                  ///< 控制器输出（下发给执行器的控制量）
		Pose_t 			 current_pos;
    MotionParams desired, applied;                     ///< desired：界面期望；applied：已应用到系统的参数
    float        debug_cable_lengths[APP_MOTORS];      ///< 调试用绳长快照（米）
    TP2_LinePlan line;                                 ///< 直线轨迹规划结果
    TP2_ArcPlan  arc;                                  ///< 圆弧轨迹规划结果
    TP2_MotionState     pose_kp[MAX_FREQ];             ///< 逐点姿态序列（长度 kp_len）
		float 			 motor_torque[APP_MOTORS][MAX_FREQ];	 ///< 电机逐点力矩序列（长度 kp_len）
    int          kp_len;                               ///< pose_kp 的有效长度（<= MAX_FREQ）
    float        motor_kp[APP_MOTORS][MAX_FREQ];       ///< 每电机逐点目标（长度 motor_len）
    int          motor_len;                            ///< motor_kp 的有效长度（<= MAX_FREQ）
		// 速度估计
		float vel_meas[APP_MOTORS];       // 滤波后的测量速度（给控制或监控用）
		float vel_meas_raw[APP_MOTORS];   // 未滤波的原始速度（可选：仅监控/调试）
		float pos_meas_rel_prev[APP_MOTORS]; // 上次的相对位置（用于差分）
		float vel_alpha;                  								 /// 一阶低通的系数（0..1），越大响应越快、噪声越大// 直接给个 0.1~0.3，或按截止频率算
    volatile int   flag;                               ///< 通用标志位（中断/主循环共享）
    volatile int   count;                              ///< 通用计数器（中断/主循环共享）
    volatile int   points_duration;                    ///< 采样/片段时长（运行期可被修改）
    volatile int   freq;                               ///< 轨迹求值频率（Hz，运行期可被修改）
    float          total_time_s;                       ///< 当前轨迹总时长（秒）
    MotionState_t  motion_state;                       ///< 运动状态
    AppState_t     state;                              ///< 应用状态
    bool           motor_com_ok[APP_MOTORS];           ///< 与 4 台电机的通信健康状态
} App_t;

/* --------------------------------------------------------------------------------------------
 * @brief  轴级 PID 面板参数（运行期可热更新；与 Control_ConfigAxisPID 对应）
 * --------------------------------------------------------------------------------------------
 * 字段含义：
 *   - kp,ki,kd           ：比例/积分/微分
 *   - d_alpha            ：D 项一阶滤波系数（0..1，越小越平滑）
 *   - out_min/out_max    ：控制输出限幅（防饱和/防突变）
 *   - accel_max          ：输出/速度的加速度限幅（抑制激进指令）
 *   - use_cond_i         ：条件积分开关（1=开启，靠近目标才积分）
 *   - u_init             ：控制器输出初始值（切换/复位时安全过渡）
 *   - i_deadband         ：积分死区（误差小于阈值不积累）
 *   - kp_scale_threshold ：KP 缩放触发阈值（接近目标时降低 KP）
 *   - kp_near            ：近区的 KP（或缩放系数，依实现）
 * --------------------------------------------------------------------------------------------*/
typedef struct {
    float   kp, ki, kd;                 ///< PID 增益
    float   d_alpha;                    ///< D 项滤波系数（0..1）
    float   out_min, out_max;           ///< 输出限幅
    float   accel_max;                  ///< 允许的最大加速度/斜率（单位依实现）
    uint8_t use_cond_i;                 ///< 条件积分（0/1）
    float   u_init;                     ///< 输出初值
    float   i_deadband;                 ///< 积分死区
    float   kp_scale_threshold;         ///< KP 缩放阈值
    float   kp_near;                    ///< 近区 KP/系数
} PIDPanel_t;

/* ------------------------------------------------------------------------------------------------
 * @brief  调试面板（IDE 可写入），用于“人工”驱动应用的状态变更/参数热更新
 * ------------------------------------------------------------------------------------------------
 * 典型工作流：
 *   1) 修改 line/arc/sampler/pid 等字段
 *   2) 将 apply_motion 或 apply_pid 置 1
 *   3) 主循环调用 ApplyDebugPanelIfNeeded()，把参数写入 App_t，并清零该标志
 * ------------------------------------------------------------------------------------------------*/
typedef struct {
    uint8_t    zero_now;                                    ///< 置 1 触发一次归零（写后由应用清零）
    int        mode;                                        ///< 当前选择的轨迹模式（1=直线，2=圆弧）

    uint8_t    error_code;                                  ///< 最近一次参数应用/执行的错误码（0=OK）
    uint8_t    apply_motion;                                ///< 置 1：应用轨迹参数到 app->desired
    uint8_t    start_motion;                                ///< 置 1：触发运动启动（是否使用由实现决定）

    struct { float p0x, p0y, p1x, p1y, yaw0, yawf; } line;  ///< 直线轨迹参数（米/rad）
    struct { float cx, cy, R, phi0, phi1, yaw0, yawf; } arc;///< 圆弧轨迹参数（米/rad）
    struct { int freq; int points_duration; uint8_t apply; } sampler; ///< 轨迹采样参数+应用标志

    PIDPanel_t pid;                                         ///< 统一 PID 面板（如需每轴不同可扩展为数组）
    uint8_t    apply_pid;                                   ///< 置 1：将 pid 写入四轴控制器

    uint8_t    axis_enable[APP_MOTORS];                     ///< 四轴使能（1=使能；0=失能）
} DebugPanel_t;

/* ===========================================================================================
 * 接口函数声明（仅注释，无实现改动）
 * ===========================================================================================*/

/**
 * @brief  初始化应用对象（构建控制/通信/轨迹上下文）
 * @param  app         应用句柄（由调用者提供内存）
 * @param  huart       用于 RS485 总线的 UART 句柄（如 USART1）
 * @param  rede_port   RS485 DE/RE 所在 GPIO 端口；若硬件自动/未接，传 NULL
 * @param  rede_pin    RS485 DE/RE 引脚号；若未接，传 0
 * @param  en_port     可选的指示/使能 GPIO 端口（如发送期间点亮 LED）；可为 NULL
 * @param  en_pin      对应引脚号（或 0）
 * @param  node_ids    4 个从站地址数组（长度 APP_MOTORS）
 * @note   - 若硬件 DE/RE 与串口自动管理，rede_* 可为 NULL/0；
 *         - 若 en_* 用作 TX 活动指示灯，请在实现中仅在发送窗口置高，避免干扰；
 *         - 内部会完成 bus/ctl/零偏等默认初始化。
 */
void App_Init(App_t *app,
              UART_HandleTypeDef *huart,
              GPIO_TypeDef *rede_port, uint16_t rede_pin,
              GPIO_TypeDef *en_port, uint16_t en_pin,
              const uint8_t *node_ids);

/**
 * @brief  立即将当前位置设为零（更新 zero_off[]，并复位相关控制器状态）
 * @param  app 应用句柄
 * @note   - 影响 pos_meas_rel = pos_meas_abs - zero_off；
 *         - 建议在机械静止、无张力异常时执行。
 */
void App_ZeroNow(App_t *app);

/**
 * @brief  配置轨迹采样参数（频率与片段持续时长）
 * @param  app               应用句柄
 * @param  freq              轨迹求值频率（Hz）
 * @param  points_duration   采样/片段持续时长参数（与 MAX_FREQ/kp_len 关联）
 * @note   修改后通常需要重建轨迹（BuildTrajectoryFromParams）。
 */
void App_ConfigSampler(App_t *app, int freq, int points_duration);

/**
 * @brief  配置直线轨迹参数（写入 app->desired）
 * @param  app   应用句柄
 * @param  p0    起点（米）
 * @param  p1    终点（米）
 * @param  yaw0  起始偏航角（rad）
 * @param  yawf  终止偏航角（rad）
 * @note   仅写入 desired，不立即重建；由外部调用 BuildTrajectoryFromParams。
 */
void App_SetLine(App_t *app, TP2_Point p0, TP2_Point p1, float yaw0, float yawf);

/**
 * @brief  配置圆弧轨迹参数（写入 app->desired）
 * @param  app    应用句柄
 * @param  center 圆心（米）
 * @param  R      半径（米）
 * @param  phi0   起始弧度（rad）
 * @param  phi1   终止弧度（rad）
 * @param  yaw0   起始偏航角（rad）
 * @param  yawf   终止偏航角（rad）
 * @note   仅写入 desired，不立即重建；由外部调用 BuildTrajectoryFromParams。
 */
void App_SetArc(App_t *app, TP2_Point center, float R, float phi0, float phi1, float yaw0, float yawf);

/**
 * @brief  将一组姿态序列（世界坐标）映射成电机长度/目标数组
 * @param  app        应用句柄
 * @param  pose_kp    姿态序列（长度 N）
 * @param  N          序列长度（<= MAX_FREQ）
 * @param  motor_kp   输出：每电机逐点目标数组 [APP_MOTORS][MAX_FREQ]
 * @note   进行逆运动学/几何映射；内部应处理可行域检查与限幅。
 */
void App_IK_FromPoseArray(App_t* app, const  TP2_MotionState* pose_kp, int N,
                          float motor_kp[APP_MOTORS][MAX_FREQ]);


/**
 * @brief  周期函数（在定时器中断中调用）：完成一次控制周期
 * @param  app 应用句柄
 * @note   典型流程：采样→计算（PID/张力分配）→限幅→下发（RS485）→更新状态/看门狗。
 */
void App_Tick(App_t *app);

/**
 * @brief  主循环轻量任务（非实时）：参数热更新后重建轨迹、通信看门狗等
 * @param  app 应用句柄
 * @note   与 App_Tick 解耦；避免在此执行耗时阻塞任务。
 */
void App_MainLoop(App_t *app);

/**
 * @brief  预览当前 MotionParams 的起点姿态（不改动 app）
 * @param  P   轨迹参数
 * @return 起点处 TP2_Pose
 * @note   可用于 UI/调试预览。
 */
TP2_Pose App_PreviewTrajectoryStart(const MotionParams *P);

/**
 * @brief  根据给定参数构建轨迹并写入 app（通常在参数变更后调用）
 * @param  app 应用句柄
 * @param  P   轨迹参数（通常为 app->applied 或 app->desired）
 * @note   构建 pose_kp/motor_kp、计算 kp_len/motor_len/total_time_s；需与 Tick 互斥。
 */
void BuildTrajectoryFromParams(App_t *app, const MotionParams *P);


void App_ID_FromPoseArray(App_t* app);

bool App_FK_FromMotorPos(const float motor_positions[APP_MOTORS],
                         Pose_t* pose_estimate);
																			

#ifdef __cplusplus
}
#endif
#endif // APP_H
