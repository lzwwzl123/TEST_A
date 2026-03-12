// =============================================================================
// FILE: cdpr.h (最终版)
// 描述:
// CDPR (缆绳驱动并联机器人) 运动学与动力学解算库的公共接口。
// 此头文件定义了外部代码与本模块交互所需的所有公共数据结构和函数原型。
// 内部实现细节（如切点计算）被封装在 cdpr.c 文件中。
// =============================================================================

#ifndef CDPR_H
#define CDPR_H

#include <stdbool.h>
#include "trajectory.h" // 引入包含 TP2_MotionState 的头文件

// =============================================================================
// 公共常量和数据结构定义
// =============================================================================

#define NUM_CABLES 4


// cdpr.h 或 debug_utils.h

#define ERROR_HISTORY_SIZE 300 // 定义你想保存的历史记录数量，100是一个不错的起点

typedef struct {
    float error_fx[ERROR_HISTORY_SIZE];
    float error_fy[ERROR_HISTORY_SIZE];
    float error_mz[ERROR_HISTORY_SIZE];
    int write_index; // 指示下一次写入的位置
    int count;       // 记录已存入的数据量，最多为 ERROR_HISTORY_SIZE
} ForceErrorHistory_t;

// 函数原型
void init_error_history(void);
void add_force_error(float fx, float fy, float mz);

// 声明一个全局的调试变量
extern ForceErrorHistory_t g_error_history;

// 动平台位姿
typedef struct {
    float x;
    float y;
    float theta; // 姿态角，单位：弧度
} Pose_t;

// CDPR固定物理参数
typedef struct {
    float bp[2][NUM_CABLES]; // 锚点(电机轮中心)坐标 [x/y][cable_index]
    float M_ee;              // 末端平台质量 (kg)
    float r_ee;              // 末端平台半径 (m)
    float I_ee;              // 末端平台转动惯量 (kg.m^2)
    float r_motor;           // 绕线圈(电机轮)半径 (m)
		float t_min;        // 要求的最小索力 (N), 防止缆绳松弛
} CDPR_Params_t;

// 结构体：绳长逆解的输出
typedef struct {
    float cable_length[NUM_CABLES];
    int exitflag; // 1: 成功, -1: 几何无解, -2: 数值计算错误
} CableLengthSolution_t;

// 结构体：张力逆解的输出
typedef struct {
    bool success;       // 如果找到了有效的正索力解，则为 true
    float tensions[4];  // 计算出的4根缆绳的张力 (单位: N)
} CableForceSolution_t;

// =============================================================================
// 公共函数原型声明
// =============================================================================

/**
 * @brief 初始化CDPR的固定物理参数。
 * @param params 指向要被初始化的参数结构体。
 */
void init_cdpr_params(CDPR_Params_t *params);

/**
 * @brief [几何逆解] 根据给定位姿计算所有缆绳的长度。
 * @param pose 指向目标位姿的结构体。
 * @param params 指向已初始化的机器人物理参数。
 * @param solution 指向用于存储计算结果的结构体。
 * @return 成功返回 true，失败返回 false。
 */
bool solve_cable_lengths(const Pose_t* pose, const CDPR_Params_t* params, CableLengthSolution_t* solution);

bool solve_cable_forces(const TP2_MotionState* motion, 
                        const CDPR_Params_t* para_cdpr, 
                        CableForceSolution_t* solution);

bool calculate_jacobian(const Pose_t* current_pose,
                        const CDPR_Params_t* para_cdpr,
                        float jacobian_out[4][3]);

												
void matrix_mult_3x4_4x1(float A[3][4], float B[4], float C[3]);

void matrix_transpose_3x4( float A[3][4], float At[4][3]);

void matrix_mult_3x4_4x3( float A[3][4],  float B[4][3], float C[3][3]);

bool matrix_invert_3x3( float A[3][3], float A_inv[3][3]);
												
void matrix_mult_4x3_3x3( float A[4][3],  float B[3][3], float C[4][3]);

void matrix_mult_4x3_3x1( float A[4][3],  float v[3], float result[4]);

void matrix_mult_3x3_3x1( float A[3][3], float v[3], float result[3]);

												
												
												
#endif // CDPR_H
