

#include "cdpr.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>
volatile float g_debug_yaw_input = 0.0f;
volatile float g_debug_W_matrix[3][4] = {0};
// =============================================================================
// 内部常量和宏定义
// =============================================================================
#ifndef PI
#define PI 3.14159265358979323846f
#endif
#ifndef INFINITY
#define INFINITY (1.0f/0.0f)
#endif
#define TOL 1e-10f
// 定义全局的调试变量
ForceErrorHistory_t g_error_history;
// =============================================================================
// 内部数据结构
// =============================================================================
typedef struct {
    float center[2];
    float radius;
} Circle_t;

// [新] 定义物理缠绕方向
typedef enum {
    WINDING_CCW, // 逆时针 (Counter-Clockwise)
    WINDING_CW   // 顺时针 (Clockwise)
} WindingDirection_t;

// [新] 简化的、无歧义的切线解结构体
typedef struct {
    bool valid;
    float points[2][2]; // [0]->电机切点(ap), [1]->平台切点(ep)
} TangentSolution_t;


// =============================================================================
// 内部(静态)辅助函数的原型声明 (已更新)
// =============================================================================
static float hypot_robust(float x, float y);
static int isfinite_f(float x);
static void find_tangents_by_winding(Circle_t s[], Circle_t *t, const WindingDirection_t req_windings[], TangentSolution_t solutions[]);
static float calculate_total_length(const Pose_t* p, const CDPR_Params_t* cdpr, const float ap[2], const float ep[2], int idx);


// =============================================================================
// 公共函数的实现
// =============================================================================

void init_cdpr_params(CDPR_Params_t *para_cdpr) {
    float bp_x = 0.19f; float bp_y = 0.19f;
    para_cdpr->bp[0][0] = bp_x;   para_cdpr->bp[1][0] = bp_y;
    para_cdpr->bp[0][1] = -bp_x;  para_cdpr->bp[1][1] = bp_y;
    para_cdpr->bp[0][2] = -bp_x;  para_cdpr->bp[1][2] = -bp_y;
    para_cdpr->bp[0][3] = bp_x;   para_cdpr->bp[1][3] = -bp_y;
    para_cdpr->M_ee = 0.25f; para_cdpr->r_ee = 0.03f;
    para_cdpr->I_ee = 0.000225f; para_cdpr->r_motor = 0.055f;
	  para_cdpr->t_min = 1.0f; 
}

bool solve_cable_lengths(const Pose_t *pose, const CDPR_Params_t *para_cdpr, CableLengthSolution_t *solution)
{
    const WindingDirection_t physical_windings[4] = {
        WINDING_CCW,  // 缆绳 0 (电机1)
        WINDING_CW, // 缆绳 1 (电机2)
        WINDING_CCW,  // 缆绳 2 (电机3)
        WINDING_CW  // 缆绳 3 (电机4)
    };

    Circle_t source_circles[4];
    for (int i = 0; i < 4; i++) {
        source_circles[i].center[0] = para_cdpr->bp[0][i];
        source_circles[i].center[1] = para_cdpr->bp[1][i];
        source_circles[i].radius = para_cdpr->r_motor;
    }
    Circle_t target_circle = { .center = {pose->x, pose->y}, .radius = para_cdpr->r_ee };
    
    TangentSolution_t tangent_solutions[4];
    find_tangents_by_winding(source_circles, &target_circle, physical_windings, tangent_solutions);

    for (int i = 0; i < 4; i++) {
        if (!tangent_solutions[i].valid) {
            solution->exitflag = -1; return false;
        }

        const float* ap = tangent_solutions[i].points[0];
        const float* ep = tangent_solutions[i].points[1];

        solution->cable_length[i] = calculate_total_length(pose, para_cdpr, ap, ep, i);
        
        if (!isfinite_f(solution->cable_length[i])) {
            solution->exitflag = -2; return false;
        }
    }
    solution->exitflag = 1;
    return true;
}

static float hypot_robust(float x, float y) { return sqrtf(x*x + y*y); }
static int isfinite_f(float x) { return (x == x) && (x != INFINITY) && (x != -INFINITY); }

// 核心几何计算函数
static void find_tangents_by_winding(Circle_t source_circles[], Circle_t* target_circle, const WindingDirection_t required_windings[], TangentSolution_t solutions[])
{
    float C2[2] = {target_circle->center[0], target_circle->center[1]};
    float r2 = target_circle->radius;

    for (int i = 0; i < 4; i++) {
        solutions[i].valid = false;

        float C1[2] = {source_circles[i].center[0], source_circles[i].center[1]};
        float r1 = source_circles[i].radius;

        float dvec[2] = {C2[0] - C1[0], C2[1] - C1[1]};
        float d = hypot_robust(dvec[0], dvec[1]);
        float dr = r1 - r2;

        if (!isfinite_f(d) || d < fabsf(dr) - TOL) { continue; }

        // 1. 计算两条几何候选切线 (pair1, pair2)
        float gamma = atan2f(dvec[1], dvec[0]);
        float denom2 = fmaxf(d*d - dr*dr, 0.0f);
        float denom = sqrtf(denom2);
        float alpha_geom = atan2f(fabsf(dr), denom);

        float theta_1 = gamma + (PI/2.0f - alpha_geom);
        float theta_2 = gamma - (PI/2.0f - alpha_geom);

        float n1[2] = {cosf(theta_1), sinf(theta_1)};
        float n2[2] = {cosf(theta_2), sinf(theta_2)};

        float pair1[2][2] = { {C1[0] + r1*n1[0], C1[1] + r1*n1[1]}, {C2[0] + r2*n1[0], C2[1] + r2*n1[1]} };
        float pair2[2][2] = { {C1[0] + r1*n2[0], C1[1] + r1*n2[1]}, {C2[0] + r2*n2[0], C2[1] + r2*n2[1]} };

        // 2. 确定第一个候选 (pair1) 的缠绕方向
        float r_vec_x1 = pair1[0][0] - C1[0]; // 半径向量 r_vec
        float r_vec_y1 = pair1[0][1] - C1[1];
        float L_vec_x1 = pair1[1][0] - pair1[0][0]; // 缆绳向量 L_vec
        float L_vec_y1 = pair1[1][1] - pair1[0][1];
        float cross_product1 = r_vec_x1 * L_vec_y1 - r_vec_y1 * L_vec_x1;
        
        WindingDirection_t pair1_winding = (cross_product1 > 0) ? WINDING_CCW : WINDING_CW;

        // 3. 根据必需的物理缠绕方向，选择正确的一对
        if (pair1_winding == required_windings[i]) {
            memcpy(solutions[i].points, pair1, sizeof(pair1));
        } else {
            memcpy(solutions[i].points, pair2, sizeof(pair2));
        }
        
        solutions[i].valid = true;
    }
}

static float calculate_total_length(const Pose_t* pose, const CDPR_Params_t* para_cdpr, const float ap[2], const float ep[2], int cable_index) {

    float dx = ap[0] - ep[0];
    float dy = ap[1] - ep[1];
    float alpha = atan2f(dy, dx);
    float r_ee = para_cdpr->r_ee;
    float r_m = para_cdpr->r_motor;
    float theta = pose->theta;
    float L_line = hypot_robust(dx, dy);
    float L_delta_theta = (cable_index == 0 || cable_index == 2) ? (theta * r_ee) : (-theta * r_ee);
    float angle_ee;
    switch (cable_index) {
        case 0: angle_ee = PI/2.0f - alpha; break;
        case 1: angle_ee = -PI/2.0f + alpha; break;
        case 2: angle_ee = -alpha - PI/2.0f; break;
        case 3: angle_ee = alpha + PI/2.0f; break;
        default: angle_ee = 0.0f;
    }
    float L_arc_ee = angle_ee * r_ee;
    float L_arc_motor = (PI/2.0f - angle_ee) * r_m;
    return L_line + L_delta_theta + L_arc_ee + L_arc_motor;
}

/**
 * @brief 使用伪逆和零空间法求解索力。
 *
 * 该函数计算产生期望平台加速度所需的索力。
 * 它通过零空间调整，确保计算出的索力均为正值（大于一个最小阈值），
 * 这满足了绳索只能拉不能推的物理约束。
 *
 * @param motion 平台的期望运动状态 (位置, 速度, 加速度)。
 * @param para_cdpr 机器人的物理参数。
 * @param solution 用于存储结果的输出结构体 (包含成功标志和索力数组)。
 * @return 如果找到有效的解，返回 true，否则返回 false。
 */
bool solve_cable_forces(const TP2_MotionState* motion, 
                        const CDPR_Params_t* para_cdpr, 
                        CableForceSolution_t* solution)
{
    // --- 1. 获取运动学几何信息 (ap 和 ep 点) ---
    // 为保持函数独立性，我们在这里重新计算它们。
    Pose_t current_pose = { .x = motion->pos.x, .y = motion->pos.y, .theta = motion->pos.yaw };
    
    const WindingDirection_t windings[4] = {WINDING_CCW, WINDING_CW, WINDING_CCW, WINDING_CW};
    Circle_t sources[4];
    for (int i = 0; i < 4; i++) {
        sources[i].center[0] = para_cdpr->bp[0][i];
        sources[i].center[1] = para_cdpr->bp[1][i];
        sources[i].radius = para_cdpr->r_motor;
    }
    Circle_t target = { .center = {current_pose.x, current_pose.y}, .radius = para_cdpr->r_ee };
    TangentSolution_t tangents[4];
    find_tangents_by_winding(sources, &target, windings, tangents);

    // --- 2. 构建 3x4 的结构矩阵 W ---
    float W[3][4];
    for (int i = 0; i < 4; i++) {
        if (!tangents[i].valid) { solution->success = false; return false; }
        
        const float* ap = tangents[i].points[0];
        const float* ep = tangents[i].points[1];

        // 这代表从平台指向电机的向量
        float vec[2] = { ap[0] - ep[0], ap[1] - ep[1] };
        float len = hypot_robust(vec[0], vec[1]);
        if (len < TOL) { solution->success = false; return false; }
        
        // W的第一行：所有绳索单位向量的x分量
        W[0][i] = vec[0] / len;
        // W的第二行：所有绳索单位向量的y分量
        W[1][i] = vec[1] / len;
    }
    // W的第三行：每根绳索产生的力矩 (r x u)
    // 根据您的MATLAB代码 f_moment = r_ee*(cf(1)-cf(2)+cf(3)-cf(4))
    W[2][0] = para_cdpr->r_ee;
    W[2][1] = -para_cdpr->r_ee;
    W[2][2] = para_cdpr->r_ee;
    W[2][3] = -para_cdpr->r_ee;

    // --- 3. 构建 3x1 的广义力向量 F_ext ---
    // F_ext = [M*ax, M*ay, I*a_theta]'
    float F_ext[3] = {
        para_cdpr->M_ee * motion->acc.x,
        para_cdpr->M_ee * motion->acc.y,
        para_cdpr->I_ee * motion->acc.yaw
    };

    // --- 4. 计算伪逆 W_plus = W' * inv(W * W') ---
    float Wt[4][3];
    matrix_transpose_3x4(W, Wt);

    float WWt[3][3];
    matrix_mult_3x4_4x3(W, Wt, WWt);

    float WWt_inv[3][3];
    if (!matrix_invert_3x3(WWt, WWt_inv)) {
        solution->success = false; // 矩阵奇异, 无法求解
        return false;
    }

    float W_plus[4][3];
    matrix_mult_4x3_3x3(Wt, WWt_inv, W_plus);

    // --- 5. 计算特解: t_p = W_plus * F_ext ---
    float t_p[4];
    matrix_mult_4x3_3x1(W_plus, F_ext, t_p);

    // --- 6. 检查特解是否已经满足要求 (所有索力 > T_MIN) ---
    const float T_MIN = para_cdpr->t_min; // 保证绳索绷紧的最小张力 (N)
    bool is_valid = true;
    for (int i = 0; i < 4; i++) {
        if (t_p[i] < T_MIN) {
            is_valid = false;
            break;
        }
    }

    // 如果特解已经满足要求，直接返回，这是最理想的情况
    if (is_valid) {
        memcpy(solution->tensions, t_p, sizeof(t_p));
        solution->success = true;
        return true;
    }

    // --- 7. 如果特解不满足，计算零空间向量 v_n ---
    // 我们需要解 W * v_n = 0。由于W是3x4，零空间是1维的。
    // 一种方法是：将 W 分解为 [W_3x3 | w4]，令 v_n 的第4个元素为1，
    // 然后解 W_3x3 * [v1,v2,v3]' = -w4。
    float W_3x3[3][3];
    float w4[3];
    for(int r=0; r<3; ++r) {
        w4[r] = W[r][3];
        for(int c=0; c<3; ++c) {
            W_3x3[r][c] = W[r][c];
        }
    }

    float W_3x3_inv[3][3];
    if (!matrix_invert_3x3(W_3x3, W_3x3_inv)) {
        solution->success = false; // 无法用此方法找到零空间
        return false;
    }

    float v_n_part[3];
    matrix_mult_3x3_3x1(W_3x3_inv, w4, v_n_part);

    float v_n[4] = { -v_n_part[0], -v_n_part[1], -v_n_part[2], 1.0f };
    
    // --- 8. 寻找标量 k 的可行范围，以满足 t_p + k * v_n >= T_MIN ---
    float k_min = -INFINITY;
    float k_max = INFINITY;

    for (int i = 0; i < 4; i++) {
        if (fabsf(v_n[i]) > TOL) {
            // 从 t_p[i] + k * v_n[i] >= T_MIN 求解 k
            float k_bound = (T_MIN - t_p[i]) / v_n[i];
            if (v_n[i] > 0) { // 不等式方向不变
                if (k_bound > k_min) k_min = k_bound;
            } else { // v_n[i] < 0, 不等式方向改变
                if (k_bound < k_max) k_max = k_bound;
            }
        } else {
            // 如果 v_n[i] 为零，则该索力无法通过零空间调整。
            // 如果此时 t_p[i] 已经小于 T_MIN，则无解。
            if (t_p[i] < T_MIN) {
                solution->success = false;
                return false;
            }
        }
    }

    // --- 9. 检查是否存在可行的 k ---
    if (k_min > k_max) {
        solution->success = false; // k 的可行域为空，无解
        return false;
    }

    // --- 10. 选择一个 k 并计算最终索力 ---
    // 选择 k = k_min 是最直接的策略，它代表了为满足约束而做出的最小调整。
    float k = k_min;
    for (int i = 0; i < 4; i++) {
        solution->tensions[i] = t_p[i] + k * v_n[i];
    }
    solution->success = true;
		
		float F_generated[3];
    matrix_mult_3x4_4x1(W, solution->tensions, F_generated);

// 计算误差 (保留符号，这更有用)
    float current_error_fx = F_generated[0] - F_ext[0];
    float current_error_fy = F_generated[1] - F_ext[1];
    float current_error_mz = F_generated[2] - F_ext[2]; // 增加力矩误差

    // 将本次计算的误差添加到历史记录中
    add_force_error(current_error_fx, current_error_fy, current_error_mz);
    return true;
}


bool calculate_jacobian(const Pose_t* current_pose,
                        const CDPR_Params_t* para_cdpr,
                        float jacobian_out[4][3])
{
    // 步骤 1: 计算 3x4 的结构矩阵 W。
    // W 的每一列代表一根缆绳对末端产生的单位力和力矩。
    float W[3][4];
    {
        const WindingDirection_t windings[4] = {WINDING_CCW, WINDING_CW, WINDING_CCW, WINDING_CW};
        Circle_t sources[4];
        for (int i = 0; i < 4; i++) {
            sources[i].center[0] = para_cdpr->bp[0][i];
            sources[i].center[1] = para_cdpr->bp[1][i];
            sources[i].radius = para_cdpr->r_motor;
        }
        Circle_t target = { .center = {current_pose->x, current_pose->y}, .radius = para_cdpr->r_ee };
        TangentSolution_t tangents[4];
        
        find_tangents_by_winding(sources, &target, windings, tangents);

        for (int i = 0; i < 4; i++) {
            if (!tangents[i].valid) {
                return false; // 几何关系无效，无法计算
            }
            
            const float* ap = tangents[i].points[0];
            const float* ep = tangents[i].points[1];

            float vec[2] = { ap[0] - ep[0], ap[1] - ep[1] };
            float len = hypot_robust(vec[0], vec[1]);
            
            if (len < TOL) {
                return false; // 缆绳长度过短，数值不稳定
            }
 
            W[0][i] = -vec[0] / len;
            W[1][i] = -vec[1] / len;
						
							
        }
        // W 的第三行是每根缆绳产生的力矩 (r_i x u_i)
        W[2][0] = para_cdpr->r_ee;
        W[2][1] = -para_cdpr->r_ee;
        W[2][2] = para_cdpr->r_ee;
        W[2][3] = -para_cdpr->r_ee;
    }

    // 步骤 2: 将结构矩阵 W (3x4) 转置，得到雅可比矩阵 J (4x3)。
    // J = W^T
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 3; ++j) {
            jacobian_out[i][j] = W[j][i];
        }
    }

    return true;
}

// =============================================================================
// 【【【 新增的静态矩阵运算辅助函数 】】】
// =============================================================================



void matrix_mult_3x4_4x1(float A[3][4], float B[4], float C[3]) {
    for (int i = 0; i < 3; i++) {
        C[i] = 0.0f;
        for (int j = 0; j < 4; j++) {
            C[i] += A[i][j] * B[j];
        }
    }
}


void matrix_transpose_3x4( float A[3][4], float At[4][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            At[j][i] = A[i][j];
        }
    }
}

void matrix_mult_3x4_4x3( float A[3][4],  float B[4][3], float C[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            C[i][j] = 0;
            for (int k = 0; k < 4; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

bool matrix_invert_3x3( float A[3][3], float A_inv[3][3]) {
    float det = A[0][0] * (A[1][1] * A[2][2] - A[2][1] * A[1][2]) -
                A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
                A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);

    if (fabsf(det) < TOL) {
        return false; // 矩阵奇异
    }

    float inv_det = 1.0f / det;
    A_inv[0][0] = (A[1][1] * A[2][2] - A[2][1] * A[1][2]) * inv_det;
    A_inv[0][1] = (A[0][2] * A[2][1] - A[0][1] * A[2][2]) * inv_det;
    A_inv[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) * inv_det;
    A_inv[1][0] = (A[1][2] * A[2][0] - A[1][0] * A[2][2]) * inv_det;
    A_inv[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) * inv_det;
    A_inv[1][2] = (A[1][0] * A[0][2] - A[0][0] * A[1][2]) * inv_det;
    A_inv[2][0] = (A[1][0] * A[2][1] - A[2][0] * A[1][1]) * inv_det;
    A_inv[2][1] = (A[2][0] * A[0][1] - A[0][0] * A[2][1]) * inv_det;
    A_inv[2][2] = (A[0][0] * A[1][1] - A[1][0] * A[0][1]) * inv_det;
    return true;
}

void matrix_mult_4x3_3x3( float A[4][3],  float B[3][3], float C[4][3]) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            C[i][j] = 0;
            for (int k = 0; k < 3; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

void matrix_mult_4x3_3x1( float A[4][3],  float v[3], float result[4]) {
    for (int i = 0; i < 4; i++) {
        result[i] = 0;
        for (int j = 0; j < 3; j++) {
            result[i] += A[i][j] * v[j];
        }
    }
}

void matrix_mult_3x3_3x1( float A[3][3], float v[3], float result[3]) {
    for (int i = 0; i < 3; i++) {
        result[i] = 0;
        for (int j = 0; j < 3; j++) {
            result[i] += A[i][j] * v[j];
        }
    }
}



// 初始化缓冲区
void init_error_history(void) {
    memset(&g_error_history, 0, sizeof(ForceErrorHistory_t));
}

// 向缓冲区中添加一条新的误差记录
void add_force_error(float fx, float fy, float mz) {
    // 在当前索引位置写入新数据
    g_error_history.error_fx[g_error_history.write_index] = fx;
    g_error_history.error_fy[g_error_history.write_index] = fy;
    g_error_history.error_mz[g_error_history.write_index] = mz;

    // 移动写入索引，如果到达末尾则绕回到开头
    g_error_history.write_index++;
    if (g_error_history.write_index >= ERROR_HISTORY_SIZE) {
        g_error_history.write_index = 0;
    }

    // 更新计数器
    if (g_error_history.count < ERROR_HISTORY_SIZE) {
        g_error_history.count++;
    }
}


