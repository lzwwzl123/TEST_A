#include "app.h"
#include <math.h>
#include <string.h>

/* ===========================================================================================
 * 作用：
 *   - 将“末端位姿轨迹（x,y,yaw）”离散点序列 → 映射为“四台电机的相对零位角度”序列 motor_kp[m][k]
 *   - 并提供一个辅助函数，从 motor_kp 序列计算电机角速度离散序列 motor_kv[m][k]
 *	 - 通过末端位置以及规划的加速度，计算绳子力
 * 与 CDPR（Cable-Driven Parallel Robot）几何/求解的关系：
 *   - 依赖 cdpr.h/.c 中的参数结构 CDPR_Params_t 与求解器 solve_cable()
 *   - g_L0[] 存储“零姿态（标定姿态）时的绳长”，用于把绝对绳长 → 相对长度（再映射为电机角）
 *   - s_ccw_to_len[] 定义“电机 CCW 角速度正方向”与“绳长的增减”的符号关系（不同卷绕方向可能不同）
 *
 * 单位/约定：
 *   - 末端位姿：pose_kp[i].x / .y 的单位由 POSE_XY_SCALE 决定（例如 mm 需 *0.001 变为 m）
 *   - 绳长/半径：米（m）
 *   - 电机角：弧度（rad）（角度→可通过外部显示时再换算）
 *   - GEAR：减速比（电机轴与输出轴/卷筒之间的传动比）
 *
 * ===========================================================================================*/

#define GEAR      6.33f      /* 减速比 G：输出角 = 电机角 * G；这里用于长度→转角的映射 */

/* ================= bridge block（让本文件在你的工程可编译） ================ */


/* 末端 (x,y) 的单位缩放：
 * - 若轨迹模块给出的 (x,y) 单位是“毫米”，请设 POSE_XY_SCALE=0.001f；
 * - 若轨迹模块已用“米”，则保持 1.0f。
 */
#ifndef POSE_XY_SCALE
#define POSE_XY_SCALE 1.0f
#endif

/* ====== CDPR 参数 & L0（零姿态绳长，单位米） ======
 * g_cdpr：几何与物理参数（锚点位置、卷筒半径 r_motor 等）由 init_cdpr_params 初始化
 * g_L0[]：零姿态（标定）时每根绳的绝对绳长，用于把“求解出的绝对绳长”转换为“相对长度”
 *         dL = L_abs - L0
 */
static CDPR_Params_t g_cdpr;
static float g_L0[NUM_CABLES] =  {0.33662331f, 0.33662331f, 0.33662331f, 0.33662331f }; /* TODO: 改成你的实测值 */



/* 绕线方向极性表：
 * s_ccw_to_len[m] = +1 表示“电机 CCW 角度增加 → 绳长增加（放绳/收绳视结构）”
 *                 = -1 表示“电机 CCW 角度增加 → 绳长减少”
 * 不同电机（卷筒安装方向）可能不同，因此为每根绳单独配置。
 */
static const float s_ccw_to_len[NUM_CABLES] = { +1.0f, -1.0f, +1.0f, -1.0f };



/* 运行时可更新 L0（标定或归零）：上位机/调试面板可调用，将当前“零姿态绳长”写入 */
void IK_CDPR_SetL0(const float L0_m[NUM_CABLES]){
    for (int i=0;i<NUM_CABLES;i++) g_L0[i] = L0_m[i];
		init_error_history(); 
}
/* 一次性初始化：确保 g_cdpr 中的几何参数已就绪（支架锚点/卷筒半径等） */
static inline void ensure_init(void){
    init_cdpr_params(&g_cdpr);
		const float initial_L0[NUM_CABLES] = { 0.33662331f, 0.33662331f, 0.33662331f, 0.33662331f };
    IK_CDPR_SetL0(initial_L0);
}
/* ===========================================================================================
 * 逆运动学主函数：末端关键点 → 四电机“相对零位角度序列”
 *
 * 输入：
 *   - app：应用全局（用于调试输出 app->debug_cable_lengths[]）
 *   - pose_kp[i]：离散末端位姿（x,y,yaw），长度 N
 *   - N：关键点个数（≤ MAX_FREQ）
 *
 * 输出：
 *   - motor_kp[m][i]：第 m 根绳（第 m 台电机）在第 i 个关键点的“相对零位角度”（rad）
 *     计算流程：   绝对绳长 L_abs[m]  → 相对长度 dL[m] = L_abs[m] - L0[m]
 *               → 电机角     θ[m] = sign[m] * dL[m] * GEAR / r_motor
 *
 * 连续性与可行性：
 *   - solve_cable(...) 传入 last_cable_lengths[] 作为“上一解”的初值，提升连续性/收敛性
 *   - 若求解失败（返回 false 或 exitflag!=1），则沿用上一点的 motor_kp[*][i]（保持轨迹连续）
 *
 * 调试：
 *   - 在每次求解成功后，把绝对绳长 sol.cable_length[m] 记录到 app->debug_cable_lengths[m]
 * ===========================================================================================*/
void App_IK_FromPoseArray(App_t* app,
                          const TP2_MotionState* pose_kp, int N,
                          float motor_kp[APP_MOTORS][MAX_FREQ])
{
    // 0) 基本校验与边界保护
    if (!pose_kp || !motor_kp) return;
    if (N <= 0) {
        // 没有有效点：整段置 0，并同步有效长度
        for (int m = 0; m < APP_MOTORS; ++m) {
            memset(&motor_kp[m][0], 0, sizeof(float) * MAX_FREQ);
        }
        if (app) app->motor_len = 0;
        return;
    }
    if (N > MAX_FREQ) N = MAX_FREQ;     // 保险：最多写到 MAX_FREQ

    // 1) 初始化
    ensure_init();
    CableLengthSolution_t sol;
    float last_cable_lengths[NUM_CABLES];
    for (int m = 0; m < NUM_CABLES; ++m) {
        last_cable_lengths[m] = g_L0[m];
    }

    // 2) 逐点进行逆运动学
    for (int i = 0; i < N; ++i) {
        Pose_t p;
        p.x     = pose_kp[i].pos.x * POSE_XY_SCALE;
        p.y     = pose_kp[i].pos.y * POSE_XY_SCALE;
        p.theta = pose_kp[i].pos.yaw;

        if (!solve_cable_lengths(&p, &g_cdpr, &sol) || sol.exitflag != 1) {
            // 求解失败：沿用上一点（i=0 则填 0）
            for (int m = 0; m < APP_MOTORS; ++m) {
                motor_kp[m][i] = (i > 0) ? motor_kp[m][i - 1] : 0.0f;
            }
            for (int m = 0; m < NUM_CABLES; ++m) {
                app->debug_cable_lengths[m] = (i > 0) ? last_cable_lengths[m] : g_L0[m];
            }
            continue;
        }

        // 求解成功：绝对绳长 -> 电机相对角度
        const float r = g_cdpr.r_motor;
        for (int m = 0; m < APP_MOTORS; ++m) {
            const float dL = sol.cable_length[m] - g_L0[m];
            motor_kp[m][i] = s_ccw_to_len[m] * dL * GEAR / r;  // motor-major: [m][i]
        }

        // 记录调试与上一解
        for (int m = 0; m < NUM_CABLES; ++m) {
            last_cable_lengths[m]        = sol.cable_length[m];
            app->debug_cable_lengths[m]  = sol.cable_length[m];
        }
    }

    // 3) 平尾：将 [N, MAX_FREQ) 用最后一个有效点的值填充，避免残留/突跳
    for (int m = 0; m < APP_MOTORS; ++m) {
        const float hold = motor_kp[m][N - 1];
        for (int k = N; k < MAX_FREQ; ++k) {
            motor_kp[m][k] = hold;
        }
    }
    // 4) 同步有效长度，确保下游只读 [0, N-1]
    if (app) {
        app->motor_len = N;
    }
}

			
/* ===========================================================================================
 * 逆动力学主函数：末端运动状态序列 → 四电机“所需力矩序列”
 *
 * 输入：
 *   - app：应用全局对象。函数将直接使用 app->pose_kp 和 app->kp_len 作为输入。
 *
 * 输出：
 *   - app->motor_torque[m][i]：第 m 台电机在第 i 个关键点所需的驱动力矩 (N·m)
 *     计算流程：   末端加速度 → 广义力 F_ext
 *               → (W, F_ext) → 索力 t (solve_cable_forces)
 *               → 电机力矩 τ[m] = t[m] * r_motor / GEAR
 *
 * 可行性：
 *   - solve_cable_forces(...) 内部使用伪逆+零空间法，寻找一组有效的正索力解。
 *   - 若求解失败（返回 false），则该点的力矩设为0或沿用上一点的值，以防突变。
 *
 * ===========================================================================================*/
void App_ID_FromPoseArray(App_t* app)
{
    // 1. 初始化和获取参数
    ensure_init(); // 确保全局参数已初始化
    CableForceSolution_t force_sol;
    const int N = app->kp_len; // 从 app 结构体中获取有效点数
    const TP2_MotionState* pose_kp = app->pose_kp; // 从 app 结构体中获取位姿序列指针

    // 检查输入有效性
    if (N <= 0 || N > MAX_FREQ) {
        // 如果没有有效的点，则不执行任何操作
        return;
    }

    // 2. 逐点进行逆动力学计算
    for (int i = 0; i < N; i++) {
        // 从 pose_kp 准备当前点的完整运动状态
        const TP2_MotionState* current_motion = &pose_kp[i];

        // 调用我们新的、稳健的索力求解器
        if (!solve_cable_forces(current_motion, &g_cdpr, &force_sol) || !force_sol.success) {
            // 如果求解失败 (例如，平台处于奇异位形或所需力无法由正索力产生)
            // 为了安全和连续性，将该点的目标力矩设为0或沿用上一点的值
            for (int m = 0; m < APP_MOTORS; m++) {
                app->motor_torque[m][i] = (i > 0) ? app->motor_torque[m][i-1] : 0.0f;
            }
            continue; // 继续处理下一个点
        }

        // 如果求解成功：
        // a. 将计算出的索力 (tensions) 转换为电机所需的驱动力矩 (torque)
        const float r = g_cdpr.r_motor;
        for (int m = 0; m < APP_MOTORS; m++) {
            // 索力 t (单位: N)
            float tension = force_sol.tensions[m];
            
            // 电机力矩 τ = t * r / η (η 是传动效率，这里假设为1)
            // GEAR 是减速比，如果力矩在电机输出轴计算，则需要考虑。
            // 假设 GEAR 是 > 1 的减速比，则电机所需力矩更小。
            // τ_motor = τ_load / GEAR = (t * r) / GEAR
            app->motor_torque[m][i] = tension * r / GEAR;
        }
    } // end of for (int i = 0; i < N; i++)
}

/**
 * @brief [正向运动学] 根据电机当前位置，迭代求解末端位姿。
 *
 * 该函数使用牛顿-拉夫逊迭代法，通过已知的4个电机绝对角度（rad）
 * 来反解出末端执行器的位姿 (x, y, theta)。
 *
 * @param motor_positions   一个包含4个电机当前绝对角度（单位：rad）的数组。
 * @param pose_estimate     一个指向 Pose_t 结构体的指针。
 *                          【输入时】应包含上一周期的末端位姿，作为本次迭代的初始猜测值。
 *                          【输出时】如果求解成功，该结构体将被更新为当前计算出的末端位姿。
 * @return                  如果成功收敛到解，返回 true；如果迭代发散或遇到奇异点，返回 false。
 */
bool App_FK_FromMotorPos(const float motor_positions[APP_MOTORS],
                         Pose_t* pose_estimate)
{
    // --- 0. 定义迭代参数 ---
    const int MAX_ITERATIONS = 20;      // 最大迭代次数
    const float TOLERANCE_SQ = 1e-9f;  // 收敛误差阈值的平方 (1e-6米 -> 1微米)
    // --- 1. 初始化 ---
    ensure_init(); // 确保 g_cdpr 和 g_L0 等参数已加载
    
    // --- 2. 输入转换: 将电机角度 (rad) 转换为绝对缆绳长度 (m) ---
    // 这是迭代的目标 (l_measured)
    float measured_lengths[NUM_CABLES];
    const float r = g_cdpr.r_motor;
    for (int m = 0; m < NUM_CABLES; ++m) {
        // 反向运算: motor_angle = sign * (L_abs - L0) * GEAR / r
        // -> (L_abs - L0) = motor_angle * r / (sign * GEAR)
        // -> L_abs = L0 + motor_angle * r * sign / GEAR
        const float dL = motor_positions[m] * r * s_ccw_to_len[m] / GEAR;
        measured_lengths[m] = g_L0[m] + dL;
    }

    // --- 3. 牛顿-拉夫逊迭代循环 ---
    Pose_t q_k = *pose_estimate; // 从传入的初始猜测值开始迭代
    CableLengthSolution_t ik_solution;

    for (int i = 0; i < MAX_ITERATIONS; ++i) {
        // 3.1) 计算当前猜测位姿 q_k 下的理论缆绳长度 g(q_k)
        if (!solve_cable_lengths(&q_k, &g_cdpr, &ik_solution) || ik_solution.exitflag != 1) {
            return false; // 当前猜测值无效，无法继续迭代
        }

        // 3.2) 计算长度误差向量 e = l_measured - g(q_k)
        float error_vec[4];
        float error_norm_sq = 0.0f;
        for (int j = 0; j < NUM_CABLES; ++j) {
            error_vec[j] = measured_lengths[j] - ik_solution.cable_length[j];
            error_norm_sq += error_vec[j] * error_vec[j];
        }

        // 3.3) 检查是否收敛
        if (error_norm_sq < TOLERANCE_SQ) {
            *pose_estimate = q_k; // 找到解，更新结果并成功返回
            return true;
        }

        // 3.4) 计算当前点的雅可比矩阵 J(q_k)
        float J[4][3];
        if (!calculate_jacobian(&q_k, &g_cdpr, J)) {
            return false; // 无法计算雅可比矩阵 (例如几何关系无效)
        }

        // 3.5) 计算伪逆 J_plus = (J^T * J)^-1 * J^T
        float Jt[3][4];
        for (int r = 0; r < 3; ++r) for (int c = 0; c < 4; ++c) Jt[r][c] = J[c][r];
        
        float JtJ[3][3];
        matrix_mult_3x4_4x3(Jt, J, JtJ);

        float JtJ_inv[3][3];
        if (!matrix_invert_3x3(JtJ, JtJ_inv)) {
            return false; // 奇异点，(J^T * J) 不可逆
        }

        float J_plus[3][4];
        // J_plus = JtJ_inv * Jt (3x3 * 3x4 -> 3x4)
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 4; ++c) {
                J_plus[r][c] = 0.0f;
                for (int k = 0; k < 3; ++k) {
                    J_plus[r][c] += JtJ_inv[r][k] * Jt[k][c];
                }
            }
        }

        // 3.6) 计算位姿更新量 delta_q = J_plus * error_vec
        float delta_q[3];
        matrix_mult_3x4_4x1(J_plus, error_vec, delta_q);

        // 3.7) 更新位姿 q_{k+1} = q_k + delta_q
        q_k.x += delta_q[0];
        q_k.y += delta_q[1];
        q_k.theta += delta_q[2];
    }

    return false; // 超过最大迭代次数仍未收敛
}

