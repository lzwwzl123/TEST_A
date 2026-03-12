// =================================
// FILE: app.c (Final Version)
// =================================

#include "app.h"




// 外部依赖声明

extern DebugPanel_t g_dbg; // 声明对 main.c 中 g_dbg 的引用

/* ===========================================================================================
 * 文件级说明（仅注释，不影响编译）
 * -------------------------------------------------------------------------------------------
 * 本模块实现“应用层”的主要逻辑与周期调度：
 *   1) 采样/插值/轨迹构建与预览（BuildTrajectoryFromParams / App_PreviewTrajectoryStart）
 *   2) 归零/零位偏置维护（App_ZeroNow）
 *   3) 运动状态机（IDLE/RUNNING）的推进（App_MainLoop / App_Tick）
 *   4) 与通信层（RS485）的粘合：
 *        - 读取执行器测量值：CommBus_GetData()
 *        - 下发控制命令：CommBus_GetCmd()->W + CommBus_TryStepRoundRobin()
 *        - 服务看门狗/超时：CommBus_ServiceTick()
 *
 * 时序约定：
 *   - App_Tick() 在控制节拍中断中被调用（如 TIM2），执行“一个固定控制周期”的工作；
 *   - App_MainLoop() 在主循环中被调用，仅做轻量工作：参数热更新、轨迹重建、安全检查、看门狗。
 * ===========================================================================================*/

/* ------------------------------------------------------------------------------------------------
 * @brief  辅助：获取“当前已应用轨迹”的终点位姿
 * @param  app  应用对象
 * @return 若 kp_len>0，则返回 pose_kp 的最后一点；否则返回原点姿态（0）
 * @note   用于在“装载新轨迹”前，检查起点是否与当前轨迹终点连续，避免位姿跳变。
 * ------------------------------------------------------------------------------------------------*/
static int cmp_float_desc(const void *a, const void *b)
{
    float fa = *(const float*)a;
    float fb = *(const float*)b;
    return (fa < fb) - (fa > fb);  // 倒序
}
static TP2_Pose App_GetCurrentPoseSet(App_t *app)
{
    if (app->kp_len > 0) {
        // 如果已经有轨迹在运行或刚结束，终点就是最后一个关键点
        return app->pose_kp[app->kp_len - 1].pos;
    } else {
        // 如果系统刚初始化，还没有任何轨迹，则认为当前在原点
        // 这与 App_ZeroNow 的行为一致
        TP2_Pose origin = {0};
        return origin;
    }
}

/* ------------------------------------------------------------------------------------------------
 * @brief  轨迹采样参数配置（频率/持续时长）
 * @param  app              应用对象
 * @param  freq             轨迹求值频率（Hz）；<=1 时回落到 APP_SAMPLER_FREQ_DEFAULT；上限 MAX_FREQ
 * @param  points_duration  每段插值的“持续时长”抽象参数（由轨迹模块定义具体语义）
 * @note   - 更新 freq/points_duration/flag/count，并计算 total_time_s = freq*points_duration*dt
 *         - flag/count 复位保证新轨迹从起点/起段开始
 *         - 仅配置，不生成轨迹点；轨迹点由 BuildTrajectoryFromParams 生成
 * ------------------------------------------------------------------------------------------------*/
void App_ConfigSampler(App_t *app, int freq, int points_duration){
    if (freq <= 1)            freq = APP_SAMPLER_FREQ_DEFAULT;
    if (freq > MAX_FREQ)      freq = MAX_FREQ;
    if (points_duration <= 0) points_duration = APP_POINTS_DUR_DEFAULT;

    app->freq = freq;
    app->points_duration = points_duration;
    app->flag  = 0;   // 当前关键点索引（段起点）
    app->count = 0;   // 段内样本计数（0..points_duration-1）
    app->total_time_s = (float)freq * (float)points_duration * app->dt; // 一次轨迹总时长
}

/* ------------------------------------------------------------------------------------------------
 * @brief  按参数构建轨迹（离散姿态 + 电机目标），并初始化推进指示变量
 * @param  app  应用对象（写入 pose_kp/motor_kp/kp_len/motor_len/flag/count/total_time_s）
 * @param  P    轨迹参数（包含模式/几何/采样频率/持续时长）
 * @note   - 先调用 App_ConfigSampler() 统一检查/归一化采样参数
 *         - 依据模式构建直线/圆弧规划对象（TP2_LinePlan/TP2_ArcPlan）
 *         - 均匀时间参数化：t 从 0..total_time_s，离散成 freq 个关键点（首尾都含）
 *         - 调用 App_IK_FromPoseArray() 将姿态序列映射到 motor_kp（逆运动学）
 * ------------------------------------------------------------------------------------------------*/
void BuildTrajectoryFromParams(App_t *app, const MotionParams *P)
{
    // 1. 设置基础采样参数 (dt, total_time_s)
    App_ConfigSampler(app, P->freq, P->points_duration);
    
    // 清理缓冲区
    memset(app->pose_kp, 0, sizeof(app->pose_kp));
    memset(app->motor_kp, 0, sizeof(app->motor_kp));
    

    // 2. 初始化轨迹规划器并获取轨迹物理长度
    if (P->mode == MODE_LINE){
        TP2_LinePlanInit(&app->line, app->dt, P->L_p0, P->L_p1, app->total_time_s,
                         0.0f, 0.0f, P->L_yaw0, P->L_yawf, 0.0f, 0.0f);
    } else if (P->mode == MODE_ARC){
        TP2_ArcPlanInit (&app->arc,  app->dt, P->C_center, P->C_R, P->C_phi0, P->C_phi1, app->total_time_s,
                         0.0f, 0.0f, P->C_yaw0, P->C_yawf, 0.0f, 0.0f);

    }
		// 预先规划，计算电机最大运动角度
    for (int i = 0; i < APP_SAMPLER_FREQ_DEFAULT; i++){
        float t = ((float)i / (float)(APP_SAMPLER_FREQ_DEFAULT - 1)) * app->total_time_s;

        if (P->mode == MODE_LINE)
            TP2_LineEvalAt(&app->line, t, &app->pose_kp[i]);
        else if (P->mode == MODE_ARC)
            TP2_ArcEvalAt (&app->arc,  t, &app->pose_kp[i]);
        else
            app->pose_kp[i].pos.x = app->pose_kp[i].pos.y = app->pose_kp[i].pos.yaw = 0.0f;
    }
		
		App_IK_FromPoseArray(app, app->pose_kp, APP_SAMPLER_FREQ_DEFAULT, app->motor_kp);
		float lengths[APP_MOTORS] = {0};
		for(int i = 0;i < APP_MOTORS;i++){
			for(int j = 0;j < APP_SAMPLER_FREQ_DEFAULT - 1;j++){
				lengths[i] += fabsf(app->motor_kp[i][j+1] - app->motor_kp[i][j]);
			}
		}
		qsort(lengths,APP_MOTORS, sizeof(float), cmp_float_desc);
		int rel_length = (int)lroundf(lengths[0] * 15.0f);  // 先乘再四舍五入
		if (rel_length < 2) rel_length = 2;
		if (rel_length > MAX_FREQ) rel_length = MAX_FREQ;   // 重新夹紧
		app->kp_len = rel_length;
		for (int i = 0; i < app->kp_len; i++){
        // 时间采样必须基于 app->kp_len 进行归一化
        float t = ((float)i / (float)(app->kp_len - 1)) * app->total_time_s;
        if (P->mode == MODE_LINE)
            TP2_LineEvalAt(&app->line, t, &app->pose_kp[i]);
        else if (P->mode == MODE_ARC)
            TP2_ArcEvalAt (&app->arc,  t, &app->pose_kp[i]);
        else
            app->pose_kp[i].pos.x = app->pose_kp[i].pos.y = app->pose_kp[i].pos.yaw = 0.0f;
    }
    // 5. 逆解：现在这里的 app->kp_len 与实际填充的点数完全一致
    App_IK_FromPoseArray(app, app->pose_kp, app->kp_len, app->motor_kp);
		App_ID_FromPoseArray(app);

    // 6. 设置运动参数
    app->motor_len = app->kp_len;
    app->flag  = 0;
    app->count = 0;
}


/* ------------------------------------------------------------------------------------------------
 * @brief  仅用于“预览”轨迹起点姿态，不修改 app 的任何状态（安全只读）
 * @param  P   轨迹参数
 * @return 起点处姿态（t=0）
 * @note   - 内部使用临时 TP2_*Plan 结构，仅在局部栈上构建
 *         - 频率/points_duration 的默认值与 BuildTrajectoryFromParams 保持一致
 *         - 目的：在装载前检查“新轨迹起点”与“当前轨迹末端”的连续性（位置/姿态差）
 * ------------------------------------------------------------------------------------------------*/
TP2_Pose App_PreviewTrajectoryStart(const MotionParams *P)
{
    TP2_MotionState start_pose = {0};

    // 1. 计算轨迹总时长 (与 BuildTrajectoryFromParams 逻辑相同)
    int freq = (P->freq > 1) ? P->freq : APP_SAMPLER_FREQ_DEFAULT;
    int points_duration = (P->points_duration > 0) ? P->points_duration : APP_POINTS_DUR_DEFAULT;
    float total_time_s = (float)freq * (float)points_duration * APP_CTRL_DT;

    // 2. 在【局部临时变量】上建立插值对象
    if (P->mode == MODE_LINE) {
        TP2_LinePlan temp_line;
        TP2_LinePlanInit(&temp_line, APP_CTRL_DT, P->L_p0, P->L_p1, total_time_s, 0.0f, 0.0f, P->L_yaw0, P->L_yawf, 0.0f, 0.0f);
        // 3. 只在 t=0 时评估，获取起点
        TP2_LineEvalAt(&temp_line, 0.0f, &start_pose);
    } else if (P->mode == MODE_ARC) {
        TP2_ArcPlan temp_arc;
        TP2_ArcPlanInit(&temp_arc, APP_CTRL_DT, P->C_center, P->C_R, P->C_phi0, P->C_phi1, total_time_s, 0.0f, 0.0f, P->C_yaw0, P->C_yawf, 0.0f, 0.0f);
        // 3. 只在 t=0 时评估，获取起点
        TP2_ArcEvalAt(&temp_arc, 0.0f, &start_pose);
    }

    return start_pose.pos;
}

/* ------------------------------------------------------------------------------------------------
 * @brief  设置“直线”轨迹参数（写入 desired，不立即重建）
 * @note   重建由 BuildTrajectoryFromParams 触发；通常在“参数应用”处统一调用。
 * ------------------------------------------------------------------------------------------------*/
void App_SetLine(App_t *app, TP2_Point p0, TP2_Point p1, float yaw0, float yawf){
    app->desired.mode  = MODE_LINE;
    app->desired.L_p0  = p0;
    app->desired.L_p1  = p1;
    app->desired.L_yaw0= yaw0;
    app->desired.L_yawf= yawf;
}

/* ------------------------------------------------------------------------------------------------
 * @brief  设置“圆弧”轨迹参数（写入 desired，不立即重建）
 * ------------------------------------------------------------------------------------------------*/
void App_SetArc (App_t *app, TP2_Point center, float R, float phi0, float phi1, float yaw0, float yawf){
    app->desired.mode    = MODE_ARC;
    app->desired.C_center= center;
    app->desired.C_R     = R;
    app->desired.C_phi0  = phi0;
    app->desired.C_phi1  = phi1;
    app->desired.C_yaw0  = yaw0;
    app->desired.C_yawf  = yawf;
}

/* ------------------------------------------------------------------------------------------------
 * @brief  归零：将当前位置写入 zero_off[]，并把相对量清零
 * @note   - 从通信层读取当前“绝对长度/位置”（CommBus_GetData()->Pos），作为零偏
 *         - pos_meas_rel/pos_set_rel 清零，控制器同步（Control_SyncAllToMeas）
 *         - 建议在机械静止、无异常张力时调用
 * ------------------------------------------------------------------------------------------------*/
void App_ZeroNow(App_t *app){
    for (int i=0;i<APP_MOTORS;i++){
        float abs_meas = CommBus_GetData(&app->bus, i)->Pos;
        app->pos_meas_abs[i] = abs_meas;
        app->zero_off[i]     = abs_meas;
        app->pos_meas_rel[i] = 0.0f;
        app->pos_set_rel [i] = 0.0f;
    }
    Control_SyncAllToMeas(&app->ctl, app->pos_meas_rel);
}

/* ------------------------------------------------------------------------------------------------
 * @brief  应用初始化：控制/通信/热身轮询/归零
 * @param  app         应用对象
 * @param  huart       RS485 使用的 UART 句柄
 * @param  rede_port   RS485 DE/RE 所在端口（若硬件自动/未接，可为 NULL）
 * @param  rede_pin    RS485 DE/RE 引脚号（未接为 0）
 * @param  en_port     可选指示/使能端口（如 TX 活动 LED），可为 NULL
 * @param  en_pin      对应引脚号（或 0）
 * @param  node_ids    4 个从站地址（长度 APP_MOTORS）
 * @note   - Control_Init：初始化 PID/限幅等，默认使能四轴
 *         - CommBus_Init：配置串口与 RS485 收发（DE/RE 控制），建立从站地址表
 *         - 预热通信：COMM_INIT_WARMUP_CYCLES 轮“轮询步进”，并等待 busy 清空，改善上电同步
 *         - 最后执行 App_ZeroNow：把当前测量写为零位
 * ------------------------------------------------------------------------------------------------*/
void App_Init(App_t *app,
              UART_HandleTypeDef *huart,
              GPIO_TypeDef *rede_port, uint16_t rede_pin,
              GPIO_TypeDef *en_port, uint16_t en_pin,
              const uint8_t *node_ids)
{
    memset(app, 0, sizeof(*app));
    app->dt = APP_CTRL_DT;
		app->vel_alpha = 0.1;
    Control_Init(&app->ctl, app->dt);
    for (int i=0;i<APP_MOTORS;i++) Control_EnableAxis(&app->ctl, i, 1);

    CommBus_Init(&app->bus, huart, rede_port, rede_pin, en_port, en_pin, node_ids);

    // [关键修正] 执行一个健壮的、有保障的预热和净化流程

    // 1. 物理稳定期
    HAL_Delay(200);

    // 2. 清除幽灵错误
    HAL_UART_Abort(huart);

    // 3. 真正可靠的预热：持续轮询，直到所有电机都有一次成功的通信
    uint32_t warmup_start_tick = HAL_GetTick();
    const uint32_t WARMUP_TIMEOUT_MS = 2000; // 设置一个总的超时时间，例如2秒
    uint8_t received_mask = 0; // 用一个位掩码来标记哪些电机已响应
    const uint8_t ALL_MOTORS_RESPONDED = (1 << APP_MOTORS) - 1;

    while (received_mask != ALL_MOTORS_RESPONDED)
    {
        // 检查是否总超时
        if (HAL_GetTick() - warmup_start_tick > WARMUP_TIMEOUT_MS) {
            // 在这里可以设置一个错误码，表示初始化失败
            // g_dbg.error_code = 0xDEAD;
            break; // 超时，跳出循环
        }

        // 尝试发起一次通信（如果总线空闲）
        (void)CommBus_TryStepRoundRobin(&app->bus);

        // 检查所有电机，看是否有新的数据进来
        for (int i = 0; i < APP_MOTORS; i++) {
            // 如果这个电机之前没响应过，并且现在的位置不再是0
            // (假设电机的绝对位置永远不会是精确的0)
            if (!(received_mask & (1 << i)) && (CommBus_GetData(&app->bus, i)->Pos != 0.0f))
            {
                received_mask |= (1 << i); // 标记该电机已响应
            }
        }
    }

    // 4. 在确保所有电机都已提供真实数据后，才执行归零
    App_ZeroNow(app);

}

/* ------------------------------------------------------------------------------------------------
 * @brief  主循环：轻量任务（启动/装载新轨迹的安全检查）+ 通信服务
 * @param  app 应用对象
 * @note   - g_dbg.start_motion：外部触发运行（仅在 IDLE 时生效；要求 motor_len>=2）
 *         - 装载新轨迹前进行“位姿连续性检查”：位置误差 < 2 mm，偏航误差 < 0.035 rad (~2°)
 *           · 通过 → applied = desired → BuildTrajectoryFromParams
 *           · 失败 → 置 error_code=1，并回滚 desired=applied，避免轨迹突跳
 *         - CommBus_ServiceTick：服务通信看门狗/超时（最后一个参数为服务时间片 ms）
 * ------------------------------------------------------------------------------------------------*/
void App_MainLoop(App_t *app)
{

    if (app->motion_state == MOTION_IDLE) {

        // --- 逻辑1: 检查是否有“启动”请求 ---
        if (g_dbg.start_motion && g_dbg.error_code == 0) {
            g_dbg.start_motion = 0;
            if (app->motor_len >= 2) { // 至少需要两个关键点才有“段”
                app->flag = 0;
                app->count = 0;
                app->motion_state = MOTION_RUNNING;
            }
        }

        // --- 逻辑2: 检查是否有“加载新轨迹”的请求 ---
        else if (memcmp(&app->desired, &app->applied, sizeof(MotionParams)) != 0) {

            // 调用安全的、只读的预览函数，而不是污染全局状态的 BuildTrajectoryFromParams
            TP2_Pose new_traj_start_pose = App_PreviewTrajectoryStart(&app->desired);
            TP2_Pose current_pose = App_GetCurrentPoseSet(app);

            // 连续性检查：起点与当前末端的距离平方/偏航差
            float dx = new_traj_start_pose.x - current_pose.x;
            float dy = new_traj_start_pose.y - current_pose.y;
            float dist_sq = dx*dx + dy*dy;
            float yaw_diff = fabsf(new_traj_start_pose.yaw - current_pose.yaw);
            if (yaw_diff > (float)M_PI) { yaw_diff = 2.0f * (float)M_PI - yaw_diff; } // wrap 到 [-π, π]

            // 阈值：位置 2 mm（0.002 m → 0.000004 m^2），姿态约 2°
            const float POS_THRESHOLD_SQ = 0.000004f;
            const float YAW_THRESHOLD_RAD = 0.005f;

            if (dist_sq < POS_THRESHOLD_SQ && yaw_diff < YAW_THRESHOLD_RAD) {
                // 检查通过！现在可以安全地修改全局状态了。
                g_dbg.error_code = 0;
                app->applied = app->desired;
                // 这是唯一一次调用 BuildTrajectoryFromParams 的地方
                BuildTrajectoryFromParams(app, &app->applied);
            } else {
                // 检查失败，报告错误，不触碰任何全局轨迹数据
                g_dbg.error_code = 1;           // 可由 UI/上位机提示“起点不连续”
                app->desired = app->applied;    // 回滚，保持系统一致性
            }
        }
    }

    // 通信服务：在主循环提供若干毫秒的服务时间片（如 25ms），用于处理队列/超时/重发等
    CommBus_ServiceTick(&app->bus, HAL_GetTick(), 25);
}

/* ------------------------------------------------------------------------------------------------
 * @brief  控制周期函数（在定时器中断中被调用）：推进运动并下发控制命令
 * @param  app 应用对象
 * @note   三个阶段：
 *   1) 轨迹推进（若 RUNNING）：段内线性插值（progress = count/points_duration）
 *      - 每次周期输出 pos_set_rel[m]，平滑过渡而非阶跃
 *      - flag/count 推进；到达末尾时进入 IDLE，并把目标固定在最终关键点
 *   2) 采样：从通信层读取测量值（绝对）并换算为相对（减零偏）
 *   3) 控制：设置目标 → TickAll 输出 vel_out → 写入 CommBus 命令 → 轮询下一步通讯
 * ------------------------------------------------------------------------------------------------*/
void App_Tick(App_t *app)
{
    float torque_cmd[APP_MOTORS] = {0};

    if (app->motion_state == MOTION_RUNNING) {
        if (app->motor_len < 2 || app->points_duration <= 0) {
            // 异常：没有足够关键点或段内时长非法，直接停机进入空闲
            app->motion_state = MOTION_IDLE;
						for (int i = 0; i < APP_MOTORS; i++) {
                        PID_Reset(&app->ctl.axis[i].pid, app->pos_meas_rel[i], 0.0f);
                    }
					
        } else {
            // 使用平滑的线性插值，而不是阶跃
            float progress = (float)app->count / (float)app->points_duration;
            for (int m = 0; m < APP_MOTORS; m++){
                float p0 = app->motor_kp[m][app->flag];
                float p1 = app->motor_kp[m][app->flag + 1];
                app->pos_set_rel[m] = p0 + (p1 - p0) * progress;
            }
#if APP_CTRL_MODE == APP_CTRL_MODE_TORQUE
            for (int m = 0; m < APP_MOTORS; m++) {
                float t0 = app->motor_torque[m][app->flag];
                float t1 = app->motor_torque[m][app->flag + 1];
                torque_cmd[m] = t0 + (t1 - t0) * progress;
            }
#endif

            app->count++;
            if (app->count >= app->points_duration){
                app->count = 0;
                app->flag++;
                if (app->flag >= app->motor_len - 1){
                    // 已经推进到最后一个段的末尾：退出运行态，目标固定在末端
                    app->motion_state = MOTION_IDLE;
                    for (int m = 0; m < APP_MOTORS; m++) {
                        app->pos_set_rel[m] = app->motor_kp[m][app->motor_len - 1];
                    }
										     for (int i = 0; i < APP_MOTORS; i++) {
                        PID_Reset(&app->ctl.axis[i].pid, app->pos_meas_rel[i], 0.0f);
                    }
                }
            }
        }
    }

    // 采样：从通信层读取当前绝对测量值，并换算为“相对值”（减去零偏）
    for (int i = 0; i < APP_MOTORS; i++){
        float abs_meas = CommBus_GetData(&app->bus, i)->Pos;
        app->pos_meas_abs[i] = abs_meas;
        app->pos_meas_rel[i] = abs_meas - app->zero_off[i];
    }
		// === 基于位置反馈的速度估计 ===
		for (int i = 0; i < APP_MOTORS; ++i) {
				// 若没有编码器“回卷”（wrap-around），直接差分即可
				float dpos = app->pos_meas_rel[i] - app->pos_meas_rel_prev[i];

				// 做一个简单的尖峰抑制，避免偶发通讯毛刺
				const float max_reasonable = 4000.0f * app->dt; // 例如 1000 单位/秒
				if (dpos >  max_reasonable) dpos =  max_reasonable;
				if (dpos < -max_reasonable) dpos = -max_reasonable;

				float v_raw = dpos / app->dt;                 // 原始差分速度
				app->vel_meas_raw[i] = v_raw;                 // 留给监控/日志

				// 一阶低通滤波（指数平均）
				float a = app->vel_alpha;                     // 0..1
				app->vel_meas[i] = app->vel_meas[i] + a * (v_raw - app->vel_meas[i]);

				// 更新上次位置
				app->pos_meas_rel_prev[i] = app->pos_meas_rel[i];
		}
		
    // 控制：设置目标并对所有轴执行一次控制 Tick，输出 vel_out（单位视控制层/底层约定）
#if APP_CTRL_MODE == APP_CTRL_MODE_TORQUE
    // 力控模式：不做位置→速度闭环
    (void)app;
#else
    Control_SetTargets(&app->ctl, app->pos_set_rel);
    Control_TickAll(&app->ctl, app->pos_meas_rel, app->vel_out);
#endif

    // 下发：把控制输出写入到通信层命令缓存（例如速度/转速通道），然后尝试轮询发送
    for (int i = 0; i < APP_MOTORS; i++){
#if APP_CTRL_MODE == APP_CTRL_MODE_TORQUE
        CommBus_GetCmd(&app->bus, i)->T   = torque_cmd[i];
        CommBus_GetCmd(&app->bus, i)->W   = 0.0f;
        CommBus_GetCmd(&app->bus, i)->Pos = 0.0f;
#else
        CommBus_GetCmd(&app->bus, i)->T   = 0.0f;
        CommBus_GetCmd(&app->bus, i)->W   = app->vel_out[i];
        CommBus_GetCmd(&app->bus, i)->Pos = app->pos_set_rel[i] + app->zero_off[i];
#endif

    }
    (void)CommBus_TryStepRoundRobin(&app->bus);
    (void)CommBus_TryStepRoundRobin(&app->bus);
}

