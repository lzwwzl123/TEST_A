#include "comm_rs485.h"
// =======================================================================================
// 角色：RS485 主站轮询通信的最小实现（“发完立收”的异步中断链路）
// 约定：
//   - 上层通过 CommBus_TryStepRoundRobin() 在空闲时发起一次 Tx→Rx 事务；真实的收/发推进靠 HAL 回调。
//   - 帧编解码：由协议层 gom_protocol.h 提供 modify_data() / extract_data() 完成。
//   - 方向/指示：本实现未显式驱动 RE/DE（半双工方向引脚）；提供了 EN_Tx/EN_Rx 钩子点（通常接 LED）。
//     * 若你的硬件将 EN_* 所在引脚直接接到了收发器的 DE（或 DE/RE 合并脚），则本实现即可完成方向切换。
//     * 若未接，请保持 en_port=NULL；需要方向控制时，可在 EN_Tx/EN_Rx 内加入 RE/DE 翻转（不改本文件其他逻辑）。
// 并发/时序：
//   - busy=1 表示一次事务正在进行（从发起发送到接收完成/失败之间）；防止重入。
//   - last_send_ms 记录事务开始时间；ServiceTick() 用于超时兜底，避免状态机卡死。
// 安全提示：
//   - HAL_UART_Transmit_IT / HAL_UART_Receive_IT 需在同一 huart 上工作；在 CubeMX 开启 USART 中断。
//   - motor_send_data/motor_recv_data 的大小必须与协议定义一致，否则会接收溢出/截断。
// =======================================================================================


// 发送前LED=SET；接收前/完成后LED=RESET
// -------------------------------------------------------------------------------------------------
// @brief  发送阶段的指示/方向控制钩子：这里仅点亮指示灯。如需方向控制，可在此处同时置 DE=1。
// @note   若 en_port==NULL 表示未连接指示/方向引脚，函数将不做任何操作。
static inline void EN_Tx(CommBus_t *b){
    if (b->en_port) HAL_GPIO_WritePin(b->en_port, b->en_pin, GPIO_PIN_SET);
}
// @brief  接收/空闲阶段的指示/方向控制钩子：这里仅熄灭指示灯。如需方向控制，可在此处 DE=0。
static inline void EN_Rx(CommBus_t *b){
    if (b->en_port) HAL_GPIO_WritePin(b->en_port, b->en_pin, GPIO_PIN_RESET);
}

/* 尝试对指定节点发送一帧（异步IT模式）。成功返回true。 */
// -------------------------------------------------------------------------------------------------
// 流程：busy 检查 → 填充命令（含 ID）→ 协议层组帧 modify_data() → EN_Tx → Transmit_IT → 标记 busy/时间戳
// 注意：
//   - 本函数不阻塞等待发送完成；真正的“切收+启动接收”在 TxCplt 回调中进行。
//   - 若 Transmit_IT 启动失败，立即恢复 EN_Rx 并返回 false，避免长时间处于“方向=发”。
static bool CommBus_TrySendTo(CommBus_t *bus, int idx){
    if (bus->busy) return false;                         // 正在事务中，拒绝重入
    if (idx < 0 || idx >= APP_MOTORS) return false;      // 索引越界保护

    // 准备发送内容：把节点地址写入 cmd，并通过协议层完成编码/校验/打包
    bus->nodes[idx].cmd.id = bus->nodes[idx].id;
	  bus->nodes[idx].cmd.K_P  = KP_MOTOR;
    bus->nodes[idx].cmd.K_W  = KD_MOTOR;
    modify_data(&bus->nodes[idx].cmd);                   // >>> 协议层：填充 motor_send_data 字节流 <<<

    // 先切 Tx、置 LED=SET（如将该脚并到 DE，则同时完成方向切换）
    EN_Tx(bus);                                          // 发送期间 LED 亮（或 DE=1）

    // 非阻塞发送；等中断回调继续链路（TxCplt → 切 Rx 并启动接收）
    if (HAL_UART_Transmit_IT(bus->huart,
          (uint8_t*)&bus->nodes[idx].cmd.motor_send_data,
          sizeof(bus->nodes[idx].cmd.motor_send_data)) == HAL_OK)
    {
        bus->busy = 1;
        bus->last_send_ms = HAL_GetTick();
        bus->tx_node_idx = idx;                          // 记录当前事务属于哪个节点
        return true;
    } else {
        // 启动失败：回到接收/空闲态，避免方向/灯长期保持在“发送中”
        EN_Rx(bus);
        return false;
    }
}

/* 初始化：绑定串口/引脚与节点ID，给命令结构体默认值并进入接收态 */
// -------------------------------------------------------------------------------------------------
// 说明：
//  - rede_port/rede_pin 目前未在本文件中使用（保留给需要的方向控制扩展）；你也可以把它接到 EN_*。
//  - nodes[i].cmd 默认写入一些合理初值（如 K_W），请根据协议/电机要求调整。
void CommBus_Init(CommBus_t *bus,
                  UART_HandleTypeDef *huart,
                  GPIO_TypeDef *rede_port, uint16_t rede_pin,
                  GPIO_TypeDef *en_port,   uint16_t en_pin,
                  const uint8_t *node_ids)
{
    bus->huart = huart;
    bus->rede_port = rede_port; bus->rede_pin = rede_pin; // 预留：需要时在 EN_Tx/EN_Rx 使用
    bus->en_port   = en_port;   bus->en_pin   = en_pin;

    bus->busy = 0;
    bus->last_send_ms = 0;
    bus->tx_node_idx = -1;                                 // -1 表示当前没有活跃事务
    bus->next_rr_idx = 0;

    // 初始化节点默认命令
    for (int i=0;i<APP_MOTORS;i++){
        bus->nodes[i].id = node_ids ? node_ids[i] : (uint8_t)i;

        MotorCmd_t *cmd = &bus->nodes[i].cmd;
        cmd->id   = bus->nodes[i].id;
        cmd->mode = 1;                                    
        cmd->K_P  = KP_MOTOR;
        cmd->K_W  = KD_MOTOR;                                 
        cmd->Pos  = 0.0f;
        cmd->W    = 0.0f;
        cmd->T    = 0.0f;
    }

    // 上电先进入接收/空闲态（LED灭/DE=0）
    EN_Rx(bus);
}

/* 主站轮询：从 next_rr_idx 开始尝试对每个节点发起一次事务（找到空闲就返回） */
// -------------------------------------------------------------------------------------------------
// 用法：在控制周期或主循环末尾调用一次；若返回 true，说明已经启动了一个 Tx→Rx 事务。
// 备注：如果所有节点都无法启动（例如 UART 忙），则返回 false，等待下一次调用。
bool CommBus_TryStepRoundRobin(CommBus_t *bus){
    if (bus->busy) return false;                           // 正在事务中
    for (int k=0;k<APP_MOTORS;k++){
        int idx = (bus->next_rr_idx + k) % APP_MOTORS;
        if (CommBus_TrySendTo(bus, idx)) {
            bus->next_rr_idx = (idx + 1) % APP_MOTORS;     // 下次从下一个节点开始，形成“轮询”
            return true;
        }
    }
    return false;
}

/* 看门狗：Tx 超时则回收为接收态，避免死锁 */
// -------------------------------------------------------------------------------------------------
// 典型触发：从 TxCplt → RxCplt 的链路丢失（例如电机未响应/线缆断开），busy 会长期为 1。
// 处理：超过 send_timeout_ms 未完成接收则清 busy、复位事务索引，熄灯/切回接收。
void CommBus_ServiceTick(CommBus_t *bus, uint32_t now_ms, uint32_t send_timeout_ms){
    if (bus->busy && (now_ms - bus->last_send_ms > send_timeout_ms)) {
        bus->busy = 0;
        bus->tx_node_idx = -1;
        EN_Rx(bus);                                        // 回到接收/空闲
        // 可在此增加：统计超时次数、跳过问题节点等策略（保持你代码逻辑不变，仅注释）
    }
}

/* ==== HAL 串口回调（在 main.c 转发到这里）==== */

// Tx 完成：切换到接收态并启动一次接收（“发完立收”的关键一步）
// -------------------------------------------------------------------------------------------------
// 注意：使用 Receive_IT 立即启动固定长度的接收（长度为协议定义的反馈帧大小）。
void CommBus_OnTxCplt(CommBus_t *bus, UART_HandleTypeDef *huart){

    if (huart != bus->huart) return;                       // 过滤其他串口的回调

    // Tx完成：切到Rx、LED灭，然后启动一次接收
    EN_Rx(bus);
		HAL_UART_AbortReceive(huart);
    int idx = (bus->tx_node_idx<0)? 0 : bus->tx_node_idx;  // 兜底：理论上应始终 ≥0
    if (HAL_UART_Receive_IT(bus->huart,
            (uint8_t*)&bus->nodes[idx].data.motor_recv_data,
            sizeof(bus->nodes[idx].data.motor_recv_data)) != HAL_OK)
    {
        // 接收启动失败也要回收，避免 busy 卡死
        bus->busy = 0;
        bus->tx_node_idx = -1;
    }
}

// Rx 完成：解析反馈帧 → 清 busy → 回到接收/空闲态
// -------------------------------------------------------------------------------------------------
// 说明：extract_data() 应完成 CRC/字段拆解等；解析后数据留存在 nodes[idx].data 内供上层读取。
void CommBus_OnRxCplt(CommBus_t *bus, UART_HandleTypeDef *huart){
    if (huart != bus->huart) return;

    // 收到一帧：解析→清busy→回到接收态
    int idx = (bus->tx_node_idx<0)? 0 : bus->tx_node_idx;
    extract_data(&bus->nodes[idx].data);                   // >>> 协议层：解析 motor_recv_data <<<

    bus->busy = 0;
    bus->tx_node_idx = -1;

    EN_Rx(bus);
}

// 串口错误：清 busy 并回到接收/空闲（帧错/噪声/溢出/超时等）
// -------------------------------------------------------------------------------------------------
// 处理策略：简单回收。你可以在此计数并上报错误，或对问题节点做退避/重试（保留接口不更改代码）。
void CommBus_OnError(CommBus_t *bus, UART_HandleTypeDef *huart){
    if (huart != bus->huart) return;

    // 错误/超时：强制回收
    bus->busy = 0;
    bus->last_send_ms = HAL_GetTick();                     // 记录时刻，便于上层度量“错误后冷却”
    bus->tx_node_idx = -1;
    EN_Rx(bus);
}
