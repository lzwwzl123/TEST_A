#ifndef COMM_RS485_H
#define COMM_RS485_H

#include <stdint.h>
#include <stdbool.h>
#include "usart.h"
#include "gpio.h"
#include "gom_protocol.h"  // 你已有：定义 MotorCmd_t / MotorData_t / modify_data()/extract_data()
#include "app_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===========================================================================================
 * 文件：comm_rs485.h
 * 职责（仅注释，不改动接口）：
 *   - 定义“主站 RS485 总线”对象与对外 API。主站以**轮询**方式与 4 台电机节点通信：
 *       1) 选择一个节点 → 组帧并发送命令（如速度/模式等）
 *       2) 立刻切换为接收 → 等待该节点的反馈帧
 *       3) 解析反馈（extract_data）→ 轮到下一个节点
 *   - 若接入了 RS485 收发器的 **RE/DE** 管脚，则在 Tx/Rx 之间切换方向；
 *     若硬件自动方向控制或未接线，则 `rede_port=NULL, rede_pin=0` 即可。
 *   - 可选 **LED/en** 引脚：发送期间置高，接收/空闲时置低，便于现场调试。
 *
 * 时序/安全要点：
 *   - 半双工：发送前 **DE=1**；发送完等待 TX 完成（TC=1）与必要的 **回转延时** 再 **DE=0** 进入接收。
 *   - 超时：若在设定时间内未收到反馈，ServiceTick() 应强制回收为接收/空闲，清掉 busy，避免死锁。
 *   - 帧编解码：modify_data() 负责把 MotorCmd_t 写入发送帧；extract_data() 负责把收到的字节流解析到 MotorData_t。
 *   - 中断回调：在 HAL 的 TxCplt/RxCplt/Error 回调里**仅转发**到 CommBus_On*()，具体状态机在 .c 实现。
 * ===========================================================================================*/

/* -----------------------------------------
 * 单个从站节点的缓存（命令/数据）
 * -----------------------------------------*/
typedef struct {
    uint8_t    id;     ///< 从站 ID（由上层传入的 node_ids 映射）
    MotorCmd_t cmd;    ///< 待发送命令帧的数据结构（由 modify_data() 序列化为字节流）
    MotorData_t data;  ///< 最近一次收到并解析好的反馈数据（由 extract_data() 解析得到）
} CommNode_t;

/* -----------------------------------------------------------
 * 主站总线对象（面向轮询、半双工方向控制与超时看门狗）
 * -----------------------------------------------------------*/
typedef struct {
    UART_HandleTypeDef *huart;                  ///< 使用的串口句柄（如 USART1；建议开启 IDLE/TC 等中断）
    GPIO_TypeDef *rede_port; uint16_t rede_pin; ///< RS485 RE/DE 引脚（合并控制）；若无则传 NULL,0
    GPIO_TypeDef *en_port;   uint16_t en_pin;   ///< 可选 LED/使能引脚；若不接 LED 则传 NULL,0

    volatile uint8_t  busy;           ///< 1=当前正处于一次“Tx→Rx”事务中（避免并发重入）
    volatile uint32_t last_send_ms;   ///< 最近一次开始发送的时间戳（ms，用于超时判定）
    int tx_node_idx;                  ///< 当前正在交互（已发、待收）的节点索引（0..APP_MOTORS-1）
    int next_rr_idx;                  ///< 下次轮询的起始索引（轮询指针，避免总从 0 开始）

    CommNode_t nodes[APP_MOTORS];     ///< 4 个节点的命令/反馈缓存
} CommBus_t;

/**
 * @brief  初始化 RS485 主站总线对象（仅状态与外设句柄的绑定，不做阻塞通信）
 * @param  bus         总线对象
 * @param  huart       串口句柄（半双工/全双工均可，方向由 RE/DE 决定）
 * @param  rede_port   RE/DE GPIO 端口；若硬件自动方向或未接，传 NULL
 * @param  rede_pin    RE/DE 引脚号；未接传 0
 * @param  en_port     LED/使能 GPIO 端口；未接传 NULL
 * @param  en_pin      LED/使能引脚号；未接传 0
 * @param  node_ids    从站地址表（长度 APP_MOTORS）
 * @note
 *   - 内部会把 node_ids[i] 写到 nodes[i].id，并清零 busy/指针/时间戳。
 *   - 不会启动任何 DMA/IT；真正通信由 TryStepRoundRobin() 与回调驱动。
 */
void CommBus_Init(CommBus_t *bus,
                  UART_HandleTypeDef *huart,
                  GPIO_TypeDef *rede_port, uint16_t rede_pin,
                  GPIO_TypeDef *en_port,   uint16_t en_pin,
                  const uint8_t *node_ids);

/**
 * @brief  进行一次“尝试轮询发送”：若总线空闲，则开始对下一个节点的 Tx→Rx 事务
 * @param  bus 总线对象
 * @return true：成功启动本轮节点事务（已置 busy）；false：当前忙或资源不就绪
 * @note
 *   - 典型用法：在控制周期或主循环末尾调用一次；若返回 true，实际的收发依赖 HAL 回调推进。
 *   - 方向控制：开始 Tx 前置 DE=1/LED=SET；Tx 完成回调中切 Rx 并 DE=0（视硬件极性）。
 */
bool CommBus_TryStepRoundRobin(CommBus_t *bus);

/**
 * @brief  看门狗：处理发送/事务超时，防止死锁
 * @param  bus             总线对象
 * @param  now_ms          当前时间（ms），如 HAL_GetTick()
 * @param  send_timeout_ms 发送到接收的事务允许最长耗时
 * @note
 *   - 若 busy==1 且 (now_ms - last_send_ms) > 超时，则强制回收到接收/空闲态，清 busy，熄灭 LED。
 *   - 可选择统计超时次数/重试等（在 .c 实现里处理），并推进 next_rr_idx。
 */
void CommBus_ServiceTick(CommBus_t *bus, uint32_t now_ms, uint32_t send_timeout_ms);

/* ---------------- HAL 串口回调转发（必须从用户回调中调用） ----------------
 * 你只需在 main.c 的回调中调用下列函数，把 HAL 事件转交给通信状态机：
 *   void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) { CommBus_OnTxCplt(&g_app.bus, huart); }
 *   void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) { CommBus_OnRxCplt(&g_app.bus, huart); }
 *   void HAL_UART_ErrorCallback (UART_HandleTypeDef *huart) { CommBus_OnError (&g_app.bus, huart); }
 * 实际工作（切换方向、启动接收、解析帧、推进轮询）在 .c 文件中完成。
 * ------------------------------------------------------------------------*/

/**
 * @brief  串口“发送完成”回调的业务处理
 * @param  bus   总线对象
 * @param  huart 触发回调的串口句柄（用于比对与保护）
 * @note   - 典型流程：检查 huart→（必要时等待 TC）→ 切换为接收（DE=0, 使能 Rx DMA/IT）。
 */
void CommBus_OnTxCplt(CommBus_t *bus, UART_HandleTypeDef *huart);

/**
 * @brief  串口“接收完成”回调的业务处理
 * @param  bus   总线对象
 * @param  huart 触发回调的串口句柄
 * @note   - 典型流程：extract_data() 解析 → 写入 nodes[tx_node_idx].data → 清 busy → next_rr_idx++。
 */
void CommBus_OnRxCplt(CommBus_t *bus, UART_HandleTypeDef *huart);

/**
 * @brief  串口错误回调的业务处理（帧错/噪声/超时等）
 * @param  bus   总线对象
 * @param  huart 触发回调的串口句柄
 * @note   - 典型流程：统计/清标志 → 回收为接收/空闲 → 视策略重试或跳到下个节点。
 */
void CommBus_OnError (CommBus_t *bus, UART_HandleTypeDef *huart);

/* ---------------- 便捷访问器：不做边界检查，请确保 i 合法 ---------------- */
static inline CommNode_t*       CommBus_GetNode(CommBus_t *bus, int i){ return &bus->nodes[i]; }
static inline MotorCmd_t*       CommBus_GetCmd (CommBus_t *bus, int i){ return &bus->nodes[i].cmd; }
static inline const MotorData_t*CommBus_GetData(const CommBus_t *bus, int i){ return &bus->nodes[i].data; }

#ifdef __cplusplus
}
#endif
#endif // COMM_RS485_H
