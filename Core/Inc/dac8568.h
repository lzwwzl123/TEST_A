#ifndef __DAC8568_H__
#define __DAC8568_H__

#include <inttypes.h>
#include <stdbool.h>

/* ±5V版本：0   ±10V版本：1 */
#define DAC8568_VERSION 1

/* 不同版本的电压范围 用户无需修改 */
#if DAC8568_VERSION
#define MAX_VOLTAGE 10.0
#define MIN_VOLTAGE -10.0
#else
#define MAX_VOLTAGE 5.0
#define MIN_VOLTAGE -5.0
#endif

#define VREF 2500.0 /* 基准电压 单位毫伏（mv） */

/**
 * @brief DAC8568命令
 *
 */
enum DAC8568_Commands {
  Write_DAC_Input_Reg = 0x00, /* 写DAC输入寄存器 */
  Update_DAC_Reg,             /* 更新DAC寄存器 */
  Write_Update_All_DAC_Input_Reg, /* 写DAC输入寄存器并更新所有DAC寄存器 */
  Write_Update_Respective_DAC_Reg, /* 写DAC输入寄存器并更新DAC寄存器 */
  Software_Reset = 0x07,           /* 软件复位 */
  Internal_Reference = 0x08,       /* 基准电压配置 */
};

/**
 * @brief DAC8568通道
 *
 */
enum DAC8568_Channels {
  DAC_Channel_A = 0x00,    /* 通道A */
  DAC_Channel_B,           /* 通道B */
  DAC_Channel_C,           /* 通道C */
  DAC_Channel_D,           /* 通道D */
  DAC_Channel_E,           /* 通道E */
  DAC_Channel_F,           /* 通道F */
  DAC_Channel_G,           /* 通道G */
  DAC_Channel_H,           /* 通道H */
  All_DAC_Channels = 0x0F, /* 全通道 */
};

void DAC8568_Init(void);
void DAC8568_Reset(void);
void DAC8568_Internal_Reference_Config(bool state);
void DAC8568_Set_Direct_Current(uint8_t channel, double voltage);
void DAC8568_Set_Channel_Wave(uint8_t channel, double *wave_data,
                              uint16_t data_size);

#endif
