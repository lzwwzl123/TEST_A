/*
*********************************************************************************************************
*	                                  
*	模块名称 : AD7606驱动模块 
*	文件名称 : bsp_ad7606.h
*	版    本 : V1.3
*	说    明 : 头文件
*********************************************************************************************************
*/

#include "stdint.h"
#include "main.h"

#ifndef __BSP_AD7606_H
#define __BSP_AD7606_H

// DWT微秒延时函数
#define DWT_CTRL        (*(volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT      (*(volatile uint32_t *)0xE0001004)
#define CPU_FREQUENCY   168 // 你的HCLK频率，单位MHz

static inline void DWT_Delay_Init(void) {
    // 1. 使能DWT/ITM单元
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    // 2. 清零计数器
    DWT_CYCCNT = 0;
    // 3. 使能CYCCNT计数器
    DWT_CTRL |= 1;  
}

static inline void DWT_Delay_us(uint32_t us) {
    uint32_t start_tick = DWT_CYCCNT;
    uint32_t delay_ticks = us * CPU_FREQUENCY;
    while (DWT_CYCCNT - start_tick < delay_ticks);
}

extern TIM_HandleTypeDef htim4;
extern SPI_HandleTypeDef hspi1;


/* 每个样本2字节，采集通道 */
#define CH_NUM			8				/* 采集2通道 */
#define FIFO_SIZE		2048

#define AD_CS_LOW()     				HAL_GPIO_WritePin(AD_CS_GPIO_Port, AD_CS_Pin, GPIO_PIN_RESET);
#define AD_CS_HIGH()     				HAL_GPIO_WritePin(AD_CS_GPIO_Port, AD_CS_Pin, GPIO_PIN_SET);

#define AD_RESET_LOW()					HAL_GPIO_WritePin(AD_RESET_GPIO_Port, AD_RESET_Pin, GPIO_PIN_RESET);
#define AD_RESET_HIGH()					HAL_GPIO_WritePin(AD_RESET_GPIO_Port, AD_RESET_Pin, GPIO_PIN_SET);
	
#define AD_CONVST_LOW()					HAL_GPIO_WritePin(AD_CONVST_GPIO_Port, AD_CONVST_Pin, GPIO_PIN_RESET);
#define AD_CONVST_HIGH()				HAL_GPIO_WritePin(AD_CONVST_GPIO_Port, AD_CONVST_Pin, GPIO_PIN_SET);

#define AD_RANGE_5V()						HAL_GPIO_WritePin(AD_RANGE_GPIO_Port, AD_RANGE_Pin, GPIO_PIN_RESET);
#define AD_RANGE_10V()					HAL_GPIO_WritePin(AD_RANGE_GPIO_Port, AD_RANGE_Pin, GPIO_PIN_SET);

#define AD_OS0_0()							HAL_GPIO_WritePin(AD_OS0_GPIO_Port, AD_OS0_Pin, GPIO_PIN_RESET);
#define AD_OS0_1()							HAL_GPIO_WritePin(AD_OS0_GPIO_Port, AD_OS0_Pin, GPIO_PIN_SET);

#define AD_OS1_0()							HAL_GPIO_WritePin(AD_OS1_GPIO_Port, AD_OS1_Pin, GPIO_PIN_RESET);
#define AD_OS1_1()							HAL_GPIO_WritePin(AD_OS1_GPIO_Port, AD_OS1_Pin, GPIO_PIN_SET);

#define AD_OS2_0()							HAL_GPIO_WritePin(AD_OS2_GPIO_Port, AD_OS2_Pin, GPIO_PIN_RESET);
#define AD_OS2_1()							HAL_GPIO_WritePin(AD_OS2_GPIO_Port, AD_OS2_Pin, GPIO_PIN_SET);

/* AD数据采集缓冲区 */
typedef struct
{
	volatile uint16_t usRead; 
  volatile uint16_t usWrite;
	int16_t usBuf[FIFO_SIZE];
}FIFO_T;

/* 供外部调用的函数声明 */
void ad7606_Sync(void);
void ad7606_Reset(void);
void ad7606_SetOS(uint8_t _ucMode);
void bsp_InitAD7606(void);
void ad7606_StartConv(void);
void ad7606_StartRecord(void);
void ad7606_StopRecord(void);
void ad7606_IRQSrc(void);
uint8_t GetAdcFormFifo(int16_t *_usReadAdc);
uint8_t GetAdcFrame(int16_t* frame_buffer);

extern FIFO_T  g_tAD;

#endif


