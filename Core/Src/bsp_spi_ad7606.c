/*
*********************************************************************************************************
*	                                  
*	模块名称 : AD7606驱动模块
*	文件名称 : bsp_spi_ad7606.c
*	版    本 : V1.3
*	说    明 : 驱动AD7606 ADC转换器 SPI接口
*
*
*********************************************************************************************************
*/

#include <stdio.h>
#include "bsp_spi_ad7606.h"
volatile int flag1 = 0;
volatile int flag2 = 0;
FIFO_T	g_tAD;	/* 定义一个交换缓冲区 */
/**
  * @brief  执行一次牺牲性的读取，以同步AD7606和STM32的SPI时序
  * @note   此函数应在启动定时器采集前调用，用于清除SPI硬件缓冲区中的陈旧数据。
  * @param  None
  * @retval None
  */
void ad7606_Sync(void)
{
    // ====================== 阶段一：硬件强制冲刷 ======================
    // 目的：暴力清除STM32 SPI外设在初始化后可能残留的任何垃圾数据。
    
    // 1. 短暂地禁用SPI外设，这会复位一部分内部状态机。
    __HAL_SPI_DISABLE(&hspi1);

    // 2. 【核心操作】循环读取数据寄存器，直到“接收不为空(RXNE)”标志位被清空。
    //    这是最彻底、最可靠的清空硬件接收缓冲区的方法。
    while (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_RXNE))
    {
        // 只读操作，将读出的垃圾数据丢弃。
        // (void) 是为了告诉编译器，我们故意不使用这个返回值。
        (void)hspi1.Instance->DR;
    }

    // 3. 重新使能SPI外设，让它处于一个绝对干净的初始状态。
    __HAL_SPI_ENABLE(&hspi1);


    // ====================== 阶段二：逻辑同步 ======================
    // 目的：在干净的硬件上，执行一次牺牲性的读写，以同步AD7606的通道序列。

    uint16_t dummy_buffer[CH_NUM];

    // 4. 启动一次转换，并等待其完成
    ad7606_StartConv();
    HAL_Delay(1); 

    // 5. 执行一次阻塞式的SPI读取，目的是让AD7606的内部通道序列发生器复位。
    AD_CS_LOW();
    HAL_SPI_Receive(&hspi1, (uint8_t*)dummy_buffer, CH_NUM, 10);
    AD_CS_HIGH();
    
    // 6. 【关键】为第一次真正的中断，提前启动一次新的转换。
    ad7606_StartConv();
}

/*
*********************************************************************************************************
*	函 数 名: bsp_InitAD7606
*	功能说明: 初始化AD7606 SPI口线
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitAD7606(void)
{
	/* 设置过采样模式 */
	ad7606_SetOS(0);
	AD_RANGE_10V();
	/* 设置GPIO的初始状态 */
	ad7606_Reset();				/* 硬件复位复AD7606 */
	
	AD_CONVST_HIGH();			/* CONVST脚设置为高电平 */	
	AD_CS_HIGH();

}

/*
*********************************************************************************************************
*	函 数 名: ad7606_Reset
*	功能说明: 硬件复位AD7606
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void ad7606_Reset(void)
{
	/* AD7606是高电平复位，要求最小脉宽50ns */
	
//	AD_RESET_LOW();
//	DWT_Delay_us(1); // 保证脉宽 > 50ns
//	AD_RESET_HIGH();
//	DWT_Delay_us(1); // 保证脉宽 > 50ns
//	AD_RESET_LOW();
	AD_RESET_LOW();
	DWT_Delay_us(1);
	AD_RESET_HIGH();
	AD_RESET_HIGH();
	AD_RESET_HIGH();
	AD_RESET_HIGH();
	DWT_Delay_us(1);
	AD_RESET_LOW();
}

/*
*********************************************************************************************************
*	函 数 名: ad7606_SetOS
*	功能说明: 设置过采样模式（数字滤波，硬件求平均值)
*	形    参：_ucMode : 0-6  0表示无过采样，1表示2倍，2表示4倍，3表示8倍，4表示16倍
*				5表示32倍，6表示64倍
*	返 回 值: 无
*********************************************************************************************************
*/
void ad7606_SetOS(uint8_t _ucMode)
{
	if (_ucMode == 1)
	{
		AD_OS2_0();
		AD_OS1_0();
		AD_OS0_1();
	}
	else if (_ucMode == 2)
	{
		AD_OS2_0();
		AD_OS1_1();
		AD_OS0_0();
	}
	else if (_ucMode == 3)
	{
		AD_OS2_0();
		AD_OS1_1();
		AD_OS0_1();
	}
	else if (_ucMode == 4)
	{
		AD_OS2_1();
		AD_OS1_0();
		AD_OS0_0();
	}
	else if (_ucMode == 5)
	{
		AD_OS2_1();
		AD_OS1_0();
		AD_OS0_1();
	}
	else if (_ucMode == 6)
	{
		AD_OS2_1();
		AD_OS1_1();
		AD_OS0_0();
	}
	else	/* 按0处理 */
	{
		AD_OS2_0();
		AD_OS1_0();
		AD_OS0_0();
	}
}

/*
*********************************************************************************************************
*	函 数 名: ad7606_StartConv
*	功能说明: 启动AD7606的ADC转换
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void ad7606_StartConv(void)
{
//	/* 上升沿开始转换，低电平持续时间至少25ns  */
//	AD_CONVST_LOW();
//	DWT_Delay_us(1); // 保证脉宽 > 50ns
//	AD_CONVST_HIGH();
		/* 上升沿开始转换，低电平持续时间至少25ns  */
	AD_CONVST_LOW();
	AD_CONVST_LOW();
	AD_CONVST_LOW();	/* 连续执行2次，低电平约50ns */
	AD_CONVST_LOW();
	AD_CONVST_LOW();
	AD_CONVST_LOW();
	AD_CONVST_LOW();
	AD_CONVST_LOW();
	AD_CONVST_LOW();
	AD_CONVST_HIGH();
}


/*
*********************************************************************************************************
*	函 数 名: ad7606_IRQSrc
*	功能说明: 定时调用本函数，用于读取AD转换器数据
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void ad7606_IRQSrc(void)
{
	/* 读取数据 */
	AD_CS_LOW();
    // 使用硬件SPI一次性接收所有通道数据
   if (HAL_SPI_Receive(&hspi1, (uint8_t*)&g_tAD.usBuf[g_tAD.usWrite], CH_NUM, 10) == HAL_OK)
   {
       g_tAD.usWrite += CH_NUM;
       if (g_tAD.usWrite >= FIFO_SIZE)
       {
           g_tAD.usWrite = 0; // 环形缓冲区
       }
			 flag2++;
   }
	AD_CS_HIGH();
	ad7606_StartConv();
}

/*
*********************************************************************************************************
*	函 数 名: GetAdcFormFifo
*	功能说明: 从FIFO中读取一个ADC值
*	形    参：_usReadAdc : 存放ADC结果的变量指针
*	返 回 值: 1 表示OK，0表示暂无数据
*********************************************************************************************************
*/
uint8_t GetAdcFormFifo(int16_t *_usReadAdc)
{
    uint8_t result = 0;

    // 在访问共享变量前，禁止中断
    __disable_irq(); 

    if (g_tAD.usRead != g_tAD.usWrite)
    {
        *_usReadAdc = g_tAD.usBuf[g_tAD.usRead];

        if (++g_tAD.usRead >= FIFO_SIZE)
        {
            g_tAD.usRead = 0;
        }
        result = 1;
    }
    
    // 访问完毕后，重新使能中断
    __enable_irq();

    return result;
}

uint8_t GetAdcFrame(int16_t* frame_buffer)
{
    uint16_t data_count;

    // 1. 安全地检查FIFO中是否有足够的数据
    // -- 进入临界区 --
    __disable_irq();
    if (g_tAD.usWrite >= g_tAD.usRead)
    {
        data_count = g_tAD.usWrite - g_tAD.usRead;
    }
    else
    {
        data_count = FIFO_SIZE - g_tAD.usRead + g_tAD.usWrite;
    }
    __enable_irq();
    // -- 退出临界区 --

    // 2. 判断数据量是否足够
    if (data_count >= CH_NUM)
    {
        // 3. 数据充足，循环读取一整帧
        for (int i = 0; i < CH_NUM; i++)
        {
            // 调用底层的GetAdcFormFifo来取出每个样本
            GetAdcFormFifo(&frame_buffer[i]);
        }
        return 1; // 返回成功
    }
    else
    {
        // 4. 数据不足，直接返回失败
        return 0;
    }
}
/*
*********************************************************************************************************
*	函 数 名: ad7606_StartRecord
*	功能说明: 开始采集
*	形    参：_ulFreq : 采样频率, 单位 HZ
*	返 回 值: 无
*********************************************************************************************************
*/
void ad7606_StartRecord(void)
{
		
    g_tAD.usRead = 0;
    g_tAD.usWrite = 0;

    
    // 启动TIM4的中断
    // &htim4 是CubeMX生成的TIM4句柄
    HAL_TIM_Base_Start_IT(&htim4); 
}

/*
*********************************************************************************************************
*	函 数 名: ad7606_StopRecord
*	功能说明: 停止采集
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void ad7606_StopRecord(void)
{
	HAL_TIM_Base_Stop_IT(&htim4);
}
