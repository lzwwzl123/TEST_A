/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
	
	
	/* =================================================================================================
 * -------------------------------------------------------------------------------------------------
 * 模块职责：
 *   - 完成系统上电初始化（HAL/CubeMX 外设、时钟、GPIO、DMA、USART、TIM、IWDG）。
 *   - 构建并持有一个全局的应用对象 App_t（包含通信总线、控制器、期望/实际状态、轨迹等）。
 *   - 将 Keil/IDE 调试“面板参数”（g_dbg）应用到系统：轨迹参数、PID、轴使能、归零等。
 *   - 以 TIM2 周期中断为控制循环节拍源，回调中执行 App_Tick(&g_app) 完成 1 个控制周期。
 *   - 主循环中负责：应用面板修改、维护通信看门狗、上报状态（串口2打印）、轻负载延时。
 *
 * 通信要点（本项目为 RS485 半双工，见 usart.c/comm 层）：
 *   - 发送方向：置 DE=1 → 发送 → 等待 TX 完成（TC=1）→ 按需等待回转延时 → DE=0。
 *   - 接收方向：RE 使能（常为低有效/高有效视收发器），空闲/IDLE 用于帧边界判定。
 *   - 多节点（4 台电机）地址 node_ids[] 用于主站轮询/广播。
 *
 * 平面绳驱并联机器人（2D-CDPR）控制链路：
 *   传感采样 → 几何/运动学映射 → 张力/长度分配 → 轴级环路（PID/限幅/反饱和）→ RS485 下发。
 *
 * 安全/鲁棒性关注点（建议在控制/comm层实现，主控此处仅提示）：
 *   - 看门狗超时、串口帧超时/重发、CRC 校验失败统计。
 *   - 轨迹切换/参数热更新时的“原子性”：先写 g_app.desired，再触发 BuildTrajectoryFromParams()。
 *   - 物理限幅：避免积分风up（use_cond_i / i_deadband 已体现在 PIDPanel）。
 * =================================================================================================*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <math.h>
#include "app.h"
#include <stdio.h>
#include "bsp_spi_ad7606.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define KK  9.042831f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile int flag_solve = 0;
volatile int flag_solv = 0;
int16_t my_adc_frame[CH_NUM] = {0};
volatile int16_t adc_frame[8] = {0};
static App_t g_app;		// 全局应用对象（控制/通信/轨迹都在里面）
/* —— 全局调试面板（Keil里直接改它即可）—— */
DebugPanel_t g_dbg = {
    /* zero_now */      0,
    /* mode */          1,              /* 默认直线 */
		/* error_code */    0,
    /* apply_motion */  0,
		/* start_motion */		0,
    /* line */          { 0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f },
    /* arc  */          { 0.0f, 0.0f, 0.10f, 0.0f, (float)M_PI/1.0f, 0.0f, 0.0f },
    /* sampler */       { 150, 150, 0 },
    /* pid[] */         
        {10.0f ,1000.0f, 0.4f, 0.001f, -20.0f,20.0f, 100.0f, 1, 0.0f, 0.0005f, 4.5, 6.5f},
    /* apply_pid */     0,	 
    /* axis_enable */   {1,1,1,1},

};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void ApplyDebugPanelIfNeeded(void);
void IK_CDPR_SetL0(const float L0_m[4]);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ===== HAL 串口回调：把事件转发给通信层（comm_rs485.c）
 * 注意：在 stm32f4xx_it.c 的 USARTx_IRQHandler() 里必须调用 HAL_UART_IRQHandler(&huartx)，
 * HAL 才会触发这些回调。CubeMX 默认会生成。
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){ CommBus_OnTxCplt(&g_app.bus, huart); flag_solv++;}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){ CommBus_OnRxCplt(&g_app.bus, huart); }
void HAL_UART_ErrorCallback (UART_HandleTypeDef *huart){ CommBus_OnError (&g_app.bus, huart); }

/* TIM2 4kHz：执行本周期控制（段内插值→相对零位PID→下发） */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
				ad7606_IRQSrc();
        App_Tick(&g_app);
		if (GetAdcFrame(my_adc_frame))
			{
					for(int i=0;i<8;i++){
						adc_frame[i] = my_adc_frame[i];
					}
			}
    }
}



/* —— 将调试面板改动“应用”到系统（主循环中调用）—— */
static void ApplyDebugPanelIfNeeded(void)
{
    for (int i=0; i<APP_MOTORS; i++) {
        Control_EnableAxis(&g_app.ctl, i, g_dbg.axis_enable[i] ? 1 : 0);
    }

    if (g_dbg.zero_now) {
        App_ZeroNow(&g_app);
        g_dbg.zero_now = 0;
    }

    // 当用户点击 apply_motion 时，才同步所有轨迹参数
    if (g_dbg.apply_motion) {
        g_dbg.error_code = 0; // 清除上一次的错误码

        if (g_dbg.mode == 1) {
            App_SetLine(&g_app,
                (TP2_Point){g_dbg.line.p0x, g_dbg.line.p0y},
                (TP2_Point){g_dbg.line.p1x, g_dbg.line.p1y},
                g_dbg.line.yaw0, g_dbg.line.yawf);
        } else if (g_dbg.mode == 2) {
            App_SetArc(&g_app,
                (TP2_Point){g_dbg.arc.cx, g_dbg.arc.cy},
                g_dbg.arc.R, g_dbg.arc.phi0, g_dbg.arc.phi1,
                g_dbg.arc.yaw0, g_dbg.arc.yawf);
        }
        
        // 必须同时同步时间轴参数
        g_app.desired.freq = g_dbg.sampler.freq;
        g_app.desired.points_duration = g_dbg.sampler.points_duration;
        
        g_dbg.apply_motion = 0;
    }

    if (g_dbg.apply_pid) {
        for (int i=0; i<APP_MOTORS; i++) {
            PIDPanel_t p = g_dbg.pid;
            Control_ConfigAxisPID(&g_app.ctl, i, p.kp, p.ki, p.kd, p.d_alpha, p.out_min, p.out_max, p.accel_max, p.use_cond_i, p.u_init, p.i_deadband, p.kp_scale_threshold, p.kp_near);
        }
        g_dbg.apply_pid = 0;
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	
	//run_jacobian_validation_test();
	bsp_InitAD7606();
	ad7606_Sync();
	ad7606_StartRecord(); 
	const uint8_t node_ids[APP_MOTORS] = {0,1,2,3};
	App_Init(&g_app, &huart1,
         0, 0,
         LED_GPIO_Port, LED_Pin,
         node_ids);
	{
  /* 用你的实测/标定值替换下面四个数（单位：米） */
  const float L0[4] = { 0.33662331f, 0.33662331f, 0.33662331f, 0.33662331f };
  IK_CDPR_SetL0(L0);
}


	// 1. 将 g_dbg 的初始值同步到 app->desired
	g_dbg.apply_motion = 1; // 伪造一次点击
	ApplyDebugPanelIfNeeded();

	// 2. 将 desired 同步到 applied 并构建初始轨迹
	g_app.applied = g_app.desired;
	BuildTrajectoryFromParams(&g_app, &g_app.applied);

	// 3. 初始状态必须是空闲，等待您的指令
	g_app.motion_state = MOTION_IDLE; 

	/* 启动 4kHz 控制中断 */
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim4);
		
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		ApplyDebugPanelIfNeeded(); // 把 Keil 面板的更改落实到系统
		
    App_MainLoop(&g_app);      // 参数变化 → 自动重建轨迹；通信看门狗
		// 串口调试
//		char print_buf[256];
//		// 在数据最前面加上 "S,"，最后面加上 ",E"
//		snprintf(print_buf, sizeof(print_buf), "S,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,E\n",
//						 g_app.pos_set_rel[0]*KK, g_app.pos_meas_rel[0]*KK,  // 电机 1
//						 g_app.pos_set_rel[1]*KK, g_app.pos_meas_rel[1]*KK,  // 电机 2
//						 g_app.pos_set_rel[2]*KK, g_app.pos_meas_rel[2]*KK,  // 电机 3
//						 g_app.pos_set_rel[3]*KK, g_app.pos_meas_rel[3]*KK); // 电机 4
//		HAL_UART_Transmit(&huart2, (uint8_t*)print_buf, strlen(print_buf), 50);
		bool fk_success = App_FK_FromMotorPos(g_app.pos_meas_rel,	&g_app.current_pos);
		if (fk_success) {
							flag_solve++;
    } else {
        // 正解失败！
        // 这可能意味着机器人接近奇异点，或者迭代发散了。
        // 在这种情况下，g_current_platform_pose 保持为上一周期的值。
        // 你需要在这里添加错误处理逻辑，例如：
        // - 保持上一周期的控制输出
        // - 发出一个警告
        // - 切换到更安全的控制模式
			flag_solve--;
    }
							 
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
