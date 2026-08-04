/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_config.h"
#include "bsp_rs485.h"
#include "ld2_motor.h"
#include "app_chassis_motor_ctrl.h"
#include "bsp_vofa.h"
#include "bsp_rc.h"
#include "app_ld2rs_task.h"
#include "app_led_indicator.h"
#include "arm_math.h"
#include "bsp_dwt.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void App_Vofa_UpperDisplay(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
//  float32_t sine = arm_sin_f32(12);
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
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_UART5_Init();
  MX_SPI6_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */


  /* DWT 使用 480 MHz 内核周期提供微秒时间戳。 */
  if (!BSP_DWT_Init())
  {
    Error_Handler();
  }
	if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  /* BSP 层：初始化 RS485 总线 (硬件 DE 由 CubeMX HAL_RS485Ex_Init 配置) */
  bsp_rs485_init(&g_rs485_bus, &huart3);

  /* 中间件层：初始化 LD2-RS 设备句柄 */
  /* 电机1 (ID=16) */
  ld2_motor_init(&g_ld2rs_dev_m1, &g_rs485_bus, APP_LD2_MOTOR_SLAVE_ID_M1, APP_LD2_MOTOR_TIMEOUT_MS);

  /* 电机2 (ID=17) */
  ld2_motor_init(&g_ld2rs_dev_m2, &g_rs485_bus, APP_LD2_MOTOR_SLAVE_ID_M2, APP_LD2_MOTOR_TIMEOUT_MS);

  // /* 等待驱动器能接收 Modbus */
  // HAL_Delay(APP_SYSTEM_START_DELAY_MS);

  /* 应用层：初始化电机状态机 */
    /* 电机1 (ID=1) */
  app_chassis_ld2rs_motor_ctrl_init(&g_ld2rs_motor_ctrl_m1, &g_ld2rs_dev_m1, APP_MOTOR_CTRL_REF_SPEED_RPM_M1, 1u);
	
    /* 电机2 (ID=2) */
  app_chassis_ld2rs_motor_ctrl_init(&g_ld2rs_motor_ctrl_m2, &g_ld2rs_dev_m2, APP_MOTOR_CTRL_REF_SPEED_RPM_M2, 2u);

  /* 读回驱动器参数 (仅读不写, 存入设备句柄) */
  app_chassis_ld2rs_motor_ctrl_param_read_back(&g_ld2rs_dev_m1);
  app_chassis_ld2rs_motor_ctrl_param_read_back(&g_ld2rs_dev_m2);

  /* 硬写入 M1 速度环 PI */
  app_chassis_ld2rs_motor_ctrl_pi_init(&g_ld2rs_dev_m1, &g_ld2rs_motor_param_m1);
  /* 硬写入 M2 速度环 PI */
  app_chassis_ld2rs_motor_ctrl_pi_init(&g_ld2rs_dev_m2, &g_ld2rs_motor_param_m2);

  /* 启动遥控接收 (UART5 DMA+IDLE) */
  bsp_rc_init(&g_rc);

  /* 初始化非阻塞电机控制状态机 */
  app_ld2rs_task_init();

  /* WS4810 LED 状态指示 */
  app_led_indicator_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    app_ld2rs_task_run();           /* 差速解算 + M1/M2 Modbus 状态机 (非阻塞)  */

#if APP_VOFA_JUSTFLOAT_ENABLE
    App_Vofa_UpperDisplay();        /* VOFA+ 波形 */
#endif

    app_led_indicator_update();     /* WS4810 状态灯 */
	
    /* USART1 (PA9/PA10, 115200 8N1) available for debug output via USB-TTL.
     *  Use HAL_UART_Transmit(&huart1, buf, len, timeout_ms) — blocking, not for ISR. */
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 30;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* VOFA+ JustFloat 转速通道数据 (MotorRuntime 状态机周期写入) */
vofa_motor_info_t g_vofa_speed = {0.0f};

static void App_Vofa_UpperDisplay(void)
{
  extern volatile uint8_t g_emergency_stop_flag;
    /* VOFA+ JustFloat 发送: 每 APP_VOFA_JUSTFLOAT_PERIOD_MS 一帧 */
  {
      static uint32_t s_u32LastVofaTick = 0u;
      uint32_t now_tick = HAL_GetTick();
      if ((now_tick - s_u32LastVofaTick) >= APP_VOFA_JUSTFLOAT_PERIOD_MS) {
          float vofa_channels[19] = {
              (float)g_vofa_speed.m1_ref_speed_rpm,          /* CH01 */
              (float)g_vofa_speed.m1_feedback_speed_rpm,     /* CH02 */

              (float)g_vofa_speed.m2_ref_speed_rpm,          /* CH03 */
              (float)g_vofa_speed.m2_feedback_speed_rpm,     /* CH04 */

              (float)g_emergency_stop_flag,                  /* CH05 */
              (float)g_rc.lost_flag,                         /* CH06 */
              (float)g_rc.sw_st[eRC_SW_A].curr,              /* CH07 */

              (float)g_vofa_speed.m1_offline_total_count,    /* CH08 */
              (float)g_vofa_speed.m1_offline_confirmed,      /* CH09 */

              (float)g_vofa_speed.m2_offline_total_count,    /* CH10 */
              (float)g_vofa_speed.m2_offline_confirmed,      /* CH11 */

              (float)g_rc_filter.ch_ry,                      /* CH12 */
              (float)g_rc_filter.ch_rx,                      /* CH13 */

              (float)g_rc_chassis.fLinearVel,                /* CH14 */
              (float)g_rc_chassis.fAngularVel,               /* CH15 */

              (float)g_vofa_speed.m1_cycle_ms,               /* CH16 */
              (float)g_vofa_speed.m2_cycle_ms,               /* CH17 */
							(float)g_vofa_speed.m1_encoder_position_counts,
							(float)g_vofa_speed.m2_encoder_position_counts,

          };
          bsp_vofa_just_float(vofa_channels, 19);
          s_u32LastVofaTick = now_tick;
      }
  }
}
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
