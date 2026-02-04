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
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "posUpdate.h"
#include "motor.h"
#include "pid.h"
#include "setpara.h"
#include "sweep.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  float in_prev[3];
  float out_prev[3];
} pid_state_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// 水平轴（motor_id = 1）
float current_angle_sp = 0.0f;
float current_speed_sp = 0.0f;
float given_sp = 180.0f;
float speed_given_sp = 0.0f;
float position_error_sp = 0.0f;

// 俯仰轴（motor_id = 0）
float current_angle_el = 0.0f;
float current_speed_el = 0.0f;
float given_el = 75.0f;
float speed_given_el = 0.0f;
float position_error_el = 0.0f;

//扫频用
float sweep_speed_set;
float sweep_freq_global = 0.5f;
uint8_t ctrl_mode = 1;

//图像用
int16_t pixel_error = 0;
static uint16_t position_counter = 0;
extern uint8_t g_run_flag;

// 控制器参数（双轴可拆分）
float speed_num[4], speed_den[4];
float pos_num[4], pos_den[4];

pid_state_t pos_pid_sp = {0}; // 水平轴位置环状态
pid_state_t pos_pid_el = {0}; // 俯仰轴位置环状态
pid_state_t spd_pid_sp = {0}; // 水平轴速度环状态
pid_state_t spd_pid_el = {0}; // 俯仰轴速度环状态
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
float get_signed_angle_error(float target_angle, float current_angle);
float speed_pid(float speed_error, pid_state_t* state, const float num[4], const float den[4]);
float position_pid(float error, pid_state_t* state, const float num[4], const float den[4]);

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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_SPI2_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  motor_init();
  pid_init();
  set_speedpara(speed_num, speed_den);
  set_pospara(pos_num, pos_den);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief 获取带符号的角度误差
  * @param target_angle 目标角度
  * @param current_angle 当前角度
  * @return 带符号的角度误差（-180° ~ 180°）
  */
float get_signed_angle_error(const float target_angle, const float current_angle)
{
  float error = target_angle - current_angle;
  while (error > 180.0f) error -= 360.0f;
  while (error < -180.0f) error += 360.0f;
  return error;
}

/**
  * @brief 定时器3周期中断回调函数1000Hz
  *
  * 完成对于转台的所有控制逻辑,通过ctrl_mode切换扫频
  *
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3) // 1000Hz
  {
    // 获取当前角度
    current_angle_sp = get_pos_x(&current_speed_sp);
    current_angle_el = get_pos_y(&current_speed_el);

    if (ctrl_mode == 0)
    {
      position_counter++;
      if (position_counter >= 20) // 50Hz
      {
        position_counter = 0;
        if (abs(pixel_error) > 5)
        {
          const float correction = 0.002f * (float)pixel_error;
          given_sp += correction;
        }
      }
      // 水平轴控制
      position_error_sp = get_signed_angle_error(given_sp, current_angle_sp);
      speed_given_sp = position_pid(position_error_sp,&pos_pid_sp, pos_num, pos_den);
      const float pidOut_sp = speed_pid((speed_given_sp - current_speed_sp), &spd_pid_sp, speed_num, speed_den);
      motor_pwm_set(1, pidOut_sp); // motor_id=1: 水平
      // 俯仰轴控制
      position_error_el = get_signed_angle_error(given_el, current_angle_el);
      speed_given_el = position_pid(position_error_el, &pos_pid_el, pos_num, pos_den);
      const float pidOut_el = speed_pid((speed_given_el - current_speed_el), &spd_pid_el, speed_num, speed_den);
      motor_pwm_set(0, pidOut_el); // motor_id=0: 俯仰
    }

    if (ctrl_mode == 1)
    {
      sweep_function(1, sweep_freq_global, 20.0f, 4000,
                    30);
    }
  }
}

/**
  * @brief 位置环PID控制器
  * @param error 输入误差
  * @param state PID状态结构体
  * @param num 传递函数分子系数
  * @param den 传递函数分母系数
  * @return PID输出值
  */
float position_pid(const float error, pid_state_t* state, const float num[4], const float den[4])
{
  const float in = error;
  const float out =
      num[0] * in +
      num[1] * state->in_prev[0] +
      num[2] * state->in_prev[1] +
      num[3] * state->in_prev[2] +
      den[1] * state->out_prev[0] +
      den[2] * state->out_prev[1] +
      den[3] * state->out_prev[2];

  // 更新历史
  state->in_prev[2] = state->in_prev[1];
  state->in_prev[1] = state->in_prev[0];
  state->in_prev[0] = in;

  state->out_prev[2] = state->out_prev[1];
  state->out_prev[1] = state->out_prev[0];
  state->out_prev[0] = out;

  return out;
}

/**
  * @brief 速度环PID控制器
  * @param speed_error 速度误差
  * @param state PID状态结构体
  * @param num 传递函数分子系数
  * @param den 传递函数分母系数
  * @return PID输出值（限幅后）
  */
float speed_pid(const float speed_error, pid_state_t* state, const float num[4], const float den[4])
{
  const float in = speed_error;
  float out =
      num[0] * in +
      num[1] * state->in_prev[0] +
      num[2] * state->in_prev[1] +
      num[3] * state->in_prev[2] +
      den[1] * state->out_prev[0] +
      den[2] * state->out_prev[1] +
      den[3] * state->out_prev[2];

  // 输出限幅（±8400  对应PWM）
  if (out > 8400.0f) out = 8400.0f;
  if (out < -8400.0f) out = -8400.0f;

  // 更新历史
  state->in_prev[2] = state->in_prev[1];
  state->in_prev[1] = state->in_prev[0];
  state->in_prev[0] = in;

  state->out_prev[2] = state->out_prev[1];
  state->out_prev[1] = state->out_prev[0];
  state->out_prev[0] = out;

  return out;
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
