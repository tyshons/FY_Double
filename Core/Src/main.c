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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h> // for sinf

#include "Reverse.h"
#include "motor.h"
#include "pid.h"
#include "setpara.h"
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
#define DATA_SEQUENCE_SIZE 6
#define POSITION_UPDATE_INTERVAL_S 0.001f
#define CONTROL_PERIOD 0.001f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t txBuffer_x[DATA_SEQUENCE_SIZE] = {0x07, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t txBuffer_y[DATA_SEQUENCE_SIZE] = {0x07, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t rxBuffer_x[DATA_SEQUENCE_SIZE] = {0};
uint8_t rxBuffer_y[DATA_SEQUENCE_SIZE] = {0};
volatile uint8_t spi_xfer_done_x = 1;
volatile uint8_t spi_xfer_done_y = 1;
volatile uint8_t sck_edge_count_x = 0;
volatile uint8_t sck_edge_count_y = 0;
volatile uint8_t data_ready_x = 0;
volatile uint8_t data_ready_y = 0;

volatile float latest_angle_sp = 0.0f; // 水平轴最新有效角度
volatile float latest_angle_el = 0.0f; // 俯仰轴最新有效角度
volatile uint8_t angle_valid_sp = 0;   // 角度有效
volatile uint8_t angle_valid_el = 0;

// 水平轴（motor_id = 1）
float last_angle_sp = 0.0f;
float current_angle_sp = 0.0f;
float h_speed_sp = 0.0f;
float given_sp = 125.0f;
float speed_given_sp = 0.0f;
float position_error_sp = 0.0f;

// 俯仰轴（motor_id = 0）=====
float last_angle_el = 0.0f;
float current_angle_el = 0.0f;
float h_speed_el = 0.0f;
float given_el = 0.0f;
float speed_given_el = 0.0f;
float position_error_el = 0.0f;

int16_t pixel_error = 0;
static uint16_t position_counter = 0;
extern uint8_t g_run_flag;

// 控制器参数（两个轴共用同一组，也可拆开）
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
float Updatespeed(float current_angle, float* last_angle, uint32_t* last_time);
float get_signed_angle_error(float target_angle, float current_angle);
float speed_pid(float speed_error, pid_state_t* state, const float num[4], const float den[4]);
float position_pid(float error, pid_state_t* state, const float num[4], const float den[4]);
void UART1_DATA_PRO(void);
float get_pos_pitch(void); // 新增：获取俯仰角度
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
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  motor_init();
  pid_init();
  set_speedpara(speed_num, speed_den);
  set_pospara(pos_num, pos_den);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Angle_Update_Task();
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


float Updatespeed(float current_angle, float* last_angle, uint32_t* last_time)
{
  uint32_t current_time_ms = HAL_GetTick();
  if (*last_time == 0) {
    *last_time = current_time_ms;
    *last_angle = current_angle;
    return 0.0f;
  }

  float time = (current_time_ms - *last_time) / 1000.0f;
  if (time < 0.001f) time = 0.001f;

  float delta_angle = current_angle - *last_angle;
  if (delta_angle > 180.0f) delta_angle -= 360.0f;
  else if (delta_angle < -180.0f) delta_angle += 360.0f;

  const float rpm = delta_angle / (time*6.0f); // deg/s

  *last_time = current_time_ms;
  *last_angle = current_angle;
  return rpm;
}

float get_signed_angle_error(float target, float current)
{
  float error = target - current;
  while (error > 180.0f) error -= 360.0f;
  while (error < -180.0f) error += 360.0f;
  return error;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI1)
  {
    spi_xfer_done_x = 1;
    data_ready_x = 1;
  }
  if (hspi->Instance == SPI2) {
    spi_xfer_done_y = 1;
    data_ready_y = 1;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  static uint32_t last_time_sp = 0, last_time_el = 0;

  if (htim->Instance == TIM3) // 1000Hz
  {
    position_counter++;
    if (position_counter >= 20) // 50Hz
    {
      UART1_DATA_PRO();
      position_counter = 0;

      if (abs(pixel_error) > 5)
      {
        float correction = 0.002f * pixel_error;
        given_sp += correction;
      }
    }

        // 获取当前角度
    if (angle_valid_sp) {
      current_angle_sp = latest_angle_sp;
    }

    if (angle_valid_el) {
      current_angle_el = latest_angle_el;
    }

        // 速度估计（deg/s）
    h_speed_sp = Updatespeed(current_angle_sp, &last_angle_sp, &last_time_sp);
    h_speed_el = Updatespeed(current_angle_el, &last_angle_el, &last_time_el);

        // 水平轴控制
    position_error_sp = get_signed_angle_error(given_sp, current_angle_sp);
    speed_given_sp = position_pid(position_error_sp,&pos_pid_sp, pos_num, pos_den);
    float pidout_sp = speed_pid(speed_given_sp - h_speed_sp, &spd_pid_sp, speed_num, speed_den);
    motor_pwm_set(1, pidout_sp); // motor_id=1: 水平

        // 俯仰轴控制
    position_error_el = get_signed_angle_error(given_el, current_angle_el);
    speed_given_el = position_pid(position_error_el, &pos_pid_el, pos_num, pos_den);
    float pidout_el = speed_pid(speed_given_el - h_speed_el, &spd_pid_el, speed_num, speed_den);
    motor_pwm_set(0, pidout_el); // motor_id=0: 俯仰

        // 打印水平轴状态
    float wgeiding_sp = (current_angle_sp > 180) ? current_angle_sp - 360.0f : current_angle_sp;
    printf("%.3f,%.3f,%.3f\r\n", wgeiding_sp, given_sp - wgeiding_sp, given_sp);
  }
}

float position_pid(float error, pid_state_t* state, const float num[4], const float den[4])
{
  float in = error;
  float out =
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

float speed_pid(float speed_error, pid_state_t* state, const float num[4], const float den[4])
{
  float in = speed_error;
  float out =
      num[0] * in +
      num[1] * state->in_prev[0] +
      num[2] * state->in_prev[1] +
      num[3] * state->in_prev[2] +
      den[1] * state->out_prev[0] +
      den[2] * state->out_prev[1] +
      den[3] * state->out_prev[2];

  // 输出限幅（±8400 对应 PWM）
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

void UART1_DATA_PRO(void)
{
  uint16_t rx_status = uart_get_rx_status();
  if ((rx_status & 0x8000) == 0)
  {
    return;
  }

  uint8_t len_temp = rx_status & 0x3fff;
  uart_clear_rx_status();

  if (len_temp == 5 &&
      g_usart_rx_buf[0] == 0xA5 &&
      g_usart_rx_buf[1] == 0x5A)
  {
    uint8_t biaozhi2;
    memcpy(&biaozhi2, &g_usart_rx_buf[2], sizeof(uint8_t));
    memcpy(&pixel_error, &g_usart_rx_buf[3], sizeof(int16_t));
    if (biaozhi2 == 0)
    {
      pixel_error = -pixel_error;
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
