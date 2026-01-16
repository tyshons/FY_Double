//
// Created by tyshon on 25-8-11.
//

#include "motor.h"
#include "tim.h"

Motor_TypeDef g_motor_data = {0};
uint8_t g_run_flag = 0;

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
} MotorConfig_TypeDef;

static const MotorConfig_TypeDef motor_configs[2] = {
    {&htim1, TIM_CHANNEL_1},
    {&htim8, TIM_CHANNEL_1}
};

void motor_init(void)
{
    motor_stop(0);             // 初始化俯仰电机
    motor_start(0);
    motor_set(0, 0, 0);

    motor_stop(1);             // 初始化水平电机
    motor_start(1);
    motor_set(1, 0, 0);
}

void motor_start(uint8_t motor_id)
{
    if (motor_id == 0) {
        ENABLE_MOTOR_0;
    }
    else if (motor_id == 1) {
        ENABLE_MOTOR_1;
    }
}

void motor_stop(uint8_t motor_id)
{
    if (motor_id == 0) {
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
        DISABLE_MOTOR_0;

    }
    else if (motor_id == 1) {
        HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
        HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_1);
        DISABLE_MOTOR_1;

    }
}


void motor_set(uint8_t motor_id, uint8_t para, uint16_t motor_speed)
{
    if (motor_id >= 2) return;
    const MotorConfig_TypeDef *config = &motor_configs[motor_id];
    if (motor_speed < (__HAL_TIM_GetAutoreload(config->htim) - 0x0F))
    {
        HAL_TIMEx_PWMN_Stop(config->htim, config->channel);
        HAL_TIM_PWM_Stop(config->htim, config->channel);

        __HAL_TIM_SetCompare(config->htim, config->channel, motor_speed);
        if (para == 0)
        {
            HAL_TIM_PWM_Start(config->htim, config->channel);
        }
        else if (para == 1)
        {
            HAL_TIMEx_PWMN_Start(config->htim, config->channel);
        }
    }
}


void motor_pwm_set(uint8_t motor_id, float para)
{
    int speed = (int)para;

    if (motor_id == 0)
    {
        if (speed >= 0)
        {
            motor_set(0, 1, speed);
        }
        else
        {
            motor_set(0, 0, -speed);
        }
    }
    else if (motor_id == 1)
    {
        if (speed >= 0)
        {
            motor_set(1, 0, speed);
        }
        else
        {
            motor_set(1, 1, -speed);
        }
    }
}