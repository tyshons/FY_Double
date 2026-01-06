//
// Created by tyshon on 25-8-11.
//

#ifndef MOTOR_H
#define MOTOR_H
#include "main.h"

#define ENABLE_MOTOR    HAL_GPIO_WritePin(Shutdown2_GPIO_Port, Shutdown2_Pin, GPIO_PIN_SET)
#define DISABLE_MOTOR   HAL_GPIO_WritePin(Shutdown2_GPIO_Port, Shutdown2_Pin, GPIO_PIN_RESET)

typedef struct
{
    uint8_t state;
    float current;
    float volatage;
    float power;
    float speed;
    float location;
    int32_t motor_pwm;
} Motor_TypeDef;

extern Motor_TypeDef  g_motor_data;

void motor_init(void);
void motor_start(uint8_t motor_id);
void motor_stop(uint8_t motor_id);
void motor_set(uint8_t motor_id, uint8_t para, uint16_t motor_speed);
void motor_pwm_set(uint8_t motor_id, float para);

#endif //MOTOR_H
