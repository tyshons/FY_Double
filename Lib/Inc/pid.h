//
// Created by tyshon on 25-8-12.
//

#ifndef PID_H
#define PID_H

#include <stdint.h>

#define  INCR_LOCT_SELECT  1         /* 1增量 0位置 */

#if INCR_LOCT_SELECT

/* 外环pid调节幅度不要太大 */

/* 位置环 */
#define  L_KP      0.5f
#define  L_KI      0.00042f
#define  L_KD      0.092f

/* 速度环 */
#define  S_KP      145.0f
#define  S_KI      4.0f
#define  S_KD      0.0f
#define  SMAPLSE_PID_SPEED  50

#else

/* 位置环 */
#define  L_KP      0.18f
#define  L_KI      0.00f
#define  L_KD      0.08f

/* 速度环 */
#define  S_KP      20.0f
#define  S_KI      10.00f
#define  S_KD      0.02f
#define  SMAPLSE_PID_SPEED  50

#endif

typedef struct
{
    volatile float  SetPoint;            /* 目标转速 */
    volatile float  ActualValue;         /* 期望转速 */
    volatile float  SumError;            /* 累计误差 */
    volatile float  Proportion;          /*  P */
    volatile float  Integral;            /*  I */
    volatile float  Derivative;          /*  D */
    volatile float  Error;               /* Error[1] */
    volatile float  LastError;           /* Error[-1] */
    volatile float  PrevError;           /* Error[-2] */
    volatile float  wucha;
} PID_TypeDef;

extern PID_TypeDef  g_location_pid;
extern PID_TypeDef  g_speed_pid;

void pid_init(void);
int32_t increment_pid_ctrl(PID_TypeDef *PID,float Feedback_value);

#endif


