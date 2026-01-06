//
// Created by tyshon on 25-8-12.
//

#include "pid.h"
#include "motor.h"
#include "math.h"

PID_TypeDef  g_speed_pid;
PID_TypeDef  g_location_pid;

void pid_init(void)
{
    g_location_pid.SetPoint = 0.0f;       /* 目标 */
    g_location_pid.ActualValue = 0.0f;    /* 期望 */
    g_location_pid.SumError = 0.0f;       /* 累计误差 */
    g_location_pid.Error = 0.0f;          /* Error[1] */
    g_location_pid.LastError = 0.0f;      /* Error[-1] */
    g_location_pid.PrevError = 0.0f;      /* Error[-2] */
    g_location_pid.Proportion = L_KP;    /* P */
    g_location_pid.Integral = L_KI;      /* I */
    g_location_pid.Derivative = L_KD;    /* D */

    g_speed_pid.SetPoint = 0;       /* 目标 */
    g_speed_pid.ActualValue = 0.0f;  /* 期望 */
    g_speed_pid.SumError = 0.0f;     /* 累计误差 */
    g_speed_pid.Error = 0.0f;        /* Error[1] */
    g_speed_pid.LastError = 0.0f;    /* Error[-1] */
    g_speed_pid.PrevError = 0.0f;    /* Error[-2] */
    g_speed_pid.Proportion = S_KP;    /* P */
    g_speed_pid.Integral = S_KI;      /* I */
    g_speed_pid.Derivative = S_KD;    /* D */
}

float normalize_angle_error(float setpoint, float feedback)
{
    float error = setpoint - feedback;
    return remainder(error, 360.0f);  // 返回 [-180, 180) 的误差
}

int32_t increment_pid_ctrl(PID_TypeDef *PID,float Feedback_value)
{
    PID->Error = normalize_angle_error(PID->SetPoint, Feedback_value);                   /* 偏差 */


#if  INCR_LOCT_SELECT                                                       /* 增量式 */

    PID->ActualValue += (PID->Proportion * (PID->Error - PID->LastError))                          /* p */
                        + (PID->Integral * PID->Error)                                             /* i */
                        + (PID->Derivative * (PID->Error - 2 * PID->LastError + PID->PrevError));  /* d */

    PID->PrevError = PID->LastError;
    PID->LastError = PID->Error;

#else                                                                       /* 位置式 */

    PID->SumError += PID->Error;
    PID->ActualValue = (PID->Proportion * PID->Error)                       /* p */
                       + (PID->Integral * PID->SumError)                    /* i */
                       + (PID->Derivative * (PID->Error - PID->LastError)); /* d */
    PID->LastError = PID->Error;

#endif
    return ((int32_t)(PID->ActualValue));
}
