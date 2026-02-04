//
// Created by tyshon on 2026/2/4.
//

#include "motor.h"
#include "sweep.h"

#include <math.h>
#include <stdint.h>

//扫频所用变量定义
//static const float f_start        = 0.50f;   // 起始频率 Hz
//static const float f_end          = 20.0f;  // 终止频率 Hz
//static const float sweep_amp      = 3000.0f;  // 电压设定幅值
//static const uint8_t n_points     = 30;     // 频点数量

static float new_freq; // 当前频率
static float old_freq; // 上一次频率
static uint8_t freq_step;
static float sweep_freq;
static float phase; // 相位累加器
static float time_in_freq;

void sweep_function(const uint8_t motor_id, const float f_start, const float f_end, const float sweep_amp,
                    const uint8_t n_points)
{
    static uint8_t sweep_done = 0;
    static uint8_t initialized = 0;
    static const float dt = 0.001f;

    if (!initialized)
    {
        new_freq = f_start;
        old_freq = f_start;
        freq_step = 0;
        sweep_freq = f_start;
        phase = 0.0f;
        time_in_freq = 0.0f;
        initialized = 1;
    }

    if (sweep_done)
    {
        sweep_speed_set = 0.0f;
        motor_pwm_set(motor_id, sweep_speed_set);
        return;
    }

    /** 2. 相位累加，避免时间累积误差 **/
    float dphi = 2.0f * M_PI * sweep_freq * dt;
    phase += dphi;
    if (phase >= 2.0f * M_PI)
    {
        phase -= 2.0f * M_PI;
    }
    /** 3. 生成正弦信号 **/
    sweep_speed_set = sweep_amp * sinf(phase);
    /** 4. 运行 3 个周期 **/
    time_in_freq += dt;
    if (time_in_freq >= (3.0f / sweep_freq))
    {
        time_in_freq = 0.0f;
        freq_step++;
        if (freq_step < n_points)
        {
            // 对数递增计算新频率
            old_freq = sweep_freq;
            float ratio = powf(f_end / f_start, 1.0f / (n_points - 1));
            sweep_freq = f_start * powf(ratio, freq_step);
            new_freq = sweep_freq;
            sweep_freq_global = sweep_freq;
            // 每次换频率时重置相位，保证波峰对齐
            float old_phase = phase;
            phase = fmodf(old_phase + 2.0f * M_PI * (new_freq - old_freq) * dt, 2.0f * M_PI);
        }
        else
        {
            sweep_done = 1;
            sweep_speed_set = 0.0f;
        }
    }
    /** 6. 输出 PWM **/
    motor_pwm_set(motor_id, sweep_speed_set);
}
