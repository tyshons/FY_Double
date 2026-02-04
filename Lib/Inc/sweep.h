//
// Created by tyshon on 2026/2/4.
//

#ifndef FY_DOUBLE_SWEEP_H
#define FY_DOUBLE_SWEEP_H

extern float sweep_speed_set;
extern float sweep_freq_global;

void sweep_function(uint8_t motor_id, float f_start, float f_end, float sweep_amp, uint8_t n_points);

#endif //FY_DOUBLE_SWEEP_H