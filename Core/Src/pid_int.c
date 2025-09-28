/**
 * @file pid_int.c
 * @brief Integer-based PID Controller using Q16.16 fixed-point
 * @version 0.2
 * @date 2024
 *
 * Simple, efficient integer PID without floating-point operations
 */

#include "pid_int.h"

void PID_INT_Init(PID_INT *pid) {
  // Reset controller state
  pid->integral   = 0;
  pid->prev_error = 0;
}

int16_t PID_INT_Update(PID_INT *pid, int16_t setpoint, int16_t measurement) {
  // Calculate error
  int32_t error = setpoint - measurement;

  // Proportional term: P = Kp * error
  // Kp is Q16.16, error is unscaled, result needs >> 16
  int64_t p_term = ((int64_t)pid->Kp * error) >> PID_SCALE_BITS;

  // Integral term: I = I + Ki * error * dt (dt = 1)
  // Ki is Q16.16, accumulate in Q16.16
  pid->integral += ((int64_t)pid->Ki * error);

  // Apply integral windup limits
  // Convert limits to Q16.16 for comparison
  int64_t int_max = ((int64_t)pid->out_max << PID_SCALE_BITS);
  int64_t int_min = ((int64_t)pid->out_min << PID_SCALE_BITS);

  if (pid->integral > int_max) {
    pid->integral = int_max;
  } else if (pid->integral < int_min) {
    pid->integral = int_min;
  }

  // Convert integral to output scale
  int64_t i_term = pid->integral >> PID_SCALE_BITS;

  // Derivative term: D = Kd * (error - prev_error) / dt
  // Kd is Q16.16, difference is unscaled
  int64_t d_term =
      ((int64_t)pid->Kd * (error - pid->prev_error)) >> PID_SCALE_BITS;

  // Total output = P + I - D
  // Note: derivative is subtracted (acts as damping on error change)
  int32_t output = (int32_t)(p_term + i_term - d_term);

  // Apply output limits
  if (output > pid->out_max) {
    output = pid->out_max;
  } else if (output < pid->out_min) {
    output = pid->out_min;
  }

  // Store state for next iteration
  pid->prev_error = error;

  return (int16_t)output;
}