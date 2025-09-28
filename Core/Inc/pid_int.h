/**
 * @file pid_int.h
 * @brief Integer-based PID Controller using Q16.16 fixed-point
 * @version 0.2
 * @date 2024
 */

#ifndef __PID_INT_H
#define __PID_INT_H

#include <stdint.h>

// Q16.16 fixed-point format (16 bits integer, 16 bits fraction)
#define PID_SCALE_BITS 16
#define PID_SCALE      (1L << PID_SCALE_BITS) // 65536

typedef struct {
  // Gains in Q16.16 format
  int32_t Kp; // Proportional gain
  int32_t Ki; // Integral gain
  int32_t Kd; // Derivative gain

  // State variables
  int64_t integral;   // Accumulated integral (Q16.16)
  int32_t prev_error; // Previous error (unscaled)

  // Output limits (unscaled)
  int32_t out_min;
  int32_t out_max;
} PID_INT;

// Initialize PID controller
void PID_INT_Init(PID_INT *pid);

// Update PID controller
int16_t PID_INT_Update(PID_INT *pid, int16_t setpoint, int16_t measurement);

#endif // __PID_INT_H