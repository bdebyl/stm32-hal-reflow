/**
 * @file pid.c
 * @author Bastian de Byl (bastian@bdebyl.net)
 * @brief PID Control function implementation. Most of the logic has been taken
 * from Phil's Lab YouTube video on PID Control implementation in C.
 * @version 0.1
 * @date 2022-02-19
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "pid.h"

void PID_Init(PID *pid) {
  // Re-set controller struct values in memory
  pid->I           = 0.0f;
  pid->PrevError   = 0.0f;
  pid->D           = 0.0f;
  pid->PrevMeasure = 0.0f;
}

int16_t PID_Update(PID *pid, int16_t setpoint, int16_t measurement) {
  float   _limitMinI, _limitMaxI;
  int16_t out  = 0;
  float   _err = (float)(setpoint - measurement);

  /* P */
  pid->P = pid->Kp * _err;

  /* I */
  // integrator += 0.5 * Ki * Sampletime * (error + previous_error)
  pid->I += 0.5f * pid->Ki * pid->T * (_err + pid->PrevError);
  // Set up anti-wind-up via dynamic integrator clamping
  // Maximum
  if (pid->LimitMax > pid->P) {
    _limitMaxI = pid->LimitMax - pid->P;
  } else {
    _limitMaxI = 0.0f;
  }
  // Minimum
  if (pid->LimitMin < pid->P) {
    _limitMinI = pid->LimitMin - pid->P;
  } else {
    _limitMinI = 0.0f;
  }
  // Clamp integrator values
  if (pid->I > _limitMaxI) {
    pid->I = _limitMaxI;
  } else if (pid->I < _limitMinI) {
    pid->I = _limitMinI;
  }

  /* D */
  pid->D = (2.0f * pid->Kd * ((float)measurement - pid->PrevMeasure)) +
           ((2.0f * pid->tau - pid->T) * pid->D) / (2.0f * pid->tau + pid->T);

  // Output and limit
  out = (int16_t)(pid->P + pid->I + pid->D);

  if (out > pid->LimitMax) {
    out = (int16_t)pid->LimitMax;
  } else if (out < pid->LimitMin) {
    out = (int16_t)pid->LimitMin;
  }

  // Remember error and measurement
  pid->PrevError   = _err;
  pid->PrevMeasure = (float)measurement;

  return out;
}