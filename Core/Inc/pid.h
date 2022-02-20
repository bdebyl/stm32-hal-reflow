/**
 * @file pid.h
 * @author Bastian de Byl (bastian@bdebyl.net)
 * @brief PID Control header. Most of the logic has been taken from Phil's Lab
 * YouTube video on PID Control implementation in C.
 * @version 0.1
 * @date 2022-02-19
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef __PID_H
#define __PID_H
#include <stdint.h>

typedef struct {
  float Kp;
  float Ki;
  float Kd;

  float tau;

  float LimitMin;
  float LimitMax;

  float T; // Sample time in seconds

  float P;
  float I;
  float D;
  float PrevError;
  float PrevMeasure;
} PID;

void    PID_Init(PID *pid);
int16_t PID_Update(PID *pid, int16_t setpoint, int16_t measurement);
#endif // __PID_H