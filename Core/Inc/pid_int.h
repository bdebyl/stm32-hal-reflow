/**
 * @file pid_int.h
 * @author Bastian de Byl (bastian@bdebyl.net)
 * @brief Integer-based PID Controller implementation for improved performance
 * @version 1.0
 * @date 2022-03-05
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef __PID_INT_H
#define __PID_INT_H
#include <stdint.h>

// Scaling factors for integer arithmetic
#define PID_TEMP_SCALE     (100)     // Temperature scaling: 0.01°C precision
#define PID_GAIN_SCALE     (1000)    // Gain scaling: 0.001 precision
#define PID_TIME_SCALE     (1000)    // Time scaling: 0.001s precision
#define PID_CALC_SCALE     (1000000) // Intermediate calculation scaling

// Helper macros for scaling conversions
#define TEMP_TO_SCALED(temp)     ((int32_t)((temp) * PID_TEMP_SCALE))
#define SCALED_TO_TEMP(scaled)   ((int16_t)((scaled) / PID_TEMP_SCALE))
#define GAIN_TO_SCALED(gain)     ((uint32_t)((gain) * PID_GAIN_SCALE))
#define TIME_TO_SCALED(time)     ((uint32_t)((time) * PID_TIME_SCALE))

typedef struct {
    // PID gains (scaled by PID_GAIN_SCALE)
    uint32_t Kp_scaled;
    uint32_t Ki_scaled;
    uint32_t Kd_scaled;
    
    // Filter time constant (scaled by PID_TIME_SCALE)
    uint32_t tau_scaled;
    
    // Output limits (unscaled, direct PWM values)
    int16_t LimitMin;
    int16_t LimitMax;
    
    // Sample time (scaled by PID_TIME_SCALE)
    uint32_t T_scaled;
    
    // PID component accumulators (scaled by PID_CALC_SCALE)
    int32_t P_scaled;
    int32_t I_scaled;
    int32_t D_scaled;
    
    // Previous values (scaled by PID_TEMP_SCALE)
    int32_t PrevError_scaled;
    int32_t PrevMeasure_scaled;
} PID_Int;

// Function prototypes
void    PID_Int_Init(PID_Int *pid);
int16_t PID_Int_Update(PID_Int *pid, int16_t setpoint, int16_t measurement);

// Utility functions for parameter setting
void PID_Int_SetGains(PID_Int *pid, float Kp, float Ki, float Kd);
void PID_Int_SetLimits(PID_Int *pid, int16_t min, int16_t max);
void PID_Int_SetSampleTime(PID_Int *pid, float sample_time_seconds);
void PID_Int_SetFilterTimeConstant(PID_Int *pid, float tau_seconds);

#endif // __PID_INT_H