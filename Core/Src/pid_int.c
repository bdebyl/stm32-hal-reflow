/**
 * @file pid_int.c
 * @author Bastian de Byl (bastian@bdebyl.net)
 * @brief Integer-based PID Controller implementation for improved performance
 * @version 1.0
 * @date 2022-03-05
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "pid_int.h"

/**
 * @brief Initialize integer PID controller
 * @param pid Pointer to PID_Int structure
 */
void PID_Int_Init(PID_Int *pid) {
    // Reset all accumulated values
    pid->P_scaled = 0;
    pid->I_scaled = 0;
    pid->D_scaled = 0;
    pid->PrevError_scaled = 0;
    pid->PrevMeasure_scaled = 0;
}

/**
 * @brief Update PID controller with integer arithmetic
 * @param pid Pointer to PID_Int structure
 * @param setpoint Target temperature (unscaled, °C)
 * @param measurement Current temperature (unscaled, °C)
 * @return PID output (0-60 PWM cycles)
 */
int16_t PID_Int_Update(PID_Int *pid, int16_t setpoint, int16_t measurement) {
    int32_t limitMinI_scaled, limitMaxI_scaled;
    int16_t out = 0;
    
    // Convert inputs to scaled values
    int32_t setpoint_scaled = TEMP_TO_SCALED(setpoint);
    int32_t measurement_scaled = TEMP_TO_SCALED(measurement);
    int32_t error_scaled = setpoint_scaled - measurement_scaled;
    
    /* Proportional Term */
    // P = Kp * error
    // Result in PID_CALC_SCALE units
    pid->P_scaled = (pid->Kp_scaled * error_scaled) / PID_TEMP_SCALE;
    
    /* Integral Term */
    // I += 0.5 * Ki * T * (error + prev_error)
    // Careful with intermediate calculations to prevent overflow
    int32_t integral_increment = (pid->Ki_scaled * pid->T_scaled) / (2 * PID_TIME_SCALE);
    integral_increment = (integral_increment * (error_scaled + pid->PrevError_scaled)) / PID_TEMP_SCALE;
    pid->I_scaled += integral_increment;
    
    /* Anti-windup via dynamic integrator clamping */
    // Calculate integrator limits based on current P term
    if ((int32_t)pid->LimitMax * PID_CALC_SCALE > pid->P_scaled) {
        limitMaxI_scaled = (int32_t)pid->LimitMax * PID_CALC_SCALE - pid->P_scaled;
    } else {
        limitMaxI_scaled = 0;
    }
    
    if ((int32_t)pid->LimitMin * PID_CALC_SCALE < pid->P_scaled) {
        limitMinI_scaled = (int32_t)pid->LimitMin * PID_CALC_SCALE - pid->P_scaled;
    } else {
        limitMinI_scaled = 0;
    }
    
    // Clamp integrator
    if (pid->I_scaled > limitMaxI_scaled) {
        pid->I_scaled = limitMaxI_scaled;
    } else if (pid->I_scaled < limitMinI_scaled) {
        pid->I_scaled = limitMinI_scaled;
    }
    
    /* Derivative Term with filtering */
    // D = -(2 * Kd * (measurement - prev_measurement)) + ((2*tau - T) * D) / (2*tau + T)
    
    // First part: -(2 * Kd * (measurement - prev_measurement))
    int32_t meas_diff = measurement_scaled - pid->PrevMeasure_scaled;
    int32_t d_new = -(2 * (int32_t)pid->Kd_scaled * meas_diff) / PID_TEMP_SCALE;
    
    // Second part: filtering with tau
    // ((2*tau - T) * D) / (2*tau + T)
    int32_t tau_2 = 2 * (int32_t)pid->tau_scaled;
    int32_t numerator = (tau_2 - (int32_t)pid->T_scaled) * pid->D_scaled;
    int32_t denominator = tau_2 + (int32_t)pid->T_scaled;
    
    if (denominator != 0) {
        pid->D_scaled = d_new + (numerator / denominator);
    } else {
        pid->D_scaled = d_new;
    }
    
    /* Calculate output */
    int32_t total_output_scaled = pid->P_scaled + pid->I_scaled + pid->D_scaled;
    out = (int16_t)(total_output_scaled / PID_CALC_SCALE);
    
    /* Apply output limits */
    if (out > pid->LimitMax) {
        out = pid->LimitMax;
    } else if (out < pid->LimitMin) {
        out = pid->LimitMin;
    }
    
    /* Store previous values for next iteration */
    pid->PrevError_scaled = error_scaled;
    pid->PrevMeasure_scaled = measurement_scaled;
    
    return out;
}

/**
 * @brief Set PID gains from floating-point values
 * @param pid Pointer to PID_Int structure
 * @param Kp Proportional gain
 * @param Ki Integral gain
 * @param Kd Derivative gain
 */
void PID_Int_SetGains(PID_Int *pid, float Kp, float Ki, float Kd) {
    pid->Kp_scaled = GAIN_TO_SCALED(Kp);
    pid->Ki_scaled = GAIN_TO_SCALED(Ki);
    pid->Kd_scaled = GAIN_TO_SCALED(Kd);
}

/**
 * @brief Set output limits
 * @param pid Pointer to PID_Int structure
 * @param min Minimum output value
 * @param max Maximum output value
 */
void PID_Int_SetLimits(PID_Int *pid, int16_t min, int16_t max) {
    pid->LimitMin = min;
    pid->LimitMax = max;
}

/**
 * @brief Set sample time from floating-point seconds
 * @param pid Pointer to PID_Int structure  
 * @param sample_time_seconds Sample time in seconds
 */
void PID_Int_SetSampleTime(PID_Int *pid, float sample_time_seconds) {
    pid->T_scaled = TIME_TO_SCALED(sample_time_seconds);
}

/**
 * @brief Set derivative filter time constant
 * @param pid Pointer to PID_Int structure
 * @param tau_seconds Filter time constant in seconds
 */
void PID_Int_SetFilterTimeConstant(PID_Int *pid, float tau_seconds) {
    pid->tau_scaled = TIME_TO_SCALED(tau_seconds);
}