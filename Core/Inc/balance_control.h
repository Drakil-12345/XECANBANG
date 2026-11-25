#ifndef BALANCE_CONTROL_H
#define BALANCE_CONTROL_H

#include "main.h"
#include "mpu6050.h"
#include "motor.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    BAL_OK = 0,
    BAL_FAULT_NOT_ARMED,
    BAL_FAULT_IMU,
    BAL_FAULT_ANGLE,
    BAL_FAULT_RATE,
    BAL_FAULT_TIMING,
    BAL_FAULT_NUMERIC
} Balance_Status_t;

typedef struct
{
    /* Inner loop: angle PD(+optional I). Units: PWM. */
    float kp;                  /* PWM / degree */
    float kd;                  /* PWM / (degree/second) */
    float ki;                  /* PWM / (degree*second), keep 0 initially */
    float integral;
    float integral_limit;

    /* Sign conventions. See README before changing these. */
    float angle_sign;          /* +1: forward lean gives positive angle */
    float gyro_sign;           /* +1: forward angular motion gives positive rate */
    float output_sign;         /* +1: positive output drives wheels forward */

    float target_angle_deg;    /* desired body angle in the chosen convention */
    float angle_limit_deg;     /* safety cutoff */
    float rate_limit_dps;      /* safety cutoff */
    int16_t pwm_limit;         /* controller output limit */

    /* Optional outer wheel-speed loop. */
    bool speed_loop_enabled;
    float speed_kp;            /* degree / (encoder count/second) */
    float speed_ki;
    float speed_integral;
    float speed_integral_limit;
    float speed_target_cps;
    float speed_filtered_cps;
    float speed_filter_alpha;  /* 0..1, higher = less filtering */
    float speed_angle_limit_deg;

    /* Dynamic auto-trim: slowly learns true COM / floor balance angle online. */
    bool auto_trim_enabled;
    float auto_trim_ki;          /* degree / (count/s * second) */
    float auto_trim_limit_deg;   /* max learned offset (+/- deg) */
    float auto_trim_max_rate_dps;/* max adaptation speed (deg/s) */
    float auto_trim_deadband_cps;/* ignore tiny speed below this */
    float speed_slow_cps;        /* heavily filtered speed (~0.5 s tau) */
    float dynamic_trim_deg;      /* learned offset added to target_angle_deg */

    /* Steering from the remote: PWM added to the left wheel and subtracted
       from the right wheel (positive = turn right). Set by main.c. */
    float turn_pwm;

    /* Debug values: visible in Keil Live Expressions. */
    float angle_deg;
    float rate_dps;
    float angle_error_deg;
    float effective_target_deg;
    float p_term;
    float d_term;
    float i_term;
    float output_unclamped;
    float output_pwm;
    uint32_t update_count;
    uint16_t stable_count;
    uint8_t stable;
    Balance_Status_t status;
    bool armed;
} Balance_t;

void Balance_Init(Balance_t *balance);
void Balance_Arm(Balance_t *balance);
void Balance_Disarm(Balance_t *balance);
void Balance_Reset(Balance_t *balance);
void Balance_SetWheelSpeed(Balance_t *balance, float speed_cps);
Balance_Status_t Balance_Update(Balance_t *balance,
                                const MPU6050_Data_t *imu,
                                Motor_Handle_t *motor,
                                float dt_s);

#endif /* BALANCE_CONTROL_H */
