/*
 * STM32F407 self-balancing robot
 * Control loop: 250 Hz (TIM6) -> MPU6050 -> encoder -> angle PD -> TB6612.
 *
 * This file keeps the pin names and peripheral assignments from the project
 * supplied by the user. Keep the matching CubeMX MSP configuration and main.h.
 */
#include "main.h"
#include "mpu6050.h"
#include "motor.h"
#include "encoder.h"
#include "balance_control.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

/* ---------------------- Peripheral handles ---------------------- */
I2C_HandleTypeDef hi2c2;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim9;

/* ---------------------- Control timing -------------------------- */
#define CONTROL_DT_S             0.004f
#define CONTROL_RATE_HZ          250U
#define IMU_STARTUP_ATTEMPTS     10U
#define IMU_STARTUP_CALIB_SAMPLES 100U
#define IMU_WARMUP_SAMPLES       50U
#define IMU_RECONNECT_TICKS      50U   /* retry every 200 ms at 250 Hz */
#define STATUS_LED_BLINK_TICKS   25U   /* toggle every 100 ms at 250 Hz (5 Hz) */

/* ---------------------- Application objects --------------------- */
MPU6050_Data_t g_imu;
Motor_Handle_t g_motor;
Encoder_Handle_t g_enc_left;
Encoder_Handle_t g_enc_right;
Balance_t g_balance;

typedef enum
{
    SYS_IDLE = 0,
    SYS_RUN,
    SYS_FAULT
} System_State_t;

volatile System_State_t g_state = SYS_IDLE;
volatile Balance_Status_t g_fault_code = BAL_FAULT_NOT_ARMED;

/* ---------------------- Live Expressions controls ----------------
 * Robot starts armed automatically on power-on.
 * Only safety cutoff: motors disable when |angle| > g_angle_limit_deg (50 deg).
 */
volatile uint8_t  g_arm_request = 0U;          /* write 1 to re-ARM after manual disarm */
volatile uint8_t  g_disarm_request = 0U;       /* write 1 to DISARM */
volatile uint8_t  g_clear_fault_request = 0U;  /* kept for Live Expressions compatibility */
volatile uint8_t  g_calib_request = 0U;        /* write 1 while robot is still */
volatile uint8_t  g_capture_target_request = 0U; /* write 1 at the balance point */
volatile uint8_t  g_auto_arm_enable = 1U;      /* auto-run on power-on enabled */

/* Starting gains for a small JGA25-370/TB6612 robot.
 * Tune with the wheels raised first, then on the ground:
 *   Kp too low -> falls; too high -> fast oscillation.
 *   Kd too low -> keeps oscillating; too high -> buzzing / jitter. */
volatile float    g_kp = 800.0f;
volatile float    g_kd = 21.0f;
volatile float    g_ki = 0.0f;
volatile float    g_target_angle_deg = 0.0f;

/* g_angle_sign only flips the telemetry/control convention.  The gyro sign
 * is tied to it automatically (rate must be d(angle)/dt).  To fix a robot
 * that drives the wrong way when tilted, change g_output_sign, NOT this.
 *
 * Hardware-verified on this robot: with the motors wired as built, a
 * positive controller command drives the wheels opposite to a forward
 * lean, so the default is -1.  If you rewire/replace a motor or mount the
 * MPU differently, repeat README step 4 and set +1/-1 accordingly. */
volatile float    g_angle_sign = 1.0f;
volatile float    g_output_sign = -1.0f;
volatile int32_t  g_pwm_limit = 7500;
volatile float    g_angle_limit_deg = 50.0f;
volatile float    g_rate_limit_dps = 700.0f;

/* Outer speed loop: fast P damping + separate slow dynamic auto-trim. */
volatile uint8_t  g_speed_loop_enable = 1U;
volatile float    g_speed_kp = 0.005f;   /* deg / (encoder count/s) */
volatile float    g_speed_ki = 0.0f;     /* kept 0: dynamic auto-trim handles offset learning */
volatile float    g_speed_target_cps = 0.0f;
volatile float    g_speed_angle_limit_deg = 8.0f;

/* Dynamic balance-point auto-trim (self-adjusts target angle online). */
volatile uint8_t  g_auto_trim_enable = 1U;
volatile float    g_auto_trim_ki = 0.0018f;      /* deg / (count/s * s) */
volatile float    g_auto_trim_limit_deg = 15.0f; /* max learned offset (+/- deg) */
volatile float    g_dynamic_trim_deg = 0.0f;     /* debug: current learned offset */

/* Encoder direction normalisation (both positive when the robot rolls forward). */
volatile uint8_t  g_encoder_left_invert = 1U;
volatile uint8_t  g_encoder_right_invert = 0U;

/* Motor correction: all zero initially.  min PWM is a floor for gearbox
 * friction; leave 0 until you have measured the real deadband. */
volatile int32_t  g_trim_left = 0;
volatile int32_t  g_trim_right = 0;
volatile int32_t  g_min_pwm_left = 0;
volatile int32_t  g_min_pwm_right = 0;

/* Direct motor test: only with wheels raised and SYS_IDLE.
 * Set g_motor_test = +/-500; the enable is 1 by default for convenience. */
volatile uint8_t  g_motor_test_enable = 1U;
volatile int32_t  g_motor_test = 0;

/* ---------------------- Debug telemetry -------------------------- */
volatile uint8_t  g_imu_read_ok = 0U;
volatile uint32_t g_imu_fail_count = 0U;
volatile uint32_t g_tick_overrun_count = 0U;
volatile uint32_t g_control_tick_count = 0U;
volatile float    g_debug_dt_s = CONTROL_DT_S;
volatile float    g_encoder_left_cps = 0.0f;
volatile float    g_encoder_right_cps = 0.0f;
volatile float    g_encoder_average_cps = 0.0f;
volatile uint16_t g_auto_arm_count = 0U;

/* Timer ISR only posts events. No I2C, PID or motor operations in the ISR. */
volatile uint8_t g_pending_ticks = 0U;

/* ---------------------- Function prototypes ---------------------- */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM9_Init(void);

static int16_t clamp_to_i16(int32_t value);
static uint16_t clamp_to_u16(int32_t value, uint16_t maximum);
static void apply_debug_configuration(void);
static void status_led_set(bool on);
static void update_status_led(bool error_active);
static void enter_idle(void);
static bool try_arm(void);
static void process_motor_test(void);
static void warmup_imu_filter(void);
static bool recalibrate_imu(void);
static void run_one_control_tick(void);
volatile uint32_t g_reset_flags = 0U;

/* ---------------------- Small utilities --------------------------- */
static int16_t clamp_to_i16(int32_t value)
{
    if (value > INT16_MAX) value = INT16_MAX;
    if (value < INT16_MIN) value = INT16_MIN;
    return (int16_t)value;
}

static uint16_t clamp_to_u16(int32_t value, uint16_t maximum)
{
    if (value < 0) value = 0;
    if (value > (int32_t)maximum) value = maximum;
    return (uint16_t)value;
}

static void apply_debug_configuration(void)
{
    float angle_sign = (g_angle_sign >= 0.0f) ? 1.0f : -1.0f;

    g_balance.kp = g_kp;
    g_balance.kd = g_kd;
    g_balance.ki = g_ki;
    g_balance.target_angle_deg = g_target_angle_deg;

    /* Angle and gyro MUST keep the same sign: gyro_x_dps is d(angle)/dt.
       g_output_sign maps a positive controller output to a motor direction.
       Multiplying by angle_sign keeps the physical feedback unchanged when
       the user only flips the telemetry convention. */
    g_balance.angle_sign = angle_sign;
    g_balance.gyro_sign = angle_sign;
    g_balance.output_sign = ((g_output_sign >= 0.0f) ? 1.0f : -1.0f) * angle_sign;

    g_balance.angle_limit_deg = fabsf(g_angle_limit_deg);
    g_balance.rate_limit_dps = fabsf(g_rate_limit_dps);
    g_balance.pwm_limit = clamp_to_i16(g_pwm_limit);

    g_balance.speed_loop_enabled = (g_speed_loop_enable != 0U);
    g_balance.speed_kp = g_speed_kp;
    g_balance.speed_ki = g_speed_ki;
    g_balance.speed_target_cps = g_speed_target_cps;
    g_balance.speed_angle_limit_deg = fabsf(g_speed_angle_limit_deg);

    g_balance.auto_trim_enabled = (g_auto_trim_enable != 0U);
    g_balance.auto_trim_ki = g_auto_trim_ki;
    g_balance.auto_trim_limit_deg = fabsf(g_auto_trim_limit_deg);

    Motor_SetTrim(&g_motor, clamp_to_i16(g_trim_left), clamp_to_i16(g_trim_right));
    Motor_SetMinPwm(&g_motor,
                    clamp_to_u16(g_min_pwm_left, g_motor.pwm_max),
                    clamp_to_u16(g_min_pwm_right, g_motor.pwm_max));

    g_enc_left.invert = (g_encoder_left_invert != 0U);
    g_enc_right.invert = (g_encoder_right_invert != 0U);
}

static void status_led_set(bool on)
{
    /* Active-LOW on STM32F407VET6 Black Board (PA6 / LED D2). */
    HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin,
                      on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void update_status_led(bool error_active)
{
    static uint16_t blink_div = 0U;
    static bool blink_on = false;

    if (error_active)
    {
        if (++blink_div >= STATUS_LED_BLINK_TICKS)
        {
            blink_div = 0U;
            blink_on = !blink_on;
        }
        status_led_set(blink_on);
    }
    else if (g_state == SYS_RUN)
    {
        blink_div = 0U;
        blink_on = false;
        status_led_set(true);
    }
    else
    {
        blink_div = 0U;
        blink_on = false;
        status_led_set(false);
    }
}

static void enter_idle(void)
{
    Balance_Disarm(&g_balance);
    Motor_Disable(&g_motor);
    g_state = SYS_IDLE;
    g_fault_code = BAL_FAULT_NOT_ARMED;
    g_auto_arm_count = 0U;
}

static bool try_arm(void)
{
    apply_debug_configuration();
    Balance_Arm(&g_balance);
    g_state = SYS_RUN;
    g_fault_code = BAL_OK;
    g_auto_arm_count = 0U;
    return true;
}

static void process_motor_test(void)
{
    static bool test_active = false;
    int16_t command;

    if ((g_state == SYS_IDLE) &&
        (g_motor_test_enable != 0U) && (g_motor_test != 0))
    {
        if (!test_active)
        {
            Motor_Enable(&g_motor);
            test_active = true;
        }

        command = clamp_to_i16(g_motor_test);
        Motor_SetBoth(&g_motor, command, command);
    }
    else if (test_active)
    {
        test_active = false;
        /* Do not disable the driver if the controller has just taken over. */
        if ((g_state != SYS_RUN) && !g_balance.armed)
        {
            Motor_Disable(&g_motor);
        }
    }
}

static void warmup_imu_filter(void)
{
    float acc_sum = 0.0f;
    uint16_t acc_count = 0U;

    for (uint16_t i = 0U; i < IMU_WARMUP_SAMPLES; i++)
    {
        if (!MPU6050_Update(&g_imu, CONTROL_DT_S)) break;
        acc_sum += g_imu.angle_acc_raw;
        acc_count++;
        HAL_Delay(4U);
    }

    if (acc_count > 0U)
    {
        g_imu.angle = acc_sum / (float)acc_count;
        g_imu.angle_initialized = true;
    }
}

static bool recalibrate_imu(void)
{
    bool ok;

    if (g_balance.armed) return false;

    Motor_Disable(&g_motor);
    (void)HAL_TIM_Base_Stop_IT(&htim6);

    ok = MPU6050_Calibrate(&g_imu, MPU6050_CALIB_SAMPLES, HAL_Delay);
    if (ok)
    {
        warmup_imu_filter();
        Encoder_Reset(&g_enc_left);
        Encoder_Reset(&g_enc_right);
    }

    __disable_irq();
    g_pending_ticks = 0U;
    __enable_irq();
    (void)HAL_TIM_Base_Start_IT(&htim6);

    return ok && g_imu.valid;
}

static void run_one_control_tick(void)
{
    static uint16_t imu_retry_div = 0U;
    float average_speed;
    Balance_Status_t status;

    g_control_tick_count++;
    g_debug_dt_s = CONTROL_DT_S;

    apply_debug_configuration();

    /* Motor test works when manually disarmed (SYS_IDLE). */
    process_motor_test();

    /* Auto-reconnect IMU if it was not ready at power-on or dropped out. */
    if (!g_imu.connected)
    {
        g_imu_read_ok = 0U;
        if (g_state == SYS_RUN)
        {
            Motor_Disable(&g_motor);
        }

        if (++imu_retry_div >= IMU_RECONNECT_TICKS)
        {
            imu_retry_div = 0U;
            if (MPU6050_Init(&hi2c2, &g_imu))
            {
                g_imu.calibrated = true;
            }
        }
        update_status_led(true);
        return;
    }
    imu_retry_div = 0U;

    /* Read IMU: never latch a fault on transient I2C glitches. */
    if (!MPU6050_Update(&g_imu, CONTROL_DT_S))
    {
        g_imu_read_ok = 0U;
        g_imu_fail_count++;
        update_status_led(true);
        return;
    }

    g_imu_read_ok = 1U;

    /* 2) Update wheel encoders. Their signs must both be positive when rolling forward. */
    Encoder_Update(&g_enc_left, CONTROL_DT_S);
    Encoder_Update(&g_enc_right, CONTROL_DT_S);
    g_encoder_left_cps = g_enc_left.speed_cps;
    g_encoder_right_cps = g_enc_right.speed_cps;
    average_speed = 0.5f * (g_enc_left.speed_cps + g_enc_right.speed_cps);
    g_encoder_average_cps = average_speed;
    Balance_SetWheelSpeed(&g_balance, average_speed);

    /* 3) Handle optional debugger requests. */
    if (g_disarm_request != 0U)
    {
        g_disarm_request = 0U;
        enter_idle();
        update_status_led(false);
        return;
    }

    if (g_clear_fault_request != 0U)
    {
        g_clear_fault_request = 0U;
        Balance_Reset(&g_balance);
        g_balance.dynamic_trim_deg = 0.0f;
        g_dynamic_trim_deg = 0.0f;
    }

    if (g_calib_request != 0U)
    {
        g_calib_request = 0U;
        if (!g_balance.armed)
        {
            (void)recalibrate_imu();
        }
    }

    /* Optional: hold the robot at its real mechanical balance point and write
       g_capture_target_request=1.  That angle becomes the new target, which
       compensates for a slightly tilted MPU or an offset centre of mass. */
    if (g_capture_target_request != 0U)
    {
        g_capture_target_request = 0U;
        g_target_angle_deg = g_imu.angle * g_balance.angle_sign;
        g_balance.target_angle_deg = g_target_angle_deg;
        g_balance.dynamic_trim_deg = 0.0f;
        g_dynamic_trim_deg = 0.0f;
    }

    if (g_arm_request != 0U)
    {
        g_arm_request = 0U;
        (void)try_arm();
    }

    /* 4) Inner balance controller.
       Only safety cutoff: |angle| > 50 deg disables motors and resets integrals;
       when stood back upright (|angle| <= 50 deg), motors re-enable automatically. */
    if ((g_state == SYS_RUN) && g_balance.armed)
    {
        status = Balance_Update(&g_balance, &g_imu, &g_motor, CONTROL_DT_S);
        g_fault_code = status;
        g_dynamic_trim_deg = g_balance.dynamic_trim_deg;
    }

    update_status_led((g_state == SYS_RUN) && (g_fault_code != BAL_OK));
}

/* ---------------------- Main -------------------------------------- */
int main(void)
{
    g_reset_flags = RCC->CSR;
    __HAL_RCC_CLEAR_RESET_FLAGS();

    bool imu_ok = false;

    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_I2C2_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM6_Init();
    MX_TIM9_Init();

    /* Ensure TB6612 is disabled during startup. */
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);

    Motor_Init(&g_motor, &htim9);
    Motor_SetPwmLimit(&g_motor, MOTOR_PWM_ABS_MAX);
    Motor_SetInvert(&g_motor, false, true); /* preserve project wiring */
    Motor_SetTrim(&g_motor, 0, 0);
    Motor_SetMinPwm(&g_motor, 0U, 0U);
    Motor_Disable(&g_motor);

    /* Preserve the original encoder direction convention:
       left encoder counts negative while moving forward; right counts positive. */
    Encoder_Init(&g_enc_left, &htim3, true);
    Encoder_Init(&g_enc_right, &htim2, false);

    Balance_Init(&g_balance);
    apply_debug_configuration();

    /* Brief delay so battery/3.3V power rail stabilizes on power-on. */
    HAL_Delay(100U);

    for (uint8_t attempt = 0U; (attempt < IMU_STARTUP_ATTEMPTS) && !imu_ok; attempt++)
    {
        imu_ok = MPU6050_Init(&hi2c2, &g_imu);
        if (!imu_ok) HAL_Delay(50U);
    }

    if (imu_ok)
    {
        (void)MPU6050_Calibrate(&g_imu, IMU_STARTUP_CALIB_SAMPLES, HAL_Delay);
        warmup_imu_filter();
    }

    /* Auto-start immediately on power-on (only cutoff is |angle| > 50 deg). */
    (void)try_arm();

    /* TIM6 clock = 84 MHz (APB1 timer clock), PSC=83 -> 1 MHz,
       ARR=3999 -> exactly 250 Hz. */
    (void)HAL_TIM_Base_Start_IT(&htim6);

    while (1)
    {
        uint8_t pending;

        __disable_irq();
        pending = g_pending_ticks;
        g_pending_ticks = 0U;
        __enable_irq();

        if (pending == 0U)
        {
            continue;
        }

        if (pending > 1U)
        {
            g_tick_overrun_count += (uint32_t)(pending - 1U);
        }

        /* Do not replay old ticks: one fresh sensor sample per control update. */
        run_one_control_tick();
    }
}

/* ---------------------- Clock and peripheral setup ---------------- */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    osc.PLL.PLLM = 16;
    osc.PLL.PLLN = 336;
    osc.PLL.PLLP = RCC_PLLP_DIV2;
    osc.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();

    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV4;
    clk.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK) Error_Handler();
}

static void MX_I2C2_Init(void)
{
    hi2c2.Instance = I2C2;
    hi2c2.Init.ClockSpeed = 400000U;
    hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c2.Init.OwnAddress1 = 0U;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2 = 0U;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK) Error_Handler();
}

static void MX_TIM2_Init(void)
{
    TIM_Encoder_InitTypeDef encoder = {0};
    TIM_MasterConfigTypeDef master = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0U;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0xFFFFFFFFU;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    encoder.EncoderMode = TIM_ENCODERMODE_TI12;
    encoder.IC1Polarity = TIM_ICPOLARITY_RISING;
    encoder.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    encoder.IC1Prescaler = TIM_ICPSC_DIV1;
    encoder.IC1Filter = 8U;
    encoder.IC2Polarity = TIM_ICPOLARITY_RISING;
    encoder.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    encoder.IC2Prescaler = TIM_ICPSC_DIV1;
    encoder.IC2Filter = 8U;
    if (HAL_TIM_Encoder_Init(&htim2, &encoder) != HAL_OK) Error_Handler();

    master.MasterOutputTrigger = TIM_TRGO_RESET;
    master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &master) != HAL_OK) Error_Handler();
}

static void MX_TIM3_Init(void)
{
    TIM_Encoder_InitTypeDef encoder = {0};
    TIM_MasterConfigTypeDef master = {0};

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0U;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 0xFFFFU;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    encoder.EncoderMode = TIM_ENCODERMODE_TI12;
    encoder.IC1Polarity = TIM_ICPOLARITY_RISING;
    encoder.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    encoder.IC1Prescaler = TIM_ICPSC_DIV1;
    encoder.IC1Filter = 8U;
    encoder.IC2Polarity = TIM_ICPOLARITY_RISING;
    encoder.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    encoder.IC2Prescaler = TIM_ICPSC_DIV1;
    encoder.IC2Filter = 8U;
    if (HAL_TIM_Encoder_Init(&htim3, &encoder) != HAL_OK) Error_Handler();

    master.MasterOutputTrigger = TIM_TRGO_RESET;
    master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &master) != HAL_OK) Error_Handler();
}

static void MX_TIM6_Init(void)
{
    TIM_MasterConfigTypeDef master = {0};

    htim6.Instance = TIM6;
    htim6.Init.Prescaler = 83U;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = 3999U;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK) Error_Handler();

    master.MasterOutputTrigger = TIM_TRGO_RESET;
    master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &master) != HAL_OK) Error_Handler();

    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1U, 0U);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

static void MX_TIM9_Init(void)
{
    TIM_ClockConfigTypeDef clock = {0};
    TIM_OC_InitTypeDef pwm = {0};

    htim9.Instance = TIM9;
    htim9.Init.Prescaler = 0U;
    htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim9.Init.Period = MOTOR_PWM_ABS_MAX;
    htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim9) != HAL_OK) Error_Handler();

    clock.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim9, &clock) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Init(&htim9) != HAL_OK) Error_Handler();

    pwm.OCMode = TIM_OCMODE_PWM1;
    pwm.Pulse = 0U;
    pwm.OCPolarity = TIM_OCPOLARITY_HIGH;
    pwm.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim9, &pwm, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim9, &pwm, TIM_CHANNEL_2) != HAL_OK) Error_Handler();

    HAL_TIM_MspPostInit(&htim9);
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOE, BIN1_Pin | AIN1_Pin | AIN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, BIN2_Pin | STBY_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);

    gpio.Pin = BIN1_Pin | AIN1_Pin | AIN2_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &gpio);

    gpio.Pin = BIN2_Pin | STBY_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &gpio);

    gpio.Pin = STATUS_LED_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(STATUS_LED_GPIO_Port, &gpio);
}

/* ---------------------- Interrupt callbacks ----------------------- */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        if (g_pending_ticks < 255U) g_pending_ticks++;
    }
}

void Error_Handler(void)
{
    __disable_irq();
    TIM9->CCR1 = 0U;
    TIM9->CCR2 = 0U;
    GPIOC->BSRR = (uint32_t)STBY_Pin << 16U;
    while (1)
    {
        STATUS_LED_GPIO_Port->ODR ^= STATUS_LED_Pin;
        for (volatile uint32_t i = 0U; i < 2000000U; i++) { }
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
}
#endif
