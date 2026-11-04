/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Xe tu can bang STM32F407 - MPU6050 + TB6612 + 2 encoder
  *
  * VIET LAI THEO KIEN TRUC DA CHUNG MINH CHAY DUOC:
  *   - Vong lap polling HAL_GetTick (khong dung TIM6 flag phuc tap)
  *   - MPU6050_Init + Calibrate don gian
  *   - state machine IDLE/RUN/FAULT ro rang
  *   - an toan: STBY thap khi boot, khong tu chay motor, IMU loi -> ngat
  *
  * Debug qua Live Expressions (g_...).
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "motor.h"
#include "encoder.h"
#include "balance_control.h"
#include <math.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LOOP_MS         4U          /* 4ms = 250Hz */
#define DT_S            0.004f
#define ARM_ANGLE_DEG   10.0f       /* chi ARM khi gan thang dung */
#define ARM_GYRO_DPS    40.0f       /* ...va gan nhu dung yen */
#define ARM_HOLD_MS     1000U       /* giu lien tuc 1s */
#define TILT_CUTOFF_DEG 35.0f       /* nghieng qua -> cat motor */
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim9;

/* USER CODE BEGIN PV */

/* ---- Module dieu khien ---- */
MPU6050_Data_t   g_imu;
Motor_Handle_t   g_motor;
Encoder_Handle_t g_enc_left;    /* TIM3 - encoder trai  */
Encoder_Handle_t g_enc_right;   /* TIM2 - encoder phai  */
Balance_t        g_balance;

/* Trang thai he thong (giong code chay duoc) */
typedef enum { ST_IDLE = 0, ST_RUN = 1, ST_FAULT = 2 } SysState_t;
volatile SysState_t g_state = ST_IDLE;

/* Bien chinh/hieu chinh qua debugger (Live Expressions):
 *   g_arm_request = 1 -> ARM; 0 -> DISARM
 *   g_calib_request = 1 -> calibrate lai (xe dung yen)
 *   g_motor_test = -pwm..+pwm -> test motor truc tiep (banh nhac khoi dat)
 *   g_auto_arm = 1 -> tu ARM khi giu thang dung 1s
 *   g_kp / g_kd -> he so PD                                             */
volatile int32_t g_arm_request   = 0;
volatile int32_t g_calib_request = 0;
volatile int32_t g_motor_test    = 0;
volatile int32_t g_auto_arm      = 0;
volatile float   g_kp            = 0.0f;
volatile float   g_kd            = 0.0f;

uint32_t last_time = 0;
uint32_t arm_ms    = 0;
uint8_t  imu_fail  = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM9_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Trang thai an toan: tat motor + STBY thap */
static void Enter_Idle(void)
{
  g_state = ST_IDLE;
  Balance_Disarm(&g_balance);
  Motor_Disable(&g_motor);
  arm_ms = 0;
}

static void Enter_Fault(void)
{
  g_state = ST_FAULT;
  Balance_Disarm(&g_balance);
  Motor_Disable(&g_motor);
  arm_ms = 0;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */

  /* An toan ngay tu dau */
  HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);

  Motor_Init(&g_motor, &htim9);
  Motor_SetPwmLimit(&g_motor, 3000);   /* gioi han an toan ~36% */

  Encoder_Init(&g_enc_left,  &htim3, false);
  Encoder_Init(&g_enc_right, &htim2, true);
  g_enc_left.invert  = true;    /* encoder trai dem am khi tien (du lieu test) */
  g_enc_right.invert = false;

  Balance_Init(&g_balance);

  /* IMU init + calibrate — xe phai dung yen, thang dung ~2s */
  bool imu_ok = false;
  for (int i = 0; i < 5 && !imu_ok; i++) {
    imu_ok = MPU6050_Init(&hi2c2, &g_imu);
    if (!imu_ok) HAL_Delay(200);
  }
  if (imu_ok) {
    MPU6050_Calibrate(&g_imu, MPU6050_CALIB_SAMPLES, HAL_Delay);
  }

  /* Cho bo loc hoi tu */
  for (int i = 0; i < 100 && imu_ok; i++) {
    MPU6050_Update(&g_imu, DT_S);
    HAL_Delay(4);
  }

  HAL_TIM_Base_Start_IT(&htim6);   /* van bat TIM6 nhung khong dung flag phuc tap */
  last_time = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now = HAL_GetTick();
    if ((now - last_time) < LOOP_MS) {
      continue;
    }
    /* Chong troi chu ky: neu tre qua lau thi dong bo lai */
    if ((now - last_time) > (5U * LOOP_MS)) { last_time = now; }
    else                                    { last_time += LOOP_MS; }

    /* ---------- Xu ly yeu cau debugger ---------- */
    if (g_calib_request && !g_balance.armed) {
      g_calib_request = 0;
      MPU6050_Calibrate(&g_imu, MPU6050_CALIB_SAMPLES, HAL_Delay);
    }

    /* ---------- IMU ---------- */
    if (!MPU6050_Update(&g_imu, DT_S)) {
      Motor_Stop(&g_motor);
      if (++imu_fail >= 3U) { Enter_Fault(); continue; }
    }
    imu_fail = 0U;

    /* ---------- Encoder ---------- */
    Encoder_Update(&g_enc_left,  DT_S);
    Encoder_Update(&g_enc_right, DT_S);

    /* ---------- Test motor truc tiep (bypass, chi khi DISARM) ---------- */
    if (g_motor_test != 0 && !g_balance.armed && !g_auto_arm) {
      if (!g_motor.enabled) Motor_Enable(&g_motor);
      int16_t t = (int16_t)g_motor_test;
      if (t >  (int16_t)g_motor.pwm_max) t =  (int16_t)g_motor.pwm_max;
      if (t < -(int16_t)g_motor.pwm_max) t = -(int16_t)g_motor.pwm_max;
      Motor_SetBoth(&g_motor, t, t);
      continue;
    }
    if (g_motor_test == 0 && !g_balance.armed && g_motor.enabled
        && (g_motor.cmd_left != 0 || g_motor.cmd_right != 0)) {
      Motor_Stop(&g_motor);
    }

    /* ---------- ARM: thu cong (g_arm_request) hoac tu dong (g_auto_arm) ---- */
    bool want_arm = false;
    if (g_auto_arm) {
      /* Tu ARM khi gan thang dung + yen, giu 1s */
      float a = g_imu.angle; if (a < 0) a = -a;
      float r = g_imu.gyro_x_dps; if (r < 0) r = -r;
      if (g_imu.connected && g_imu.valid && g_imu.calibrated
          && a < ARM_ANGLE_DEG && r < ARM_GYRO_DPS) {
        arm_ms += LOOP_MS;
        if (arm_ms >= ARM_HOLD_MS) want_arm = true;
      } else {
        arm_ms = 0;
      }
    } else if (g_arm_request) {
      /* Thu cong: chi ARM khi IMU ok (edge-trigger, khong can hold) */
      static bool arm_latch = false;
      if (!arm_latch && g_imu.connected && g_imu.valid && g_imu.calibrated) {
        want_arm = true;
      }
      arm_latch = true;
    } else {
      arm_ms = 0;
      static bool arm_latch_reset = true;
      (void)arm_latch_reset;
    }

    if (want_arm && !g_balance.armed && g_state != ST_FAULT) {
      Motor_Enable(&g_motor);
      Balance_Arm(&g_balance);
      g_balance.kp = g_kp;     /* cap nhat he so tu debugger */
      g_balance.kd = g_kd;
      g_state = ST_RUN;
    }

    /* ---------- An toan + dieu khien ---------- */
    if (g_state == ST_RUN && g_balance.armed) {
      /* Che do can bang */
      float ang = g_imu.angle;
      if (ang < 0) ang = -ang;
      if (ang > TILT_CUTOFF_DEG) { Enter_Idle(); continue; }

      /* PD: cap nhat kp/kd truc tiep tu bien debug */
      g_balance.kp = g_kp;
      g_balance.kd = g_kd;
      Balance_Update(&g_balance, &g_imu, &g_motor);
    }
    /* state ST_IDLE / ST_FAULT: khong lam gi, motor da tat */
  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function (encoder phai, PA5/PA1)
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 4;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 4;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function (encoder trai, PC6/PC7)
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 4;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 4;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 83;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 3999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM9 Initialization Function (PWM motor, PE5/PE6)
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 0;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 8399;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim9);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Output init LOW */
  HAL_GPIO_WritePin(GPIOE, BIN1_Pin|AIN1_Pin|AIN2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, BIN2_Pin|STBY_Pin, GPIO_PIN_RESET);

  /* AIN1, AIN2, BIN1 = output */
  GPIO_InitStruct.Pin = BIN1_Pin|AIN1_Pin|AIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* BIN2, STBY = output */
  GPIO_InitStruct.Pin = BIN2_Pin|STBY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  *         Tat motor ngay (khong phu thuoc HAL) roi dung.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  TIM9->CCR1 = 0;
  TIM9->CCR2 = 0;
  GPIOC->BSRR = (uint32_t)GPIO_PIN_4 << 16U;   /* STBY = LOW */
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
