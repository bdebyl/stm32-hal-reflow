/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "pid.h"
#include "pid_int.h"
#include "menu.h"
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim17;

/* USER CODE BEGIN PV */
static uint8_t     tcSPIData[4]; // 32-bits
static LCD_TypeDef LCD       = {0};
static Menu_TypeDef Menu     = {0};
static PID         reflowPid = {.Kp       = 40.0f,
                                .Ki       = 1.6f,
                                .Kd       = 0.2f,
                                .T        = (float)PID_T,
                                .tau      = 0.2f,
                                .LimitMax = (float)PID_MAX,
                                .LimitMin = (float)PID_MIN};

static PID_Int     reflowPidInt = {0};

// static uint8_t              TestState = 0x00;
static uint16_t ZXCounter         = 0x00;
static uint8_t  OvenControlEnable = 0x00;
static int16_t  OvenTemperature   = 0x00;
static int16_t  OvenPWM           = 0x00;
static int16_t  SetPoint          = 20;

static uint8_t  DoReflow          = 0;
static uint16_t ReflowTime        = 0;
static uint8_t  ReflowIndex       = 0;
static ReflowState_TypeDef ReflowState = REFLOW_STATE_RAMPING;
static uint16_t HoldTimer         = 0;

// http://www.chipquik.com/datasheets/NC191SNL50.pdf
// Enhanced reflow profile with ramp and hold phases
static ReflowProfile_TypeDef ReflowProfile[] = {
    // Preheat ramp: 25°C to 150°C over 90 seconds
    {.StartTemperature = 25, .TargetTemperature = 150, .RampTimeSeconds = 90, .HoldTimeSeconds = 0, .Type = REFLOW_PHASE_RAMP},
    // Preheat soak: Hold at 150°C for 60 seconds
    {.StartTemperature = 150, .TargetTemperature = 150, .RampTimeSeconds = 0, .HoldTimeSeconds = 60, .Type = REFLOW_PHASE_HOLD},
    // Ramp to reflow: 150°C to 217°C over 30 seconds
    {.StartTemperature = 150, .TargetTemperature = 217, .RampTimeSeconds = 30, .HoldTimeSeconds = 0, .Type = REFLOW_PHASE_RAMP},
    // Reflow peak: 217°C to 249°C over 30 seconds, hold for 10 seconds
    {.StartTemperature = 217, .TargetTemperature = 249, .RampTimeSeconds = 30, .HoldTimeSeconds = 10, .Type = REFLOW_PHASE_RAMP},
    // Cooling: Let natural cooling take over
    {.StartTemperature = 249, .TargetTemperature = 25, .RampTimeSeconds = 0, .HoldTimeSeconds = 0, .Type = REFLOW_PHASE_RAMP}
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void        SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM17_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
static void MX_LCD_1_Init(void);
static void MX_PID_Init(void);
static void MX_PID_Int_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
static uint32_t Enc_GetValue(TIM_HandleTypeDef *htim) {
  uint32_t count = (__HAL_TIM_GET_COUNTER(htim) >> 2);
  if (count > 300)
    count = 300;

  return 300 - count;
}
*/
static int16_t CalculateLinearSetPoint(int16_t beginTemp, int16_t endTemp,
                                       uint16_t totalSeconds,
                                       uint16_t currentSecond) {
  if (totalSeconds == 0 || beginTemp == endTemp) {
    return endTemp;  // No ramp needed
  }
  
  // Simple linear interpolation: y = mx + b
  float slope = (float)(endTemp - beginTemp) / (float)totalSeconds;
  float newSetPoint = slope * (float)currentSecond + (float)beginTemp;

  return (int16_t)newSetPoint;
}

static uint8_t IsTemperatureWithinTolerance(int16_t current, int16_t target) {
  int16_t diff = current - target;
  if (diff < 0) diff = -diff;  // Absolute value
  return (diff <= TEMP_TOLERANCE);
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == GPIO_BTN_Pin) {
    //__HAL_TIM_SET_COUNTER(&htim3, 0);
    // TemperatureSetPoint = (float32_t)Enc_GetValue(&htim3);
    /*
    switch (TestState) {
    case 0x00:
      HAL_GPIO_WritePin(GPIO_R1E_GPIO_Port, GPIO_R1E_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIO_R2E_GPIO_Port, GPIO_R2E_Pin, GPIO_PIN_RESET);
      TestState = 0x01;
      break;
    case 0x01:
      HAL_GPIO_WritePin(GPIO_R1E_GPIO_Port, GPIO_R1E_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIO_R2E_GPIO_Port, GPIO_R2E_Pin, GPIO_PIN_SET);
      TestState = 0x02;
      break;
    case 0x02:
      HAL_GPIO_WritePin(GPIO_R1E_GPIO_Port, GPIO_R1E_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIO_R2E_GPIO_Port, GPIO_R2E_Pin, GPIO_PIN_RESET);
      TestState = 0x03;
      break;
    case 0x03:
      HAL_GPIO_WritePin(GPIO_R1E_GPIO_Port, GPIO_R1E_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIO_R2E_GPIO_Port, GPIO_R2E_Pin, GPIO_PIN_SET);
      TestState = 0x00;
      break;
    default:
      break;
    }
    */
    /* Toggle
     if (OvenControlEnable) {
       OvenControlEnable = 0x00;
     } else {
       OvenControlEnable = 0x01;
     }
     */
    if (DoReflow == 0) {
      // Start the reflow
      DoReflow          = 1;
      OvenControlEnable = 1;
      ReflowIndex       = 0;
      ReflowTime        = 0;
      ReflowState       = REFLOW_STATE_RAMPING;
      HoldTimer         = 0;
      HAL_TIM_Base_Start_IT(&htim17);
    }
  }

  // Update GPIO PID from zero-cross detection
  if (GPIO_Pin == GPIO_ZX_DET_Pin) {
    // Get latest thermocouple temperature
    OvenTemperature =
        ((tcSPIData[0] & 0x7F) << 4) | ((tcSPIData[1] >> 4) & 0x0F);

    // Increment zero-cross counter
    ZXCounter++;
    if (ZXCounter == ZX_COUNT_MAX) {

      // Run both PID implementations for comparison
      int16_t ovenPWM_float = PID_Update(&reflowPid, SetPoint, OvenTemperature);
      int16_t ovenPWM_int = PID_Int_Update(&reflowPidInt, SetPoint, OvenTemperature);
      
      // Use integer PID output for actual control 
      OvenPWM = ovenPWM_int;
      
      // For debugging: both outputs are available for comparison
      // ovenPWM_float and ovenPWM_int can be observed during debugging

      // Re-set the Zero-cross counter
      ZXCounter = 0;
    }
    if (OvenControlEnable) {
      if (ZXCounter < OvenPWM) {
        HAL_GPIO_WritePin(GPIO_R1E_GPIO_Port, GPIO_R1E_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIO_R2E_GPIO_Port, GPIO_R2E_Pin, GPIO_PIN_SET);
      } else {
        HAL_GPIO_WritePin(GPIO_R1E_GPIO_Port, GPIO_R1E_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIO_R2E_GPIO_Port, GPIO_R2E_Pin, GPIO_PIN_RESET);
      }

    } else {
      HAL_GPIO_WritePin(GPIO_R1E_GPIO_Port, GPIO_R1E_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIO_R2E_GPIO_Port, GPIO_R2E_Pin, GPIO_PIN_RESET);
    }
  }
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_TIM14_Init();
  MX_TIM3_Init();
  MX_TIM17_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  MX_LCD_1_Init();
  MX_PID_Init();
  MX_PID_Int_Init();
  
  /* Initialize menu system */
  Menu_Init(&Menu, &LCD);
  
  /* Display startup message */
  Menu_HandleState(&Menu);
  
  /* Allow hardware to settle before enabling EXTI interrupts */
  HAL_Delay(500);
  
  /* Clear any pending EXTI interrupts that may have occurred during init */
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_BTN_Pin);
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_ZX_DET_Pin);
  
  /* Enable EXTI interrupts after settling delay */
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
  
  /* Switch to main operating display */
  Menu_SetState(&Menu, MENU_MAIN);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Update menu display system
    Menu_Update(&Menu);
    
    // Update temperature display
    Menu_DisplayTemperature(&Menu, OvenTemperature);

    // Update status display
    char stateChar = 'I';  // Idle
    if (DoReflow) {
      if (ReflowState == REFLOW_STATE_RAMPING) {
        stateChar = 'R';  // Ramping
      } else if (ReflowState == REFLOW_STATE_HOLDING) {
        stateChar = 'H';  // Holding
      }
    }
    Menu_DisplayStatus(&Menu, SetPoint, stateChar, ReflowIndex);
    
    HAL_Delay(200);
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV     = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType =
      RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief NVIC Configuration.
 * @retval None
 */
static void MX_NVIC_Init(void) {
  /* TIM14_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM14_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM14_IRQn);
  /* SPI1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SPI1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(SPI1_IRQn);
  /* EXTI4_15_IRQn interrupt configuration - priority only, enable later */
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 3, 0);
  /* TIM17_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM17_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(TIM17_IRQn);
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance               = SPI1;
  hspi1.Init.Mode              = SPI_MODE_MASTER;
  hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;
  hspi1.Init.NSS               = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial     = 7;
  hspi1.Init.CRCLength         = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */
  /* USER CODE END SPI1_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig       = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance               = TIM3;
  htim3.Init.Prescaler         = 0;
  htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim3.Init.Period            = 300 * 4 + 1;
  htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode          = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity          = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection         = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler         = TIM_ICPSC_DIV1;
  sConfig.IC1Filter            = 10;
  sConfig.IC2Polarity          = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection         = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler         = TIM_ICPSC_DIV1;
  sConfig.IC2Filter            = 10;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  __HAL_TIM_SET_COUNTER(&htim3, htim3.Init.Period - 1);
  if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE END TIM3_Init 2 */
}

/**
 * @brief TIM14 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM14_Init(void) {

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance               = TIM14;
  htim14.Init.Prescaler         = 4800 - 1;
  htim14.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim14.Init.Period            = 100 - 1;
  htim14.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */
  if (HAL_TIM_Base_Start_IT(&htim14) != HAL_OK) {
    Error_Handler();
  }

  /* USER CODE END TIM14_Init 2 */
}

/**
 * @brief TIM17 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM17_Init(void) {

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance               = TIM17;
  htim17.Init.Prescaler         = 999;
  htim17.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim17.Init.Period            = 47999;
  htim17.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC,
                    GPIO_CS0_Pin | GPIO_CS1_Pin | GPIO_R1E_Pin | GPIO_R2E_Pin,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB,
                    GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_LCD_RS_Pin |
                        GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 |
                        GPIO_PIN_7 | GPIO_LCD_E_Pin | GPIO_LCD_RW_Pin,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIO_ZX_EN_GPIO_Port, GPIO_ZX_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : GPIO_CS0_Pin GPIO_CS1_Pin GPIO_R1E_Pin GPIO_R2E_Pin */
  GPIO_InitStruct.Pin =
      GPIO_CS0_Pin | GPIO_CS1_Pin | GPIO_R1E_Pin | GPIO_R2E_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 GPIO_LCD_RS_Pin
                           PB3 PB4 PB5 PB6
                           PB7 GPIO_LCD_E_Pin GPIO_LCD_RW_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_LCD_RS_Pin |
                        GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 |
                        GPIO_PIN_7 | GPIO_LCD_E_Pin | GPIO_LCD_RW_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_BTN_Pin */
  GPIO_InitStruct.Pin  = GPIO_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO_BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_ZX_EN_Pin */
  GPIO_InitStruct.Pin   = GPIO_ZX_EN_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIO_ZX_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_ZX_DET_Pin */
  GPIO_InitStruct.Pin  = GPIO_ZX_DET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO_ZX_DET_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/**
 * @brief LCD 1 Initialization function
 * @param None
 * @retval None
 */
static void MX_LCD_1_Init(void) {
  LCD_InitTypeDef LCD_InitStruct = {0};

  LCD_InitStruct.Columns         = 20;
  LCD_InitStruct.Rows            = 2; // TODO(bastian): Currently unused
  LCD_InitStruct.GPIODataPort    = GPIOB;
  LCD_InitStruct.GPIORWPort      = GPIO_LCD_RW_GPIO_Port;
  LCD_InitStruct.GPIORWPin       = GPIO_LCD_RW_Pin;
  LCD_InitStruct.GPIOEnPort      = GPIO_LCD_E_GPIO_Port;
  LCD_InitStruct.GPIOEnPin       = GPIO_LCD_E_Pin;
  LCD_InitStruct.GPIORSPort      = GPIO_LCD_RS_GPIO_Port;
  LCD_InitStruct.GPIORSPin       = GPIO_LCD_RS_Pin;

  /*   LCD_InitStruct.EntryModeSet    = LCD_INST_EMS_ID | LCD_INST_EMS_S; */
  LCD_InitStruct.FunctionSet =
      LCD_INST_FSET_DL | LCD_INST_FSET_F | LCD_INST_FSET_N;
  LCD_InitStruct.DisplayMode = LCD_INST_DISP_ON;
  /*   LCD_InitStruct.CursorBehavior    = 0x00; */

  LCD_InitStruct.GPIOState        = LCD_GPIO_INVERTED;
  LCD_PositionTypeDef LCD_InitPos = {.Row = LCD_ROW_1, .Column = 0};
  LCD_InitStruct.InitPosition     = LCD_InitPos;

  LCD_Init(&LCD, &LCD_InitStruct);
}

static void MX_PID_Init(void) { PID_Init(&reflowPid); }

static void MX_PID_Int_Init(void) { 
  PID_Int_Init(&reflowPidInt);
  PID_Int_SetGains(&reflowPidInt, 40.0f, 1.6f, 0.2f);
  PID_Int_SetLimits(&reflowPidInt, PID_MIN, PID_MAX);
  PID_Int_SetSampleTime(&reflowPidInt, (float)PID_T);
  PID_Int_SetFilterTimeConstant(&reflowPidInt, 0.2f);
}

void        HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM14) {
    HAL_GPIO_WritePin(GPIO_CS1_GPIO_Port, GPIO_CS1_Pin, GPIO_PIN_RESET);
    HAL_SPI_Receive_IT(&hspi1, tcSPIData, 4);
  } else if (htim->Instance == TIM17) {
    if (DoReflow) {
      // Check if we've completed all profile stages
      if (ReflowIndex >= (sizeof(ReflowProfile) / sizeof(ReflowProfile[0]))) {
        // Stop the reflow
        DoReflow          = 0;
        OvenControlEnable = 0;
        ReflowIndex       = 0;
        ReflowTime        = 0;
        ReflowState       = REFLOW_STATE_RAMPING;
        HoldTimer         = 0;
        SetPoint          = 20;
        HAL_TIM_Base_Stop_IT(&htim17);
        return;
      }

      ReflowProfile_TypeDef currentProfile = ReflowProfile[ReflowIndex];

      switch (currentProfile.Type) {
        case REFLOW_PHASE_RAMP:
          // Handle ramping phase
          if (ReflowState == REFLOW_STATE_RAMPING) {
            if (currentProfile.RampTimeSeconds > 0) {
              // Calculate linear setpoint during ramp
              SetPoint = CalculateLinearSetPoint(
                         currentProfile.StartTemperature, 
                         currentProfile.TargetTemperature,
                         currentProfile.RampTimeSeconds, 
                         ReflowTime);
              
              ReflowTime++;
              
              // Check if ramp time completed or target reached
              if (ReflowTime >= currentProfile.RampTimeSeconds || 
                  IsTemperatureWithinTolerance(OvenTemperature, currentProfile.TargetTemperature)) {
                
                if (currentProfile.HoldTimeSeconds > 0) {
                  // Start hold phase
                  ReflowState = REFLOW_STATE_HOLDING;
                  HoldTimer = 0;
                  SetPoint = currentProfile.TargetTemperature;
                } else {
                  // No hold phase, move to next profile
                  ReflowIndex++;
                  ReflowTime = 0;
                  ReflowState = REFLOW_STATE_RAMPING;
                }
              }
            } else {
              // Instant target (no ramp time)
              SetPoint = currentProfile.TargetTemperature;
              if (currentProfile.HoldTimeSeconds > 0) {
                ReflowState = REFLOW_STATE_HOLDING;
                HoldTimer = 0;
              } else {
                ReflowIndex++;
                ReflowTime = 0;
              }
            }
          } else if (ReflowState == REFLOW_STATE_HOLDING) {
            // Hold at target temperature
            SetPoint = currentProfile.TargetTemperature;
            
            // Only increment hold timer if temperature is within tolerance
            if (IsTemperatureWithinTolerance(OvenTemperature, currentProfile.TargetTemperature)) {
              HoldTimer++;
            }
            
            // Check if hold time completed
            if (HoldTimer >= currentProfile.HoldTimeSeconds) {
              // Move to next profile
              ReflowIndex++;
              ReflowTime = 0;
              ReflowState = REFLOW_STATE_RAMPING;
              HoldTimer = 0;
            }
          }
          break;

        case REFLOW_PHASE_HOLD:
          // Pure hold phase (no ramping)
          SetPoint = currentProfile.TargetTemperature;
          
          // Only increment hold timer if temperature is within tolerance
          if (IsTemperatureWithinTolerance(OvenTemperature, currentProfile.TargetTemperature)) {
            HoldTimer++;
          }
          
          // Check if hold time completed
          if (HoldTimer >= currentProfile.HoldTimeSeconds) {
            // Move to next profile
            ReflowIndex++;
            ReflowTime = 0;
            ReflowState = REFLOW_STATE_RAMPING;
            HoldTimer = 0;
          }
          break;
      }
    }
  }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
  HAL_GPIO_WritePin(GPIO_CS1_GPIO_Port, GPIO_CS1_Pin, GPIO_PIN_SET);
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
