/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  WHEELGUARD_STATE_IDLE = 0,
  WHEELGUARD_STATE_CHECK_BATTERY,
  WHEELGUARD_STATE_RUNNING,
  WHEELGUARD_STATE_SUCCESS_BLINK,
  WHEELGUARD_STATE_FAULT
} WheelGuardState;

typedef enum
{
  WHEELGUARD_FAULT_NONE = 0,
  WHEELGUARD_FAULT_LOW_BATTERY,
  WHEELGUARD_FAULT_OVERLOAD,
  WHEELGUARD_FAULT_STALL
} WheelGuardFault;

typedef struct
{
  uint8_t raw_state;
  uint8_t stable_state;
  uint32_t last_edge_tick;
} WheelGuardButton;

typedef struct
{
  HAL_StatusTypeDef status;
  int32_t bus_voltage_mv;
  int32_t current_deci_ma;
  int32_t avg_bus_voltage_mv;
  int32_t avg_current_deci_ma;
  int32_t voltage_samples[10];
  int32_t current_samples[30];
  uint32_t voltage_index;
  uint32_t current_index;
  uint32_t voltage_count;
  uint32_t current_count;
  int64_t voltage_sum;
  int64_t current_sum;
} WheelGuardSensorData;

typedef struct
{
  WheelGuardState state;
  WheelGuardFault fault;
  uint32_t run_start_tick;
  uint32_t fault_start_tick;
  uint32_t overload_start_tick;
  uint32_t stall_start_tick;
  uint32_t led_toggle_tick;
  uint8_t success_toggles_remaining;
  uint8_t battery_status_valid;
  uint8_t battery_low_active;
} WheelGuardApp;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WHEELGUARD_BATTERY_NOMINAL_MV         7600L
#define WHEELGUARD_BATTERY_LOW_MV             6500L
#define WHEELGUARD_OVERLOAD_DECIMA            3500L
#define WHEELGUARD_STALL_DECIMA               2000L
#define WHEELGUARD_OVERLOAD_TIME_MS           500U
#define WHEELGUARD_STALL_TIME_MS              1000U
#define WHEELGUARD_RUN_TIME_MS                10000U
#define WHEELGUARD_MOTOR1_PWM_VALUE           500U
#define WHEELGUARD_MOTOR2_PWM_VALUE           900U
#define WHEELGUARD_PWM_REFERENCE_PERIOD       999U
#define WHEELGUARD_LOW_BATTERY_BLINK_MS       150U
#define WHEELGUARD_LOW_BATTERY_FAULT_MS       2000U
#define WHEELGUARD_OVERLOAD_BLINK_MS          500U
#define WHEELGUARD_STALL_BLINK_MS             250U
#define WHEELGUARD_SUCCESS_BLINK_MS           200U
#define WHEELGUARD_SUCCESS_FLASHES            3U
#define WHEELGUARD_BUTTON_DEBOUNCE_MS         50U
#define WHEELGUARD_STATUS_PERIOD_MS           500U
#define WHEELGUARD_VOLTAGE_AVG_SAMPLES        10U
#define WHEELGUARD_CURRENT_AVG_SAMPLES        30U
#define WHEELGUARD_TIMER_INACTIVE             0xFFFFFFFFU

#define WHEELGUARD_BUTTON_PORT                Push_Button_GPIO_Port
#define WHEELGUARD_BUTTON_PIN                 Push_Button_Pin
#define WHEELGUARD_RED_LED_PORT               RED_LED_GPIO_Port
#define WHEELGUARD_RED_LED_PIN                RED_LED_Pin
#define WHEELGUARD_GREEN_LED_PORT             GREEN_LED_GPIO_Port
#define WHEELGUARD_GREEN_LED_PIN              GREEN_LED_Pin
#define WHEELGUARD_BLUE_LED_PORT              BLUE_LED_GPIO_Port
#define WHEELGUARD_BLUE_LED_PIN               BLUE_LED_Pin

#define WHEELGUARD_MOTOR1_TIMER               htim3
#define WHEELGUARD_MOTOR1_CHANNEL             TIM_CHANNEL_1
#define WHEELGUARD_MOTOR2_TIMER               htim2
#define WHEELGUARD_MOTOR2_CHANNEL             TIM_CHANNEL_1

#define INA219_I2C_ADDR                       (0x40U << 1)
#define INA219_REG_CONFIG                     0x00U
#define INA219_REG_BUS_VOLTAGE                0x02U
#define INA219_REG_CURRENT                    0x04U
#define INA219_REG_CALIBRATION                0x05U
#define INA219_CONFIG_VALUE                   0x399FU
#define INA219_CALIBRATION_VALUE              4096U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static WheelGuardApp g_app = {0};
static WheelGuardButton g_button = {0};
static WheelGuardSensorData g_sensor = {0};
static uint32_t g_last_status_tick = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static uint16_t WheelGuard_ScalePwmCompare(uint16_t legacy_compare, uint32_t timer_period);
static void set_motor_pwm(uint16_t motor1_pwm, uint16_t motor2_pwm);
static void stop_motors(void);
static void set_red_led(GPIO_PinState state);
static void set_green_led(GPIO_PinState state);
static void set_blue_led(GPIO_PinState state);
static void set_fault_indicator(WheelGuardFault fault);
static HAL_StatusTypeDef INA219_Init(void);
static HAL_StatusTypeDef INA219_Read(int32_t *bus_voltage_mv, int32_t *current_deci_ma);
static HAL_StatusTypeDef INA219_WriteRegister(uint8_t reg, uint16_t value);
static HAL_StatusTypeDef INA219_ReadRegister(uint8_t reg, uint16_t *value);
static int32_t update_moving_average(int32_t sample, int32_t *buffer, uint32_t window_size, uint32_t *index,
                                     uint32_t *count, int64_t *sum);
static uint8_t WheelGuard_UpdateButton(WheelGuardButton *button);
static void WheelGuard_UpdateSensors(WheelGuardSensorData *sensor);
static void WheelGuard_Init(WheelGuardApp *app);
static void WheelGuard_Process(WheelGuardApp *app, const WheelGuardSensorData *sensor, uint8_t button_pressed_event);
static void WheelGuard_EnterIdle(WheelGuardApp *app);
static void WheelGuard_EnterBatteryCheck(WheelGuardApp *app);
static void WheelGuard_StartRun(WheelGuardApp *app);
static void WheelGuard_HandleRun(WheelGuardApp *app, const WheelGuardSensorData *sensor);
static void WheelGuard_StartSuccessBlink(WheelGuardApp *app);
static void WheelGuard_HandleSuccessBlink(WheelGuardApp *app);
static void WheelGuard_EnterFault(WheelGuardApp *app, WheelGuardFault fault);
static void WheelGuard_HandleFault(WheelGuardApp *app, uint8_t button_pressed_event);
static void WheelGuard_ResetCurrentDetection(WheelGuardApp *app);
static void WheelGuard_RefreshBatteryHealthLeds(const WheelGuardApp *app);
static uint32_t WheelGuard_GetFaultBlinkPeriod(WheelGuardFault fault);
static const char *WheelGuard_GetStateName(WheelGuardState state);
static const char *WheelGuard_GetFaultName(WheelGuardFault fault);
static void print_status(const WheelGuardApp *app, const WheelGuardSensorData *sensor, uint8_t button_on);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint16_t WheelGuard_ScalePwmCompare(uint16_t legacy_compare, uint32_t timer_period)
{
  uint32_t scaled_compare;

  scaled_compare = ((uint32_t)legacy_compare * (timer_period + 1U)) / (WHEELGUARD_PWM_REFERENCE_PERIOD + 1U);
  if (scaled_compare > timer_period)
  {
    scaled_compare = timer_period;
  }

  return (uint16_t)scaled_compare;
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

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  g_sensor.status = HAL_ERROR;

  if (HAL_TIM_PWM_Start(&WHEELGUARD_MOTOR2_TIMER, WHEELGUARD_MOTOR2_CHANNEL) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Start(&WHEELGUARD_MOTOR1_TIMER, WHEELGUARD_MOTOR1_CHANNEL) != HAL_OK)
  {
    Error_Handler();
  }

  stop_motors();
  set_red_led(GPIO_PIN_RESET);
  set_green_led(GPIO_PIN_RESET);
  set_blue_led(GPIO_PIN_RESET);
  WheelGuard_Init(&g_app);

  if (INA219_Init() != HAL_OK)
  {
    print_status(&g_app, &g_sensor, 0U);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint8_t button_pressed_event = WheelGuard_UpdateButton(&g_button);

    WheelGuard_UpdateSensors(&g_sensor);
    WheelGuard_Process(&g_app, &g_sensor, button_pressed_event);

    if ((HAL_GetTick() - g_last_status_tick) >= WHEELGUARD_STATUS_PERIOD_MS)
    {
      g_last_status_tick = HAL_GetTick();
      print_status(&g_app, &g_sensor, g_button.stable_state);
    }
  }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 800;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 15;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 800;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RED_LED_Pin|GREEN_LED_Pin|BLUE_LED_Pin|SAFE_OUT2_Pin
                          |SAFE_OUT1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RED_LED_Pin GREEN_LED_Pin BLUE_LED_Pin SAFE_OUT2_Pin
                           SAFE_OUT1_Pin */
  GPIO_InitStruct.Pin = RED_LED_Pin|GREEN_LED_Pin|BLUE_LED_Pin|SAFE_OUT2_Pin
                          |SAFE_OUT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Push_Button_Pin */
  GPIO_InitStruct.Pin = Push_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Push_Button_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* Re-apply the intended active-high pulldown button config so the input
     cannot float high if the generated GPIO settings drift. */
  GPIO_InitStruct.Pin = Push_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(Push_Button_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void set_motor_pwm(uint16_t motor1_pwm, uint16_t motor2_pwm)
{
  uint16_t motor1_compare;
  uint16_t motor2_compare;

  motor1_compare = WheelGuard_ScalePwmCompare(motor1_pwm, __HAL_TIM_GET_AUTORELOAD(&WHEELGUARD_MOTOR1_TIMER));
  motor2_compare = WheelGuard_ScalePwmCompare(motor2_pwm, __HAL_TIM_GET_AUTORELOAD(&WHEELGUARD_MOTOR2_TIMER));

  __HAL_TIM_SET_COMPARE(&WHEELGUARD_MOTOR1_TIMER, WHEELGUARD_MOTOR1_CHANNEL, motor1_compare);
  __HAL_TIM_SET_COMPARE(&WHEELGUARD_MOTOR2_TIMER, WHEELGUARD_MOTOR2_CHANNEL, motor2_compare);
}

static void stop_motors(void)
{
  set_motor_pwm(0U, 0U);
}

static void set_red_led(GPIO_PinState state)
{
  HAL_GPIO_WritePin(WHEELGUARD_RED_LED_PORT, WHEELGUARD_RED_LED_PIN, state);
}

static void set_green_led(GPIO_PinState state)
{
  HAL_GPIO_WritePin(WHEELGUARD_GREEN_LED_PORT, WHEELGUARD_GREEN_LED_PIN, state);
}

static void set_blue_led(GPIO_PinState state)
{
  HAL_GPIO_WritePin(WHEELGUARD_BLUE_LED_PORT, WHEELGUARD_BLUE_LED_PIN, state);
}

static void set_fault_indicator(WheelGuardFault fault)
{
  if (fault == WHEELGUARD_FAULT_LOW_BATTERY)
  {
    set_red_led(GPIO_PIN_SET);
  }
  else
  {
    set_red_led(GPIO_PIN_SET);
  }
}

static HAL_StatusTypeDef INA219_Init(void)
{
  if (HAL_I2C_IsDeviceReady(&hi2c1, INA219_I2C_ADDR, 2, 100) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (INA219_WriteRegister(INA219_REG_CONFIG, INA219_CONFIG_VALUE) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (INA219_WriteRegister(INA219_REG_CALIBRATION, INA219_CALIBRATION_VALUE) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

static HAL_StatusTypeDef INA219_Read(int32_t *bus_voltage_mv, int32_t *current_deci_ma)
{
  uint16_t bus_voltage_raw = 0;
  uint16_t current_raw = 0;

  if (INA219_ReadRegister(INA219_REG_BUS_VOLTAGE, &bus_voltage_raw) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (INA219_ReadRegister(INA219_REG_CURRENT, &current_raw) != HAL_OK)
  {
    return HAL_ERROR;
  }

  *bus_voltage_mv = (int32_t)(bus_voltage_raw >> 3) * 4;
  *current_deci_ma = (int32_t)((int16_t)current_raw);
  return HAL_OK;
}

static HAL_StatusTypeDef INA219_WriteRegister(uint8_t reg, uint16_t value)
{
  uint8_t data[2];

  data[0] = (uint8_t)(value >> 8);
  data[1] = (uint8_t)(value & 0xFFU);
  return HAL_I2C_Mem_Write(&hi2c1, INA219_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, sizeof(data), 100);
}

static HAL_StatusTypeDef INA219_ReadRegister(uint8_t reg, uint16_t *value)
{
  uint8_t data[2];

  if (HAL_I2C_Mem_Read(&hi2c1, INA219_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, sizeof(data), 100) != HAL_OK)
  {
    return HAL_ERROR;
  }

  *value = ((uint16_t)data[0] << 8) | data[1];
  return HAL_OK;
}

static int32_t update_moving_average(int32_t sample, int32_t *buffer, uint32_t window_size, uint32_t *index,
                                     uint32_t *count, int64_t *sum)
{
  if (*count < window_size)
  {
    (*count)++;
  }
  else
  {
    *sum -= buffer[*index];
  }

  buffer[*index] = sample;
  *sum += sample;
  *index = (*index + 1U) % window_size;

  return (int32_t)(*sum / (int64_t)(*count));
}

static uint8_t WheelGuard_UpdateButton(WheelGuardButton *button)
{
  uint8_t raw_state = (HAL_GPIO_ReadPin(WHEELGUARD_BUTTON_PORT, WHEELGUARD_BUTTON_PIN) == GPIO_PIN_SET);
  uint32_t now = HAL_GetTick();

  if (raw_state != button->raw_state)
  {
    button->raw_state = raw_state;
    button->last_edge_tick = now;
  }

  if (((now - button->last_edge_tick) >= WHEELGUARD_BUTTON_DEBOUNCE_MS) &&
      (raw_state != button->stable_state))
  {
    button->stable_state = raw_state;
    if (button->stable_state != 0U)
    {
      return 1U;
    }
  }

  return 0U;
}

static void WheelGuard_UpdateSensors(WheelGuardSensorData *sensor)
{
  sensor->status = INA219_Read(&sensor->bus_voltage_mv, &sensor->current_deci_ma);

  if (sensor->status == HAL_OK)
  {
    sensor->avg_bus_voltage_mv = update_moving_average(sensor->bus_voltage_mv, sensor->voltage_samples,
                                                       WHEELGUARD_VOLTAGE_AVG_SAMPLES, &sensor->voltage_index,
                                                       &sensor->voltage_count, &sensor->voltage_sum);
    sensor->avg_current_deci_ma = update_moving_average(sensor->current_deci_ma, sensor->current_samples,
                                                        WHEELGUARD_CURRENT_AVG_SAMPLES, &sensor->current_index,
                                                        &sensor->current_count, &sensor->current_sum);
  }
  else
  {
    (void)INA219_Init();
  }
}

static void WheelGuard_Init(WheelGuardApp *app)
{
  WheelGuard_EnterIdle(app);
}

static void WheelGuard_Process(WheelGuardApp *app, const WheelGuardSensorData *sensor, uint8_t button_pressed_event)
{
  if (sensor->status == HAL_OK)
  {
    app->battery_status_valid = 1U;
    app->battery_low_active = (uint8_t)(sensor->avg_bus_voltage_mv < WHEELGUARD_BATTERY_LOW_MV);
  }

  WheelGuard_RefreshBatteryHealthLeds(app);

  switch (app->state)
  {
    case WHEELGUARD_STATE_IDLE:
      stop_motors();
      set_blue_led(GPIO_PIN_RESET);
      if (button_pressed_event != 0U)
      {
        if (app->battery_low_active != 0U)
        {
          WheelGuard_EnterFault(app, WHEELGUARD_FAULT_LOW_BATTERY);
        }
        else
        {
          WheelGuard_EnterBatteryCheck(app);
        }
      }
      break;

    case WHEELGUARD_STATE_CHECK_BATTERY:
      stop_motors();
      set_blue_led(GPIO_PIN_RESET);
      if (sensor->status == HAL_OK)
      {
        if (sensor->avg_bus_voltage_mv < WHEELGUARD_BATTERY_LOW_MV)
        {
          WheelGuard_EnterFault(app, WHEELGUARD_FAULT_LOW_BATTERY);
        }
        else
        {
          WheelGuard_StartRun(app);
        }
      }
      break;

    case WHEELGUARD_STATE_RUNNING:
      WheelGuard_HandleRun(app, sensor);
      break;

    case WHEELGUARD_STATE_SUCCESS_BLINK:
      WheelGuard_HandleSuccessBlink(app);
      break;

    case WHEELGUARD_STATE_FAULT:
      WheelGuard_HandleFault(app, button_pressed_event);
      break;

    default:
      WheelGuard_EnterIdle(app);
      break;
  }
}

static void WheelGuard_EnterIdle(WheelGuardApp *app)
{
  app->state = WHEELGUARD_STATE_IDLE;
  app->fault = WHEELGUARD_FAULT_NONE;
  app->run_start_tick = 0U;
  app->fault_start_tick = 0U;
  app->led_toggle_tick = HAL_GetTick();
  app->success_toggles_remaining = 0U;
  WheelGuard_ResetCurrentDetection(app);
  stop_motors();
  WheelGuard_RefreshBatteryHealthLeds(app);
  set_blue_led(GPIO_PIN_RESET);
}

static void WheelGuard_EnterBatteryCheck(WheelGuardApp *app)
{
  app->state = WHEELGUARD_STATE_CHECK_BATTERY;
  app->fault = WHEELGUARD_FAULT_NONE;
  app->fault_start_tick = 0U;
  app->led_toggle_tick = HAL_GetTick();
  WheelGuard_ResetCurrentDetection(app);
  stop_motors();
  WheelGuard_RefreshBatteryHealthLeds(app);
  set_blue_led(GPIO_PIN_RESET);
}

static void WheelGuard_StartRun(WheelGuardApp *app)
{
  app->state = WHEELGUARD_STATE_RUNNING;
  app->fault = WHEELGUARD_FAULT_NONE;
  app->run_start_tick = HAL_GetTick();
  app->fault_start_tick = 0U;
  WheelGuard_ResetCurrentDetection(app);
  WheelGuard_RefreshBatteryHealthLeds(app);
  set_blue_led(GPIO_PIN_SET);
  set_motor_pwm(WHEELGUARD_MOTOR1_PWM_VALUE, WHEELGUARD_MOTOR2_PWM_VALUE);
}

static void WheelGuard_HandleRun(WheelGuardApp *app, const WheelGuardSensorData *sensor)
{
  uint32_t now = HAL_GetTick();

  set_blue_led(GPIO_PIN_SET);
  set_motor_pwm(WHEELGUARD_MOTOR1_PWM_VALUE, WHEELGUARD_MOTOR2_PWM_VALUE);

  if (sensor->status == HAL_OK)
  {
    if (sensor->avg_current_deci_ma >= WHEELGUARD_OVERLOAD_DECIMA)
    {
      if (app->overload_start_tick == WHEELGUARD_TIMER_INACTIVE)
      {
        app->overload_start_tick = now;
      }
    }
    else
    {
      app->overload_start_tick = WHEELGUARD_TIMER_INACTIVE;
    }

    if (sensor->avg_current_deci_ma >= WHEELGUARD_STALL_DECIMA)
    {
      if (app->stall_start_tick == WHEELGUARD_TIMER_INACTIVE)
      {
        app->stall_start_tick = now;
      }
    }
    else
    {
      app->stall_start_tick = WHEELGUARD_TIMER_INACTIVE;
    }

    if ((app->overload_start_tick != WHEELGUARD_TIMER_INACTIVE) &&
        ((now - app->overload_start_tick) >= WHEELGUARD_OVERLOAD_TIME_MS))
    {
      WheelGuard_EnterFault(app, WHEELGUARD_FAULT_OVERLOAD);
      return;
    }

    if ((app->stall_start_tick != WHEELGUARD_TIMER_INACTIVE) &&
        ((now - app->stall_start_tick) >= WHEELGUARD_STALL_TIME_MS))
    {
      WheelGuard_EnterFault(app, WHEELGUARD_FAULT_STALL);
      return;
    }
  }
  else
  {
    WheelGuard_ResetCurrentDetection(app);
  }

  if ((now - app->run_start_tick) >= WHEELGUARD_RUN_TIME_MS)
  {
    WheelGuard_StartSuccessBlink(app);
  }
}

static void WheelGuard_StartSuccessBlink(WheelGuardApp *app)
{
  app->state = WHEELGUARD_STATE_SUCCESS_BLINK;
  app->fault = WHEELGUARD_FAULT_NONE;
  app->fault_start_tick = 0U;
  app->led_toggle_tick = HAL_GetTick();
  app->success_toggles_remaining = (uint8_t)(WHEELGUARD_SUCCESS_FLASHES * 2U);
  WheelGuard_ResetCurrentDetection(app);
  stop_motors();
  WheelGuard_RefreshBatteryHealthLeds(app);
  set_blue_led(GPIO_PIN_RESET);
}

static void WheelGuard_HandleSuccessBlink(WheelGuardApp *app)
{
  if ((HAL_GetTick() - app->led_toggle_tick) >= WHEELGUARD_SUCCESS_BLINK_MS)
  {
    app->led_toggle_tick = HAL_GetTick();
    HAL_GPIO_TogglePin(WHEELGUARD_BLUE_LED_PORT, WHEELGUARD_BLUE_LED_PIN);

    if (app->success_toggles_remaining > 0U)
    {
      app->success_toggles_remaining--;
    }

    if (app->success_toggles_remaining == 0U)
    {
      set_blue_led(GPIO_PIN_RESET);
      WheelGuard_EnterIdle(app);
    }
  }
}

static void WheelGuard_EnterFault(WheelGuardApp *app, WheelGuardFault fault)
{
  app->state = WHEELGUARD_STATE_FAULT;
  app->fault = fault;
  app->fault_start_tick = HAL_GetTick();
  app->led_toggle_tick = HAL_GetTick();
  app->success_toggles_remaining = 0U;
  WheelGuard_ResetCurrentDetection(app);
  stop_motors();
  WheelGuard_RefreshBatteryHealthLeds(app);
  set_blue_led(GPIO_PIN_RESET);
  set_fault_indicator(fault);
}

static void WheelGuard_HandleFault(WheelGuardApp *app, uint8_t button_pressed_event)
{
  uint32_t now = HAL_GetTick();

  stop_motors();
  set_blue_led(GPIO_PIN_RESET);

  if (app->fault == WHEELGUARD_FAULT_LOW_BATTERY)
  {
    if ((now - app->fault_start_tick) >= WHEELGUARD_LOW_BATTERY_FAULT_MS)
    {
      WheelGuard_EnterIdle(app);
      return;
    }

    if ((now - app->led_toggle_tick) >= WHEELGUARD_LOW_BATTERY_BLINK_MS)
    {
      app->led_toggle_tick = now;
      HAL_GPIO_TogglePin(WHEELGUARD_RED_LED_PORT, WHEELGUARD_RED_LED_PIN);
    }
    return;
  }

  if (button_pressed_event != 0U)
  {
    WheelGuard_EnterBatteryCheck(app);
    return;
  }

  if ((now - app->led_toggle_tick) >= WheelGuard_GetFaultBlinkPeriod(app->fault))
  {
    app->led_toggle_tick = now;
    HAL_GPIO_TogglePin(WHEELGUARD_RED_LED_PORT, WHEELGUARD_RED_LED_PIN);
  }
}

static void WheelGuard_ResetCurrentDetection(WheelGuardApp *app)
{
  app->overload_start_tick = WHEELGUARD_TIMER_INACTIVE;
  app->stall_start_tick = WHEELGUARD_TIMER_INACTIVE;
}

static void WheelGuard_RefreshBatteryHealthLeds(const WheelGuardApp *app)
{
  GPIO_PinState green_state = GPIO_PIN_RESET;

  if ((app->battery_status_valid != 0U) && (app->battery_low_active == 0U))
  {
    green_state = GPIO_PIN_SET;
  }

  set_green_led(green_state);

  if ((app->state == WHEELGUARD_STATE_FAULT) && (app->fault == WHEELGUARD_FAULT_LOW_BATTERY))
  {
    return;
  }

  if ((app->state == WHEELGUARD_STATE_IDLE) &&
      (app->battery_status_valid != 0U) &&
      (app->battery_low_active != 0U))
  {
    set_red_led(GPIO_PIN_SET);
  }
  else if (app->state != WHEELGUARD_STATE_FAULT)
  {
    set_red_led(GPIO_PIN_RESET);
  }
}

static uint32_t WheelGuard_GetFaultBlinkPeriod(WheelGuardFault fault)
{
  if (fault == WHEELGUARD_FAULT_LOW_BATTERY)
  {
    return WHEELGUARD_LOW_BATTERY_BLINK_MS;
  }

  if (fault == WHEELGUARD_FAULT_STALL)
  {
    return WHEELGUARD_STALL_BLINK_MS;
  }

  return WHEELGUARD_OVERLOAD_BLINK_MS;
}

static const char *WheelGuard_GetStateName(WheelGuardState state)
{
  switch (state)
  {
    case WHEELGUARD_STATE_IDLE:
      return "IDLE";
    case WHEELGUARD_STATE_CHECK_BATTERY:
      return "CHECK_BATTERY";
    case WHEELGUARD_STATE_RUNNING:
      return "RUNNING";
    case WHEELGUARD_STATE_SUCCESS_BLINK:
      return "SUCCESS_BLINK";
    case WHEELGUARD_STATE_FAULT:
      return "FAULT";
    default:
      return "UNKNOWN";
  }
}

static const char *WheelGuard_GetFaultName(WheelGuardFault fault)
{
  switch (fault)
  {
    case WHEELGUARD_FAULT_NONE:
      return "NONE";
    case WHEELGUARD_FAULT_LOW_BATTERY:
      return "LOW_BATTERY";
    case WHEELGUARD_FAULT_OVERLOAD:
      return "OVERLOAD";
    case WHEELGUARD_FAULT_STALL:
      return "STALL";
    default:
      return "UNKNOWN";
  }
}

static void print_status(const WheelGuardApp *app, const WheelGuardSensorData *sensor, uint8_t button_on)
{
  char message[160];
  const char *current_sign = "";
  int32_t current_deci_ma_abs = sensor->avg_current_deci_ma;
  int length;

  if (current_deci_ma_abs < 0)
  {
    current_sign = "-";
    current_deci_ma_abs = -current_deci_ma_abs;
  }

  if (sensor->status == HAL_OK)
  {
    length = snprintf(message, sizeof(message),
                      "State:%s Fault:%s Button:%s Voltage:%ld.%03ld/%ld.%03ldV Current:%s%ld.%01ldmA\r\n",
                      WheelGuard_GetStateName(app->state),
                      WheelGuard_GetFaultName(app->fault),
                      button_on ? "PRESSED" : "RELEASED",
                      (long)(sensor->avg_bus_voltage_mv / 1000),
                      (long)(sensor->avg_bus_voltage_mv % 1000),
                      (long)(WHEELGUARD_BATTERY_NOMINAL_MV / 1000),
                      (long)(WHEELGUARD_BATTERY_NOMINAL_MV % 1000),
                      current_sign,
                      (long)(current_deci_ma_abs / 10),
                      (long)(current_deci_ma_abs % 10));
  }
  else
  {
    length = snprintf(message, sizeof(message),
                      "State:%s Fault:%s Button:%s INA219 read error\r\n",
                      WheelGuard_GetStateName(app->state),
                      WheelGuard_GetFaultName(app->fault),
                      button_on ? "PRESSED" : "RELEASED");
  }

  if (length > 0)
  {
    uint16_t tx_size = (uint16_t)((length < (int)sizeof(message)) ? length : (int)(sizeof(message) - 1U));
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)message, tx_size, 100);
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
