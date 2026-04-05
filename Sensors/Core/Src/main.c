/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
#include "main.h"

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

/* USER CODE BEGIN PV */
volatile uint16_t adc_buf[2] = {0};  /* [0]=Y (IN5/PB14), [1]=Z (IN11/PB12) */
volatile uint16_t adc_x      = 0;    /* X (ADC2_IN15/PB15) */
volatile float    accel_x    = 0.0f;
volatile float    accel_y    = 0.0f;
volatile float    accel_z    = 0.0f;
/* USER CODE END PV */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);

/* USER CODE BEGIN 0 */

#define ACCEL_X_ZERO_G    17899
#define ACCEL_X_SENS      3971.0f
#define ACCEL_Y_ZERO_G    17899
#define ACCEL_Y_SENS      3971.0f
#define ACCEL_Z_ZERO_G    17899
#define ACCEL_Z_SENS      3971.0f

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        accel_y = ((float)adc_buf[0] - ACCEL_Y_ZERO_G) / ACCEL_Y_SENS;
        accel_z = ((float)adc_buf[1] - ACCEL_Z_ZERO_G) / ACCEL_Z_SENS;
    }
    else if (hadc->Instance == ADC2)
    {
        accel_x = ((float)adc_x - ACCEL_X_ZERO_G) / ACCEL_X_SENS;
    }
}

/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  SystemCoreClockUpdate();
  HAL_InitTick(TICK_INT_PRIORITY);
  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();

  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, 2);
  HAL_ADC_Start_DMA(&hadc2, (uint32_t*)&adc_x, 1);
  /* USER CODE END 2 */

  while (1)
  {
    HAL_GPIO_TogglePin(GPIOA, Axis2A_Pin);
    HAL_Delay(500);
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM            = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN            = 24;
  RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ            = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR            = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                   | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) Error_Handler();
}

static void MX_ADC1_Init(void)
{
  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance                               = ADC1;
  hadc1.Init.ClockPrescaler                    = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution                        = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign                         = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation                  = 0;
  hadc1.Init.ScanConvMode                      = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection                      = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait                  = DISABLE;
  hadc1.Init.ContinuousConvMode                = ENABLE;
  hadc1.Init.NbrOfConversion                   = 2;
  hadc1.Init.DiscontinuousConvMode             = DISABLE;
  hadc1.Init.ExternalTrigConv                  = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge              = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests             = ENABLE;
  hadc1.Init.Overrun                           = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode                  = ENABLE;
  hadc1.Init.Oversampling.Ratio                = ADC_OVERSAMPLING_RATIO_256;
  hadc1.Init.Oversampling.RightBitShift        = ADC_RIGHTBITSHIFT_4;
  hadc1.Init.Oversampling.TriggeredMode        = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) Error_Handler();

  /* Rank 1 — ADC1_IN5 = Y-axis (PB14, pin 28) */
  sConfig.Channel      = ADC_CHANNEL_5;
  sConfig.Rank         = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff   = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset       = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();

  /* Rank 2 — ADC1_IN11 = Z-axis (PB12, pin 26) */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank    = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
}

static void MX_ADC2_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc2.Instance                               = ADC2;
  hadc2.Init.ClockPrescaler                    = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution                        = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign                         = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation                  = 0;
  hadc2.Init.ScanConvMode                      = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection                      = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait                  = DISABLE;
  hadc2.Init.ContinuousConvMode                = ENABLE;
  hadc2.Init.NbrOfConversion                   = 1;
  hadc2.Init.DiscontinuousConvMode             = DISABLE;
  hadc2.Init.ExternalTrigConv                  = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge              = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests             = ENABLE;
  hadc2.Init.Overrun                           = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode                  = ENABLE;
  hadc2.Init.Oversampling.Ratio                = ADC_OVERSAMPLING_RATIO_256;
  hadc2.Init.Oversampling.RightBitShift        = ADC_RIGHTBITSHIFT_4;
  hadc2.Init.Oversampling.TriggeredMode        = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc2.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK) Error_Handler();

  /* ADC2_IN15 = X-axis (PB15, pin 29) */
  sConfig.Channel      = ADC_CHANNEL_15;
  sConfig.Rank         = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff   = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset       = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) Error_Handler();
}

static void MX_DMA_Init(void)
{
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|Axis2B_Pin
                          |Axis2A_Pin|Axis1B_Pin|Axis1A_Pin|LED_Blink_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin   = GPIO_PIN_1|GPIO_PIN_2|Axis2B_Pin
                         |Axis2A_Pin|Axis1B_Pin|Axis1A_Pin|LED_Blink_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
