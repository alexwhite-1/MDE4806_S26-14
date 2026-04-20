/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body — Kalman filter + quadrature output
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "kalman.h"
#include "quadrature_output.h"
#include "stm32g4xx_it.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CLOCK_SPEED 138000000.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */
/* --- Accelerometer (ADC, DMA) --- */
static volatile uint16_t adc_buf[2] = {0};  /* [0]=Y (CH5/PB14), [1]=Z (CH11/PB12) */
static volatile uint16_t adc_x      = 0;    /* X  (CH15/PB15, ADC2) */
static volatile float    accel_x    = 0.0f;
static volatile float    accel_y    = 0.0f;
static volatile float    accel_z    = 0.0f;

/* Calibration: zero-g offset (16-bit counts) and sensitivity (counts/g)
 * Measured on hardware at 138 MHz, ADC_CLOCK_ASYNC_DIV4, 256x oversample >>4 */
#define ACCEL_X_ZERO_G  17286
#define ACCEL_X_SENS    3887.0f
#define ACCEL_Y_ZERO_G  17899
#define ACCEL_Y_SENS    3971.0f
#define ACCEL_Z_ZERO_G  17899
#define ACCEL_Z_SENS    3971.0f

/* --- Gyroscope (SPI, ICM-42688 or compatible) --- */
#define IMU_REG_PWR_MGMT_1   0x6B
#define IMU_REG_USER_CTRL    0x6A

#define IMU_REG_WHO_AM_I     0x75
#define IMU_WHO_AM_I_VAL     0xB5

#define IMU_REG_GYRO_XOUT_H  0x43

#define IMU_CS_PORT          GPIOA
#define IMU_CS_PIN           SPI_CS_Pin
#define IMU_CS_LOW()         HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_RESET)
#define IMU_CS_HIGH()        HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_SET)

static volatile uint8_t who_am_i = 0;

static volatile float   gyro_x   = 0.0f;
static volatile float   gyro_y   = 0.0f;
static volatile float   gyro_z   = 0.0f;

static uint8_t gyro_dma_buf[6];
static uint8_t gyro_addr = IMU_REG_GYRO_XOUT_H | 0x80;
static volatile bool gyro_dma_ready = false;

static volatile float log_dt_s;
static volatile float log_roll_s;
static volatile float log_pitch_s;
static volatile float log_gyro_x_s;
static volatile float log_gyro_y_s;
static volatile float log_gyro_z_s;
static volatile float log_accel_x_s;
static volatile float log_accel_y_s;
static volatile float log_accel_z_s;
static volatile int   log_axis1_a_s;
static volatile int   log_axis1_b_s;

/* --- I2C (to Arduino) --- */
#define ARDUINO_SLAVE_ADDR 0x48

typedef struct __attribute__((packed)) {
    uint8_t header;      // always 0xAA
    uint8_t counter;     // increments every packet
    int16_t pitch_x100;  // pitch in degrees * 100
    int16_t roll_x100;   // roll in degrees * 100
} imu_packet_t;

static uint8_t packet_counter = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_I2C3_Init(void);
/* USER CODE BEGIN PFP */
static void     DWT_Init(void);
static float    ComputeKalmanDT(void);
static uint8_t  IMU_ReadReg(uint8_t reg);
static void     IMU_WriteReg(uint8_t reg, uint8_t data);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* --- ADC conversion complete callback (called from DMA ISR) --- */
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

/* --- SPI gyroscope callback --- */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi1)
    {
        IMU_CS_HIGH();

        int16_t raw_x = (int16_t)((gyro_dma_buf[0] << 8) | gyro_dma_buf[1]);
        int16_t raw_y = (int16_t)((gyro_dma_buf[2] << 8) | gyro_dma_buf[3]);
        int16_t raw_z = (int16_t)((gyro_dma_buf[4] << 8) | gyro_dma_buf[5]);

        gyro_x = (float)raw_x / 131.0f * (M_PI / 180.0f);
        gyro_y = (float)raw_y / 131.0f * (M_PI / 180.0f);
        gyro_z = (float)raw_z / 131.0f * (M_PI / 180.0f);

        gyro_dma_ready = true;

        IMU_CS_LOW();
        HAL_SPI_Transmit(&hspi1, &gyro_addr, 1, HAL_MAX_DELAY);
        HAL_SPI_Receive_DMA(&hspi1, gyro_dma_buf, 6);
    }
}

/* --- SPI / IMU helpers --- */
static void IMU_WriteReg(uint8_t reg, uint8_t data)
{
    uint8_t tx[2] = { reg & 0x7F, data };
    IMU_CS_LOW();
    HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);
    IMU_CS_HIGH();
}

static uint8_t IMU_ReadReg(uint8_t reg)
{
    uint8_t tx[2] = { reg | 0x80, 0x00 };
    uint8_t rx[2] = { 0x00, 0x00 };
    IMU_CS_LOW();
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, HAL_MAX_DELAY);
    IMU_CS_HIGH();
    return rx[1];
}

/* --- DWT cycle counter for precise dt --- */
static void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

static float ComputeKalmanDT(void)
{
    static uint32_t last_tick  = 0;
    static bool     initialized = false;
    uint32_t now = DWT->CYCCNT;
    if (!initialized) { last_tick = now; initialized = true; return 0.0f; }
    float dt = (float)(now - last_tick) / CLOCK_SPEED;
    last_tick = now;
    return dt;
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
  DWT_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_ADC2_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */

  /* Start timers (interrupt mode) */
  HAL_TIM_Base_Start_IT(&htim6);   /* 640  Hz — Kalman flag */
  HAL_TIM_Base_Start_IT(&htim7);   /* 6900 Hz — Quad flag   */

  /* Calibrate and start ADC DMA */
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, 2);
  HAL_ADC_Start_DMA(&hadc2, (uint32_t*)&adc_x,  1);

  /* IMU init */
  IMU_CS_HIGH();
  HAL_Delay(10);
  who_am_i = IMU_ReadReg(IMU_REG_WHO_AM_I);
  if (who_am_i == IMU_WHO_AM_I_VAL) {
      IMU_WriteReg(IMU_REG_PWR_MGMT_1, 0x01);   /* clock select: PLL */
      HAL_Delay(50);
      IMU_WriteReg(IMU_REG_USER_CTRL, 0x10);    /* disable I2C */

      IMU_CS_LOW(); // Start callback
      HAL_SPI_Transmit(&hspi1, &gyro_addr, 1, HAL_MAX_DELAY);
      HAL_SPI_Receive_DMA(&hspi1, gyro_dma_buf, 6);
  }

  /* Kalman filter structures */
  StateVector            state_vector            = StateVector_Construct();
  ErrorCovarianceMatrix  error_covariance_matrix = ErrorCovarianceMatrix_Construct();
  const ProcessNoiseMatrix     process_noise_matrix     = ProcessNoiseMatrix_Construct();
  const MeasurementNoiseMatrix measurement_noise_matrix = MeasurementNoiseMatrix_Construct();

  /* Quadrature output — 4096 CPR, 1 axis (pitch) */
  QuadratureOutput quad_pkg = QuadratureOutput_Construct(4096, 1);
  QuadratureOutput_Initialize(&quad_pkg, 0.0, 0.0);

  HAL_GPIO_TogglePin(GPIOA, LED_Blink_Pin);   /* signal init complete */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* ---- Kalman filter @ 640 Hz ---- */
    if (GetKalmanReady()) {
		/* Read gyroscope (already in rad/s, updated by DMA callback) */
//		GyroSample gyro_sample;
//		if (gyro_dma_ready) {
//			gyro_dma_ready = false;
//
//			gyro_sample.gx = gyro_x;
//			gyro_sample.gy = gyro_y;
//			gyro_sample.gz = gyro_z;
//		}
		GyroSample gyro_sample;
		gyro_sample.gx = gyro_x;
		gyro_sample.gy = gyro_y;
		gyro_sample.gz = gyro_z;

		/* Read accelerometer (already in g, updated by DMA callback) */
		AccelSample accel_sample;
		accel_sample.ax = accel_x;
		accel_sample.ay = accel_y;
		accel_sample.az = accel_z;

		float dt = ComputeKalmanDT();

		if (dt > 0.0f) {
			/* Run kalman filter */
			kalman_run(dt, &state_vector, &error_covariance_matrix, &gyro_sample, &accel_sample, &process_noise_matrix, &measurement_noise_matrix);

			/* Convert to deg */
			float pitch_deg = state_vector.vector[PITCH] * (180.0f / (float)M_PI);
			float roll_deg  = state_vector.vector[ROLL]  * (180.0f / (float)M_PI);

			/* Build packet */
			imu_packet_t packet;
			packet.header     = 0xAA;
			packet.counter    = packet_counter++;
			packet.pitch_x100 = (int16_t)(pitch_deg * 100.0f);
			packet.roll_x100  = (int16_t)(roll_deg  * 100.0f);

			/* Send to Arduino */
			HAL_I2C_Master_Transmit(&hi2c3, (ARDUINO_SLAVE_ADDR << 1), (uint8_t *)&packet, sizeof(packet), 10);

	      	log_dt_s = dt;

	      	log_roll_s = state_vector.vector[ROLL] * (180.0 / M_PI);
	      	log_pitch_s  = state_vector.vector[ROLL] * (180.0 / M_PI);

	      	log_gyro_x_s = gyro_sample.gx;
	      	log_gyro_y_s = gyro_sample.gy;
	      	log_gyro_z_s = gyro_sample.gz;

	      	log_accel_x_s = accel_sample.ax;
	      	log_accel_y_s = accel_sample.ay;
	      	log_accel_z_s = accel_sample.az;

	      	log_axis1_a_s = quad_pkg.axis1.channel_a;
	      	log_axis1_b_s = quad_pkg.axis1.channel_b;
		}
    }

    /* ---- Quadrature output @ ~6900 Hz ---- */
    if (GetQuadReady()) {
        /* Convert pitch (rad) to degrees for quadrature output */
        double pitch_deg = (double)state_vector.vector[PITCH] * (180.0 / M_PI);
        QuadratureOutput_Update(&quad_pkg, pitch_deg, 0.0);

        /* Drive GPIO pins from quadrature state */
        HAL_GPIO_WritePin(GPIOA, Axis1A_Pin, quad_pkg.axis1.channel_a ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, Axis1B_Pin, quad_pkg.axis1.channel_b ? GPIO_PIN_SET : GPIO_PIN_RESET);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 69;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */
  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */
  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_256;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_4;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_RESUMED_MODE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */
  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */
  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = ENABLE;
  hadc2.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_256;
  hadc2.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_4;
  hadc2.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc2.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_RESUMED_MODE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */
  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x40916E9B;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */
  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */
  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */
  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */
  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */
  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 74;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 2874;
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
  /* USER CODE BEGIN TIM6_Init 2 */
  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */
  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */
  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 9;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */
  /* USER CODE END TIM7_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|SPI_CS_Pin|Axis2B_Pin
                          |Axis2A_Pin|Axis1B_Pin|Axis1A_Pin|LED_Blink_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA2 SPI_CS_Pin Axis2B_Pin
                           Axis2A_Pin Axis1B_Pin Axis1A_Pin LED_Blink_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|SPI_CS_Pin|Axis2B_Pin
                          |Axis2A_Pin|Axis1B_Pin|Axis1A_Pin|LED_Blink_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_GPIO_WritePin(GPIOA, SPI_CS_Pin, GPIO_PIN_SET);  /* deselect IMU */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1) {}
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
