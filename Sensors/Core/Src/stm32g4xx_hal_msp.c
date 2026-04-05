/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32g4xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"

extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;

static uint32_t HAL_RCC_ADC12_CLK_ENABLED = 0;

void HAL_MspInit(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWREx_DisableUCPDDeadBattery();
}

void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /* Shared clock config — same for both ADC1 and ADC2 */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection  = RCC_ADC12CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) Error_Handler();

  HAL_RCC_ADC12_CLK_ENABLED++;
  if (HAL_RCC_ADC12_CLK_ENABLED == 1) __HAL_RCC_ADC12_CLK_ENABLE();

  __HAL_RCC_GPIOB_CLK_ENABLE();

  if (hadc->Instance == ADC1)
  {
    /* USER CODE BEGIN ADC1_MspInit 0 */
    /* USER CODE END ADC1_MspInit 0 */

    /**ADC1 GPIO Configuration
    PB12  ------> ADC1_IN11  (Z-axis, pin 26)
    PB14  ------> ADC1_IN5   (Y-axis, pin 28)
    */
    GPIO_InitStruct.Pin  = Acc_Zout_Pin | GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* ADC1 DMA — DMA1 Channel 1, halfword transfers, circular */
    hdma_adc1.Instance                 = DMA1_Channel1;
    hdma_adc1.Init.Request             = DMA_REQUEST_ADC1;
    hdma_adc1.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode                = DMA_CIRCULAR;
    hdma_adc1.Init.Priority            = DMA_PRIORITY_MEDIUM;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) Error_Handler();

    __HAL_LINKDMA(hadc, DMA_Handle, hdma_adc1);

    /* USER CODE BEGIN ADC1_MspInit 1 */
    /* USER CODE END ADC1_MspInit 1 */
  }
  else if (hadc->Instance == ADC2)
  {
    /* USER CODE BEGIN ADC2_MspInit 0 */
    /* USER CODE END ADC2_MspInit 0 */

    /**ADC2 GPIO Configuration
    PB15  ------> ADC2_IN15  (X-axis, pin 29)
    */
    GPIO_InitStruct.Pin  = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* ADC2 DMA — DMA1 Channel 2, halfword transfers, circular */
    hdma_adc2.Instance                 = DMA1_Channel2;
    hdma_adc2.Init.Request             = DMA_REQUEST_ADC2;
    hdma_adc2.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_adc2.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_adc2.Init.MemInc              = DMA_MINC_DISABLE;
    hdma_adc2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc2.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_adc2.Init.Mode                = DMA_CIRCULAR;
    hdma_adc2.Init.Priority            = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_adc2) != HAL_OK) Error_Handler();

    __HAL_LINKDMA(hadc, DMA_Handle, hdma_adc2);

    /* USER CODE BEGIN ADC2_MspInit 1 */
    /* USER CODE END ADC2_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{
  HAL_RCC_ADC12_CLK_ENABLED--;
  if (HAL_RCC_ADC12_CLK_ENABLED == 0) __HAL_RCC_ADC12_CLK_DISABLE();

  if (hadc->Instance == ADC1)
  {
    /* USER CODE BEGIN ADC1_MspDeInit 0 */
    /* USER CODE END ADC1_MspDeInit 0 */

    /**ADC1 GPIO Configuration
    PB12  ------> ADC1_IN11
    PB14  ------> ADC1_IN5
    */
    HAL_GPIO_DeInit(GPIOB, Acc_Zout_Pin | GPIO_PIN_14);
    HAL_DMA_DeInit(hadc->DMA_Handle);

    /* USER CODE BEGIN ADC1_MspDeInit 1 */
    /* USER CODE END ADC1_MspDeInit 1 */
  }
  else if (hadc->Instance == ADC2)
  {
    /* USER CODE BEGIN ADC2_MspDeInit 0 */
    /* USER CODE END ADC2_MspDeInit 0 */

    /**ADC2 GPIO Configuration
    PB15  ------> ADC2_IN15
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_15);
    HAL_DMA_DeInit(hadc->DMA_Handle);

    /* USER CODE BEGIN ADC2_MspDeInit 1 */
    /* USER CODE END ADC2_MspDeInit 1 */
  }
}

void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (hspi->Instance == SPI1)
  {
    /* USER CODE BEGIN SPI1_MspInit 0 */
    /* USER CODE END SPI1_MspInit 0 */

    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /**SPI1 GPIO Configuration
    PA5  ------> SPI1_SCK
    PA6  ------> SPI1_MISO
    PA7  ------> SPI1_MOSI
    */
    GPIO_InitStruct.Pin       = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USER CODE BEGIN SPI1_MspInit 1 */
    /* USER CODE END SPI1_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{
  if (hspi->Instance == SPI1)
  {
    /* USER CODE BEGIN SPI1_MspDeInit 0 */
    /* USER CODE END SPI1_MspDeInit 0 */

    __HAL_RCC_SPI1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);

    /* USER CODE BEGIN SPI1_MspDeInit 1 */
    /* USER CODE END SPI1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/* USER CODE END 1 */
