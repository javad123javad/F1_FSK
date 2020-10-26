/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define N 128
uint16_t gSinWave[N] =
  { 2048, 2149, 2250, 2350, 2450, 2549, 2646, 2742, 2837, 2929, 3020, 3108,
      3193, 3275, 3355, 3431, 3504, 3574, 3639, 3701, 3759, 3812, 3861, 3906,
      3946, 3982, 4013, 4039, 4060, 4076, 4087, 4094, 4095, 4091, 4082, 4069,
      4050, 4026, 3998, 3965, 3927, 3884, 3837, 3786, 3730, 3671, 3607, 3539,
      3468, 3394, 3316, 3235, 3151, 3064, 2975, 2883, 2790, 2695, 2598, 2500,
      2400, 2300, 2199, 2098, 1997, 1896, 1795, 1695, 1595, 1497, 1400, 1305,
      1212, 1120, 1031, 944, 860, 779, 701, 627, 556, 488, 424, 365, 309, 258,
      211, 168, 130, 97, 69, 45, 26, 13, 4, 0, 1, 8, 19, 35, 56, 82, 113, 149,
      189, 234, 283, 336, 394, 456, 521, 591, 664, 740, 820, 902, 987, 1075,
      1166, 1258, 1353, 1449, 1546, 1645, 1745, 1845, 1946, 2047 };
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void
SystemClock_Config (void);
static void
MX_GPIO_Init (void);
static void
MX_DMA_Init (void);
static void
MX_DAC_Init (void);
static void
MX_TIM2_Init (void);
static void
MX_USART1_UART_Init (void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int
main (void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init ();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config ();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init ();
  MX_DMA_Init ();
  MX_DAC_Init ();
  MX_TIM2_Init ();
  MX_USART1_UART_Init ();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT (&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
    {
//      TIM2->ARR = 59;
//      HAL_Delay(1);
//      TIM2->ARR = 199;
//
//      HAL_Delay(1);
//      TIM2->ARR = 299;
//
//      HAL_Delay(1);
//      TIM2->ARR = 399;
//      HAL_Delay(1);
      /* USER CODE END WHILE */

      /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void
SystemClock_Config (void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct =
    { 0 };
  RCC_ClkInitTypeDef RCC_ClkInitStruct =
    { 0 };

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  if (HAL_RCC_OscConfig (&RCC_OscInitStruct) != HAL_OK)
    {
      Error_Handler ();
    }
  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
      | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig (&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
      Error_Handler ();
    }
  /** Enables the Clock Security System
   */
  HAL_RCC_EnableCSS ();
}

/**
 * @brief DAC Initialization Function
 * @param None
 * @retval None
 */
static void
MX_DAC_Init (void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  LL_DAC_InitTypeDef DAC_InitStruct =
    { 0 };

  LL_GPIO_InitTypeDef GPIO_InitStruct =
    { 0 };

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock (LL_APB1_GRP1_PERIPH_DAC1);

  LL_APB2_GRP1_EnableClock (LL_APB2_GRP1_PERIPH_GPIOA);
  /**DAC GPIO Configuration
   PA4   ------> DAC_OUT1
   PA5   ------> DAC_OUT2
   */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4 | LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  LL_GPIO_Init (GPIOA, &GPIO_InitStruct);

  /* DAC DMA Init */

  /* DAC_CH1 Init */
  LL_DMA_SetDataTransferDirection (DMA2, LL_DMA_CHANNEL_3,
				   LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetChannelPriorityLevel (DMA2, LL_DMA_CHANNEL_3, LL_DMA_PRIORITY_HIGH);

  LL_DMA_SetMode (DMA2, LL_DMA_CHANNEL_3, LL_DMA_MODE_CIRCULAR);

  LL_DMA_SetPeriphIncMode (DMA2, LL_DMA_CHANNEL_3, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode (DMA2, LL_DMA_CHANNEL_3, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize (DMA2, LL_DMA_CHANNEL_3, LL_DMA_PDATAALIGN_HALFWORD);

  LL_DMA_SetMemorySize (DMA2, LL_DMA_CHANNEL_3, LL_DMA_MDATAALIGN_HALFWORD);

  /* DAC_CH2 Init */
  LL_DMA_SetDataTransferDirection (DMA2, LL_DMA_CHANNEL_4,
				   LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetChannelPriorityLevel (DMA2, LL_DMA_CHANNEL_4, LL_DMA_PRIORITY_HIGH);

  LL_DMA_SetMode (DMA2, LL_DMA_CHANNEL_4, LL_DMA_MODE_CIRCULAR);

  LL_DMA_SetPeriphIncMode (DMA2, LL_DMA_CHANNEL_4, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode (DMA2, LL_DMA_CHANNEL_4, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize (DMA2, LL_DMA_CHANNEL_4, LL_DMA_PDATAALIGN_HALFWORD);

  LL_DMA_SetMemorySize (DMA2, LL_DMA_CHANNEL_4, LL_DMA_MDATAALIGN_HALFWORD);

  /* USER CODE BEGIN DAC_Init 1 */
  LL_DMA_SetPeriphAddress (DMA2, LL_DMA_CHANNEL_3, (uint32_t) &DAC->DHR12R1);
  LL_DMA_SetMemoryAddress (DMA2, LL_DMA_CHANNEL_3, (uint32_t) &gSinWave[0]);
  LL_DMA_SetDataLength (DMA2, LL_DMA_CHANNEL_3, N);
  LL_DMA_EnableChannel (DMA2, LL_DMA_CHANNEL_3);

  /* USER CODE END DAC_Init 1 */
  /** DAC channel OUT1 config
   */
  DAC_InitStruct.TriggerSource = LL_DAC_TRIG_EXT_TIM2_TRGO;
  DAC_InitStruct.WaveAutoGeneration = LL_DAC_WAVE_AUTO_GENERATION_NONE;
  DAC_InitStruct.OutputBuffer = LL_DAC_OUTPUT_BUFFER_ENABLE;
  LL_DAC_Init (DAC, LL_DAC_CHANNEL_1, &DAC_InitStruct);
  LL_DAC_EnableTrigger (DAC, LL_DAC_CHANNEL_1);
  /** DAC channel OUT2 config
   */
  LL_DAC_Init (DAC, LL_DAC_CHANNEL_2, &DAC_InitStruct);
  LL_DAC_EnableTrigger (DAC, LL_DAC_CHANNEL_2);
  LL_DAC_Enable (DAC, LL_DAC_CHANNEL_1);
  LL_DAC_EnableDMAReq (DAC, LL_DAC_CHANNEL_1);
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void
MX_TIM2_Init (void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig =
    { 0 };
  TIM_MasterConfigTypeDef sMasterConfig =
    { 0 };

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init (&htim2) != HAL_OK)
    {
      Error_Handler ();
    }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource (&htim2, &sClockSourceConfig) != HAL_OK)
    {
      Error_Handler ();
    }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization (&htim2, &sMasterConfig) != HAL_OK)
    {
      Error_Handler ();
    }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void
MX_USART1_UART_Init (void)
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
  if (HAL_UART_Init (&huart1) != HAL_OK)
    {
      Error_Handler ();
    }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void
MX_DMA_Init (void)
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock (LL_AHB1_GRP1_PERIPH_DMA2);

  /* DMA interrupt init */
  /* DMA2_Channel3_IRQn interrupt configuration */
  NVIC_SetPriority (DMA2_Channel3_IRQn,
		    NVIC_EncodePriority (NVIC_GetPriorityGrouping (), 0, 0));
  NVIC_EnableIRQ (DMA2_Channel3_IRQn);
  /* DMA2_Channel4_5_IRQn interrupt configuration */
  NVIC_SetPriority (DMA2_Channel4_5_IRQn,
		    NVIC_EncodePriority (NVIC_GetPriorityGrouping (), 0, 0));
  NVIC_EnableIRQ (DMA2_Channel4_5_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void
MX_GPIO_Init (void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM3 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void
HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3)
    {
      HAL_IncTick ();
    }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void
Error_Handler (void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
