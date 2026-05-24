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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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

/* USER CODE BEGIN PV */
// Edit Check
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define CCD_BUFFER_LEN 6000

volatile uint16_t CCDPixelBuffer[CCD_BUFFER_LEN];
volatile uint8_t frame_done = 0;
volatile uint8_t frame_sent = 0;
volatile uint8_t tim2_ready = 0;

static void ADC_Calibrate_And_Enable(void)
{
    if (LL_ADC_IsEnabled(ADC1)) {
        return;
    }
    LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
    while (LL_ADC_IsCalibrationOnGoing(ADC1)) { }
    /* 4 ADC clock cycles between calibration and enable */
    for (volatile uint32_t i = 0; i < 32; i++) { __NOP(); }
    LL_ADC_Enable(ADC1);
    while (!LL_ADC_IsActiveFlag_ADRDY(ADC1)) { }
}

static void DMA_Arm_For_Capture(void)
{
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
    LL_DMA_ClearFlag_GI1(DMA1);
    LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_1,
                           LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA),
                           (uint32_t)CCDPixelBuffer,
                           LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, CCD_BUFFER_LEN);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
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

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  MX_ADC1_Init();
  MX_USB_DEVICE_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* Enable PWM channel outputs (LL OC init leaves them disabled) */
  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1); /* fM   on PA6 */
  LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH4); /* ADC trig (PB9 scope) */
  LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1); /* ICG  on PA5 */
  LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH3); /* SH   on PC8 */
  LL_TIM_EnableAllOutputs(TIM8); /* advanced timer MOE */

  /* ADC voltage regulator was started in MX_ADC1_Init; now calibrate + enable */
  ADC_Calibrate_And_Enable();

  /* Arm DMA for the first capture before any trigger can arrive */
  DMA_Arm_For_Capture();
  LL_ADC_REG_StartConversion(ADC1); /* external trigger takes it from here */

  /* TIM2 update event = end of one full ICG period -> tim2_ready */
  LL_TIM_ClearFlag_UPDATE(TIM2);
  LL_TIM_EnableIT_UPDATE(TIM2);

  HAL_Delay(3000); /* window for the host Python script */

  /* Start everything. Order matters: master (TIM3) last so the slaved
   * timers come up with their counters at 0 in lockstep. TIM8 runs on
   * its own internal clock; same APB so it stays tick-aligned. */
  LL_TIM_EnableCounter(TIM4);
  LL_TIM_EnableCounter(TIM2);
  LL_TIM_SetCounter(TIM2, 66); /* phase offset between ICG and SH */
  LL_TIM_EnableCounter(TIM8);
  LL_TIM_EnableCounter(TIM3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      if (tim2_ready && !frame_done && !frame_sent)
      {
          DMA_Arm_For_Capture();
          tim2_ready = 0;
      }

      if (frame_done && !frame_sent)
      {
          uint8_t header[] = "FRAME\n";
          HAL_UART_Transmit(&huart1, header, sizeof(header) - 1, HAL_MAX_DELAY);
          HAL_UART_Transmit(&huart1, (uint8_t*)CCDPixelBuffer,
                            CCD_BUFFER_LEN * 2, HAL_MAX_DELAY);
          frame_sent = 1;
      }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_4)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  while (LL_PWR_IsActiveFlag_VOS() != 0)
  {
  }
  LL_RCC_MSI_Enable();

   /* Wait till MSI is ready */
  while(LL_RCC_MSI_IsReady() != 1)
  {

  }
  LL_RCC_MSI_EnableRangeSelection();
  LL_RCC_MSI_SetRange(LL_RCC_MSIRANGE_6);
  LL_RCC_MSI_SetCalibTrimming(0);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_MSI, LL_RCC_PLLM_DIV_1, 40, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_EnableDomain_SYS();
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_SetSystemCoreClock(80000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  LL_RCC_PLLSAI1_ConfigDomain_48M(LL_RCC_PLLSOURCE_MSI, LL_RCC_PLLM_DIV_1, 24, LL_RCC_PLLSAI1Q_DIV_2);
  LL_RCC_PLLSAI1_EnableDomain_48M();
  LL_RCC_PLLSAI1_Enable();

   /* Wait till PLLSAI1 is ready */
  while(LL_RCC_PLLSAI1_IsReady() != 1)
  {

  }
}

/* USER CODE BEGIN 4 */
/* IRQ handlers in stm32l4xx_it.c call back into these helpers */
void on_tim2_update(void)
{
    tim2_ready = 1;
}

void on_dma_transfer_complete(void)
{
    /* One full frame of CCD_BUFFER_LEN samples landed. Stop the channel so
     * we don't wrap into the next frame before the main loop sends this one. */
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
    frame_done = 1;
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
