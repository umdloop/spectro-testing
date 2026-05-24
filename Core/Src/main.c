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

extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
extern USBD_HandleTypeDef hUsbDeviceFS;

/* Periods for the timers controlling SH- and ICG-pulses. Sized for F401
 * wall-clock parity (SH ~625 us, ICG ~250 ms / 4 Hz frame rate).
 *
 * SH is on TIM8 internal clock at 80 MHz (10x F401's 8 MHz), so SH_period
 * scales 10x: 5000 -> 50000.
 *
 * ICG is on TIM2 clocked by TIM3 TRGO. TIM3 ARR=40 gives a 2 MHz tick
 * (vs F401's 800 kHz: 8 MHz / 10). That's 2.5x faster than F401, so ICG_period
 * scales 2.5x, not 10x: 200000 -> 500000. */
__IO uint32_t SH_period = 50000;
__IO uint32_t ICG_period = 500000;

__IO uint16_t aTxBuffer[CCDSize];

/* Flags */
__IO uint8_t change_exposure_flag = 0; /* set by CDC RX when host sends new ICG/SH */
__IO uint8_t data_flag = 0;
/* Flag to signal what action to do in main-loop
 *   0 do nothing
 *   1 transmit data in aTxBuffer
 */

__IO uint8_t pulse_counter = 8;
__IO uint8_t CCD_flushed = 0;  /* set to 1 by TIM2 IRQ after 2nd ICG pulse */
__IO uint8_t in_reading = 0;   /* 1 while ADC is sampling a frame */
__IO uint8_t coll_mode = 1;    /* 1 = continuous capture */
__IO uint8_t avg_exps  = 1;    /* 1 = single-shot per frame (no averaging) */
__IO uint8_t exps_left = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */
void flush_CCD(void);
void CCD_set_clocks(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* TIM Configuration (L476 port of F401 tcd1304-usb)
 *   fM  (TIM3)  PA6 (CH1)  - master clock
 *   SH  (TIM8)  PC8 (CH3)  - was TIM5 CH4 / PA3 on F401
 *   ICG (TIM2)  PA5 (CH1)  - was TIM2 CH3 / PA2 on F401
 *   CCD-output  PA3 (ADC1_IN8) - was PA0 / ADC1_IN0 on F401
 */

static void ADC_Calibrate_And_Enable(void)
{
    if (LL_ADC_IsEnabled(ADC1)) return;
    LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
    while (LL_ADC_IsCalibrationOnGoing(ADC1)) { }
    /* short delay between calibration and enable */
    for (volatile uint32_t i = 0; i < 32; i++) { __NOP(); }
    LL_ADC_Enable(ADC1);
    while (!LL_ADC_IsActiveFlag_ADRDY(ADC1)) { }
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
  /* Enable the capture/compare interrupt for channel 1 (ICG rising edge on
   * L476 — was CC3 on the F401 because ICG was TIM2_CH3 there). */
  LL_TIM_EnableIT_CC1(TIM2);

  /**********************************/
  /* Start output signal generation */
  /**********************************/
  /* Enable output channels (LL OC init leaves them disabled) */
  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH4);
  /* Enable counter */
  LL_TIM_EnableCounter(TIM3);
  LL_TIM_EnableCounter(TIM4);
  /* stop ADC timer */
  TIM4->CR1 &= (uint16_t)~TIM_CR1_CEN;

  /* start TIM2 (ICG) and TIM8 (SH) */
  LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH3);
  LL_TIM_EnableAllOutputs(TIM8); /* advanced-timer MOE */
  LL_TIM_EnableCounter(TIM2);
  LL_TIM_EnableCounter(TIM8);

  CCD_set_clocks();

  /* Arm DMA1 Channel 1 for ADC->memory transfers. Cube's adc.c sets the DMA
   * *mode* (direction/circular/widths) but does NOT set addresses, length,
   * the TC interrupt, or the channel enable bit — those land here. */
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1,
      LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA));
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)aTxBuffer);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, CCDSize);
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

  /* Start the ADC, so it's ready to convert when ADC timer (TIM4) is restarted.
   * L476 needs calibration + ADRDY wait first (F4 didn't). */
  ADC_Calibrate_And_Enable();
  LL_ADC_REG_StartConversion(ADC1);

  /* F401 stays idle until a host command resets pulse_counter via flush_CCD().
   * We have no command channel, so kick off a flush so frame 1 isn't garbage
   * and the IRQ pulse_counter cycle (2 -> flushed, 4 -> start TIM4) begins. */
  flush_CCD();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (change_exposure_flag == 1)
    {
      change_exposure_flag = 0;

      /* wait for any in-flight ADC capture to finish */
      while (in_reading == 1);

      flush_CCD();
      CCD_set_clocks();
    }

    switch (data_flag)
    {
      case 1: /* transmit data in aTxBuffer */
        data_flag = 0;

        /* continuous mode? then collect again at next ICG-pulse */
        if (coll_mode == 1)
          pulse_counter = 4;

        /* Only transmit if the host has enumerated the CDC class. Without
         * this guard, CDC_Transmit_FS dereferences a NULL pClassData and
         * hard-faults if USB isn't plugged in / enumerated yet. */
        if (hUsbDeviceFS.pClassData != NULL)
          CDC_Transmit_FS((uint8_t*) aTxBuffer, 2*CCDSize);
        break;
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
/* Direct port of F401 flush_CCD(): run a short ICG cycle twice to dump
 * residual charge before real readout. Flush periods scaled 10x to match
 * F401 wall-clock at 80 MHz timer tick (was 8 MHz on F401). */
void flush_CCD(void)
{
    TIM2->ARR = 150000 - 1;
    TIM8->ARR = 200 - 1;
    TIM3->ARR = 40 - 1;   /* L476: 40 -> fM = 2 MHz (F401 had 10 @ 8 MHz tick) */

    LL_TIM_GenerateEvent_UPDATE(TIM2);
    LL_TIM_GenerateEvent_UPDATE(TIM3);
    LL_TIM_GenerateEvent_UPDATE(TIM8);

    /* align output */
    TIM3->CNT = 0;
    TIM2->CNT = 150000 - ICG_delay;
    TIM8->CNT = 200 - SH_delay;

    CCD_flushed = 0;
    pulse_counter = 0;

    while (CCD_flushed == 0);

    /* Restore operational ARRs so the next ICG/SH pulses fire at the
     * capture timing, not the flush timing. */
    CCD_set_clocks();
}

void CCD_set_clocks(void)
{
    TIM2->ARR = ICG_period - 1;
    TIM8->ARR = SH_period - 1;
    TIM3->ARR = 40 - 1;

    LL_TIM_GenerateEvent_UPDATE(TIM2);
    LL_TIM_GenerateEvent_UPDATE(TIM3);
    LL_TIM_GenerateEvent_UPDATE(TIM8);

    /* align output */
    TIM3->CNT = 0;
    TIM2->CNT = ICG_period - ICG_delay;
    TIM8->CNT = SH_period - SH_delay;
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
