/*
 ******************************************************************************
 * @file    main.c
 * @module  app
 * @author  Mohamed Hnezli
 * @version V0.2
 * @date    08-09-2019
 * @brief   implements oscilloscope function: samples input signal and displays
 *          on LCD.
 ******************************************************************************
 * @attention
 *
 * - The present software had been tested only on STM32F3CCTx and may be non
 * functional on other targets.
 *
 * <h2><center>&copy COPYRIGHT 2019 Mohamed Hnezli </center></h2>
 ******************************************************************************
 */

/* include ------------------------------------------------------------------ */
#include "stm32f4xx_hal.h"
#include "stm32f4_discovery.h"
#include "lcd.h"                       // LCD driver
#include "gui.h"                       // LCD generic API
#include <math.h>                      // math funtion
#include <stdio.h>                     // snprintf


/* define ------------------------------------------------------------------- */
/*
 * @brief AD620 signal generator interface
 */
#define DAC_AD620                      DAC
#define DAC_CLK_EN_AD620()             __HAL_RCC_DAC_CLK_ENABLE()
#define DAC_GPIO_PORT                  GPIOA
#define DAC_GPIO_PIN                   GPIO_PIN_4
#define GPIO_CLK_EN_AD620()            __HAL_RCC_GPIOA_CLK_ENABLE()

/*
 * @brief samplers interfaces
 */
#define ADC_SAMPLER1                   ADC1
#define ADC_CLK_EN_SAMPLER1()          __HAL_RCC_ADC1_CLK_ENABLE()
#define ADC_CH_SAMPLER1                ADC_CHANNEL_3

#define ADC_SAMPLER2                   ADC2
#define ADC_CLK_EN_SAMPLER2()          __HAL_RCC_ADC2_CLK_ENABLE()
#define ADC_CH_SAMPLER2                ADC_CHANNEL_2

/*
 * @brief samplers GPIO ports and pins
 */
#define GPIO_PORT_SAMPLER1_INPUT       GPIOA
#define GPIO_PIN_SAMPLER1_INPUT        GPIO_PIN_3
#define GPIO_CLK_EN_SAMPLER1_INPUT()   __HAL_RCC_GPIOA_CLK_ENABLE()
#define GPIO_PORT_SAMPLER1_A           GPIOC
#define GPIO_PIN_SAMPLER1_A            GPIO_PIN_1
#define GPIO_CLK_EN_SAMPLER1_A()       __HAL_RCC_GPIOC_CLK_ENABLE()
#define GPIO_PORT_SAMPLER1_B           GPIOC
#define GPIO_PIN_SAMPLER1_B            GPIO_PIN_3
#define GPIO_CLK_EN_SAMPLER1_B()       __HAL_RCC_GPIOC_CLK_ENABLE()
#define GPIO_PORT_SAMPLER1_C           GPIOA
#define GPIO_PIN_SAMPLER1_C            GPIO_PIN_1
#define GPIO_CLK_EN_SAMPLER1_C()       __HAL_RCC_GPIOA_CLK_ENABLE()

#define GPIO_PORT_SAMPLER2_INPUT       GPIOA
#define GPIO_PIN_SAMPLER2_INPUT        GPIO_PIN_2
#define GPIO_CLK_EN_SAMPLER2_INPUT()   __HAL_RCC_GPIOA_CLK_ENABLE()
#define GPIO_PORT_SAMPLER2_A           GPIOE
#define GPIO_PIN_SAMPLER2_A            GPIO_PIN_6
#define GPIO_CLK_EN_SAMPLER2_A()       __HAL_RCC_GPIOE_CLK_ENABLE()
#define GPIO_PORT_SAMPLER2_B           GPIOE
#define GPIO_PIN_SAMPLER2_B            GPIO_PIN_4
#define GPIO_CLK_EN_SAMPLER2_B()       __HAL_RCC_GPIOE_CLK_ENABLE()
#define GPIO_PORT_SAMPLER2_C           GPIOE
#define GPIO_PIN_SAMPLER2_C            GPIO_PIN_2
#define GPIO_CLK_EN_SAMPLER2_C()       __HAL_RCC_GPIOE_CLK_ENABLE()

/*
 * @brief timer interface
 */
#define TIMER_IFACE_DISPLAY            TIM2
#define TIMER_CLK_EN_DISPLAY()         __TIM2_CLK_ENABLE()
#define TIMER_IFACE_DELAY              TIM3
#define TIMER_CLK_EN_DELAY()           __TIM3_CLK_ENABLE()

/*
 * @brief maximum samples in an array
 */
#define DISPLAY_MAX_POINTS             1024

/*
 * @brief IRQ configuration
 */
#define IRQ_LINE_TIMER_DISPLAY         TIM2_IRQn
#define IRQ_PRIO_TIMER_DISPLAY         3
#define IRQ_LINE_TIMER_DELAY           TIM3_IRQn
#define IRQ_PRIO_TIMER_DELAY           4

/*
 * @brief signal modulation parameters
 */
#define SAMPLER_VMAX_1                 ((float) 4.0)       // [V]
#define SAMPLER_VMAX_2                 ((float) 8.0)       // [V]
#define SAMPLER_VMAX_3                 ((float)20.0)       // [V]
#define SAMPLER_VMAX_4                 ((float)40.0)       // [V]
#define SAMPLER_VMAX_5                 ((float)60.0)       // [V]

#define RESISTOR_R1_11                 ((float)200000.0)   // [ohm]
#define RESISTOR_R2_12                 ((float)270000.0)   // [ohm]
#define RESISTOR_R3_21                 ((float)680000.0)   // [ohm]
#define RESISTOR_R4_22                 ((float)270000.0)   // [ohm]
#define RESISTOR_R5_31                 RESISTOR_R3_21      // [ohm]
#define RESISTOR_R6_32                 ((float)100000.0)   // [ohm]
#define RESISTOR_R7_41                 RESISTOR_R3_21      // [ohm]
#define RESISTOR_R8_42                 ((float) 51000.0)   // [ohm]
#define RESISTOR_R9_51                 RESISTOR_R3_21      // [ohm]
#define RESISTOR_R10_52                ((float) 33000.0)   // [ohm]
#define SAMPLER_GAIN_1                 (RESISTOR_R2_12 / (RESISTOR_R1_11 + RESISTOR_R2_12))
#define SAMPLER_GAIN_2                 (RESISTOR_R4_22 / (RESISTOR_R3_21 + RESISTOR_R4_22))
#define SAMPLER_GAIN_3                 (RESISTOR_R6_32 / (RESISTOR_R5_31 + RESISTOR_R6_32))
#define SAMPLER_GAIN_4                 (RESISTOR_R8_42 / (RESISTOR_R7_41 + RESISTOR_R8_42))
#define SAMPLER_GAIN_5                 (RESISTOR_R10_52 / (RESISTOR_R9_51 + RESISTOR_R10_52))

/*
 * @brief display setting
 */
#define DISP_COLOR_BACKGND             BLACK
#define DISP_COLOR_TRIGGLINE           BLUE
#define DISP_COLOR_GRID                WHITE
#define DISP_COLOR_CH1                 YELLOW
#define DISP_COLOR_CH2                 RED
#define DISP_SIZE_HEADER               20                  // [px]
#define DISP_SIZE_FOOTER               20                  // [px]
#define DISP_SIZE_SPACING              5                   // [px]

#define DISP_GRID_ROWS                 8
#define DISP_GRID_COLS                 10

#define DISP_SIZE_PLOT                 (LCD_H - (DISP_SIZE_HEADER + DISP_SIZE_FOOTER + (2 * DISP_SIZE_SPACING)))

#define DISP_MILLISPERDIV_10US         10
#define DISP_MILLISPERDIV_50US         50
#define DISP_MILLISPERDIV_1MS          1000
#define DISP_MILLISPERDIV_10MS         (10 * 1000)
#define DISP_MILLISPERDIV_50MS         (50 * 1000)
#define DISP_MILLISPERDIV_1S           (1000 * 1000)
#define DISP_MILLISPERDIV_5S           (5 * 1000 * 1000)

/*
 * @brief serial line parameters. TODO change
 */
#define SERIAL_BAUDRATE                115200
#define SERIAL_UART                    UART4
#define SERIAL_UART_CLK_EN()           __HAL_RCC_UART4_CLK_ENABLE()
#define SERIAL_GPIO_PORT_TX             GPIOA
#define SERIAL_GPIO_PIN_TX             GPIO_PIN_0
#define SERIAL_TX_GPIO_CLK_EN()        __HAL_RCC_GPIOA_CLK_ENABLE()
#define SERIAL_GPIO_PORT_RX            GPIOC
#define SERIAL_GPIO_PIN_RX             GPIO_PIN_11
#define SERIAL_RX_GPIO_CLK_EN()        __HAL_RCC_GPIOC_CLK_ENABLE()


/* typedef ------------------------------------------------------------------ */
/* variable ----------------------------------------------------------------- */
/*
 * @brief same ADC used for all analog sensors
 */
static ADC_HandleTypeDef sampler1_adc_;
static ADC_HandleTypeDef sampler2_adc_;

/*
 * @brief timer instances
 */
TIM_HandleTypeDef disp_tim_;           // LCD display refresh
TIM_HandleTypeDef delay_tim_;          // delay counter

/*
 * @brief microseconds counter;
 */
uint64_t micros_ = 0.0;

/*
 * @brief signal samples
 */
static uint32_t samples1_freq = 50;              // [Hz]
static float samples1_delay = 0;                 // time between to successive samples
static uint32_t samples2_freq = 0;               // [Hz]
static float samples2_delay = 0;                 // time between to successive samples
static float samples1_[DISPLAY_MAX_POINTS];      // stores ADC values
static float samples2_[DISPLAY_MAX_POINTS];      // stores ADC values
static float samples1_range_ = 0.0;              // display value range [V]
static float samples1_max_ = 0.0, samples1_min_ = 0.0, samples1_mean_ = 0.0;    // samples stats [V]
static float samples2_range_ = 0.0;              // display value range [V]
static float samples2_max_ = 0.0, samples2_min_ = 0.0, samples2_mean_ = 0.0;    // samples stats [V]
static float sampler1_gain_ = 0.0;               // signal modulation gain
static float sampler2_gain_ = 0.0;               // signal modulation gain

/*
 * @brief display setting
 */
static float millisperdiv = 0.0;
static float voltsperdiv = 0.0;
static float trigger_line_ = 0.0;
uint32_t refresh_pending = 0;

/*
 * @brief serial interface.
 */
static UART_HandleTypeDef serial_uart_;


/* class -------------------------------------------------------------------- */
/* method ------------------------------------------------------------------- */
/* function ----------------------------------------------------------------- */
/*
 * @name   error_critical.
 * @brief  halts MCU on hard error occurrence.
 * @param  file: name of file in which error occurred.
 *         line: number of line in which error occurred.
 * @retval none.
 */
static void
error_critical (char* file, int line)
{
  UNUSED (file);
  UNUSED (line);

  while (1)
    {
      ; // halt program
    }

  return;
}

/*
 * @name   sysclk_Config.
 * @brief  system Clock configuration.
 * @param  none.
 * @retval none.
 * @note   HCLK is set to 48MHz, APB1 24MHz, APB2 24MHz
 */
static void
sysclk_Config (void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* configure the main internal regulator output voltage */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* initializes the CPU, AHB and APB busses clocks
   * SySclk = (((SRC / M) * N) / P) */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
      error_critical (__FILE__, __LINE__);
    }

  /* initializes the CPU, AHB and APB busses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    {
      error_critical (__FILE__, __LINE__);
    }

  return;
}

/*
 * @name   sampler1_init.
 * @brief  configures signal sampler 1 GPIO and ADC interfaces.
 * @param  none.
 * @retval none.
 */
static void
sampler1_init (void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* GPIO configuration */
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Pin = GPIO_PIN_SAMPLER1_INPUT;
  GPIO_CLK_EN_SAMPLER1_INPUT ();
  HAL_GPIO_Init (GPIO_PORT_SAMPLER1_INPUT, &GPIO_InitStruct);

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Pin = GPIO_PIN_SAMPLER1_A;
  GPIO_CLK_EN_SAMPLER1_A ();
  HAL_GPIO_Init (GPIO_PORT_SAMPLER1_A, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_SAMPLER1_B;
  GPIO_CLK_EN_SAMPLER1_B ();
  HAL_GPIO_Init (GPIO_PORT_SAMPLER1_B, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_SAMPLER1_C;
  GPIO_CLK_EN_SAMPLER1_C ();
  HAL_GPIO_Init (GPIO_PORT_SAMPLER1_C, &GPIO_InitStruct);

  /* configure ADc interface with 10bit accuracy */
  sampler1_adc_.Instance = ADC_SAMPLER1;
  sampler1_adc_.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  sampler1_adc_.Init.Resolution = ADC_RESOLUTION_10B;
  sampler1_adc_.Init.ScanConvMode = DISABLE;
  sampler1_adc_.Init.ContinuousConvMode = ENABLE;
  sampler1_adc_.Init.DiscontinuousConvMode = DISABLE;
  sampler1_adc_.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  sampler1_adc_.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  sampler1_adc_.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  sampler1_adc_.Init.NbrOfConversion = 1;
  sampler1_adc_.Init.DMAContinuousRequests = DISABLE;
  sampler1_adc_.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  ADC_CLK_EN_SAMPLER1 ();              // enable clock
  if (HAL_ADC_Init(&sampler1_adc_) != HAL_OK)
    {
      error_critical (__FILE__, __LINE__);
    }

  /* link ADC to GPIO */
  sConfig.Channel = ADC_CH_SAMPLER1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&sampler1_adc_, &sConfig) != HAL_OK)
    {
      error_critical (__FILE__, __LINE__);
    }

  return;
}

/*
 * @name   sampler1_setrange.
 * @brief  selects which voltage input to apply to input circuit.
 * @param  range: voltage maximum index: [1..5].
 * @retval none.
 */
static void
sampler1_setrange (uint32_t range)
{
  uint32_t a = 0, b = 0, c = 0;

  switch (range)
  {
  case 1:
    samples1_range_ = SAMPLER_VMAX_1;
    sampler1_gain_ = SAMPLER_GAIN_1;
    break;

  case 2:
    samples1_range_ = SAMPLER_VMAX_2;
    sampler1_gain_ = SAMPLER_GAIN_2;
    break;

  case 3:
    samples1_range_ = SAMPLER_VMAX_3;
    sampler1_gain_ = SAMPLER_GAIN_3;
    break;

  case 4:
    samples1_range_ = SAMPLER_VMAX_4;
    sampler1_gain_ = SAMPLER_GAIN_4;
    break;

  case 5:
    samples1_range_ = SAMPLER_VMAX_5;
    sampler1_gain_ = SAMPLER_GAIN_5;
    break;

  default:
    samples1_range_ = (-1.0);// unknown value
    sampler1_gain_ = (-1.0); // unknown value
  }

  /* select input line from AD620 */
  a = ((range - 1)     ) & 0x01;
  b = ((range - 1) >> 1) & 0x01;
  c = ((range - 1) >> 2) & 0X01;

  HAL_GPIO_WritePin (GPIO_PORT_SAMPLER1_A, GPIO_PIN_SAMPLER1_A, a);
  HAL_GPIO_WritePin (GPIO_PORT_SAMPLER1_B, GPIO_PIN_SAMPLER1_B, b);
  HAL_GPIO_WritePin (GPIO_PORT_SAMPLER1_C, GPIO_PIN_SAMPLER1_C, c);

  return;
}

/*
 * @name   sampler1_getval.
 * @brief  computes effective input signal amplitude.
 * @param  none.
 * @retval float: positive: signal voltage [V].
 *                negative: sampler failure.
 */
static float
sampler1_getval (void)
{
  float retval = 0.0;
  float voltage = 0.0;

  if (HAL_ADC_Start (&sampler1_adc_) != HAL_OK)
    {
      retval = (-1.0);
    }
  else
    {
//      HAL_Delay (5);
      if (HAL_ADC_PollForConversion (&sampler1_adc_, 100) != HAL_OK)
        {
          retval = (-1.0);
        }
      else
        {
          voltage = (((float)HAL_ADC_GetValue (&sampler1_adc_)) / 1023.0) * 3.3;
          retval = ((2 * voltage) - 3.3) / sampler1_gain_; // handle level shifter equation
        }
    }

  HAL_ADC_Stop (&sampler1_adc_);

  return retval;
}

/*
 * @name   sampler2_init.
 * @brief  configure signal sampler 2 GPIO and ADC interfaces.
 * @param  none.
 * @retval none.
 */
static void
sampler2_init (void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* GPIO configuration */
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Pin = GPIO_PIN_SAMPLER2_INPUT;
  GPIO_CLK_EN_SAMPLER2_INPUT ();
  HAL_GPIO_Init (GPIO_PORT_SAMPLER2_INPUT, &GPIO_InitStruct);

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Pin = GPIO_PIN_SAMPLER2_A;
  GPIO_CLK_EN_SAMPLER2_A ();
  HAL_GPIO_Init (GPIO_PORT_SAMPLER2_A, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_SAMPLER2_B;
  GPIO_CLK_EN_SAMPLER2_B ();
  HAL_GPIO_Init (GPIO_PORT_SAMPLER2_B, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_SAMPLER2_C;
  GPIO_CLK_EN_SAMPLER2_C ();
  HAL_GPIO_Init (GPIO_PORT_SAMPLER2_C, &GPIO_InitStruct);

  /* configure ADc interface with 10bit accuracy */
  sampler1_adc_.Instance = ADC_SAMPLER2;
  sampler1_adc_.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  sampler1_adc_.Init.Resolution = ADC_RESOLUTION_10B;
  sampler1_adc_.Init.ScanConvMode = DISABLE;
  sampler1_adc_.Init.ContinuousConvMode = ENABLE;
  sampler1_adc_.Init.DiscontinuousConvMode = DISABLE;
  sampler1_adc_.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  sampler1_adc_.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  sampler1_adc_.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  sampler1_adc_.Init.NbrOfConversion = 1;
  sampler1_adc_.Init.DMAContinuousRequests = DISABLE;
  sampler1_adc_.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  ADC_CLK_EN_SAMPLER2 ();              // enable clock
  if (HAL_ADC_Init(&sampler1_adc_) != HAL_OK)
    {
      error_critical (__FILE__, __LINE__);
    }

  /* link ADC to GPIO */
  sConfig.Channel = ADC_CH_SAMPLER2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&sampler2_adc_, &sConfig) != HAL_OK)
    {
      error_critical (__FILE__, __LINE__);
    }

  return;
}

/*
 * @name   sampler2_setrange.
 * @brief  selects which voltage input to apply to input circuit.
 * @param  range: voltage maximum index: [1..5].
 * @retval none.
 */
static void
sampler2_setrange (uint32_t range)
{
  uint32_t a = 0, b = 0, c = 0;

  switch (range)
  {
  case 1:
    samples2_range_ = SAMPLER_VMAX_1;
    sampler2_gain_ = SAMPLER_GAIN_1;
    break;

  case 2:
    samples2_range_ = SAMPLER_VMAX_2;
    sampler2_gain_ = SAMPLER_GAIN_2;
    break;

  case 3:
    samples2_range_ = SAMPLER_VMAX_3;
    sampler2_gain_ = SAMPLER_GAIN_3;
    break;

  case 4:
    samples2_range_ = SAMPLER_VMAX_4;
    sampler2_gain_ = SAMPLER_GAIN_4;
    break;

  case 5:
    samples2_range_ = SAMPLER_VMAX_5;
    sampler2_gain_ = SAMPLER_GAIN_5;
    break;

  default:
    samples2_range_ = (-1.0);// unknown value
    sampler2_gain_ = (-1.0); // unknown value
  }

  /* select input line from AD620 */
  a = ((range - 1)     ) & 0x01;
  b = ((range - 1) >> 1) & 0x01;
  c = ((range - 1) >> 2) & 0X01;

  HAL_GPIO_WritePin (GPIO_PORT_SAMPLER2_A, GPIO_PIN_SAMPLER2_A, a);
  HAL_GPIO_WritePin (GPIO_PORT_SAMPLER2_B, GPIO_PIN_SAMPLER2_B, b);
  HAL_GPIO_WritePin (GPIO_PORT_SAMPLER2_C, GPIO_PIN_SAMPLER2_C, c);

  return;
}

/*
 * @name   sampler2_getval.
 * @brief  computes effective input signal amplitude.
 * @param  none.
 * @retval float: positive: signal voltage [V].
 *                negative: sampler failure.
 */
static float
sampler2_getval (void)
{
  float retval = 0.0;
  float voltage = 0.0;

  if (HAL_ADC_Start (&sampler2_adc_) != HAL_OK)
    {
      retval = (-1.0);
    }
  else
    {
//      HAL_Delay (5);
      if (HAL_ADC_PollForConversion (&sampler2_adc_, 100) != HAL_OK)
        {
          retval = (-1.0);
        }
      else
        {
          voltage = (((float)HAL_ADC_GetValue (&sampler2_adc_)) / 1023.0) * 3.3;
          retval = ((2 * voltage) - 3.3) / sampler2_gain_; // handle level shifter equation
        }
    }

  HAL_ADC_Stop (&sampler2_adc_);

  return retval;
}

/*
 * @name   disp_timer_init.
 * @brief  configures timer to sample and print data.
 * @param  none.
 * @retval none.
 * @note   timer is set to 1ms resolution.
 */
static void
disp_timer_init (void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /*
   * 48MHz / ((47999 + 1) * (9999 + 1))
   * = 48000000 / (48000 * 1000)
   * = 0.1Hz
   */
  disp_tim_.Instance = TIMER_IFACE_DISPLAY;
  disp_tim_.Init.Prescaler = 47999;
  disp_tim_.Init.CounterMode = TIM_COUNTERMODE_UP;
  disp_tim_.Init.Period = 9999;
  disp_tim_.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  TIMER_CLK_EN_DISPLAY();
  if (HAL_TIM_Base_Init (&disp_tim_) != HAL_OK)
    {
      error_critical (__FILE__, __LINE__);
    }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization (&disp_tim_, &sMasterConfig) != HAL_OK)
    {
      error_critical (__FILE__, __LINE__);
    }

  HAL_NVIC_SetPriority(IRQ_LINE_TIMER_DISPLAY, IRQ_PRIO_TIMER_DISPLAY, 0);
  HAL_NVIC_EnableIRQ(IRQ_LINE_TIMER_DISPLAY);

  /* start samples displaying */
  HAL_TIM_Base_Start_IT (&disp_tim_);

  return;
}

/*
 * @name   delay_timer_init.
 * @brief  configures timer to ensure delay periods.
 * @param  none.
 * @retval none.
 * @note   timer is set to 1us resolution.
 */
static void
delay_timer_init (void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /*
   * 48MHz / ((0 + 1) * (47 + 1))
   * = 48000000 / (1 * 48)
   * = 1MHz
   */
  delay_tim_.Instance = TIMER_IFACE_DELAY;
  delay_tim_.Init.Prescaler = 0;
  delay_tim_.Init.CounterMode = TIM_COUNTERMODE_UP;
  delay_tim_.Init.Period = 47;
  delay_tim_.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  TIMER_CLK_EN_DISPLAY();
  if (HAL_TIM_Base_Init (&delay_tim_) != HAL_OK)
    {
      error_critical (__FILE__, __LINE__);
    }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization (&delay_tim_, &sMasterConfig) != HAL_OK)
    {
      error_critical (__FILE__, __LINE__);
    }

  HAL_NVIC_SetPriority(IRQ_LINE_TIMER_DELAY, IRQ_PRIO_TIMER_DELAY, 0);
  HAL_NVIC_EnableIRQ(IRQ_LINE_TIMER_DELAY);

  return;
}

/*
 * @name _ delay_us.
 * @brief  halts program from microseconds accurate delay.
 * @param  usec: delay time [us].
 * @retval none.
 */
static void
delay_us (uint32_t usec)
{
  uint64_t deadline = usec + micros_;

  /* keep waiting until delay elapses */
  HAL_TIM_Base_Start_IT (&delay_tim_);

  while (micros_ < deadline)
    {
      ;            // do nothing
    }

  HAL_TIM_Base_Stop_IT (&delay_tim_);

  return;
}

/*
 * @name   serial_init.
 * @brief  configures GPIO and UART ports of serial interface.
 * @param  none.
 * @retval none.
 */
static void
serial_init (void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  HAL_StatusTypeDef ret_val = HAL_OK;

  /* setup GPIO pins */
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  GPIO_InitStruct.Pin = SERIAL_GPIO_PIN_TX | SERIAL_GPIO_PIN_RX;

  SERIAL_TX_GPIO_CLK_EN();
  HAL_GPIO_Init (SERIAL_GPIO_PORT_TX, &GPIO_InitStruct);

  /* set-up hardware */
  serial_uart_.Instance = SERIAL_UART;
  HAL_UART_DeInit (&serial_uart_);
  serial_uart_.Init.BaudRate = SERIAL_BAUDRATE;
  serial_uart_.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  serial_uart_.Init.Mode = UART_MODE_TX_RX;
  serial_uart_.Init.Parity = UART_PARITY_NONE;
  serial_uart_.Init.StopBits = UART_STOPBITS_1;
  serial_uart_.Init.WordLength = UART_WORDLENGTH_8B;
  serial_uart_.Init.OverSampling = UART_OVERSAMPLING_16;
//  serial_uart_.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
//  serial_uart_.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  SERIAL_UART_CLK_EN();
  ret_val = HAL_UART_Init (&serial_uart_);

  if (ret_val != HAL_OK)
    {
      while (1)
        {
          ;    // halt program
        }
    }

  return;
}

/*
 * @name   serial_send.
 * @brief  write a sampled value to serial interface.
 * @param  sample: sample value..
 * @retval int32_t: positive: number of bytes transferred.
 *                  negative: transmission has failed.
 */
static int32_t
serial_send ()
{
  int32_t rx_status = HAL_OK;
  int32_t retval = 0;
  char buffer[32];
  uint32_t len = 0;

  /* format data */
// TODO apply format

  rx_status = HAL_UART_Transmit (&serial_uart_, (uint8_t*)buffer, len, 0xFFFFF);
  if (rx_status != HAL_OK)
    {
      retval = (-1);
    }
  else
    {
      retval = (int32_t)len;
    }

  return retval;
}

/*
 * @name   plot_init.
 * @brief  sets up and displays initial GUI.
 * @param  none.
 * @retval none.
 */
static void
plot_init (void)
{
  uint8_t header[128];
  uint8_t len = 0;

  LCD_Clear (DISP_COLOR_BACKGND);
  LCD_direction (USE_VERTICAL);

  return;
}

/*
 * @name   plot_setheader.
 * @brief  updates display header.
 * @param  none.
 * @retval none.
 */
static void
plot_setheader (void)
{
  uint32_t str_max_size = 56;
  uint8_t header[str_max_size];

  voltsperdiv = (samples1_range_ / (DISP_GRID_ROWS / 2));
//  millisperdiv = (samples1_delay * (DISPLAY_MAX_POINTS / DISP_GRID_COLS)) / 1000.0;

  POINT_COLOR = WHITE;
  LCD_DrawRectangle (0, 0, lcddev.width - 1, DISP_SIZE_HEADER);

  LCD_Fill (1, 1, lcddev.width - 2, DISP_SIZE_HEADER - 1, DISP_COLOR_BACKGND);
  snprintf ((char*)header, str_max_size, "X: %.3fms Y: %.2fV Freq %uHz | Gain: CH1(%.2f) CH2(%.2f)",
            millisperdiv, voltsperdiv, samples1_freq,sampler1_gain_, sampler2_gain_) + 1;
  Gui_StrCenter(0, 2, YELLOW, DISP_COLOR_BACKGND, (uint8_t*)header, 14, 0);

  return;
}

/*
 * @name   plot_setfooter.
 * @brief  updates display footer.
 * @param  none.
 * @retval none.
 */
void
plot_setfooter (void)
{
  uint32_t str_max_size = 56;
  uint8_t footer[str_max_size];

  POINT_COLOR = WHITE;
  LCD_DrawRectangle (0, lcddev.height - DISP_SIZE_FOOTER, lcddev.width - 1, lcddev.height - 1);

  LCD_Fill (1, (lcddev.height - DISP_SIZE_FOOTER) + 1, lcddev.width - 2, lcddev.height - 2, DISP_COLOR_BACKGND);
  snprintf ((char*)footer, str_max_size, "CH1: Vmax %.2fV Vmin %.2fV Vmean %.2fV Veff %.2fV Per %.2fs",// Freq %u",
            samples1_max_, samples1_min_, samples1_mean_, samples1_max_ / sqrt(2), 1.0 / ((float)samples1_freq)) + 1;
  Gui_StrCenter(0, (lcddev.height - DISP_SIZE_FOOTER) + 1, YELLOW, DISP_COLOR_BACKGND, (uint8_t*)footer, 14, 0);

  return;
}

/*
 * @name   plot_setgrid.
 * @brief  displays the grid in the middle..
 * @param  none.
 * @retval none.
 */
void
plot_setgrid (void)
{
  uint32_t idx = 0;
  uint32_t rows = DISP_GRID_ROWS, columns = DISP_GRID_COLS;

  /* clear current plot */
  LCD_Fill (0, DISP_SIZE_HEADER + 1, lcddev.width, (lcddev.height - DISP_SIZE_FOOTER) - 1, DISP_COLOR_BACKGND);

  /* plot grid */
  POINT_COLOR = DISP_COLOR_GRID;
  for (idx = 1; idx<columns; idx += 1)
    {
      LCD_DrawLine ((uint16_t)(idx * (lcddev.width/columns)), (uint16_t)(DISP_SIZE_HEADER+1),
                    (uint16_t)(idx * (lcddev.width/columns)), (uint16_t)lcddev.height - (DISP_SIZE_FOOTER+1));
    }
  for (idx = 1; idx<rows; idx += 1)
    {
      LCD_DrawLine ((uint16_t)0, (uint16_t)((idx*((lcddev.height - (DISP_SIZE_HEADER + DISP_SIZE_FOOTER)) / rows)) + DISP_SIZE_HEADER) + 1,
                    (uint16_t)lcddev.width, ((idx*((lcddev.height - (DISP_SIZE_HEADER + DISP_SIZE_FOOTER)) / rows)) + DISP_SIZE_HEADER) + 1);
    }

  /* plot trigger line */
  POINT_COLOR = DISP_COLOR_TRIGGLINE;
  LCD_DrawLine (0, (uint16_t)(DISP_SIZE_HEADER+ 1 + trigger_line_),
                (uint16_t)lcddev.width, (DISP_SIZE_HEADER + 1 + trigger_line_));

  return;
}

/*
 * @name   plot_refresh.
 * @brief  displays CH1 and CH2 curves.
 * @param  none.
 * @retval none.
 */
void
plot_refresh (void)
{
  uint32_t idx;
  float range_x = ((float)lcddev.width) / (float)DISPLAY_MAX_POINTS;
  float range_y = ((float)(lcddev.height - (DISP_SIZE_HEADER + DISP_SIZE_FOOTER + (2 * DISP_SIZE_SPACING)))) / 2.0;
  float points1[DISPLAY_MAX_POINTS];
  float points2[DISPLAY_MAX_POINTS];

  /* reset value */
  samples1_mean_ = 0.0;
  samples1_max_ = (-samples1_range_);
  samples1_min_ = samples1_range_;
  samples2_mean_ = 0.0;
  samples2_max_ = (-samples2_range_);
  samples2_min_ = samples2_range_;

  /* compute display mapping of signal samples */
  for (idx=0; idx<(DISPLAY_MAX_POINTS-1); idx++)
    {
      points1[idx] = (range_y + (float)(DISP_SIZE_HEADER + DISP_SIZE_SPACING))
                      - ((samples1_[idx] / samples1_range_) * range_y);
      samples1_mean_ += samples1_[idx] / ((float)DISPLAY_MAX_POINTS);
      if (samples1_[idx] > samples1_max_)
        {
          samples1_max_ = samples1_[idx];
        }
      else if (samples1_[idx] < samples1_min_)
        {
          samples1_min_ = samples1_[idx];
        }

      points2[idx] = (range_y + (float)(DISP_SIZE_HEADER + DISP_SIZE_SPACING))
                     - ((samples2_[idx] / samples2_range_) * range_y);
      samples2_mean_ += samples2_[idx] / ((float)DISPLAY_MAX_POINTS);
      if (samples2_[idx] > samples2_max_)
        {
          samples2_max_ = samples2_[idx];
        }
      else if (samples2_[idx] < samples2_min_)
        {
          samples2_min_ = samples2_[idx];
        }
    }

  /* print each curve */
  POINT_COLOR = DISP_COLOR_CH1;
  for (idx=0; idx<(DISPLAY_MAX_POINTS - 2); idx += 1)
    {
      LCD_DrawLine ((uint16_t)(((float)idx) * range_x),(uint16_t)points1[idx],
                    (uint16_t)(((float)(idx + 1)) * range_x), (uint16_t)points1[idx+1]);
    }

  POINT_COLOR = DISP_COLOR_CH2;
  for (idx=0; idx<(DISPLAY_MAX_POINTS - 2); idx += 1)
    {
      LCD_DrawLine ((uint16_t)(((float)idx) * range_x), (uint16_t)points2[idx],
                    (uint16_t)(((float)(idx + 1)) * range_x), (uint16_t)points2[idx+1]);
    }

  return;
}

/*
 * @name   main.
 * @brief  oscilloscope function: samples input signal and displays on LCD.
 * @param  none.
 * @retval none.
 */
int
main (void)
{
  uint32_t idx = 0;

  /* configure clocks */
  sysclk_Config ();

  /* configure interfaces */
  LCD_Init ();
  plot_init ();
  BSP_LED_Init (LED4);

  /* application set-up */
  sampler1_init ();
  sampler2_init ();

  millisperdiv = DISP_MILLISPERDIV_1MS;

  samples1_delay = ((1000000.0 / ((float)samples1_freq)) / DISPLAY_MAX_POINTS) - 2;
  samples2_delay = ((1000000.0 / ((float)samples2_freq)) / DISPLAY_MAX_POINTS) - 2;

  sampler1_setrange (1);
  sampler2_setrange (1);

  trigger_line_ = 100;
  trigger_line_ = (int)trigger_line_ % DISP_SIZE_PLOT;
  plot_setheader ();
  plot_setfooter ();

  /* test data set */
//  for (idx = 0; idx<DISPLAY_MAX_POINTS; idx++)
//    {
//      samples1_[idx] = sinf(((float)idx / (float)DISPLAY_MAX_POINTS) * (2*3.14)) * samples1_range_;
//      samples2_[idx] = cosf(((float)idx / (float)DISPLAY_MAX_POINTS) * (2*3.14)) * samples2_range_;
//    }

  disp_timer_init ();
  delay_timer_init ();
//  serial_init ();

  while (1)
    {
      /* wait for display refresh to finish */
      if (refresh_pending == 0)
        {
          /* sample input signal */
          for (idx = 0; idx<DISPLAY_MAX_POINTS; idx++)
            {
              samples1_[idx] = sampler1_getval ();
//              delay_us (samples1_delay);
              samples2_[idx] = sampler2_getval ();
//              delay_us (samples2_delay);
//              serial_send (samples1_[idx]);
//              serial_send (samples2_[idx]);
            }

          /* command display refresh */
          refresh_pending = 1;
        }

      HAL_Delay (1000);
    }

  return (-1);     // should not get here
}
