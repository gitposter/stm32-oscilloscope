/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @author  Ac6
  * @version V1.0
  * @date    02-Feb-2015
  * @brief   Default Interrupt Service Routines.
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#ifdef USE_RTOS_SYSTICK
#include <cmsis_os.h>
#endif
#include "stm32f4xx_it.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/*
 * @brief maximum samples in an array
 */
#define DISPLAY_MAX_POINTS             1024


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/*
 * @brief display timer
 */
extern TIM_HandleTypeDef disp_tim_;            // LCD display refresh
extern TIM_HandleTypeDef delay_tim_;           // delay counter

/*
 * @brief microseconds counter;
 */
extern uint64_t micros_;

/*
 * @brief display setting
 */
uint32_t refresh_pending ;


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*
 * @name   plot_setgrid.
 * @brief  displays the grid in the middle..
 * @param  none.
 * @retval none.
 */
extern void
plot_setgrid (void);

/*
 * @name   plot_refresh.
 * @brief  displays CH1 and CH2 curves.
 * @param  none.
 * @retval none.
 */
extern void
plot_refresh ();

/*
 * @name   plot_setfooter.
 * @brief  updates display footer.
 * @param  none.
 * @retval none.
 */
void
plot_setfooter (void);


/******************************************************************************/
/*            	  	    Processor Exceptions Handlers                         */
/******************************************************************************/
/**
  * @brief  This function handles SysTick Handler, but only if no RTOS defines it.
  * @param  None
  * @retval None
  */
void
SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
#ifdef USE_RTOS_SYSTICK
	osSystickHandler();
#endif
}

void
TIM2_IRQHandler()
{
  HAL_TIM_IRQHandler(&disp_tim_);

  HAL_GPIO_EXTI_IRQHandler();
}

/*
 * @brief
 */
void
TIM3_IRQHandler()
{
  HAL_TIM_IRQHandler(&delay_tim_);
}

/*
 * @name   HAL_TIM_PeriodElapsedCallback.
 * @brief  refreshes the display with new samples.
 * @param  none.
 * @retval none.
 */
void
HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == delay_tim_.Instance)
    {
    }
  else if ((htim->Instance == disp_tim_.Instance) && (refresh_pending == 1))
    {
      /* refresh display */
	  plot_setgrid ();
      plot_refresh ();
      plot_setfooter ();

      HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
      refresh_pending = 0;
    }
}
