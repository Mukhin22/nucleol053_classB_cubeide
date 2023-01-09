/**
  ******************************************************************************
  * @file    stm32l0xx_it.c 
  * @author  MCD Application Team
  * @version V2.2.0
  * @date    19-Jun-2017
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32fxx_STLparam.h"
#include "stm32fxx_STLlib.h"
#include "stm32l0xx_it.h"

/** @addtogroup STM32L0xx_IEC60335_Example
  * @{
  */


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
   volatile static uint16_t tmpCC1_last;	  /* keep last TIM21/Chn1 captured value */
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0+ Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  : None
  * @retval : None
  */
void NMI_Handler(void)
{
#ifdef STL_VERBOSE
  if (__HAL_RCC_GET_IT_SOURCE(RCC_IT_CSS))
  {
    while(__HAL_USART_GET_FLAG(&UartHandle, USART_FLAG_TC) == 0)
    {   /* Wait previous transmission completion */
    }
    /* Re-configure USART baud rate to have 115200 bds with HSI clock (8MHz) */
    USART_Configuration();
    printf("\n\r Clock Source failure (Clock Security System)\n\r");
  }
  else
  {
    printf("\n\r NMI Exception\n\r");
  }
#endif /* STL_VERBOSE */

#if defined STL_EVAL_MODE
  /* LED_ERR Off for debug purposes */
  BSP_LED_Off(LED_ERR);
#endif  /* STL_EVAL_MODE */

  FailSafePOR();
}

/******************************************************************************/
/**
  * @brief  This function handles Hard Fault exception.
  * @param  : None
  * @retval : None
  */
void HardFault_Handler(void)
{
#ifdef STL_VERBOSE
  printf("\n\r Hard fault Exception \n\r");
#endif /* STL_VERBOSE */

#if defined STL_EVAL_MODE
  /* LED_ERR Off for debug purposes */
  BSP_LED_Off(LED_ERR);
#endif  /* STL_EVAL_MODE */

  FailSafePOR();
}

/******************************************************************************/
/**
  * @brief  This function handles SVCall exception.
  * @param  : None
  * @retval : None
  */
void SVC_Handler(void)
{
#ifdef STL_VERBOSE
  printf("\n\r SVCall Exception \n\r");
#endif /* STL_VERBOSE */

  FailSafePOR();
}

/******************************************************************************/
/**
  * @brief  This function handles Debug Monitor exception.
  * @param  : None
  * @retval : None
  */
void DebugMon_Handler(void)
{
#ifdef STL_VERBOSE
  printf("\n\r Debug Monitor Exception \n\r");
#endif /* STL_VERBOSE */
}

/******************************************************************************/
/**
  * @brief  This function handles PendSVC exception.
  * @param  : None
  * @retval : None
  */
void PendSV_Handler(void)
{
#ifdef STL_VERBOSE
  printf("\n\r PendSVC Exception \n\r");
#endif /* STL_VERBOSE */

  FailSafePOR();
}

/******************************************************************************/
/**
  * @brief  This function handles SysTick Handler.
  * @param  : None
  * @retval : None
  */
void SysTick_Handler(void)
{
  HAL_IncTick();
    
  /* Verify TickCounter integrity */
  if ((TickCounter ^ TickCounterInv) == 0xFFFFFFFFuL)
  {
    TickCounter++;
    TickCounterInv = ~TickCounter;

    if (TickCounter >= SYSTICK_10ms_TB)
    {
      uint32_t RamTestResult;

      #if defined STL_EVAL_MODE
        /* Toggle LED_VLM for debug purposes */
        BSP_LED_Toggle(LED_VLM);
      #endif  /* STL_EVAL_MODE */

      /* Reset timebase counter */
      TickCounter = 0u;
      TickCounterInv = 0xFFFFFFFFuL;

      /* Set Flag read in main loop */
      TimeBaseFlag = 0xAAAAAAAAuL;
      TimeBaseFlagInv = 0x55555555uL;

      ISRCtrlFlowCnt += RAM_MARCHC_ISR_CALLER;
			__disable_irq();
      RamTestResult = STL_TranspMarch();
			__enable_irq();
      ISRCtrlFlowCntInv -= RAM_MARCHC_ISR_CALLER;

      switch ( RamTestResult )
      {
        case TEST_RUNNING:
          break;
        case TEST_OK:
          #ifdef STL_VERBOSE
          /* avoid any long string output here in the interrupt, '#' marks ram test completed ok */
            #ifndef __GNUC__
              putchar((int16_t)'#');
            #else
              __io_putchar((int16_t)'#');
            #endif /* __GNUC__ */
          #endif  /* STL_VERBOSE */
          #ifdef STL_EVAL_MODE
            /* Toggle LED_VLM for debug purposes */
            BSP_LED_Toggle(LED_VLM);
          #endif /* STL_EVAL_MODE */
          #if defined(STL_EVAL_LCD)
            ++MyRAMCounter;
          #endif /* STL_EVAL_LCD */
          break;
        case TEST_FAILURE:
        case CLASS_B_DATA_FAIL:
        default:
          #ifdef STL_VERBOSE
            printf("\n\r >>>>>>>>>>>>>>>>>>>  RAM Error (March C- Run-time check)\n\r");
          #endif  /* STL_VERBOSE */
          FailSafePOR();
          break;
      } /* End of the switch */

      /* Do we reached the end of RAM test? */
      /* Verify 1st ISRCtrlFlowCnt integrity */
      if ((ISRCtrlFlowCnt ^ ISRCtrlFlowCntInv) == 0xFFFFFFFFuL)
      {
        if (RamTestResult == TEST_OK)
        {
  /* ==============================================================================*/
  /* MISRA violation of rule 17.4 - pointer arithmetic is used to check RAM test control flow */
	#ifdef __IAR_SYSTEMS_ICC__  /* IAR Compiler */
		#pragma diag_suppress=Pm088
	#endif /* __IAR_SYSTEMS_ICC__ */
          if (ISRCtrlFlowCnt != RAM_TEST_COMPLETED)
	#ifdef __IAR_SYSTEMS_ICC__  /* IAR Compiler */
		#pragma diag_default=Pm088
	#endif /* __IAR_SYSTEMS_ICC__ */
  /* ==============================================================================*/
          {
          #ifdef STL_VERBOSE
            printf("\n\r Control Flow error (RAM test) \n\r");
          #endif  /* STL_VERBOSE */
          FailSafePOR();
          }
          else  /* Full RAM was scanned */
          {
            ISRCtrlFlowCnt = 0u;
            ISRCtrlFlowCntInv = 0xFFFFFFFFuL;
          }
        } /* End of RAM completed if*/
      } /* End of control flow monitoring */
      else
      {
      #ifdef STL_VERBOSE
        printf("\n\r Control Flow error in ISR \n\r");
      #endif  /* STL_VERBOSE */
      FailSafePOR();
      }
      #if defined STL_EVAL_MODE
        /* Toggle LED_VLM for debug purposes */
        BSP_LED_Toggle(LED_VLM);
      #endif  /* STL_EVAL_MODE */
    } /* End of the 10 ms timebase interrupt */
  }  
}

/******************************************************************************/
/**
  * @brief  This function handles TIM21 global interrupt request.
  * @param  : None
  * @retval : None
  */
void TIM21_IRQHandler(void)
{
  uint16_t tmpCC1_last_cpy;
   
  if ((TIM21->SR & TIM_SR_CC1IF) != 0u )
  {
    /* store previous captured value */
    tmpCC1_last_cpy = tmpCC1_last; 
    /* get currently captured value */
    tmpCC1_last = (uint16_t)(TIM21->CCR1);
    /* The CC4IF flag is already cleared here be reading CCR4 register */

    /* overight results only in case the data is required */
    if (LSIPeriodFlag == 0u)
    {
      /* take correct measurement only */
      if ((TIM21->SR & TIM_SR_CC1OF) == 0u)
      {
        /* Compute period length */
        PeriodValue = ((uint32_t)(tmpCC1_last) - (uint32_t)(tmpCC1_last_cpy)) & 0xFFFFuL;
        PeriodValueInv = ~PeriodValue;
              
        /* Set Flag tested at main loop */
        LSIPeriodFlag = 0xAAAAAAAAuL;
        LSIPeriodFlagInv = 0x55555555uL;
      }
      else
      {
        /* ignore computation in case of IC overflow */
        TIM21->SR &= (uint16_t)(~TIM_SR_CC1OF);
      }
    }
    /* ignore computation in case data is not required */
  }
}

/******************************************************************************/
/**
  * @brief Configure TIM21 to measure LSI period
  * @param  : None
  * @retval : ErrorStatus = (ERROR, SUCCESS)
  */
ErrorStatus STL_InitClock_Xcross_Measurement(void)
{
  ErrorStatus result = SUCCESS;
  TIM_HandleTypeDef  tim_capture_handle;
  TIM_IC_InitTypeDef tim_input_config;
  
  /*## Enable peripherals and GPIO Clocks ####################################*/
  /* TIMx Peripheral clock enable */
  __TIM21_CLK_ENABLE();
  
  /*## Configure the NVIC for TIMx ###########################################*/
  HAL_NVIC_SetPriority(TIM21_IRQn, 0u, 0u);
  
  /* Enable the TIM21 global Interrupt */
  HAL_NVIC_EnableIRQ(TIM21_IRQn);
  
  /* TIM21 configuration: Input Capture mode ---------------------
  The LSI oscillator is connected to TIM21 CH1.
  The Rising edge is used as active edge, ICC input divided by 8
  The TIM21 CCR1 is used to compute the frequency value. 
  ------------------------------------------------------------ */
  tim_capture_handle.Instance = TIM21;
  tim_capture_handle.Init.Prescaler         = 0u; 
  tim_capture_handle.Init.CounterMode       = TIM_COUNTERMODE_UP;  
  tim_capture_handle.Init.Period            = 0xFFFFFFFFul; 
  tim_capture_handle.Init.ClockDivision     = 0u;     
  /* define internal HAL driver status here as handle structure is defined locally */
  __HAL_RESET_HANDLE_STATE(&tim_capture_handle);
  if(HAL_TIM_IC_Init(&tim_capture_handle) != HAL_OK)
  {
    /* Initialization Error */
    result = ERROR;
  }
  /* Connect internally the TIM21_CH1 Input Capture to the LSI clock output */
  HAL_TIMEx_RemapConfig(&tim_capture_handle, TIM21_TI1_LSI);
  
  /* Configure the TIM21 Input Capture of channel 1 */
  tim_input_config.ICPolarity  = TIM_ICPOLARITY_RISING;
  tim_input_config.ICSelection = TIM_ICSELECTION_DIRECTTI;
  tim_input_config.ICPrescaler = TIM_ICPSC_DIV8;
  tim_input_config.ICFilter    = 0u;
  if(HAL_TIM_IC_ConfigChannel(&tim_capture_handle, &tim_input_config, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Initialization Error */
    result = ERROR;
  }
  
  /* Reset the flags */
  tim_capture_handle.Instance->SR = 0u;
  LSIPeriodFlag = 0u;
  
  /* Start the TIM Input Capture measurement in interrupt mode */
  if(HAL_TIM_IC_Start_IT(&tim_capture_handle, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Initialization Error */
    result = ERROR;
  }
  return(result);
}

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
