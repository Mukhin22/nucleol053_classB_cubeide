/**
  ******************************************************************************
  * @file    main.c 
  * @author  MCD Application Team
  * @version V2.2.0
  * @date    19-Jun-2017
  * @brief   Main program body
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

/** @addtogroup STM32L0xx_IEC60335_Example
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#if defined (STL_EVAL_MODE) || defined(STL_VERBOSE)
  #define MESSAGE1   " STM32L0xx Cortex-M0+"
  #ifdef __IAR_SYSTEMS_ICC__  /* IAR Compiler */
  #define MESSAGE2   " IEC60335 test @IARc "
  #endif /* __IAR_SYSTEMS_ICC__ */
  #ifdef __CC_ARM             /* KEIL Compiler */
  #define MESSAGE2   " IEC60335 test @ARMc "
  #endif /* __CC_ARM */
  #ifdef __GNUC__             /* GCC Compiler */
  #define MESSAGE2   " IEC60335 test @GCCc "
  #endif /* __GNUC__ */
#endif /* STL_EVAL_MODE */
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#if defined(STL_EVAL_MODE) || defined(STL_VERBOSE)
  UART_HandleTypeDef UartHandle;
#endif /* STL_EVAL_MODE */

#if defined(STL_EVAL_LCD)
  char MyString[6];
  uint32_t MyRAMCounter;
  uint32_t MyFLASHCounter;
#endif /* STL_EVAL_LCD */

/* Private function prototypes -----------------------------------------------*/
static void NVIC_Configuration(void);

#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int16_t __io_putchar(int16_t ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
/* int32_t fputc(int32_t ch, FILE *f) */
#endif /* __GNUC__ */
	PUTCHAR_PROTOTYPE;

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Main program.
  * @param  : None
  * @retval : None
  */
int32_t main(void)
{
  /* Configure the System clock to have a system clock = 32 MHz */
  SystemClock_Config();
  
  /* STM32L0xx HAL library initialization:
       - Configure the Flash prefetch, Flash preread and Buffer caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Low Level Initialization
     */
  HAL_Init();
  
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_16);
    
  #if defined (STL_VERBOSE)
    /* Verbose messages ------------------------------------------------------*/
    USART_Configuration();
    printf("\n\r %s\n\r", MESSAGE1);
    printf(" %s\n\r", MESSAGE2);
    printf(" ... main routine starts ...\r\n");
  #endif /* STL_VERBOSE */

  #if defined (STL_EVAL_MODE)
    /* Evaluation board control ----------------------------------------------*/
    Eval_Board_HW_Init();
  #endif /* STL_EVAL_MODE */
  /* NVIC configuration ------------------------------------------------------*/
  NVIC_Configuration();

  /* if you debug TIM21 it is helpful to uncomment next lines */
  /*
  __DBGMCU_CLK_ENABLE();
  __HAL_FREEZE_TIM21_DBGMCU();
  */
  
  /* Self test routines initialization ---------------------------------------*/
  #if defined(STL_EVAL_MODE)
    /* if you debug TIM21 it is helpful to uncomment next lines */
    /*
    __DBGMCU_CLK_ENABLE();
    __HAL_FREEZE_TIM21_DBGMCU();
    */  
    BSP_LED_On(LED_ERR);
  #endif  /* STL_EVAL_MODE */
  /* -------------------------------------------------------------------------*/
  /* This is where the main self-test routines are initialized */        
  STL_InitRunTimeChecks();
  /* -------------------------------------------------------------------------*/
  #if defined STL_EVAL_MODE
    BSP_LED_Off(LED_ERR);
  #endif  /* STL_EVAL_MODE */
    
  /* Add your application initialization code here  */

  /* Infinite loop */
  while (1)
  {
    /* -----------------------------------------------------------------------*/
    /* This is where the main self-test routines are executed */
    STL_DoRunTimeChecks();
    /* -----------------------------------------------------------------------*/    

   }
}

/* -------------------------------------------------------------------------*/
/**
  * @brief  NVIC configuration
  * @param :  None
  * @retval : None
  */
static void NVIC_Configuration(void)
{  
  HAL_SYSTICK_Config(SystemCoreClock / 1000);
}

/* -------------------------------------------------------------------------*/
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 32000000
  *            HCLK(Hz)                       = 32000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            PLL_MUL                        = 8
  *            PLL_DIV                        = 2
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale 1 mode
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  
  /* Enable Power Control clock */
  __PWR_CLK_ENABLE();
  
  /* The voltage scaling  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

#ifdef HSE_CLOCK_APPLIED    
  /* Enable HSE Oscillator */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_8;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
#else
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
#endif
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    #ifdef STL_VERBOSE_POR
      printf("PLL Osc config failure\n\r");
    #endif  /* STL_VERBOSE_POR */
    FailSafePOR();
  }
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, MAX_FLASH_LATENCY) != HAL_OK)
  {
    #ifdef STL_VERBOSE_POR
      printf("PLL clock config failure\n\r");
    #endif  /* STL_VERBOSE_POR */
    FailSafePOR();
  }
}
/* ---------------------------------------------------------------------------*/
/**
  * @brief  Startup Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 32000000
  *            HCLK(Hz)                       = 32000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            PLL_MUL                        = 4
  *            PLL_DIV                        = 2
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale 1 mode
  *            Flash Latency(WS)              = 1
  * @param  None
  * @retval None
  */

void StartUpClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  
  /* Enable Power Control clock */
  __PWR_CLK_ENABLE();
  
  /* The voltage scaling  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Enable HSI Oscillator */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.HSICalibrationValue = 0x10; /* !!! default HSI trimming value !!! */
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    #ifdef STL_VERBOSE_POR
      printf("PLL clock config failure\n\r");
    #endif  /* STL_VERBOSE_POR */
    FailSafePOR();
  }
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    #ifdef STL_VERBOSE_POR
      printf("PLL clock switch failure\n\r");
    #endif  /* STL_VERBOSE_POR */
    FailSafePOR();
  }
}

#if defined STL_EVAL_MODE
/* -------------------------------------------------------------------------*/
/**
  * @brief  Initialization of evaluation board HW
  * @param :  None
  * @retval : None
  */
void Eval_Board_HW_Init(void)
{
  /* init LED and monitoring signals on evaluation board -------------------*/
    BSP_LED_Init(LED_VLM);
    BSP_LED_Init(LED_NVM);
    BSP_LED_Init(LED_ERR);
}
/* -------------------------------------------------------------------------*/
/**
  * @brief  Initialization of evaluation board HW
  * @param :  None
  * @retval : None
  */
void User_signal_Init(uint16_t gpio_pin)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /* Enable the GPIO_LED Clock */
  __GPIOA_CLK_ENABLE();

  /* Configure the GPIO_LED pin */
  GPIO_InitStruct.Pin = gpio_pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
#endif /* STL_EVAL_MODE */

#if defined(STL_VERBOSE) || defined(STL_VERBOSE_POR)
/* -------------------------------------------------------------------------*/
/**
  * @brief  Configure the UART peripheral 
  * @param  None
  * @retval None
  */
void USART_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  
  /* Enable GPIO clock */
  __GPIOA_CLK_ENABLE();
    
  /* Configure USART Tx as alternate function */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Alternate = GPIO_AF4_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  /* Configure USART Rx as alternate function */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Alternate = GPIO_AF4_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
   /* Start high speed internal (HSI) oscillator */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT; /* !!! 0x10 - default HSI trimming value !!! */
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

   /* HSI feeds USART2 */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2; 
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_HSI;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
  
  /* Enable USART clock */
  __USART2_CLK_ENABLE();
  
  /* UART configuration */
  UartHandle.Instance = USART2;
  UartHandle.Init.BaudRate   = 115200;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits   = UART_STOPBITS_1;
  UartHandle.Init.Parity     = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode       = UART_MODE_TX;
  __HAL_UART_RESET_HANDLE_STATE(&UartHandle);
  HAL_UART_Init(&UartHandle); 
}
/* -------------------------------------------------------------------------*/
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, 10); /* 0xFFFF */

  return ch;
}
#ifdef __GNUC__
/* -------------------------------------------------------------------------*/
/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  None
 * @retval None
 */
int16_t _write(int16_t file, int8_t *ptr, int16_t len) {
	int16_t DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++) {
		__io_putchar(*ptr++);
	}
	return len;
}
#endif /* __GNUC__ */
#endif /* STL_VERBOSE */

#ifdef __CC_ARM   /* KEIL Compiler */
__weak void HAL_UARTEx_WakeupCallback(UART_HandleTypeDef *h)
{
}
#endif

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval : None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
