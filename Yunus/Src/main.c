
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

struct __FILE
{
	int dummy;
};

FILE __stdout;

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

uint32_t tim2counter01 = 0;
uint32_t tim2counter02 = 0;
uint32_t tim2counter03 = 0;
uint32_t tim2counter04 = 0;
uint8_t extiTick01 = 0;

uint8_t t2tick_flag01 = 0;
char strText[40];

__IO uint32_t pulsecounter01 = 0;
__IO uint32_t totalcounter01 = 0;
__IO uint32_t totalcounter02 = 0;
uint32_t pseq01 = 16;
uint32_t pseq02 = 11;
uint32_t pseq03 = 5;
uint32_t seq01 = 0;
uint32_t seq02 = 0;
uint32_t seq03 = 0;

uint32_t outpulseqntt = 74;
uint32_t inout_pulse_ratio = 0;
uint32_t out_pulse_width = 0;
uint32_t out_pulse_half_width = 0;
uint32_t out_pulse_counter01 = 0; //To control pulse width
uint32_t out_pulse_counter02 = 0; //To control output pulse quantity

uint8_t op_flag = 0;

uint8_t start_flag01 = 0;
uint8_t start_flag02 = 0;

uint32_t inpulsewidthacc = 0;
uint32_t inpwarr[10];
uint32_t ipwidx = 0;

uint32_t pulse_on_counter = 0;

__IO uint32_t cycle_counter = 0;

uint32_t ppdist01 = 0;
uint32_t ppdist02 = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{
		if(tim2counter01 < 1000UL) //320000)
		{
			tim2counter01++;
		}
		else
		{
			tim2counter01 = 0;
			t2tick_flag01 = 1;
			
			if(totalcounter02 >= outpulseqntt)
			{
				inout_pulse_ratio = (((totalcounter02 * 1000UL) - 1400UL) / outpulseqntt);
			}
			else
			{
				inout_pulse_ratio = (((totalcounter02  * 1000UL) - 1400UL) / outpulseqntt);
			}
			
			out_pulse_width = ((tim2counter03 * inout_pulse_ratio) / 1000UL);
			
			if(out_pulse_width > 1)
			{
				out_pulse_half_width = out_pulse_width / 2;
			}
			
		}
		
		
		
		if(pulse_on_counter > 0)
		{
			pulse_on_counter--;
		}
		else
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
			
			if(start_flag01 == 1)
			{
				out_pulse_counter02 = (outpulseqntt * 2UL);
				out_pulse_counter01 = out_pulse_half_width;
				start_flag01 = 0;
			}
		}
		
		
		
		
		if(out_pulse_half_width > 0)
		{
			if(out_pulse_counter01 < out_pulse_half_width)
			{
				out_pulse_counter01++;
			}
			else
			{
				out_pulse_counter01 = 0;
				
				if(out_pulse_counter02 > 0)
				{
					HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
					out_pulse_counter02--;
				}
				else
				{
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
				}
			}
		}
		
		
		if(tim2counter02 < 1000000)
		{
			tim2counter02++;
		}
	}
}

uint8_t round_flag = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_0)
	{
		tim2counter04 = tim2counter02;
		tim2counter02 = 0;
		pulsecounter01++;
		
		if(ipwidx < 10)
		{
			
			inpulsewidthacc = inpwarr[0] + inpwarr[1] + inpwarr[2] + inpwarr[3] + inpwarr[4] + inpwarr[5] + inpwarr[6] + inpwarr[7] + inpwarr[8] + inpwarr[9];			
			tim2counter03 = (inpulsewidthacc / 10);
			
			inpwarr[ipwidx] = tim2counter04;

			ipwidx++;
		}
		else
		{
			ipwidx = 0;
			round_flag = 1;
			//inpulsewidthacc = 0;
		}
		
		
	}
	if(GPIO_Pin == GPIO_PIN_1)
	{
		seq03 = pulsecounter01;
		
		totalcounter01 += pulsecounter01;
		//ppdist02 = ppdist01;
		//ppdist01 = 0;
		
		//if((seq01 == pseq01) || (seq02 == pseq02) || (seq03 == pseq03))
		//{
		//}
		
		if(seq03 == pseq01)
		{
			if(totalcounter01 == (totalcounter02 - (pseq02 + pseq03)))
			{
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
				pulse_on_counter = out_pulse_half_width;
				
				start_flag01 = 1;
				//out_pulse_counter02 = (outpulseqntt * 2UL);
			}
		}
		else if(seq03 == pseq02)
		{
			if(totalcounter01 == (totalcounter02 - pseq03))
			{
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
				pulse_on_counter = out_pulse_half_width;
			}
		}
		else if(seq03 == pseq03)
		{
			if(totalcounter01 == totalcounter02)
			{
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
				pulse_on_counter = out_pulse_half_width;
			}
		}
		
		
		if((seq01 == pseq01) && (seq02 == pseq02) && (seq03 == pseq03))
		{
			totalcounter02 = totalcounter01; //totalcounter02 : pulse per cycle matched.
			totalcounter01 = 0;
			
			if(cycle_counter < 100000)
			{
				cycle_counter++;
			}
			else
			{
				cycle_counter = 0;
			}
		}
		
		seq01 = seq02;
		seq02 = seq03;
		
		pulsecounter01 = 0;
	}
}

PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  HAL_UART_Transmit(&huart1, (uint8_t*) &ch, 1, 0x0fff);

  /* Loop until the end of transmission */
  //while (USART_GetFlagStatus(Port_USART, USART_FLAG_TC) == RESET)
  //{}

  return ch;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	
	uint32_t lambda02 = 0;
	uint32_t lambda01 = 0;

	HAL_TIM_Base_Start_IT(&htim2);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
		//HAL_Delay(50);

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		if(t2tick_flag01 == 1)
		{
			t2tick_flag01 = 0;
			//sprintf(strText, "pulse_width : %05d", tim2counter03);
			//printf("%s ", strText);
			
			//from calc assuming that timer2 resolution is 3.949uS . 
			
			lambda02 = (1000000000UL / (3949UL * tim2counter03));
			//lambda01 = (lambda02/1000);
			sprintf(strText, "freq : %05d ", lambda02);
			printf("%s ", strText);
			
			////sprintf(strText, "seq : %03d %03d %03d ", seq01, seq02, seq03);
			//sprintf(strText, "cycles : %06d", cycle_counter);
			//printf("%s ", strText);
			
			//sprintf(strText, "pulse/cycle %04d \r\n", totalcounter02);
			//printf("%s ", strText);
			
			//sprintf(strText, "pulse dist %05d\r\n", ppdist02);
			//printf("%s ", strText);
			
			/*	We know that our Timer2 tick time is 3.949 uS = 3949 pico seconds. Total pulse per circle is totalcounter02 . 
					Each pulse width is (tim2counter03 * 3949) pico seconds. So full circle needs 
					( totalcounter02 * tim2counter03 * 3949 ) picoseconds. We need to deliver outpulseqntt amount of pulses within the 
					circle time. One of the factor is this circle time is real time variable depends on input pulse width. 
					So we need to find out a relational factor between input pulse width and output pulse width.
					Lets assume that our time unit is each tick time of Timer2 ( i.e. 3.949 uS = 3949 picoseconds ) . 
					Pulse width is tim2counter03 variable.
			*/
			
			/*
			if(totalcounter02 > outpulseqntt)
			{
				inout_pulse_ratio = (((totalcounter02 - 1) * 1000UL) / outpulseqntt);
			}
			else
			{
				inout_pulse_ratio = 1000;
			}
			
			out_pulse_width = ((tim2counter03 * inout_pulse_ratio) / 1000UL);
			
			if(out_pulse_width > 1)
			{
				out_pulse_half_width = out_pulse_width / 2;
			}
			*/
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 255;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 70;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 100;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
