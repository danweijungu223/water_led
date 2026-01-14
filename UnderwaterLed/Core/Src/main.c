/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "pwm_pulse_capture.h"
#include "voltage_acquisition.h"

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	static float temp_protect = 1.0f;
	static float hi_voltage_power_limit = 1.0f;
	static uint32_t led_brightness_tmp = 0;
	static uint32_t led_brightness_tmp_cnt = 0;
	static uint32_t led_brightness_tmp_buf[25] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	static uint32_t led_brightness_tmp_add = 0;
	static uint32_t led_brightness_tmp_average = 0;
	static uint32_t led_brightness_tmp2 = 0;
	static uint32_t led_brightness_ture = 0;
	memset(led_brightness_tmp_buf, 0, 25*sizeof(uint32_t));
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM16_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	
	//PB0 PWM输入（脉宽）捕获
  TIM3->SR=0; //清除中断标志位
	LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH3);
	LL_TIM_EnableIT_CC3(TIM3);
	LL_TIM_EnableIT_UPDATE(TIM3);
	LL_TIM_SetCounter(TIM3, 0); //计数器清空
	LL_TIM_EnableCounter(TIM3);
	
	//2路ADC配置，并开启转换
	Adc1_Additional_Config();
	LL_ADC_REG_StartConversion(ADC1);
	
	//开启PA6的PWM输出
	LL_TIM_OC_SetCompareCH1(TIM16, 0);
	LL_TIM_CC_EnableChannel(TIM16, LL_TIM_CHANNEL_CH1);
	LL_TIM_EnableAllOutputs(TIM16);
	LL_TIM_EnableCounter(TIM16);
	
	//等待电压有效
VoltageProtect:
	while(0 == Is_Voltage_Valid()) { };
	
	//UVLO 设置为高电平
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		//调试时使用
		//printf("aaa : %d   ", Get_Pulse_Width());		
		//printf("bbb : %d  %d  %d\r\n", Get_Input_Mode(), led_brightness_tmp, led_brightness_ture);
		//printf("ccc : %d  %d\r\n", Get_Adc1_Data(0), Get_Adc1_Data(1));		
		
		#if 1
		//采集电压
		if(Voltage_Protect_Update())
		{
			LL_TIM_OC_SetCompareCH1(TIM16, 0);
			LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4);
			
			Reset_Input_Mode();
			memset(led_brightness_tmp_buf, 0, 25*sizeof(uint32_t));
			led_brightness_tmp_add = 0;
			led_brightness_ture = 0;
			goto VoltageProtect;
		}
		#endif
		
		#if 1
		//高电压功率限制
		hi_voltage_power_limit = High_Voltage_Protect();
		#endif
		
		#if 1
		//采集温度
		switch( Temp_Protect_Update() )
		{
			case 0:
			{
				temp_protect = 1.0f;
				break;
			}
			case 1:
			{
				temp_protect = 0.5f;
				break;
			}
			case 2:
			{
				temp_protect = 0.0f;
				Reset_Input_Mode();
				break;
			}
		}
		#endif
		
		//调光
		switch( Get_Input_Mode() )
		{
			//脉宽调光
			case 1:
			{
				if(Is_Pulse_Width_Update())
				{
					
					//亮度更新
					if( (Get_Pulse_Width() > 1000) && (Get_Pulse_Width() < 2000) )
					{
						if(Get_Pulse_Width() < 1135)
						{
							led_brightness_tmp = 0;
						}
						else if(Get_Pulse_Width() > 1890)
						{
							led_brightness_tmp = 1280;
						}
						else
						{
							led_brightness_tmp = (Get_Pulse_Width() - 1135) * 1280 / 755;
						}
					}
					
					//计算25个缓存亮度的平均值（降低杂波影响）
					led_brightness_tmp_add -= led_brightness_tmp_buf[led_brightness_tmp_cnt];
					led_brightness_tmp_buf[led_brightness_tmp_cnt] = led_brightness_tmp;
					led_brightness_tmp_add += led_brightness_tmp;
					
					led_brightness_tmp_average = led_brightness_tmp_add / 25;
					led_brightness_tmp_cnt++;
					if(led_brightness_tmp_cnt > 24)
					{
						led_brightness_tmp_cnt = 0;
					}
					
					led_brightness_tmp2 = led_brightness_tmp_average * temp_protect * hi_voltage_power_limit * 0.7;
					
					//缓慢亮暗（平滑亮度曲线，降低杂波影响）
					if(led_brightness_tmp2 < led_brightness_ture)
					{
						if( (led_brightness_ture - led_brightness_tmp2) > 5)
						{
							led_brightness_ture -= (led_brightness_ture - led_brightness_tmp2)*0.075;
						}
						
						led_brightness_ture--;
					}
					else if(led_brightness_tmp2 > led_brightness_ture)
					{
						if( (led_brightness_tmp2 - led_brightness_ture) > 5)
						{
							led_brightness_ture += (led_brightness_tmp2 - led_brightness_ture)*0.05;
						}
						led_brightness_ture++;
					}
					
					//改变占空比
					Reset_Pulse_Width_Update();
					LL_TIM_OC_SetCompareCH1(TIM16, led_brightness_ture );
					
					//调试时使用
					//printf("aaa : %d   ", Get_Pulse_Width());
					//{ printf("xxx   "); for(uint8_t i=0; i<25; i++) { printf("%d ", led_brightness_tmp_buf[i]); } printf("\r\n"); }
					//printf("yyy :    %d   \r\n", led_brightness_tmp_average);
				
				}
				break;
			}
			
			//0 1信号调光（开关模式）
			case 2:
			{
				led_brightness_tmp = Get_Led_Brightness_Switch() * 20 * temp_protect * hi_voltage_power_limit;
				
				if(led_brightness_tmp < led_brightness_ture) 
				{
					led_brightness_ture--;
				}
				else if(led_brightness_tmp > led_brightness_ture)
				{
					led_brightness_ture++;
				}
				
				memset(led_brightness_tmp_buf, 0, 25*sizeof(uint32_t));
				led_brightness_tmp_add = 0;
				
				LL_TIM_OC_SetCompareCH1(TIM16, led_brightness_ture);
				break;
			}
			
			//错误信号
			default:
			{
				memset(led_brightness_tmp_buf, 0, 25*sizeof(uint32_t));
				led_brightness_tmp_add = 0;
				
				led_brightness_ture = 0;
				LL_TIM_OC_SetCompareCH1(TIM16, 0);
				break;
			}
		}
		
		//延时
		//for(int i=0; i<1000000; i++){ };
		
		
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2)
  {
  }

  /* HSI configuration and activation */
  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }

  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_1, 8, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_Enable();
  LL_RCC_PLL_EnableDomain_SYS();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  }

  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  /* Sysclk activation on the main PLL */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  }

  /* Set APB1 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);

  LL_Init1msTick(64000000);

  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(64000000);
  LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSOURCE_SYSCLK);
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
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
