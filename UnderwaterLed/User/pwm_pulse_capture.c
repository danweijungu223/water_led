#include "pwm_pulse_capture.h"
#include "usart.h"

//定时器一次滴答为 1/4 us
//重新装载计时器为 40000 即10ms重装载一次

#define LED_MAX_PWM 1280
#define PB0_READ ((GPIOB->IDR & LL_GPIO_PIN_0) != 0x00u)

static volatile uint8_t input_mode = 0; //0无效 1脉宽 2开关
static volatile uint8_t mode_to_pwm_cnt = 0;
static volatile uint8_t mode2_to_mode0_cnt = 0;
static volatile uint8_t mode1_to_mode0_cnt = 0;

static volatile uint8_t pulse_width_update = 0;
static volatile uint8_t this_pulse_width_invalid = 0;
static volatile uint32_t pulse_width = 0; //单位us
static volatile uint32_t _tmp_pulse_width = 1100; //单位us

static volatile uint8_t rising_edge_occurred = 0; //上升沿发生标志位
static volatile uint32_t rising_edge_cnt_num = 0; //上升沿时计数器数值
static volatile uint32_t falling_edge_cnt_num = 0; //下降沿时计数器数值
static volatile uint32_t high_timer_overflow_times = 0;
static volatile uint32_t low_timer_overflow_times = 0;

static volatile uint32_t led_brightness_switch = 0; //开关模式LED亮度（用于计算实际亮度0~64）

void Occur_Edge_Trigger(void)
{
	if( 0 == rising_edge_occurred ) //如果未发生上升沿，则次次边沿检测到了上升沿
	{
		//记录发生上升沿时的计数器数值
		rising_edge_cnt_num = LL_TIM_IC_GetCaptureCH3(TIM3);
		
		//切换为下降沿触发
		LL_TIM_IC_SetPolarity(TIM3, LL_TIM_CHANNEL_CH3, LL_TIM_IC_POLARITY_FALLING);
		
		//将 上升沿发生标志位 设为1
		rising_edge_occurred = 1;
		
		//清除定时器超时次数
		low_timer_overflow_times = 0;
		high_timer_overflow_times = 0;
	}
	
	else //此次边沿是下降沿，此时可确定的知道上一个高电平时间（脉宽）
	{
		//记录发生上升沿时的计数器数值
		falling_edge_cnt_num = LL_TIM_IC_GetCaptureCH3(TIM3);
		
		//计算高电平时间（脉宽）
		if(0 == this_pulse_width_invalid)
		{
			pulse_width = ( (10000*high_timer_overflow_times) + ( (int32_t)(falling_edge_cnt_num - rising_edge_cnt_num) / 4 ) );
		}
		else
		{
			this_pulse_width_invalid = 0;
		}
		
		//切换为上升沿触发
		LL_TIM_IC_SetPolarity(TIM3, LL_TIM_CHANNEL_CH3, LL_TIM_IC_POLARITY_RISING);
		
		//重置 上升沿发生标志位
		rising_edge_occurred = 0;
		
		//清除定时器超时次数
		high_timer_overflow_times = 0;
		low_timer_overflow_times = 0;
		
		/* 模式2时， 高电平时间<10ms，切为模式0 BEGIN */
		if(2 == input_mode && pulse_width < 10000)
		{
			mode2_to_mode0_cnt++;
			if(mode2_to_mode0_cnt>10)
			{
				mode2_to_mode0_cnt = 0;
				input_mode = 0;
				led_brightness_switch = 0;
			}
		}
		else
		{
			mode2_to_mode0_cnt = 0;
		}
		/* 模式2时， 高电平时间<10ms，切为模式0 END */
		
		/* 采集到10次有效启动(1100)脉宽，将模式切换至模式1 BEGIN */
		if(input_mode != 1)
		{
			if( (pulse_width >1090) && (pulse_width <1135) )
			{
				mode_to_pwm_cnt ++;
				if(mode_to_pwm_cnt > 9)
				{
					input_mode = 1;
					led_brightness_switch = 0;
					mode_to_pwm_cnt = 0;
				}
			}
			else
			{
				mode_to_pwm_cnt = 0;
			}
		}
		/* 采集到10次有效启动脉宽，将模式切换至模式1 END */
		
		/* 模式1时无效脉宽，模式重置 BEGIN */
		if( (input_mode == 1) && (pulse_width <1000 || pulse_width >2000) )
		{
			mode1_to_mode0_cnt++;
			if(mode1_to_mode0_cnt > 20)
			{
				mode1_to_mode0_cnt = 0;
				input_mode = 0;
				led_brightness_switch = 0;
			}
		}
		else
		{
			mode1_to_mode0_cnt = 0;
		}
		/* 模式1时无效脉宽，模式重置 END */
		
		//设置 脉宽更新标志位
		pulse_width_update = 1;
		
	}
	
}


void Occur_Timer_Update(void)
{
	if( PB0_READ ) //如果是高电平，应该已经发生了上升沿
	{
		
		if(1 == rising_edge_occurred) //正常边沿（检测到高电平，已经发生上升沿）
		{
			//记录定时器reload
			high_timer_overflow_times++;
			
			/* 模式2时，检测到高电平将提高亮度 BEGIN */
			if(2 == input_mode)
			{
				if(led_brightness_switch < 64)
				{
					led_brightness_switch++;
				}
			}
			/* 模式2时，检测到高电平将提高亮度 END */
			
			/* 模式1时，检测到脉宽过长，将切换至模式0 BEGIN */
			else if( input_mode == 1 )
			{
				if(high_timer_overflow_times > 1)
				{
					input_mode = 0;
				}
			}
			/* 模式1时，检测到脉宽过长，将切换至模式0 END */
			
		}
		
		else //异常边沿（已经发生下降沿（未检测到上降沿），却还为高电平）
		{
			
			//无法记录发生上升沿时的计数器数值
			//无上升沿时间，此次脉宽无效
			this_pulse_width_invalid = 1;
			
			//切换为下降沿触发
			LL_TIM_IC_SetPolarity(TIM3, LL_TIM_CHANNEL_CH3, LL_TIM_IC_POLARITY_FALLING);
			
			//将 上升沿发生标志位 设为1
			rising_edge_occurred = 1;
			
			//清除定时器超时次数
			low_timer_overflow_times = 0;
			high_timer_overflow_times = 0;
			
		}
		
	}
	
	else //如果是低电平，应该已经发生了下降沿
	{
		
		if(0 == rising_edge_occurred) //边沿正常（检测到低电平，已经发生下降沿（未发生新的上升沿））
		{
			//记录定时器reload
			low_timer_overflow_times ++;
			
			/* 开关模式时，检测到低电平，降低亮度 BEGIN */
			if(2 == input_mode)
			{
				if(led_brightness_switch > 0)
				{
					led_brightness_switch--;
					if(led_brightness_switch > 0)
					{
						led_brightness_switch--;
					}
				}
			}
			/* 开关模式时，检测到低电平，降低亮度 END */
			
			/* 非开关模式时，检测到0.5s低电平，将切换至开关模式 BEGIN */
			if(2 != input_mode)
			{
				if(low_timer_overflow_times > 55)
				{
					input_mode = 2;
					led_brightness_switch = 0;
				}
			}
			/* 非开关模式时，检测到1s以上低电平，将切换至开关模式 END */
			
		}
		
		else //异常边沿（已经发生上升沿（未检测到下降沿），却为低电平）
		{
			
			//切换为上升沿触发
			LL_TIM_IC_SetPolarity(TIM3, LL_TIM_CHANNEL_CH3, LL_TIM_IC_POLARITY_RISING);
			
			//重置 上升沿发生标志位
			rising_edge_occurred = 0;
			
			//清除定时器超时次数
			high_timer_overflow_times = 0;
			low_timer_overflow_times = 0;
			
		}
		
	}
	
}


uint32_t Get_Pulse_Width(void)
{
	return pulse_width;
}

void Reset_Input_Mode(void)
{
	input_mode = 0;
}

uint8_t Get_Input_Mode(void)
{
	return input_mode;
}

uint32_t Get_Led_Brightness_Switch(void)
{
	return led_brightness_switch;
}

uint8_t Is_Pulse_Width_Update(void)
{
	return pulse_width_update;
}

void Reset_Pulse_Width_Update(void)
{
	pulse_width_update = 0;
}
