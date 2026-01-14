#include "voltage_acquisition.h"
#include "usart.h"

//VSN PA0
//TSN PA1

#define VSN_NUM 1
#define TSN_NUM 0

#define VSN_DATA adc_data[VSN_NUM]
#define TSN_DATA adc_data[TSN_NUM]

#define INPUT_V ( (float) ((VSN_DATA / 4095.0f * 3.3f) / 5.1f * (120.0f + 5.1f)) )

static volatile uint16_t adc_data[2];

void Adc1_Additional_Config(void)
{
	/* ADC1 的 DMA 配置 */
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 2);
	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA));
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)&adc_data[0]);
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
	
	/* ADC1 配置 */
	LL_ADC_StartCalibration(ADC1);
	while(LL_ADC_IsCalibrationOnGoing(ADC1)){ };
	LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);
	LL_ADC_Enable(ADC1);
}

uint8_t Is_Voltage_Valid(void)
{
	static uint8_t voltage_valid_cnt = 0;
	if(INPUT_V < 9.9f || INPUT_V > 53.1f)
	{
		voltage_valid_cnt = 0;
		for(uint16_t _i = 0; _i<55555; _i++) { };
	}
	else
	{

		voltage_valid_cnt ++;
	}
	if(voltage_valid_cnt > 200)
	{
		return 1;
	}
	return 0;
	
}

float High_Voltage_Protect(void)
{
	if(INPUT_V > 55.0f) {
		return 0.1;
	} else if(INPUT_V > 12.0f){
		float input_v = INPUT_V;
		float max = 1.0672f - 0.0069 * input_v;
		return max;
	} else {
		return 1.0f;
	}
}

//电压保护
//分压电阻120K  采样电阻10K
//37V - 6.5V
//2.85V - 0.50V
//3531.8 - 620.5
//13V -> 1V -> 1240.9 
static uint8_t voltage_protect_cnt = 0;
uint8_t Voltage_Protect_Update(void)
{
	if(INPUT_V < 9.9f || INPUT_V > 53.1f)
	{
		voltage_protect_cnt++;
		if(voltage_protect_cnt > 200)
		{
			return 1;
		}
	}
	else
	{
		voltage_protect_cnt = 0;
	}
	return 0;
}



//温度保护
//分压电阻 10K
//温度低 电阻高 数值高

//B3380
//45C 4.88K 1.082V 1342.98
//50C 4.13K 0.965V 1196.91
//55C 3.51K 0.857V 1063.91
//60C 3.00K 0.762V 945
//65C 2.67K 0.695V 862.96
//70C 2.21K 0.597V 741.19

//B3450
//50C 4.085K 0.957V 1187.65
//55C 3.467K 0.850V 1054.23
//60C 2.995K 0.761V 943.78
//65C 2.529K 0.666V 826.58

#define TEMP_CNT 30

//B3380
//#define TEMP_LEVEL1_PROTECT 1064
//#define TEMP_LEVEL1_UNPROTECT 1197
//#define TEMP_LEVEL2_PROTECT 863
//#define TEMP_LEVEL2_UNPROTECT 945

//B3450
//#define TEMP_LEVEL1_PROTECT 1055
//#define TEMP_LEVEL1_UNPROTECT 1188
//#define TEMP_LEVEL2_PROTECT 827
//#define TEMP_LEVEL2_UNPROTECT 944

//B3380 2025.9.4 update
#define TEMP_LEVEL1_PROTECT 945
#define TEMP_LEVEL1_UNPROTECT 1064
#define TEMP_LEVEL2_PROTECT 741
#define TEMP_LEVEL2_UNPROTECT 863

static uint8_t temp_protection_status = 0;
static uint8_t temp_protect_cnt = 0;
static uint8_t temp_unprotect_cnt = 0;

uint8_t Temp_Protect_Update(void)
{
	switch(temp_protection_status)
	{
		case 0:
		{
			if(TSN_DATA < TEMP_LEVEL1_PROTECT) //温度大于v1温度保护
			{
				temp_protect_cnt++;
				if(temp_protect_cnt > TEMP_CNT)
				{
					temp_protection_status = 1;
					temp_protect_cnt = 0;
				}
			}
			else
			{
				temp_protect_cnt = 0;
			}
			break;
		}
		case 1:
		{
			if(TSN_DATA < TEMP_LEVEL2_PROTECT) //温度大于v2温度保护
			{
				temp_protect_cnt++;
				if(temp_protect_cnt > TEMP_CNT)
				{
					temp_protection_status = 2;
					temp_protect_cnt = 0;
				}
			}
			else if(TSN_DATA > TEMP_LEVEL1_UNPROTECT) //温度小于v1温度保护解除
			{
				temp_unprotect_cnt++;
				if(temp_unprotect_cnt > TEMP_CNT)
				{
					temp_protection_status = 0;
					temp_unprotect_cnt = 0;
				}
			}
			else
			{
				temp_unprotect_cnt = 0;
				temp_protect_cnt = 0;
			}
			break;
		}
		case 2:
		{
			if(TSN_DATA > TEMP_LEVEL2_UNPROTECT) //温度小于v2温度保护解除
			{
				temp_unprotect_cnt++;
				if(temp_unprotect_cnt > TEMP_CNT)
				{
					temp_protection_status = 1;
					temp_unprotect_cnt = 0;
				}
			}
			else
			{
				temp_unprotect_cnt = 0;
			}
			break;
		}
		
	}
	
	return temp_protection_status;
}

uint16_t Get_Adc1_Data(uint8_t num)
{
	return adc_data[num];
}
