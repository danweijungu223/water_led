#ifndef __VOLTAGE_ACQUISITION_H
#define __VOLTAGE_ACQUISITION_H

#include "stm32g0xx.h"
#include "stm32g0xx_ll_adc.h"
#include "stm32g0xx_ll_dma.h"

void Adc1_Additional_Config(void);
uint8_t Is_Voltage_Valid(void);
float High_Voltage_Protect(void);
uint8_t Voltage_Protect_Update(void);
uint8_t Temp_Protect_Update(void);
uint16_t Get_Adc1_Data(uint8_t num);

#endif
