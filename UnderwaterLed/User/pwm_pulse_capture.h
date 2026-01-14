#ifndef __PWM_PULSE_CAPTURE_H
#define __PWM_PULSE_CAPTURE_H

#include "stm32g0xx.h"
#include "stm32g0xx_ll_gpio.h"
#include "stm32g0xx_ll_tim.h"

void Occur_Edge_Trigger(void);
void Occur_Timer_Update(void);

uint32_t Get_Pulse_Width(void);

void Reset_Input_Mode(void);
uint8_t Get_Input_Mode(void);
uint32_t Get_Led_Brightness_Switch(void);

uint8_t Is_Pulse_Width_Update(void);
void Reset_Pulse_Width_Update(void);

#endif
