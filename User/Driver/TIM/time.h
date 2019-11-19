#ifndef __TIME_H__
#define __TIME_H__
#include "stm32f10x.h"

#define TIM_ON  TIM_Cmd(TIM5,ENABLE)
#define TIM_OFF  TIM_Cmd(TIM5,DISABLE)
void Nvic_Init(void);
void TIM5_Config(void);
#endif 
