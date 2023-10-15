#ifndef TIM_H
#define TIM_H

#include <stdint.h>

#define LED_TIM_UP_IRQHandler TIM1_UP_IRQHandler
#define LED_LowSide_TIM TIM3
#define LED_Main_TIM TIM1

void TIM_Config();

#endif // TIM_H
