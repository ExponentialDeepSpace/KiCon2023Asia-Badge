#ifndef LED_H
#define LED_H

#include <stdint.h>

#define TIM_SELECTED_SLAVE_MODE TIM_SLAVE_MODE_GATED

#define LED_TIM_UP_IRQHandler TIM1_UP_IRQHandler
#define LED_LowSide_TIM TIM8
#define LED_Main_TIM TIM1
#define LED_SPEED 20

#define LED_SAO_GPIO_TIM TIM4

void LED_Config(void);

#endif // LED_H
