#ifndef TIM_H
#define TIM_H

#include <n32l40x.h>

typedef enum TIM_OCChannels {
  CH1 = 0x01,
  CH2 = 0x02,
  CH3 = 0x04,
  CH4 = 0x08,
} TIM_OCChannels;

uint32_t TIM_Clk(TIM_Module *TIMx);
void TIM_Config();
void TIMx_GPIO_InitConfig(GPIO_InitType *initGPIO);

#endif // TIM_H
