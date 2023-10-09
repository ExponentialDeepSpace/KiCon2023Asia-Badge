/*
 * Copyright 2023 Yang Hongbo, Beijing Exponential Deep Space Ltd.,
 *
 * KiCon 2023 Asia Badge Firmware is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * KiCon 2023 Asia Badge Firmware is distributed in the hope that it will
 * be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with KiCon 2023 Asia Badge Firmware. If not, see
 * <https://www.gnu.org/licenses/>. 
 */

#include <n32l40x.h>
#include <assert.h>
#include "setup.h"

typedef struct GPIO_HighSide_Def {
  GPIO_Module *gpio;
  uint16_t pin;
}GPIO_HighSide_Def;

static GPIO_HighSide_Def GPIOs_HighSide_Rows[12] = {
  {GPIOB, GPIO_PIN_15}, // Row 1
  {GPIOC, GPIO_PIN_10}, // Row 2
  {GPIOA, GPIO_PIN_3},  // Row 3
  {GPIOA, GPIO_PIN_2},  // Row 4
  {GPIOB, GPIO_PIN_14}, // Row 5
  {GPIOA, GPIO_PIN_1},  // Row 6
  {GPIOB, GPIO_PIN_12}, // Row 7
  {GPIOB, GPIO_PIN_13}, // Row 8
  {GPIOA, GPIO_PIN_0},  // Row 9
  {GPIOB, GPIO_PIN_11}, // Row 10
  {GPIOB, GPIO_PIN_10}, // Row 11
  {GPIOB, GPIO_PIN_1},  // Row 12
};

int main(void)
{
  Setup();
  
  volatile unsigned long long i = 0;

  GPIO_ResetBits(GPIOB, GPIO_PIN_4);
  GPIO_ResetBits(GPIOB, GPIO_PIN_5);

  while (1) {
    PWR_EnterSLEEPMode(1, PWR_SLEEPENTRY_WFI);
    // GPIO_SetBits(GPIOB, GPIO_PIN_4);
    // GPIO_ResetBits(GPIOB, GPIO_PIN_5);
    // i = 0;
    // while (i < 1000000) i ++;
    // i = 0;
    // GPIO_SetBits(GPIOB, GPIO_PIN_5);
    // GPIO_ResetBits(GPIOB, GPIO_PIN_4);
    // while (i < 1000000) i ++;
  }
  return 0;
}

void DMA_Channel1_IRQHandler(void) {
  int i = 0;
  i++;

  DMA_ClrIntPendingBit(DMA_INT_GLB1|DMA_INT_TXC1|DMA_INT_HTX1|DMA_INT_ERR1, DMA);
}

void TIM1_UP_IRQHandler(void) {
  TIM_ClrIntPendingBit(TIM1, TIM_INT_UPDATE);

  static int i = 0;
  static int j = 11;
  GPIO_ResetBits(GPIOs_HighSide_Rows[j].gpio, GPIOs_HighSide_Rows[j].pin);
  GPIO_SetBits(GPIOs_HighSide_Rows[i].gpio, GPIOs_HighSide_Rows[i].pin);
  i++; j++;

  if (i > 11) {
    i = 0;
  }

  if (j > 11) {
    j = 0;
  }
}

void TIM1_CC_IRQHandler(void) {
  int i = 0;
  i++;
  TIM_ClrIntPendingBit(TIM1, TIM_INT_CC1);
}

