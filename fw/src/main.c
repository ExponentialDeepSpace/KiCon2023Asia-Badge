/*
 * Copyright 2023 Yang Hongbo, Qingdao IotPi Ltd.,
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

#include "setup.h"

#define LED_PWM_CLOCK 20000 // 20Khz

int main(void)
{
  // Default Clock (defined in system_n32l40x.c:SetSysClock())
  // SystemCoreclock = 48Mhz (for USB)
  // HCLK/AHB = 48Mhz (DIV1)
  // PCLK2/APB2 = 24Mhz (DIV2)
  // PCLK1/APB1 = 12Mhz (DIV4)
  
  RCC_Config();
  GPIO_Config();
  
  volatile unsigned long long i = 0;

  GPIO_ResetBits(GPIOB, GPIO_PIN_4);
  GPIO_ResetBits(GPIOB, GPIO_PIN_5);

  while (1) {
    GPIO_SetBits(GPIOB, GPIO_PIN_4);
    GPIO_ResetBits(GPIOB, GPIO_PIN_5);
    i = 0;
    while (i < 1000000) i ++;
    i = 0;
    GPIO_SetBits(GPIOB, GPIO_PIN_5);
    GPIO_ResetBits(GPIOB, GPIO_PIN_4);
    while (i < 1000000) i ++;

  }
  return 0;
}

