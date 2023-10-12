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
#include "setup.h"
#include "tim.h"
#include "memory_in_pixel_display.h"

// Default Clock (defined in system_n32l40x.c:SetSysClock())
// SystemCoreclock = 48Mhz (for USB)
// HCLK/AHB = 48Mhz (DIV1)
// PCLK2/APB2 = 24Mhz (DIV2)
// PCLK1/APB1 = 12Mhz (DIV4)

static void
RCC_Config() {
  RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_DMA, ENABLE);
  RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_USB
                          |RCC_APB1_PERIPH_I2C1
                          |RCC_APB1_PERIPH_TIM3
                          , ENABLE);
  RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA
                          |RCC_APB2_PERIPH_GPIOB
                          |RCC_APB2_PERIPH_GPIOC
                          |RCC_APB2_PERIPH_AFIO
                          |RCC_APB2_PERIPH_SPI2
                          |RCC_APB2_PERIPH_TIM1
                          |RCC_APB2_PERIPH_TIM8
                          , ENABLE);

  DBG_ConfigPeriph(DBG_TIM1_STOP | DBG_TIM8_STOP, ENABLE);
}
static void GPIO_SPI_Config() {

  // Setup SPI1

  // Setup PA4 for SPI1_nSS
  // initValue.Pin = GPIO_PIN_4;
  // initValue.GPIO_Alternate = GPIO_AF0_SPI1;
  // GPIO_InitPeripheral(GPIOA, &initValue);
  
  // Setup PB3/4 for SPI1_SCK/MISO
  // initValue.Pin = GPIO_PIN_3 | GPIO_PIN_4;
  // initValue.GPIO_Alternate = GPIO_AF1_SPI1;
  // GPIO_InitPeripheral(GPIOB, &initValue);

  // Setup PB5 for SPI1_MOSI
  // initValue.Pin = GPIO_PIN_5;
  // initValue.GPIO_Alternate = GPIO_AF0_SPI1;
  // GPIO_InitPeripheral(GPIOB, &initValue);
  

}

static void GPIO_I2C_Config() {
  GPIO_InitType initValue;
  GPIO_InitStruct(&initValue);
  initValue.GPIO_Mode = GPIO_Mode_AF_OD;
  initValue.GPIO_Current = GPIO_DC_8mA;
  initValue.GPIO_Pull = GPIO_Pull_Up;

  // Setup I2C1
  
  // Setup PC4/5 for I2C1_SCL/SDA
  initValue.Pin = GPIO_PIN_4 | GPIO_PIN_5;
  initValue.GPIO_Alternate = GPIO_AF7_I2C1;
  GPIO_InitPeripheral(GPIOC, &initValue);

}

static void GPIO_IO_Config() {
  GPIO_InitType initGPIO;
  GPIO_InitStruct(&initGPIO);
  initGPIO.GPIO_Mode = GPIO_Mode_Out_PP;
  initGPIO.GPIO_Current = GPIO_DC_8mA;

  // PB4/5 for EVB testing
  initGPIO.Pin = GPIO_PIN_4 | GPIO_PIN_5;
  initGPIO.GPIO_Pull = GPIO_No_Pull;
  initGPIO.GPIO_Mode = GPIO_Mode_Out_PP;
  initGPIO.GPIO_Current = GPIO_DC_8mA;
  GPIO_InitPeripheral(GPIOB, &initGPIO);
}

static void GPIO_HighSide_Config() {
  GPIO_InitType initGPIO;
  GPIO_InitStruct(&initGPIO);
  initGPIO.GPIO_Mode = GPIO_Mode_Out_PP;
  initGPIO.GPIO_Current = GPIO_DC_8mA;

  // (PA0)  -> HighRow_Row_9
  // (PA1)  -> HighRow_Row_6
  initGPIO.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitPeripheral(GPIOA, &initGPIO);

  // (PB10) -> HighRow_Row_11
  // (PB11) -> HighRow_Row_10
  initGPIO.Pin = GPIO_PIN_10 | GPIO_PIN_11;
  GPIO_InitPeripheral(GPIOB, &initGPIO);

  // (PB1) -> HighSide_Row_12
  initGPIO.Pin = GPIO_PIN_1;
  GPIO_InitPeripheral(GPIOB, &initGPIO);

  // (PA2) -> HighRow_Row_4
  initGPIO.Pin = GPIO_PIN_2;
  GPIO_InitPeripheral(GPIOA, &initGPIO);
  // (PA3) -> HighRow_Row_3
  initGPIO.Pin =  GPIO_PIN_3;
  GPIO_InitPeripheral(GPIOA, &initGPIO);

  // (PB12) -> HighRow_Row_7
  // (PB13) -> HighRow_Row_8
  // (PB14) -> HighRow_Row_5
  // (PB15) -> HighRow_Row_1

  initGPIO.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitPeripheral(GPIOB, &initGPIO);

  // (PC10) -> HighRow_Row_2
  initGPIO.Pin =  GPIO_PIN_10;
  GPIO_InitPeripheral(GPIOC, &initGPIO);

}

static void GPIO_Config() {

  GPIO_SPI_Config();
  GPIO_I2C_Config();

  GPIO_IO_Config();
  GPIO_HighSide_Config();
}


static void NVIC_Config() {
  // as required in https://www.freertos.org/RTOS-Cortex-M3-M4.html
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

  NVIC_InitType nvicInit = {0};
  nvicInit.NVIC_IRQChannel = DMA_Channel1_IRQn;
  nvicInit.NVIC_IRQChannelPreemptionPriority = TIM_DMA_INTERRUPT_PRIORITY_LEVEL;
  nvicInit.NVIC_IRQChannelCmd = ENABLE;
  
  NVIC_Init(&nvicInit);

  nvicInit.NVIC_IRQChannel = TIM1_UP_IRQn;
  nvicInit.NVIC_IRQChannelPreemptionPriority = TIM_INTERRUPT_PRIORITY_LEVEL;
  nvicInit.NVIC_IRQChannelCmd = ENABLE;
  
  NVIC_Init(&nvicInit);

  nvicInit.NVIC_IRQChannel = TIM1_CC_IRQn;
  nvicInit.NVIC_IRQChannelPreemptionPriority = TIM_INTERRUPT_PRIORITY_LEVEL;
  nvicInit.NVIC_IRQChannelCmd = ENABLE;
  
  NVIC_Init(&nvicInit);
}

void Setup() {
  RCC_Config();
  NVIC_Config();
  
  GPIO_Config();

  TIM_Config();

  Display_Config();

}
