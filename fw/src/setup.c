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

static uint16_t periods[] = {2400, 1800, 1200, 600, 0, 600, 1200, 1800};

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
  GPIO_InitType initValue;
  GPIO_InitStruct(&initValue);
  initValue.GPIO_Current = GPIO_DC_8mA;
  initValue.GPIO_Mode = GPIO_Mode_AF_OD;
  initValue.GPIO_Pull = GPIO_Pull_Up;

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
  
  // Setup SPI2

  // Setup PC7/9 for SPI2_SCK/MOSI
  initValue.Pin = GPIO_PIN_7 | GPIO_PIN_9;
  initValue.GPIO_Alternate = GPIO_AF5_SPI2;
  GPIO_InitPeripheral(GPIOC, &initValue);

  // Setup PA15 for SPI2_nSS
  initValue.Pin = GPIO_PIN_15;
  initValue.GPIO_Alternate = GPIO_AF1_SPI2;
  GPIO_InitPeripheral(GPIOA, &initValue);

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

static void GPIO_PWM_Config() {
  GPIO_InitType initValue;
  GPIO_InitStruct(&initValue);
  initValue.GPIO_Mode = GPIO_Mode_AF_PP;
  initValue.GPIO_Current = GPIO_DC_12mA;

  // Setup TIM1

  // Setup PA8/9/10 for TIM1_CH1/2/3
  // TIM1_CH1 for SAO_1_GPIO1
  // TIM1_CH2 for SAO_2_GPIO2
  // TIM1_CH3 for SAO_2_GPIO1
  initValue.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
  initValue.GPIO_Alternate = GPIO_AF2_TIM1;
  GPIO_InitPeripheral(GPIOA, &initValue);

  // Setup TIM2

  // Setup PB10/11 for TIM2_CH3/4
  // TIM2_CH3 
  // initValue.Pin = GPIO_PIN_10 | GPIO_PIN_11;
  // initValue.GPIO_Alternate = GPIO_AF2_TIM2;
  // GPIO_InitPeripheral(GPIOB, &initValue);

  // Setup TIM3

  // Setup PA6/7 for TIM3_CH1/2
  // TIM3_CH1 for LowSide_B_Col_1
  // TIM3_CH2 for LowSide_R_Col_1
  initValue.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  initValue.GPIO_Alternate = GPIO_AF2_TIM3;
  GPIO_InitPeripheral(GPIOA, &initValue);

  // Setup PB0 for TIM3_CH3
  // TIM3_CH3 for LowSide_G_Col_1
  initValue.Pin = GPIO_PIN_0;// | GPIO_PIN_1;
  initValue.GPIO_Alternate = GPIO_AF2_TIM3;
  GPIO_InitPeripheral(GPIOB, &initValue);

  // Setup TIM4

  // Setup PB6/7/8/9 for TIM4_CH1/2/3/4
  // initValue.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
  // initValue.GPIO_Alternate = GPIO_AF2_TIM4;
  // GPIO_InitPeripheral(GPIOB, &initValue);

  // Setup TIM5

  // Setup PA0 for TIM5_CH1
  // initValue.Pin = GPIO_PIN_0;
  // initValue.GPIO_Alternate = GPIO_AF1_TIM5;
  // GPIO_InitPeripheral(GPIOA, &initValue);

  // Setup PA1/3 for TIM5_CH2/4
  // initValue.Pin = GPIO_PIN_1 | GPIO_PIN_3;
  // initValue.GPIO_Alternate = GPIO_AF7_TIM5;
  // GPIO_InitPeripheral(GPIOA, &initValue);

  // Setup PA2 for TIM5_CH3
  // initValue.Pin = GPIO_PIN_2;
  // initValue.GPIO_Alternate = GPIO_AF6_TIM5;
  // GPIO_InitPeripheral(GPIOA, &initValue);

  // Setup TIM8

  // Setup PC6 for TIM8_CH1
  // TIM8_CH1 for SAO_1_GPIO2
  initValue.Pin = GPIO_PIN_6;
  initValue.GPIO_Alternate = GPIO_AF6_TIM8;
  GPIO_InitPeripheral(GPIOC, &initValue);

  // Setup TIM9

  // Setup PB12/13/14/15 for TIM9_CH1/2/3/4
  // initValue.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  // initValue.GPIO_Alternate = GPIO_AF1_TIM9;
  // GPIO_InitPeripheral(GPIOB, &initValue);

}

static void GPIO_IO_Config() {
  GPIO_InitType initValue;
  GPIO_InitStruct(&initValue);
  initValue.GPIO_Current = GPIO_DC_12mA;
  // Setup GPIOA
  // PA0 for HighSide_Row_9
  // PA1 for HighSide_Row_6
  // PA2 for HighSide_Row_4
  // PA3 for HighSide_Row_3
  // initValue.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_6 | GPIO_PIN_4 | GPIO_PIN_3;
  // initValue.GPIO_Pull = GPIO_No_Pull;
  // initValue.GPIO_Mode = GPIO_Mode_Out_PP;
  // GPIO_InitPeripheral(GPIOA, &initValue);

  // Setup GPIOB
  // PB1 for HighSide_Row_12
  // PB10 for HighSide_Row_11
  // PB11 for HighSide_Row_10
  // PB12 for HighSide_Row_7
  // PB13 for HighSide_Row_8
  // PB14 for HighSide_Row_5
  // PB15 for HighSide_Row_1
  // initValue.Pin = GPIO_PIN_1 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  // initValue.GPIO_Pull = GPIO_No_Pull;
  // initValue.GPIO_Mode = GPIO_Mode_Out_PP;
  // GPIO_InitPeripheral(GPIOB, &initValue);

  // Setup GPIOC
  // PC10 for HighSide_Row_2
  // initValue.Pin = GPIO_PIN_10;
  // initValue.GPIO_Pull = GPIO_No_Pull;
  // initValue.GPIO_Mode = GPIO_Mode_Out_PP;
  // GPIO_InitPeripheral(GPIOC, &initValue);


  // PB4/5 for EVB testing
  initValue.Pin = GPIO_PIN_4 | GPIO_PIN_5;
  initValue.GPIO_Pull = GPIO_No_Pull;
  initValue.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitPeripheral(GPIOB, &initValue);

}

static void GPIO_Config() {

  GPIO_PWM_Config();
  GPIO_SPI_Config();
  GPIO_I2C_Config();

  GPIO_IO_Config();
}

static void DMA_Config() {
  DMA_InitType dmaInit;
  DMA_StructInit(&dmaInit);

  DMA_DeInit(DMA_CH1);
  dmaInit.PeriphAddr = (uint32_t)&TIM1->DADDR;
  dmaInit.PeriphInc = DMA_PERIPH_INC_DISABLE;
  dmaInit.PeriphDataSize = DMA_PERIPH_DATA_SIZE_HALFWORD;
  dmaInit.Direction = DMA_DIR_PERIPH_DST;
  dmaInit.MemAddr = (uint32_t)&periods;
  dmaInit.Mem2Mem = DMA_M2M_DISABLE;
  dmaInit.BufSize = sizeof(periods) / sizeof(periods[0]);
  dmaInit.MemDataSize = DMA_MemoryDataSize_HalfWord;
  dmaInit.DMA_MemoryInc = DMA_MEM_INC_ENABLE;
  dmaInit.CircularMode = DMA_MODE_CIRCULAR;
  dmaInit.Priority = DMA_PRIORITY_HIGH;

  DMA_Init(DMA_CH1, &dmaInit);
  DMA_ConfigInt(DMA_CH1, DMA_INT_HTX1|DMA_INT_TXC1|DMA_INT_ERR1, ENABLE);
  DMA_RequestRemap(DMA_REMAP_TIM1_UP, DMA, DMA_CH1, ENABLE);
}

/*
 * LowSide_B_Col_1 -> TIM3_CH1                        -------     -------     -------
 * LowSide_R_Col_1 -> TIM3_CH2                        |     |    /     /      | D10 |
 * LowSide_G_Col_1 -> TIM3_CH3                        | D2  |   / D6  /       |_____|
 * HighSide_Row_1  -> TIM9_CH4 -> D2                  |     |  /     /        
 * HighSide_Row_2  -> PC10     -> D3                  |     | /     /         
 * HighSide_Row_3  -> TIM5_CH4 -> D4                  |     |/     /           _____          ______       ______          _______
 * HighSide_Row_4  -> TIM5_CH3 -> D5                  | D3     D5 |           |     |        /      |     /      \        |  D12  |
 * HighSide_Row_5  -> TIM9_CH3 -> D6                  |     |\     \          | D9  |       /      /     /        \       |   _   |
 * HighSide_Row_6  -> TIM2_CH2 -> D7                  |     | \     \         |     |      /  _ _ /     /          \      |  | |  |
 * HighSide_Row_7  -> TIM9_CH1 -> D10                 |     |  \     \        |     |     /   | D11    |    D13     |     |  | |  |
 * HighSide_Row_8  -> TIM9_CH2 -> D9                  |     |   \     \       |     |     \   |        |            |     |  | |  |
 * HighSide_Row_9  -> TIM2_CH1 -> D8                  |     |    \     \      |     |      \  ----      \          /      |  | |  |
 * HighSide_Row_10 -> TIM2_CH4 -> D11                 | D4  |     \ D7  \     | D8  |       \     \      \        /       |  | |  |
 * HighSide_Row_11 -> TIM2_CH3 -> D13                 |     |      \     \    |     |        \_____\__    \______/        |__| |__|
 * HighSide_Row_12 -> TIM3_CH4 -> D12                 -------       -------   -------
 */
static void TIM_Config() {
  TIM_TimeBaseInitType timInit;
  TIM_InitTimBaseStruct(&timInit);

  OCInitType ocInit;
  TIM_InitOcStruct(&ocInit);

  const uint16_t prescaler = 1;
  uint16_t period = SystemCoreClock / prescaler / LED_PWM_CLOCK - 1;
  timInit.Period = period;
  timInit.Prescaler = prescaler - 1;
  timInit.CntMode = TIM_CNT_MODE_UP;
  timInit.RepetCnt = 0xff;
  TIM_InitTimeBase(TIM1, &timInit);

  ocInit.OcMode = TIM_OCMODE_PWM1;
  ocInit.OutputState = TIM_OUTPUT_STATE_ENABLE;
  ocInit.Pulse = period / 2;

  // TIM3_CH1 for LowSide_B_Col_1
  TIM_InitOc1(TIM1, &ocInit);

  // TIM3_CH2 for LowSide_R_Col_1
  TIM_InitOc2(TIM1, &ocInit);

  // TIM3_CH3 for LowSide_G_Col_1
  TIM_InitOc3(TIM1, &ocInit);

  TIM_ConfigArPreload(TIM1, DISABLE);
  TIM_ConfigDma(TIM1, TIM_DMABASE_CAPCMPDAT1, TIM_DMABURST_LENGTH_1TRANSFER);
  TIM_EnableDma(TIM1, TIM_DMA_UPDATE, ENABLE);

  TIM_ConfigInt(TIM1, TIM_INT_UPDATE|TIM_INT_CC1, ENABLE);
  // TIM_EnableUpdateEvt_r(TIM1, ENABLE);


}

static void NVIC_Config() {
  NVIC_InitType nvicInit = {0};
  nvicInit.NVIC_IRQChannel = DMA_Channel1_IRQn;
  nvicInit.NVIC_IRQChannelCmd = ENABLE;
  
  NVIC_Init(&nvicInit);

  nvicInit.NVIC_IRQChannel = TIM1_UP_IRQn;
  nvicInit.NVIC_IRQChannelCmd = ENABLE;
  
  NVIC_Init(&nvicInit);

  nvicInit.NVIC_IRQChannel = TIM1_CC_IRQn;
  nvicInit.NVIC_IRQChannelCmd = ENABLE;
  
  NVIC_Init(&nvicInit);
}

void Setup() {
  NVIC_Config();
  
  RCC_Config();
  GPIO_Config();

  DMA_Config();
  TIM_Config();


  TIM_SetCnt(TIM1, periods[0] + 1);
  TIM_GenerateEvent(TIM1, TIM_EVTGEN_UDGN);

  DMA_ClrIntPendingBit(DMA_INT_GLB1|DMA_INT_TXC1|DMA_INT_HTX1|DMA_INT_ERR1, DMA);
  DMA_EnableChannel(DMA_CH1, ENABLE);

  TIM_ClrIntPendingBit(TIM1, TIM_INT_UPDATE|TIM_INT_CC1|TIM_INT_CC2|TIM_INT_CC3|TIM_INT_CC4);
  TIM_Enable(TIM1, ENABLE);
  TIM_EnableCtrlPwmOutputs(TIM1, ENABLE);

}
