#include <n32l40x.h>

#include "setup.h"
#include "tim.h"

#define TIM_SELECTED_SLAVE_MODE TIM_SLAVE_MODE_GATED

static uint16_t periods[] = {2400, 1800, 1200, 600, 0, 600, 1200, 1800};

static void TIM_DMA_Config() {
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
 * LowSide_B_Col_1 -> TIM3_CH1(PA6)                         -------     -------     -------
 * LowSide_R_Col_1 -> TIM3_CH2(PA7)                         |     |    /     /      | D10 |
 * LowSide_G_Col_1 -> TIM3_CH3(PB0)                         | D2  |   / D6  /       |_____|
 * HighSide_Row_1  -> TIM9_CH4(PB15) -> D2                  |     |  /     /        
 * HighSide_Row_2  -> PC10           -> D3                  |     | /     /         
 * HighSide_Row_3  -> TIM5_CH4(PA3) -> D4                   |     |/     /           _____          ______       ______          _______
 * HighSide_Row_4  -> TIM5_CH3(PA2) -> D5                   | D3     D5 |           |     |        /      |     /      \        |  D12  |
 * HighSide_Row_5  -> TIM9_CH3(PB14) -> D6                  |     |\     \          | D9  |       /      /     /        \       |   _   |
 * HighSide_Row_6  -> TIM2_CH2(PA1)  -> D7                  |     | \     \         |     |      /  _ _ /     /          \      |  | |  |
 * HighSide_Row_7  -> TIM9_CH1(PB12) -> D10                 |     |  \     \        |     |     /   | D11    |    D13     |     |  | |  |
 * HighSide_Row_8  -> TIM9_CH2(PB13) -> D9                  |     |   \     \       |     |     \   |        |            |     |  | |  |
 * HighSide_Row_9  -> TIM2_CH1(PA0)  -> D8                  |     |    \     \      |     |      \  ----      \          /      |  | |  |
 * HighSide_Row_10 -> TIM2_CH4(PB11) -> D11                 | D4  |     \ D7  \     | D8  |       \     \      \        /       |  | |  |
 * HighSide_Row_11 -> TIM2_CH3(PB10) -> D13                 |     |      \     \    |     |        \_____\__    \______/        |__| |__|
 * HighSide_Row_12 -> TIM3_CH4(PB1)  -> D12                 -------       -------   -------
 * SAO_1_GPIO1 -> TIM1_CH1(PA8)
 * SAO_1_GPIO2 -> TIM8_CH1(PC6)
 * SAO_2_GPIO1 -> TIM1_CH3(PA10)
 * SAO_2_GPIO2 -> TIM1_CH2(PA9)
 */

/*
 * For 30fps, 33.33 ms/frame, use 33ms/frame
 * There are 12 LEDs, so each LED need to work for 2.75ms in each frame
 * If using 20Khz PWM, it needs 55 cycles ( to complete 2.75ms )
 * TIM1 as master, all other are slaves
 * due to TIM5 cannot be salve of TIM1, so TIM5 is slave of TIM8
 */
typedef enum TIM_OCChannels {
  CH1 = 0x01,
  CH2 = 0x02,
  CH3 = 0x04,
  CH4 = 0x08,
} TIM_OCChannels;

static uint32_t TIM_Clk(TIM_Module *TIMx, const uint16_t prescaler) {

  uint32_t TIMClk = 0;

  if (IsTimList1Module(TIMx)) {
    uint32_t APB2Clk = 0;
    uint32_t APB2Cfg = RCC->CFG & RCC_CFG_APB2PRES;
    // TIM1, TIM8 clk from APB2 (
    if (0 == (APB2Cfg & RCC_CFG_APB2PRES_2)) {
      APB2Clk = SystemCoreClock;
    } else {
      switch (APB2Cfg) {
      case RCC_CFG_APB2PRES_DIV2:
        APB2Clk = SystemCoreClock / 2;
        break;
      case RCC_CFG_APB2PRES_DIV4:
        APB2Clk = SystemCoreClock / 4;
        break;
      case RCC_CFG_APB2PRES_DIV8:
        APB2Clk = SystemCoreClock / 8;
        break;
      case RCC_CFG_APB2PRES_DIV16:
        APB2Clk = SystemCoreClock / 16;
        break;
      }
    }

    if (0 == prescaler) {
      TIMClk = APB2Clk;
    } else {
      TIMClk = APB2Clk * 2 / (prescaler + 1);
    }

  } else if (IsTimList3Module(TIMx) && !IsTimList1Module(TIMx)) {
    // TIM 2, 3, 4, 5, 9 clk from APB1
    uint32_t APB1Clk = 0;
    uint32_t APB1Cfg = RCC->CFG & RCC_CFG_APB1PRES;
    if (0 == (APB1Cfg & RCC_CFG_APB1PRES_2)) {
      APB1Clk = SystemCoreClock;
    } else {
      switch (APB1Cfg) {
      case RCC_CFG_APB1PRES_DIV2:
        APB1Clk = SystemCoreClock / 2;
        break;
      case RCC_CFG_APB1PRES_DIV4:
        APB1Clk = SystemCoreClock / 4;
        break;
      case RCC_CFG_APB1PRES_DIV8:
        APB1Clk = SystemCoreClock / 8;
        break;
      case RCC_CFG_APB1PRES_DIV16:
        APB1Clk = SystemCoreClock / 16;
        break;
      }
    }
    if (0 == prescaler) {
      TIMClk = APB1Clk;
    } else {
      TIMClk = APB1Clk * 2 / (prescaler + 1);
    }
  }

  return TIMClk;
}

static void TIMx_PWM_Config(TIM_Module *TIMx, uint8_t channels) {
  TIM_TimeBaseInitType timInit;
  OCInitType ocInit;

  TIM_InitTimBaseStruct(&timInit);
  TIM_InitOcStruct(&ocInit);

  uint16_t prescaler = IsTimList1Module(TIMx) ?
    TIM_ADVANCED_PRESCALER : TIM_GENERIC_PRESCALER;

  uint32_t TIMClk = TIM_Clk(TIMx, prescaler);

  uint16_t period = TIMClk / LED_PWM_CLOCK - 1;
  timInit.Period = period;
  timInit.Prescaler = prescaler;
  timInit.CntMode = TIM_CNT_MODE_UP;
  TIM_InitTimeBase(TIMx, &timInit);

  ocInit.OcMode = TIM_OCMODE_PWM1;
  ocInit.OutputState = TIM_OUTPUT_STATE_ENABLE;
  ocInit.Pulse = period / 2;

  if (channels & CH1) {
    TIM_InitOc1(TIMx, &ocInit);
    TIM_ConfigOc1Fast(TIMx, TIM_OC_FAST_ENABLE);
  }
  if (channels & CH2) {
    TIM_InitOc2(TIMx, &ocInit);
    TIM_ConfigOc2Fast(TIMx, TIM_OC_FAST_ENABLE);
  }
  if (channels & CH3) {
    TIM_InitOc3(TIMx, &ocInit);
    TIM_ConfigOc3Fast(TIMx, TIM_OC_FAST_ENABLE);
  }
  if (channels & CH4) {
    TIM_InitOc4(TIMx, &ocInit);
    TIM_ConfigOc4Fast(TIMx, TIM_OC_FAST_ENABLE);
  }

  TIM_ConfigArPreload(TIMx, DISABLE);
  // TIM_ConfigDma(TIMx, TIM_DMABASE_CAPCMPDAT1, TIM_DMABURST_LENGTH_4TRANSFERS);
  // TIM_EnableDma(TIMx, TIM_DMA_UPDATE, ENABLE);

  TIM_ConfigInt(TIMx, TIM_INT_UPDATE|TIM_INT_CC1, ENABLE);
  // TIM_EnableUpdateEvt_r(TIM1, ENABLE);
}


static void TIMx_GPIO_InitConfig(GPIO_InitType *initGPIO) {
  GPIO_InitStruct(initGPIO);
  initGPIO->GPIO_Mode = GPIO_Mode_AF_PP;
  initGPIO->GPIO_Current = GPIO_DC_12mA;
}

static void TIM1_Config() {
  // TIM1_CH1(PA8) for SAO_1_GPIO1
  // TIM1_CH2(PA9) for SAO_2_GPIO2
  // TIM1_CH3(PA10) for SAO_2_GPIO1

  // Setup GPIO for TIM1
  GPIO_InitType initGPIO;
  TIMx_GPIO_InitConfig(&initGPIO);
  // Setup PA8/9/10 for TIM1_CH1/2/3
  initGPIO.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
  initGPIO.GPIO_Alternate = GPIO_AF2_TIM1;
  GPIO_InitPeripheral(GPIOA, &initGPIO);

  TIMx_PWM_Config(TIM1, (uint8_t)(CH1|CH2|CH3));

  // Master Trigger
  TIM_SelectOutputTrig(TIM1, TIM_TRGO_SRC_ENABLE);
  TIM_SelectMasterSlaveMode(TIM1, TIM_MASTER_SLAVE_MODE_ENABLE);

  TIM1->REPCNT = 55;
}

static void TIM8_Config() {
  // TIM8_CH1(PC6) for SAO_1_GPIO2

  // Setup GPIO for TIM8

  GPIO_InitType initGPIO;
  TIMx_GPIO_InitConfig(&initGPIO);
  // Setup PC6 for TIM8_CH1
  initGPIO.Pin = GPIO_PIN_6;
  initGPIO.GPIO_Alternate = GPIO_AF6_TIM8;
  GPIO_InitPeripheral(GPIOC, &initGPIO);

  TIMx_PWM_Config(TIM8, (uint8_t)(CH1));

  TIM_SelectSlaveMode(TIM8, TIM_SELECTED_SLAVE_MODE);
  TIM_SelectInputTrig(TIM8, TIM_TRIG_SEL_IN_TR0); // triggered from TIM1
}

static void TIM2_Config() {
  
  // TIM2_CH1(PA0)  -> HighRow_Row_9
  // TIM2_CH2(PA1)  -> HighRow_Row_6
  // TIM2_CH3(PB10) -> HighRow_Row_11
  // TIM2_CH4(PB11) -> HighRow_Row_10

  // Setup GPIO for TIM2
  GPIO_InitType initGPIO;
  TIMx_GPIO_InitConfig(&initGPIO);
  // Setup PA0/1 for TIM2_CH1/2
  initGPIO.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  initGPIO.GPIO_Alternate = GPIO_AF2_TIM2;
  GPIO_InitPeripheral(GPIOA, &initGPIO);
  // Setup PB10/11 for TIM2_CH3/4
  initGPIO.Pin = GPIO_PIN_10 | GPIO_PIN_11;
  initGPIO.GPIO_Alternate = GPIO_AF2_TIM2;
  GPIO_InitPeripheral(GPIOB, &initGPIO);

  TIMx_PWM_Config(TIM2, (uint8_t)(CH1|CH2|CH3|CH4));

  TIM_SelectSlaveMode(TIM2, TIM_SELECTED_SLAVE_MODE);
  TIM_SelectInputTrig(TIM2, TIM_TRIG_SEL_IN_TR0); // triggered from TIM1
}

static void TIM3_Config() {
  
  // TIM3_CH1(PA6) -> LowSide_B_Col_1
  // TIM3_CH2(PA7) -> LowSide_R_Col_1
  // TIM3_CH3(PB0) -> LowSide_G_Col_1
  // TIM3_CH4(PB1) -> HighSide_Row_12

  // Setup GPIO for TIM3

  GPIO_InitType initGPIO;
  TIMx_GPIO_InitConfig(&initGPIO);
  // Setup PA6/7 for TIM3_CH1/2
  initGPIO.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  initGPIO.GPIO_Alternate = GPIO_AF2_TIM3;
  GPIO_InitPeripheral(GPIOA, &initGPIO);
  // Setup PB0 for TIM3_CH3
  initGPIO.Pin = GPIO_PIN_0;
  initGPIO.GPIO_Alternate = GPIO_AF2_TIM3;
  GPIO_InitPeripheral(GPIOB, &initGPIO);

  TIMx_PWM_Config(TIM3, (uint8_t)(CH1|CH2|CH3));

  TIM_SelectSlaveMode(TIM3, TIM_SELECTED_SLAVE_MODE);
  TIM_SelectInputTrig(TIM3, TIM_TRIG_SEL_IN_TR0); // triggered from TIM1
}

static void TIM5_Config() {
  
  // TIM5_CH3(PA2) -> HighRow_Row_4
  // TIM5_CH4(PA3) -> HighRow_Row_3

  // Setup GPIO for TIM5
  GPIO_InitType initGPIO;
  TIMx_GPIO_InitConfig(&initGPIO);
  // Setup PA2 for TIM5_CH3
  initGPIO.Pin = GPIO_PIN_2;
  initGPIO.GPIO_Alternate = GPIO_AF6_TIM5;
  GPIO_InitPeripheral(GPIOA, &initGPIO);
  // Setup PA3 for TIM5_CH4
  initGPIO.Pin =  GPIO_PIN_3;
  initGPIO.GPIO_Alternate = GPIO_AF7_TIM5;
  GPIO_InitPeripheral(GPIOA, &initGPIO);

  TIMx_PWM_Config(TIM5, (uint8_t)(CH3|CH4));

  TIM_SelectSlaveMode(TIM5, TIM_SELECTED_SLAVE_MODE);
  TIM_SelectInputTrig(TIM5, TIM_TRIG_SEL_IN_TR3); // triggered from TIM8
}

static void TIM9_Config() {
  
  // TIM9_CH1(PB12) -> HighRow_Row_7
  // TIM9_CH2(PB13) -> HighRow_Row_8
  // TIM9_CH3(PB14) -> HighRow_Row_5
  // TIM9_CH4(PB15) -> HighRow_Row_1

  // Setup GPIO for TIM9
  GPIO_InitType initGPIO;
  TIMx_GPIO_InitConfig(&initGPIO);

  // Setup PB12/13/14/15 for TIM9_CH1/2/3/4
  initGPIO.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  initGPIO.GPIO_Alternate = GPIO_AF1_TIM9;
  GPIO_InitPeripheral(GPIOB, &initGPIO);

  TIMx_PWM_Config(TIM9, (uint8_t)(CH1|CH2|CH3|CH4));
  TIM_SelectSlaveMode(TIM9, TIM_SELECTED_SLAVE_MODE);
  TIM_SelectInputTrig(TIM9, TIM_TRIG_SEL_IN_TR0); // triggered from TIM1
}

void TIM_Config() {
  TIM1_Config();
  // TIM2_Config();
  TIM3_Config();
  // TIM5_Config();
  TIM8_Config();
  // TIM9_Config();

  TIM_DMA_Config();
  
  TIM_SetCnt(TIM1, periods[0] + 1);
  TIM_GenerateEvent(TIM1, TIM_EVTGEN_UDGN);

  DMA_ClrIntPendingBit(DMA_INT_GLB1|DMA_INT_TXC1|DMA_INT_HTX1|DMA_INT_ERR1, DMA);
  // DMA_EnableChannel(DMA_CH1, ENABLE);

  TIM_ClrIntPendingBit(TIM1,
                       TIM_INT_UPDATE
                       |TIM_INT_CC1
                       |TIM_INT_CC2
                       |TIM_INT_CC3
                       |TIM_INT_CC4);
  TIM_ClrIntPendingBit(TIM2,
                       TIM_INT_UPDATE
                       |TIM_INT_CC1
                       |TIM_INT_CC2
                       |TIM_INT_CC3
                       |TIM_INT_CC4);
  TIM_ClrIntPendingBit(TIM3,
                       TIM_INT_UPDATE
                       |TIM_INT_CC1
                       |TIM_INT_CC2
                       |TIM_INT_CC3
                       |TIM_INT_CC4);
  TIM_ClrIntPendingBit(TIM5,
                       TIM_INT_UPDATE
                       |TIM_INT_CC1
                       |TIM_INT_CC2
                       |TIM_INT_CC3
                       |TIM_INT_CC4);
  TIM_ClrIntPendingBit(TIM8,
                       TIM_INT_UPDATE
                       |TIM_INT_CC1
                       |TIM_INT_CC2
                       |TIM_INT_CC3
                       |TIM_INT_CC4);
  TIM_ClrIntPendingBit(TIM9,
                       TIM_INT_UPDATE
                       |TIM_INT_CC1
                       |TIM_INT_CC2
                       |TIM_INT_CC3
                       |TIM_INT_CC4);
  TIM_EnableCtrlPwmOutputs(TIM1, ENABLE);
  TIM_EnableCtrlPwmOutputs(TIM8, ENABLE);

  {
    const uint32_t TIMClk = TIM_Clk(TIM1, TIM_ADVANCED_PRESCALER);
    const uint16_t period = TIMClk / LED_PWM_CLOCK - 1;
    TIM_SetCmp1(TIM1, period / 4);
    TIM_SetCmp1(TIM8, period / 4);
  }
  {
    const uint32_t TIMClk = TIM_Clk(TIM2, TIM_GENERIC_PRESCALER);
    const uint16_t period = TIMClk / LED_PWM_CLOCK - 1;
    TIM_SetCmp1(TIM2, period / 4);
    TIM_SetCmp1(TIM3, period / 4);
    TIM_SetCmp3(TIM5, period / 4);
    TIM_SetCmp1(TIM9, period / 4);
  }

  
  // TIM_Enable(TIM2, ENABLE);
  TIM_Enable(TIM3, ENABLE);
  // TIM_Enable(TIM5, ENABLE);
  TIM_Enable(TIM8, ENABLE);
  // TIM_Enable(TIM9, ENABLE);

  // Use TIM1 as Master TIM, so it is enabled last
  TIM_Enable(TIM1, ENABLE);

}
