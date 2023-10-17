#include <n32l40x.h>

#include "setup.h"
#include "led.h"

#define TIM_SELECTED_SLAVE_MODE TIM_SLAVE_MODE_GATED

typedef struct GPIO_HighSide_Def {
  GPIO_Module *port;
  uint16_t pin;
}GPIO_HighSide_Def;

static const GPIO_HighSide_Def GPIOs_HighSide_Rows[12] = {
  {LED_ROW_1_PORT, LED_ROW_1_PIN}, // Row 1
  {LED_ROW_2_PORT, LED_ROW_2_PIN}, // Row 2
  {LED_ROW_3_PORT, LED_ROW_3_PIN}, // Row 3
  {LED_ROW_4_PORT, LED_ROW_4_PIN}, // Row 4
  {LED_ROW_5_PORT, LED_ROW_5_PIN}, // Row 5
  {LED_ROW_6_PORT, LED_ROW_6_PIN}, // Row 6
  {LED_ROW_7_PORT, LED_ROW_7_PIN}, // Row 7
  {LED_ROW_9_PORT, LED_ROW_9_PIN}, // Row 9
  {LED_ROW_8_PORT, LED_ROW_8_PIN}, // Row 8
  {LED_ROW_10_PORT, LED_ROW_10_PIN}, // Row 10
  {LED_ROW_11_PORT, LED_ROW_11_PIN}, // Row 11
  {LED_ROW_12_PORT, LED_ROW_12_PIN}, // Row 12
};

#define MAX_ROWS (sizeof(GPIOs_HighSide_Rows) / sizeof(GPIOs_HighSide_Rows[0]))

typedef enum RGB {
  B = 0,
  R = 1,
  G = 2,
} RGB;

uint16_t periods[MAX_ROWS][3] = {0};

static void LED_NVIC_Config() {
  NVIC_InitType nvicInit = {0};
  nvicInit.NVIC_IRQChannel = TIM1_UP_IRQn;
  nvicInit.NVIC_IRQChannelPreemptionPriority = TIM_INTERRUPT_PRIORITY_LEVEL;
  nvicInit.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvicInit);

#if 0
  nvicInit.NVIC_IRQChannel = DMA_Channel1_IRQn;
  nvicInit.NVIC_IRQChannelPreemptionPriority = TIM_DMA_INTERRUPT_PRIORITY_LEVEL;
  nvicInit.NVIC_IRQChannelCmd = ENABLE;

  NVIC_Init(&nvicInit);

  nvicInit.NVIC_IRQChannel = TIM1_CC_IRQn;
  nvicInit.NVIC_IRQChannelPreemptionPriority = TIM_INTERRUPT_PRIORITY_LEVEL;
  nvicInit.NVIC_IRQChannelCmd = ENABLE;

  NVIC_Init(&nvicInit);

  nvicInit.NVIC_IRQChannel = TIM3_IRQn;
  nvicInit.NVIC_IRQChannelPreemptionPriority = TIM_INTERRUPT_PRIORITY_LEVEL;
  nvicInit.NVIC_IRQChannelCmd = ENABLE;

  NVIC_Init(&nvicInit);
#endif
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

static uint32_t TIM_Clk(TIM_Module *TIMx) {

  uint32_t TIMClk = 0;

  if (IsTimList1Module(TIMx)) {
    uint32_t APB2Clk = 0;
    uint32_t APB2Cfg = RCC->CFG & RCC_CFG_APB2PRES;
    // TIM1, TIM8 clk from APB2 (
    if (0 == (APB2Cfg & RCC_CFG_APB2PRES_2)) {
      APB2Clk = SystemCoreClock;
      TIMClk = APB2Clk;
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
      TIMClk = APB2Clk * 2;
    }
  } else if (IsTimList3Module(TIMx) && !IsTimList1Module(TIMx)) {
    // TIM 2, 3, 4, 5, 9 clk from APB1
    uint32_t APB1Clk = 0;
    uint32_t APB1Cfg = RCC->CFG & RCC_CFG_APB1PRES;
    if (0 == (APB1Cfg & RCC_CFG_APB1PRES_2)) {
      APB1Clk = SystemCoreClock;
      TIMClk = APB1Clk;
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
      TIMClk = APB1Clk * 2;
    }
  }

  return TIMClk;
}

// in default configuration (system_n32l40x)
// HCLK = SYSCLK = 48Mhz (for USB)
// PCLK2 = HCLK / 2 = 24Mhz
// PCLK1 = HCLK / 4 = 12Mhz
// TIM1,8 CLK = PCLK2 * 2 = 48Mhz
// TIM2,3,4,5,6,7,9 CLK = PCLK1 * 2 = 24Mhz

static void TIMx_PWM_Config(TIM_Module *TIMx, uint8_t channels) {
  TIM_TimeBaseInitType timInit;
  OCInitType ocInit;

  TIM_InitTimBaseStruct(&timInit);
  TIM_InitOcStruct(&ocInit);
  uint32_t TIMClk = TIM_Clk(TIMx);

  // set prescaler for both TIM1 and TIM3 to be same TIMClk (24Mhz)
  uint16_t prescaler = IsTimList1Module(TIMx) ?
    TIM_ADVANCED_PRESCALER : TIM_GENERIC_PRESCALER;

  uint16_t period = TIMClk / (prescaler + 1) / LED_PWM_CLOCK - 1;
  timInit.Period = period;
  timInit.Prescaler = prescaler;
  timInit.CntMode = TIM_CNT_MODE_UP;
  TIM_InitTimeBase(TIMx, &timInit);

  ocInit.OcMode = TIM_OCMODE_PWM1;
  ocInit.OutputState = TIM_OUTPUT_STATE_ENABLE;
  ocInit.Pulse = 0;

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
  TIM_ConfigInt(TIM1, TIM_INT_UPDATE|TIM_INT_CC1, ENABLE);

  // Master Trigger
  TIM_SelectOutputTrig(TIM1, TIM_TRGO_SRC_ENABLE);
  TIM_SelectMasterSlaveMode(TIM1, TIM_MASTER_SLAVE_MODE_ENABLE);

  TIM1->REPCNT = 0;
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
  TIM_ConfigInt(TIM1, TIM_INT_UPDATE|TIM_INT_CC1, ENABLE);

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

// Cannot reload TIM3 from DMA by TIM1 triggering
// for 2.0 revision, just load TIM3 by CPU
#if 0
static void
TIM_DMA_Config() {
  DMA_InitType dmaInit;
  DMA_StructInit(&dmaInit);

  DMA_DeInit(DMA_CH1);
  dmaInit.PeriphAddr = (uint32_t)&TIM3->DADDR;
  dmaInit.PeriphInc = DMA_PERIPH_INC_DISABLE;
  dmaInit.PeriphDataSize = DMA_PERIPH_DATA_SIZE_HALFWORD;
  dmaInit.Direction = DMA_DIR_PERIPH_DST;
  dmaInit.MemAddr = (uint32_t)&periods;
  dmaInit.Mem2Mem = DMA_M2M_DISABLE;
  dmaInit.BufSize = sizeof(periods) / sizeof(periods[0][0]);
  dmaInit.MemDataSize = DMA_MemoryDataSize_HalfWord;
  dmaInit.DMA_MemoryInc = DMA_MEM_INC_ENABLE;
  dmaInit.CircularMode = DMA_MODE_CIRCULAR;
  dmaInit.Priority = DMA_PRIORITY_HIGH;

  DMA_Init(DMA_CH1, &dmaInit);

  DMA_ConfigInt(DMA_CH1, DMA_INT_HTX1|DMA_INT_TXC1|DMA_INT_ERR1, ENABLE);
  DMA_RequestRemap(DMA_REMAP_TIM1_UP, DMA, DMA_CH1, ENABLE);
  TIM_ConfigDma(TIM3, TIM_DMABASE_CAPCMPDAT1, TIM_DMABURST_LENGTH_3TRANSFERS);
  DMA_EnableChannel(DMA_CH1, ENABLE);
  TIM_EnableDma(TIM1, TIM_DMA_UPDATE, ENABLE);
}
#endif

static void LED_GPIO_Config() {
  GPIO_InitType initGPIO;
  GPIO_InitStruct(&initGPIO);
  initGPIO.GPIO_Mode = GPIO_Mode_Out_PP;
  initGPIO.GPIO_Current = GPIO_DC_8mA;

  for (int i = 0;
       i < sizeof(GPIOs_HighSide_Rows) / sizeof(GPIOs_HighSide_Rows[0]); i++) {

    GPIO_HighSide_Def gpio = GPIOs_HighSide_Rows[i];
    initGPIO.Pin = gpio.pin;

    GPIO_InitPeripheral(gpio.port, &initGPIO);
  }
}

void TIM_Config() {
  LED_GPIO_Config();

  for (int i = 0;
       i < sizeof(GPIOs_HighSide_Rows) / sizeof(GPIOs_HighSide_Rows[0]);
       i++) {
    GPIO_ResetBits(GPIOs_HighSide_Rows[i].port, GPIOs_HighSide_Rows[i].pin);
  }

  TIM1_Config();
  // TIM2_Config();
  TIM3_Config();
  // TIM5_Config();
  TIM8_Config();
  // TIM9_Config();

  // TIM_DMA_Config();
  LED_NVIC_Config();
  
  // DMA_ClrIntPendingBit(DMA_INT_GLB1|DMA_INT_TXC1|DMA_INT_HTX1|DMA_INT_ERR1, DMA);

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

  const uint16_t period = TIM_GetAutoReload(LED_Main_TIM);

  /*
  for (int i = 0; i < sizeof(periods) / sizeof(periods[0]); i++) {
    periods[i][B] = period / ((i % 2) ? (1) : (8));
    periods[i][R] = period / ((i % 2) ? (8) : (1));
    periods[i][G] = 0;
  }

  TIM_SetCmp1(TIM3, periods[0][B]); // B
  TIM_SetCmp2(TIM3, periods[0][R]); // R
  TIM_SetCmp3(TIM3, periods[0][G]); // G
  */

  TIM_SetCmp1(LED_LowSide_TIM, 0); // B
  TIM_SetCmp2(LED_LowSide_TIM, 0); // R
  TIM_SetCmp3(LED_LowSide_TIM, 0); // G
  
  // TIM_Enable(TIM2, ENABLE);
  TIM_Enable(LED_LowSide_TIM, ENABLE);
  // TIM_Enable(TIM5, ENABLE);
  TIM_Enable(TIM8, ENABLE);
  // TIM_Enable(TIM9, ENABLE);

  TIM_SetCnt(LED_Main_TIM, period + 1);
  TIM_GenerateEvent(LED_Main_TIM, TIM_EVTGEN_UDGN);
  // Use TIM1 as Master TIM, so it is enabled last
  TIM_Enable(LED_Main_TIM, ENABLE);
}

#if 0
void DMA_Channel1_IRQHandler(void) {
  int i = 0;
  i++;

  DMA_ClrIntPendingBit(DMA_INT_GLB1|DMA_INT_TXC1|DMA_INT_HTX1|DMA_INT_ERR1, DMA);
}
#endif

void LED_TIM_UP_IRQHandler(void) {
  TIM_ClrIntPendingBit(LED_Main_TIM, TIM_INT_UPDATE);

  static uint32_t speed = 0;

  // counting for current first (brightest) LED
  static int head = 0;

  // current/prev _row is recording the all ROWS running during TIM PWM cycles
  // running from 0 (row 1) to MAX_ROWS-1 (row 12) as one cycle
  static int current_row = 0; // current
  static int prev_row = MAX_ROWS-1; // prev

  // set prev index GPIO to low level
  GPIO_ResetBits(GPIOs_HighSide_Rows[prev_row].port, GPIOs_HighSide_Rows[prev_row].pin);
  GPIO_SetBits(GPIOs_HighSide_Rows[current_row].port, GPIOs_HighSide_Rows[current_row].pin);

  const uint16_t period = TIM_GetAutoReload(LED_Main_TIM);

  // how many led behid current_index LED will be dimming
  const uint16_t trail = 5;
    
  // D10 Row 7 is always lit
  if ( 7 - 1 == current_row ) {

    TIM_SetCmp1(LED_LowSide_TIM, 0); // B
    TIM_SetCmp2(LED_LowSide_TIM, period); // R
    TIM_SetCmp3(LED_LowSide_TIM, period); // G
  }
  else {
    uint16_t r = 0, g = 0, b = 0;
    int16_t m = head - current_row;
    if (m > trail && head >= MAX_ROWS) {
      m = head - MAX_ROWS - current_row;
    }
    if (0 <= m && m < trail) {
      r = period / (1 << m);
      g = r;
      b = r;
    }

    TIM_SetCmp1(LED_LowSide_TIM, b); // B
    TIM_SetCmp2(LED_LowSide_TIM, r); // R
    TIM_SetCmp3(LED_LowSide_TIM, g); // G
  }
  
  current_row++; prev_row++;

  if (current_row >= MAX_ROWS) {
    current_row = 0;
    speed ++;
    if(speed > LED_SPEED) {
      speed = 0;
      head ++;
      if (7 - 1 == head) {
        head++;
      }
      if (head >= MAX_ROWS + trail) {
        head = trail;
      }
    }
  }

  if (prev_row > MAX_ROWS - 1) {
    prev_row = 0;
  }
}

