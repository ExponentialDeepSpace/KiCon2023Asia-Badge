#include <n32l40x.h>

#include "setup.h"
#include "led.h"

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

/*
 * LowSide_B_Col_1 -> TIM3_CH1(PA6)                         -------     -------     -------
 * LowSide_R_Col_1 -> TIM3_CH2(PA7)                         |     |    /     /      | D10 |
 * LowSide_G_Col_1 -> TIM3_CH3(PB0)                         | D2  |   / D6  /       |_____|
 * HighSide_Row_1  -> TIM9_CH4(PB15) -> D2                  |     |  /     /        
 * HighSide_Row_2  -> PA15           -> D3                  |     | /     /         
 * HighSide_Row_3  -> TIM5_CH4(PA3) -> D4                   |     |/     /           _____          ______       ______          _______
 * HighSide_Row_4  -> TIM5_CH3(PA2) -> D5                   | D3     D5 |           |     |        /      |     /      \        |  D12  |
 * HighSide_Row_5  -> TIM9_CH3(PB14) -> D6                  |     |\     \          | D9  |       /      /     /        \       |   _   |
 * HighSide_Row_6  -> TIM2_CH2(PA1)  -> D7                  |     | \     \         |     |      /  _ _ /     /          \      |  | |  |
 * HighSide_Row_7  -> TIM9_CH1(PB12) -> D10                 |     |  \     \        |     |     /   | D11    |    D13     |     |  | |  |
 * HighSide_Row_8  -> TIM9_CH2(PB13) -> D9                  |     |   \     \       |     |     \   |        |            |     |  | |  |
 * HighSide_Row_9  -> TIM2_CH1(PA0)  -> D8                  |     |    \     \      |     |      \  ----      \          /      |  | |  |
 * HighSide_Row_10 -> TIM2_CH4(PB11) -> D11                 | D4  |     \ D7  \     | D8  |       \     \      \        /       |  | |  |
 * HighSide_Row_11 -> TIM2_CH3(PB10) -> D13                 |     |      \     \    |     |        \_____\__    \______/        |__| |__|
 * HighSide_Row_12 -> TIM3_CH4(PB2)  -> D12                 -------       -------   -------
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

#if 0
void DMA_Channel1_IRQHandler(void) {
  int i = 0;
  i++;

  DMA_ClrIntPendingBit(DMA_INT_GLB1|DMA_INT_TXC1|DMA_INT_HTX1|DMA_INT_ERR1, DMA);
}
#endif

void LED_Config(void) {
    LED_GPIO_Config();

    for (int i = 0;
         i < sizeof(GPIOs_HighSide_Rows) / sizeof(GPIOs_HighSide_Rows[0]);
         i++) {
      GPIO_ResetBits(GPIOs_HighSide_Rows[i].port, GPIOs_HighSide_Rows[i].pin);
    }
}

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

    TIM_SetCmp1(LED_LowSide_TIM, r); // B
    TIM_SetCmp2(LED_LowSide_TIM, g); // R
    TIM_SetCmp3(LED_LowSide_TIM, b); // G
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

