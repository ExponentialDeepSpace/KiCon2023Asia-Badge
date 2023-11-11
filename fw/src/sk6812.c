#include <n32l40x.h>
#include <FreeRTOSConfig.h>
#include "setup.h"
#include "led.h"
#include "tim.h"
#include "sk6812.h"

uint16_t sk6812_0;
uint16_t sk6812_1;
uint16_t sk6812_p;

void SK6812_Config() {
  NVIC_InitType nvicInit = {0};
  nvicInit.NVIC_IRQChannel = TIM4_IRQn;
  nvicInit.NVIC_IRQChannelPreemptionPriority = TIM_INTERRUPT_PRIORITY_LEVEL;
  nvicInit.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvicInit);

  // TIM4_CH1(PB6) -> SAO_1_GPIO1
  // TIM4_CH2(PB7) -> SAO_1_GPIO2
  // TIM4_CH3(PB8) -> SAO_2_GPIO1
  // TIM4_CH4(PB9) -> SAO_2_GPIO2

  assert_param(LED_SAO_GPIO_TIM == TIM4);
  // Setup GPIO for TIM4
  GPIO_InitType initGPIO;
  TIMx_GPIO_InitConfig(&initGPIO);
  // Setup PB6/7/8/9 for TIM4_CH1/2/3/4
  initGPIO.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  initGPIO.GPIO_Current = GPIO_DC_12mA;
  initGPIO.GPIO_Mode = GPIO_Mode_AF_PP;
  initGPIO.GPIO_Alternate = GPIO_AF2_TIM4;
  GPIO_InitPeripheral(GPIOB, &initGPIO);

  // TIMx_PWM_Config(TIM4, (uint8_t)(CH1|CH2|CH3|CH4));

  TIM_Module *TIMx = LED_SAO_GPIO_TIM;
  uint8_t channels = (uint8_t)(CH1|CH2|CH3|CH4);
  TIM_TimeBaseInitType timInit;
  OCInitType ocInit;

  TIM_InitTimBaseStruct(&timInit);
  TIM_InitOcStruct(&ocInit);
  uint32_t TIMClk = TIM_Clk(TIMx);

  // set prescaler for both TIM1 and TIM3 to be same TIMClk (24Mhz)
  uint16_t prescaler = IsTimList1Module(TIMx) ?
    TIM_ADVANCED_PRESCALER : TIM_GENERIC_PRESCALER;

  uint16_t period = TIMClk / (prescaler + 1) / SK6812_PWM_CLOCK - 1;
  timInit.Period = period;
  sk6812_0 = period / 4;
  sk6812_1 = period / 2;
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

#ifdef LED_SAO_USE_DMA
  // initilize sk6812 data
  for (uint16_t led = 0; led < COUNT; led ++) {
    uint32_t data_idx = 0;
    for (uint16_t seq = 0; seq < SEQLEN; seq ++) {
      uint8_t *rgb = &sk6812_rgbs[led][seq][0];
      uint8_t r = rgb[0];
      uint8_t g = rgb[1];
      uint8_t b = rgb[2];
      uint32_t data = g << 16 | r << 8| b;
      for (uint32_t i = 0; i < 24; i++) {
        sk6812_data[data_idx][led] = (data & (1 << (23 - i))) ? sk6812_1 : sk6812_0;
        data_idx ++;
        assert_param(data_idx <= PATTERN_LEN);
      }
    }
    // reset code 50us
    for (uint32_t seq = SEQLEN; seq < SEQLEN+2; seq ++) {
      for (uint32_t i = 0; i < 24; i++) {
        sk6812_data[data_idx][led] = 0;
        data_idx++;
        assert_param(data_idx <= PATTERN_LEN);
      }
    }
  }

  // TIM4 DMA
  DMA_InitType dmaInit;
  DMA_StructInit(&dmaInit);

  DMA_DeInit(LED_SAO_TIM_DMA_CH);
  dmaInit.PeriphAddr = (uint32_t)&TIMx->DADDR;
  dmaInit.PeriphInc = DMA_PERIPH_INC_DISABLE;
  dmaInit.PeriphDataSize = DMA_PERIPH_DATA_SIZE_HALFWORD;
  dmaInit.Direction = DMA_DIR_PERIPH_DST;
  dmaInit.MemAddr = (uint32_t)&sk6812_data;
  dmaInit.Mem2Mem = DMA_M2M_DISABLE;
  dmaInit.BufSize = sizeof(sk6812_data) / sizeof(sk6812_data[0][0]);
  dmaInit.MemDataSize = DMA_MemoryDataSize_HalfWord;
  dmaInit.DMA_MemoryInc = DMA_MEM_INC_ENABLE;
  dmaInit.CircularMode = DMA_MODE_CIRCULAR;
  dmaInit.Priority = DMA_PRIORITY_MEDIUM;

  DMA_Init(LED_SAO_TIM_DMA_CH, &dmaInit);

  DMA_ConfigInt(LED_SAO_TIM_DMA_CH, DMA_INT_HTX|DMA_INT_TXC|DMA_INT_ERR, ENABLE);
  DMA_RequestRemap(LED_SAO_TIM_DMA_REMAP, DMA, LED_SAO_TIM_DMA_CH, ENABLE);
  TIM_ConfigDma(TIMx, TIM_DMABASE_CAPCMPDAT1, TIM_DMABURST_LENGTH_4TRANSFERS);
  DMA_EnableChannel(LED_SAO_TIM_DMA_CH, ENABLE);
  
  TIM_EnableDma(TIMx, TIM_DMA_UPDATE, ENABLE);

  // TIM_EnableUpdateEvt_r(TIMx, ENABLE);
#else
  TIM_SetCmp1(LED_SAO_GPIO_TIM, 0); // SAO1 GPIO1
  TIM_SetCmp2(LED_SAO_GPIO_TIM, 0); // SAO1 GPIO2
  TIM_SetCmp3(LED_SAO_GPIO_TIM, 0); // SAO2 GPIO1
  TIM_SetCmp4(LED_SAO_GPIO_TIM, 0); // SAO2 GPIO2

#endif // LED_SAO_USE_DMA
  TIM_SetCnt(LED_SAO_GPIO_TIM, 0);

  TIM_ConfigInt(TIMx, TIM_INT_UPDATE, ENABLE);

  TIM_SelectSlaveMode(TIMx, TIM_SELECTED_SLAVE_MODE);
  TIM_SelectInputTrig(TIMx, TIM_TRIG_SEL_IN_TR0); // triggered from TIM1
}

#ifdef LED_SAO_USE_DMA
void LED_SAO_DMA_IRQHandler(void) {
  if (DMA_GetIntStatus(LED_SAO_TIM_DMA_INT_TXC, DMA)) {
  }

  DMA_ClrIntPendingBit(LED_SAO_TIM_DMA_INT_GLB | LED_SAO_TIM_DMA_INT_TXC |
                       LED_SAO_TIM_DMA_INT_HTX | LED_SAO_TIM_DMA_INT_ERR,
                       DMA);
}
#else
typedef (*TIM_SetCmpFunc)(TIM_Module* TIMx, uint16_t Compare1);
TIM_SetCmpFunc TIM_SetCmpFuncs[LED_COUNT] = {
    TIM_SetCmp1,
    TIM_SetCmp2,
    TIM_SetCmp3,
    TIM_SetCmp4,
};;

#endif // LED_SAO_USE_DMA

void LED_SAO_TIM_IRQHandler(void) {
  TIM_ClrIntPendingBit(LED_SAO_GPIO_TIM, TIM_INT_UPDATE);

  static uint16_t pwm_period = 0;
  static uint16_t pwm_direction = 1;
  
  for (uint16_t led = 0; led < LED_COUNT; led ++) {
    // SAO1 GPIO1 and SAO2 GPIO1 use SK6812
    if (0 == led || 2 == led) {
      static uint32_t delay = 0;
      static uint16_t seq = 0;
      static uint16_t bit = 0;

      uint16_t period = 0;
      if (seq < SEQLEN) {
        uint8_t *rgb = &sk6812_rgbs[led][seq][0];
        uint8_t r = rgb[0];
        uint8_t g = rgb[1];
        uint8_t b = rgb[2];
        uint32_t data = g << 16 | r << 8 | b;

        period = (data & (1 << (23 - bit))) ? sk6812_1 : sk6812_0;
      } else if (seq < SEQLEN + SEQ_RST_LEN) {
        period = 0;
      }

      TIM_SetCmpFuncs[led](LED_SAO_GPIO_TIM, period);

#define MAX_DELAY (200)
      delay ++;
      bit ++;
      if (bit == 24) {
        bit = 0;
        if (delay >= MAX_DELAY) {
          seq++;
          delay = 0;
        }

        if (seq == SEQLEN + 2) {
          seq = 0;
        }
      }

    } else { // SAO1 GPIO2 and SAO2 GPIO2 use PWM
      static uint32_t delay = 0;
      TIM_SetCmpFuncs[led](LED_SAO_GPIO_TIM, pwm_period);
      uint16_t period = TIM_GetAutoReload(LED_SAO_GPIO_TIM);
#define PWM_MAX_DELAY (2000)
      delay ++;
      if (delay >= PWM_MAX_DELAY) {
        delay = 0;
        if (pwm_period == period) {
          pwm_direction = -1;
        } else if (pwm_period == 0) {
          pwm_direction = 1;
        }
        pwm_period += pwm_direction;      
      }
    }
  }


}
