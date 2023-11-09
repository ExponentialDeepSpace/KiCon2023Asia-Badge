#include <n32l40x.h>
#include "setup.h"
#include "led.h"
#include "tim.h"
#include "sk6812.h"

uint16_t sk6812_0;
uint16_t sk6812_1;
uint16_t sk6812_p;

uint8_t sk6812_rgbs[10][3] = {
    {255, 255, 0}, {0, 255, 0}, {0, 0, 255}, {255, 0, 0}};
uint16_t sk6812_data[100][4] = {};

void TIM4_Config() {
  
  // TIM4_CH1(PB6) -> SAO_1_GPIO1
  // TIM4_CH2(PB7) -> SAO_1_GPIO2
  // TIM4_CH3(PB8) -> SAO_2_GPIO1
  // TIM4_CH4(PB9) -> SAO_2_GPIO2

  // Setup GPIO for TIM4
  GPIO_InitType initGPIO;
  TIMx_GPIO_InitConfig(&initGPIO);
  // Setup PB6/7/8/9 for TIM4_CH1/2/3/4
  initGPIO.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  initGPIO.GPIO_Alternate = GPIO_AF2_TIM4;
  GPIO_InitPeripheral(GPIOB, &initGPIO);

  // TIMx_PWM_Config(TIM4, (uint8_t)(CH1|CH2|CH3|CH4));

  TIM_Module *TIMx = TIM4;
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
  TIM_ConfigDma(TIMx, TIM_DMABASE_CAPCMPDAT1, TIM_DMABURST_LENGTH_4TRANSFERS);
  TIM_EnableDma(TIMx, TIM_DMA_UPDATE, ENABLE);

  // TIM_EnableUpdateEvt_r(TIMx, ENABLE);

  TIM_SelectSlaveMode(TIM4, TIM_SELECTED_SLAVE_MODE);
  TIM_SelectInputTrig(TIM4, TIM_TRIG_SEL_IN_TR0); // triggered from TIM1
}
