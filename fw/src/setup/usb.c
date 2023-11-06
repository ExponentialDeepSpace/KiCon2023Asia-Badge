#include <n32l40x.h>

#include "setup.h"
#include "usb.h"

static void USB_NVIC_Config() {
  NVIC_InitType nvicInit = {0};
  EXTI_InitType extiInit = {0};

  nvicInit.NVIC_IRQChannel = USB_LP_IRQn;
  nvicInit.NVIC_IRQChannelPreemptionPriority = (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
  nvicInit.NVIC_IRQChannelSubPriority = 0;
  nvicInit.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvicInit);

  nvicInit.NVIC_IRQChannel = USBWakeUp_IRQn;
  nvicInit.NVIC_IRQChannelPreemptionPriority = (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2);
  nvicInit.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvicInit);

  EXTI_ClrITPendBit(EXTI_LINE17);
  extiInit.EXTI_Line = EXTI_LINE17;
  extiInit.EXTI_Mode = EXTI_Mode_Interrupt;
  extiInit.EXTI_Trigger = EXTI_Trigger_Rising;
  extiInit.EXTI_LineCmd = ENABLE;
  EXTI_InitPeripheral(&extiInit);
}

void USBWakeUp_IRQHandler(void)
{
    EXTI_ClrITPendBit(EXTI_LINE17);
}

#define OSC300_CTRL         ((__IO unsigned*)(0x40001824))

#define _DisOsc300Ldo()    (*OSC300_CTRL = (*OSC300_CTRL) | 0x800000);
#define _EnOsc300Ldo()     (*OSC300_CTRL = (*OSC300_CTRL) & (~0x800000));
#define _DisOsc300Core()   (*OSC300_CTRL = (*OSC300_CTRL) | 0x400000);
#define _EnOsc300Core()    (*OSC300_CTRL = (*OSC300_CTRL) & (~0x400000));
#define _DisOsc300Ibias()  (*OSC300_CTRL = (*OSC300_CTRL) | 0x1000000);
#define _EnOsc300Ibias()   (*OSC300_CTRL = (*OSC300_CTRL) & (~0x1000000));

/* Pull up controller register */
#define DP_CTRL ((__IO unsigned*)(0x40001824))

#define _ClrDPCtrl() (*DP_CTRL = (*DP_CTRL) & (~0x8000000));
#define _EnPortPullup() (*DP_CTRL = (*DP_CTRL) | 0x02000000);
#define _DisPortPullup() (*DP_CTRL = (*DP_CTRL) & 0xFDFFFFFF);

void usb_dc_low_level_init(void) {
  // USB GPIO are enabled when USB module is enabled
  USB_NVIC_Config();

  _EnPortPullup();
  
#if defined (XTALLESS) && (XTALLESS == 1)
  // RCC_ConfigUCDRClk(RCC_UCDR300M_SRC_OSC300M, ENABLE);
  // RCC_ConfigUSBXTALESSMode(RCC_USBXTALESS_LESSMODE);

  RCC->APB1PCLKEN |= RCC_APB1PCLKEN_AFECEN;
  RCC->CFG3 &= RCC_UCDR300MSource_MASK;
  RCC->CFG3 |= RCC_UCDR300M_SRC_OSC300M;
  RCC->CFG3 |= RCC_USBXTALESS_LESSMODE;

  uint32_t time;
  
  _EnOsc300Ldo();
  time = 0x4000;
  while(time --);

  _EnOsc300Ibias();
  time = 0x4000;
  while(time --);

  _EnOsc300Core();
  time = 0x4000;
  while(time --);

  RCC->CFG3 |= RCC_UCDR_ENABLE;
  time = 0x4000;
  while(time --);
#else
  RCC_ConfigUsbClk(RCC_USBCLK_SRC_PLLCLK_DIV1);
#endif
  RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_USB, ENABLE);
}
