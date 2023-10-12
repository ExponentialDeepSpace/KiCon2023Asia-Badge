#include "memory_in_pixel_display.h"

#include <n32l40x.h>

#include <string.h>
#include <assert.h>
#include "setup.h"

#define LINE_PREFIX_BYTES (2)
#define PIXEL_SIZE_BITS (4) // use 4-bit mode
#define LINE_WIDTH_BYTES (LINE_PREFIX_BYTES + DISP_WIDTH * PIXEL_SIZE_BITS / 8)
#define LINE_WIDTH_HALFWORDS (LINE_WIDTH_BYTES / 2)
#define DISP_BUFFER_PADDING_HALFWORD 1
#define M0_MASK 0x8000
#define M1_MASK 0x4000
#define M2_MASK 0x2000
#define M3_MASK 0x1000
#define M4_MASK 0x0800
#define M5_MASK 0x0400
#define AG_MASK 0x03ff

#define CMD_UPDATE_MULTIPLE_LINES_4BIT (M0_MASK|M3_MASK)
#define CMD_NO_UPDATE (M0_MASK|M2_MASK)
#define CMD_ALL_CLEAR (M2_MASK)

// each line has following format
// M0-M5 AG9-AG0 DATA(D1R D1G D1B ...)
uint16_t DisplayBuffer[ LINE_WIDTH_HALFWORDS * DISP_HEIGHT
                        + DISP_BUFFER_PADDING_HALFWORD ];

void DisplayBufferInit() {
  memset(DisplayBuffer, 0, sizeof(DisplayBuffer));
  for (uint8_t row = 1; row <= DISP_HEIGHT; row++) {
    DisplayBuffer[row * LINE_WIDTH_HALFWORDS] = CMD_UPDATE_MULTIPLE_LINES_4BIT | row;
  }
}

void DisplayTransferLines(uint8_t first_line, uint8_t lines) {
  assert(1 <= first_line && first_line < DISP_HEIGHT);
  assert(lines >= 1);

  DMA_ClrIntPendingBit(
                       DISPLAY_DMA_INT_GLB
                       |DISPLAY_DMA_INT_TXC
                       |DISPLAY_DMA_INT_HTX
                       |DISPLAY_DMA_INT_ERR,
                       DMA);
  uint16_t *buf = DisplayBuffer + (first_line - 1) * LINE_WIDTH_HALFWORDS;
  DISPLAY_SPI_DMA_CHANNEL->MADDR = (uint32_t)buf;
  // need to initiate SPI transfer first
  DISPLAY_SPI_DMA_CHANNEL->TXNUM = lines * LINE_WIDTH_HALFWORDS - 1;

  DMA_EnableChannel(DISPLAY_SPI_DMA_CHANNEL, ENABLE);

  // SPI TE flag is set by default
  SPI_Enable(DISPLAY_SPI, ENABLE);
  SPI_I2S_TransmitData(DISPLAY_SPI, DisplayBuffer[0 + first_line * LINE_WIDTH_HALFWORDS]);
}

static void Display_GPIO_Config() {
  // Setup GPIO for SPI2
  GPIO_InitType initValue;
  GPIO_InitStruct(&initValue);
  initValue.GPIO_Current = GPIO_DC_8mA;
  initValue.GPIO_Mode = GPIO_Mode_AF_OD;
  initValue.GPIO_Pull = GPIO_Pull_Up;

  // Setup PC7/9 for SPI2_SCK/MOSI
  initValue.Pin = GPIO_PIN_7 | GPIO_PIN_9;
  initValue.GPIO_Alternate = GPIO_AF5_SPI2;
  GPIO_InitPeripheral(GPIOC, &initValue);

  // Setup PA15 for SPI2_nSS
  initValue.Pin = GPIO_PIN_15;
  initValue.GPIO_Alternate = GPIO_AF1_SPI2;
  GPIO_InitPeripheral(GPIOA, &initValue);
}

static void Display_SPI_Config() {
  SPI_I2S_DeInit(DISPLAY_SPI);
  
  SPI_InitType initSPI;
  SPI_InitStruct(&initSPI);

  // PCLK1 is 12Mhz
  // Display has max 2Mhz frequency
  // set PCLK1 / 8 = 1.5Mhz
  initSPI.SpiMode = SPI_MODE_MASTER;
  initSPI.DataDirection = SPI_DIR_SINGLELINE_TX;
  initSPI.BaudRatePres = SPI_BR_PRESCALER_8;
  initSPI.CLKPOL = SPI_CLKPOL_LOW;
  initSPI.CLKPHA = SPI_CLKPHA_FIRST_EDGE;
  initSPI.NSS = SPI_NSS_HARD;
  initSPI.FirstBit = SPI_FB_MSB;
  initSPI.DataLen = SPI_DATA_SIZE_16BITS;

  SPI_Init(DISPLAY_SPI, &initSPI);

  SPI_I2S_EnableInt(DISPLAY_SPI,
                    SPI_I2S_INT_TE
                    |SPI_I2S_INT_RNE
                    |SPI_I2S_INT_ERR,
                    ENABLE);
}

static void Display_DMA_Config() {
  DMA_DeInit(DISPLAY_SPI_DMA_CHANNEL);

  DMA_InitType dmaInit;
  DMA_StructInit(&dmaInit);

  dmaInit.PeriphAddr = (uint32_t)&DISPLAY_SPI->DAT;
  dmaInit.PeriphInc = DMA_PERIPH_INC_DISABLE;
  dmaInit.PeriphDataSize = DMA_PERIPH_DATA_SIZE_HALFWORD;
  dmaInit.Direction = DMA_DIR_PERIPH_DST;
  dmaInit.MemAddr = (uint32_t)&DisplayBuffer;
  dmaInit.BufSize = sizeof(DisplayBuffer) / sizeof(DisplayBuffer[0]);
  dmaInit.Mem2Mem = DMA_M2M_DISABLE;
  dmaInit.MemDataSize = DMA_MemoryDataSize_HalfWord;
  dmaInit.DMA_MemoryInc = DMA_MEM_INC_ENABLE;
  dmaInit.CircularMode = DMA_MODE_CIRCULAR;
  dmaInit.Priority = DMA_PRIORITY_HIGH;

  DMA_Init(DISPLAY_SPI_DMA_CHANNEL, &dmaInit);
  DMA_ConfigInt(DISPLAY_SPI_DMA_CHANNEL, DMA_INT_HTX|DMA_INT_TXC|DMA_INT_ERR, ENABLE);
  DMA_RequestRemap(DISPLAY_SPI_DMA_CHANNEL_REMAP, DMA, DISPLAY_SPI_DMA_CHANNEL, ENABLE);
  DMA_ClrIntPendingBit(
                       DISPLAY_DMA_INT_GLB
                       |DISPLAY_DMA_INT_TXC
                       |DISPLAY_DMA_INT_HTX
                       |DISPLAY_DMA_INT_ERR,
                       DMA);
}

static void Display_NVIC_Config() {
  NVIC_InitType nvicInit = {0};
  nvicInit.NVIC_IRQChannel = DMA_Channel2_IRQn;
  nvicInit.NVIC_IRQChannelPreemptionPriority = DISPLAY_SPI_DMA_INTERRUPT_PRIORITY_LEVEL;
  nvicInit.NVIC_IRQChannelCmd = ENABLE;

  NVIC_Init(&nvicInit);

  nvicInit.NVIC_IRQChannel = SPI2_IRQn;
  nvicInit.NVIC_IRQChannelCmd = ENABLE;
  nvicInit.NVIC_IRQChannelPreemptionPriority = DISPLAY_SPI_INTERRUPT_PRIORITY_LEVEL;

  NVIC_Init(&nvicInit);
}

void Display_Config() {
  Display_GPIO_Config();
  Display_SPI_Config();
  Display_DMA_Config();
  Display_NVIC_Config();

  // DMA_EnableChannel(DISPLAY_SPI_DMA_CHANNEL, ENABLE);
  SPI_I2S_EnableDma(DISPLAY_SPI, SPI_I2S_DMA_TX, ENABLE);
}
