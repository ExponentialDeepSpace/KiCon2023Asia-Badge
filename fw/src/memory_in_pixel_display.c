#include "memory_in_pixel_display.h"

#include <n32l40x.h>

#include <string.h>
#include <assert.h>
#include "setup.h"

#define LINE_PREFIX_BYTES (2)
#define PIXEL_SIZE_BITS (4) // use 4-bit mode
#define LINE_WIDTH_BYTES (LINE_PREFIX_BYTES + DISP_WIDTH * PIXEL_SIZE_BITS / 8)
#define LINE_WIDTH_HALFWORDS (LINE_WIDTH_BYTES / 2)
#define DISP_BUFFER_PADDING_HALFWORD (1)
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

uint16_t DisplayBuffer2[ LINE_WIDTH_HALFWORDS * DISP_HEIGHT
                        + DISP_BUFFER_PADDING_HALFWORD ];

const uint16_t cmd_no_update = CMD_NO_UPDATE;
const uint16_t cmd_all_clear = CMD_ALL_CLEAR;
const uint16_t cmd_color_blink_white = M3_MASK|M4_MASK;
TaskHandle_t DisplayTaskHandle = NULL;
StaticTask_t xDisplayTaskBuffer;
StackType_t xDisplayStack[ DISPLAY_STACK_SIZE ];


void DisplayBufferInit() {
  memset(DisplayBuffer, 0x0000, sizeof(DisplayBuffer));
  for (uint8_t row = 1; row <= DISP_HEIGHT; row++) {
    DisplayBuffer[(row - 1) * LINE_WIDTH_HALFWORDS] = row;
  }

  DisplayBuffer[DISP_HEIGHT * LINE_WIDTH_HALFWORDS] = 0; // last dummy data


  memset(DisplayBuffer2, 0xaaaa, sizeof(DisplayBuffer2));
}

void DisplayStartupSequence() {
  // Power up
  GPIO_ResetBits(DISP_SS_PORT, DISP_SS_PIN);
  GPIO_ResetBits(GPIOC, GPIO_PIN_8);
  vTaskDelay(1 / portTICK_PERIOD_MS);

  // All Clear
  SPI_Enable(DISPLAY_SPI, ENABLE);
  GPIO_SetBits(DISP_SS_PORT, DISP_SS_PIN);
  SPI_I2S_TransmitData(
      DISPLAY_SPI, &cmd_all_clear);

  while(SPI_I2S_GetStatus(DISPLAY_SPI, SPI_I2S_BUSY_FLAG));
  GPIO_ResetBits(DISP_SS_PORT, DISP_SS_PIN);
  SPI_Enable(DISPLAY_SPI, DISABLE);
  vTaskDelay(100 / portTICK_PERIOD_MS);

  // DISP
  GPIO_SetBits(DISP_DISP_PORT, DISP_DISP_PIN);
  vTaskDelay(1 / portTICK_PERIOD_MS);

  SPI_Enable(DISPLAY_SPI, DISABLE);
}

void DisplayTransferLines_DMA(uint8_t first_row, uint8_t rows) {
  assert(1 <= first_row && first_row <= DISP_HEIGHT);
  assert(rows >= 1 && (first_row + rows - 1) <= DISP_HEIGHT);

  DMA_ClrIntPendingBit(
                       DISPLAY_DMA_INT_GLB
                       |DISPLAY_DMA_INT_TXC
                       |DISPLAY_DMA_INT_HTX
                       |DISPLAY_DMA_INT_ERR,
                       DMA);

  // DISPLAY_SPI_DMA_CHANNEL->MADDR = (uint32_t)buf;
  // need to initiate SPI transfer first, but it needs 1 halfword dummy data
  // DISPLAY_SPI_DMA_CHANNEL->TXNUM = rows * LINE_WIDTH_HALFWORDS;


  // The display uses High Level for SS
  GPIO_SetBits(DISP_SS_PORT, DISP_SS_PIN);
  {
    volatile int i = 200;
    while (i--)
      ;
  }
  // SPI TE flag is set by default
  // uint16_t first_cmd = CMD_UPDATE_MULTIPLE_LINES_4BIT | first_row;
  DisplayBuffer[(first_row - 1) * LINE_WIDTH_HALFWORDS] = CMD_UPDATE_MULTIPLE_LINES_4BIT | first_row;
  const uint16_t last_row = first_row + rows;
  DisplayBuffer[(last_row - 1) * LINE_WIDTH_HALFWORDS] = 0; // Dummy Data

  uint16_t *buf = DisplayBuffer + (first_row - 1) * LINE_WIDTH_HALFWORDS;

  DMA_InitType dmaInit;
  DMA_StructInit(&dmaInit);

  dmaInit.PeriphAddr = (uint32_t)&DISPLAY_SPI->DAT;
  dmaInit.PeriphInc = DMA_PERIPH_INC_DISABLE;
  dmaInit.PeriphDataSize = DMA_PERIPH_DATA_SIZE_HALFWORD;
  dmaInit.Direction = DMA_DIR_PERIPH_DST;
  dmaInit.MemAddr = (uint32_t)buf;
  dmaInit.BufSize = (rows * LINE_WIDTH_HALFWORDS + 1) * 2;
  dmaInit.Mem2Mem = DMA_M2M_DISABLE;
  dmaInit.MemDataSize = DMA_MemoryDataSize_HalfWord;
  dmaInit.DMA_MemoryInc = DMA_MEM_INC_ENABLE;
  dmaInit.CircularMode = DMA_MODE_NORMAL;
  dmaInit.Priority = DMA_PRIORITY_HIGH;

  DMA_Init(DISPLAY_SPI_DMA_CHANNEL, &dmaInit);

  SPI_Enable(DISPLAY_SPI, ENABLE);
  SPI_I2S_EnableDma(DISPLAY_SPI, SPI_I2S_DMA_TX, ENABLE);
  DMA_RequestRemap(DISPLAY_SPI_DMA_CHANNEL_REMAP, DMA, DISPLAY_SPI_DMA_CHANNEL, ENABLE);
  DMA_EnableChannel(DISPLAY_SPI_DMA_CHANNEL, ENABLE);

  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  while(1) {
    vTaskDelay(1);
    if (0 == SPI_I2S_GetStatus(DISPLAY_SPI, SPI_I2S_BUSY_FLAG) &&
        0 != SPI_I2S_GetStatus(DISPLAY_SPI, SPI_I2S_TE_FLAG)) {
      break;
    }
  }
  {
    volatile int i = 200;
    while (i--)
      ;
  }
  GPIO_ResetBits(DISP_SS_PORT, DISP_SS_PIN);
  DMA_EnableChannel(DISPLAY_SPI_DMA_CHANNEL, DISABLE);
  SPI_Enable(DISPLAY_SPI, DISABLE);
  SPI_I2S_EnableDma(DISPLAY_SPI, SPI_I2S_DMA_TX, DISABLE);

  DisplayBuffer[(first_row - 1) * LINE_WIDTH_HALFWORDS] = first_row;
  DisplayBuffer[(last_row - 1) * LINE_WIDTH_HALFWORDS] = last_row;
}

void DisplayTransferLines_Poll(uint8_t first_row, uint8_t rows) {
  assert(1 <= first_row && first_row <= DISP_HEIGHT);
  assert(rows >= 1 && (first_row + rows - 1) <= DISP_HEIGHT);

  // The display uses High Level for SS
  GPIO_SetBits(DISP_SS_PORT, DISP_SS_PIN);
  {
    volatile int i = 200;
    while (i--)
      ;
  }
  // SPI TE flag is set by default
  DisplayBuffer[(first_row - 1) * LINE_WIDTH_HALFWORDS] = CMD_UPDATE_MULTIPLE_LINES_4BIT | first_row;
  const uint16_t last_row = first_row + rows;
  DisplayBuffer[(last_row - 1) * LINE_WIDTH_HALFWORDS] = 0; // Dummy Data

  uint16_t *buf = DisplayBuffer + (first_row - 1) * LINE_WIDTH_HALFWORDS;

  uint32_t first = (first_row - 1) * LINE_WIDTH_HALFWORDS;
  SPI_Enable(DISPLAY_SPI, ENABLE);
  for (uint16_t i = 0; i < rows * LINE_WIDTH_BYTES + 1; i++) {
    SPI_I2S_TransmitData(DISPLAY_SPI, DisplayBuffer[first + i]);
    while (0 != SPI_I2S_GetStatus(DISPLAY_SPI, SPI_I2S_BUSY_FLAG))
      ;

    while (0 == SPI_I2S_GetStatus(DISPLAY_SPI, SPI_I2S_TE_FLAG))
      ;
  }

  GPIO_ResetBits(DISP_SS_PORT, DISP_SS_PIN);
  DisplayBuffer[(first_row - 1) * LINE_WIDTH_HALFWORDS] = first_row;
  DisplayBuffer[(last_row - 1) * LINE_WIDTH_HALFWORDS] = last_row;
  // while(SPI_I2S_GetStatus(DISPLAY_SPI, SPI_I2S_TE_FLAG) == 0);
}

void prvDisplayTask(void *pvParameters) {
  DisplayBufferInit();

  Display_Config();
  
  DisplayStartupSequence();
  
  uint32_t line = 1;
  while (1) {
    TickType_t last = xTaskGetTickCount();

    for (int j = line-1; j < line + 1; j ++) {
      for (int i = 1; i <= 18; i++) {
        const uint16_t d = DisplayBuffer[j * LINE_WIDTH_HALFWORDS + i];
        if (d != 0xffff) {
          DisplayBuffer[j * LINE_WIDTH_HALFWORDS + i] = 0xffff;
        } else {
          DisplayBuffer[j * LINE_WIDTH_HALFWORDS + i] = 0xbbbb;
        }
      }
    }
    DisplayTransferLines_DMA(line, 2);

    line += 2;
    if (line >= DISP_HEIGHT - 1) {
      line = 1;
    }

    vTaskDelayUntil(&last, 100 / portTICK_PERIOD_MS);
  }
}

void Display_SPI_IRQHandler(void) {
  if (SPI_I2S_GetStatus(SPI2, SPI_MODERR_FLAG) != 0) {
    while (1)
      ;
  } else if( SPI_I2S_GetStatus(SPI2, SPI_I2S_TE_FLAG) != 0) {
    int i = 0;
    i ++;
  } else {
    int i = 0;
    i ++;
  }
}

void Display_DMA_IRQHandler(void) {
  if (DMA_GetIntStatus(DISPLAY_DMA_INT_TXC, DMA)) {

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    vTaskNotifyGiveFromISR(DisplayTaskHandle, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
  
  DMA_ClrIntPendingBit(
                       DISPLAY_DMA_INT_GLB
                       |DISPLAY_DMA_INT_TXC
                       |DISPLAY_DMA_INT_HTX
                       |DISPLAY_DMA_INT_ERR,
                       DMA);

}

static void Display_GPIO_Config() {
  // Setup GPIO for SPI2
  GPIO_InitType initValue;
  GPIO_InitStruct(&initValue);
  initValue.GPIO_Current = GPIO_DC_8mA;
  initValue.GPIO_Mode = GPIO_Mode_AF_PP;
  initValue.GPIO_Pull = GPIO_Pull_Up;

  // Setup PC7/9 for SPI2_SCK/MOSI
  initValue.Pin = GPIO_PIN_7 | GPIO_PIN_9;
  initValue.GPIO_Alternate = GPIO_AF5_SPI2;
  GPIO_InitPeripheral(GPIOC, &initValue);

  // Setup PA15 for SPI2_nSS
  initValue.Pin = DISP_SS_PIN;
  initValue.GPIO_Current = GPIO_DC_4mA;
  initValue.GPIO_Mode = GPIO_Mode_Out_PP;
  initValue.GPIO_Pull = GPIO_Pull_Down;
  // initValue.GPIO_Mode = GPIO_Mode_AF_PP;
  // initValue.GPIO_Alternate = GPIO_AF1_SPI2;
  GPIO_InitPeripheral(DISP_SS_PORT, &initValue);

  // Setup PC8 for DISP
  initValue.Pin = DISP_DISP_PIN;
  initValue.GPIO_Current = GPIO_DC_4mA;
  initValue.GPIO_Mode = GPIO_Mode_Out_OD;
  initValue.GPIO_Pull = GPIO_Pull_Up;
  GPIO_InitPeripheral(DISP_DISP_PORT, &initValue);

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
  initSPI.FirstBit = SPI_FB_MSB;
  initSPI.NSS = SPI_NSS_SOFT;
  initSPI.DataLen = SPI_DATA_SIZE_16BITS;

  SPI_Init(DISPLAY_SPI, &initSPI);

  /* SPI_I2S_EnableInt(DISPLAY_SPI, */
  /*                  SPI_I2S_INT_TE, */
  /*                  ENABLE); */
  SPI_I2S_EnableInt(DISPLAY_SPI,
                    SPI_I2S_INT_RNE,
                    ENABLE);
  SPI_I2S_EnableInt(DISPLAY_SPI,
                    SPI_I2S_INT_ERR,
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
  dmaInit.BufSize = sizeof(DisplayBuffer) / sizeof(DisplayBuffer[0]) / 2;
  dmaInit.Mem2Mem = DMA_M2M_DISABLE;
  dmaInit.MemDataSize = DMA_MemoryDataSize_HalfWord;
  dmaInit.DMA_MemoryInc = DMA_MEM_INC_ENABLE;
  dmaInit.CircularMode = DMA_MODE_NORMAL;
  dmaInit.Priority = DMA_PRIORITY_HIGH;

  DMA_Init(DISPLAY_SPI_DMA_CHANNEL, &dmaInit);
  DMA_ConfigInt(DISPLAY_SPI_DMA_CHANNEL, DMA_INT_HTX|DMA_INT_TXC|DMA_INT_ERR, ENABLE);
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
  // SPI_I2S_EnableDma(DISPLAY_SPI, SPI_I2S_DMA_TX, ENABLE);
  // SPI_Enable(DISPLAY_SPI, ENABLE);
}
