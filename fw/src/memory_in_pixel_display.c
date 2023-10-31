#include "memory_in_pixel_display.h"

#include <n32l40x.h>

#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <assert.h>

#include "setup.h"
#include "generated_image.h"

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
#define CMD_BLINK_BLACK (M3_MASK)
#define CMD_BLINK_WHITE (M3_MASK|M4_MASK)
#define CMD_NORMAL (0)

// each line has following format
// M0-M5 AG9-AG0 DATA(D1R D1G D1B ...)
uint16_t DisplayBuffer[ LINE_WIDTH_HALFWORDS * DISP_HEIGHT
                        + DISP_BUFFER_PADDING_HALFWORD ];

const uint16_t cmd_no_update = CMD_NO_UPDATE;
const uint16_t cmd_all_clear = CMD_ALL_CLEAR;
const uint16_t cmd_color_blink_white = M3_MASK|M4_MASK;
TaskHandle_t DisplayTaskHandle = NULL;
StaticTask_t xDisplayTaskBuffer;
StackType_t xDisplayStack[ DISPLAY_STACK_SIZE ];

void DisplayBufferInit() {
  memset(DisplayBuffer, 0x0000, sizeof(DisplayBuffer));

  assert(LINE_WIDTH_HALFWORDS == IMAGE_ROW_LENGTH);
  memcpy(DisplayBuffer, image + 0 * IMAGE_ROW_LENGTH, sizeof(uint16_t) * DISP_HEIGHT * IMAGE_ROW_LENGTH);
  for (int row = 1; row <= DISP_HEIGHT; row++) {
    DisplayBuffer[(row-1) * LINE_WIDTH_HALFWORDS] = row;
  }
}

static inline void wait_for_spi_completion() {
  while(1) {
    // vTaskDelay(1);
    if (0 == SPI_I2S_GetStatus(DISPLAY_SPI, SPI_I2S_BUSY_FLAG) &&
        0 != SPI_I2S_GetStatus(DISPLAY_SPI, SPI_I2S_TE_FLAG)) {
      break;
    }
  }
}

static void wait_for_transmission_completion() {
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  wait_for_spi_completion();
}

static inline void set_display_ss(void) {
    GPIO_SetBits(DISP_SS_PORT, DISP_SS_PIN);
    {
      volatile int i = 200;
      while (i--)
        ;
    }
}

static inline void clear_display_ss(void) {
  {
    volatile int i = 200;
    while (i--)
      ;
  }
  GPIO_ResetBits(DISP_SS_PORT, DISP_SS_PIN);
}

static void send_cmd(uint16_t cmd) {
  // All Clear
  set_display_ss();

#ifdef DISPLAY_USE_DMA
  DMA_DeInit(DISPLAY_SPI_DMA_CHANNEL);

  DMA_InitType dmaInit;
  DMA_StructInit(&dmaInit);

  dmaInit.PeriphAddr = (uint32_t)&DISPLAY_SPI->DAT;
  dmaInit.PeriphInc = DMA_PERIPH_INC_DISABLE;
  dmaInit.PeriphDataSize = DMA_PERIPH_DATA_SIZE_HALFWORD;
  dmaInit.Direction = DMA_DIR_PERIPH_DST;
  dmaInit.MemAddr = (uint32_t)&cmd;
  dmaInit.BufSize = 1;
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

  wait_for_transmission_completion();
#else
  SPI_Enable(DISPLAY_SPI, ENABLE);
  SPI_I2S_TransmitData(
                       DISPLAY_SPI, &cmd);
  wait_for_spi_completion();
#endif

  clear_display_ss();
}

void DisplayStartupSequence() {
  // Power up
  GPIO_ResetBits(DISP_SS_PORT, DISP_SS_PIN);
  GPIO_ResetBits(DISP_DISP_PORT, DISP_DISP_PIN);
  vTaskDelay(1);

  // All Clear
  send_cmd(CMD_ALL_CLEAR);

  vTaskDelay(100 / portTICK_PERIOD_MS);

  // DISP
  GPIO_SetBits(DISP_DISP_PORT, DISP_DISP_PIN);
  vTaskDelay(1);

  uint32_t TIMClk = TIM_Clk(TIM3);
  // set prescaler for both TIM1 and TIM3 to be same TIMClk (24Mhz)
  uint16_t prescaler = IsTimList1Module(TIM3) ?
    TIM_ADVANCED_PRESCALER : TIM_GENERIC_PRESCALER;

#define DISP_COM_FREQ (30) // Hz
  uint16_t period = TIMClk / (prescaler + 1) / DISP_COM_FREQ - 1;
  TIM_SetCmp4(TIM3, period / 2);
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
  set_display_ss();
  // SPI TE flag is set by default
  // uint16_t first_cmd = CMD_UPDATE_MULTIPLE_LINES_4BIT | first_row;
  DisplayBuffer[(first_row - 1) * LINE_WIDTH_HALFWORDS] = CMD_UPDATE_MULTIPLE_LINES_4BIT | first_row;
  const uint16_t last_row = first_row + rows;
  DisplayBuffer[(last_row - 1) * LINE_WIDTH_HALFWORDS] = 0; // Dummy Data

  uint16_t *buf = DisplayBuffer + (first_row - 1) * LINE_WIDTH_HALFWORDS;

  DMA_InitType dmaInit;
  DMA_StructInit(&dmaInit);

  DMA_DeInit(DISPLAY_SPI_DMA_CHANNEL);
  
  dmaInit.PeriphAddr = (uint32_t)&DISPLAY_SPI->DAT;
  dmaInit.PeriphInc = DMA_PERIPH_INC_DISABLE;
  dmaInit.PeriphDataSize = DMA_PERIPH_DATA_SIZE_HALFWORD;
  dmaInit.Direction = DMA_DIR_PERIPH_DST;
  dmaInit.MemAddr = (uint32_t)buf;
  dmaInit.BufSize = rows * LINE_WIDTH_HALFWORDS + 1;
  dmaInit.Mem2Mem = DMA_M2M_DISABLE;
  dmaInit.MemDataSize = DMA_MemoryDataSize_HalfWord;
  dmaInit.DMA_MemoryInc = DMA_MEM_INC_ENABLE;
  dmaInit.CircularMode = DMA_MODE_NORMAL;
  dmaInit.Priority = DMA_PRIORITY_HIGH;

  DMA_Init(DISPLAY_SPI_DMA_CHANNEL, &dmaInit);

  SPI_Enable(DISPLAY_SPI, ENABLE);
  set_display_ss();

  SPI_I2S_EnableDma(DISPLAY_SPI, SPI_I2S_DMA_TX, ENABLE);
  DMA_RequestRemap(DISPLAY_SPI_DMA_CHANNEL_REMAP, DMA, DISPLAY_SPI_DMA_CHANNEL, ENABLE);
  DMA_EnableChannel(DISPLAY_SPI_DMA_CHANNEL, ENABLE);

#ifdef DISPLAY_USE_DMA
  wait_for_transmission_completion();
#endif
  clear_display_ss();

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
  set_display_ss();
  // SPI TE flag is set by default
  DisplayBuffer[(first_row - 1) * LINE_WIDTH_HALFWORDS] = CMD_UPDATE_MULTIPLE_LINES_4BIT | first_row;
  const uint16_t last_row = first_row + rows;
  DisplayBuffer[(last_row - 1) * LINE_WIDTH_HALFWORDS] = 0; // Dummy Data

  uint32_t first = (first_row - 1) * LINE_WIDTH_HALFWORDS;
  SPI_Enable(DISPLAY_SPI, ENABLE);
  for (uint16_t i = 0; i < rows * LINE_WIDTH_HALFWORDS + 1; i++) {
    SPI_I2S_TransmitData(DISPLAY_SPI, DisplayBuffer[first + i]);
    wait_for_spi_completion();
  }
  clear_display_ss();

  DisplayBuffer[(first_row - 1) * LINE_WIDTH_HALFWORDS] = first_row;
  DisplayBuffer[(last_row - 1) * LINE_WIDTH_HALFWORDS] = last_row;
}

void prvDisplayTask(void *pvParameters) {
  DisplayBufferInit();

  Display_Config();

  DisplayStartupSequence();

  uint32_t line = DISP_HEIGHT;
  uint32_t direction = -1;
  while (1) {
    TickType_t last = xTaskGetTickCount();

    /*
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
    */
#ifdef DISPLAY_USE_DMA
    DisplayTransferLines_DMA(DISP_FIRST_LINE, DISP_HEIGHT);
#else
    DisplayTransferLines_Poll(DISP_FIRST_LINE, DISP_HEIGHT);
#endif

    /*
    while (1) {
      send_cmd(CMD_BLINK_BLACK);
      vTaskDelay(500 / portTICK_PERIOD_MS);
      send_cmd(CMD_BLINK_WHITE);
      vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    */
    assert(line > 0 && line <= DISP_HEIGHT);
    const uint16_t * imagep = image;
    memcpy(DisplayBuffer, imagep + (line - 1) * IMAGE_ROW_LENGTH, sizeof(uint16_t) * DISP_HEIGHT * IMAGE_ROW_LENGTH);
    for (int i = 0 * LINE_WIDTH_HALFWORDS + 1; i < LINE_WIDTH_HALFWORDS; i++) {
      DisplayBuffer[i] = 0x0;
    }
    for (int row = 1; row <= DISP_HEIGHT; row++) {
      DisplayBuffer[(row-1) * LINE_WIDTH_HALFWORDS] = row;
    }

    line += direction;

    if (line == DISP_HEIGHT || 1 == line) {
      direction = -direction;
    }
    
    vTaskDelayUntil(&last, 1 /*1000 / portTICK_PERIOD_MS*/);

    /*
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    send_cmd(CMD_BLINK_WHITE);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    send_cmd(CMD_NORMAL);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    send_cmd(CMD_BLINK_WHITE);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    send_cmd(CMD_NORMAL);
    */
  }
}

void Display_SPI_IRQHandler(void) {
  if (SPI_I2S_GetStatus(DISPLAY_SPI, SPI_MODERR_FLAG) != 0) {
    while (1)
      ;
  } else if( SPI_I2S_GetStatus(DISPLAY_SPI, SPI_I2S_TE_FLAG) != 0) {
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
  // Setup GPIO for SPI1
  GPIO_InitType initValue;
  GPIO_InitStruct(&initValue);
  initValue.GPIO_Current = GPIO_DC_8mA;
  initValue.GPIO_Mode = GPIO_Mode_AF_PP;
  initValue.GPIO_Pull = GPIO_Pull_Up;

  // Setup PB3 for SPI1_SCK
  initValue.Pin = GPIO_PIN_3;
  initValue.GPIO_Alternate = GPIO_AF1_SPI1;
  GPIO_InitPeripheral(GPIOB, &initValue);

  // Setup PB5 for SPI1_MOSI
  initValue.Pin = GPIO_PIN_5;
  initValue.GPIO_Alternate = GPIO_AF0_SPI1;
  GPIO_InitPeripheral(GPIOB, &initValue);

  // Setup PB4 for SPI1_nSS
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

  // Setup PB1 for EXT COM IN
  // initValue.Pin = DISP_COM_PIN;
  // initValue.GPIO_Current = GPIO_DC_4mA;
  // initValue.GPIO_Mode = GPIO_Mode_AF_PP;
  // initValue.GPIO_Pull = GPIO_Pull_Up;
  // initValue.GPIO_Alternate = DISP_COM_AF;
  // GPIO_InitPeripheral(DISP_COM_PORT, &initValue);
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
  initSPI.BaudRatePres = SPI_BR_PRESCALER_16;
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
