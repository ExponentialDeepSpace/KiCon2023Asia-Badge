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
#include <assert.h>
#include "setup.h"
#include <FreeRTOS.h>
#include <task.h>
#include "memory_in_pixel_display.h"

typedef struct GPIO_HighSide_Def {
  GPIO_Module *gpio;
  uint16_t pin;
}GPIO_HighSide_Def;

static GPIO_HighSide_Def GPIOs_HighSide_Rows[12] = {
  {GPIOB, GPIO_PIN_15}, // Row 1
  {GPIOC, GPIO_PIN_10}, // Row 2
  {GPIOA, GPIO_PIN_3},  // Row 3
  {GPIOA, GPIO_PIN_2},  // Row 4
  {GPIOB, GPIO_PIN_14}, // Row 5
  {GPIOA, GPIO_PIN_1},  // Row 6
  {GPIOB, GPIO_PIN_12}, // Row 7
  {GPIOB, GPIO_PIN_13}, // Row 8
  {GPIOA, GPIO_PIN_0},  // Row 9
  {GPIOB, GPIO_PIN_11}, // Row 10
  {GPIOB, GPIO_PIN_10}, // Row 11
  {GPIOB, GPIO_PIN_1},  // Row 12
};

static void prvTestTask( void *pvParameters );
static void prvDisplayTask( void *pvParameters );

#define STACK_SIZE 200
#define tskDisplay_PRIORITY 1

StaticTask_t xTaskBuffer;
StackType_t xStack[ STACK_SIZE ];

static volatile TaskHandle_t DisplayTaskHandle = NULL;
StaticTask_t xDisplayTaskBuffer;
StackType_t xDisplayStack[ STACK_SIZE ];


int main(void)
{
  Setup();
  
  DisplayBufferInit();
  
  volatile unsigned long long i = 0;

  GPIO_ResetBits(GPIOB, GPIO_PIN_4);
  GPIO_ResetBits(GPIOB, GPIO_PIN_5);

  xTaskCreateStatic(prvTestTask,
                    "TestTask",
                    configMINIMAL_STACK_SIZE,
                    (void *)NULL,
                    tskIDLE_PRIORITY,
                    xStack,
                    &xTaskBuffer);
  DisplayTaskHandle = xTaskCreateStatic(prvDisplayTask,
                    "Display",
                    configMINIMAL_STACK_SIZE,
                    (void *)NULL,
                    tskDisplay_PRIORITY,
                    xDisplayStack,
                    &xDisplayTaskBuffer);

  vTaskStartScheduler();
  
  return 0;
}

static void prvTestTask(void *pvParameters) {

  while(1);
}

static void prvDisplayTask(void *pvParameters) {
  DisplayTransferLines(DISP_FIRST_LINE, DISP_HEIGHT);
  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    DisplayTransferLines(DISP_FIRST_LINE, DISP_HEIGHT);
  }
}

void DMA_Channel1_IRQHandler(void) {
  int i = 0;
  i++;

  DMA_ClrIntPendingBit(DMA_INT_GLB1|DMA_INT_TXC1|DMA_INT_HTX1|DMA_INT_ERR1, DMA);
}

void SPI2_IRQHandler(void) {
  int i = 0;
  i++;
}

void DMA_Channel2_IRQHandler(void) {

  if (DMA_GetIntStatus(DISPLAY_DMA_INT_TXC, DMA)) {

    DMA_EnableChannel(DISPLAY_SPI_DMA_CHANNEL, DISABLE);

    // SPI TE flag is set by default
    SPI_Enable(DISPLAY_SPI, DISABLE);

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

void TIM1_UP_IRQHandler(void) {
  TIM_ClrIntPendingBit(TIM1, TIM_INT_UPDATE);

  static int i = 0;
  static int j = 11;
  GPIO_ResetBits(GPIOs_HighSide_Rows[j].gpio, GPIOs_HighSide_Rows[j].pin);
  GPIO_SetBits(GPIOs_HighSide_Rows[i].gpio, GPIOs_HighSide_Rows[i].pin);
  i++; j++;

  if (i > 11) {
    i = 0;
  }

  if (j > 11) {
    j = 0;
  }
}

void TIM1_CC_IRQHandler(void) {
  int i = 0;
  i++;
  TIM_ClrIntPendingBit(TIM1, TIM_INT_CC1);
}

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}

#if (configSUPPORT_STATIC_ALLOCATION == 1)
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleTaskStackBuffer,
                                    uint32_t *pulIdleTaskStackSize )
{
/* If the buffers to be provided to the Idle task are declared inside this
function then they must be declared static - otherwise they will be allocated on
the stack and so not exists after this function exits. */
static StaticTask_t xIdleTaskTCB;
static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

    /* Pass out a pointer to the StaticTask_t structure in which the Idle task's
    state will be stored. */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task's stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
    Note that, as the array is necessarily of type StackType_t,
    configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
#endif

#ifdef USE_FULL_ASSERT
void assert_failed(const uint8_t *expr, const uint8_t *file, uint32_t line) {
  while (1)
    ;
}
#endif
