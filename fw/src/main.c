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

int main(void)
{
  Setup();
  
  // cdc_acm_init();
  // usb_dc_low_level_init();

  volatile unsigned long long i = 0;

  DisplayTaskHandle = xTaskCreateStatic(
                                        prvDisplayTask,
                                        "Display",
                                        DISPLAY_STACK_SIZE,
                                        (void *)NULL,
                                        tskDisplay_PRIORITY,
                                        xDisplayStack,
                                        &xDisplayTaskBuffer);

  vTaskStartScheduler();
  
  return 0;
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
