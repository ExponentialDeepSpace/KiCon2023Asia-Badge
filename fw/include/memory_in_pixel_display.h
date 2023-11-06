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

#ifndef MEMORY_IN_PIXEL_DISPLAY_H
#define MEMORY_IN_PIXEL_DISPLAY_H

#include <stdint.h>
#include <FreeRTOS.h>
#include <task.h>

#define DISP_WIDTH (72)
#define DISP_HEIGHT (144)
#define DISP_FIRST_LINE (1)

#define Display_DMA_IRQHandler DMA_Channel2_IRQHandler
#define Display_SPI_IRQHandler SPI1_IRQHandler
#define DISPLAY_STACK_SIZE 400
#define tskDisplay_PRIORITY 3

extern TaskHandle_t DisplayTaskHandle;
extern StaticTask_t xDisplayTaskBuffer;
extern StackType_t xDisplayStack[ DISPLAY_STACK_SIZE ];

extern const uint16_t cmd_no_update;
extern const uint16_t cmd_all_clear;

void DisplayBufferInit();
void DisplayTransferLines_DMA(uint8_t start, uint8_t end);
void DisplayTransferLines_Poll(uint8_t start, uint8_t end);
void prvDisplayTask(void *pvParameters);
void Display_Config();

#endif // MEMORY_IN_PIXEL_DISPLAY_H
