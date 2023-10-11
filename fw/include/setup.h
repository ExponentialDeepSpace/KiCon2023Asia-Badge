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

#ifndef SETUP_H
#define SETUP_H

#include <FreeRTOS.h>

#define LED_PWM_CLOCK 20000 // 20Khz
#define TIM_ADVANCED_PRESCALER 0
#define TIM_GENERIC_PRESCALER 0

#define TIM_INTERRUPT_PRIORITY_LEVEL (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY - 1)
#define TIM_DMA_INTERRUPT_PRIORITY_LEVEL (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY - 1)

#define DISPLAY_SPI SPI2
#define DISPLAY_SPI_DMA_CHANNEL DMA_CH2
#define DISPLAY_SPI_DMA_CHANNEL_REMAP DMA_REMAP_SPI2_TX

#define DISPLAY_SPI_INT_GLB DMA_INT_GLB2
#define DISPLAY_SPI_INT_TXC DMA_INT_TXC2
#define DISPLAY_SPI_INT_HTX DMA_INT_HTX2
#define DISPLAY_SPI_INT_ERR DMA_INT_ERR2

#define DISPLAY_SPI_DMA_INTERRUPT_PRIORITY_LEVEL (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY)
#define DISPLAY_SPI_INTERRUPT_PRIORITY_LEVEL (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY)

void Setup();
#endif // SETUP_H
