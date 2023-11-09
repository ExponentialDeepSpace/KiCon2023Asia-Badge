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

// Do not include <FreeRTOS.h>
// To make setup.h independent to all components
// CherryUSB has macro definitions conflicted with <n32l40x.h>

#define LED_PWM_CLOCK 2000 // Hz
#define SK6812_PWM_CLOCK 833333 // Hz, 1.2us

#define TIM_ADVANCED_PRESCALER (1)
#define TIM_GENERIC_PRESCALER (0)

#define TIM_INTERRUPT_PRIORITY_LEVEL (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY+2)
#define TIM_DMA_INTERRUPT_PRIORITY_LEVEL (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY+2)

#define DISPLAY_SPI SPI1
// #define DISPLAY_USE_DMA
#define DISPLAY_SPI_DMA_CHANNEL DMA_CH2
#define DISPLAY_SPI_DMA_CHANNEL_REMAP DMA_REMAP_SPI1_TX

#define DISPLAY_DMA_INT_GLB DMA_INT_GLB2
#define DISPLAY_DMA_INT_TXC DMA_INT_TXC2
#define DISPLAY_DMA_INT_HTX DMA_INT_HTX2
#define DISPLAY_DMA_INT_ERR DMA_INT_ERR2

#define DISPLAY_SPI_DMA_INTERRUPT_PRIORITY_LEVEL (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY+1)
#define DISPLAY_SPI_INTERRUPT_PRIORITY_LEVEL (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY+1)

#define DISP_SS_PORT GPIOB
#define DISP_SS_PIN GPIO_PIN_4

#define DISP_DISP_PORT GPIOD
#define DISP_DISP_PIN GPIO_PIN_2

// TIM3_CH4
#define DISP_COM_PORT GPIOB
#define DISP_COM_PIN GPIO_PIN_1
#define DISP_COM_AF  GPIO_AF2_TIM3

#define LED_ROW_1_PORT GPIOB
#define LED_ROW_1_PIN GPIO_PIN_15
#define LED_ROW_2_PORT GPIOA
#define LED_ROW_2_PIN GPIO_PIN_15
#define LED_ROW_3_PORT GPIOA
#define LED_ROW_3_PIN GPIO_PIN_3
#define LED_ROW_4_PORT GPIOA
#define LED_ROW_4_PIN GPIO_PIN_2
#define LED_ROW_5_PORT GPIOB
#define LED_ROW_5_PIN GPIO_PIN_14
#define LED_ROW_6_PORT GPIOA
#define LED_ROW_6_PIN GPIO_PIN_1
#define LED_ROW_7_PORT GPIOB
#define LED_ROW_7_PIN GPIO_PIN_12
#define LED_ROW_8_PORT GPIOB
#define LED_ROW_8_PIN GPIO_PIN_13
#define LED_ROW_9_PORT GPIOA
#define LED_ROW_9_PIN GPIO_PIN_0
#define LED_ROW_10_PORT GPIOB
#define LED_ROW_10_PIN GPIO_PIN_11
#define LED_ROW_11_PORT GPIOB
#define LED_ROW_11_PIN GPIO_PIN_10
#define LED_ROW_12_PORT GPIOB
#define LED_ROW_12_PIN GPIO_PIN_2


#define DISK_DMA_CHANNEL DMA_CH3
#define DISK_DMA_INT_GLB DMA_INT_GLB3
#define DISK_DMA_INT_TXC DMA_INT_TXC3
#define DISK_DMA_INT_HTX DMA_INT_HTX3
#define DISK_DMA_INT_ERR DMA_INT_ERR3

#define DISK_DMA_FLG_GLB DMA_FLAG_GL3
#define DISK_DMA_FLG_TXC DMA_FLAG_TC3
#define DISK_DMA_FLG_HTX DMA_FLAG_HT3
#define DISK_DMA_FLG_ERR DMA_FLAG_TE3

#define DISK_DMA_IRQHandler DMA_Channel3_IRQHandler

void Setup();
#endif // SETUP_H
