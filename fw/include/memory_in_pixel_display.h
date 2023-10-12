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

#define DISP_WIDTH (72)
#define DISP_HEIGHT (144)
#define DISP_FIRST_LINE (1)

void DisplayBufferInit();
void DisplayTransferLines(uint8_t start, uint8_t end);

void Display_Config();

#endif // MEMORY_IN_PIXEL_DISPLAY_H
