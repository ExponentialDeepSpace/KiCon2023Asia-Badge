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

#define LINE_PREFIX_BYTES (2)
#define PIXEL_SIZE_BITS (4) // use 4-bit mode
#define LINE_WIDTH_BYTES (LINE_PREFIX_BYTES + DISP_WIDTH * PIXEL_SIZE_BITS / 8)
#define LINE_WIDTH_HALFWORDS (LINE_WIDTH_BYTES / 2)

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
extern uint16_t DisplayBuffer[ LINE_WIDTH_HALFWORDS * DISP_HEIGHT ];

void DisplayBuferInit();

#endif // MEMORY_IN_PIXEL_DISPLAY_H
