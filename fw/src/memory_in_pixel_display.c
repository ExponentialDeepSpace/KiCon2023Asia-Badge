#include "memory_in_pixel_display.h"

#include <string.h>

uint16_t DisplayBuffer[ LINE_WIDTH_HALFWORDS * DISP_HEIGHT ];

void DisplayBuferInit() {
  memset(DisplayBuffer, sizeof(DisplayBuffer), 0);
  for (uint32_t row = 1; row <= DISP_HEIGHT; row++) {
    DisplayBuffer[row * LINE_WIDTH_BYTES] = CMD_UPDATE_MULTIPLE_LINES_4BIT | row;
  }
}
