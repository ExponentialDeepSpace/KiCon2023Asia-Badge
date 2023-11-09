#ifndef SK6812_H
#define SK6812_H

#include <stdint.h>

#define SK6812_RGB_LENGTH 10
#define SK6812_DATA_LENGTH 100

extern uint16_t sk6812_0;
extern uint16_t sk6812_1;
extern uint16_t sk6812_p; // period

extern uint8_t sk6812_rgbs[SK6812_RGB_LENGTH][3];
extern uint16_t sk6812_data[SK6812_DATA_LENGTH][4];

void TIM4_Config();

#endif // SK6812_H
