#ifndef SK6812_H
#define SK6812_H

#include <stdint.h>

#define LED_COUNT 4
#define SK_COUNT 2
#define SEQLEN 10
#define SEQ_RST_LEN 3

extern uint16_t sk6812_0;
extern uint16_t sk6812_1;
extern uint16_t sk6812_p; // period

extern uint8_t sk6812_rgbs[SK_COUNT][SEQLEN][3];

#ifdef LED_SAO_USE_DMA
#define PATTERN_LEN ((SEQLEN + 2 /* reset code */) * 24)
extern uint16_t sk6812_data[PATTERN_LEN][COUNT];
#endif

void SK6812_Config();

#endif // SK6812_H
