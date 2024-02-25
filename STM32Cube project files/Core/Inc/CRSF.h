#ifndef CRSFV2
#define CRSFV2

#include "usart.h"
#include "stdbool.h"

extern uint8_t rx_buffer[64*2];
void crsf_init();
uint8_t calculateCRC( int start_index , int bytes);

#endif
