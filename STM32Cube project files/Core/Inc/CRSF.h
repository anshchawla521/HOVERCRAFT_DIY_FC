#ifndef CRSFV2
#define CRSFV2

#include "usart.h"
#include "stdbool.h"
#include "crsf_protocol.h"
#include "string.h"
#include "tim.h"

extern volatile uint8_t rx_buffer[64*2];
extern crsf_channels_t channel_data;
extern volatile uint16_t  last_packet_received_time;
extern volatile bool new_packet_recieved ;
void crsf_init();
uint8_t calculateCRC( int start_index , int bytes);

#endif
