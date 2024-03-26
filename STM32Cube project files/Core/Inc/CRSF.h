#ifndef CRSFV2
#define CRSFV2

#include "usart.h"
#include "stdbool.h"
#include "crsf_protocol.h"
#include "string.h"
#include "tim.h"

extern uint8_t rx_buffer[64*2];
extern uint8_t tx_buffer[64];
extern crsf_channels_t channel_data;
extern volatile uint32_t  last_packet_received_time;
extern volatile bool new_packet_recieved ;
extern bool send_telemetry,sent_telemetry;
extern uint16_t telem_last_sent;
volatile extern crsf_sensor_battery_t bat ;
void crsf_init();
uint8_t calculateCRC(uint8_t* buffer , int start_index , int bytes);
void send_telem(uint8_t type , uint8_t* payload , uint8_t payload_length);
void convert_to_big_endian(uint8_t * dst , uint8_t bytes);


#endif
