#include "CRSF.h"

uint8_t rx_buffer[128] ={}; // this ensures that the buffer contains atleast one packet
uint8_t tx_buffer[64] = {};
crsf_channels_t channel_data;
uint8_t gen_poly = 0xd5; //x8 + x5 + x4 + 1
volatile uint16_t last_packet_received_time = 0;
volatile bool new_packet_recieved = false;

int last_parsed_packet_location = 0;


void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	uint8_t length = 0;

		// check if its a valid packet
		for(int i = 0 ; i < sizeof(rx_buffer)/2 ; i++)
		{
			if (rx_buffer[i] == CRSF_ADDRESS_FLIGHT_CONTROLLER)
			{
				length = rx_buffer[i+1];
				// check length
				//then get all packets and check CRC if pass then packet valid
				if((length+i+2 > (sizeof(rx_buffer)/2)-1) || (i+1 > (sizeof(rx_buffer)/2)-1))
				{
					// handel cases with non full packets
					// can occur when the whole packet crc fails
					continue;
				}
				if(calculateCRC(rx_buffer ,i+2,length) == 0) // start crc calculation from type byte
				{
					// valid packet
					last_parsed_packet_location = i+2+length;
				    last_packet_received_time = TIM4->CNT; // non hal way of doing it
	//				last_packet_received_time = __HAL_TIM_GET_COUNTER(&htim4);
					new_packet_recieved = true;
					if(rx_buffer[i+2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED && length-2 == CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE)
					{
						memcpy(&channel_data, &(rx_buffer[i+3]),sizeof(channel_data));
					}
				}
			}
		}

}


void crsf_init()
{
//	(&huart6)->hdmarx->XferCpltCallback = check_and_decode_crsf;
	HAL_UART_Receive_DMA(&huart6, rx_buffer, 128);

}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uint8_t length = 0;

	// check if its a valid packet
	for(int i = last_parsed_packet_location ; i < sizeof(rx_buffer) ; i++)
	{
		if (rx_buffer[i] == CRSF_ADDRESS_FLIGHT_CONTROLLER)
		{
			length = rx_buffer[i+1];
			// check length
			//then get all packets and check CRC if pass then packet valid
			if((length+i+2 > sizeof(rx_buffer)-1) || (i+1 > sizeof(rx_buffer)-1))
			{
				// handel cases with non full packets
				// can occur when the whole packet crc fails
				continue;
			}
			if(calculateCRC(rx_buffer , i+2,length) == 0) // start crc calculation from type byte
			{
				// valid packet
			    last_packet_received_time = TIM4->CNT; // non hal way of doing it
//				last_packet_received_time = __HAL_TIM_GET_COUNTER(&htim4);
				new_packet_recieved = true;
				if(rx_buffer[i+2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED && length-2 == CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE)
				{
					memcpy(&channel_data, &(rx_buffer[i+3]),sizeof(channel_data));
				}
			}
		}
	}

	// no need
//	for(int i = 0 ; i < sizeof(rx_buffer) ; i++ )
//	{
//		rx_buffer[i] = 0; // clear buffer
//	}

//	__HAL_DMA_DISABLE(huart6.hdmarx); // workaround for clearing the interrupt bit
	// after processing re-enable DMA for new data
	last_parsed_packet_location = 0;
	HAL_UART_Receive_DMA(&huart6, rx_buffer, sizeof(rx_buffer));
}


uint8_t calculateCRC(uint8_t* buffer , int start_index , int bytes)
{

	// CRC is calculated for payload + type
    uint8_t dividend = 0;
    uint8_t next_byte;
    int numberOfBytesProcessed = 0;
    int numberOfBitsLeft = 0;
    bool isMsbOne = false;
    for(int i = 0 ; i <9999 ; i++) // avoiding infinite loops
    {

        if (numberOfBitsLeft <=0 && numberOfBytesProcessed >= bytes)
        {
            // ALL BITS PROCEESSED
            break;
        }
        if (numberOfBitsLeft <= 0 && numberOfBytesProcessed < bytes)
        {
            // load bits into buffer if empty and if bits available
            next_byte = buffer[start_index + numberOfBytesProcessed];
            numberOfBytesProcessed++;
            numberOfBitsLeft =8;
        }

        isMsbOne = dividend & 0b10000000;
        dividend = dividend << 1 | (next_byte>>7);   // shift First bit of Next_byte into dividend
        next_byte = next_byte << 1;  // Shift out the first bit
        numberOfBitsLeft --;
        dividend = isMsbOne ? dividend ^ gen_poly : dividend; //if bit aligning with MSB of gen_poly is 1 then do XOR

    }

    return dividend;
}

void send_telem(uint8_t type , uint8_t* payload , uint8_t payload_length)
{
	tx_buffer[0] = CRSF_ADDRESS_CRSF_RECEIVER;
	tx_buffer[1] = payload_length+2;
	tx_buffer[2] = type;
	memcpy(&(tx_buffer[3]),payload,payload_length);
	tx_buffer[3+payload_length] = 0; // CRC set as 0 for calculation
	tx_buffer[3+payload_length] = calculateCRC(tx_buffer ,2, payload_length+2);

	HAL_UART_Transmit_DMA(&huart6 ,tx_buffer, payload_length+4);
}
