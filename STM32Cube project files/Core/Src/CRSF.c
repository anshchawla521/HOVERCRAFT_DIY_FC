#include "CRSF.h"

uint8_t rx_buffer[128] ={}; // this ensures that the buffer contains atleast one packet
crsf_channels_t channel_data;
uint8_t gen_poly = 0xd5; //x8 + x5 + x4 + 1



void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	// just for eg
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
	for(int i = 0 ; i < sizeof(rx_buffer) ; i++)
	{
		if (rx_buffer[i] == 0xC8)
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
			if(calculateCRC(i+2,length) == 0) // start crc calculation from type byte
			{
				// valid packet
				if(rx_buffer[i+2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED && length-2 == CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE)
				{
					memcpy(&channel_data, &(rx_buffer[i+3]),sizeof(channel_data));
					HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
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
	HAL_UART_Receive_DMA(&huart6, rx_buffer, sizeof(rx_buffer));
}


uint8_t calculateCRC( int start_index , int bytes)
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
            next_byte = rx_buffer[start_index + numberOfBytesProcessed];
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
