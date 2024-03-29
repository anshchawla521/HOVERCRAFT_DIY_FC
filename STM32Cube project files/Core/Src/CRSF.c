#include "CRSF.h"

uint8_t rx_buffer[128] ={}; // this ensures that the buffer contains atleast one packet
uint8_t tx_buffer[64] = {};
crsf_channels_t channel_data;
uint8_t gen_poly = 0xd5; //x8 + x5 + x4 + 1
uint8_t _lut[256] = {};
volatile uint32_t last_packet_received_time = 0;
volatile bool new_packet_recieved = false;
bool send_telemetry = false;
bool sent_telemetry = true; // if set false then code even wont run the first time
uint16_t telem_last_sent = 0;
volatile crsf_sensor_battery_t bat = {1,2,4,34};

int last_parsed_packet_location = 0;

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART6)
	{
		if((USART6->SR && (1<<3)))
		{	// over run error
			HAL_UART_DMAStop(&huart6);
			uint8_t temp = USART6->DR;
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_9);
			HAL_UARTEx_ReceiveToIdle_DMA(&huart6, rx_buffer, sizeof(rx_buffer));
			__HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT);
			channel_data.channel5 = CRSF_CHANNEL_VALUE_MIN; // disarm
			// log in future how many times have to reset
			last_packet_received_time = HAL_GetTick();
		}
	}
}


void crsf_init()
{
//	(&huart6)->hdmarx->XferCpltCallback = check_and_decode_crsf;
//	HAL_UART_Receive_DMA(&huart6, rx_buffer, sizeof(rx_buffer));
	HAL_UARTEx_ReceiveToIdle_DMA(&huart6, rx_buffer, sizeof(rx_buffer));
	__HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT);


	// gen poly lookup table
	for(int i= 0 ; i < 256 ; i++)
	{
		uint8_t crc = i;
		for(int shift = 0 ; shift < 8 ;shift++)
		{
			crc = (crc << 1) ^ ((crc&0x80) ? gen_poly : 0);
		}
		_lut[i] = crc & 0xff;
	}


}

int time_to_execute = 0;

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{

	if(huart->Instance == USART6){
//	HAL_UART_DMAStop(&huart6); // y do i need to do this
	uint8_t length = 0;

	// check if its a valid packet
	for(int i = last_parsed_packet_location ; i < sizeof(rx_buffer) && i < Size ; i++)
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
			// if any packet received then reset counter
			last_packet_received_time = HAL_GetTick(); // non hal way of doing it
			if(calculateCRC(rx_buffer , i+2,length) == 0) // start crc calculation from type byte
			{
				// valid packet

//				last_packet_received_time = __HAL_TIM_GET_COUNTER(&htim4);
				new_packet_recieved = true;
				if(rx_buffer[i+2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED && length-2 == CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE)
				{
					memcpy(&channel_data, &(rx_buffer[i+3]),sizeof(channel_data));
				}
				i = i + 2 + length - 1; // -1 because for loop will increment this
			}
		}
	}

	last_parsed_packet_location = 0;
	// chcek RXNE before starting dma
	if(send_telemetry)
	{     crsf_sensor_battery_t temp_bat = {0};
		  memcpy((void*)&temp_bat , (void*)&bat, sizeof(bat));
		  convert_to_big_endian((uint8_t*)&temp_bat, 2); // battery voltage
		  convert_to_big_endian(&(((uint8_t*)&temp_bat)[2]), 2); // battery current
		  convert_to_big_endian(&(((uint8_t*)&temp_bat)[4]), 3); // mah
		send_telem(CRSF_FRAMETYPE_BATTERY_SENSOR, (uint8_t*)&temp_bat, sizeof(temp_bat)/sizeof(uint8_t));
		sent_telemetry = true;
		send_telemetry = false;
		telem_last_sent = HAL_GetTick();
	}
	HAL_UARTEx_ReceiveToIdle_DMA(&huart6, rx_buffer, sizeof(rx_buffer));
	__HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT);

}
}


//uint8_t calculateCRC(uint8_t* buffer , int start_index , int bytes)
//{
//	time_to_execute = TIM4->CNT;
//	// CRC is calculated for payload + type
//    uint8_t dividend = 0;
//    uint8_t next_byte;
//    int numberOfBytesProcessed = 0;
//    int numberOfBitsLeft = 0;
//    bool isMsbOne = false;
//    for(int i = 0 ; i <9999 ; i++) // avoiding infinite loops
//    {
//
//        if (numberOfBitsLeft <=0 && numberOfBytesProcessed >= bytes)
//        {
//            // ALL BITS PROCEESSED
//            break;
//        }
//        if (numberOfBitsLeft <= 0 && numberOfBytesProcessed < bytes)
//        {
//            // load bits into buffer if empty and if bits available
//            next_byte = buffer[start_index + numberOfBytesProcessed];
//            numberOfBytesProcessed++;
//            numberOfBitsLeft =8;
//        }
//
//        isMsbOne = dividend & 0b10000000;
//        dividend = dividend << 1 | (next_byte>>7);   // shift First bit of Next_byte into dividend
//        next_byte = next_byte << 1;  // Shift out the first bit
//        numberOfBitsLeft --;
//        dividend = isMsbOne ? dividend ^ gen_poly : dividend; //if bit aligning with MSB of gen_poly is 1 then do XOR
//
//    }
//
//    time_to_execute = (TIM4->CNT) - time_to_execute;
//    return dividend;
//}

uint8_t calculateCRC(uint8_t* buffer , int start_index , int bytes)
{
	uint8_t crc = 0;
	uint8_t* data = &(buffer[start_index]);
	while(bytes--)
	{
		crc = _lut[crc ^ *(data++)];
	}
    return crc;
}

void send_telem(uint8_t type , uint8_t* payload , uint8_t payload_length)
{
	tx_buffer[0] = CRSF_ADDRESS_CRSF_RECEIVER;
	tx_buffer[1] = payload_length+2;
	tx_buffer[2] = type;
	memcpy(&(tx_buffer[3]),payload,payload_length);
	tx_buffer[3+payload_length] = 0; // CRC set as 0 for calculation
	tx_buffer[3+payload_length] = calculateCRC(tx_buffer ,2, payload_length+1);

	HAL_UART_Transmit_DMA(&huart6 ,tx_buffer, payload_length+4);
}

void convert_to_big_endian(uint8_t * dst , uint8_t bytes)
{
	uint8_t temp =0;
	for(int i = 0; i < bytes/2;i++)
	{
		//swap
		temp = dst[i];
		dst[i] = dst[(bytes-1)-i];
		dst[(bytes-1)-i] = temp;
	}

}
