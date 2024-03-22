/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dshot.h"
#include "crsf.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define min_servo 950
#define max_servo 2180
#define deadband 100
#define vbat_scale 110
#define vbat_divider 10
#define vbat_multiplier 1
#define ibata_scale 400
#define ibata_offset 0

#define total_number_of_samples 200
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t my_motor_value[4] = {0, 0, 0, 0};
int angle = 0;
typedef enum {IDLE = 0,PREARMED,NOPREARM,ARMED,FAILSAFE} armingstate_t;
armingstate_t arm_state = 0;
uint16_t battery_telem_last_sent = 0;
volatile uint16_t raw_adc_data[3] = {0};
volatile crsf_sensor_battery_t bat = {1,2,4,34};


uint16_t average_motor_values[4][total_number_of_samples] = {0};
double final_motor_values[4] = {0};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
float map(float value_to_map , float from_low ,float from_high , float to_low , float to_high , bool constrain_within_range);
void convert_to_big_endian(uint8_t * dst , uint8_t bytes);
bool subtract_and_compare_unsigned(uint32_t a,uint32_t b, uint32_t c , uint32_t max_size);



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */


  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_USART6_UART_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  dshot_init(DSHOT600);
  crsf_init();

	// want the timer to run at 1 mhz (u can choose any )
	// so prescaler = 48mhz(apb1) / 1mhz = 48
	__HAL_TIM_SET_PRESCALER(&htim5, 48);
		//for 50hz the arr value should be 1mhz/50 = 20000
	__HAL_TIM_SET_AUTORELOAD(&htim5, 20000);
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1 , 1200);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);


	// tim4 used for measurment of time
	__HAL_TIM_SET_PRESCALER(&htim4, 960); // so timer running at 0.05mhz ==> 20 us = 1 step => timer overflows in 1.3 seconds
	HAL_TIM_Base_Start(&htim4);

	// adc
    //HAL_ADC_Start_DMA(&hadc1,(uint32_t *)raw_adc_data , 2); // take readings from adc


	// arm esc
    dshot_arm();
    //enable_dshot_3d(0);
    //dshot_reverse_direction(0);// back motor
    //dshot_reverse_direction(2);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
	  if(arm_state == ARMED && channel_data.channel5 < CRSF_CHANNEL_VALUE_MID + 20) arm_state = IDLE ;
	  else if(arm_state == IDLE && channel_data.channel6 > CRSF_CHANNEL_VALUE_MID + 20 && channel_data.channel5 < CRSF_CHANNEL_VALUE_MID + 20) arm_state = PREARMED ;
	  else if(arm_state == IDLE && channel_data.channel6 < CRSF_CHANNEL_VALUE_MID + 20 && channel_data.channel5 > CRSF_CHANNEL_VALUE_MID + 20) arm_state = NOPREARM ;
	  else if(arm_state == PREARMED && channel_data.channel6 > CRSF_CHANNEL_VALUE_MID + 20 && channel_data.channel5 > CRSF_CHANNEL_VALUE_MID + 20) arm_state = ARMED ;
	  else if(arm_state == NOPREARM && channel_data.channel6 < CRSF_CHANNEL_VALUE_MID + 20 ) arm_state = IDLE ;

//	  if(rx_buffer[0] | rx_buffer[1] | rx_buffer[2] | rx_buffer[3] | rx_buffer[4])
//	  {
//		  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
//	  }

	  if(arm_state == ARMED) // arm channel
	  {
		  if(channel_data.channel6 < 1500)
			  my_motor_value[2] = map(channel_data.channel3, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000, DSHOT_3DN_MIN_THROTTLE, DSHOT_3DN_MAX_THROTTLE, true);
		  else
			  my_motor_value[2] = map(channel_data.channel3, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000, DSHOT_3DR_MIN_THROTTLE, DSHOT_3DR_MAX_THROTTLE, true);


		  // back motor
		  if(channel_data.channel2 >= CRSF_CHANNEL_VALUE_MID + deadband)
			  my_motor_value[0] = map(channel_data.channel2, CRSF_CHANNEL_VALUE_MID + deadband, CRSF_CHANNEL_VALUE_2000, DSHOT_3DN_MIN_THROTTLE, DSHOT_3DN_MAX_THROTTLE, true); // correct this
		  else if(channel_data.channel2 <= CRSF_CHANNEL_VALUE_MID - deadband)
			  my_motor_value[0] = map(channel_data.channel2, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_MID - deadband, DSHOT_3DR_MAX_THROTTLE, DSHOT_3DR_MIN_THROTTLE, true); // correct this
		  else
			  my_motor_value[0]= 0;


		  angle = map(channel_data.channel1,CRSF_CHANNEL_VALUE_1000,CRSF_CHANNEL_VALUE_2000,min_servo, max_servo,true);
	  }
	  else{
		  my_motor_value[0] = 0;
		  my_motor_value[2] = 0;
		  angle = (min_servo+max_servo)/2;
	  }

	  if(arm_state == FAILSAFE )
	  {
		  if(!(USART6->SR && (1<<3)))
		  {
			  // not over run error
			  dshot_beep(0,2);
		  }else
		  {
//			  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
		  }
		  dshot_beep(2,2);
		  angle = 1000;

	  }
	  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1 ,angle);
	  if(my_motor_value[0] == 0 && my_motor_value[2] == 0 && channel_data.channel9 > 1500)
	  {
		  dshot_beep(0,2);
		  dshot_beep(2,2);
	  }else{
	  dshot_write(my_motor_value , false);

	  }


	  // telemetry
	  if((uint16_t)(HAL_GetTick() - battery_telem_last_sent) > 500) // send battery telemetry every 500ms
	  {
		  battery_telem_last_sent = HAL_GetTick();
		  crsf_sensor_battery_t temp_bat = {0};
		  memcpy((void*)&temp_bat , (void*)&bat, sizeof(bat));
		  convert_to_big_endian((uint8_t*)&temp_bat, 2); // battery voltage
		  convert_to_big_endian(&(((uint8_t*)&temp_bat)[2]), 2); // battery current
		  convert_to_big_endian(&(((uint8_t*)&temp_bat)[4]), 3); // mah
		  send_telem(CRSF_FRAMETYPE_BATTERY_SENSOR, (uint8_t*)&temp_bat, sizeof(temp_bat)/sizeof(uint8_t));
		  HAL_ADC_Start_DMA(&hadc1,(uint32_t *)raw_adc_data , 2); // take readings from adc
		 // in future before sending check whether old data sent or not
	  }


	// FAILSAFE detection
	  if((uint32_t)(HAL_GetTick() - last_packet_received_time) > 1000) //no packet received in 1 second
	  {
		  arm_state = FAILSAFE;
		  new_packet_recieved = false;
		  last_packet_received_time = HAL_GetTick();

	  }else if(arm_state == FAILSAFE && new_packet_recieved == true)
	  {
		  arm_state = IDLE;
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */

float map(float value_to_map , float from_low ,float from_high , float to_low , float to_high , bool constrain_within_range)
{

	value_to_map = (value_to_map- from_low)*((to_high - to_low)/(from_high- from_low)) + to_low;
	if(constrain_within_range)
	{
		if(to_high>=to_low){
			value_to_map = value_to_map > to_high ? to_high : value_to_map;
			value_to_map = value_to_map < to_low ? to_low : value_to_map;
		}else{
			// to low is the higher limit
			value_to_map = value_to_map < to_high ? to_high : value_to_map;
			value_to_map = value_to_map > to_low ? to_low : value_to_map;

		}
	}
	return value_to_map;
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

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == ADC1)
	{
		bat.voltage = (uint16_t)(((raw_adc_data[0]*3.3/(float)1024)*vbat_scale*vbat_multiplier/(float)vbat_divider)*10.0); // battery voltage in 0.1V units
		bat.current = (uint16_t)(((raw_adc_data[1]*3.3/(float)1024)*ibata_scale - ibata_offset)*10.0); // battery voltage in 0.1A units
		bat.remaining = (uint8_t)(map(bat.voltage,150,168,0,100,true));
	}
}

bool subtract_and_compare_unsigned(uint32_t a,uint32_t b, uint32_t c ,uint32_t max_size)
{
	uint32_t difference= 0;
	if(a>b)difference = a-b;
	if(a<=b)difference = a-b+max_size;

	if(difference > c)
		return true;
	return false;


}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {

  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
