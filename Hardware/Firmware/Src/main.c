/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */

#define DEFINEGLOBALSHERE

#include "stdio.h"
#include "math.h"
#include "mlx90363.h"
#include "74HC4051D.h"

#include "utility.h"
#include "bus.h"
#include "string.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

WWDG_HandleTypeDef hwwdg;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_WWDG_Init(void);

/* USER CODE BEGIN PFP */

#include "global.h"

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

// timeout counters
uint8_t down_command_counter=0;
uint8_t up_command_counter=0;

// watchdog bools
uint8_t WD_UP = 1;
uint8_t WD_DOWN = 1;

// stores time when the sensor was last read
uint16_t sensor_timer = 0;
uint16_t init_timer = 0;

uint8_t UP_RESEND_MESSAGE = 0;
uint8_t DOWN_RESEND_MESSAGE = 0;
Line command_in_line;
Line send_up_line;

#define UP_MESSAGES_MAX 12

uint8_t poll_up_buffer[UP_MESSAGES_MAX*UP_MESSAGE_LENGTH];
uint8_t poll_up_size;

uint8_t poll_down_buffer[UP_MESSAGES_MAX*UP_MESSAGE_LENGTH];
uint8_t poll_down_size;

uint8_t splitter_kids_counter[] = {0,0,0,0,0,0,0,0};
uint8_t skc[] = {0,0,0,0,0,0,0,0};

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

	// create pointer to MX_SPI_Init() function in order to use it in utility functions
   MX_SPI1_Init_Pointer = &MX_SPI1_Init;

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  //MX_SPI1_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_WWDG_Init();

  /* USER CODE BEGIN 2 */
	// delete MX_SPI1_Init(); up there after regenerating code with mx cube

	//=================================================================================
	/// some basic setups
	//=================================================================================

	// input is up
	// output is down
	uart_queue_initialize(&uart_input_queue);
	uart_queue_initialize(&uart_output_queue);
	uart_queue_initialize(&uart_command_queue);

	// load setup_data from flash to struct: config_data_flash_struct setup_data;
	get_from_Flash();

	LED_init();

	if(setup_data.initied_0xAA!=0xAA)
	{
		reset_joint();
	}

	part_init();

	/// FIND DIRECTION = WAIT FOR FIRST 'R'
	DIRECTION_SET = 0;

	PROGRAMM_SENSOR = 0;
	SET_FLASH = 0;

	UP_STATE = IDLE;
	DOWN_STATE = IDLE;

	// init some huart stuff
	huart_DOWN = &huart1;
	huart_UP = &huart2;

	uint16_t si = 0;
	data_mode = 0;

	HAL_StatusTypeDef status;

	// create empty messages
	Line empty_command_line;
	empty_command_line.text[0] = empty;
	for(si=1; si < DOWN_MESSAGE_LENGTH-1; si++)
	{
		empty_command_line.text[si] = si;
	}

	empty_command_line.text[DOWN_MESSAGE_LENGTH-1] = ComputeCRCN(empty_command_line.text, DOWN_MESSAGE_LENGTH-1);
	empty_command_line.length = DOWN_MESSAGE_LENGTH;

	Line empty_up_line;
	empty_up_line.text[0] = empty;
	for(si=1; si < UP_MESSAGE_LENGTH-1; si++)
	{
		empty_up_line.text[si] = si;
	}

	empty_up_line.text[UP_MESSAGE_LENGTH-1] = ComputeCRCN(empty_up_line.text, UP_MESSAGE_LENGTH-1);
	empty_up_line.length = UP_MESSAGE_LENGTH;

	// start timer
	HAL_TIM_Base_Start(&htim16);
	HAL_TIM_Base_Start(&htim17);

	// send some initial init messages
	uart_queue_push_line(&uart_input_queue, &init_message_line);

	if(is_splitter(setup_data.type))
	{
		for(si = 0; si < 8; si++)
		{
			if(setup_data.splitter_outputs[si]==1)
			{
				uart_queue_push_line(&uart_input_queue, &splitter_message_lines[si]);
				skc[si] = 1;
			}
		}
	}

	int redled_timeout_timer = __HAL_TIM_GetCounter(&htim16);

	int init_timout_timer = __HAL_TIM_GetCounter(&htim16);

	interrupt_up_timeout_time = __HAL_TIM_GetCounter(&htim16);
	interrupt_down_timeout_time = __HAL_TIM_GetCounter(&htim16);

	int new_sensor_timer = __HAL_TIM_GetCounter(&htim16);

	int up_timeout_timer = __HAL_TIM_GetCounter(&htim16);

	int kids_timer[] = {-1,-1,-1,-1,-1,-1,-1,-1};

	transmittedUP=0;
	receivedUP=0;
	transmittedDOWN=0;
	receivedDOWN=0;

	uint8_t TIMEDOUT = 0;

	update_sensor_messages();

	HAL_HalfDuplex_EnableTransmitterReceiver(huart_DOWN);
	HAL_HalfDuplex_EnableTransmitterReceiver(huart_UP);
	HAL_UART_Receive_DMA(huart_UP, up_buffer, DOWN_MESSAGE_LENGTH + 1);
	HAL_UART_Receive_DMA(huart_DOWN, down_buffer, DOWN_MESSAGE_LENGTH + 1);

	led_set(1, 1, 0);

	//while(1);

	//if(is_splitter(setup_data.type))
	HAL_WWDG_Start(&hwwdg);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	get_up_time = __HAL_TIM_GetCounter(&htim16);

	uint8_t WDGO = 0;

	uint8_t WDSET = 0;

  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

//	if((WD_UP && WD_DOWN) || DIRECTION_SET==0)
//	{
//		WD_UP = 0;
//		WD_DOWN = 0;
//		// for up and dwn we have own flags
//
//		HAL_WWDG_Refresh(&hwwdg, 127);
//	}

	if(DIRECTION_SET && WDSET==0)
	{
		HAL_WWDG_Start(&hwwdg);
		WDSET = 1;
		up_timeout_timer = __HAL_TIM_GetCounter(&htim16);
	}

	if(WDGO==0 || is_splitter(setup_data.type)==0)
	{
		HAL_WWDG_Refresh(&hwwdg, 127);
	}


	// if there was no poll from above a certain time = reset!
	if(__HAL_TIM_GetCounter(&htim16) - interrupt_up_timeout_time > 250)
	{
		WDGO = 1;

		get_up(1);

		interrupt_up_timeout_time = __HAL_TIM_GetCounter(&htim16);

		UP_STATE = IDLE;
	}


	// if there was no interrupt call for data from below
	if(__HAL_TIM_GetCounter(&htim16) - interrupt_down_timeout_time > 250)
	{
		WDGO = 1;

		HAL_UART_DMAStop(huart_DOWN);
		HAL_UART_DMAResume(huart_DOWN);

		if(!DIRECTION_SET)
		{
			HAL_HalfDuplex_EnableTransmitterReceiver(huart_DOWN);
			HAL_UART_Receive_DMA(huart_DOWN, down_buffer, DOWN_MESSAGE_LENGTH + 1);
		}

		interrupt_down_timeout_time = __HAL_TIM_GetCounter(&htim16);

		DOWN_STATE = IDLE;
	}

	  // wait for directions to be set
	if(!DIRECTION_SET)
	{
		led_set(8, 1, 8);
		continue;
	}

	led_set(8, 0, 8);

	if(__HAL_TIM_GetCounter(&htim16) - get_up_time > 20 && receivedUP == 0)
	{
		get_up(0);
	}

	// red blinking led
	if(__HAL_TIM_GetCounter(&htim16) - redled_timeout_timer > 250)
	{
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
		redled_timeout_timer = __HAL_TIM_GetCounter(&htim16);
	}

	if(__HAL_TIM_GetCounter(&htim16) - up_timeout_timer > 500)
	{
		WDGO = 1;
	}

	// sent init messages
	if(__HAL_TIM_GetCounter(&htim16) - init_timout_timer > 1000)
	{
		uart_queue_replace_push_line(&uart_input_queue, &init_message_line, setup_data.id);

		if(is_splitter(setup_data.type))
		{
			for(si = 0; si < 8; si++)
			{
				if(setup_data.splitter_outputs[si]==1)
				{
					uart_queue_push_line(&uart_input_queue, &splitter_message_lines[si]);
				}
			}
		}

		init_timout_timer = __HAL_TIM_GetCounter(&htim16);
	}

	if(SET_FLASH && UP_STATE == IDLE && DOWN_STATE == IDLE)
	{
		HAL_HalfDuplex_EnableTransmitter(huart_UP);

		set_in_Flash(&setup_data);
		SET_FLASH = 0;

		get_up(1);
	}

	if(PROGRAMM_SENSOR && UP_STATE == IDLE && DOWN_STATE == IDLE)
	{
		HAL_HalfDuplex_EnableTransmitter(huart_UP);

		if(setup_data.hardware == 2)
			init_2D(hspi1, GPIOA, GPIO_PIN_3);
		else
			init_2D(hspi1, GPIOB, GPIO_PIN_1);

		PROGRAMM_SENSOR = 0;

		get_up(1);
	}

	/// UP_STATE_MACHINE
	switch(UP_STATE)
	{
		case IDLE:
			if(SET_FLASH || PROGRAMM_SENSOR)
			{
				UP_STATE = IDLE;
				break;
			}

			if(receivedUP)
			{
				receivedUP = 0;

				if(up_buffer[0] == poll_receive /*&& __HAL_TIM_GetCounter(&htim16) - receivedUPtime < 5*/)
				{
					if(UP_RESEND_MESSAGE==0)
					{
						//update_sensor_messages();

						uint8_t n_up_messages = 0;

						while(uart_queue_is_empty(&uart_input_queue)==0 && (n_up_messages < UP_MESSAGES_MAX))
						{
							uart_queue_pop_line(&uart_input_queue, &send_up_line);
							memcpy(&poll_up_buffer[n_up_messages*(UP_MESSAGE_LENGTH)], send_up_line.text, UP_MESSAGE_LENGTH);

							n_up_messages++;
						}

						while(n_up_messages < UP_MESSAGES_MAX)
						{
							memcpy(&poll_up_buffer[n_up_messages*(UP_MESSAGE_LENGTH)], empty_up_line.text, UP_MESSAGE_LENGTH);

							n_up_messages++;
						}

						poll_up_size = n_up_messages*UP_MESSAGE_LENGTH;
					}

					UP_RESEND_MESSAGE = 0;
					UP_STATE = SENDING;

					HAL_HalfDuplex_EnableTransmitter(huart_UP);
					HAL_UART_Transmit_DMA(huart_UP, poll_up_buffer, poll_up_size);
					int something_else = __HAL_TIM_GetCounter(&htim16);

					if(up_buffer[1]!=empty)
					{
						memcpy(command_in_line.text, &up_buffer[1], DOWN_MESSAGE_LENGTH);
						command_in_line.length = DOWN_MESSAGE_LENGTH;

						uint8_t checksum = ComputeCRCN(command_in_line.text, DOWN_MESSAGE_LENGTH-1);

						if(checksum == command_in_line.text[DOWN_MESSAGE_LENGTH-1])
						{
							if(command_in_line.text[0] == poll_direct || getIDfromMessage(command_in_line.text)==setup_data.id)
							{
								uart_queue_push_line(&uart_command_queue, &command_in_line);

								if((command_in_line.text[0] == poll_send) && (setup_data.type==NODE_JOINT_TUT_TU))
									uart_queue_push_line(&uart_output_queue, &command_in_line);
							}
							else if(getIDfromMessage(command_in_line.text)==BROADCAST_ID)
							{
								uart_queue_push_line(&uart_command_queue, &command_in_line);
								uart_queue_push_line(&uart_output_queue, &command_in_line);
							}
							else
							{
								uart_queue_push_line(&uart_output_queue, &command_in_line);
							}
						}
					}

					if(is_splitter(setup_data.type))
					{
						/*__disable_irq();
						delayUS(10000);
						__enable_irq();*/
					}
					else
					{
						update_sensor_messages();
					}

					up_timeout_time = __HAL_TIM_GetCounter(&htim16);
				}
				else
				{
					//HAL_UART_DMAStop(huart_UP);
					//HAL_UART_DMAResume(huart_UP);

					// HAL_UART_Receive(huart_UP, &up_buffer[1], DOWN_MESSAGE_LENGTH, 2);

					get_up(1);
				}
			}
			break;

		case SENDING:
			if(transmittedUP)
			{
				UP_STATE = END;
				transmittedUP = 0;
			}
			else if(__HAL_TIM_GetCounter(&htim16) - up_timeout_time > 10)
			{
				HAL_UART_DMAStop(huart_UP);
				HAL_UART_DMAResume(huart_UP);
				UP_STATE = END;
			}
			break;

		case END:
			WD_UP = 1;
			up_timeout_timer = __HAL_TIM_GetCounter(&htim16);

			// work on commands

			UP_STATE = IDLE;
			get_up(1);
			break;
	}

	uint8_t s=0;

	switch(DOWN_STATE)
	{
		case DELAY:
			if(is_splitter(setup_data.type) || (__HAL_TIM_GetCounter(&htim16) - down_timeout_time > 7) /* && TIMEDOUT==0 */)
				DOWN_STATE = IDLE;

			/*else if (__HAL_TIM_GetCounter(&htim16) - down_timeout_time > 5)
				DOWN_STATE = IDLE;*/
			break;

		case IDLE:
			// do nothing
			if(SET_FLASH || PROGRAMM_SENSOR)
			{
				DOWN_STATE = IDLE;
				break;
			}


			if(is_splitter(setup_data.type))
			{
				if(splitter_kids_counter[i_p_splitter] > 10)
				{
					splitter_kids_counter[i_p_splitter] = 9;
					kids_timer[i_p_splitter] = __HAL_TIM_GetCounter(&htim16) + 200;
				}

				if(kids_timer[i_p_splitter]!=-1 && kids_timer[i_p_splitter] > __HAL_TIM_GetCounter(&htim16))
				{
					interrupt_down_timeout_time = __HAL_TIM_GetCounter(&htim16);
					DOWN_STATE = END;
					break;
				}

				if(skc[i_p_splitter] == 0)
				{
					send_down_line = send_down_line_now;
				}
				else
				{
					send_down_line = empty_command_line;
				}
			}


			if(skc[0]+skc[1]+skc[2]+skc[3]+skc[4]+skc[5]+skc[6]+skc[7] >= n_splitter)
			{
				if(uart_queue_is_empty(&uart_output_queue)==0)
				{
					uart_queue_pop_line(&uart_output_queue, &send_down_line_now);
					send_down_line = send_down_line_now;
					for(s=0; s<n_splitter; s++)
						skc[s] = 0;
				}
				else
				{
					send_down_line = empty_command_line;
					for(s=0; s<n_splitter; s++)
						skc[s] = 1;
				}
			}

			//send_down_line = empty_command_line;

			if(is_splitter(setup_data.type))
			{
				MPX_UART_Open(i_p_splitter);
			}

			DOWN_STATE = RECEIVING;

			//__disable_irq();
			HAL_HalfDuplex_EnableTransmitter(huart_DOWN);
			status = HAL_UART_Transmit(huart_DOWN, "R", 1, 3);

			status = HAL_UART_Transmit(huart_DOWN, send_down_line.text, DOWN_MESSAGE_LENGTH, 5);
			//__enable_irq();

			// DMA Receive for 3
			HAL_HalfDuplex_EnableTransmitterReceiver(huart_DOWN);
			HAL_UART_Receive_DMA(huart_DOWN, poll_down_buffer, UP_MESSAGE_LENGTH*UP_MESSAGES_MAX);

			down_timeout_time = __HAL_TIM_GetCounter(&htim16);
			break;

		case RECEIVING:
			if(receivedDOWN)
			{
				receivedDOWN = 0;
				splitter_kids_counter[i_p_splitter] = 0;
				skc[i_p_splitter] = 1;
				kids_timer[i_p_splitter] = -1;

				// no more transmitting checksum, only calculating
				uint8_t checksum = ComputeCRCN(&poll_down_buffer[poll_down_size-UP_MESSAGE_LENGTH], UP_MESSAGE_LENGTH-1);

				uint8_t i = 0;

				for(i=0; i<UP_MESSAGES_MAX; i++)
				{
					// check checksum
					if(1 /*checksum == poll_down_buffer[i*UP_MESSAGE_LENGTH-1]*/)
					{
						Line sensor_message_line;
						memcpy(sensor_message_line.text, &poll_down_buffer[i*UP_MESSAGE_LENGTH], UP_MESSAGE_LENGTH);
						sensor_message_line.length = UP_MESSAGE_LENGTH;

						if(sensor_message_line.text[0]==empty)
							break;

						// delete messages from T joint but the answer_data, there update angles[3]
						if(getIDfromMessage(sensor_message_line.text)==setup_data.id && setup_data.type == NODE_JOINT_TUT_TU && data_mode!=2)
						{
							if(sensor_message_line.text[0] == answer_data)
								angles[3] = ((sensor_message_line.text[15] << 8) & 0xFF00) + sensor_message_line.text[16];
						}
						else // for the other kinds (splitter, init, data from other) replace them if already in queue, ack messages always add to queue
						{
							// if it's from own child, add own id
							if(sensor_message_line.text[5]==0)
							{
								sensor_message_line.text[5] = (setup_data.id >> 24) & 0xFF;
								sensor_message_line.text[6] =  (setup_data.id >> 16) & 0xFF;
								sensor_message_line.text[7] = (setup_data.id >> 8) & 0xFF;
								sensor_message_line.text[8] = setup_data.id & 0xFF;

								// add splitter branch number
								if(is_splitter(setup_data.type))
									sensor_message_line.text[5] |= ((i_p_splitter << 4) & 0xF0);
							}

							UpdateCRCLine(&sensor_message_line);

							if(sensor_message_line.text[0] == answer_ack || sensor_message_line.text[0] == splitter)
								uart_queue_push_line(&uart_input_queue, &sensor_message_line);
							else
								uart_queue_replace_push_line(&uart_input_queue, &sensor_message_line, getIDfromMessage(sensor_message_line.text));
						}
					}
				}

				DOWN_STATE = END;
				TIMEDOUT = 0;
			}
			else if(__HAL_TIM_GetCounter(&htim16) - down_timeout_time > 13)
			{
				HAL_UART_DMAStop(huart_DOWN);
				HAL_UART_DMAResume(huart_DOWN);

				TIMEDOUT = 1;
				splitter_kids_counter[i_p_splitter]++;
				DOWN_STATE = END;
			}
			break;

		case END:
			WD_DOWN = 1;
			i_p_splitter = (i_p_splitter+1) % (n_splitter);

			HAL_UART_DMAStop(huart_DOWN);
			HAL_UART_DMAResume(huart_DOWN);

			if(i_p_splitter==0)
			{
				down_timeout_time = __HAL_TIM_GetCounter(&htim16);
				DOWN_STATE = DELAY;
			}
			else
			{
				DOWN_STATE = IDLE;
			}
			break;
	}

	/// work on commands
	if(uart_queue_is_empty(&uart_command_queue)==0)
	{
		// process command queue
		uart_queue_pop_line(&uart_command_queue, &current_command_line);

		if(process_uart_command(&current_command_line))
		{
			// acknowledge
			create_up_message(&ack_message_line, answer_ack, current_command_line.text);
			uart_queue_push_line(&uart_input_queue, &ack_message_line);
		}
	}
  }

  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

}

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  hspi1.Init.CRCPolynomial = 10;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLED;
  HAL_SPI_Init(&hspi1);

}

/* TIM16 init function */
void MX_TIM16_Init(void)
{

  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 8000;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 0xFFFF;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&htim16);

}

/* TIM17 init function */
void MX_TIM17_Init(void)
{

  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 8;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 0xFFFF;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&htim17);

}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 230400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED ;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_HalfDuplex_Init(&huart1);

}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 230400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED ;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_HalfDuplex_Init(&huart2);

}

/* WWDG init function */
void MX_WWDG_Init(void)
{

  hwwdg.Instance = WWDG;
  hwwdg.Init.Prescaler = WWDG_PRESCALER_2;
  hwwdg.Init.Window = 127;
  hwwdg.Init.Counter = 127;
  HAL_WWDG_Init(&hwwdg);

}

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
