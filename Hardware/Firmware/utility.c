/*
 * spi_functions.c
 *
 *  Created on: Mar 05, 2015
 *      Author: glausero
 */

/* includes */

#include "utility.h"
#include "global.h"

// memory variables
uint32_t Memory_Page_Address = 0x0800FC00;
config_data_flash_struct setup_data;

extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;


void get_up(uint8_t reset)
{
	if(reset)
	{
		HAL_UART_DMAStop(huart_UP);
		HAL_UART_DMAResume(huart_UP);
	}

	get_up_time = __HAL_TIM_GetCounter(&htim16);

	HAL_HalfDuplex_EnableTransmitterReceiver(huart_UP);
	HAL_UART_Receive_DMA(huart_UP, up_buffer, DOWN_MESSAGE_LENGTH + 1);
}

void delayUS(uint16_t delayus)
{
	static int end;

	end = __HAL_TIM_GetCounter(&htim17) + delayus;

	if(end > 0xFFFF)
	{
		__HAL_TIM_SetCounter(&htim17,0);
		while(__HAL_TIM_GetCounter(&htim17) < delayus);
	}
	else
	{
		while(__HAL_TIM_GetCounter(&htim17) < end);
	}
}

unsigned char charToHexDigit(char c)
{
  if (c >= 'A')
    return c - 'A' + 10;
  else
    return c - '0';
}

char hexDigitToChar(unsigned char c)
{
  if (c >= 10)
    return c - 10 + 'A';
  else
    return c + '0';
}


uint8_t process_uart_command(Line *l)
{
	static int module_id = 0, joint_type;
	//static int rate_r = 0, rate_g = 0, rate_b = 0;
	static int n;
	static uint8_t green, blue, red;

	// (P | S) (ID)(ID)(ID)(ID) (command character) (parameters) (parameters)

	switch (l->text[5])
	{
		case 'b':
			// what?
			break;
		case 'l':
			if(l->text[6]==0)
			{
				blue=0;
			}
			else
			{
				blue=1;
			}

			if(l->text[7]==0)
			{
				red=0;
			}
			else
			{
				red=1;
			}

			if(l->text[8]==0)
			{
				green=0;
			}
			else
			{
				green=1;
			}

			led_set(red, blue, green);

			break;
		case 's':
			if(l->text[0]==poll_direct)
			{
				char sensor_message[] = {'S', (setup_data.id >> 24) & 0xFF, (setup_data.id >> 16) & 0xFF, (setup_data.id >> 8) & 0xFF, setup_data.id & 0xFF, (setup_data.type >> 4), setup_data.type & 0x0F, '0', '0' ,'I', 'N', 'I', 'T', 'M', '\r', '\n'};

				Line com_message_line;
				memcpy(com_message_line.text, sensor_message, UP_MESSAGE_LENGTH);
				com_message_line.length = UP_MESSAGE_LENGTH;

				uart_queue_push_line(&uart_input_queue, &com_message_line);
			}
			break;
		case 'i':
			if(l->text[0]==poll_direct)
			{
				setup_data.id = (l->text[1] << 24) + (l->text[2] << 16) + (l->text[3] << 8) + l->text[4];

				// the brother T joint gets the same ID, that way the T joint can easily discover
				if(setup_data.type == NODE_JOINT_TUT_TU )
				{
					char sensor_message[] = {'P', l->text[1], l->text[2], l->text[3], l->text[4], 'i', 0, 0, 0, 0};

					Line com_message_line;
					memcpy(com_message_line.text, sensor_message, DOWN_MESSAGE_LENGTH);
					com_message_line.length = DOWN_MESSAGE_LENGTH;
					UpdateCRCLine(&com_message_line);


					uart_queue_push_line(&uart_output_queue, &com_message_line);
				}

				SET_FLASH = 1;
				part_init();
			}
			break;
		case 't':
			if(l->text[0]==poll_direct) // set type of part
			{
				uint16_t temp = charToHexDigit(l->text[6]);
				uint16_t temp2 = charToHexDigit(l->text[7]);

				setup_data.type = ((temp << 4) & 0xF0) + (temp2 & 0xF);

				if(is_splitter(setup_data.type))
				{
					uint8_t n_splitter = (setup_data.type & 0xF0) >> 4;

					for(n=0; n<n_splitter; n++)
					{
						setup_data.splitter_outputs[n] = 1;
					}

					for(n=n_splitter; n<8; n++)
					{
						setup_data.splitter_outputs[n] = 0;
					}
				}

				// if it's a TU joint, also the type of it's brother T joint has to be set
				if(setup_data.type == NODE_JOINT_TUT_TU )
				{
					char sensor_message[] = {'P', 0, 0, 0, 0, 't', hexDigitToChar((NODE_JOINT_TUT_T >> 4) & 0xF) , hexDigitToChar(NODE_JOINT_TUT_T & 0xF),0,0};

					Line com_message_line;
					memcpy(com_message_line.text, sensor_message, DOWN_MESSAGE_LENGTH);
					com_message_line.length = DOWN_MESSAGE_LENGTH;
					UpdateCRCLine(&com_message_line);

					uart_queue_push_line(&uart_output_queue, &com_message_line);
				}

				SET_FLASH = 1;
				part_init();
			}
			break;
		case 'd':
			data_mode = l->text[6];
			break;
		case 'p':
			if(l->text[0]==poll_direct && setup_data.type == NODE_JOINT_TUT_TU) // program 2D melexis sensor
			{
				PROGRAMM_SENSOR = 1;
			}
			break;
		case 'x': // set polarity
			setup_data.polarity = l->text[6];
			SET_FLASH = 1;
			part_init();
		case 'z': // zeroing .. set current angles as zero
			for(n=0; n<4; n++)
			{
				setup_data.zero_angles[n] = angles[n];
			}
			SET_FLASH = 1;
			part_init();
			break;
		case 'g': // set splitter geometry
			if(l->text[0]==poll_direct)
			{
				//get 4 bytes (1 - 4) & 2 bytes (6 - 7)
				setup_data.splitter_roll[l->text[8]] = (l->text[1] << 8) + l->text[2];
				setup_data.splitter_pitch[l->text[8]] = (l->text[3] << 8) + l->text[4];
				setup_data.splitter_yaw[l->text[8]] = (l->text[6] << 8) + l->text[7];
				//setup_data.splitter_outputs[l->text[8]] = 1;
				SET_FLASH = 1;
				part_init();
			}
			break;
		case 'c': // colring, set colors
				//get 4 bytes (1 - 4) & 2 bytes (6 - 7)
				setup_data.color[0] = (uint8_t) l->text[6];
				setup_data.color[1] = (uint8_t) l->text[7];
				setup_data.color[2] = (uint8_t) l->text[8];
				SET_FLASH = 1;
				part_init();
			break;
		case 'h': // colring, set colors
			if(l->text[0]==poll_direct)
			{
				//get 4 bytes (1 - 4) & 2 bytes (6 - 7)
				setup_data.hardware = l->text[6];
				SET_FLASH = 1;

				if(setup_data.type == NODE_JOINT_TUT_TU )
				{
					char sensor_message[] = {'P', 0, 0, 0, 0, 'h', l->text[5], 0, 0};

					Line com_message_line;
					memcpy(com_message_line.text, sensor_message, DOWN_MESSAGE_LENGTH);
					com_message_line.length = DOWN_MESSAGE_LENGTH;

					UpdateCRCLine(&com_message_line);

					uart_queue_push_line(&uart_output_queue, &com_message_line);
				}

				part_init();
			}
			break;
 		default:
			not_understood_label:
			return 0;
			break;
	}

	return 1;
}

void create_up_messageZ(Line* message_line, uint8_t type)
{
	uint8_t zeros[8] = {0};

	create_up_message(message_line, type, zeros);
}

void create_up_message(Line* message_line, uint8_t type, uint8_t* content_8_bytes)
{
	message_line->text[0] = type;

	message_line->text[1] = (setup_data.id >> 24) & 0xFF;
	message_line->text[2] = (setup_data.id >> 16) & 0xFF;
	message_line->text[3] = (setup_data.id >> 8) & 0xFF;
	message_line->text[4] = (setup_data.id) & 0xFF;

	message_line->text[5] = 0;
	message_line->text[6] = 0;
	message_line->text[7] = 0;
	message_line->text[8] = 0;

	uint8_t i;

	for(i=0; i<8; i++)
	{
		message_line->text[9+i] = content_8_bytes[i];
	}

	message_line->text[17] = ComputeCRCN(message_line->text, 17);

	message_line->length = 18;

	return;
}

uint8_t is_splitter(uint16_t type)
{
	if((type & 0x0F) == 0x08)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

uint8_t is_joint(uint16_t type)
{
	if((type & 0x0F) == 0x04)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

void reset_joint()
{
	uint8_t i=0;

	for(i=0; i<4; i++)
	{
		setup_data.zero_angles[i] = 0;
	}

	for(i=0; i<8; i++)
	{
		setup_data.splitter_roll[i] = 0;
		setup_data.splitter_pitch[i] = 0;
		setup_data.splitter_yaw[i] = 0;
		setup_data.splitter_outputs[i] = 0;
	}

	for(i=0; i<3; i++)
	{
		setup_data.color[i] = 0;
	}

	setup_data.type = NODE_UNDEFINED;
	setup_data.id = 0xFFFFFFF;
	setup_data.hardware = 2;
	setup_data.initied_0xAA = 0xAA;

	set_in_Flash(&setup_data);
}

void LED_init()
{
	// RED
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

	// BLUE
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);

	if(setup_data.hardware == 2)
	{
		// set spi chip select high /actually led green
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	}
}

void led_on()
{
	// RED
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

	// BLUE
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);

	if(setup_data.hardware == 2)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	}
}

void led_off()
{
	// RED
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

	// BLUE
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

	if(setup_data.hardware == 2)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	}
}

void led_set(uint8_t red, uint8_t blue, uint8_t green)
{
	// RED
	if(red!=8)
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, red);

	// BLUE
	if(blue!=8)
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, blue);

	if(setup_data.hardware == 2)
	{
		// BLUE
		if(green!=8)
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, green);
	}
}

/**
  * @brief  This function stores the current config data in flash
  * @param  None
  * @retval None
  */
void set_in_Flash(config_data_flash_struct *setup)
{
	uint32_t *pData = (uint32_t *)setup;
	uint8_t i;
	int x = (sizeof(config_data_flash_struct) / sizeof(int));

	__disable_irq();

	HAL_FLASH_Unlock();

	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);

	FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.TypeErase = TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = Memory_Page_Address;
	EraseInitStruct.NbPages = 1;

	uint32_t PageError;

	HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);

	for(i=0;i<x;i++)
	{
		HAL_FLASH_Program(TYPEPROGRAM_WORD, Memory_Page_Address+4*i, pData[i]);
	}

	HAL_FLASH_Lock();

	__enable_irq();

/*
	uint8_t i;
	int x = (sizeof(config_data_flash_struct) / sizeof(int));

	HAL_FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
	FLASH_ErasePage(Memory_Page_Address);

	for(i=0;i<x;i++)
	{
		FLASH_ProgramWord(Memory_Page_Address+4*i, pData[i]);
	}

	FLASH_Lock();

	__enable_irq();*/
}

void get_from_Flash()
{
	memcpy ( &setup_data, ((config_data_flash_struct*)Memory_Page_Address), sizeof(config_data_flash_struct) );
}

void part_init()
{
	// some initialisation depending if it's a joint or a splitter
	if(is_joint(setup_data.type))
	{
		// setup GPIO_PIN_3 ..shiiiiiiiiit
		if(setup_data.hardware == 2)
		{

			GPIO_InitTypeDef GPIO_InitStruct;
		  /*Configure GPIO pin : PA3 */
		  GPIO_InitStruct.Pin = GPIO_PIN_3;
		  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		  GPIO_InitStruct.Pull = GPIO_NOPULL;
		  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
		  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		}

		// init multiplexer pins
		MX_SPI1_Init_Pointer();

		if(setup_data.hardware != 2)
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

		}
		else
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
		}

		// set spi chip select high
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

		// init mlx sensors, actually just some defines
		init_1D();

		n_splitter = 1;
	}

	uint8_t j;

	for(j=0; j<4; j++)
	{
		angles[j] = 0;
	}

	if(is_splitter(setup_data.type))
	{
	  // init multiplexer pins and enable it
	  MPX_UART_Init();
	  MPX_UART_ONOFF(1);

	  // read number of inputs .. should actually do even more?
	  n_splitter = (setup_data.type & 0xF0) >> 4;
	}

	//=================================================================================
	/// mess from here on TOOD: do better!
	//=================================================================================
	//---------------------------------------------------------------------------------
	// create some basic messsage types

	// INIT MESSAGE
	uint8_t init_data[] = {setup_data.type, setup_data.color[0], setup_data.color[1], setup_data.color[2], 0, 0, 0, 0};
	create_up_message(&init_message_line, init, init_data);

	// SPLITTER MESSAGE

	uint16_t si = 0;

	if(is_splitter(setup_data.type))
	{
		uint8_t splitter_data[8];

		for(si = 0; si < 8; si++)
		{
			if(setup_data.splitter_outputs[si]==1)
			{
				splitter_data[0] = 0;
				splitter_data[1] = si;
				splitter_data[2] = (setup_data.splitter_roll[si] >> 8) & 0xFF;
				splitter_data[3] = setup_data.splitter_roll[si] & 0xFF;
				splitter_data[4] = (setup_data.splitter_pitch[si] >> 8) & 0xFF;
				splitter_data[5] = setup_data.splitter_pitch[si] & 0xFF;
				splitter_data[6] = (setup_data.splitter_yaw[si] >> 8) & 0xFF;
				splitter_data[7] = setup_data.splitter_yaw[si] & 0xFF;

				create_up_message(&splitter_message_lines[si], splitter, splitter_data);
			}
		}
	}

	// ACK MESSAGE
	//...

	// SENSOR MESSAGE
	create_up_messageZ(&sensor_message_line, answer_data);

	i_p_splitter=0;
	i_s_splitter=0;
}


void update_sensor_messages()
{
	static uint16_t anglesZ[3];
	static uint8_t good_data;

	good_data=0;

	if(setup_data.type == NODE_JOINT_TUT_TU)
	{
		uint16_t time = __HAL_TIM_GetCounter(&htim16);

		get_1D_hex(hspi1, GPIOA, GPIO_PIN_4, &angles[0]);

		if(setup_data.hardware==2)
		{
			get_2D_hex(hspi1, GPIOA, GPIO_PIN_3, &angles[1], &angles[2]);
		}
		else
		{
			get_2D_hex(hspi1, GPIOB, GPIO_PIN_1, &angles[1], &angles[2]);
		}


		/*anglesZ[0]= (angles[0] - setup_data.zero_angles[0] + 0x3FFF) % 0x3FFF;

		anglesZ[1]= (angles[1] - setup_data.zero_angles[1] + 0x3FFF +  0xFFF) % 0x3FFF;
		anglesZ[2]= (angles[2] - setup_data.zero_angles[2] + 0x3FFF +  0xFFF) % 0x3FFF;

		anglesZ[3]= (angles[3] - setup_data.zero_angles[3] + 0x3FFF) % 0x3FFF;*/


		//uint16_t time = __HAL_TIM_GetCounter(&htim16);

		//data_mode = 1;

		if((angles[0]!=0 && angles[1]!=0 && angles[2]!=0 && angles[3]!=0) || data_mode !=0)
		{
			if(setup_data.polarity==1)
			{
				angles[1] = 0x3FFF - angles[1];
				angles[2] = 0x3FFF - angles[2];
			}

			anglesZ[0]= (angles[0] - setup_data.zero_angles[0] + 0x3FFF) % 0x3FFF;

			anglesZ[1]= (angles[1] - setup_data.zero_angles[1] + 0x3FFF +  0xFFF) % 0x3FFF;
			anglesZ[2]= (angles[2] - setup_data.zero_angles[2] + 0x3FFF +  0xFFF) % 0x3FFF;

			anglesZ[3]= (angles[3] - setup_data.zero_angles[3] + 0x3FFF) % 0x3FFF;

			/*float a0 = 0.02197*angles[0];
			float a1 = 0.02197*angles[1];
			float a2 = 0.02197*angles[2];
			float a3 = 0.02197*angles[3];*/

			uint8_t ai;

			for(ai = 0; ai < 4; ai++)
			{
				sensor_message_line.text[9+2*ai] = (anglesZ[ai] >> 8) & 0xFF;
				sensor_message_line.text[10+2*ai] = anglesZ[ai] & 0xFF;
			}

			angles[3] = 0;
			good_data=1;
		}
	}

	if(setup_data.type == NODE_JOINT_TUT_T)
	{
		get_1D_hex(hspi1, GPIOA, GPIO_PIN_4,  &angles[3]);

		if(angles[3]!=0)
		{
			sensor_message_line.text[15] = (angles[3] >> 8) & 0xFF;
			sensor_message_line.text[16] = angles[3] & 0xFF;

			good_data=1;
		}
	}

	if(data_mode!=0)
	{
		good_data = 1;
	}

	//sgood_data = 1;

	if((setup_data.type == NODE_JOINT_TUT_TU || setup_data.type == NODE_JOINT_TUT_T) && (good_data==1))
	{
		UpdateCRCLine(&sensor_message_line);

		uart_queue_replace_push_line(&uart_input_queue, &sensor_message_line, setup_data.id);
	}
}
