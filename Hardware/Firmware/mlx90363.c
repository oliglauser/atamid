#include "mlx90363.h"

//Define and initialize CRC array, 256 bytes
char CRCArray[] = {
0x00, 0x2F, 0x5E, 0x71, 0xBC, 0x93, 0xE2, 0xCD, 0x57, 0x78, 0x09, 0x26,
0xEB, 0xC4, 0xB5, 0x9A, 0xAE, 0x81, 0xF0, 0xDF, 0x12, 0x3D, 0x4C, 0x63,
0xF9, 0xD6, 0xA7, 0x88, 0x45, 0x6A, 0x1B, 0x34, 0x73, 0x5C, 0x2D, 0x02,
0xCF, 0xE0, 0x91, 0xBE, 0x24, 0x0B, 0x7A, 0x55, 0x98, 0xB7, 0xC6, 0xE9,
0xDD, 0xF2, 0x83, 0xAC, 0x61, 0x4E, 0x3F, 0x10, 0x8A, 0xA5, 0xD4, 0xFB,
0x36, 0x19, 0x68, 0x47, 0xE6, 0xC9, 0xB8, 0x97, 0x5A, 0x75, 0x04, 0x2B,
0xB1, 0x9E, 0xEF, 0xC0, 0x0D, 0x22, 0x53, 0x7C, 0x48, 0x67, 0x16, 0x39,
0xF4, 0xDB, 0xAA, 0x85, 0x1F, 0x30, 0x41, 0x6E, 0xA3, 0x8C, 0xFD, 0xD2,
0x95, 0xBA, 0xCB, 0xE4, 0x29, 0x06, 0x77, 0x58, 0xC2, 0xED, 0x9C, 0xB3,
0x7E, 0x51, 0x20, 0x0F, 0x3B, 0x14, 0x65, 0x4A, 0x87, 0xA8, 0xD9, 0xF6,
0x6C, 0x43, 0x32, 0x1D, 0xD0, 0xFF, 0x8E, 0xA1, 0xE3, 0xCC, 0xBD, 0x92,
0x5F, 0x70, 0x01, 0x2E, 0xB4, 0x9B, 0xEA, 0xC5, 0x08, 0x27, 0x56, 0x79,
0x4D, 0x62, 0x13, 0x3C, 0xF1, 0xDE, 0xAF, 0x80, 0x1A, 0x35, 0x44, 0x6B,
0xA6, 0x89, 0xF8, 0xD7, 0x90, 0xBF, 0xCE, 0xE1, 0x2C, 0x03, 0x72, 0x5D,
0xC7, 0xE8, 0x99, 0xB6, 0x7B, 0x54, 0x25, 0x0A, 0x3E, 0x11, 0x60, 0x4F,
0x82, 0xAD, 0xDC, 0xF3, 0x69, 0x46, 0x37, 0x18, 0xD5, 0xFA, 0x8B, 0xA4,
0x05, 0x2A, 0x5B, 0x74, 0xB9, 0x96, 0xE7, 0xC8, 0x52, 0x7D, 0x0C, 0x23,
0xEE, 0xC1, 0xB0, 0x9F, 0xAB, 0x84, 0xF5, 0xDA, 0x17, 0x38, 0x49, 0x66,
0xFC, 0xD3, 0xA2, 0x8D, 0x40, 0x6F, 0x1E, 0x31, 0x76, 0x59, 0x28, 0x07,
0xCA, 0xE5, 0x94, 0xBB, 0x21, 0x0E, 0x7F, 0x50, 0x9D, 0xB2, 0xC3, 0xEC,
0xD8, 0xF7, 0x86, 0xA9, 0x64, 0x4B, 0x3A, 0x15, 0x8F, 0xA0, 0xD1, 0xFE,
0x33, 0x1C, 0x6D, 0x42 };


uint8_t SPI_MASTER_Buffer_Rx[RxBufferSize];

float f32_alpha_angle_degrees, f32_beta_angle_degrees;
uint16_t u16_alpha_angle_lsb, u16_beta_angle_lsb;
const float f32_lsb_to_dec_degrees = 0.02197;

float f32_angle_degrees;
uint16_t u16_angle_lsb;
char u8_error_lsb;
char u8_rollcnt_dec;
char u8_virtualgain_dec;
char u8_crc_dec;

char nop_command[8];
char nop_answer[8];

char get1_command[8];
char get1_answer[8];

extern WWDG_HandleTypeDef hwwdg;

uint8_t SPI_MASTER_Buffer_Tx[TxBufferSize];
//

uint8_t ComputeCRC(uint8_t Byte0, uint8_t Byte1, uint8_t Byte2, uint8_t Byte3, uint8_t Byte4, uint8_t Byte5, uint8_t Byte6)
{
	uint8_t CRCC = 0xFF;
	CRCC = CRCArray[CRCC ^ Byte0];
	CRCC = CRCArray[CRCC ^ Byte1];
	CRCC = CRCArray[CRCC ^ Byte2];
	CRCC = CRCArray[CRCC ^ Byte3];
	CRCC = CRCArray[CRCC ^ Byte4];
	CRCC = CRCArray[CRCC ^ Byte5];
	CRCC = CRCArray[CRCC ^ Byte6];
	CRCC = ~CRCC;
	return CRCC;
}

void UpdateCRCLine(Line* message_line)
{
	uint8_t CRCC = 0xFF;

	uint8_t i;

	for(i=0; i < message_line->length-1; i++)
	{
		CRCC = CRCArray[CRCC ^ (message_line->text[i])];
	}

	CRCC = ~CRCC;

	message_line->text[message_line->length-1] = CRCC;
}

uint8_t ComputeCRCN(uint8_t* bytes, uint8_t length)
{
	uint8_t CRCC = 0xFF;

	uint8_t i;

	for(i=0; i<length; i++)
	{
		CRCC = CRCArray[CRCC ^ bytes[i]];
	}

	CRCC = ~CRCC;
	return CRCC;
}

void init_1D()
{
	SPI_MASTER_Buffer_Tx[0] = 0x00;
	SPI_MASTER_Buffer_Tx[1] = 0x00;
	SPI_MASTER_Buffer_Tx[2] = 0xFF;
	SPI_MASTER_Buffer_Tx[3] = 0xFF;
	SPI_MASTER_Buffer_Tx[4] = 0x00;
	SPI_MASTER_Buffer_Tx[5] = 0x00;
	SPI_MASTER_Buffer_Tx[6] = 0x53;
	SPI_MASTER_Buffer_Tx[7] = ComputeCRC(0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x53);

	nop_command[0] = 0x00;
	nop_command[1] = 0x00;
	nop_command[2] = 0xAA;
	nop_command[3] = 0xAA;
	nop_command[4] = 0x00;
	nop_command[5] = 0x00;
	nop_command[6] = 0b11010000;
	nop_command[7] = ComputeCRC(0x00, 0x00, 0xAA, 0xAA, 0x00, 0x00, 0b11010000);

	get1_command[0] = 0x00;
	get1_command[1] = 0x00;
	get1_command[2] = 0xFF;
	get1_command[3] = 0xFF;
	get1_command[4] = 0x00;
	get1_command[5] = 0x00;
	get1_command[6] = 0x13;
	get1_command[7] = ComputeCRC(0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x13);

	// INIT & GET 1D
}

void get_1D(SPI_HandleTypeDef hspx, GPIO_TypeDef* GPIOx, uint16_t GPIO_PIN_x, float *alpha)
{
	//__disable_irq();

	HAL_GPIO_WritePin(GPIOx, GPIO_PIN_x, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_SPI_TransmitReceive(&hspx, get1_command, get1_answer, 8, 50);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOx, GPIO_PIN_x, GPIO_PIN_SET);

	HAL_Delay(10); //delay for writing EEPROM

	HAL_GPIO_WritePin(GPIOx, GPIO_PIN_x, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_SPI_TransmitReceive(&hspx, nop_command, nop_answer, 8, 50);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOx, GPIO_PIN_x, GPIO_PIN_SET);

	//Extract and convert the angle to degrees
	//remove error bits and shift to high byte
	u16_angle_lsb = (nop_answer[1] & 0x3F) << 8;
	//add LSB of angle
	u16_angle_lsb = u16_angle_lsb + nop_answer[0];
	//convert to decimal degrees
	f32_angle_degrees = u16_angle_lsb * f32_lsb_to_dec_degrees;
	//Extract the error bits
	u8_error_lsb = nop_answer[1] >> 6;
	//Extract the CRC
	u8_crc_dec = nop_answer[7];
	//Extract the virtual gain byte
	u8_virtualgain_dec = nop_answer[4];
	//Extract the rolling counter
	u8_rollcnt_dec = nop_answer[6] & 0x3F;

	*alpha = f32_angle_degrees;

	//__enable_irq();
}

void get_1D_hex(SPI_HandleTypeDef hspx, GPIO_TypeDef* GPIOx, uint16_t GPIO_PIN_x, uint16_t *alpha)
{
	//

	__disable_irq();
	HAL_GPIO_WritePin(GPIOx, GPIO_PIN_x, GPIO_PIN_RESET);

	//HAL_Delay(1);
	delayUS(50);
	HAL_SPI_TransmitReceive(&hspx, get1_command, get1_answer, 8, 5);
	//HAL_Delay(1);
	delayUS(50);

	HAL_GPIO_WritePin(GPIOx, GPIO_PIN_x, GPIO_PIN_SET);

	//HAL_Delay(10);
	delayUS(1000);

	HAL_GPIO_WritePin(GPIOx, GPIO_PIN_x, GPIO_PIN_RESET);

	//HAL_Delay(1);
	delayUS(50);
	HAL_SPI_TransmitReceive(&hspx, nop_command, nop_answer, 8, 5);
	//HAL_Delay(1);
	delayUS(50);

	HAL_GPIO_WritePin(GPIOx, GPIO_PIN_x, GPIO_PIN_SET);

	__enable_irq();

	//Extract and convert the angle to degrees
	//remove error bits and shift to high byte
	u16_angle_lsb = (nop_answer[1] & 0x3F) << 8;
	//add LSB of angle
	u16_angle_lsb = u16_angle_lsb + nop_answer[0];

	u8_error_lsb = nop_answer[1] >> 6;

	if((u8_error_lsb & 0b10) && (ComputeCRCN(nop_answer, 7)==nop_answer[7]))
	{
		*alpha = u16_angle_lsb;
	}
	else
	{
		char reboot_command[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xEF, ComputeCRC(0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xEF)};

		/*delayUS(1000);

		HAL_GPIO_WritePin(GPIOx, GPIO_PIN_x, GPIO_PIN_RESET);
		delayUS(10);
		HAL_GPIO_WritePin(GPIOx, GPIO_PIN_x, GPIO_PIN_SET);

		delayUS(1000);*/

		/*HAL_SPI_MspDeInit(&hspx);
		HAL_SPI_MspInit(&hspx);*/

		__disable_irq();

		HAL_SPI_DeInit (&hspx);

		delayUS(100);

		HAL_SPI_Init(&hspx);

		delayUS(100);

		__enable_irq();

		/*HAL_GPIO_WritePin(GPIOx, GPIO_PIN_x, GPIO_PIN_RESET);
		delayUS(50);
		HAL_SPI_TransmitReceive(&hspx, reboot_command, nop_answer, 8, 5);
		delayUS(50);
		HAL_GPIO_WritePin(GPIOx, GPIO_PIN_x, GPIO_PIN_SET);

		delayUS(2000);*/

		//*alpha = u16_angle_lsb;

		*alpha = 0;
	}

	//
}

// INIT & GET 2D

void init_2D(SPI_HandleTypeDef hspx, GPIO_TypeDef* GPIOx, uint16_t GPIO_PIN_x)
{
	//__disable_irq();

	__disable_irq();
	init_1D();

	static int i=0;

	delayUS(5000);

	char mem_read_command[] = {0x2A, 0x10, 0x00, 0x10, 0x00, 0x00, 0xC1, ComputeCRC(0x2A, 0x10, 0x00, 0x10, 0x00, 0x00, 0xC1)};
	char mem_read_answer[8];

	for (i = 0; i <= 1; i++)
	{
		HAL_GPIO_WritePin(GPIOx, GPIO_PIN_x, GPIO_PIN_RESET);
		delayUS(100);
		HAL_SPI_TransmitReceive(&hspx, mem_read_command, mem_read_answer, 8, 50);
		//send_receive_spi(selected_spi, mem_read_command, mem_read_answer, 8);
		delayUS(100);
		HAL_GPIO_WritePin(GPIOx, GPIO_PIN_x, GPIO_PIN_SET);

		delayUS(5000);
	}

	char Addr_102A_LSB = (mem_read_answer[0] & 0xF0) ^ 0x08;
	char Addr_102A_MSB = mem_read_answer[1] | 0x03;
	//Set Kalpha to 1.4
	//char Addr_1022_LSB = 0x33;
	//char Addr_1022_MSB = 0xB3;

	//Set Kbeta to 1.4
	//char Addr_1024_LSB = 0x33;
	//char Addr_1024_MSB = 0xB3;

	//Set Kalpha to 1.8
	char Addr_1022_LSB = 0x00;
	char Addr_1022_MSB = 0x40;

	//Set Kbeta to 1.8
	char Addr_1024_LSB = 0x00;
	char Addr_1024_MSB = 0x40;


	HAL_GPIO_WritePin(GPIOx, GPIO_PIN_x, GPIO_PIN_RESET);
	delayUS(100);
	HAL_SPI_TransmitReceive(&hspx, nop_command, nop_answer, 8, 50);
	delayUS(100);
	HAL_GPIO_WritePin(GPIOx, GPIO_PIN_x, GPIO_PIN_SET);

	delayUS(5000);

	for(i=0; i<3; i++)
	{
		HAL_WWDG_Refresh(&hwwdg, 127);

		if(i==0)
		{
			char MAPXYZ_command[] = {0x00, 0x2A, 0x45, 0x8F, Addr_102A_LSB, Addr_102A_MSB, 0xC3, ComputeCRC(0x00, 0x2A, 0x45, 0x8F, Addr_102A_LSB, Addr_102A_MSB, 0xC3)};

			HAL_GPIO_WritePin(GPIOx, GPIO_PIN_x, GPIO_PIN_RESET);
			delayUS(100);
			HAL_SPI_TransmitReceive(&hspx, MAPXYZ_command, nop_answer, 8, 50);
			delayUS(100);
			HAL_GPIO_WritePin(GPIOx, GPIO_PIN_x, GPIO_PIN_SET);

			delayUS(5000);
		}
		else if(i==1)
		{
			char Kalpha_command[] = {0x00, 0x22, 0xDD, 0xFB, Addr_1022_LSB, Addr_1022_MSB, 0xC3, ComputeCRC(0x00, 0x22, 0xDD, 0xFB, Addr_1022_LSB, Addr_1022_MSB, 0xC3)};

			HAL_GPIO_WritePin(GPIOx, GPIO_PIN_x, GPIO_PIN_RESET);
			delayUS(100);
			HAL_SPI_TransmitReceive(&hspx, Kalpha_command, nop_answer, 8, 50);
			delayUS(100);
			HAL_GPIO_WritePin(GPIOx, GPIO_PIN_x, GPIO_PIN_SET);

			delayUS(5000);
		}
		else if(i==2)
		{
			char Kbeta_command[] = {0x00, 0x24, 0xC9, 0x9F, Addr_1024_LSB, Addr_1024_MSB, 0xC3, ComputeCRC(0x00, 0x24, 0xC9, 0x9F, Addr_1024_LSB, Addr_1024_MSB, 0xC3)};

			HAL_GPIO_WritePin(GPIOx, GPIO_PIN_x, GPIO_PIN_RESET);
			delayUS(100);;
			HAL_SPI_TransmitReceive(&hspx, Kbeta_command, nop_answer, 8, 50);
			delayUS(100);
			HAL_GPIO_WritePin(GPIOx, GPIO_PIN_x, GPIO_PIN_SET);

			delayUS(5000);
		}

		//Define ReadChallenge message
		//Requires: Nothing
		//Results: EE Challenge
		char Challenge_command[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xCF, ComputeCRC(0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xCF)};
		char Challenge_answer[8];

		HAL_GPIO_WritePin(GPIOx, GPIO_PIN_x, GPIO_PIN_RESET);
		delayUS(100);
		HAL_SPI_TransmitReceive(&hspx, Challenge_command, Challenge_answer, 8, 50);
		//send_receive_spi(selected_spi, Challenge_command, Challenge_answer, 8);
		delayUS(100);
		HAL_GPIO_WritePin(GPIOx, GPIO_PIN_x, GPIO_PIN_SET);

		delayUS(5000);

		//Requires: Key Echo, Inverted Key Echo
		// Key Echo to be XOR'd with 0x1234
		//Results: EEReadAnswer
		char keyecho_command[] = {0x00, 0x00, Challenge_answer[2] ^ 0x34, Challenge_answer[3] ^ 0x12, ~(Challenge_answer[2] ^ 0x34), ~(Challenge_answer[3] ^ 0x12), 0xC5, ComputeCRC(0x00, 0x00, Challenge_answer[2] ^ 0x34, Challenge_answer[3] ^ 0x12, ~(Challenge_answer[2] ^ 0x34), ~(Challenge_answer[3] ^ 0x12), 0xC5)};

		HAL_GPIO_WritePin(GPIOx, GPIO_PIN_x, GPIO_PIN_RESET);
		delayUS(100);
		HAL_SPI_TransmitReceive(&hspx, keyecho_command, nop_answer, 8, 50);
		delayUS(100);
		HAL_GPIO_WritePin(GPIOx, GPIO_PIN_x, GPIO_PIN_SET);

		HAL_WWDG_Refresh(&hwwdg, 127);

		delayUS(10000);
		delayUS(10000);
		delayUS(10000);
		delayUS(10000);
		delayUS(10000);

		HAL_GPIO_WritePin(GPIOx, GPIO_PIN_x, GPIO_PIN_RESET);
		delayUS(100);
		HAL_SPI_TransmitReceive(&hspx, nop_command, nop_answer, 8, 50);
		//		send_receive_spi(selected_spi, nop_command, nop_answer, 8);
		delayUS(100);
		HAL_GPIO_WritePin(GPIOx, GPIO_PIN_x, GPIO_PIN_SET);

		delayUS(5000);

		//usart_buffer[0] = 'A';
		//length = 1;

		//length = snprintf(usart_buffer, 128, "Error code (1=success): 0x%02X\r\n", nop_answer[0]);
		//HAL_UART_Transmit(huart_in, usart_buffer, length, 20);
	}

	//Define reboot command
	//Required to reset chip to use new EEPROM parameters
	char reboot_command[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xEF, ComputeCRC(0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xEF)};

	HAL_GPIO_WritePin(GPIOx, GPIO_PIN_x, GPIO_PIN_RESET);
	delayUS(100);
	HAL_SPI_TransmitReceive(&hspx, reboot_command, nop_answer, 8, 50);
	delayUS(100);
	HAL_GPIO_WritePin(GPIOx, GPIO_PIN_x, GPIO_PIN_SET);

	HAL_WWDG_Refresh(&hwwdg, 127);

	delayUS(10000);
	delayUS(10000);
	delayUS(10000);

	__enable_irq();
}


void get_2D(SPI_HandleTypeDef hspx, GPIO_TypeDef* GPIOx, uint16_t GPIO_PIN_x, float *alpha, float *beta)
{
	//__disable_irq();

	HAL_GPIO_WritePin(GPIOx, GPIO_PIN_x, GPIO_PIN_RESET);

	HAL_Delay(1);
	HAL_SPI_TransmitReceive(&hspx, SPI_MASTER_Buffer_Tx, nop_answer, 8, 50);
	HAL_Delay(1);

	HAL_GPIO_WritePin(GPIOx, GPIO_PIN_x, GPIO_PIN_SET);

	HAL_Delay(10);

	HAL_GPIO_WritePin(GPIOx, GPIO_PIN_x, GPIO_PIN_RESET);

	HAL_Delay(1);
	HAL_SPI_TransmitReceive(&hspx, nop_command, SPI_MASTER_Buffer_Rx, 8, 50);
	HAL_Delay(1);

	HAL_GPIO_WritePin(GPIOx, GPIO_PIN_x, GPIO_PIN_SET);

	u16_beta_angle_lsb = ((SPI_MASTER_Buffer_Rx[3] & 0x3F) << 8)+SPI_MASTER_Buffer_Rx[2];
	u16_alpha_angle_lsb = ((SPI_MASTER_Buffer_Rx[1] & 0x3F) << 8)+SPI_MASTER_Buffer_Rx[0];

	f32_beta_angle_degrees = u16_beta_angle_lsb * f32_lsb_to_dec_degrees;
	f32_alpha_angle_degrees = u16_alpha_angle_lsb * f32_lsb_to_dec_degrees;

	*alpha = f32_alpha_angle_degrees;
	*beta = f32_beta_angle_degrees;

	//__enable_irq();
}

void get_2D_hex(SPI_HandleTypeDef hspx, GPIO_TypeDef* GPIOx, uint16_t GPIO_PIN_x, uint16_t *alpha, uint16_t*beta)
{
	__disable_irq();

	HAL_GPIO_WritePin(GPIOx, GPIO_PIN_x, GPIO_PIN_RESET);

	//HAL_Delay(1);
	delayUS(50);
	HAL_SPI_TransmitReceive(&hspx, SPI_MASTER_Buffer_Tx, nop_answer, 8, 5);
	//HAL_Delay(1);
	delayUS(50);

	HAL_GPIO_WritePin(GPIOx, GPIO_PIN_x, GPIO_PIN_SET);

	//HAL_Delay(10);
	delayUS(1000);

	HAL_GPIO_WritePin(GPIOx, GPIO_PIN_x, GPIO_PIN_RESET);

	//HAL_Delay(1);
	delayUS(50);
	HAL_SPI_TransmitReceive(&hspx, nop_command, SPI_MASTER_Buffer_Rx, 8, 5);
	//HAL_Delay(1);
	delayUS(50);

	__enable_irq();

	HAL_GPIO_WritePin(GPIOx, GPIO_PIN_x, GPIO_PIN_SET);

	u16_beta_angle_lsb = ((SPI_MASTER_Buffer_Rx[3] & 0x3F) << 8) + SPI_MASTER_Buffer_Rx[2];
	u16_alpha_angle_lsb = ((SPI_MASTER_Buffer_Rx[1] & 0x3F) << 8) + SPI_MASTER_Buffer_Rx[0];

	u8_error_lsb = SPI_MASTER_Buffer_Rx[1] >> 6;

	if((u8_error_lsb & 0b10) && (ComputeCRCN(SPI_MASTER_Buffer_Rx, 7)==SPI_MASTER_Buffer_Rx[7]))
	{
		*alpha = u16_alpha_angle_lsb;
		*beta = u16_beta_angle_lsb;
	}
	else
	{
		char reboot_command[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xEF, ComputeCRC(0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xEF)};

		__disable_irq();

		HAL_SPI_DeInit (&hspx);

		delayUS(100);

		HAL_SPI_Init(&hspx);

		delayUS(100);

		__enable_irq();

		/*delayUS(1000);

		HAL_GPIO_WritePin(GPIOx, GPIO_PIN_x, GPIO_PIN_RESET);
		delayUS(50);
		HAL_SPI_TransmitReceive(&hspx, reboot_command, nop_answer, 8, 5);
		delayUS(50);
		HAL_GPIO_WritePin(GPIOx, GPIO_PIN_x, GPIO_PIN_SET);

		delayUS(2000);

		//*alpha = u16_alpha_angle_lsb;
		//*beta = u16_beta_angle_lsb;*/

		*alpha = 0;
		*beta = 0;
	}

	//__enable_irq();
}
