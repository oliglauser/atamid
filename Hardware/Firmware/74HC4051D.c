#include "74HC4051D.h"


void MPX_UART_Transmit(UART_HandleTypeDef* usart_x, uint8_t line, uint8_t* usart_buffer, int length, int timeout)
{
	MPX_UART_Open(line);
	HAL_Delay(5);
	HAL_UART_Transmit(usart_x, usart_buffer, length, timeout);
}

void MPX_UART_Open(uint8_t line)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, line & 0b001);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, line & 0b010);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, line & 0b100);
}

void MPX_UART_Init()
{
	// init multiplexer pins
	GPIO_InitTypeDef GPIO_InitStruct;

	/*Configure GPIO pin : PA4 */
	GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void MPX_UART_ONOFF(uint8_t on_off)
{
	if(on_off!=0)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
	}
}



