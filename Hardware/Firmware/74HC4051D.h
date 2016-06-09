#ifndef __74HC4051D_H
#define __74HC4051D_H

#include "stm32f0xx_hal.h"

void MPX_UART_Open(uint8_t line);
void MPX_UART_Transmit(UART_HandleTypeDef* usart_x, uint8_t line, uint8_t* usart_buffer, int length, int timeout);
void MPX_UART_Init();
void MPX_UART_ONOFF(uint8_t on_off);

#endif /* __74HC4051D_H */
