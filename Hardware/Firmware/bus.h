/**
  ******************************************************************************
  * @file    ueisart.h
  * @author  Oliver Glauser, IGL, ETH Zurich
  * @version V0.2
  * @date    14-January-2015
  * @brief   Some definition for a half-duplex multi slave local tree usart deviation
  ******************************************************************************

  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LUEISART_H
#define __LUEISART_H

#include "stm32f0xx_hal.h"
#include "uart_queue.h"
#include "stdio.h"
#include "string.h"
#include "74HC4051D.h"
#include "utility.h"

#define DOWN_MESSAGE_LENGTH 10
#define UP_MESSAGE_LENGTH 18

typedef enum {
	poll_receive='R',
	poll_send='C',
	poll_direct='P',
	answer_data='D',
	answer_ack='A',
	init='I',
	splitter = 'S',
	empty = 'E'
} LUEISART_POLL_KIND;

typedef enum {
	yes='Y',
	no='N'
} LUEISART_POLL_ANSWER;

typedef enum {
	idle,
	polling,
	reading,
	sending,
	reading_direct,
	dma
} LUEISART_STATE;

uint8_t length;

uint8_t up_buffer[32];
uint8_t down_buffer[32];

uint8_t direction_set;

uint8_t LUEISART_Receive_Child(uint8_t* buffer);
uint32_t getIDfromMessage(char * message);
void LUSARTC_IRQHandler(uint8_t *received_byte);
void LUSARTP_IRQHandler(uint8_t *received_byte, HAL_UART_StateTypeDef *huart_X, uint8_t *buffer_X);

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle);
void UARTDMAErrorHandler();

HAL_StatusTypeDef HAL_HalfDuplex_EnableTransmitterReceiver(UART_HandleTypeDef *huart);

#endif /* __LUEISART_H */
