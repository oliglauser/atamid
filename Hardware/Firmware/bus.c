/**
  ******************************************************************************
  * @file    ueisart.c
  * @author  Oliver Glauser, IGL, ETH Zurich
  * @version V0.2
  * @date    09-October-2015
  * @brief   Some definition for a half-duplex multi slave local tree usart deviation
  ******************************************************************************

  ******************************************************************************
  */

#include "bus.h"
#include "global.h"

// is set whenever a command message shoul dbe restet

extern TIM_HandleTypeDef htim16;


uint32_t getIDfromMessage(char * message)
{
	return ((message[1] & 0xFF) << 24) + ((message[2] & 0xFF) << 16) +  ((message[3] & 0xFF) << 8) + message[4];
}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle.
  * @note   This example shows a simple way to report end of DMA Rx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	if(!DIRECTION_SET)
	{
		if(up_buffer[0] == poll_receive || down_buffer[0] == poll_receive)
		{
			if(UartHandle == huart_UP)
			{
				//huart_UP = huart_DOWN;
				//huart_DOWN = UartHandle;

				DIRECTION_SET = 1;
			}
			else if(UartHandle == huart_DOWN)
			{
				huart_DOWN = huart_UP;
				huart_UP = UartHandle;
				DIRECTION_SET = 1;
			}

			if(DIRECTION_SET)
			{
				//huart_UP = huart_DOWN;
				//huart_DOWN = UartHandle;

				// abort DMA?
				HAL_UART_DMAStop(huart_DOWN);
				HAL_UART_DMAResume(huart_DOWN);

				HAL_UART_DMAStop(huart_UP);
				HAL_UART_DMAResume(huart_UP);

				UP_STATE=IDLE;
				DOWN_STATE=IDLE;
			}

			//uint8_t tno[] = {no,0};

			//HAL_HalfDuplex_EnableTransmitter(huart_UP);
			//HAL_UART_Transmit(huart_UP, tno, 2, 5);
		}
		else
		{
			HAL_HalfDuplex_EnableTransmitterReceiver(huart_DOWN);
			HAL_UART_Receive_DMA(huart_DOWN, down_buffer, DOWN_MESSAGE_LENGTH + 1);
		}

		HAL_HalfDuplex_EnableTransmitterReceiver(huart_UP);
		HAL_UART_Receive_DMA(huart_UP, up_buffer, DOWN_MESSAGE_LENGTH + 1);

		return;
	}

	if(UartHandle==huart_UP)
	{
		interrupt_up_timeout_time = __HAL_TIM_GetCounter(&htim16);

		receivedUPtime = __HAL_TIM_GetCounter(&htim16);

		receivedUP = 1;
	}


	// what should happen if we receive something on UART_U (towards universal joint)
	if(UartHandle==huart_DOWN)
	{
		interrupt_down_timeout_time = __HAL_TIM_GetCounter(&htim16);

		receivedDOWNtime = __HAL_TIM_GetCounter(&htim16);
		receivedDOWN = 1;
	}
}

/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle.
  * @note   This example shows a simple way to report end of DMA Tx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{

	// what should happen if we finish transmitting something on UART_T (towards twist joint)
	if(UartHandle==huart_DOWN)
	{
		transmittedDOWNtime = __HAL_TIM_GetCounter(&htim16);
		transmittedDOWN = 1;
	}

	// what should happen if we finish transmitting something on UART_T (towards universal joint)
	if(UartHandle==huart_UP)
	{
		transmittedUPtime = __HAL_TIM_GetCounter(&htim16);
		transmittedUP = 1;
	}
}

/**
  * @brief  Enables the UART transmitter and enable the UART receiver.
  * @param  huart: UART handl
  * @retval HAL status
  * @retval None
  */
HAL_StatusTypeDef HAL_HalfDuplex_EnableTransmitterReceiver(UART_HandleTypeDef *huart)
{
  /* Process Locked */
  __HAL_LOCK(huart);
  huart->State = HAL_UART_STATE_BUSY;

  /* SET TE and RE bits */
  SET_BIT(huart->Instance->CR1, (USART_CR1_TE | USART_CR1_RE));

  huart->State = HAL_UART_STATE_READY;
  /* Process Unlocked */
  __HAL_UNLOCK(huart);

  return HAL_OK;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_DMAStop(huart);
	HAL_UART_DMAResume(huart);

	if(huart == huart_UP)
	{
		HAL_HalfDuplex_EnableTransmitterReceiver(huart);
		HAL_UART_Receive_DMA(huart, up_buffer, DOWN_MESSAGE_LENGTH + 1);
	}
}

void UARTDMAErrorHandler()
{

}
