#ifndef __MLX90363_H
#define __MLX90363_H

#include "stm32f0xx_hal.h"
#include "uart_queue.h"

#define SPI_SS_LOW(N)	GPIO_ResetBits(SPI_MASTER_GPIO, SPI_MASTER_PIN_SS_2)
#define SPI_SS_HIGH(N)	GPIO_SetBits(SPI_MASTER_GPIO, SPI_MASTER_PIN_SS_2)

//Function ComputeCRC
//Requires: 7 bytes to be transmitted to slave device
//Returns: 1 byte corresponding to CRC code
uint8_t ComputeCRC(uint8_t Byte0, uint8_t Byte1, uint8_t Byte2, uint8_t Byte3, uint8_t Byte4, uint8_t Byte5, uint8_t Byte6);

uint8_t ComputeCRCN(uint8_t* bytes, uint8_t length);
void UpdateCRCLine(Line* message_line);

void init_1D();
void get_1D(SPI_HandleTypeDef hspx, GPIO_TypeDef* GPIOx, uint16_t GPIO_PIN_x, float *alpha);
void init_2D(SPI_HandleTypeDef hspx, GPIO_TypeDef* GPIOx, uint16_t GPIO_PIN_x);
void get_2D(SPI_HandleTypeDef hspx, GPIO_TypeDef* GPIOx, uint16_t GPIO_PIN_x, float *alpha, float *beta);

void get_1D_hex(SPI_HandleTypeDef hspx, GPIO_TypeDef* GPIOx, uint16_t GPIO_PIN_x, uint16_t *alpha);
void get_2D_hex(SPI_HandleTypeDef hspx, GPIO_TypeDef* GPIOx, uint16_t GPIO_PIN_x, uint16_t *alpha, uint16_t*beta);

#define TxBufferSize 9
#define RxBufferSize 9

#endif /* __MLX90363_H */
