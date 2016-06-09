#ifndef UART_QUEUE_H
#define UART_QUEUE_H

#include "stm32f0xx_hal.h"

#define UART_QUEUE_LINE_LENGTH 18
#define UART_QUEUE_LENGTH 32

typedef struct {
    int length;
    char text[UART_QUEUE_LINE_LENGTH];
} Line;

typedef struct {
    Line line[UART_QUEUE_LENGTH];
    int read_index;
    int write_index;
} UartQueue;

extern UartQueue uart_input_queue;
extern UartQueue uart_output_queue;
extern UartQueue uart_command_queue;

void uart_queue_initialize(UartQueue * q);
unsigned int uart_queue_length(UartQueue * q);
int uart_queue_is_empty(UartQueue *q);
void uart_queue_pop_line(UartQueue *q, Line *l) ;
void uart_queue_push_line(UartQueue *q, Line * line) ;
void uart_queue_push_charp(UartQueue *q, char * line) ;
void uart_queue_replace_line(UartQueue *q, Line * line, int i);
void uart_queue_replace_push_line(UartQueue *q, Line * line, uint32_t ID);

#endif // UART_QUEUE_H

