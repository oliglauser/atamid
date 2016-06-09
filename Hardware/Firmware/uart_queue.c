#include "uart_queue.h"

UartQueue uart_input_queue;
UartQueue uart_output_queue;
UartQueue uart_command_queue;

void uart_queue_initialize(UartQueue * q) {
    int i;
    q->read_index = 0;
    q->write_index = 0;
    for (i=0;i<UART_QUEUE_LENGTH;i++) {
        q->line[i].length = 0;
        q->line[i].text[0] = 0;
        q->line[i].text[UART_QUEUE_LINE_LENGTH-1] = 0;
    }
}

int uart_queue_is_empty(UartQueue *q) {
    return (q->read_index == q->write_index);
}

unsigned int uart_queue_length(UartQueue * q) {
    if (q->write_index < q->read_index) {
        return q->write_index + UART_QUEUE_LENGTH - q->read_index;
    } else {
        return q->write_index - q->read_index;
    }
}

void uart_queue_pop_line(UartQueue *q, Line *l) {
	__disable_irq();
    if (uart_queue_is_empty(q)) {
        l->length=0;
        l->text[0]=0;
    } else {
        memcpy(l,&(q->line[q->read_index]),sizeof(Line));
        q->line[q->read_index].length = 0;
        q->read_index = (q->read_index + 1) % UART_QUEUE_LENGTH;
    }
	__enable_irq();
}

void uart_queue_replace_push_line(UartQueue *q, Line * line, uint32_t ID)
{
	int i = q->read_index;

	while(i != q->write_index)
	{
		if(getIDfromMessage(q->line[i].text) == ID && q->line[i].text[0]==line->text[0])
		{
			uart_queue_replace_line(q, line, i);
			break;
		}

		i = (i+1)%UART_QUEUE_LENGTH;
	}

	if(i == q->write_index)
	{
		uart_queue_push_line(q, line);
	}
}

void uart_queue_replace_line(UartQueue *q, Line * line, int i)
{
	__disable_irq();
    memcpy(&(q->line[i]),line,sizeof(Line));
	__enable_irq();
}

void uart_queue_push_line(UartQueue *q, Line * line) {
	__disable_irq();
    memcpy(&(q->line[q->write_index]),line,sizeof(Line));
    q->write_index = (q->write_index + 1) % UART_QUEUE_LENGTH;
    if (q->read_index == q->write_index) {
        q->read_index = (q->read_index + 1) % UART_QUEUE_LENGTH;
    }
	__enable_irq();
}

void uart_queue_push_charp(UartQueue *q, char * line) {
	__disable_irq();
    q->line[q->write_index].length=strlen(line);
    strncpy(q->line[q->write_index].text,line,(UART_QUEUE_LINE_LENGTH-1));
    q->write_index = (q->write_index + 1) % UART_QUEUE_LENGTH;
    if (q->read_index == q->write_index) {
        q->read_index = (q->read_index + 1) % UART_QUEUE_LENGTH;
    }
	__enable_irq();
}
