#include "uart_buffer.h"
#include "string.h"

void uart_buffer_init(uart_buffer *Uart_Data)
{
	memset(Uart_Data, 0, sizeof(uart_buffer));
}

void uart_Enqueue(uart_buffer *Uart_Data, char ch)
{
	if(uart_buffer_full(Uart_Data)==0)
	{
		if(ch == '\r')
			Uart_Data->flag = 1;

		if(Uart_Data->head < UART_BUFFER_MAX)
		{
			Uart_Data->buffer[Uart_Data->head++] = ch;
		}
		else
		{
			Uart_Data->head = 0;
			Uart_Data->buffer[Uart_Data->head] = ch;
		}
	}
}
char uart_Dequeue(uart_buffer *Uart_Data)
{
	if(uart_buffer_empty(Uart_Data) == 0)
	{
		if(Uart_Data->tail < UART_BUFFER_MAX)
		{
			return Uart_Data->buffer[Uart_Data->tail++];
		}
		else
		{
			Uart_Data->tail = 0;
			return Uart_Data->buffer[Uart_Data->tail];
		}
	}
	return -1;
}

char get_uart_flag(uart_buffer *Uart_Data)
{
	return Uart_Data->flag;
}

void uart_flag_reset(uart_buffer *Uart_Data)
{
	Uart_Data->flag = 0;
}

int uart_Used_Buffer(uart_buffer *Uart_Data)
{
	if(Uart_Data->head > Uart_Data->tail)
		return Uart_Data->head - Uart_Data->tail;
	else if(Uart_Data->head < Uart_Data->tail)
		return UART_BUFFER_MAX - Uart_Data->tail + Uart_Data->head;
	return 0;
}
char uart_buffer_full(uart_buffer *Uart_Data)
{
	if((Uart_Data->tail == 0)&&(Uart_Data->head == UART_BUFFER_MAX - 1))
	{	//tail first and Head end so buffer full
		return 1;
	}
	else if(Uart_Data->tail - Uart_Data->head == 1)
	{
		return 2;
	}
	return 0;
}
char uart_buffer_empty(uart_buffer *Uart_Data)
{
	if(Uart_Data->tail == Uart_Data->head)
		return 1;

	return 0;
}

