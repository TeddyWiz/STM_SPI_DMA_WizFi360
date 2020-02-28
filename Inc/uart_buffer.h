#ifndef __UART_BUFFER_H
#define __UART_BUFFER_H

#ifdef __cplusplus
extern "C" {
#endif

#define UART_BUFFER_MAX 		512
typedef struct uart_buffer_t
{
	char buffer[512];
	char flag;
	int head;
	int tail;
}uart_buffer;

void uart_buffer_init(uart_buffer *Uart_Data);
void uart_Enqueue(uart_buffer *Uart_Data, char ch);
char uart_Dequeue(uart_buffer *Uart_Data);
void uart_flag_reset(uart_buffer *Uart_Data);
char get_uart_flag(uart_buffer *Uart_Data);
int uart_Used_Buffer(uart_buffer *Uart_Data);
char uart_buffer_full(uart_buffer *Uart_Data);
char uart_buffer_empty(uart_buffer *Uart_Data);


#endif
