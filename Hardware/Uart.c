#include <stdio.h>
#include "Uart.h"


/*print重定向*/
#ifdef PRINT_TO_UART

// 禁用半主机模式
#pragma import(__use_no_semihosting)

struct __FILE { int handle; };
FILE __stdout;
FILE __stdin;

int fputc(int ch, FILE *f) {
	while(__HAL_UART_GET_FLAG( PRINT_USARTX_HANDLE,UART_FLAG_TXE) == RESET) {}
	HAL_UART_Transmit( PRINT_USARTX_HANDLE ,(uint8_t *)&ch  , 1 ,HAL_MAX_DELAY );

  return(ch);
}

int fgetc(FILE *f) {
  char ch;
  while(__HAL_UART_GET_FLAG( PRINT_USARTX_HANDLE,UART_FLAG_RXNE) == RESET){}
  HAL_UART_Receive( PRINT_USARTX_HANDLE ,(uint8_t *)&ch  , 1 ,HAL_MAX_DELAY );
  return((int)ch);
}

int ferror(FILE *f) {
  return EOF;
}

void _ttywrch(int ch) {
	
	while(__HAL_UART_GET_FLAG( PRINT_USARTX_HANDLE,UART_FLAG_TXE) == RESET) {}
	HAL_UART_Transmit( PRINT_USARTX_HANDLE ,(uint8_t *)&ch  , 1 ,HAL_MAX_DELAY );
		
}

void _sys_exit(int return_code) {
  while (1); /* endless loop */
}

#endif


void USART1_Init(void)
{
	//UART1: AP9, AP10
}

