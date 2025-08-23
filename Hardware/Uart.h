#ifndef __UART_H_
#define __UART_H_

#include "stm32f1xx_hal.h"
#include "usart.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#define PRINT_TO_UART 1								//开启print重定向

#define PRINT_USARTX   USART2         //通过usart1实现
#define PRINT_USARTX_HANDLE   &huart2


void USART1_Init(void);

#endif
