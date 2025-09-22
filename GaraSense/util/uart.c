/*
 * uart.c
 *
 *  Created on: Sep 11, 2025
 *      Author: timji
 */

#include "uart.h"

void UART_Print(const char* str){
	if(!tx_busy){
		tx_busy = 1;
		HAL_UART_Transmit_DMA(&huart3, (uint8_t*)str, strlen(str));
	}
	osDelay(1000);
}

