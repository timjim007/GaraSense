/*
 * uart.h
 *
 *  Created on: Sep 11, 2025
 *      Author: timji
 */

#ifndef UART_H_
#define UART_H_

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>

//Diagnotocs
#define TX_BUFFER_SIZE 250

extern UART_HandleTypeDef huart3;

//For diagnotics
extern char TxBuffer[TX_BUFFER_SIZE];
extern volatile uint8_t tx_busy;


void UART_Print(const char* str);

#endif /* UART_H_ */
