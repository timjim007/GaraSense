/*
 * ws2812.h
 *
 *  Created on: Sep 9, 2025
 *      Author: timji
 */

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#ifndef WS2812_WS2812_H_
#define WS2812_WS2812_H_

// Constants
#define LED_COUNT     	48
#define BITS_PER_LED  	24
#define RESET_SLOTS		50		// ≥50us low
#define PWM0			35		// duty cycle for 0
#define PWM1			70		// duty cycle for 1

#define BUFFER_SIZE		(LED_COUNT * BITS_PER_LED + RESET_SLOTS)


void ws2812B_SetColor(uint8_t r, uint8_t g, uint8_t b);
void ws2812B_Show(TIM_HandleTypeDef *htim);

#endif /* WS2812_WS2812_H_ */
