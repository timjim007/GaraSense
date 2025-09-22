/*
 * ws2812.c
 *
 *  Created on: Sep 9, 2025
 *      Author: timji
 */

#include "ws2812.h"



// Array for 48 LEDs (R,G,B order)
static uint8_t colors[LED_COUNT * 3];

//Buffer
volatile uint16_t pwmData[LED_COUNT * BITS_PER_LED + RESET_SLOTS];


void ws2812B_SetColor(uint8_t r, uint8_t g, uint8_t b)
{
	//	Fill all LEDs
	for (int led = 0; led < LED_COUNT; led++) {
	      colors[led*3 + 0] = r; // R
	      colors[led*3 + 1] = g;   // G
	      colors[led*3 + 2] = b;   // B
	}
}

void ws2812B_Show(TIM_HandleTypeDef *htim)
{

	    // Calculate buffer size
//	    uint16_t buffer_size = LED_COUNT * BITS_PER_LED + RESET_SLOTS;
//	    uint16_t pwmData[buffer_size];

	    // Fill PWM buffer
	    for (uint16_t led = 0; led < LED_COUNT; led++)
	    {
	        uint32_t color = ((uint32_t)colors[led*3 + 1] << 16) |  // G
	                         ((uint32_t)colors[led*3 + 0] << 8)  |  // R
	                         ((uint32_t)colors[led*3 + 2]);         // B

	        for (uint8_t bit = 0; bit < 24; bit++)
	        {
	            pwmData[led * BITS_PER_LED + bit] = (color & (1 << (23 - bit))) ? PWM1 : PWM0;
	        }
	    }

	    // Append reset slots
	    for (uint16_t i = 0; i < RESET_SLOTS; i++)
	        pwmData[LED_COUNT * BITS_PER_LED + i] = 0;

	    // Start PWM + DMA
	    HAL_TIM_PWM_Start_DMA(htim, TIM_CHANNEL_1, (uint32_t*)pwmData, BUFFER_SIZE);
//	    osDelay(1);



}

