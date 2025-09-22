/*
 * ao_display.h
 *
 *  Created on: Sep 9, 2025
 *      Author: timji
 */

#ifndef INC_AO_DISPLAY_H_
#define INC_AO_DISPLAY_H_

#include "ao.h"
#include "ao_broker.h"


#include "../../ECUAL/ssd1306/ssd1306.h"
#include "../../ECUAL/ssd1306/fonts.h"

extern Broker broker;


typedef enum{
	TURNED_OFF,
	DISTANCE,
	TEMP,
	AIR,
	UNMOUNT_PAGE,
	DONE,
	FAILED
}DisplayState;


/* Display AO */
typedef struct{
	Active super;

	/* private data */
	DisplayState page;
	uint8_t temp;
	uint8_t hum;
	float ppm;
	float distance;
	uint8_t alert;
	uint8_t unmountDone;
	uint8_t unmountChoice;
}Display;

void Display_Init(Display * const me);

#endif /* INC_AO_DISPLAY_H_ */
