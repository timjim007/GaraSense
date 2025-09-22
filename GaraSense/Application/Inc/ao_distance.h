/*
 * ao_distance.h
 *
 *  Created on: Sep 9, 2025
 *      Author: timji
 */

#ifndef INC_AO_DISTANCE_H_
#define INC_AO_DISTANCE_H_

#include "ao.h"
#include "ao_broker.h"
#include "../../ECUAL/HCSR04/HCSR04.h"
#include "../../ECUAL/WS2812/WS2812.h"


#define STOP_THRESHOLD		2
#define WARNING_THRESHOLD	10


extern Broker broker;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

typedef enum{
		OFF,
		RED,
		YELLOW,
		GREEN
}LedState;

/* Distance AO */
typedef struct{
	Active super;  /*Inherit Active base object*/

	/* private data */
	float Distance;
	LedState Light;
}DistanceSensor;

void DistanceSensor_Init(DistanceSensor * const me);

#endif /* INC_AO_DISTANCE_H_ */
