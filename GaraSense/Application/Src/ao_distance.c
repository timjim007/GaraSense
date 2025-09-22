/*
 * ao_distance.c
 *
 *  Created on: Sep 9, 2025
 *      Author: timji
 */


#include "../Inc/ao_distance.h"

static void lightHelper(LedState state){
	switch(state){
		case OFF:
			ws2812B_SetColor(0,0,0);
			ws2812B_Show(&htim1);
			break;
		case RED:
			ws2812B_SetColor(255,0,0);
			ws2812B_Show(&htim1);
			break;
		case YELLOW:
			ws2812B_SetColor(255,255,0);
			ws2812B_Show(&htim1);
			break;
		case GREEN:
			ws2812B_SetColor(0,255,0);
			ws2812B_Show(&htim1);
			break;
		default:
			break;
	}
}

static void DistanceSensorDispatch(DistanceSensor * const me, Event const * const e){
	switch(e->eventsig){
		case INIT_SIG:
			HCSR04_Init(HCSR04_SENSOR1,&htim2);
			me->Light = OFF;
			lightHelper(me->Light);
			break;

		case UPDATE_LED:
			me->Distance = HCSR04_Read(HCSR04_SENSOR1);

			//Update LED state
			if(me->Distance < STOP_THRESHOLD){
				me->Light = RED;
			}else if(me->Distance < WARNING_THRESHOLD){
				me->Light = YELLOW;
			}else{
				me->Light = GREEN;
			}

			lightHelper(me->Light);

			break;
		case DIST_REQ:
			Event dispEvt = {
					.eventsig = DIST_UPDATED,
					.dest = DISPLAY,
					.payload.distance = me->Distance
			};

			Active_post(&broker.super, &dispEvt);
			//TODO: post to broker with destination display

			break;
		case SENSOR_TIMER:

			if(me->Distance < STOP_THRESHOLD){
				 Event logEvt = {
					.eventsig = LOG_DATA,
					.dest = DATA_LOGGER,
					.payload.distance = me->Distance
				};

			//TODO: Post to sd card
				Active_post(&broker.super, &logEvt);
			}

			break;
		default:
			break;
	}
}

void DistanceSensor_Init(DistanceSensor * const me){
	Active_ctor(&me->super, (DispatchHandler)&DistanceSensorDispatch);
}
