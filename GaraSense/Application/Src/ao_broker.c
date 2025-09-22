/*
 * ao_broker.c
 *
 *  Created on: Sep 9, 2025
 *      Author: timji
 */


#include "../Inc/ao_broker.h"

#include "../Inc/ao_display.h"
#include "../Inc/ao_distance.h"
#include "../Inc/ao_env.h"
#include "../Inc/ao_logger.h"



extern Display display;
extern DistanceSensor distance;
extern EnvMonitor envmonitor;
extern DataLogger logger;

static void BrokerDispatch(Broker * const me, Event const * const e){

	switch(e->dest){
		case DIST_SENSOR:
			Active_post(&distance.super, e);

			break;
		case  ENV_MONITOR:
			Active_post(&envmonitor.super, e);

			break;
		case DATA_LOGGER:
			if(me->isMount){
				Active_post(&logger.super, e);
			}

			break;
		case DISPLAY:
			Active_post(&display.super, e);

		case BROKER:
			if(e->eventsig == SD_AVAILABLE){
				me->isMount = 1;
			}else if (e->eventsig == SD_UNAVAILABLE){
				me->isMount = 0;
			}

			break;
		default:
			break;
	}
}

void Broker_Init(Broker * const me){
	Active_ctor(&me->super, (DispatchHandler)&BrokerDispatch);
}
