/*
 * ao_env.c
 *
 *  Created on: Sep 9, 2025
 *      Author: timji
 */


#include "../Inc/ao_env.h"

static void EnvMonitorDispatch(EnvMonitor * const me, Event const * const e){
	uint8_t rv;
	switch(e->eventsig){
		case INIT_SIG:
			MQ135_Init(MQ135_1);
			break;

		case TEMP_REQ:
			rv = dht11_Read(DHT11_1,&me->temp,&me->hum);

			if(rv != DHT11_OK){
				break;
			}

			Event dispEvt = {
					.eventsig = TEMP_UPDATED,
					.dest = DISPLAY,
					.payload.temp = me->temp,
					.payload.hum = me->hum
			};

			Active_post(&broker.super, &dispEvt);
			//TODO: Post event to broker

			break;
		case PPM_REQ:
			MQ135_StartMeasurement(0);
			me->ppm = MQ135_GetPPM();

			if(MQ135_IsSmokeDetected()){

				Event alertEvt = {
					.eventsig = ALERT_EVT,
					.dest = DISPLAY,
					.payload.ppm = me->ppm
				};

				Active_post(&broker.super, &alertEvt);
				//TODO: Post an alert event with values to broker with dst display
			}else{
				Event ppmEvt = {
					.eventsig = PPM_UPDATED,
					.dest = DISPLAY,
					.payload.ppm = me->ppm
				};

				Active_post(&broker.super, &ppmEvt);
				//TODO: Post the ppm value to broker with dst display
			}

			break;
		case SENSOR_TIMER:
			rv = dht11_Read(DHT11_1,&me->temp,&me->hum);

			if(rv){
				me->temp = -99;
				me->hum = -1;
			}

			MQ135_StartMeasurement(0);
			me->ppm = MQ135_GetPPM();

			if(MQ135_IsSmokeDetected() || me->temp > TEMP_MAX || me->temp < TEMP_MIN){
				 Event alertEvt = {
					.eventsig = ALERT_EVT,
					.dest = DATA_LOGGER,
					.payload.temp = me->temp,
					.payload.hum = me->hum,
					.payload.ppm = me->ppm
				};
				 Active_post(&broker.super, &alertEvt);
				//TODO: Post an alert event with values to broker with dest sdcard
			}

			// Always log normal values
			Event logEvt = {
				.eventsig = LOG_DATA,
				.dest = DATA_LOGGER,
				.payload.temp = me->temp,
				.payload.hum = me->hum,
				.payload.ppm = me->ppm
			};
			Active_post(&broker.super, &logEvt);
			//TODO: Post an event with values to broker with dest sdcard

			break;
		default:
			break;
	}
}

void EnvMonitor_Init(EnvMonitor * const me){
	Active_ctor(&me->super, (DispatchHandler)&EnvMonitorDispatch);
}
