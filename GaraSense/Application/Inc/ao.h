/*
 * ao.h
 *
 *  Created on: Sep 2, 2025
 *      Author: timji
 */

#ifndef INC_AO_H_
#define INC_AO_H_

#include "cmsis_os.h"
#include "../../util/uart.h"



/*----------------------------------------*/
/****Event Facilities****/
//typedef enum {
//    AIR_GOOD,
//    AIR_MODERATE,
//    AIR_POOR,
//    AIR_UNHEALTHY,
//    AIR_SENSOR_ERROR
//} AirQuality;

typedef enum {
	INIT_SIG, /* dispatched to AO before entering event-loop */
	SENSOR_TIMER,
	LEFT_BUTTON_EVT,
	RIGHT_BUTTON_EVT,
	CONFIRM_BUTTON_EVT,
	UPDATE_LED,
	DIST_REQ,
	TEMP_REQ,
	PPM_REQ,
	DIST_UPDATED,
	TEMP_UPDATED,
	PPM_UPDATED,
	LOG_DATA,
	UNMOUNT_REQ,
	UNMOUNT_UPDATED,
	DELETE_LOG,
	ALERT_EVT,
	SD_UNAVAILABLE,
	SD_AVAILABLE

}EventSignal;

typedef enum{
	BROKER,
	DIST_SENSOR,
	ENV_MONITOR,
	DATA_LOGGER,
	DISPLAY
}EventDestination;

typedef struct {
	float distance;
	uint8_t temp;
	uint8_t hum;
	float ppm;
	uint8_t isUnmount;
//	AirQuality airquality;
}EventPayload;

/* Event base class */
typedef struct{
	EventSignal eventsig;
	EventDestination dest;
	EventPayload payload;
}Event;

/*----------------------------------------*/
/****Active object Facilities****/

typedef struct Active Active;

typedef void (*DispatchHandler)(Active * const me, Event const * const e);

/* Active Object base class */
struct Active{
	osThreadId_t thread;  /* Private thread */
	osMessageQueueId_t queue;	/* Private message queue */

	DispatchHandler dispatch; /* pointer to the dispatch function */

	/* active object data added in subclasses of Active */
};

void Active_ctor(Active * const me, DispatchHandler dispatch);
void Active_start(Active * const me, const osThreadAttr_t * threadAttr, osMessageQueueId_t queueHandle);
void Active_post(Active * const me, Event const * const e);
void AO_ReportStack(Active * const me, const char *name);
void AO_ReportQueue(Active * const me, const char *name);



#endif /* INC_AO_H_ */
