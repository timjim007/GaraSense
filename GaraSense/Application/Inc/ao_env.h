/*
 * ao_env.h
 *
 *  Created on: Sep 9, 2025
 *      Author: timji
 */

#ifndef INC_AO_ENV_H_
#define INC_AO_ENV_H_

#include "ao.h"
#include "ao_broker.h"
#include "../../ECUAL/MQ135/mq135.h"
#include "../../ECUAL/DHT11/dht11.h"

#define TEMP_MAX 100
#define TEMP_MIN 0

extern Broker broker;


/* EnvMonitor AO */
typedef struct{
	Active super;  /*Inherit Active base object*/

	/* private data */
	uint8_t temp;
	uint8_t hum;
	float ppm;

}EnvMonitor;

void EnvMonitor_Init(EnvMonitor * const me);

#endif /* INC_AO_ENV_H_ */
