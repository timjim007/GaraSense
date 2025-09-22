/*
 * ao_broker.h
 *
 *  Created on: Sep 9, 2025
 *      Author: timji
 */

#ifndef INC_AO_BROKER_H_
#define INC_AO_BROKER_H_

#include "ao.h"


/* Broker AO */
typedef struct{
	Active super;

	/*private data */
	uint8_t isMount;

}Broker;

void Broker_Init(Broker * const me);

#endif /* INC_AO_BROKER_H_ */
