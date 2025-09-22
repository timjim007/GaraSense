/*
 * dht11_cfg.h
 *
 *  Created on: Sep 8, 2025
 *      Author: timji
 */

#ifndef DHT11_DHT11_CFG_H_
#define DHT11_DHT11_CFG_H_

#include "dht11.h"

//Number of sensors
#define DHT11_MAX 	1
#define DHT11_1		0     //First instance

typedef struct{

	uint8_t DHT11_Instance;

	GPIO_TypeDef* GPIO_PORT;

	uint16_t GPIO_PIN;

}DHT11_CfgType;

extern const DHT11_CfgType dht11_CfgParam[DHT11_MAX];

#endif /* DHT11_DHT11_CFG_H_ */
