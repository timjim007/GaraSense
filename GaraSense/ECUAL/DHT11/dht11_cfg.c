/*
 * dht11_cfg.c
 *
 *  Created on: Sep 8, 2025
 *      Author: timji
 */

#include "dht11_cfg.h"


const DHT11_CfgType dht11_CfgParam[DHT11_MAX] = {
		{
				DHT11_1,
				GPIOE,
				GPIO_PIN_12
		}
};
