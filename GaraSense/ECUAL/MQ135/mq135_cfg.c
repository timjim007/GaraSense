/*
 * mq135_cfg.c
 *
 *  Created on: Sep 8, 2025
 *      Author: timji
 */

#include "mq135.h"

extern ADC_HandleTypeDef hadc1;

const mq135_CfgType mq135_CfgParam[MQ135_MAX] = {
		{
				MQ135_1,
				&hadc1
		}
};

