/*
 * mq135_cfg.h
 *
 *  Created on: Sep 8, 2025
 *      Author: timji
 */

#ifndef MQ135_MQ135_CFG_H_
#define MQ135_MQ135_CFG_H_

#include "mq135.h"

//Number of sensors
#define MQ135_MAX 	1
#define MQ135_1		0     //First instance

// Adjust as needed
#define MQ135_SAMPLES 16
#define MQ135_THRESHOLD 4000000000
#define MQ135_VOLTAGE_REF 3.3f
#define MQ135_ADC_RESOLUTION 4095.0f
#define MQ135_LOAD_RES       10000.0f  // 10kohms load resistor
#define MQ135_R0             10.0f     // Calibrated in clean air

#define MQ135_ADC_CLEAN 1000000 // ADC value in clean air
#define MQ135_ADC_POLLUTED 3000000  // ADC value in bad air

typedef struct{

	uint8_t mq123_Instance;

	ADC_HandleTypeDef* ADC_Handle;
}mq135_CfgType;

extern const mq135_CfgType mq135_CfgParam[MQ135_MAX];

#endif /* MQ135_MQ135_CFG_H_ */
