/*
 * mq135.h
 *
 *  Created on: Sep 7, 2025
 *      Author: timji
 */

#ifndef MQ135_MQ135_H_
#define MQ135_MQ135_H_

#include "stm32f4xx_hal.h"
#include "mq135_cfg.h"


typedef enum {
    AIR_GOOD,
    AIR_MODERATE,
    AIR_POOR,
    AIR_UNHEALTHY,
    AIR_SENSOR_ERROR
} AirQuality_t;

extern ADC_HandleTypeDef hadc1;   // Defined in main.c or CubeMX init

// Public functions
void MQ135_Init(uint8_t mq123_Instance);
void MQ135_StartMeasurement(uint8_t mq123_Instance);
void MQ135_AverageRaw();
float MQ135_GetPPM(void);
uint8_t MQ135_IsSmokeDetected(void);
AirQuality_t MQ135_GetAirQuality(void);


#endif /* MQ135_MQ135_H_ */
