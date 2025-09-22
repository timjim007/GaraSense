/*
 * mq135.c
 *
 *  Created on: Sep 7, 2025
 *      Author: timji
 */

#include <math.h>
#include "mq135.h"
#include "../../util/delay.h"

uint32_t mq135_buffer[MQ135_SAMPLES];
volatile uint32_t mq135_avg = 0;
volatile uint8_t  mq135_ready = 0;


void MQ135_Init(uint8_t mq123_Instance)
{

	HAL_ADC_Start_DMA(mq135_CfgParam[mq123_Instance].ADC_Handle, mq135_buffer, MQ135_SAMPLES);
}

void MQ135_StartMeasurement(uint8_t mq123_Instance)
{
//	DELAY_MS(1000);

    // Start a DMA transfer for MQ135_SAMPLES conversions
    HAL_ADC_Start_DMA(mq135_CfgParam[mq123_Instance].ADC_Handle, mq135_buffer, MQ135_SAMPLES);
}


void MQ135_AverageRaw()
{
	uint32_t sum = 0;
	for (int i = 0; i < MQ135_SAMPLES; i++)
	{
		sum += mq135_buffer[i];
	}
	mq135_avg = sum / MQ135_SAMPLES;
	mq135_ready = 1;

}


float MQ135_GetPPM(void)
{

	if (!mq135_ready) return 0;


	 float index = ((float)mq135_avg * MQ135_VOLTAGE_REF) / (float)MQ135_ADC_RESOLUTION;

	    return index;
}

uint8_t MQ135_IsSmokeDetected(void)
{
    return (mq135_avg > MQ135_THRESHOLD) ? 1 : 0;
}

// Map PPM to qualitative air quality
AirQuality_t MQ135_GetAirQuality(void)
{
    uint32_t idx = MQ135_GetPPM();

    if (idx <= 200)       return AIR_GOOD;
    else if (idx <= 500)  return AIR_MODERATE;
    else if (idx <= 800)  return AIR_POOR;
    else                   return AIR_UNHEALTHY;
}



