/*
 * dht11.c
 *
 *
 */

#include "dht11.h"


#include "../../util/delay.h"



void dht11_SetPinOutput(uint8_t DHT11_Instance){
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = dht11_CfgParam[DHT11_Instance].GPIO_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(dht11_CfgParam[DHT11_Instance].GPIO_PORT, &GPIO_InitStruct);
}


void dht11_SetPinInput(uint8_t DHT11_Instance){
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = dht11_CfgParam[DHT11_Instance].GPIO_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(dht11_CfgParam[DHT11_Instance].GPIO_PORT, &GPIO_InitStruct);
}


void dht11_Start(uint8_t DHT11_Instance){
	dht11_SetPinOutput(DHT11_Instance);
	HAL_GPIO_WritePin(dht11_CfgParam[DHT11_Instance].GPIO_PORT, dht11_CfgParam[DHT11_Instance].GPIO_PIN, GPIO_PIN_RESET);
	DELAY_MS(18); //wait for 18ms
	HAL_GPIO_WritePin(dht11_CfgParam[DHT11_Instance].GPIO_PORT, dht11_CfgParam[DHT11_Instance].GPIO_PIN, GPIO_PIN_SET);
	DELAY_US(30);   // 20–40µs
	dht11_SetPinInput(DHT11_Instance);

}


uint8_t dht11_CheckResponse(uint8_t DHT11_Instance){
	uint8_t response = 0;

	DELAY_US(40); //wait 40us


	if(!(HAL_GPIO_ReadPin(dht11_CfgParam[DHT11_Instance].GPIO_PORT, dht11_CfgParam[DHT11_Instance].GPIO_PIN))){

		DELAY_US(80); // wait 80us

		if((HAL_GPIO_ReadPin(dht11_CfgParam[DHT11_Instance].GPIO_PORT, dht11_CfgParam[DHT11_Instance].GPIO_PIN))){
			response = 1;

		}
	}

	uint32_t timeout = 0;
	while((HAL_GPIO_ReadPin(dht11_CfgParam[DHT11_Instance].GPIO_PORT, dht11_CfgParam[DHT11_Instance].GPIO_PIN))){// wait for end of response
		if (++timeout > 10000) return 0; // fail safely
	}


	return response;
}


uint8_t dht11_Read(uint8_t DHT11_Instance, uint8_t* temperature, uint8_t* humidity){
	uint8_t bits[5] = {0};

	dht11_Start(DHT11_Instance);


	if (!dht11_CheckResponse(DHT11_Instance)){

		return DHT11_NO_RESPONSE;
	}


	for (int i = 0; i < 40; i++) {

		// Wait for pin to go high
		uint32_t timeout = 0;
		while (!HAL_GPIO_ReadPin(dht11_CfgParam[DHT11_Instance].GPIO_PORT, dht11_CfgParam[DHT11_Instance].GPIO_PIN)) {
			if (++timeout > 1000) return DHT11_TIMEOUT;
		}

		// Delay ~30us to sample bit value
		DELAY_US(30);

		bits[i / 8] <<= 1;
		if (HAL_GPIO_ReadPin(dht11_CfgParam[DHT11_Instance].GPIO_PORT, dht11_CfgParam[DHT11_Instance].GPIO_PIN))
			bits[i / 8] |= 1;

		timeout = 0;
		while (HAL_GPIO_ReadPin(dht11_CfgParam[DHT11_Instance].GPIO_PORT, dht11_CfgParam[DHT11_Instance].GPIO_PIN)) {
			if (++timeout > 1000) return DHT11_TIMEOUT;
		}
	}



	// Checksum
	if ((bits[0] + bits[1] + bits[2] + bits[3]) != bits[4]) return DHT11_CHECKSUM_ERR;

	*humidity = bits[0];
	*temperature = bits[2];

//	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14); // Toggle red LED every time sensor values are read

	return DHT11_OK;
}
