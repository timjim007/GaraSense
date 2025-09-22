/*
 * dht11.h
 *
 *
 */

#ifndef INC_DHT11_H_
#define INC_DHT11_H_

#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "dht11_cfg.h"

#define DHT11_OK            0
#define DHT11_NO_RESPONSE   1
#define DHT11_TIMEOUT       2
#define DHT11_CHECKSUM_ERR  3

void dht11_SetPinOutput(uint8_t DHT11_Instance);
void dht11_SetPinInput(uint8_t DHT11_Instance);
void dht11_Start(uint8_t DHT11_Instance);
uint8_t dht11_CheckResponse(uint8_t DHT11_Instance);
uint8_t dht11_Read(uint8_t DHT11_Instance, uint8_t* temperature, uint8_t* humidity);


#endif /* INC_DHT11_H_ */
