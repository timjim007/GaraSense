/*
 * ssd1306_cfg.h
 *
 *  Created on: Sep 6, 2025
 *      Author: timji
 */

#ifndef SSD1306_SSD1306_CFG_H_
#define SSD1306_SSD1306_CFG_H_

#include "ssd1306.h"


typedef struct{

	// I2C LCD module Instance Index
	uint8_t LCD_Instance;

	// I2C Hardware Peripheral Handle
	I2C_HandleTypeDef* I2C_Handle;

}ssd1306_CfgType;

extern const ssd1306_CfgType ssd1306_CfgParam[SSD1306_MAX];

#endif /* SSD1306_SSD1306_CFG_H_ */
