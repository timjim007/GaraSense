/*
 * ssd1306_cfg.c
 *
 *  Created on: Sep 6, 2025
 *      Author: timji
 */

#include "ssd1306_cfg.h"
#include "ssd1306.h"

extern I2C_HandleTypeDef hi2c1;

const ssd1306_CfgType ssd1306_CfgParam[SSD1306_MAX] = {
		{
				SSD1306_1,
				&hi2c1,
		}
};
