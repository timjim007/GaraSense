/*
 * ao_logger.h
 *
 *  Created on: Sep 9, 2025
 *      Author: timji
 */

#ifndef INC_AO_LOGGER_H_
#define INC_AO_LOGGER_H_

#include "ao.h"
#include "ao_broker.h"
#include "fatfs.h"

#include "../../MIDWARE/FATFS_SD/FATFS_SD.h"



extern Broker broker;



//
//FATFS FatFs;
//FRESULT FR_Status;
//FATFS *FS_Ptr;
//UINT RWC, WWC; // Read/Write Word Counter
//DWORD FreeClusters;
//uint32_t TotalSize, FreeSpace;
//char RW_Buffer[200];



/* DataLogger AO */
typedef struct{
	Active super;  /*Inherit Active base object*/

	/* private data */
	uint8_t isOpen;
	uint8_t alertFlag;
	char line[64];
//	FIL logfile;
//	FATFS FatFs;
//	FRESULT FR_Status;
//	FATFS *FS_Ptr;

}DataLogger;

void DataLogger_Init(DataLogger * const me);

#endif /* INC_AO_LOGGER_H_ */
