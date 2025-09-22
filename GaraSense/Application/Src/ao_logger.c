/*
 * ao_logger.c
 *
 *  Created on: Sep 9, 2025
 *      Author: timji
 */


#include "../Inc/ao_logger.h"


static void SD_Card_Test(void)
{
  FATFS FatFs;
  FIL Fil;
  FRESULT FR_Status;
  FATFS *FS_Ptr;
  UINT RWC, WWC; // Read/Write Word Counter
  DWORD FreeClusters;
  uint32_t TotalSize, FreeSpace;
  char RW_Buffer[200];
  do
  {
    //------------------[ Mount The SD Card ]--------------------
    FR_Status = f_mount(&FatFs, "", 1);
    if (FR_Status != FR_OK)
    {
      sprintf(TxBuffer, "Error! While Mounting SD Card, Error Code: (%i)\r\n", FR_Status);
      UART_Print(TxBuffer);
      break;
    }
    sprintf(TxBuffer, "SD Card Mounted Successfully! \r\n\n");
    UART_Print(TxBuffer);
    //------------------[ Get & Print The SD Card Size & Free Space ]--------------------
    f_getfree("", &FreeClusters, &FS_Ptr);
    TotalSize = (uint32_t)((FS_Ptr->n_fatent - 2) * FS_Ptr->csize * 0.5);
    FreeSpace = (uint32_t)(FreeClusters * FS_Ptr->csize * 0.5);
    sprintf(TxBuffer, "Total SD Card Size: %lu Bytes\r\n", TotalSize);
    UART_Print(TxBuffer);
    sprintf(TxBuffer, "Free SD Card Space: %lu Bytes\r\n\n", FreeSpace);
    UART_Print(TxBuffer);
    //------------------[ Open A Text File For Write & Write Data ]--------------------
    //Open the file
    FR_Status = f_open(&Fil, "TextFileWrite.txt", FA_WRITE | FA_READ | FA_CREATE_ALWAYS);
    if(FR_Status != FR_OK)
    {
      sprintf(TxBuffer, "Error! While Creating/Opening A New Text File, Error Code: (%i)\r\n", FR_Status);
      UART_Print(TxBuffer);
      break;
    }
    sprintf(TxBuffer, "Text File Created & Opened! Writing Data To The Text File..\r\n\n");
    UART_Print(TxBuffer);
    // (1) Write Data To The Text File [ Using f_puts() Function ]
    f_puts("Hello! From STM32 To SD Card Over SPI, Using f_puts()\n", &Fil);
    // (2) Write Data To The Text File [ Using f_write() Function ]
    strcpy(RW_Buffer, "Hello! From STM32 To SD Card Over SPI, Using f_write()\r\n");
    f_write(&Fil, RW_Buffer, strlen(RW_Buffer), &WWC);
    // Close The File
    f_close(&Fil);
    //------------------[ Open A Text File For Read & Read Its Data ]--------------------
    // Open The File
    FR_Status = f_open(&Fil, "TextFileWrite.txt", FA_READ);
    if(FR_Status != FR_OK)
    {
      sprintf(TxBuffer, "Error! While Opening (TextFileWrite.txt) File For Read.. \r\n");
      UART_Print(TxBuffer);
      break;
    }
    // (1) Read The Text File's Data [ Using f_gets() Function ]
    f_gets(RW_Buffer, sizeof(RW_Buffer), &Fil);
    sprintf(TxBuffer, "Data Read From (TextFileWrite.txt) Using f_gets():%s", RW_Buffer);
    UART_Print(TxBuffer);
    // (2) Read The Text File's Data [ Using f_read() Function ]
    f_read(&Fil, RW_Buffer, f_size(&Fil), &RWC);
    sprintf(TxBuffer, "Data Read From (TextFileWrite.txt) Using f_read():%s", RW_Buffer);
    UART_Print(TxBuffer);
    // Close The File
    f_close(&Fil);
    sprintf(TxBuffer, "File Closed! \r\n\n");
    UART_Print(TxBuffer);
    //------------------[ Open An Existing Text File, Update Its Content, Read It Back ]--------------------
    // (1) Open The Existing File For Write (Update)
    FR_Status = f_open(&Fil, "TextFileWrite.txt", FA_OPEN_EXISTING | FA_WRITE);
    FR_Status = f_lseek(&Fil, f_size(&Fil)); // Move The File Pointer To The EOF (End-Of-File)
    if(FR_Status != FR_OK)
    {
      sprintf(TxBuffer, "Error! While Opening (TextFileWrite.txt) File For Update.. \r\n");
      UART_Print(TxBuffer);
      break;
    }
    // (2) Write New Line of Text Data To The File
    FR_Status = f_puts("This New Line Was Added During Update!\r\n", &Fil);
    f_close(&Fil);
    memset(RW_Buffer,'\0',sizeof(RW_Buffer)); // Clear The Buffer
    // (3) Read The Contents of The Text File After The Update
    FR_Status = f_open(&Fil, "TextFileWrite.txt", FA_READ); // Open The File For Read
    f_read(&Fil, RW_Buffer, f_size(&Fil), &RWC);
    sprintf(TxBuffer, "Data Read From (TextFileWrite.txt) After Update:%s", RW_Buffer);
    UART_Print(TxBuffer);
    f_close(&Fil);
    //------------------[ Delete The Text File ]--------------------
    // Delete The File
    /*
    FR_Status = f_unlink(TextFileWrite.txt);
    if (FR_Status != FR_OK){
        sprintf(TxBuffer, "Error! While Deleting The (TextFileWrite.txt) File.. \r\n");
        UART_Print(TxBuffer);
    }
    */
  } while(0);
  //------------------[ Test Complete! Unmount The SD Card ]--------------------
  FR_Status = f_mount(NULL, "", 0);
  if (FR_Status != FR_OK)
  {
      sprintf(TxBuffer, "Error! While Un-mounting SD Card, Error Code: (%i)\r\n", FR_Status);
      UART_Print(TxBuffer);
  } else{
      sprintf(TxBuffer, "SD Card Un-mounted Successfully! \r\n");
      UART_Print(TxBuffer);
  }
}


//static void writeToFile(DataLogger * const me, Event const * const e){
//	snprintf(me->line, sizeof(me->line),
//			 "%lu,%.2f,%.2d,%.2d,%.2f,%d\r\n",
//			 osKernelGetTickCount(),
//			 e->payload.distance,
//			 e->payload.temp,
//			 e->payload.hum,
//			 e->payload.ppm,
//			 me->alertFlag);
//
//			f_puts(me->line, &me->logfile);
//			f_sync(&me->logfile);
//
//}

static void DataLoggerDispatch(DataLogger * const me, Event const * const e){


	switch(e->eventsig){
		case INIT_SIG:

			SD_Card_Test();




//			if (f_mount(&me->FatFs, "", 1) != FR_OK) {
//				sprintf(TxBuffer, "Error! While Mounting SD Card\r\n");
//				UART_Print(TxBuffer);
//				me->isOpen = 0;
//
//				Event noSD = {
//					  .eventsig = SD_UNAVAILABLE,
//					  .dest = BROKER
//				  };
//				 Active_post(&broker.super, &noSD);
//
//			}else{
//				sprintf(TxBuffer, "SD Card Mounted Successfully! \r\n\n");
//				UART_Print(TxBuffer);
//			}
//
//			if (f_open(&me->logfile, "log.csv", FA_WRITE | FA_OPEN_APPEND) == FR_OK) {
//
//				sprintf(TxBuffer, "Text File Created & Opened! Writing Data To The Text File..\r\n\n");
//				UART_Print(TxBuffer);
//
//				me->isOpen = 1;
//				// If file is new, write header
//				if (f_size(&me->logfile) == 0) {
//					f_puts("Time,Distance,Temp,Hum,PPM,Alert\r\n", &me->logfile);
//					f_sync(&me->logfile);
//				}
//
//				Event yesSD = {
//				        .eventsig = SD_AVAILABLE,
//				        .dest = BROKER
//				};
//
//				Active_post(&broker.super, &yesSD);
//				//TODO: Post to broker
//
//			} else {
//				UART_Print("Logger: failed to open log file\r\n");
//				me->isOpen = 0;
//
//
//			}
//			break;

//		case ALERT_EVT:
//			if(!me->isOpen) break;
//
//			me->alertFlag = 1;
//			writeToFile(me, e);
//
//			break;
//
//		case LOG_DATA:
//			if(!me->isOpen) break;
//
//			me->alertFlag = 0;
//			writeToFile(me, e);
//
//			break;
//
//		case DELETE_LOG:
//			if (me->isOpen){
//				f_close(&me->logfile);
//				me->isOpen = 0;
//			}
//			me->FR_Status = f_unlink("log.csv");
//			if (me->FR_Status != FR_OK){
//				sprintf(TxBuffer, "Error! Logger: failed to delete log.csv....\r\n");
//				UART_Print(TxBuffer);
//			}else{
//				sprintf(TxBuffer, "Logger: log.csv deleted....\r\n");
//				UART_Print(TxBuffer);
//			}
//			break;
//
//		case UNMOUNT_REQ:
//			if (me->isOpen) {
//				f_close(&me->logfile);
//				me->isOpen = 0;
//			}
//
//			me->FR_Status = f_mount(NULL, "", 0);
//
//			 Event dispEvt = {
//					  .eventsig = UNMOUNT_UPDATED,
//					  .dest = DISPLAY,
//					  .payload.isUnmount =  (me->FR_Status == FR_OK) ? 1 : 0
//			  };
//			 Active_post(&broker.super, &dispEvt);
//			 //TODO: post event of unmount result to broker with dst display
//
//			  if (me->FR_Status != FR_OK)
//			  {
//				  sprintf(TxBuffer, "Error! While Un-mounting SD Card, Error Code: (%i)\r\n", me->FR_Status);
//				  UART_Print(TxBuffer);
//			  } else{
//				  sprintf(TxBuffer, "SD Card Un-mounted Successfully! \r\n");
//				  UART_Print(TxBuffer);
//			  }
//
//
//			  Event noSD = {
//				  .eventsig = SD_UNAVAILABLE,
//				  .dest = BROKER
//			  };
//			  Active_post(&broker.super, &noSD);
//			  //TODO: also post to broker that sdcard has been unmount so dont send request here

		default:
			break;
	}
}

void DataLogger_Init(DataLogger * const me){
	Active_ctor(&me->super, (DispatchHandler)&DataLoggerDispatch);
}
