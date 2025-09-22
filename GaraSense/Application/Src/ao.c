/*
 * ao.c
 *
 *  Created on: Sep 9, 2025
 *      Author: timji
 */


#include "../Inc/ao.h"






/*--------------------------------------------------------*/
void Active_ctor(Active * const me, DispatchHandler dispatch){
	me->dispatch = dispatch;
}

/*--------------------------------------------------------*/
/* Thread function for all Active objects */
static void Active_eventLoop(void *argument){
	Active *me = (Active*)argument;

	/*Initialise the AO*/
	static Event const initEvt = {INIT_SIG};
	me->dispatch(me, &initEvt);

	/* event loop ("message pump") */
	for(;;){
		Event e; /*event object ("message")*/

		/*wait for any event and receive it into object 'e' */
		 if (osMessageQueueGet(me->queue, &e, NULL, osWaitForever) == osOK) {

			 /* dispatch event to the active object 'me' */
			me->dispatch(me, &e); /* NO BLOCKING */

		  }


	}

}

/*----------------------------------------------------------------*/
void Active_start(Active * const me, const osThreadAttr_t * threadAttr, osMessageQueueId_t  queueHandle){
	me->queue = queueHandle;
	me->thread = osThreadNew(Active_eventLoop, me, threadAttr);

}

/*----------------------------------------------------------------*/
void Active_post(Active * const me, Event const * const e){
	osStatus_t rv = osMessageQueuePut(me->queue,e ,0,0);
	if(rv != osOK){
		//error handle
	}
}

void AO_ReportStack(Active * const me, const char *name)
{
    size_t unused = osThreadGetStackSpace(me->thread);
    sprintf(TxBuffer,"%s thread: %u bytes free\r\n", name, (unsigned)unused);
    UART_Print(TxBuffer);
}

void AO_ReportQueue(Active * const me, const char *name)
{
	size_t free  = osMessageQueueGetSpace(me->queue);
	sprintf(TxBuffer, "%s queue Available: %u free\r\n",
	        name, (unsigned)free);
	UART_Print(TxBuffer);
}



