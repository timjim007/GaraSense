/*
 * ao_display.c
 *
 *  Created on: Sep 9, 2025
 *      Author: timji
 */


#include "../Inc/ao_display.h"

#include <stdio.h>

static void displayRender(Display * const me){
	ssd1306_Fill(Black);
//	 if (me->alert) {
//		ssd1306_SetCursor(0, 0);
//		ssd1306_WriteString("ALERT!", Font_11x18, White);
//		ssd1306_UpdateScreen(SSD1306_1);
//		me->alert = 0;
//		return;
//	}

	 switch (me->page) {
	 	 case TURNED_OFF:
			 ssd1306_Fill(Black);
			 ssd1306_SetCursor(0, 0);
			 ssd1306_WriteString("Alive and well", Font_11x18, White);
			 break;

		 case DISTANCE:
			 ssd1306_SetCursor(0, 0);
			 ssd1306_WriteString("Distance:", Font_11x18, White);
			 ssd1306_SetCursor(0, 20);
			 char buf1[16];
			 sprintf(buf1, "%.1f cm", me->distance);
			 ssd1306_WriteString(buf1, Font_11x18, White);
			 break;

		 case TEMP:
			 ssd1306_SetCursor(0, 0);
			 ssd1306_WriteString("Temp/Hum:", Font_11x18, White);
			 char buf2[16];
			 sprintf(buf2, "%.1d C", me->temp);
			 ssd1306_SetCursor(0, 20);
			 ssd1306_WriteString(buf2, Font_11x18, White);
			 sprintf(buf2, "%.1d %%", me->hum);
			 ssd1306_SetCursor(0, 40);
			 ssd1306_WriteString(buf2, Font_11x18, White);
			 break;

		 case AIR:
			 ssd1306_SetCursor(0, 0);
			 ssd1306_WriteString("CO2 Level:", Font_11x18, White);
			 char buf3[16];
			 sprintf(buf3, "%4.2f ppm", me->ppm);
			 ssd1306_SetCursor(0, 20);
			 ssd1306_WriteString(buf3, Font_11x18, White);
			 break;

		 case UNMOUNT_PAGE:
			 ssd1306_SetCursor(0, 0);
			ssd1306_WriteString("Unmount SDcard?", Font_7x10, White);

			ssd1306_SetCursor(0, 20);
			if (me->unmountChoice == 0) {
				ssd1306_WriteString("[NO]   YES", Font_7x10, White);
			} else {
				ssd1306_WriteString(" NO   [YES]", Font_7x10, White);
			}
			break;

		 case DONE:
			 ssd1306_SetCursor(0, 20);
			 ssd1306_WriteString("Success!", Font_11x18, White);

		 case FAILED:
			 ssd1306_SetCursor(0, 20);
			 ssd1306_WriteString("Failed", Font_11x18, White);

		 default:
			 break;

	 }

	 ssd1306_UpdateScreen(SSD1306_1);
}


static void DisplayDispatch(Display * const me, Event const * const e){
	switch(e->eventsig){
		case INIT_SIG:
			ssd1306_Init(SSD1306_1);
			me->page = DISTANCE;
			me->alert = 0;
			me->unmountDone = 0;

			displayRender(me);
			break;

		case LEFT_BUTTON_EVT:
			if(me->page == TURNED_OFF){
				me->page = UNMOUNT_PAGE;
			}else if(me->page == UNMOUNT_PAGE){
				me->unmountChoice ^= 1;
			}else if (me->page == DONE || me->page == FAILED) {
		        me->page = TURNED_OFF;
			}
			else{
				me->page--;
			}
			displayRender(me);

			break;
		case RIGHT_BUTTON_EVT:
			if(me->page == UNMOUNT_PAGE){
				me->unmountChoice ^= 1;

			}else  if (me->page == DONE || me->page == FAILED) {
		        me->page = TURNED_OFF;
			}
			else{
				me->page++;
			}
			displayRender(me);

			break;

		case CONFIRM_BUTTON_EVT:
			if (me->page == UNMOUNT_PAGE && me->unmountChoice == 1) {
				Event unmountEvt = {
					.eventsig = UNMOUNT_REQ,
					.dest = DATA_LOGGER
				};

				Active_post(&broker.super, &unmountEvt);
				//TODO:Post event to broker

			}else if(me->page == UNMOUNT_PAGE && me->unmountChoice == 0){
				me->page = DISTANCE;
			}
			break;

		case DIST_UPDATED:
			if(me->page == DISTANCE){
				me->distance = e->payload.distance;
				displayRender(me);
			}

			break;

		case TEMP_UPDATED:
			if(me->page == TEMP){
				me->temp = e->payload.temp;
				me->hum = e->payload.hum;
				displayRender(me);
			}

			break;

		case PPM_UPDATED:
			me->ppm = e->payload.ppm;
			if(me->page == AIR){
				displayRender(me);
			}
			break;

		case ALERT_EVT:
			me->alert = 1;

			displayRender(me);

			break;
		case UNMOUNT_UPDATED:

			me->unmountDone = (e->payload.isUnmount) ? 1 : 0;

			if(me->unmountDone){
				me->page = DONE;
			}else{
				me->page = FAILED;
			}
			displayRender(me);

			break;
		default:
			break;
	}
}

void Display_Init(Display * const me){
	Active_ctor(&me->super, (DispatchHandler)&DisplayDispatch);
}
