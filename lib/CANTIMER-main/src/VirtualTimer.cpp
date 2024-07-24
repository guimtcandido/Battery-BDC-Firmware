/*
 * VirtualTimer.c
 *
 *  Created on: Jan 21, 2024
 *      Author: Candido
 */
#include "VirtualTimer.h"

virtualTimer :: virtualTimer(unsigned long time_Set,char type){

	switch(type) {
	case 's':  //segundos
		timeSet = time_Set * 1000;
		break;
	case 'm': //min
		timeSet = time_Set * 60000;
		break;

	default: //ms
		timeSet = time_Set;
	break;
	}
}

uint8_t virtualTimer :: Q(){

	if((millis() - timeNow >= timeSet) & timerState){
		timerState = 0;
		return 1;
	}
	return 0;
}

void virtualTimer :: start(){
	if(!timerState){
		timerState = 1;
		timeNow = millis();
	}else{

	}

}

void virtualTimer :: reset(){
	timerState = 0;
}





