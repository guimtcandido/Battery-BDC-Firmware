/*
 * VirtualTimer.h
 *
 *  Created on: Jan 21, 2024
 *      Author: Candido
 */

#ifndef INC_VIRTUALTIMER_HPP_
#define INC_VIRTUALTIMER_HPP_

#include "Arduino.h"

class virtualTimer{
private:

	unsigned long timeSet = 0;
	unsigned long timeNow = 0;
	uint8_t timerState = 0;

public:
	virtualTimer(unsigned long time_Set, char type);
	uint8_t Q();
	void start();
	void reset();
};

#endif /* INC_VIRTUALTIMER_HPP_ */
