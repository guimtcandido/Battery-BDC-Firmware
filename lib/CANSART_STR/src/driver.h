#ifndef DRIVER_H
#define DRIVER_H

#include"Arduino.h"


void setCANSART_Driver(HardwareSerial &usart,unsigned long baudrate);
void sendData(uint16_t sendByte);
void sendFinishData();

int newData();
uint8_t getData();


#endif