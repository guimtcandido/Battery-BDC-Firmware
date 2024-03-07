#include "driver.h"

HardwareSerial *Lusart;

void setCANSART_Driver(HardwareSerial &usart,unsigned long baudrate)
{
  Lusart = &usart;
  Lusart->begin(baudrate);
 //Serial.begin(38400);
}

void sendData(uint16_t sendByte)
{
  
  Lusart->print(sendByte);
  Lusart->print('\0');
}

void sendFinishData()
{
  Lusart->print('\n');
}

int newData(){
  return Lusart->available();
}

uint8_t getData(){
  return Lusart->read();
}



