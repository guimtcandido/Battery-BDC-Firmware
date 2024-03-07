#ifndef CANSART_H

#define CANSART_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include "cansart_db.h"
#include "cansartP.h"
#include "driver.h"


uint8_t transmitMessage(uint8_t ID, uint8_t *txmessageBuffer, uint8_t messageLength);
void tx_checksum_calculator();
uint8_t availableMessage(uint8_t *rxmessageBuffer);
uint8_t rx_checksum_calculator();
uint8_t convertData(char *receivedData);
uint8_t receiveUSART(char *buffer);

#endif
