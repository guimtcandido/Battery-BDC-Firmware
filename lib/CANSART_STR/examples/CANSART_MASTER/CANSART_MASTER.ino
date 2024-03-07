#include <Arduino.h>
#include <HardwareSerial.h>
#include "cansart.h"
#include "SoftwareSerial.h"

SoftwareSerial mySerial(11, 10); // RX, TX

HardwareSerial &serialPort = Serial;
frame10 frames10;
frame23 frames23;
frame121 frames121;

uint8_t rx_ID = 0;
uint8_t rx_MSG1[8];
uint8_t data_RAW[20];
uint16_t j = 0;
unsigned long oldMillis = 0;
uint8_t newDATA_A = 0;
uint8_t newDATA_B = 0;
uint8_t RPM_C = 0;
void setup()
{
  setCANSART_Driver(serialPort, 9600);
  mySerial.begin(9600);
  mySerial.println("Vs3");
  mySerial.println("Receiver...");
  delay(500);
  mySerial.println("Looping...");
  mySerial.println("Master Mode");
  pinMode(2, INPUT_PULLUP);
}

void loop()
{
 
  if (digitalRead(2) == LOW)
  {
    mySerial.println("Data Change...");
    RPM_C = 0;
    newDATA_A = 1;
  }

 updateDB(&frames10);
 updateDB(&frames23);
  uint8_t var1 = updateDB(&frames121);
 
  if ( var1 == 1 && newDATA_A)
  {
    frames121.SetPower = RPM_C;
    write_slave_DB(&frames121);
    newDATA_A = 0;
  }

  if (millis() - oldMillis > 500)
  {
    mySerial.print("setRPM: ");
    mySerial.print(frames121.SetRPM);
    mySerial.print("  setPWR: ");
    mySerial.print(frames121.SetPower);
    mySerial.print("  RPM: ");
    mySerial.print(frames10.RPM);
    mySerial.print("  SPEED: ");
    mySerial.print(frames10.SPEED);
    mySerial.print("  TEMP: ");
    mySerial.print(frames23.TEMP);
    mySerial.print("  OIL: ");
    mySerial.println(frames23.OIL);

    oldMillis = millis();
  }
}
