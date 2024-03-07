#ifndef CANSART_DB_H
#define CANSART_DB_H

#include <Arduino.h>

#define SLAVEMODE 1

struct frame10
{
    uint8_t ID = 10;
    uint8_t RPM = 0;
    uint8_t SPEED = 0;
    uint8_t DATA3 = 0;
    uint8_t DATA4 = 0;
    uint8_t DATA5 = 0;
    uint8_t DATA6 = 0;
    uint8_t DATA7 = 0;
    uint8_t DATA8 = 0;
    uint8_t LENGHT = 8;
};

struct frame23
{
    uint8_t ID = 23;
    uint8_t TEMP = 0;
    uint8_t OIL = 0;
    uint8_t DATA3 = 0;
    uint8_t DATA4 = 0;
    uint8_t DATA5 = 0;
    uint8_t DATA6 = 0;
    uint8_t DATA7 = 0;
    uint8_t DATA8 = 0;
    uint8_t LENGHT = 8;
};

struct frame121
{
    uint8_t ID = 121;
    uint8_t SetRPM = 0;
    uint8_t SetPower = 0;
    uint8_t DATA3 = 0;
    uint8_t DATA4 = 0;
    uint8_t DATA5 = 0;
    uint8_t DATA6 = 0;
    uint8_t DATA7 = 0;
    uint8_t DATA8 = 0;
    uint8_t LENGHT = 8;
};

#endif