#ifndef CANSART_DB_H
#define CANSART_DB_H

#include <Arduino.h>

#define SLAVEMODE 1

struct frame10
{
    uint8_t ID = 10;
    uint8_t DATA1 = 0;
    uint8_t DATA2 = 0;
    uint8_t DATA3 = 0;
    uint8_t DATA4 = 0;
    uint8_t DATA5 = 0;
    uint8_t DATA6 = 0;
    uint8_t DATA7 = 0;
    uint8_t DATA8 = 0;
    uint8_t LENGHT = 8;
};

struct frame11
{
    uint8_t ID = 11;
    uint8_t DATA1 = 0;
    uint8_t DATA2 = 0;
    uint8_t DATA3 = 0;
    uint8_t DATA4 = 0;
    uint8_t DATA5 = 0;
    uint8_t DATA6 = 0;
    uint8_t DATA7 = 0;
    uint8_t DATA8 = 0;
    uint8_t LENGHT = 8;
};

struct frame12
{
    uint8_t ID = 12;
    uint8_t DATA1 = 0;
    uint8_t DATA2 = 0;
    uint8_t DATA3 = 0;
    uint8_t DATA4 = 0;
    uint8_t DATA5 = 0;
    uint8_t DATA6 = 0;
    uint8_t DATA7 = 0;
    uint8_t DATA8 = 0;
    uint8_t LENGHT = 8;
};

struct frame13
{
    uint8_t ID = 13;
    uint8_t DATA1 = 0;
    uint8_t DATA2 = 0;
    uint8_t DATA3 = 0;
    uint8_t DATA4 = 0;
    uint8_t DATA5 = 0;
    uint8_t DATA6 = 0;
    uint8_t DATA7 = 0;
    uint8_t DATA8 = 0;
    uint8_t LENGHT = 8;
};

struct frame14
{
    uint8_t ID = 14;
    uint8_t DATA1 = 0;
    uint8_t DATA2 = 0;
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
    uint8_t DATA1 = 0;
    uint8_t DATA2 = 0;
    uint8_t DATA3 = 0;
    uint8_t DATA4 = 0;
    uint8_t DATA5 = 0;
    uint8_t DATA6 = 0;
    uint8_t DATA7 = 0;
    uint8_t DATA8 = 0;
    uint8_t LENGHT = 8;
};

struct frame122
{
    uint8_t ID = 122;
    uint8_t DATA1 = 0;
    uint8_t DATA2 = 0;
    uint8_t DATA3 = 0;
    uint8_t DATA4 = 0;
    uint8_t DATA5 = 0;
    uint8_t DATA6 = 0;
    uint8_t DATA7 = 0;
    uint8_t DATA8 = 0;
    uint8_t LENGHT = 8;
};

#endif