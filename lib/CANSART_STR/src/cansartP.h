#ifndef CANSARTP_H
#define CANSARTP_H

#include "cansart.h"
struct framesT
{
    uint8_t ID = 0;
    uint8_t DATA1 = 0;
    uint8_t DATA2 = 0;
    uint8_t DATA3 = 0;
    uint8_t DATA4 = 0;
    uint8_t DATA5 = 0;
    uint8_t DATA6 = 0;
    uint8_t DATA7 = 0;
    uint8_t DATA8 = 0;
    uint8_t LENGHT = 0;
};


uint8_t updateDB(void *source);
uint8_t write_slave_DB(void *source);

#endif