#include "cansartP.h"

#if !SLAVEMODE
static uint8_t BUS_Available_A = 1;
static uint8_t temp_ID_C = 0;
#endif

static uint8_t BUS_Available_B = 1;
static uint8_t rx_buffer[8];
static uint8_t temp_ID_B = 0;


static uint8_t tx_verify_buffer[8];

uint8_t startupDB = 1;

uint8_t updateDB(void *source)
{

    struct framesT dest;
    memcpy(&dest, source, sizeof(dest));

    uint8_t temp_ID_A = availableMessage(rx_buffer);
    if (temp_ID_A >= 10 && temp_ID_A <= 240)
    {
        temp_ID_B = temp_ID_A;
    }

#if !SLAVEMODE
    if (temp_ID_B == dest.ID && !BUS_Available_A)
#else
    if (temp_ID_B == dest.ID)
#endif
    {
#if !SLAVEMODE

        dest.DATA1 = rx_buffer[0];
        dest.DATA2 = rx_buffer[1];
        dest.DATA3 = rx_buffer[2];
        dest.DATA4 = rx_buffer[3];
        dest.DATA5 = rx_buffer[4];
        dest.DATA6 = rx_buffer[5];
        dest.DATA7 = rx_buffer[6];
        dest.DATA8 = rx_buffer[7];
        if (dest.ID >= 121 && dest.ID <= 140)
        {
            if (dest.DATA1 == tx_verify_buffer[0] && dest.DATA2 == tx_verify_buffer[1] && dest.DATA3 == tx_verify_buffer[2] && dest.DATA4 == tx_verify_buffer[3] && dest.DATA5 == tx_verify_buffer[4] && dest.DATA6 == tx_verify_buffer[5] && dest.DATA7 == tx_verify_buffer[6] && dest.DATA8 == 0)
            {
                temp_ID_C = 0;

                BUS_Available_B = 1;
            }
        }
        memcpy(source, &dest, sizeof(dest));
        BUS_Available_A = 1;
        temp_ID_B = 0;
        return 1;
#else
        if (temp_ID_B <= 120)
        {

            uint8_t temp_tx_buffer[8];
            temp_tx_buffer[0] = dest.DATA1;
            temp_tx_buffer[1] = dest.DATA2;
            temp_tx_buffer[2] = dest.DATA3;
            temp_tx_buffer[3] = dest.DATA4;
            temp_tx_buffer[4] = dest.DATA5;
            temp_tx_buffer[5] = dest.DATA6;
            temp_tx_buffer[6] = dest.DATA7;
            temp_tx_buffer[7] = dest.DATA8;

            transmitMessage(dest.ID, temp_tx_buffer, dest.LENGHT);
            temp_ID_B = 0;
        }
        else
        {
            if (!startupDB)
            {
                if (rx_buffer[7] == 1)
                {
                    dest.DATA1 = rx_buffer[0];
                    dest.DATA2 = rx_buffer[1];
                    dest.DATA3 = rx_buffer[2];
                    dest.DATA4 = rx_buffer[3];
                    dest.DATA5 = rx_buffer[4];
                    dest.DATA6 = rx_buffer[5];
                    dest.DATA7 = rx_buffer[6];
                    dest.DATA8 = 0;
                    rx_buffer[7] = 0;
                    memcpy(source, &dest, sizeof(dest));
                    transmitMessage(dest.ID, rx_buffer, dest.LENGHT);
                }
                else
                {
                    uint8_t temp_tx_buffer[8];
                    temp_tx_buffer[0] = dest.DATA1;
                    temp_tx_buffer[1] = dest.DATA2;
                    temp_tx_buffer[2] = dest.DATA3;
                    temp_tx_buffer[3] = dest.DATA4;
                    temp_tx_buffer[4] = dest.DATA5;
                    temp_tx_buffer[5] = dest.DATA6;
                    temp_tx_buffer[6] = dest.DATA7;
                    temp_tx_buffer[7] = dest.DATA8;
                    transmitMessage(dest.ID, temp_tx_buffer, dest.LENGHT);
                }
            }
            else
            {
                uint8_t temp_tx_buffer[8];
                temp_tx_buffer[0] = dest.DATA1;
                temp_tx_buffer[1] = dest.DATA2;
                temp_tx_buffer[2] = dest.DATA3;
                temp_tx_buffer[3] = dest.DATA4;
                temp_tx_buffer[4] = dest.DATA5;
                temp_tx_buffer[5] = dest.DATA6;
                temp_tx_buffer[6] = dest.DATA7;
                temp_tx_buffer[7] = dest.DATA8;
                transmitMessage(dest.ID, temp_tx_buffer, dest.LENGHT);
                startupDB = 0;
            }
            temp_ID_B = 0;
        }
        return 1;
#endif
    }
#if !SLAVEMODE
    else if (BUS_Available_A)
    {

        if (dest.ID >= 10 && dest.ID <= 120)
        {

            uint8_t tx_buffer[8] = {};

            transmitMessage(dest.ID, tx_buffer, dest.LENGHT);
            BUS_Available_A = 0;

            return 0;
        }
        else if (dest.ID >= 121 && dest.ID <= 240)
        {

            uint8_t tx_buffer[8] = {};

            transmitMessage(dest.ID, tx_buffer, dest.LENGHT);
            BUS_Available_A = 0;

            return 0;
        }
        return 2;
    }
#endif
    return 3;
}

uint8_t write_slave_DB(void *source)
{
    struct framesT dest;
    memcpy(&dest, source, sizeof(struct framesT));
    if (BUS_Available_B)
    {
        if (dest.ID >= 121 && dest.ID <= 240)
        {

            tx_verify_buffer[0] = dest.DATA1;
            tx_verify_buffer[1] = dest.DATA2;
            tx_verify_buffer[2] = dest.DATA3;
            tx_verify_buffer[3] = dest.DATA4;
            tx_verify_buffer[4] = dest.DATA5;
            tx_verify_buffer[5] = dest.DATA6;
            tx_verify_buffer[6] = dest.DATA7;
            tx_verify_buffer[7] = 1;

            transmitMessage(dest.ID, tx_verify_buffer, 8);
            BUS_Available_B = 0;
            return 1;
        }
        return 2;
    }
    else if (!BUS_Available_B)
    {
        return 0;
    }
    return 3;
}