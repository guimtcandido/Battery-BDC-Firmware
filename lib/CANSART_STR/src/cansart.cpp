#include "cansart.h"
#include "driver.h"

static uint8_t tx_ID;
static uint8_t tx_LENGHT;
static uint8_t tx_DATA[8];
static uint16_t tx_CHECKSUM;

static char rx_DATA_RAW[48];
static uint8_t rx_COUNTER = 0;
static uint8_t rx_ID = 0;
static uint8_t rx_LENGHT = 0;
static uint8_t rx_DATA[8];
static uint16_t rx_CHECKSUM = 0;

static uint16_t checksum = 0;

uint8_t transmitMessage(uint8_t ID, uint8_t *txmessageBuffer, uint8_t messageLength)
{
  tx_ID = ID;
  tx_LENGHT = messageLength;

  for (int i = 0; i < messageLength; i++)
  {
    tx_DATA[i] = txmessageBuffer[i];
  }

  tx_checksum_calculator();

  sendData(tx_ID);

  sendData(tx_LENGHT);

  for (int i = 0; i < tx_LENGHT; i++)
  {
    sendData(tx_DATA[i]);
  }

  sendData(tx_CHECKSUM);
  sendFinishData();
  return 0;
}

void tx_checksum_calculator()
{
  tx_CHECKSUM = tx_ID + tx_LENGHT;

  for (int i = 0; i < tx_LENGHT; i++)
  {
    tx_CHECKSUM += tx_DATA[i];
  }
}

uint8_t availableMessage(uint8_t *rxmessageBuffer)
{

  if (receiveUSART(rx_DATA_RAW))
  {

    if (convertData(rx_DATA_RAW))
    {

      if (rx_checksum_calculator() && rx_ID >= 10)
      {
        memcpy(rxmessageBuffer, rx_DATA, 8 * sizeof(uint8_t));
        return rx_ID;
      }
      else
      {
        rx_ID = 2;
      }
    }
    return 1;
  }
  return 0;
}

uint8_t rx_checksum_calculator()
{
  uint8_t temp_rx_LENGHT = rx_LENGHT;

  checksum = rx_ID + temp_rx_LENGHT;

  for (int i = 0; i < temp_rx_LENGHT; i++)
  {
    checksum += rx_DATA[i];
  }

  if (checksum == rx_CHECKSUM)
  {
    checksum = 0;
    return 1;
  }
  return temp_rx_LENGHT;
}

uint8_t convertData(char *receivedData)
{

  int numbersIndex = 0;
  int startIndex = 0;
  uint8_t flagControl = 0;
  uint8_t i = 0;

  while (receivedData[i] != '\n')
  {

    if (receivedData[i] == '\0')
    {

      if (startIndex == 0)
      {
        rx_ID = atoi(((char *)&receivedData[startIndex]));
        flagControl = 1;
      }
      else if (flagControl == 1)
      {
        rx_LENGHT = atoi((char *)&receivedData[startIndex]);
        flagControl = 0;
      }
      else if (numbersIndex < rx_LENGHT)
      {
        rx_DATA[numbersIndex++] = atoi((char *)&receivedData[startIndex]);
      }
      else
      {
        rx_CHECKSUM = atoi((char *)&receivedData[startIndex]);
      }
      startIndex = i + 1;
    }
    i++;
  }

  return 1;
}

uint8_t receiveUSART(char *buffer)
{
  if (newData() > 0)
  {

    buffer[rx_COUNTER] = getData();

    if (buffer[rx_COUNTER] == '\n')
    {
      rx_COUNTER = 0;
      return 1;
    }

    rx_COUNTER++;

  }
  return 0;
}



