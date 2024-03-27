#include <Arduino.h>
#include "CANDPID.h"
#include "cansart.h"
#include <HardwareSerial.h>
#include "VirtualTimer.h"

HardwareSerial &serialPort = Serial;

#define VO_BATT_SENRATIO 9.6501 // Medir empiricamente
#define IO_BATT_SENRATIO 1  // Medir empiricamente
#define VO_BST_SENRATIO 8.4671  // Medir empiricamente
#define IO_BST_SENRATIO 1   // Medir empiricamente

#define OFF_MODE 0
#define BUCK_MODE 1
#define BOOST_MODE 2
#define BATT_CHARGE_MODE 3

#define BOOST_MODE_OFF ledcWrite(2, 0)
#define BUCK_MODE_OFF ledcWrite(1, 1024)

#define BUCK_PWM_PIN 23
#define BOOST_PWM_PIN 22
#define INVERTER_POS_PWM_PIN 21
#define INVERTER_NEG_PWM_PIN 19

#define BAT_VOLTAGE_PIN 15
#define BAT_CURRENT_PIN 0
#define DCBUS_VOLTAGE_PIN 4
#define DCBUS_CURRENT_PIN 2

#define CIRCUIT_BREAKER_PIN 18

void buckProcess();
void boostProcess();
void monitorState();
void processCycle();
void getConverterValues();
void battCharge();
void batt_currentControl();
void safetyCheck();
void printGAINS();
void sensorCalib();
void cansartTasks();
uint8_t cansartInit();

uint8_t operationMode = 0;

class PID Buck_PID(1024, 0);
class PID Buck_I_PID(1024, 0);
float batt_V_RAW = 0;
float batt_V_INTEGRAL = 0;
float batt_V = 0;
float batt_V_Setpoint = 0;
float batt_I_RAW = 0;
float batt_I_INTEGRAL = 0;
float batt_I = 0;
float batt_I_Setpoint = 2;
float Vcs_BAT_ref = 0;

class PID Boost_PID(1000, 0); // Não pode ter max value de 1024 senão a o mosfet fica sempre ligado
float dcbus_V_RAW = 0;
float dcbus_V_INTEGRAL = 0;
float dcbus_V = 0;
float dcbus_V_Setpoint = 0;
float dcbus_I_RAW = 0;
float dcbus_I_INTEGRAL = 0;
float dcbus_I = 0;
float dcbus_I_Setpoint = 0;
float Vcs_DCBUS_ref = 0;
float batt_V_temp = 0;
float batt_I_temp = 0;
float dcbus_V_temp = 0;
float dcbus_I_temp = 0;

uint8_t avgCounter = 0;

uint8_t batt_CHARGED = false;

unsigned long vTimer_prev_0 = 0;

bool awakeMCU = false;

virtualTimer sensorACQ_Timer(10, '-');

frame10 frames10;
frame11 frames11;
frame12 frames12;
frame13 frames13;
frame14 frames14;
frame121 frames121;
frame122 frames122;

void setup()
{
  setCANSART_Driver(serialPort, 115200);
  ledcSetup(1, 20000, 10); // PWM BUCK
  ledcAttachPin(BUCK_PWM_PIN, 1);
  ledcWrite(1, 1024);

  ledcSetup(2, 20000, 10); // PWM BOOST
  ledcAttachPin(BOOST_PWM_PIN, 2);
  ledcWrite(2, 0);

  ledcSetup(3, 20000, 10); // PWM BOOST
  ledcAttachPin(INVERTER_POS_PWM_PIN, 3);
  ledcWrite(3, 512);

  ledcSetup(4, 20000, 10); // PWM BOOST
  ledcAttachPin(INVERTER_NEG_PWM_PIN, 4);
  ledcWrite(4, 512);

  Serial.begin(115200);

  pinMode(CIRCUIT_BREAKER_PIN, OUTPUT);
  digitalWrite(CIRCUIT_BREAKER_PIN, LOW);
  sensorCalib();

  sensorACQ_Timer.start();
}

void loop()
{

  //  monitorState();

  getConverterValues();

  processCycle();

  // safetyCheck();

  cansartTasks();
}

void processCycle()
{
  switch (operationMode)
  {
  case OFF_MODE:

    BOOST_MODE_OFF;
    BUCK_MODE_OFF;
    Buck_PID.resetPID();
    Boost_PID.resetPID();
    Buck_I_PID.resetPID();

    break;

  case BUCK_MODE:

    buckProcess();

    break;

  case BOOST_MODE:

    boostProcess();

    break;

  case BATT_CHARGE_MODE:
    battCharge();
    break;

  default:
    break;
  }
}

void buckProcess()
{
  Boost_PID.resetPID();
  BOOST_MODE_OFF;

  Buck_PID.updtError(batt_V_Setpoint, batt_V);

  if (Buck_PID.OutSignal())
  {

    ledcWrite(1, 1024 - Buck_PID.Control());
  }
}

void boostProcess()
{
  Buck_PID.resetPID();
  BUCK_MODE_OFF;

  Boost_PID.updtError(dcbus_V_Setpoint, dcbus_V);

  if (Boost_PID.OutSignal())
  {

    ledcWrite(2, Boost_PID.Control());
  }
}

void getConverterValues()
{
  batt_V_RAW = (float)(analogRead(BAT_VOLTAGE_PIN) >> 2) * (float)3.3 / (float)1023;
  batt_V_RAW = (float)VO_BATT_SENRATIO * batt_V_RAW;

  batt_I_RAW = (float)(analogRead(BAT_CURRENT_PIN) >> 2) * (float)3.3 / (float)1023;
  batt_I_RAW = (batt_I_RAW / 0.12) - Vcs_BAT_ref;
  // batt_I_RAW = (batt_I_RAW - 2.29) / (float)0.1431;

  dcbus_V_RAW = (float)(analogRead(DCBUS_VOLTAGE_PIN) >> 2) * (float)3.3 / (float)1023;
  dcbus_V_RAW = VO_BST_SENRATIO * dcbus_V_RAW;

  dcbus_I_RAW = (float)(analogRead(DCBUS_CURRENT_PIN) >> 2) * (float)3.3 / (float)1023;
  dcbus_I_RAW = (dcbus_I_RAW / 0.12) - Vcs_BAT_ref;

  batt_V_INTEGRAL += batt_V_RAW;
  batt_I_INTEGRAL += batt_I_RAW;
  dcbus_V_INTEGRAL += dcbus_V_RAW;
  dcbus_I_INTEGRAL += dcbus_I_RAW;

  if (avgCounter == 5)
  {
    batt_V_temp = (batt_V_INTEGRAL / (float)5.00);
    batt_I_temp = (batt_I_INTEGRAL / (float)5.00);
    dcbus_V_temp = (dcbus_V_INTEGRAL / (float)5.00);
    dcbus_I_temp = (dcbus_I_INTEGRAL / (float)5.00);
    avgCounter = 0;
    batt_V_INTEGRAL = 0;
    batt_I_INTEGRAL = 0;
    dcbus_V_INTEGRAL = 0;
    dcbus_I_INTEGRAL = 0;
  }
  else
  {
    avgCounter++;
  }
  
  if (sensorACQ_Timer.Q()) // Acquisition time 10ms
  {
    batt_V = batt_V_temp;
    batt_I = batt_I_temp;
    dcbus_V = dcbus_V_temp;
    dcbus_I = dcbus_I_temp;
    sensorACQ_Timer.start();
  }
}

void monitorState()

{
  char letter;
  float tempValue;
  if (millis() - vTimer_prev_0 >= 500)
  {
    Serial.print("CN");
    Serial.print(Buck_I_PID.Control());
    Serial.print("WK");
    Serial.print(awakeMCU = !awakeMCU);
    Serial.print("BV");
    if (batt_V >= 0)
    {
      Serial.print("+");
    }
    Serial.print(batt_V); // Já está
    Serial.print("BC");
    if (batt_I >= 0)
    {
      Serial.print("+");
    }
    Serial.print(batt_I); // Já está
    Serial.print("IV");
    if (dcbus_V >= 0)
    {
      Serial.print("+");
    }
    Serial.print(dcbus_V); // Já está
    Serial.print("IC");
    if (dcbus_I >= 0)
    {
      Serial.print("+");
    }
    Serial.print(dcbus_I); // Já está
    Serial.print("CM");
    Serial.print(operationMode); // Já está
    Serial.print("RE");
    Serial.print(digitalRead(CIRCUIT_BREAKER_PIN)); // Já está
    Serial.print("BS");
    Serial.print(batt_CHARGED); // Já está
    printGAINS();               // Já está

    Serial.print("SP");
    if (operationMode == BUCK_MODE)
    {
      Serial.println(batt_V_Setpoint); // Já está
    }
    else if (operationMode == BOOST_MODE)
    {
      Serial.println(dcbus_V_Setpoint); // Já está
    }
    else if (operationMode == BATT_CHARGE_MODE)
    {
      Serial.println(batt_I_Setpoint); // Já está
    }
    else if (operationMode == OFF_MODE)
    {
      Serial.println(0); // Já está
    }

    vTimer_prev_0 = millis();
  }

  if (Serial.available() > 0)
  {

    sscanf(Serial.readStringUntil('\n').c_str(), "%c%f", &letter, &tempValue);

    switch (letter)
    {
    case 'A': // Já está
      operationMode = (uint8_t)tempValue;
      break;

    case 'B': // Já está

      if (operationMode == BUCK_MODE)
      {
        batt_V_Setpoint = tempValue;
      }
      else if (operationMode == BOOST_MODE)
      {
        dcbus_V_Setpoint = tempValue;
      }
      break;

    case 'C': // Já está
      if (tempValue)
      {
        digitalWrite(CIRCUIT_BREAKER_PIN, HIGH);
      }
      else
      {
        digitalWrite(CIRCUIT_BREAKER_PIN, LOW);
      }
      break;

    case 'D':

      batt_V_Setpoint = tempValue;

      break;

    case 'E':
      batt_I_Setpoint = tempValue;
      break;

    case 'F':
      if (operationMode == BUCK_MODE)
        Buck_PID.ParamSet(tempValue, Buck_PID.getKi(), Buck_PID.getKd(), 1, 1);
      else if (operationMode == BOOST_MODE)
        Boost_PID.ParamSet(tempValue, Boost_PID.getKi(), Boost_PID.getKd(), 1, 1);
      else if (operationMode == BATT_CHARGE_MODE)
        Buck_I_PID.ParamSet(tempValue, Buck_I_PID.getKi(), Buck_I_PID.getKd(), 1, 1);

      break;

    case 'G':
      if (operationMode == BUCK_MODE)
        Buck_PID.ParamSet(Buck_PID.getKp(), tempValue, Buck_PID.getKd(), 1, 1);
      else if (operationMode == BOOST_MODE)
        Boost_PID.ParamSet(Boost_PID.getKp(), tempValue, Boost_PID.getKd(), 1, 1);
      else if (operationMode == BATT_CHARGE_MODE)
        Buck_I_PID.ParamSet(Buck_I_PID.getKp(), tempValue, Buck_I_PID.getKd(), 1, 1);
      break;

    case 'H':
      if (operationMode == BUCK_MODE)
      {
        Buck_PID.ParamSet(Buck_PID.getKp(), Buck_PID.getKi(), tempValue, 1, 1);
        Buck_PID.resetPID();
      }
      else if (operationMode == BOOST_MODE)
      {
        Boost_PID.ParamSet(Boost_PID.getKp(), Boost_PID.getKi(), tempValue, 1, 1);
        Boost_PID.resetPID();
      }
      else if (operationMode == BATT_CHARGE_MODE)
      {
        Buck_I_PID.ParamSet(Buck_I_PID.getKp(), Buck_I_PID.getKi(), tempValue, 1, 1);
        Buck_I_PID.resetPID();
      }
      break;

    default:
      break;
    }
  }
}

void battCharge()
{

  if (batt_V >= 13.6 && batt_I <= 0.5)
  {
    BUCK_MODE_OFF;
    batt_CHARGED = true;
    Buck_PID.resetPID();
  }
  else if (!batt_CHARGED)
  {
    batt_CHARGED = false;
    if (batt_V < 13.6)
    {
      batt_currentControl();
    }
    else
    {
      batt_V_Setpoint = 13.6;
      buckProcess();
    }
  }
}

void batt_currentControl()
{

  BOOST_MODE_OFF;

  Buck_I_PID.updtError(batt_I_Setpoint, batt_I);

  if (Buck_I_PID.OutSignal())
  {

    ledcWrite(1, 1024 - Buck_I_PID.Control());
  }
}

void safetyCheck()
{
  if (batt_V >= 14.2 || batt_I >= 4 || batt_I <= -4 || dcbus_V >= 25)
  {
    digitalWrite(CIRCUIT_BREAKER_PIN, LOW);
    operationMode = OFF_MODE;
  }
}

void printGAINS()
{
  Serial.print("KP");
  if (operationMode == BUCK_MODE)
  {
    Serial.print(Buck_PID.getKp());
  }
  else if (operationMode == BOOST_MODE)
  {
    Serial.print(Boost_PID.getKp());
  }
  else if (operationMode == BATT_CHARGE_MODE)
  {
    Serial.print(Buck_I_PID.getKp());
  }
  Serial.print("KI");
  if (operationMode == BUCK_MODE)
  {
    Serial.print(Buck_PID.getKi());
  }
  else if (operationMode == BOOST_MODE)
  {
    Serial.print(Boost_PID.getKi());
  }
  else if (operationMode == BATT_CHARGE_MODE)
  {
    Serial.print(Buck_I_PID.getKi());
  }
  Serial.print("KD");
  if (operationMode == BUCK_MODE)
  {
    Serial.print(Buck_PID.getKd());
  }
  else if (operationMode == BOOST_MODE)
  {
    Serial.print(Boost_PID.getKd());
  }
  else if (operationMode == BATT_CHARGE_MODE)
  {
    Serial.print(Buck_I_PID.getKd());
  }
}

void sensorCalib()
{

  for (int i = 0; i < 100; i++)
  {
    Vcs_BAT_ref += (float)(analogRead(BAT_CURRENT_PIN) >> 2) * (float)3.3 / (float)1023;
    Vcs_DCBUS_ref += (float)(analogRead(DCBUS_CURRENT_PIN) >> 2) * (float)3.3 / (float)1023;
    delay(2); // MUDAR ao MUDAR MCU
  }

  Vcs_BAT_ref = Vcs_BAT_ref / (float)100;
  Vcs_BAT_ref = Vcs_BAT_ref / 0.12; // Mudar dependendo do Sensor
  Vcs_DCBUS_ref = Vcs_BAT_ref / (float)100;
  Vcs_DCBUS_ref = Vcs_BAT_ref / 0.12; // Mudar dependendo do Sensor
}

void cansartTasks()
{

  frames10.DATA1 = ((uint16_t)(batt_V * 100 + 1000) >> 8);
  frames10.DATA2 = (uint16_t)(batt_V * 100 + 1000);

  frames10.DATA3 = ((uint16_t)(batt_I * 100 + 1000) >> 8);
  frames10.DATA4 = (uint16_t)(batt_I * 100 + 1000);

  frames10.DATA5 = ((uint16_t)(dcbus_V * 100 + 1000) >> 8);
  frames10.DATA6 = (uint16_t)(dcbus_V * 100 + 1000);

  frames10.DATA7 = operationMode;

  frames11.DATA1 = ((uint16_t)(dcbus_I * 100 + 1000) >> 8);
  frames11.DATA2 = (uint16_t)(dcbus_I * 100 + 1000);

  if (operationMode == BUCK_MODE)
  {
    frames11.DATA3 = ((uint16_t)(batt_V_Setpoint * 100) >> 8);
    frames11.DATA4 = (uint16_t)(batt_V_Setpoint * 100);
    frames14.DATA7 = ((uint16_t)(Buck_PID.Control()) >> 8); 
    frames14.DATA8 = (uint16_t)(Buck_PID.Control());
    frames13.DATA7 = ((uint16_t)(Buck_PID.getIntegral()) >> 8);
    frames13.DATA8 = (uint16_t)(Buck_PID.getIntegral());
  }
  else if (operationMode == BOOST_MODE)
  {
    frames11.DATA3 = ((uint16_t)(dcbus_V_Setpoint * 100) >> 8);
    frames11.DATA4 = (uint16_t)(dcbus_V_Setpoint * 100);
    frames14.DATA7 = ((uint16_t)(Boost_PID.Control()) >> 8); 
    frames14.DATA8 = (uint16_t)(Boost_PID.Control());
    frames13.DATA7 = ((uint16_t)(Boost_PID.getIntegral()) >> 8);
    frames13.DATA8 = (uint16_t)(Boost_PID.getIntegral());
  }
  else if (operationMode == BATT_CHARGE_MODE)
  {
    frames11.DATA3 = ((uint16_t)(batt_I_Setpoint * 100) >> 8);
    frames11.DATA4 = (uint16_t)(batt_I_Setpoint * 100);
    frames14.DATA7 = ((uint16_t)(Buck_I_PID.Control()) >> 8); 
    frames14.DATA8 = (uint16_t)(Buck_I_PID.Control());
  }
  else if (operationMode == OFF_MODE)
  {
    frames11.DATA3 = 0;
    frames11.DATA4 = 0;
  }

  frames11.DATA5 = digitalRead(CIRCUIT_BREAKER_PIN);
  frames11.DATA6 = batt_CHARGED;

  frames12.DATA1 = ((uint16_t)(Buck_PID.getKp() * 100) >> 8);
  frames12.DATA2 = (uint16_t)(Buck_PID.getKp() * 100);
  frames12.DATA3 = ((uint16_t)(Buck_PID.getKi() * 100) >> 8);
  frames12.DATA4 = (uint16_t)(Buck_PID.getKi() * 100);
  frames12.DATA5 = ((uint16_t)(Buck_PID.getKd() * 100) >> 8);
  frames12.DATA6 = (uint16_t)(Buck_PID.getKd() * 100);

  frames13.DATA1 = ((uint16_t)(Boost_PID.getKp() * 100) >> 8);
  frames13.DATA2 = (uint16_t)(Boost_PID.getKp() * 100);
  frames13.DATA3 = ((uint16_t)(Boost_PID.getKi() * 100) >> 8);
  frames13.DATA4 = (uint16_t)(Boost_PID.getKi() * 100);
  frames13.DATA5 = ((uint16_t)(Boost_PID.getKd() * 100) >> 8);
  frames13.DATA6 = (uint16_t)(Boost_PID.getKd() * 100);

  frames14.DATA1 = ((uint16_t)(Buck_I_PID.getKp() * 100) >> 8);
  frames14.DATA2 = (uint16_t)(Buck_I_PID.getKp() * 100);
  frames14.DATA3 = ((uint16_t)(Buck_I_PID.getKi() * 100) >> 8);
  frames14.DATA4 = (uint16_t)(Buck_I_PID.getKi() * 100);
  frames14.DATA5 = ((uint16_t)(Buck_I_PID.getKd() * 100) >> 8);
  frames14.DATA6 = (uint16_t)(Buck_I_PID.getKd() * 100);

  

  updateDB(&frames10);
  updateDB(&frames11);
  updateDB(&frames12);
  updateDB(&frames13);
  updateDB(&frames14);
  updateDB(&frames121);
  updateDB(&frames122);

  operationMode = frames121.DATA1;

  digitalWrite(CIRCUIT_BREAKER_PIN, frames121.DATA4);

  float tempValue = ((float)(frames122.DATA1 << 8 | frames122.DATA2) / 100);
  float tempValue2 = ((float)(frames122.DATA3 << 8 | frames122.DATA4) / 100);
  float tempValue3 = ((float)(frames122.DATA5 << 8 | frames122.DATA6) / 100);

  if (operationMode == BUCK_MODE)
  {
    batt_V_Setpoint = ((float)(frames121.DATA2 << 8 | frames121.DATA3) / 100);
    Buck_PID.ParamSet(tempValue, tempValue2, tempValue3, 1, 1);
  }
  else if (operationMode == BOOST_MODE)
  {
    dcbus_V_Setpoint = ((float)(frames121.DATA5 << 8 | frames121.DATA6) / 100);
    Boost_PID.ParamSet(tempValue, tempValue2, tempValue3, 1, 1);
  }

  if (operationMode == BATT_CHARGE_MODE)
  {
    Buck_I_PID.ParamSet(tempValue, tempValue2, tempValue3, 1, 1);
  }
}
//GITUPDATE