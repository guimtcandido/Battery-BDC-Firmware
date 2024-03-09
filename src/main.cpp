#include <Arduino.h>
#include "CANDPID.h"

#define VO_BATT_SENRATIO 11 // Medir empiricamente
#define IO_BATT_SENRATIO 1  // Medir empiricamente
#define VO_BST_SENRATIO 11  // Medir empiricamente
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

class PID Boost_PID(1000, 0);  //Não pode ter max value de 1024 senão a o mosfet fica sempre ligado
float dcbus_V_RAW = 0;
float dcbus_V_INTEGRAL = 0;
float dcbus_V = 0;
float dcbus_V_Setpoint = 0;
float dcbus_I_RAW = 0;
float dcbus_I_INTEGRAL = 0;
float dcbus_I = 0;
float dcbus_I_Setpoint = 0;

uint8_t avgCounter = 0;

uint8_t batt_CHARGED = false;

unsigned long vTimer_prev_0 = 0;

bool awakeMCU = false;

void setup()
{

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
}

void loop()
{

  monitorState();

  getConverterValues();

  processCycle();

  // safetyCheck();
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
  batt_I_RAW = (batt_I_RAW / 0.12)-Vcs_BAT_ref;
  //batt_I_RAW = (batt_I_RAW - 2.29) / (float)0.1431;

  dcbus_V_RAW = (float)(analogRead(DCBUS_VOLTAGE_PIN) >> 2) * (float)3.3 / (float)1023;
  dcbus_V_RAW = VO_BST_SENRATIO * dcbus_V_RAW;

  dcbus_I_RAW = (float)(analogRead(DCBUS_CURRENT_PIN) >> 2) * (float)3.3 / (float)1023;
  dcbus_I_RAW = (dcbus_I_RAW - 2.29) / (float)0.1746;

  batt_V_INTEGRAL += batt_V_RAW;
  batt_I_INTEGRAL += batt_I_RAW;
  dcbus_V_INTEGRAL += dcbus_V_RAW;
  dcbus_I_INTEGRAL += dcbus_I_RAW;

  if (avgCounter == 5)
  {
    batt_V = (batt_V_INTEGRAL / (float)5.00) - 1;
    batt_I = (batt_I_INTEGRAL / (float)5.00+0.1);
    dcbus_V = (dcbus_V_INTEGRAL / (float)5.00) - 3;
    dcbus_I = dcbus_I_INTEGRAL / (float)5.00;
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
    if(batt_V >=0){
      Serial.print("+");
    }
    Serial.print(batt_V);
    Serial.print("BC");
    if(batt_I >=0){
      Serial.print("+");
    }
    Serial.print(batt_I);
    Serial.print("IV");
    if(dcbus_V >=0){
      Serial.print("+");
    }	
    Serial.print(dcbus_V);
    Serial.print("IC");
    if(dcbus_I >=0){
      Serial.print("+");
    }
    Serial.print(dcbus_I);
    Serial.print("CM");
    Serial.print(operationMode);
    Serial.print("RE");
    Serial.print(digitalRead(CIRCUIT_BREAKER_PIN));
    Serial.print("BS");
    Serial.print(batt_CHARGED);
    printGAINS();

    Serial.print("SP");
    if (operationMode == BUCK_MODE)
    {
      Serial.println(batt_V_Setpoint);
    }
    else if (operationMode == BOOST_MODE)
    {
      Serial.println(dcbus_V_Setpoint);
    }
    else if (operationMode == BATT_CHARGE_MODE)
    {
      Serial.println(batt_I_Setpoint);
    }
    else if (operationMode == OFF_MODE)
    {
      Serial.println(0);
    }

    vTimer_prev_0 = millis();
  }

  if (Serial.available() > 0)
  {

    sscanf(Serial.readStringUntil('\n').c_str(), "%c%f", &letter, &tempValue);

    switch (letter)
    {
    case 'A':
      operationMode = (uint8_t)tempValue;
      break;

    case 'B':

      if (operationMode == BUCK_MODE)
      {
        batt_V_Setpoint = tempValue;
      }
      else if (operationMode == BOOST_MODE)
      {
        dcbus_V_Setpoint = tempValue;
      }
      break;

    case 'C':
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
    if(batt_V < 13.6){
       batt_currentControl();
    }
    else{
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
		Vcs_BAT_ref += (float)(analogRead(BAT_CURRENT_PIN) >> 2) * (float)3.3 / (float)1023;delay(2);// MUDAR ao MUDAR MCU
	}

	Vcs_BAT_ref = Vcs_BAT_ref / (float) 100;
	Vcs_BAT_ref = Vcs_BAT_ref / 0.12; //Mudar dependendo do Sensor
} 