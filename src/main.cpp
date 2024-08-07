#include <Arduino.h>
#include "CANDPID.h"
#include "cansart.h"
#include <HardwareSerial.h>
#include "VirtualTimer.h"


#define INVERTER_FREQ 50

#define INVERTER_PERIOD (1 / INVERTER_FREQ)
#define INVERTER_HALF_PERIOD_ms ((INVERTER_PERIOD / 2) * 1000)

HardwareSerial &serialPort = Serial;

#define VO_BATT_SENRATIO 0.0093673 // Medir empiricamente
#define IO_BATT_SENRATIO 0.0069767 // Medir empiricamente
#define VO_BST_SENRATIO 0.009022  // Medir empiricamente
#define IO_BST_SENRATIO 0.0086074  // Medir empiricamente

#define AVERAGE_SAMPLES 100

#define OFF_MODE 0
#define BUCK_MODE 1
#define BOOST_MODE 2
#define BATT_CHARGE_MODE 3

#define BOOST_MODE_OFF ledcWrite(2, 0)
#define BUCK_MODE_OFF ledcWrite(1, 0)

#define BUCK_PWM_PIN 22
#define BOOST_PWM_PIN 23
#define INVERTER_POS_PWM_PIN 21
#define INVERTER_NEG_PWM_PIN 19

#define BAT_VOLTAGE_PIN 15
#define BAT_CURRENT_PIN 14
#define DCBUS_VOLTAGE_PIN 4
#define DCBUS_CURRENT_PIN 35

#define BATTERY_CHARGE_CURRENT 1

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
void inverter_task();

uint8_t operationMode = 0;

class PID Buck_PID(1024, 0);
class PID Buck_I_PID(1024, 0);
float batt_V_RAW = 0;
double batt_V_INTEGRAL = 0;
float batt_V = 0;
float batt_V_Setpoint = 0;
float batt_I_RAW = 0;
double batt_I_INTEGRAL = 0;
float batt_I = 0;
float batt_I_Setpoint = 2;
float Vcs_BAT_ref = 0;

class PID Boost_PID(1000, 0); // Não pode ter max value de 1024 senão a o mosfet fica sempre ligado
float dcbus_V_RAW = 0;
double dcbus_V_INTEGRAL = 0;
float dcbus_V = 0;
float dcbus_V_Setpoint = 0;
float dcbus_I_RAW = 0;
double dcbus_I_INTEGRAL = 0;
float dcbus_I = 0;
float dcbus_I_Setpoint = 0;
float Vcs_DCBUS_ref = 0;
float batt_V_temp = 0;
float batt_I_temp = 0;
float dcbus_V_temp = 0;
float dcbus_I_temp = 0;

uint8_t inverter_cycle = 0;
uint8_t inverter_pos_request = 0;
uint8_t inverter_neg_request = 0;

uint8_t avgCounter = 0;

uint8_t batt_CHARGED = false;

unsigned long vTimer_prev_0 = 0;

bool awakeMCU = false;

virtualTimer sensorACQ_Timer(10, '-');
virtualTimer inverter_freq_Timer(9, '-'); // compensar o deadtimer
virtualTimer inverter_deadTime_Timer(1, '-');

frame10 frames10;
frame11 frames11;
frame12 frames12;
frame13 frames13;
frame14 frames14;
frame121 frames121;
frame122 frames122;

double pulse_width = 0;
unsigned long cycleTime = 0;
unsigned long start_timer_cycle = 0;

void setup()
{
  setCANSART_Driver(serialPort, 115200);
  ledcSetup(1, 20000, 10); // PWM BUCK
  ledcAttachPin(BUCK_PWM_PIN, 1);
  ledcWrite(1, 0);

  ledcSetup(2, 20000, 10); // PWM BOOST
  ledcAttachPin(BOOST_PWM_PIN, 2);
  ledcWrite(2, 0);

  // pinMode(INVERTER_POS_PWM_PIN, OUTPUT);
  // pinMode(INVERTER_NEG_PWM_PIN, OUTPUT);

  inverter_freq_Timer.start();

  // while (1)
  // {
  //   inverter_task();
  // }

  ledcSetup(3, 10000, 10); // PWM BOOST
  ledcAttachPin(INVERTER_POS_PWM_PIN, 3);
  ledcWrite(3, 0);

  ledcSetup(4, 10000, 10); // PWM BOOST
  ledcAttachPin(INVERTER_NEG_PWM_PIN, 4);
  ledcWrite(4, 0);
}

void loop()
{

  //  monitorState();

  getConverterValues();

  processCycle();
//while(1){
//  inverter_task();
//}
  // safetyCheck();

  cansartTasks();
}

void inverter_task()
{

  if (inverter_freq_Timer.Q())
  {

    if (!inverter_cycle)
    {
      inverter_neg_request = 0;

      inverter_deadTime_Timer.start();

      if (inverter_deadTime_Timer.Q())
      {

        inverter_pos_request = 1;
        inverter_cycle = 1;
        inverter_deadTime_Timer.reset();
        inverter_freq_Timer.reset();
        inverter_freq_Timer.start();
        start_timer_cycle = micros();
      }
    }
    else
    {
      inverter_pos_request = 0;

      inverter_deadTime_Timer.start();

      if (inverter_deadTime_Timer.Q())
      {
        inverter_neg_request = 1;
        inverter_cycle = 0;
        inverter_deadTime_Timer.reset();
        inverter_freq_Timer.reset();
        inverter_freq_Timer.start();
        start_timer_cycle = micros();
      }
    }
  }

  if (inverter_pos_request && inverter_neg_request)
  {
  }
  else
  {

    if (inverter_pos_request)
    {
      cycleTime = micros() - start_timer_cycle; 
      pulse_width = 1023 * sin(100 * 3.1415 * cycleTime/1e6);
      ledcWrite(3, pulse_width);
    }
    else
    {
      ledcWrite(3, 0);
    }

    if (inverter_neg_request)
    {
      cycleTime = micros() - start_timer_cycle; 
      pulse_width = 1023 * sin(100 * 3.1415 * cycleTime/1e6);
      ledcWrite(4, pulse_width);
    }
    else
    {
      ledcWrite(4, 0);
    }

    // digitalWrite(INVERTER_POS_PWM_PIN, inverter_pos_request);
    // digitalWrite(INVERTER_NEG_PWM_PIN, inverter_neg_request);
  }
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

    ledcWrite(1, Buck_PID.Control());
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
  batt_V_RAW = analogRead(BAT_VOLTAGE_PIN);

  batt_I_RAW = analogRead(BAT_CURRENT_PIN);

  dcbus_V_RAW = analogRead(DCBUS_VOLTAGE_PIN);

  dcbus_I_RAW = analogRead(DCBUS_CURRENT_PIN);

  batt_V_INTEGRAL += batt_V_RAW;
  batt_I_INTEGRAL += batt_I_RAW;
  dcbus_V_INTEGRAL += dcbus_V_RAW;
  dcbus_I_INTEGRAL += dcbus_I_RAW;

  if (avgCounter == AVERAGE_SAMPLES)
  {
    batt_V_temp = (batt_V_INTEGRAL / (float)AVERAGE_SAMPLES);
    batt_I_temp = (batt_I_INTEGRAL / (float)AVERAGE_SAMPLES);
    dcbus_V_temp = (dcbus_V_INTEGRAL / (float)AVERAGE_SAMPLES);
    dcbus_I_temp = (dcbus_I_INTEGRAL / (float)AVERAGE_SAMPLES);
    batt_V = batt_V_RAW * VO_BATT_SENRATIO;
    batt_I = batt_I_RAW * IO_BATT_SENRATIO - 14.0729416;
    dcbus_V = dcbus_V_RAW * VO_BST_SENRATIO;
    dcbus_I = dcbus_I_temp * IO_BST_SENRATIO - 17.128725;
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
      batt_I_Setpoint = BATTERY_CHARGE_CURRENT;
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

    ledcWrite(1, Buck_I_PID.Control());
  }
}

void safetyCheck()
{
  if (batt_V >= 14.2 || batt_I >= 4 || batt_I <= -4 || dcbus_V >= 25)
  {
    // digitalWrite(CIRCUIT_BREAKER_PIN, LOW);
    operationMode = OFF_MODE;
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
    frames13.DATA7 = ((uint16_t)(Buck_I_PID.getIntegral()) >> 8);
    frames13.DATA8 = (uint16_t)(Buck_I_PID.getIntegral());
  }
  else if (operationMode == OFF_MODE)
  {
    frames11.DATA3 = 0;
    frames11.DATA4 = 0;
  }

  // frames11.DATA5 = digitalRead(CIRCUIT_BREAKER_PIN);
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

  // digitalWrite(CIRCUIT_BREAKER_PIN, frames121.DATA4);

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
// GITUPDATE