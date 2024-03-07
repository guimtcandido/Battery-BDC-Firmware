#include "CANDPID.h"

PID::PID(uint16_t MAX_SIGNAL_VALUE, uint16_t MIN_SIGNAL_VALUE)
{
    this->MAX_SIGNAL_VALUE = MAX_SIGNAL_VALUE;
    this->MIN_SIGNAL_VALUE = MIN_SIGNAL_VALUE;
}

void PID ::ParamSet(float Kp_set, float Ki_set, float Kd_set, float Ti, float Td)
{

    Kp = Kp_set;
    Ki = Ki_set / Ti;
    Kd = Kd_set * Td;
}

void PID ::updtError(double Setpoint, double valueNow)
{
        
    signalError = Setpoint - valueNow;

    signalError_INTEGRAL += signalError;

    antiWindup_Check();

    signalError_DIFERENTIAL = signalError - signalError_PREVIOUS;
    signalError_PREVIOUS = signalError; 
    
}

uint8_t PID :: OutSignal()
{

    CntrlSignal = Kp * signalError + Ki * signalError_INTEGRAL + Kd * signalError_DIFERENTIAL;
    
    if(CntrlSignal>MAX_SIGNAL_VALUE){
        CntrlSignal=MAX_SIGNAL_VALUE;
    }
    else if(CntrlSignal<MIN_SIGNAL_VALUE){
        CntrlSignal=MIN_SIGNAL_VALUE;
        return false;
    }

    return true;

}

uint32_t PID ::Control()
{

    return CntrlSignal;
}

uint8_t PID :: antiWindup_Check()
{

    if (signalError_INTEGRAL > MAX_SIGNAL_VALUE)
    {
        signalError_INTEGRAL = MAX_SIGNAL_VALUE;
        return true;
    }
    else if (signalError_INTEGRAL < MIN_SIGNAL_VALUE)
    {
        signalError_INTEGRAL = MIN_SIGNAL_VALUE;
        return false;
    }

    return 2;

}

void  PID :: resetPID()

{
    signalError = 0;
    signalError_INTEGRAL = 0;
    signalError_PREVIOUS = 0;
    signalError_DIFERENTIAL = 0;
    CntrlSignal = 0;
}

float PID :: getKp()
{
    return Kp;
}

float PID :: getKi()
{
    return Ki;
}

float PID :: getKd()
{
    return Kd;
}