#ifndef CANDPID_H

#define CANDPID_H
#include <Arduino.h>

class PID{
    
    private:

    float Kp = 0;
    float Ki = 0;
    float Kd = 0;
    float signalError = 0;
    float signalError_INTEGRAL = 0;
    float signalError_PREVIOUS = 0;
    float signalError_DIFERENTIAL = 0;
    double CntrlSignal = 0;
    uint16_t MAX_SIGNAL_VALUE=0;
    uint16_t MIN_SIGNAL_VALUE=0;

    public:

    PID (uint16_t MAX_SIGNAL_VALUE, uint16_t MIN_SIGNAL_VALUE);

    void ParamSet(float Kp, float Ki, float Kd, float Ti=1, float Td=1);
    void updtError(double Setpoint, double valueNow);  
    uint8_t antiWindup_Check();
    uint8_t OutSignal(); 
    uint32_t Control();
    void resetPID();
    float getKp();
    float getKi();
    float getKd();
};



#endif
