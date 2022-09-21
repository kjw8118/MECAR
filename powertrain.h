#ifndef __POWERTRAIN_H__
#define __POWERTRAIN_H__

#include "gpio_interface.h"
using namespace GPIO;

class Propulsion
{
private:
    int PIN_FW;
    int PIN_BW;
    int gear = GEAR_P;
    double torque = 0;
    double acc_ratio = 0.1;
    double dec_ratio = 0.2;
    double cst_ratio = 5;

public:
    enum
    {
        GEAR_P = 0,
        GEAR_D = 1,
        GEAR_R = -1,
    };
    Propulsion(/* args */) {};
    void init(int PIN_FW, int PIN_BW);
    void gear_change(int gear);
    void propulsion(double aps, double bps);    
    ~Propulsion() {};
};

class MDPS
{
private:
    int pin;
    Servo servo;
    int deg_left = 45;
    int deg_mid = 90;
    int deg_right = 135;

public:
    MDPS(){};
    void init(int pin);
    void turn(double position);
    ~MDPS(){};
};

class Powertrain
{
private:
    Propulsion engine;
    MDPS mdps;

public:    
    Powertrain();
    void init(int PIN_FW, int PIN_BW, int PIN_SW);    
    void driving(double aps, double bps, double strw);
    void run();
};



#endif