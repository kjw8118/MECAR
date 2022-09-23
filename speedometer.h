#ifndef __SPEEDOMETER_H__
#define __SPEEDOMETER_H__

#include "gpio_interface.h"


class Speedometer
{
private:
    int pinA;
    int pinB;
    int counter = 0;
    double radius = 1;    
    double period = 0.01;
    double displacement = 0;
    double speed = 0;    
    double acceleration = 0;
    double get_time();
    void update(int counter_past, int counter_current);
    void inc_encoder();
    void dec_encoder();
    void (Speedometer::*f_inc_encoder)() = &(Speedometer::inc_encoder);
    void (Speedometer::*f_dec_encoder)() = &(Speedometer::dec_encoder);
    

public:
    Speedometer();    
    ~Speedometer();
    int init(int pinA, int piB, double period);
    void run();
};

#endif