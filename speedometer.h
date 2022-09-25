#ifndef __SPEEDOMETER_H__
#define __SPEEDOMETER_H__

#include "gpio_interface.h"

#include <functional>
#include <cmath>


class Speedometer
{
private:
    static int pinA;
    static int pinB;
    static int counter;
    double radius = 0.1;    
    double period = 0.01;
    double displacement = 0;
    double speed = 0;    
    double acceleration = 0;
    struct timespec ct0, ct1;
    void set_init_time();
    double get_lead_time_ms();
    void update(int counter_past, int counter_current);
    static void inc_encoder();
    static void dec_encoder();
    //void (Speedometer::*f_inc_encoder)() = NULL;
    //void (Speedometer::*f_dec_encoder)() = NULL;
    //std::function<void()> f_inc_encoder;
    //std::function<void()> f_dec_encoder;

public:
    //Speedometer();    
    //~Speedometer();
    int init(int pinA, int pinB, double period);
    void run();
};

#endif