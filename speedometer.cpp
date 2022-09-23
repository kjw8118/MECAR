#include "speedometer.h"

#include <limits.h>
#include <cmath>
#include <ctime>

double Speedometer::get_time()
{
    struct timespec ct;
    clock_gettime(CLOCK_MONOTONIC, &ct);
    return (double)(ct.tv_sec + ct.tv_nsec)/1000;
}
void Speedometer::update(int counter_past, int counter_current)
{
    int counter_diff;
    if(std::abs(counter_current - counter_past) > INT_MAX)
    {
        if(counter_current < counter_past)
        {
            counter_diff = INT_MAX + (INT_MIN - counter_current) - counter_past;
        }
        else
        {
            counter_diff =  INT_MIN - (INT_MAX - counter_current) - counter_past;
        }
    }
    else
    {
        counter_diff = counter_current - counter_past;
    }
    this->displacement += counter_diff*this->radius;
    this->speed = counter_diff*this->radius/this->period;
}
void Speedometer::inc_encoder()
{
    if(GPIO::digitalRead(pinA) == GPIO::digitalRead(this->pinB))
    {            
        this->counter++;
    }
}
void Speedometer::dec_encoder()
{
    if(GPIO::digitalRead(this->pinA) == GPIO::digitalRead(this->pinB))
    {
        this->counter--;
    }
}

int Speedometer::init(int pinA, int piB, double period)
{
    this->pinA = pinA;
    this->pinB = pinB;
    GPIO::init_gpio();
    GPIO::pinMode(this->pinA, GPIO::INPUT);
    GPIO::pinMode(this->pinB, GPIO::INPUT);
    GPIO::attachInterrupt(this->pinA, (void*)this->f_inc_encoder, GPIO::CHANGE);
    GPIO::attachInterrupt(this->pinB, (void*)this->f_dec_encoder, GPIO::CHANGE);
    //GPIO::attachInterrupt(this->pinA, (void*)&(Speedometer::inc_encoder), GPIO::CHANGE);
    //GPIO::attachInterrupt(this->pinB, (void*)&(Speedometer::dec_encoder), GPIO::CHANGE);

}
void Speedometer::run()
{
    this->init(10, 11, 0.01);
    int counter_past = this->counter;
    double t0 = this->get_time();
    while(true)
    {
        double t1 = this->get_time();
        while(t1 - t0 < this->period) 
        {
            t1 = this->get_time();
        }
        int counter_current = this->counter;

        this->update(counter_past, counter_current);
        
        t0 = t1;
        counter_past = counter_current;
        
    }
}