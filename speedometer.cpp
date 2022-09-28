#include "speedometer.h"
#include "joystick.h"

#include "timer.h"

#include <limits.h>
#include <cmath>
#include <ctime>
#include <functional>

#include <iostream>

#include <map>

extern std::map<int, double> axis;
extern std::map<int, bool> button;

int Speedometer::pinA = 0;
int Speedometer::pinB = 0;
int Speedometer::counter = 0;

/*void Speedometer::set_init_time()
{
    clock_gettime(CLOCK_MONOTONIC, &(this->ct0));
}

double Speedometer::get_lead_time_ms()
{
    clock_gettime(CLOCK_MONOTONIC, &(this->ct1));
    
    return double((this->ct1.tv_sec - this->ct0.tv_sec)*1000 + (this->ct1.tv_nsec - this->ct0.tv_nsec)/1000000);
}*/

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
    double kv = 0.025;
    this->displacement += counter_diff*this->radius;
    this->speed = (counter_diff*this->radius/this->period*kv + this->speed*(1-kv));
}
void Speedometer::inc_encoder()
{    
    if(GPIO::digitalRead(Speedometer::pinA) == GPIO::digitalRead(Speedometer::pinB))
    {            
        Speedometer::counter++;
    }
}
void Speedometer::dec_encoder()
{
    if(GPIO::digitalRead(Speedometer::pinA) == GPIO::digitalRead(Speedometer::pinB))
    {
        Speedometer::counter--;
    }
}

int Speedometer::init(int pinA, int pinB, double period)
{
    Speedometer::pinA = pinA;
    Speedometer::pinB = pinB;
    this->period = period;
    
    //GPIO::init_gpio();
    GPIO::pinMode(Speedometer::pinA, GPIO::INPUT);
    GPIO::pinMode(Speedometer::pinB, GPIO::INPUT);
    
    
    GPIO::attachInterrupt(Speedometer::pinA, Speedometer::inc_encoder, GPIO::CHANGE);
    GPIO::attachInterrupt(Speedometer::pinB, Speedometer::dec_encoder, GPIO::CHANGE);
    
    return 0;
}
void Speedometer::run()
{
    this->init(6, 5, 10);
    int counter_past = Speedometer::counter;
    //this->set_init_time();
    this->timer.set_t0_ms();
    
    
    while(true)
    {        
        //double t = this->timer.lead_ms();//this->get_lead_time_ms();                
        /*while(t < this->period*1000) 
        {
            t = this->timer.lead_ms();//this->get_lead_time_ms();            
        }*/
        this->timer.wait_until_ms(this->period);
        
        int counter_current = Speedometer::counter;

        this->update(counter_past, counter_current);
        //std::cout << "Cnt: " << Speedometer::counter << " Speed: " << this->speed << std::endl;
        //std::cout << axis[Joystick::LS_X] << " " << axis[Joystick::LS_Y] << std::endl;
        
        //this->set_init_time();
        this->timer.set_t0_ms();
        counter_past = counter_current;
        
    }
}