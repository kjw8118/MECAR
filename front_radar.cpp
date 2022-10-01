#include "front_radar.h"
#include "pcf8591.h"

#include <iostream>

#include <map>
#include <cmath>



double FrontRadar::volt2mm(int volt)
{
    if (volt > 5000)
        volt = 5000;
    if (volt < 0)
        volt = 0;

    return (40 / (double)volt) * 10000;
}
double FrontRadar::volt2cm(int volt)
{    

    return this->volt2mm(volt)/10;
}
void FrontRadar::check_object_state()
{
    double dist = this->target_distance;
    int state = this->object_state;
    if(state == this->NO_OBJECT)
    {
        if(dist < this->THRESHOLD_FAR - this->THRESHOLD_HYS)
        {
            this->object_state = this->FAR;
        }
        
    }
    else if(state == this->FAR)
    {
        if(dist < this->THRESHOLD_NEAR - this->THRESHOLD_HYS)
        {
            this->object_state = this->NEAR;
        }
        else if(dist > this->THRESHOLD_FAR + this->THRESHOLD_HYS)
        {
            this->object_state = this->NO_OBJECT;
        }
    }
    else if(state == this->NEAR)
    {
        if(dist < this->THRESHOLD_INVALID)
        {
            this->object_state = this->INVALID;
        }
        else if(dist > this->THRESHOLD_NEAR + this->THRESHOLD_HYS)
        {
            this->object_state = this->FAR;
        }
    }
    else if(state == this->INVALID)
    {
        if(dist > this->THRESHOLD_VALID)
        {
            this->object_state = this->NEAR;
        }
        
    }
}

void FrontRadar::init()
{
    this->pin = PCF8591::AIN3;
    this->pcf8591.init();
}

void FrontRadar::getTargetDistance()
{
    
    this->target_distance_past = this->target_distance;

    int volt = (int)((double)this->pcf8591.analogRead(this->pin)*5000/255);    
    double dist = this->volt2cm(volt);

    //std::cout << "volt: " << volt << " dist: " << dist << std::endl;
    if(std::abs(dist - this->target_distance_past) < 30)
        this->target_distance = dist * this->k + this->target_distance * (1 - this->k);
    else
        this->target_distance = dist * 1 + this->target_distance * 0;
    
    
}

void FrontRadar::getTargetSpeed()
{    
    double dt = this->timer.lead_ms();
    
    if(10 != 0)
    {
        double dist_diff = this->target_distance - this->target_distance_past;
        this->target_speed_past = this->target_speed;

        double spd = dist_diff/(100*0.001);
        //std::cout << "spd: " << spd << " diff " << dist_diff;
        
        this->target_speed = spd*this->vk + this->target_speed*(1-this->vk);
        //std::cout << (double)this->timer.lead_ms() << " " << dist_diff << " " << this->target_speed << std::endl;
    }
}
void FrontRadar::getTTC()
{
    double dist = this->target_distance;
    double spd = this->target_speed;
    if(spd < 0)
    {
        this->target_ttc = -dist/spd;
    }
}

void FrontRadar::check_operation_state()
{
    int target_state = this->object_state;
    int op_state = this->operation_state;
    switch(target_state)
    {
        case this->NO_OBJECT:
            this->operation_state = this->NO_DETECTION;
            break;
        case (this->FAR || this->NEAR):            
            if(this->target_ttc > 10)
            {
                this->operation_state = this->DETECTION;
            }
            else
            {
                if(this->target_ttc > 5)
                {
                    this->operation_state = this->COLLISION;
                }
                else
                    this->operation_state = this->WARNING;
            }            
            break;
        case this->INVALID:
            break;

    }

}
void FrontRadar::run()
{
    this->init();
    this->timer.set_t0_ms();
    unsigned int tick_10ms = 0, tick_100ms = 0, tick_1000ms;
    while(true)
    {
        //this->timer.wait_until_ms(this->period*1000);
        //this->timer.set_t0_ms();
        //this->timer.wait_until_ms(1);
        unsigned int tick_ms = this->timer.getTick_ms();
        if(tick_ms % 10 == 0 && tick_10ms != tick_ms)
        {
            tick_10ms = tick_ms;

            this->getTargetDistance();
            this->getTTC();            
            this->check_object_state();
            this->check_operation_state();
        }
        if(tick_ms % 100 == 0  && tick_100ms != tick_ms)
        {
            tick_100ms = tick_ms;
            
            
            this->getTargetSpeed();
            
            
            std::cout << " Tick: " << tick_ms << " Obj: " << this->object_state_map[this->object_state] << " Opr: " << this->operation_state_map[this->operation_state] << " Dist: " << this->target_distance << " Spd: " << this->target_speed << " TTC: " << this->target_ttc << std::endl;
        }
        if(tick_ms % 1000 == 0 && tick_1000ms != tick_ms)
        {
            tick_1000ms = tick_ms;
            
            
        }
        
        
        //this->timer.set_t0_ms();
    }
}