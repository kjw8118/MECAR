#include "timer.h"

#include <ctime>
#include <functional>
#include <iostream>

#include <unistd.h>

double Timer::set_t0_ms()
{
    clock_gettime(CLOCK_MONOTONIC, &(this->ct0));
    return double(this->ct0.tv_sec*1000 + this->ct0.tv_nsec/1000000);
}


double Timer::get_t1_ms()
{
    clock_gettime(CLOCK_MONOTONIC, &(this->ct1));
    return double(this->ct1.tv_sec*1000 + this->ct1.tv_nsec/1000000);
}

double Timer::lead_only_ms()
{
    this->get_t1_ms();    
    return double((this->ct1.tv_sec - this->ct0.tv_sec)*1000 + (this->ct1.tv_nsec - this->ct0.tv_nsec)/1000000);
}

double Timer::lead_ms()
{
    
    //sleep(1);
    this->get_t1_ms();    
    double ret = double((this->ct1.tv_sec - this->ct0.tv_sec)*1000 + (this->ct1.tv_nsec - this->ct0.tv_nsec)/1000000);
    this->set_t0_ms();
    //std::cout << "Lead time: " << ret << std::endl;
    
    return ret;
}

void Timer::wait_until_ms(int ms)
{
    while(this->lead_only_ms() < ms) {};
    this->set_t0_ms();
}

double Timer::wait_until_with_lead_ms(int ms)
{
    while(this->lead_only_ms() <ms){};
    return this->lead_ms();
}

bool Timer::flag_when_ms(int ms)
{
    if(this->lead_ms() > ms)
        return true;
    else
        return false;
}
void Timer::print_lead_ms()
{
    sleep(2);
    this->get_t1_ms();
    double ret = double((this->ct1.tv_sec - this->ct0.tv_sec)*1000 + (this->ct1.tv_nsec - this->ct0.tv_nsec)/1000000);
    this->set_t0_ms();
    std::cout << "Lead time: " << ret << std::endl;
}
unsigned int Timer::getTick_ms()
{
    clock_gettime(CLOCK_MONOTONIC, &(this->ct2));
    return (unsigned int)((this->ct2.tv_sec - this->ct.tv_sec)*1000 + (this->ct2.tv_nsec - this->ct.tv_nsec)/1000000);
}

void Timer::run()
{
    this->set_t0_ms();
    auto func = std::bind(&Timer::print_lead_ms, this);
    this->server_port.regist_task(func);

    std::thread server_thread(&Communication::Server<Communication::TCP_Server>::run, this->server_port);
    while(true)
    {
        //this->server_port.response();
        sleep(1);
        std::cout << "Timer task" << std::endl;
    }
}