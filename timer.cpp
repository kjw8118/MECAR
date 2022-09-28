#include "timer.h"

#include <ctime>

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



double Timer::lead_ms()
{
    this->get_t1_ms();
    return double((this->ct1.tv_sec - this->ct0.tv_sec)*1000 + (this->ct1.tv_nsec - this->ct0.tv_nsec)/1000000);
}

void Timer::wait_until_ms(int ms)
{
    while(this->lead_ms() < ms) {};
}