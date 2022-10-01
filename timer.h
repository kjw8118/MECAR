#ifndef __TIMER_H__
#define __TIMER_H__

#include <ctime>

class Timer
{
private:
    struct timespec ct, ct0, ct1, ct2;
    unsigned int tick0;

public:
    Timer()
    {
        clock_gettime(CLOCK_MONOTONIC, &(this->ct));

    }    
    double set_t0_ms();
    double get_t1_ms();
    double lead_ms();
    void wait_until_ms(int ms);
    bool flag_when_ms(int ms);
    unsigned int getTick_ms();
    
};


#endif