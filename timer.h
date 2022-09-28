#ifndef __TIMER_H__
#define __TIMER_H__

#include <ctime>

class Timer
{
private:
    struct timespec ct0, ct1;

public:

    double set_t0_ms();
    double get_t1_ms();
    double lead_ms();
    void wait_until_ms(int ms);
    
};


#endif