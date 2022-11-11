#ifndef __TIMER_H__
#define __TIMER_H__

#include <ctime>
#include "communication.h"

class Timer
{
private:
    struct timespec ct, ct0, ct1, ct2;
    unsigned int tick0;
    Communication::Server<Communication::TCP_Server> server_port;


public:
    Timer()
    {
        
        clock_gettime(CLOCK_MONOTONIC, &(this->ct));

    }    
    double set_t0_ms();
    double get_t1_ms();
    double lead_ms();
    double lead_only_ms();
    void wait_until_ms(int ms);
    double wait_until_with_lead_ms(int ms);
    bool flag_when_ms(int ms);
    unsigned int getTick_ms();
    void print_lead_ms();
    void run();
    
};


#endif