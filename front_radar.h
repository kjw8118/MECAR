#ifndef __FRONTRADAR_H__
#define __FRONTRADAR_H__

#include "pcf8591.h"

class FrontRadar
{
private:
    int pin;
    double past = 0;
    double k = 0.2;    
    double volt2mm(int volt);
    PCF8591 pcf8591;

public:
    FrontRadar(/* args */){};
    void init();
    double getDistance();
    void run();
    ~FrontRadar(){};
};

#endif