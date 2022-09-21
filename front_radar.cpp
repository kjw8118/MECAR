#include "front_radar.h"
#include "pcf8591.h"

#include <iostream>
using namespace std;

double FrontRadar::volt2mm(int volt)
{
    if (volt > 5000)
        volt = 5000;
    if (volt < 0)
        volt = 0;

    return (24.61 / (volt - 0.1696)) * 10000;
}

void FrontRadar::init()
{
    this->pin = 3;
    this->pcf8591.init();
}

double FrontRadar::getDistance()
{
    int volt = (int)((double)this->pcf8591.analogRead(this->pin)*5000/255);
    double dist = this->volt2mm(volt);
    return dist * this->k + this->past * (1 - this->k);
}

void FrontRadar::run()
{
    this->init();

    while(true)
    {
        cout << "Dist: " << this->getDistance() << endl;

    }
}