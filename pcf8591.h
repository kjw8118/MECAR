#ifndef __PCF8591_H__
#define __PCF8591_H__

#include "gpio_interface.h"
using namespace GPIO;

class PCF8591
{
private:
    const uint8_t addr = 0x48;
    const uint8_t cmd = 0x40;
    Wire i2c;
public:    
    void init();
    int analogRead(uint8_t pin);
    void analogWrite(uint8_t val);    
};





#endif