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
    enum PCF8591_PIN
    {
        AIN0 = 0,
        AIN1 = 1,
        AIN2 = 2,
        AIN3 = 3,
    };
    void init();
    int analogRead(uint8_t pin);
    void analogWrite(uint8_t val);    
};





#endif