#include "pcf8591.h"
#include <iostream>
using namespace std;

#include "gpio_interface.h"
using namespace GPIO;

#include "pigpio.h"

void PCF8591::init()
{
    this->i2c.begin(this->addr);
    
}
int PCF8591::analogRead(uint8_t pin)
{
    uint8_t buf[2];
    buf[1] = 128;
    buf[0] = this->cmd|(0x03&pin);
    
    this->i2c.writeBytes(buf, 2);

    return this->i2c.readByte();

}
void PCF8591::analogWrite(uint8_t val)
{
    this->i2c.writeReg(this->cmd, val);
}