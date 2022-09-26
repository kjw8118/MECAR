#ifndef __HANDLE_H__
#define __HANDLE_H__

#include "gpio_interface.h"

class Handle
{
private:
    GPIO::Joystick js;
public:
    /*Handle();
    ~Handle();*/
    void init();
    void run();
};

#endif