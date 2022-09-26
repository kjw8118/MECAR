#include "handle.h"

#include "gpio_interface.h"

#include <iostream>



void Handle::init()
{
    this->js.init();
}


void Handle::run()
{
    this->init();

    while(true)
    {
        this->js.update();
        std::cout << this->js.get_axis(this->js.LS_X) << " " << this->js.get_axis(this->js.LS_Y) << " " << this->js.get_axis(this->js.LT) << " " << this->js.get_axis(this->js.RT) << std::endl;
    }
}