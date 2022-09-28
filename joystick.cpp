#include "joystick.h"

#include "linux/joystick.h"

#include <unistd.h>
#include <fcntl.h>

#include <map>

#include <iostream>

std::map<int, double> axis;
std::map<int, bool> button;

void Joystick::init()
{
    this->js_fd = ::open("/dev/input/js0", O_RDONLY);
}
void Joystick::get_current_state()
{
    ::read(this->js_fd, &(this->js), sizeof(struct js_event));
    unsigned char type = this->js.type;
    unsigned char num = this->js.number;
    signed short val = this->js.value;
    switch(type & ~JS_EVENT_INIT)
    {
        case JS_EVENT_AXIS:
            //this->axis[num] = (double)val/INT16_MAX;
            axis[num] = (double)val/INT16_MAX;
            break;
        case JS_EVENT_BUTTON:
            //this->btn[num] = (bool)val;
            button[num] = (bool)val;
            break;
        default:
            break;
    }

}
double Joystick::get_axis(unsigned char axis_id)
{
    
    if(axis_id < Joystick::AXIS_ID::LS_X || axis_id > Joystick::AXIS_ID::RT)
    {
        return 0;
    }
    else
        return axis[axis_id];//this->axis[axis_id];
}
std::map<int, double> Joystick::get_axis()
{
    std::map<int, double> ret;
    //ret.insert(this->axis.begin(), this->axis.end());
    ret.insert(axis.begin(), axis.end());
    return ret;
}
bool Joystick::get_button(unsigned char button_id)
{
    if(button_id < Joystick::BUTTON_ID::A || button_id > Joystick::BUTTON_ID::RS)
    {            
        return false;
    }
    else
        return button[button_id];//this->btn[button_id];
}
std::map<int, bool> Joystick::get_button()
{
    std::map<int, bool> ret;
    //ret.insert(this->btn.begin(), this->btn.end());
    ret.insert(button.begin(), button.end());
    return ret;

}

void Joystick::run()
{

    

    this->init();

    while(true)
    {
        this->get_current_state();
        //std::map<int, double> axis_current = this->get_axis();
        //std::cout << axis_current[this->LS_X] << " " << axis_current[this->LS_Y] << std::endl;
        //std::cout << axis[this->LS_X] << " " << axis[this->LS_Y] << std::endl;
    }
}