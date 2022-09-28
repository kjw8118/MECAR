#ifndef __JOYSTICK_H__
#define __JOYSTICK_H__

#include "linux/joystick.h"

#include <map>



class Joystick
{
private:
    int js_fd = -1;
    //double axis[6] = {0,};
    //std::map<int, double> axis;
    //std::map<int, bool> btn;
    //bool btn[11] = {false,};
    struct js_event js;        
public:
    enum AXIS_ID
    {
        LS_X = 0,
        LS_Y = 1,
        LT = 2,
        RS_X = 3,
        RS_Y = 4,
        RT = 5,
    };
    enum BUTTON_ID
    {
        A = 0,
        B = 1,
        X = 2,
        Y = 3,
        LB = 4,
        RB = 5,
        BK = 6,
        ST = 7,
        HM = 8,
        LS = 9,
        RS = 10,
    };
    //Joystick(/* args */);
    //~Joystick();
    void init();
    void get_current_state();
    double get_axis(unsigned char axis_id);
    std::map<int, double> get_axis();        
    bool get_button(unsigned char button_id);
    std::map<int, bool> get_button();
    void run();
};    
#endif