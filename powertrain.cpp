//#define __DEBUG__

#include <iostream>

#ifdef __DEBUG__
#include <iostream>
using namespace std;
#endif

#include "powertrain.h"

#include "joystick.h"

#include <map>

extern std::map<int, double> axis;
extern std::map<int, bool> button;

void Propulsion::init(int PIN_FW, int PIN_BW)
{
    this->PIN_FW = PIN_FW;
    this->PIN_BW = PIN_BW;
    GPIO::init_gpio();
    GPIO::pinMode(PIN_FW, GPIO::OUTPUT);
    pinMode(PIN_BW, GPIO::OUTPUT);
}
void Propulsion::gear_change(int gear)
{
    this->gear = gear;
    this->propulsion(0, 0);
}

void Propulsion::propulsion(double aps, double bps)
{
    if(aps > 100)
        aps = 100;
    if(aps < 0)
        aps = 0;
    
    if(bps > 100)
        bps = 100;
    if(bps < 0)
        bps = 0;
    
    double torque_next = this->torque + aps * this->acc_ratio - bps * this->dec_ratio - this->cst_ratio;
    if (torque_next > 100)
        torque_next = 100;
    if (torque_next < 0)
        torque_next = 0;
    int duty = (int)(torque_next * 255 / 100);
    std::cout << "aps: " << aps << " bps: " << bps << " duty: " << duty << std::endl;
    switch (this->gear)
    {
    case Propulsion::GEAR_P:
        GPIO::digitalWrite(this->PIN_FW, GPIO::PINMODE::LOW);
        GPIO::digitalWrite(this->PIN_BW, GPIO::PINMODE::LOW);
        this->torque = 0;
        break;
    case Propulsion::GEAR_D:
        GPIO::analogWrite(this->PIN_FW, duty);
        GPIO::digitalWrite(this->PIN_BW, GPIO::PINMODE::LOW);
        this->torque = torque_next;
        break;
    case Propulsion::GEAR_R:
        GPIO::analogWrite(this->PIN_BW, duty);
        GPIO::digitalWrite(this->PIN_FW, GPIO::PINMODE::LOW);
        this->torque = torque_next;
        break;
    }

    #ifdef __DEBUG__
    cout << "propulsion put: " << this->gear << "-th " << duty << " from: " << this->torque << endl;
    #endif
}


void MDPS::init(int pin)
{
    this->pin = pin;
    GPIO::init_gpio();
    this->servo.attach(pin);
    this->servo.write(this->deg_mid);

    #ifdef __DEBUG__
    cout << "MDPS init" << endl;    
    #endif
}
void MDPS::turn(double position)
{
    if (position > 1)
        position = 1;
    if (position < -1)
        position = -1;

    int deg = this->deg_mid;
    if (deg >= 0)
    {
        deg += (int)(deg_right - deg_mid) * position;
    }
    else
    {
        deg += (int)(deg_mid - deg_left) * position;
    }    
    this->servo.write(deg);
    
    #ifdef __DEBUG__
    cout << "MDPS put: " << deg << " from: " << position << endl;
    #endif
}


Powertrain::Powertrain()
{
    
}
void Powertrain::init(int PIN_FW, int PIN_BW, int PIN_SW)
{
    GPIO::init_gpio();
    this->engine.init(PIN_FW, PIN_BW);
    this->mdps.init(PIN_SW);

    this->engine.gear_change(Propulsion::GEAR_D);
}
void Powertrain::driving(double aps, double bps, double strw)
{
    this->engine.propulsion(aps, bps);
    this->mdps.turn(strw);
}

void Powertrain::run()
{
    this->init(26, 27, 25);
    this->timer.set_t0_ms();
    while(true)
    {
        this->timer.wait_until_ms(10);
        double lt = (0.75 + axis[Joystick::LT])*100;
        double rt =  (0.75 + axis[Joystick::RT])*100;
        double axis_x_val = axis[Joystick::LS_X];

        this->driving(lt, rt, axis_x_val);
        //std::cout << "APS: " << -axis_y_val*100 << " BPS: " << axis_y_val*100 << std::endl;

        this->timer.set_t0_ms();
    }

}