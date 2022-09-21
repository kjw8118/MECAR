#define __DEBUG__

#ifdef __DEBUG__
#include <iostream>
using namespace std;
#endif

#include "powertrain.h"

void Propulsion::init(int PIN_FW, int PIN_BW)
{
    this->PIN_FW = PIN_FW;
    this->PIN_BW = PIN_BW;
    init_gpio();
    pinMode(PIN_FW, OUTPUT);
    pinMode(PIN_BW, OUTPUT);
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

    double torque_next = this->torque + aps * this->acc_ratio + bps * this->dec_ratio - this->cst_ratio;
    if (torque_next > 100)
        torque_next = 100;
    if (torque_next < 0)
        torque_next = 0;
    int duty = (int)(torque_next * 255 / 100);
    switch (this->gear)
    {
    case Propulsion::GEAR_P:
        digitalWrite(this->PIN_FW, PINMODE::LOW);
        digitalWrite(this->PIN_BW, PINMODE::LOW);
        this->torque = 0;
        break;
    case Propulsion::GEAR_D:
        analogWrite(this->PIN_FW, duty);
        digitalWrite(this->PIN_BW, PINMODE::LOW);
        this->torque = torque_next;
        break;
    case Propulsion::GEAR_R:
        analogWrite(this->PIN_BW, duty);
        digitalWrite(this->PIN_FW, PINMODE::LOW);
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
    init_gpio();
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
    init_gpio();
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

    while(true)
    {
        this->driving(0,0,0);
    }

}