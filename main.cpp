#include <iostream>
#include <thread>
using namespace std;

#include "powertrain.h"
#include "front_camera.h"
#include "front_radar.h"
#include "lidar.h"
#include "speedometer.h"

//#include <functional>

/*void f()
{
    std::cout << "function " << std::endl;
}*/

/*#include "gpio_interface.h"

#include "unistd.h"

int pinA = 6;
int pinB = 5;

int counter = 0;

void inc_encoder()
{
    if(GPIO::digitalRead(pinA) == GPIO::digitalRead(pinB))
    {            
        counter++;
    }
}
void dec_encoder()
{
    if(GPIO::digitalRead(pinA) == GPIO::digitalRead(pinB))
    {
        counter--;
    }
}*/

/*class TEST
{
    public:
    void func()
    {
        std::cout << "CLA Func" << std::endl;
    }
    
    

};*/

/*void call(void (*fp)())
{
    std::cout << "Im call " << fp() std::endl;
}*/

int main()
{        
    //thread pt_thread = thread(&Powertrain::run, Powertrain());
    //thread cam_thread = thread(&FrontCamera::run, FrontCamera());
    //thread radar_thread = thread(&FrontRadar::run, FrontRadar());
    //thread lidar_thread = thread(&LiDAR::run, LiDAR());
    thread speedometer_thread = thread(&Speedometer::run, Speedometer());

    //pt_thread.join();
    //cam_thread.join();
    //radar_thread.join();
    speedometer_thread.join();

    //TEST t1;
    //std::cout << "Start" << std::endl;
    //std::function<void()> fp = /*std::move(*/std::bind(&TEST::func, t1);//);
    //std::cout << "Def" << std::endl;
    
    //std::function<void()> fp = std::bind(&TEST::func, &t1);//&f;//std::bind(f, 6);
    //std::function<void()> fp2 = std::bind(f);//std::move(std::bind(f));
    //void (TEST::*ff)();
    //ff = &TEST::func;
    //(t1.*ff)();
    //call(t1.*ff);
    //auto ppfp = fp.target<void(*)()>();
    //auto ppfp = std::function<void()>(std::bind(&TEST::func, t1)).target<void(*)()>();
    //void (**ppfp)() = fp.target<void(*)()>();
    //auto ppfp2 = fp2.target<void(*)()>();
    //fp();
    //(*fp.target<void(*)()>())();
    //(*fp.target<void()>())();
    //(*ppfp)();
    //fp2();
    //std::cout << "Raw " << &f << std::endl;
    //std::cout << "std::f " << &fp << std::endl;
    //std::cout << "std::f2 " << &fp2 << std::endl;
    //std::cout << "std::f::targ " << ppfp << std::endl;
    //std::cout << "std::f::targ " << &(*fp.target<void(*)()>()) << std::endl;
    //std::cout << "std::f::targ2 " << ppfp2 << std::endl;
    //std::cout << "*std::f::targ " << *ppfp << std::endl;
    //std::cout << "*std::f::targ2 " << *ppfp2 << std::endl;

    
    /*auto ppfp = fp.target<void(*)()>();
    //std::cout << "Targ" << std::endl;
    //void (*pfp)() = *ppfp;
    //std::cout << "ff" << std::endl;
    std::cout << &TEST::func << std::endl;
    std::cout << &fp << std::endl;
    std::cout << *fp.target<void(*)()>() << std::endl;
    fp();
    (*ppfp)();
    std::cout << "call" << std::endl;*/

    /*GPIO::init_gpio();
    GPIO::pinMode(pinA, GPIO::INPUT);
    GPIO::pinMode(pinB, GPIO::INPUT);

    GPIO::attachInterrupt(pinA, (void*)(inc_encoder), GPIO::CHANGE);
    GPIO::attachInterrupt(pinB, (void*)(dec_encoder), GPIO::CHANGE);

    while(true)
    {
        std::cout << counter << std::endl;
        usleep(1000);
    }*/


    cout << "Main Exit" << endl;
    
        

}
