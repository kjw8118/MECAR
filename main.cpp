#include <iostream>
#include <thread>
using namespace std;

//#include "gpio_interface.h"

//#include "powertrain.h"
//#include "front_camera.h"
//#include "front_radar.h"
//#include "lidar.h"
//#include "speedometer.h"
//#include "joystick.h"

#include "timer.h"

#include "communication.h"
#include "unistd.h"


int main()
{
    //thread gpio_thread = thread(&GPIO::init_gpio);
    //gpio_thread.join();

    //thread pt_thread = thread(&Powertrain::run, Powertrain());
    //thread cam_thread = thread(&FrontCamera::run, FrontCamera());
    //thread radar_thread = thread(&FrontRadar::run, FrontRadar());
    //thread lidar_thread = thread(&LiDAR::run, LiDAR());
    //thread speedometer_thread = thread(&Speedometer::run, Speedometer());
    //thread joystick_thread = thread(&Joystick::run, Joystick());
    thread timer_thread = thread(&Timer::run, Timer());

    //pt_thread.join();
    //cam_thread.join();
    //radar_thread.join();
    //speedometer_thread.join();
    //joystick_thread.join();
    timer_thread.join();

    //Timer timer;
    //timer.run();

    Communication::Client<Communication::TCP_Client> client;
    
    client.connect("192.168.0.28", 8118);

    std::cout << "Now enter loop" << std::endl;
    while(true)
    {
        client.request();
        sleep(1);
    }


    
    /*Communication::TCP_Server server;

    auto func = [](void) { std::cout << "Task" << std::endl;};
    server.regist_task(func);

    server.begin();
    server.connect();

    while(true)
    {*/
        /*auto [str_len, msg] = server.receive();
        if(str_len < 0)
        {
            std::cout << "Error received" << std::endl;            
            break;
        }
        else
        {
            if(str_len > 0 )
            {
                std::cout << msg << std::endl;
            }
            else
            {
                server.connect();
            }
        }*/
        /*server.response();        
    }*/
    



    
    cout << "Main Exit" << endl;
    
        

}
