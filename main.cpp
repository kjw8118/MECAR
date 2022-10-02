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

    //pt_thread.join();
    //cam_thread.join();
    //radar_thread.join();
    //speedometer_thread.join();
    //joystick_thread.join();

    Communication::TCP_Client client;

    client.connect();

    while(true)
    {
        client.receive();
        sleep(1);
    }
    



    
    cout << "Main Exit" << endl;
    
        

}
