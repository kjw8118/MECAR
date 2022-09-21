#include <iostream>
#include <thread>
using namespace std;

#include "powertrain.h"
#include "front_camera.h"
#include "front_radar.h"
#include "lidar.h"


int main()
{        
    //thread pt_thread = thread(&Powertrain::run, Powertrain());
    //thread cam_thread = thread(&FrontCamera::run, FrontCamera());
    //thread radar_thread = thread(&FrontRadar::run, FrontRadar());
    thread lidar_thread = thread(&LiDAR::run, LiDAR());

    //pt_thread.join();
    //cam_thread.join();
    //radar_thread.join();
    lidar_thread.join();

    cout << "Main Exit" << endl;
    
        

}
