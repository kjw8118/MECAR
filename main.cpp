#include <iostream>
#include <thread>
using namespace std;

#include "gpio_interface.h"

//#include "powertrain.h"
//#include "front_camera.h"
//#include "front_radar.h"
//#include "lidar.h"
//#include "speedometer.h"
//#include "joystick.h"

//#include "mpu6050.h"
//#include "hmc5883l.h"
//#include "mpu9250.h"
//#include "gy87.h"
#include "imu.h"

//#include "timer.h"

//#include "communication.h"
#include "unistd.h"


int main()
{
    thread gpio_thread = thread(&GPIO::init_gpio);
    gpio_thread.join();

    //MPU6050 mpu6050;
    //mpu6050.init();

    //thread pt_thread = thread(&Powertrain::run, Powertrain());
    //thread cam_thread = thread(&FrontCamera::run, FrontCamera());
    //thread radar_thread = thread(&FrontRadar::run, FrontRadar());
    //thread lidar_thread = thread(&LiDAR::run, LiDAR());
    //thread speedometer_thread = thread(&Speedometer::run, Speedometer());
    //thread joystick_thread = thread(&Joystick::run, Joystick());
    //thread timer_thread = thread(&Timer::run, Timer());
    //thread mpu6050_thread = thread(&MPU6050::run, MPU6050(0x68, 0x68));
    //thread hmc5883l_thread = thread(&HMC5883L::run, HMC5883L());
    //thread mpu9250_thread = thread(&MPU9250::run, MPU9250());
    //thread gy87_thread = thread(&GY87::run, GY87());
    thread imu_thread = thread(&IMU::run, IMU());

    //pt_thread.join();
    //cam_thread.join();
    //radar_thread.join();
    //lidar_thread.join();
    //speedometer_thread.join();
    //joystick_thread.join();
    //timer_thread.join();

    //mpu6050_thread.join();
    //hmc5883l_thread.join();
    //mpu9250_thread.join();
    //gy87_thread.join();
    imu_thread.join();

    //Timer timer;
    //timer.run();

    //Communication::Client<Communication::TCP_Client> client;
    
    //client.connect("192.168.0.28", 8118);

    //std::cout << "Now enter loop" << std::endl;
    //while(true)
    //{
        //client.request();
        //sleep(1);
    //}


    
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

    while(true){};
    
        

}
