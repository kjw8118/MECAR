#include "imu.h"

#include <thread>
#include "mpu6050.h"

void IMU::init()
{
    
}


void IMU::run()
{
    std::thread mpu6050_thread = std::thread(&MPU6050::run, this->mpu6050);
    
    std::cout.precision(3);
    while(true)
    {
        AccelGyro accgyro;
        this->mpu6050.getData(accgyro);
        std::cout << std::fixed << accgyro.ax << "\t" << accgyro.ay << "\t" << accgyro.az << std::endl;
        usleep(100000);
    }
    mpu6050_thread.join();
    
}

void IMU::getAngle()
{

}