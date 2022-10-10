#include "imu.h"

#include <thread>
#include <future>
#include "mpu6050.h"

void IMU::init()
{
    
}


void IMU::run()
{
    //std::thread mpu6050_thread = std::thread(&MPU6050::run, &(this->mpu6050));
    std::future<void> mpu6050_async = std::async(std::launch::async, std::bind(&MPU6050::run, &(this->mpu6050)));
    //mpu6050_thread.detach();
    
    sleep(1);
    std::cout.precision(3);
    
    while(true)
    {
        this->mpu6050.getData(this->accel, this->gyro);
        std::cout << std::fixed << this->accel.x << "\t" << this->accel.y << "\t" << this->accel.z << std::endl;
        
        usleep(100000);
    }
    //mpu6050_thread.join();
    
}

void IMU::getAngle()
{

}