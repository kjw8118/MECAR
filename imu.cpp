#include "imu.h"

#include <cmath>

#include <thread>
#include <future>
#include <opencv2/opencv.hpp>
//#include "mpu6050.h"
//#include "mpu9250.h"
#include "gy87.h"

#include "linearalgebra.h"

void IMU::init()
{
    
}


void IMU::run()
{
    //std::thread mpu6050_thread = std::thread(&MPU6050::run, &(this->mpu6050));
    //std::future<void> mpu9250_async = std::async(std::launch::async, std::bind(&MPU9250::run, &(this->mpu9250)));
    std::future<void> gy87_async = std::async(std::launch::async, std::bind(&GY87::run, &(this->gy87)));
    //mpu6050_thread.detach();
    
    sleep(1);
    std::cout.precision(2);
    
    while(true)
    {
        cv::Vec3d accel_raw, gyro_raw, magnet_raw;
        this->gy87.getData(accel_raw, this->gyro, this->magnet);
        double accel_mag_raw = this->getAccelMag(accel_raw);
        this->getAngle(accel_raw, accel_mag_raw);
        this->getAccelNet(accel_raw);
        
        std::cout << std::fixed << accel_mag_raw << "\t" << this->alpha*180/3.141592 << "\t" << this->beta*180/3.141592 << "\t";
        for(int i=0; i<3; i++)
        {
            std::cout << this->accel[i] << "\t";
        }
        for(int i=0; i<3; i++)
        {
            std::cout << this->gyro[i] << "\t";
        }
        for(int i=0; i<3; i++)
        {
            std::cout << this->magnet[i] << "\t";
        }
        std::cout << std::endl;

        //std::cout << std::fixed << accel_raw[0] << "\t" << accel_raw[1] << "\t" << accel_raw[2] << std::endl;
        
        usleep(10000);
    }
    //mpu6050_thread.join();
    
}
double IMU::getAccelMag(cv::Vec3d& accel_raw)
{
    double accel_mag_raw = std::sqrt(std::pow(accel_raw[0], 2) + std::pow(accel_raw[1], 2) + std::pow(accel_raw[2], 2));
    this->accel_mag = (accel_mag_raw - 9.81)*this->k.accel_mag + this->accel_mag*(1-this->k.accel_mag);
    
    return accel_mag_raw;

}
void IMU::getAngle(cv::Vec3d& accel_raw, double accel_mag_raw)
{
    this->alpha = std::atan2(accel_raw[1], -accel_raw[2])*this->k.alpha + this->alpha * (1-this->k.alpha);
    this->beta = std::atan2(-accel_raw[0], accel_mag_raw)*this->k.beta + this->beta * (1-this->k.beta);
}
cv::Vec3d IMU::getAccelNet(cv::Vec3d& accel_raw)
{
    cv::Vec3d accel_net;
    accel_net = LinearAlgebra::RotateX(accel_raw, -this->alpha);
    accel_net = LinearAlgebra::RotateY(accel_net, -this->beta);
    accel_net[2] += 9.81;
    accel_net = LinearAlgebra::RotateX(accel_net, this->beta);
    accel_net = LinearAlgebra::RotateY(accel_net, this->alpha);
    
    this->accel[0] = accel_net[0]*this->k.accel + this->accel[0]*(1-this->k.accel);
    this->accel[1] = accel_net[1]*this->k.accel + this->accel[1]*(1-this->k.accel);
    this->accel[2] = accel_net[2]*this->k.accel + this->accel[2]*(1-this->k.accel);
    return accel_net;
}