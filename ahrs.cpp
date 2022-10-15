#include "ahrs.h"

#include <cmath>

#include <thread>
#include <future>
#include <opencv2/opencv.hpp>
//#include "mpu6050.h"
//#include "mpu9250.h"
#include "gy87.h"

#include "linearalgebra.h"

void AHRS::init()
{
    this->timer.set_t0_ms();

    this->velocity.zeros();
    this->displacement.zeros();

}


void AHRS::run()
{
    
    //std::thread mpu6050_thread = std::thread(&MPU6050::run, &(this->mpu6050));
    //std::future<void> mpu9250_async = std::async(std::launch::async, std::bind(&MPU9250::run, &(this->mpu9250)));
    std::future<void> gy87_async = std::async(std::launch::async, std::bind(&GY87::run, &(this->gy87)));
    //mpu6050_thread.detach();
    
    
    while(!this->gy87.isReady())
    {
        usleep(100000);
    };
    sleep(1);
    this->init();

    std::cout.precision(2);    
    while(true)
    {
        
        cv::Vec3d accel_raw, gyro_raw, magnet_raw;
        this->gy87.getData(accel_raw, this->gyro, this->magnet);
        this->ts = this->timer.lead_ms();
        double accel_mag_raw = this->getAccelMag(accel_raw);
        this->getAngle(accel_raw, accel_mag_raw, gyro_raw);
        this->getAccelNet(accel_raw);
        this->getVelocity();
        this->getDisplacement();
        
        std::cout << std::fixed << "ts: " << this->ts;
        std::cout << ", |a|: " << this->accel_mag;
        std::cout << ", alpha: ";
        if(this->alpha >= 0)
            std::cout << " ";
        std::cout << this->alpha*180/3.141592;
        std::cout << ", beta: ";
        if(this->beta >=0)
            std::cout << " ";
        std::cout << this->beta*180/3.141592;

        for(int i=0; i<3; i++)
        {
            std::cout << ", a" << i << ": ";
            if(this->accel[i] >= 0)
                std::cout << " ";
            std::cout << this->accel[i];
        }
        /*for(int i=0; i<3; i++)
        {
            std::cout << ", g" << i << ": ";
            if(this->gyro[i] >= 0)
                std::cout << " ";
            std::cout << this->gyro[i];
        }
        for(int i=0; i<3; i++)
        {
            std::cout << ", m" << i << ": ";
            if(this->magnet[i] >= 0)
                std::cout << " ";
            std::cout << this->magnet[i];
        }*/
        for(int i=0; i<3; i++)
        {
            std::cout << ", v" << i << ": ";
            if(this->velocity[i] >= 0)
                std::cout << " ";
            std::cout << this->velocity[i];
        }
        for(int i=0; i<3; i++)
        {
            std::cout << ", d" << i << ": ";
            if(this->displacement[i] >= 0)
                std::cout << " ";
            std::cout << this->displacement[i];
        }
        std::cout << std::endl;

        //std::cout << std::fixed << accel_raw[0] << "\t" << accel_raw[1] << "\t" << accel_raw[2] << std::endl;
        
        usleep(10000);
    }
    //mpu6050_thread.join();
    
}
double AHRS::getAccelMag(cv::Vec3d& accel_raw)
{
    double accel_mag_raw = std::sqrt(std::pow(accel_raw[0], 2) + std::pow(accel_raw[1], 2) + std::pow(accel_raw[2], 2));
    this->accel_mag = (accel_mag_raw - 9.81)*this->k.accel_mag + this->accel_mag*(1-this->k.accel_mag);
    
    return accel_mag_raw;

}
void AHRS::getAngle(cv::Vec3d& accel_raw, double accel_mag_raw, cv::Vec3d& gyro_raw)
{    
    this->alpha += (gyro_raw[0] - (this->alpha - std::atan2(accel_raw[1], -accel_raw[2]))*this->k.alpha) * this->ts;
    this->beta += (gyro_raw[1] - (this->beta - std::atan2(-accel_raw[0], accel_mag_raw))*this->k.beta * this->ts);
}
cv::Vec3d AHRS::getAccelNet(cv::Vec3d& accel_raw)
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

void AHRS::getVelocity()
{

    for(int i=0; i<3; i++)
    {
        this->velocity[i] += this->accel[i] * this->ts/1000;
    }
    
}

void AHRS::getDisplacement()
{
    for(int i=0; i<3; i++)
    {
        this->displacement[i] += this->velocity[i] * this->ts/1000;
    }    


    
}