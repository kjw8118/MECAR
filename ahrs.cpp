#include "ahrs.h"

#include <cmath>

#include <thread>
#include <future>
#include <opencv2/opencv.hpp>

#include "pointcloud.h"

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
    int height = 500;
    int width = 500;
    
    //cv::Mat canvas = cv::Mat(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
    
    //std::thread mpu6050_thread = std::thread(&MPU6050::run, &(this->mpu6050));
    //std::future<void> mpu9250_async = std::async(std::launch::async, std::bind(&MPU9250::run, &(this->mpu9250)));
    std::future<void> gy87_async = std::async(std::launch::async, std::bind(&GY87::run, &(this->gy87)));
    
    //mpu6050_thread.detach();
    
    
    while(!this->gy87.isReady())
    {
        usleep(100000);
    };
    sleep(1);
    //std::future<void> pointcloud_async = std::async(std::launch::async, std::bind(&PointCloud::plot, &(this->pointcloud)));
    std::thread pointcloud_thread = std::thread(&PointCloud::plot, &(this->pointcloud));
    //pointcloud_thread.detach();
    
    this->init();

    std::cout.precision(2);    
    while(true)
    {

        //cv::Mat axes;
        //canvas.copyTo(axes);   
        cv::Vec3d accel_raw, gyro_raw, magnet_raw;
        this->ts = this->timer.wait_until_with_lead_ms(10);
        this->gy87.getData(accel_raw, this->gyro, this->magnet);
        
        double accel_mag_raw = this->getAccelMag(accel_raw);
        this->getAngle(accel_raw, accel_mag_raw, gyro_raw, this->magnet);
        this->getAccelNet(accel_raw);
        this->getVelocity();
        this->getDisplacement();
        
        //axes.at<cv::Vec3b>((int)(this->accel_global[2] + height/2), (int)(this->accel_global[0] + width/2)) = cv::Vec3b({0,0,0});
        this->pointcloud.putPoint(this->displacement[0]*5000, this->displacement[1]*5000);
        
#define DEBUG TRUE
#ifdef DEBUG
        std::cout << std::fixed << "ts: " << this->ts;
        /*std::cout << ", |a|: " << this->accel_mag;*/
        std::cout << ", alpha: ";
        if(this->alpha >= 0)
            std::cout << " ";
        std::cout << this->alpha*180/3.141592;
        std::cout << ", beta: ";
        if(this->beta >=0)
            std::cout << " ";
        std::cout << this->beta*180/3.141592;
        std::cout << ", gamma: ";
        if(this->gamma >=0)
            std::cout << " ";
        std::cout << this->gamma*180/3.141592;

        /*for(int i=0; i<3; i++)
        {
            std::cout << ", a" << i << ": ";
            if(this->accel[i] >= 0)
                std::cout << " ";
            std::cout << this->accel[i];
        }
        for(int i=0; i<3; i++)
        {
            std::cout << ", g" << i << ": ";
            if(this->gyro[i] >= 0)
                std::cout << " ";
            std::cout << this->gyro[i];
        }
        std::cout << ", |m|: " << std::sqrt(std::pow(this->magnet[0], 2) + std::pow(this->magnet[1], 2) + std::pow(this->magnet[2], 2));
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
            std::cout << this->displacement[i]*1000;
        }

        std::cout << std::endl;

        //std::cout << std::fixed << accel_raw[0] << "\t" << accel_raw[1] << "\t" << accel_raw[2] << std::endl;
#endif    
        //usleep(10000);
        //this->timer.wait_until_ms(10);
    }
    //mpu6050_thread.join();
    
}
double AHRS::getAccelMag(cv::Vec3d& accel_raw)
{
    double accel_mag_raw = std::sqrt(std::pow(accel_raw[0], 2) + std::pow(accel_raw[1], 2) + std::pow(accel_raw[2], 2));
    this->accel_mag = (accel_mag_raw - 9.81)*this->k.accel_mag + this->accel_mag*(1-this->k.accel_mag);
    
    return accel_mag_raw;

}
void AHRS::getAngle(cv::Vec3d& accel_raw, double accel_mag_raw, cv::Vec3d& gyro_raw, cv::Vec3d& magnet_raw)
{   
    
    this->alpha += (gyro_raw[0] - (this->alpha - std::atan2(accel_raw[1], -accel_raw[2]))*this->k.alpha * 2 * 3.141592 * this->ts/1000);
    //this->alpha += gyro_raw[0]* this->ts - (this->alpha - alpha_a)*this->k.alpha * this->ts ;
    if(this->alpha > 3.141592)
        this->alpha -= 3.141592;
    else if(this->alpha < -3.141592)
        this->alpha += 3.141592;

    this->beta += (gyro_raw[1] - (this->beta - std::atan2(-accel_raw[0], accel_mag_raw))*this->k.beta * 2 * 3.141592 * this->ts/1000);    
    if(this->beta > 3.141592)
        this->beta -= 3.141592;
    else if(this->beta < -3.141592)
        this->beta += 3.141592;

    this->gamma += (gyro_raw[2] - (this->gamma - std::atan2(magnet_raw[2], magnet_raw[0]))*this->k.gamma * 2 * 3.141592 * this->ts/1000);
    if(this->gamma > 3.141592)
        this->gamma -= 3.141592;
    else if(this->gamma < -3.141592)
        this->gamma += 3.141592;
}
cv::Vec3d AHRS::getAccelNet(cv::Vec3d& accel_raw)
{
    cv::Vec3d accel_net;
    accel_net = LinearAlgebra::RotateX(accel_raw, -this->alpha);
    accel_net = LinearAlgebra::RotateY(accel_net, -this->beta);
    accel_net[2] += 9.81;
    accel_net = LinearAlgebra::RotateX(accel_net, this->beta);
    accel_net = LinearAlgebra::RotateY(accel_net, this->alpha);
    
    this->accel_global = LinearAlgebra::RotateZ(accel_net, this->gamma)*this->k.accel + this->accel*(1-this->k.accel);

    this->accel = accel_net*this->k.accel + this->accel*(1-this->k.accel);

    //this->accel[0] = accel_net[0]*this->k.accel + this->accel[0]*(1-this->k.accel);
    //this->accel[1] = accel_net[1]*this->k.accel + this->accel[1]*(1-this->k.accel);
    //this->accel[2] = accel_net[2]*this->k.accel + this->accel[2]*(1-this->k.accel);
    return accel_net;
}

void AHRS::getVelocity()
{

    this->velocity += this->accel_global * this->ts/1000;
    /*for(int i=0; i<3; i++)
    {
        this->velocity[i] += this->accel[i] * this->ts/1000;
    }*/
    
}

void AHRS::getDisplacement()
{
    this->displacement += this->velocity * this->ts/1000;
    
    /*for(int i=0; i<3; i++)
    {
        this->displacement[i] += this->velocity[i] * this->ts/1000;
    }*/    


    
}