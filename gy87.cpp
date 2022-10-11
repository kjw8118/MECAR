

#include "gy87.h"

#include "mpu6050.h"
#include "hmc5883l.h"

#include <opencv2/opencv.hpp>

GY87_Data::GY87_Data(const GY87_Data& origin)
{
    this->ax = origin.ax;
    this->ay = origin.ay;
    this->az = origin.az;

    this->gx = origin.gx;
    this->gy = origin.gy;
    this->gz = origin.gz;

    this->mx = origin.mx;
    this->my = origin.my;
    this->mz = origin.mz;

}

GY87::GY87()
: mpu6050(0x68, 0x68), hmc5883l(0x1e, 0x48)
{
    
}

GY87::~GY87()
{

}

int GY87::init()
{
    int ret = 0;
    ret += this->mpu6050.init();
    //mpu6050.enable_slave();
    
    
    sleep(1);
    
    ret += this->hmc5883l.init();
    
    

    return ret;
}

bool GY87::getStatus()
{
    return this->mpu6050.getStatus() || this->hmc5883l.getStatus();

}
void GY87::update()
{
    this->mpu6050.update();
    this->hmc5883l.update();
    

    cv::Vec3d accel, gyro, magnet;
    double temperature = 0;
    this->mpu6050.getData(accel, gyro);
    this->hmc5883l.getData(magnet);
    
    this->data.ax = accel[0] * ka + this->data.ax * (1-ka);
    this->data.ay = accel[1] * ka + this->data.ay * (1-ka);
    this->data.az = accel[2] * ka + this->data.az * (1-ka);
    
    this->data.gx = gyro[0] * kg + this->data.gx * (1-kg);
    this->data.gy = gyro[1] * kg + this->data.gy * (1-kg);
    this->data.gz = gyro[2] * kg + this->data.gz * (1-kg);
    
    this->data.mx = magnet[0] * km + this->data.mx * (1-km);
    this->data.my = magnet[1] * km + this->data.my * (1-km);
    this->data.mz = magnet[2] * km + this->data.mz * (1-km);
    
}


void GY87::run()
{
    if(this->init() < 0)
    {
        std::cout << "GY87 init failed" << std::endl;
        return;
    }
    else
    {
        std::cout << "GY87 init succ" << std::endl;        
    }
    
    std::cout.precision(2);
    while(this->getStatus())
    {
        this->update();
        //std::cout << std::fixed << "A: " << this->data.ax << " " << this->data.ay << " " << this->data.az << " G: " << this->data.gx << " " << this->data.gy << " " << this->data.gz << " M: " << this->data.mx << " " << this->data.my << " " << this->data.mz << std::endl;
        usleep(10000);
    }


    std::cout << "GY87 exit" << std::endl;
    

}

void GY87::getData(cv::Vec3d& accel, cv::Vec3d& gyro, cv::Vec3d& magnet)
{
    accel[0] = this->data.ax;
    accel[1] = this->data.ay;
    accel[2] = this->data.az;

    gyro[0] = this->data.gx;
    gyro[1] = this->data.gy;
    gyro[2] = this->data.gz;

    magnet[0] = this->data.mx;
    magnet[1] = this->data.my;
    magnet[2] = this->data.mz;
}