#include "mpu9250.h"

#include "gpio_interface.h"

#include <opencv2/opencv.hpp>


/*Vector::Vector(const Vector& origin)
{
    this->x = origin.x;
    this->y = origin.y;
    this->z = origin.z;
}*/

MPU9250_Data::MPU9250_Data(const MPU9250_Data& origin)
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

MPU9250::MPU9250()
: mpu6050(0x68, 0x71), ak8963(0x0c, 0x48)
{
    
}

MPU9250::~MPU9250()
{

}

int MPU9250::init()
{
    int ret;
    ret = this->mpu6050.init();
    //mpu6050.enable_slave();
    if(ret < 0)
    {
        std::cout << "MPU6050 init fail" << std::endl;
    }
    
    sleep(1);

    ret = this->ak8963.init();

    

    return 0;
}

bool MPU9250::getStatus()
{
    return this->mpu6050.getStatus() || this->ak8963.getStatus();

}
void MPU9250::update()
{
    this->mpu6050.update();
    this->ak8963.update();
    

    cv::Vec3d accel, gyro, magnet;
    double temperature = 0;
    this->mpu6050.getData(accel, gyro);
    this->ak8963.getData(magnet);
    
    this->data.ax = accel[0] * ka + this->data.ax * (1-ka);
    this->data.ay = accel[1] * ka + this->data.ay * (1-ka);
    this->data.az = -accel[2] * ka + this->data.az * (1-ka);
    
    this->data.gx = gyro[0] * kg + this->data.gx * (1-kg);
    this->data.gy = gyro[1] * kg + this->data.gy * (1-kg);
    this->data.gz = gyro[2] * kg + this->data.gz * (1-kg);
    
    this->data.mx = magnet[0] * km + this->data.mx * (1-km);
    this->data.mx = magnet[1] * km + this->data.my * (1-km);
    this->data.mx = magnet[2] * km + this->data.mz * (1-km);
    
}


void MPU9250::run()
{
    if(this->init() < 0)
    {
        std::cout << "MPU9250 init failed" << std::endl;
        return;
    }
    else
    {
        std::cout << "MPU9250 init succ" << std::endl;        
    }
    
    std::cout.precision(3);
    while(this->getStatus())
    {
        this->update();
        //std::cout << std::fixed << this->data.ax << "\t" << this->data.ay << "\t" << this->data.az << this->data.mx << "\t" << this->data.my << "\t" << this->data.mz << std::endl;
        usleep(10000);
    }
    

}

void MPU9250::getData(cv::Vec3d& accel, cv::Vec3d& gyro)
{
    accel[0] = this->data.ax;
    accel[1] = this->data.ay;
    accel[2] = this->data.az;

    gyro[0] = this->data.gx;
    gyro[1] = this->data.gy;
    gyro[2] = this->data.gz;
    
}