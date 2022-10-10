#ifndef __IMU_H__
#define __IMU_H__

#include <thread>

#include "mpu6050.h"


class IMU
{
private:
    MPU6050 mpu6050;
    Vector accel;
    Vector gyro;
public:
    IMU(){};
    ~IMU(){};
    void init();
    
    void run();
    void getAngle();
};







#endif