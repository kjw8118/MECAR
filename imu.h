#ifndef __IMU_H__
#define __IMU_H__

#include <thread>

#include <opencv2/opencv.hpp>

#include "mpu6050.h"


class IMU
{
private:
    MPU6050 mpu6050;
    
    cv::Vec3d accel;
    cv::Vec3d gyro;
    double accel_mag = 0;

    struct
    {
        double accel = 0.1;
        double accel_mag = 0.1;
        double alpha = 0.1;
        double beta = 0.1;
    }k;
    
    double alpha = 0;
    double beta = 0;

public:
    IMU(){};
    ~IMU(){};
    void init();
    
    void run();
    double getAccelMag(cv::Vec3d& accel_raw);
    void getAngle(cv::Vec3d& accel_raw, double accel_mag_raw);
    cv::Vec3d getAccelNet(cv::Vec3d& accel_raw);
};







#endif