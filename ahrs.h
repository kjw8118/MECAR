#ifndef __AHRS_H__
#define __AHRS_H__

#include <thread>

#include <opencv2/opencv.hpp>

#include "timer.h"
#include "pointcloud.h"
//#include "mpu6050.h"
//#include "mpu9250.h"
#include "gy87.h"

class AHRS
{
private:
    //MPU6050 mpu6050;
    //MPU9250 mpu9250;
    GY87 gy87;
    
    cv::Vec3d accel;
    cv::Vec3d accel_global;
    cv::Vec3d gyro;
    cv::Vec3d magnet;
    
    cv::Vec3d gyro_raw_past;

    double accel_mag = 0;

    struct
    {
        double accel = 1;
        double accel_mag = 1;
        double alpha = 1.5;
        double beta = 1;
        double gamma = 1;
    }k;
    
    double alpha = 0;
    double beta = 0;
    double gamma = 0;
    
    cv::Vec3d velocity, displacement;
    Timer timer;
    PointCloud pointcloud;
    double ts = 0;

public:
    AHRS(){};
    ~AHRS(){};
    void init();
    
    void run();
    double getAccelMag(cv::Vec3d& accel_raw);    
    void getAngle(cv::Vec3d& accel_raw, double accel_mag_raw, cv::Vec3d& gyro_raw, cv::Vec3d& magnet_raw);
    cv::Vec3d getAccelNet(cv::Vec3d& accel_raw);
    void getVelocity();
    void getDisplacement();
};







#endif