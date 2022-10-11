#ifndef __GY87_H__
#define __GY87_H__

#include "mpu6050.h"
#include "hmc5883l.h"

#include <opencv2/opencv.hpp>


class GY87_Data
{
public:
    double ax = 0;
    double ay = 0;
    double az = 0;
    
    double gx = 0;
    double gy = 0;
    double gz = 0;

    double mx = 0;
    double my = 0;
    double mz = 0;

    GY87_Data(){};
    ~GY87_Data(){};
    GY87_Data(const GY87_Data& origin);
};

class GY87
{
private:
    GY87_Data data;
    MPU6050 mpu6050;
    HMC5883L hmc5883l;

    double ka = 1;
    double kg = 1;
    double km = 1;
    
    void update();
public:
    GY87(/* args */);
    ~GY87();
    int init();    
    bool getStatus();    
    void run();
    void getData(cv::Vec3d& accel, cv::Vec3d& gyro, cv::Vec3d& magnet);
};



#endif