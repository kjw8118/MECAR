#ifndef __MPU9250_H__
#define __MPU9250_H__

#include "gpio_interface.h"

#include "mpu6050.h"
#include "ak8963.h"

#include <opencv2/opencv.hpp>

#include <unistd.h>
#include <iostream>


/*class Vector
{
public:
    double x = 0;
    double y = 0;
    double z = 0;
    Vector(){};
    ~Vector(){};
    Vector(const Vector& origin);
};*/

class MPU9250_Data
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

    MPU9250_Data(){};
    ~MPU9250_Data(){};
    MPU9250_Data(const MPU9250_Data& origin);
};

class MPU9250
{
private:

    MPU9250_Data data;
    MPU6050 mpu6050;
    AK8963 ak8963;

    double ka = 1;
    double kg = 1;
    double km = 1;
    double kt = 1;

    void update();
    int id;
public:
    MPU9250();
    ~MPU9250();
    int init();    
    bool getStatus();
    void getData(cv::Vec3d& accel, cv::Vec3d& gyro);
    void run();
};





#endif