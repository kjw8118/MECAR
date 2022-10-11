#ifndef __MPU6050_H__
#define __MPU6050_H__

#include "gpio_interface.h"

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

class MPU6050_Data
{
public:
    double ax = 0;
    double ay = 0;
    double az = 0;
    double tp = 0;
    double gx = 0;
    double gy = 0;
    double gz = 0;
    MPU6050_Data(){};
    ~MPU6050_Data(){};
    MPU6050_Data(const MPU6050_Data& origin);
};

class MPU6050
{
private:
    uint8_t address = 0;
    uint8_t WHO_RET = 0;
    enum REGISTER
    {
        MAG = 0x0c,
        WHO = 0x75,
        PWR_MGMT_1 = 0x6b,
        PWR_MGMT_1_DAT1 = 0x80,
        PWR_MGMT_1_DAT2 = 0x00,
        PWR_MGMT_1_DAT3 = 0x01,
        CONFIG = 0x1a,
        CONFIG_DAT = 0x06,
        SMPLRT_DIV = 0x19,
        SMPLRT_DIV_100Hz = 0x09,
        SMPLRT_DIV_1KHz = 0x10,
        GYRO_CONFIG = 0x1b,
        GYRO_CONFIG_250DPS = 0x00,
        GYRO_CONFIG_500DPS = 0x08,
        GYRO_CONFIG_1000DPS = 0x10,
        GYRO_CONFIG_2000DPS = 0x18,
        ACCEL_CONFIG = 0x1c,
        ACCEL_CONFIG_2G = 0x00,
        ACCEL_CONFIG_4G = 0x08,
        ACCEL_CONFIG_8G = 0x10,
        ACCEL_CONFIG_16G = 0x18,
        ACCEL_CONFIG2 = 0x1d,
        ACCEL_CONFIG2_5Hz = 0x0e,
        ACCEL_CONFIG2_RAW = 0x00,        
        INT_PIN_CFG = 0x37,
        INT_PIN_CFG_DAT = 0x22,
        INT_ENABLE = 0x38,
        INT_ENABLE_DAT = 0x01,
        INT_STATUS = 0x3a,
        INT_STATUS_DAT = 0x01,
        ACCEL_XOUT_H = 0x3b,
        ACCEL_XOUT_L = 0x3c,
        ACCEL_YOUT_H = 0x3d,
        ACCEL_YOUT_L = 0x3e,
        ACCEL_ZOUT_H = 0x3F,
        ACCEL_ZOUT_L = 0x40,
        TEMP_OUT_H = 0x41,
        TEMP_OUT_L = 0x42,
        GYRO_XOUT_H = 0x43,
        GYRO_XOUT_L = 0x44,
        GYRO_YOUT_H = 0x45,
        GYRO_YOUT_L = 0x46,
        GYRO_ZOUT_H = 0x47,
        GYRO_ZOUT_L = 0x48,

    };
    GPIO::Wire i2c;

    MPU6050_Data data;

    cv::Vec3d accel_offset, gyro_offset;

    double ka = 1;
    double kg = 1;
    double kt = 1;
    void getOffset();

public:
    MPU6050(uint8_t address, uint8_t WHO_RET);
    MPU6050(uint8_t address);
    ~MPU6050();
    int init();
    void enable_slave();
    bool getStatus();
    void update(); 
    void getData(cv::Vec3d& accel, cv::Vec3d& gyro);
    void run();
};





#endif