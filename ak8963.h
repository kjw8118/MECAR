#ifndef __AK8963_H__
#define __AK8963_H__

#include "gpio_interface.h"

#include <opencv2/opencv.hpp>

#include <unistd.h>
#include <iostream>

class AK8963_Data
{
public:    
    double mx = 0;
    double my = 0;
    double mz = 0;
    AK8963_Data(){};
    ~AK8963_Data(){};
    AK8963_Data(const AK8963_Data& origin);
};

class AK8963
{
private:
    uint8_t address = 0;
    uint8_t WHO_RET = 0;
    enum REGISTER
    {
        WIA = 0x00,        
        INFO = 0x01,
        ST1 = 0x02,
        ST1_DRDY = 0x00,
        HXL = 0x03,
        HXH = 0x04,
        HYL = 0x05,
        HYH = 0x06,
        HZL = 0x07,
        HZH = 0x08,
        ST2 = 0x09,
        CNTL1 = 0x0a,
        CNTL1_CNT = 0x12,
        CNTL2 = 0x0b,
        ASTC = 0x0c,
        TS1 = 0x0d,
        TS2 = 0x0e,
        I2CDIS = 0x0f,
        ASAX = 0x10,
        ASAY = 0x11,
        ASAZ = 0x12,
        RSV = 0x13,
        

    };
    GPIO::Wire i2c;

    AK8963_Data data;

    double km = 1;
    
public:
    AK8963(uint8_t address, uint8_t WHO_RET);
    AK8963(uint8_t address);
    ~AK8963();
    int init();    
    bool getStatus();
    void update(); 
    void getData(cv::Vec3d& magnet);
    void run();
};




#endif