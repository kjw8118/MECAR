#ifndef __HMC5883L_H__
#define __HMC5883L_H__

#include "gpio_interface.h"

#include <opencv2/opencv.hpp>

class HMC5883L
{
private:
    enum REGISTER
    {
        ADR = 0x1e,
        WRT = 0x3c,
        RED = 0x3d,
        CFG_A = 0x00,
        CFG_B = 0x01,
        MOD = 0x02,
        XOUT_H = 0x03,
        XOUT_L = 0x04,
        YOUT_H = 0x05,
        YOUT_L = 0x06,
        ZOUT_H = 0x07,
        ZOUT_L = 0x08,        
        STA = 0x09,
        IDA = 0x10,
        IDB = 0x11,
        IDC = 0x12,
        

    };
    
    GPIO::Wire i2c;

    cv::Vec3d compass;    

public:
    HMC5883L(/* args */) {};
    ~HMC5883L() {};
    void init();
    void update();
    void run();

};

#endif