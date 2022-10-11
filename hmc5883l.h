#ifndef __HMC5883L_H__
#define __HMC5883L_H__

#include "gpio_interface.h"

#include <opencv2/opencv.hpp>



class HMC5883L_Data
{
public:    
    double mx = 0;
    double my = 0;
    double mz = 0;
    HMC5883L_Data(){};
    ~HMC5883L_Data(){};
    HMC5883L_Data(const HMC5883L_Data& origin);
};

class HMC5883L
{
private:
    uint8_t address = 0;
    uint8_t WHO_RET = 0;
    enum REGISTER
    {
        //ADR = 0x1e,
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
        IDA = 0x0a,
        IDB = 0x0b,
        IDC = 0x0c,
        

    };
    
    GPIO::Wire i2c;
    HMC5883L_Data data;    
        

public:
    HMC5883L(uint8_t address, uint8_t WHO_RET);
    HMC5883L(uint8_t address);
    ~HMC5883L();
    int init();
    bool getStatus();
    void update();
    void getData(cv::Vec3d& magnet);
    void run();

};

#endif