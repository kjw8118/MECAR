

#include "hmc5883l.h"

#include "gpio_interface.h"

#include <unistd.h>
#include <iostream>

#include <opencv2/opencv.hpp>

HMC5883L_Data::HMC5883L_Data(const HMC5883L_Data& origin)
{
    this->mx = origin.mx;
    this->my = origin.my;
    this->mz = origin.mz;
}

HMC5883L::HMC5883L(uint8_t address, uint8_t WHO_RET)
:   address(address), WHO_RET(WHO_RET)
{

}

HMC5883L::HMC5883L(uint8_t address)
:   address(address)
{
    this->WHO_RET = 0;
}
HMC5883L::~HMC5883L()
{

}
int HMC5883L::init()
{
    this->i2c.begin(this->address);
    usleep(50000);

    uint8_t who_ret = this->i2c.readReg(this->IDA);
    
    this->i2c.writeReg(this->CFG_A, 0x70); usleep(100000);
    this->i2c.writeReg(this->CFG_B, 0xA0); usleep(100000);
    this->i2c.writeReg(this->MOD, 0x00); usleep(100000);

    if(who_ret != this->WHO_RET)
    {
        std::cout << "HMC5883L self test: " << std::hex << (unsigned)who_ret << std::endl;
        
        for(int i=0; i<13; i++)
        {
            std::cout << std::hex << i << " -> " << (unsigned)this->i2c.readReg(i) << std::endl;
        }
        return -1;
    }
    std::cout << "---------------------- HMC5883L init ----------------------" << std::endl;
    std::cout << std::hex << "CFG_A: " << (unsigned)this->i2c.readReg(this->CFG_A) << std::endl;
    std::cout << std::hex << "CFG_B: " << (unsigned)this->i2c.readReg(this->CFG_B) << std::endl;
    std::cout << std::hex << "MODE: " << (unsigned)this->i2c.readReg(this->MOD) << std::endl;
    std::cout << std::hex << "STATUS: " << (unsigned)this->i2c.readReg(this->STA) << std::endl;
    std::cout << std::hex << "ID A: " << (unsigned)this->i2c.readReg(this->IDA) << std::endl;
    std::cout << std::hex << "ID B: " << (unsigned)this->i2c.readReg(this->IDB) << std::endl;
    std::cout << std::hex << "ID C: " << (unsigned)this->i2c.readReg(this->IDC) << std::endl;
    /*for(int i=0; i<3; i++)
    {
        uint8_t msb = this->i2c.readReg(i*2+3);
        uint8_t lsb = this->i2c.readReg(i*2+4);
        std::cout << std::hex << i << " -> " << (uint16_t)(msb<<8 | lsb) << std::endl;
    }*/
    std::cout << "-----------------------------------------------------------" << std::endl;
    return 0;
}

bool HMC5883L::getStatus()
{
    uint8_t status = this->i2c.readReg(this->STA);
    if(status & 1)            
        return true;    
    else            
        return false;        
}
void HMC5883L::update()
{
    uint8_t buf[6] = {0,};
    
    this->i2c.readReg(this->XOUT_H, buf, 6);

    //std::cout << std::hex << (unsigned)((buf[0]&0xff)<<8 | (buf[1]&0xff)) << "\t" << (unsigned)((buf[2]&0xff)<<8 | (buf[3]&0xff)) << "\t" << (unsigned)((buf[4]&0xff)<<8 | (buf[5]&0xff)) << std::endl;


    this->data.mx = (double)((int16_t)(buf[0]<<8 | buf[1]))/230;
    this->data.my = (double)((int16_t)(buf[2]<<8 | buf[3]))/230;
    this->data.mz = (double)((int16_t)(buf[4]<<8 | buf[5]))/230;
    
}
void HMC5883L::run()
{
    this->init();
    std::cout.precision(3);

    while(this->getStatus())
    {
        this->update();

        //std::cout << std::fixed << this->data.mx << "\t" << this->data.my << "\t" << this->data.mz << std::endl;

        usleep(10000);
        
    }

    std::cout << "HMC5883L exit" << std::endl;
}

void HMC5883L::getData(cv::Vec3d& magnet)
{
    magnet[0] = this->data.mx;
    magnet[1] = this->data.my;
    magnet[2] = this->data.mz;
    
}
