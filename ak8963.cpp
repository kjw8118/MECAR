#include "ak8963.h"

#include "gpio_interface.h"

#include <opencv2/opencv.hpp>


AK8963_Data::AK8963_Data(const AK8963_Data& origin)
{
    this->mx = origin.mx;
    this->my = origin.my;
    this->mz = origin.mz;
}

AK8963::AK8963(uint8_t address, uint8_t WHO_RET)
    : address(address), WHO_RET(WHO_RET)
{

}
AK8963::AK8963(uint8_t address)
    : address(address)
{
     this->WHO_RET = 0x48;
}
AK8963::~AK8963()
{

}
int AK8963::init()
{
    this->i2c.begin(this->address);
    uint8_t who_ret = this->i2c.readReg(this->WIA);
    if(who_ret != this->WHO_RET)
    {
        std::cout << "AK8963 self test: " << std::hex << (unsigned)who_ret << std::endl;
        return -1;
    }
    this->i2c.writeReg(this->CNTL1, this->CNTL1_CNT);
    usleep(50000);

    return 0;
}
bool AK8963::getStatus()
{
    uint8_t status = this->i2c.readReg(this->ST1);
    if(status & this->ST1_DRDY)            
        return true;    
    else            
        return false;
}

void AK8963::update()
{
    uint8_t buf[7] = {0,};
    this->i2c.readReg(this->HXL, buf, 7);
    this->data.mx = -(double)((int16_t)(buf[0]|buf[1]<<8))/4912;
    this->data.my = -(double)((int16_t)(buf[2]|buf[3]<<8))/4912;
    this->data.mz = -(double)((int16_t)(buf[4]|buf[5]<<8))/4912;
}


void AK8963::getData(cv::Vec3d& magnet)
{
    magnet[0] = this->data.mx;
    magnet[1] = this->data.my;
    magnet[2] = this->data.mz;
    
}

void AK8963::run()
{
    this->init();

    while(this->getStatus())
    {
        this->update();
        usleep(10000);
    }
}
