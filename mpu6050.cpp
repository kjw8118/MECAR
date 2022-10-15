#include "mpu6050.h"

#include "gpio_interface.h"

#include "timer.h"

#include <opencv2/opencv.hpp>


/*Vector::Vector(const Vector& origin)
{
    this->x = origin.x;
    this->y = origin.y;
    this->z = origin.z;
}*/

MPU6050_Data::MPU6050_Data(const MPU6050_Data& origin)
{
    this->ax = origin.ax;
    this->ay = origin.ay;
    this->az = origin.az;
    this->tp = origin.tp;
    this->gx = origin.gx;
    this->gy = origin.gy;
    this->gz = origin.gz;
    
}

MPU6050::MPU6050(uint8_t address, uint8_t WHO_RET)
    : address(address), WHO_RET(WHO_RET)
{

}

MPU6050::MPU6050(uint8_t address)
    : address(address)
{
    this->WHO_RET = 0x68;
}

MPU6050::~MPU6050()
{

}

int MPU6050::init()
{
    
    this->i2c.begin(this->address);
    uint8_t who_ret = this->i2c.readReg(this->WHO);
    if(who_ret != this->WHO_RET)
    {
        std::cout << "MPU6050 self test: " << std::hex << (unsigned)who_ret << std::endl;
        return -1;
    }
    
    this->i2c.writeReg(this->PWR_MGMT_1, this->PWR_MGMT_1_DAT1);
    usleep(50000);
    this->i2c.writeReg(this->PWR_MGMT_1, this->PWR_MGMT_1_DAT2);
    usleep(10000);
    this->i2c.writeReg(this->PWR_MGMT_1, this->PWR_MGMT_1_DAT3);

    this->i2c.writeReg(this->CONFIG, this->CONFIG_DAT);
    usleep(10000);
    this->i2c.writeReg(this->SMPLRT_DIV, this->SMPLRT_DIV_100Hz);
    this->i2c.writeReg(this->GYRO_CONFIG, this->GYRO_CONFIG_500DPS);
    this->i2c.writeReg(this->ACCEL_CONFIG, this->ACCEL_CONFIG_2G);
    this->i2c.writeReg(this->ACCEL_CONFIG2, this->ACCEL_CONFIG2_5Hz);
    this->i2c.writeReg(this->INT_PIN_CFG, this->INT_PIN_CFG_DAT);
    this->i2c.writeReg(this->INT_ENABLE, this->INT_ENABLE_DAT);
    
    std::cout << "----------------------- MPU6050 init -----------------------" << std::endl;
    std::cout << std::hex << "CONFIG: " << (unsigned)this->i2c.readReg(this->CONFIG) << std::endl;
    std::cout << std::hex << "SMPLRT_DIV: " << (unsigned)this->i2c.readReg(this->SMPLRT_DIV) << std::endl;
    std::cout << std::hex << "GYRO_CONFIG: " << (unsigned)this->i2c.readReg(this->GYRO_CONFIG) << std::endl;
    std::cout << std::hex << "ACCEL_CONFIG: " << (unsigned)this->i2c.readReg(this->ACCEL_CONFIG) << std::endl;
    std::cout << std::hex << "ACCEL_CONFIG2: " << (unsigned)this->i2c.readReg(this->ACCEL_CONFIG2) << std::endl;
    std::cout << std::hex << "INT_PIN_CFG: " << (unsigned)this->i2c.readReg(this->INT_PIN_CFG) << std::endl;
    std::cout << std::hex << "INT_ENABLE: " << (unsigned)this->i2c.readReg(this->INT_ENABLE) << std::endl;    
    
    for(int i=0; i<3; i++)
    {
        this->accel_offset[i] = 0;
        this->gyro_offset[i] = 0;
    }    
    
    this->ka[0] = 0.05;
    this->ka[1] = 0.05;
    this->ka[2] = 0.05;

    this->getOffset();

    this->ka[0] = 0.08;
    this->ka[1] = 0.03;
    this->ka[2] = 0.01;
    
    std::cout << std::endl;
    
    std::cout << std::dec << "Accel Filtered X: " << this->data_accel[0] << std::endl;
    std::cout << std::dec << "Accel Filtered Y: " << this->data_accel[1] << std::endl;
    std::cout << std::dec << "Accel Filtered Z: " << this->data_accel[2] << std::endl;

    std::cout << std::dec << "Gyro Filtered X: " << this->data_gyro[0] << std::endl;
    std::cout << std::dec << "Gyro Filtered Y: " << this->data_gyro[1] << std::endl;
    std::cout << std::dec << "Gyro Filtered Z: " << this->data_gyro[2] << std::endl;

    std::cout << std::endl;

    std::cout << std::dec << "Accel Offset X: " << this->accel_offset[0] << std::endl;
    std::cout << std::dec << "Accel Offset Y: " << this->accel_offset[1] << std::endl;
    std::cout << std::dec << "Accel Offset Z: " << this->accel_offset[2] << std::endl;

    std::cout << std::dec << "Gyro Offset X: " << this->gyro_offset[0] << std::endl;
    std::cout << std::dec << "Gyro Offset Y: " << this->gyro_offset[1] << std::endl;
    std::cout << std::dec << "Gyro Offset Z: " << this->gyro_offset[2] << std::endl;
    
    std::cout << std::endl;
    

    //std::cout << std::dec << "INT_STATUS: " << this->getStatus() << std::endl;

    std::cout << "------------------------------------------------------------" << std::endl;

    timer.set_t0_ms();

    return 0;
}

void MPU6050::getOffset()
{
    int i = 0, count = 300;
    
    usleep(10000);
    while(++i <= count)
    {
        this->update();
        std::cout << "\rNow MPU6050 getting offset... (" << std::dec << i << "/" << count << ")" << std::flush;
        usleep(10000);
    }
    std::cout << " Done!" << std::endl;
    
    for(int i=0; i<3; i++)
    {
        this->accel_offset[i] = this->data_accel[i];
        this->gyro_offset[i] = this->data_gyro[i];
    }
    this->accel_offset[2] += 9.81;
    
    usleep(10000);
    i = 0;
    while(++i <= count)
    {
        this->update();
        std::cout << "\rNow MPU6050 calibrating... (" << std::dec << i << "/" << count << ")" << std::flush;
        usleep(10000);
    }
    std::cout << " Done!" << std::endl;

}

void MPU6050::enable_slave()
{
    this->i2c.writeReg(this->INT_PIN_CFG, this->INT_PIN_CFG_DAT);
    this->i2c.writeReg(this->INT_ENABLE, this->INT_ENABLE_DAT);
    
}

bool MPU6050::getStatus()
{
    uint8_t status = this->i2c.readReg(this->INT_STATUS);
    if(status & this->INT_STATUS_DAT)            
        return true;    
    else            
        return false;        
}
void MPU6050::update()
{
    uint8_t data_buf[14] = {0,};

    this->i2c.readReg(this->ACCEL_XOUT_H, data_buf, 14);
    
    cv::Vec3d accel_current, gyro_current;
    
    double ts = timer.lead_ms();
    for(int i=0; i<3; i++)
    {
        accel_current[i] = (double)((int16_t)(data_buf[i*2] << 8 | data_buf[i*2 + 1]))/16384*9.81;
        gyro_current[i] = (double)((int16_t)(data_buf[i*2 + 8] << 8 | data_buf[i*2 + 1 + 8]))/65.5;
    }
    accel_current[2] = -accel_current[2];

    for(int i=0; i<3; i++)
    {
        accel_current[i] -= this->accel_offset[i];
        gyro_current[i] -= this->gyro_offset[i];
    }

    

    for(int i=0; i<3; i++)
    {
        
        this->data_accel[i] = accel_current[i] * this->ka[i] + this->data_accel[i] * (1 - this->ka[i]);        
        this->data_gyro[i] = this->tau / (this->tau + ts) * this->data_gyro[i] + this->tau / (this->tau + ts) * (gyro_current[i] - this->data_gyro_raw[i]);
        this->data_gyro_raw[i] = gyro_current[i];
    }    

    this->data_temperature.val[0] = ((double)((int16_t)(data_buf[6] << 8 | data_buf[7]))/333.87 + 21) * this->kt + this->data_temperature.val[0] * (1-this->kt);

    /*std::cout << std::dec << std::fixed;
    for(int i=0; i<3; i++)
    {
        std::cout << "a" << i << ": " << this->data_accel[i] << " ";
    }
    std::cout << std::endl;*/
        
    //std::cout << std::dec << (int16_t)((data_buf[8]&0xff)<<8 | (data_buf[9]&0xff)) << "\t" << (int16_t)((data_buf[10]&0xff)<<8 | (data_buf[11]&0xff)) << "\t" << (int16_t)((data_buf[12]&0xff)<<8 | (data_buf[13]&0xff)) << std::endl;

    /*for(int i=0; i<7; i++)
    {
        std::cout << std::dec << (int16_t)(data_buf[i*2]<<8 | data_buf[i*2+1]) << " ";
    }
    std::cout << std::endl;*/
}


void MPU6050::run()
{
    if(this->init() < 0)
    {
        std::cout << "MPU6050 init failed" << std::endl;
        return;
    }
    else
    {
        std::cout << "MPU6050 init succ" << std::endl;        
    }
    std::cout.precision(3);
    while(this->getStatus())
    {
        this->update();
        std::cout << std::dec << std::fixed << "MPU6050 >> ";
        std::cout << "ax: " << this->data_accel[0] << "\tay: " << this->data_accel[1] << "\taz: " << this->data_accel[2];
        std::cout << "\tgx: " << this->data_gyro[0] << "\tgx: " << this->data_gyro[1] << "\tgz: " << this->data_gyro[2];
        std::cout << "\ttp: " << this->data_temperature << std::endl;
        usleep(10000);
    }
    

}

void MPU6050::getData(cv::Vec3d& accel, cv::Vec3d& gyro)
{
    for(int i=0; i<3; i++)
    {
        accel[i] = this->data_accel[i];
        gyro[i] = this->data_gyro[i];
    }
    
}