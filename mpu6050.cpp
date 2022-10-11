#include "mpu6050.h"

#include "gpio_interface.h"

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
    

    this->accel_offset[0] = 0;
    this->accel_offset[1] = 0;
    this->accel_offset[2] = 0;

    this->gyro_offset[0] = 0;
    this->gyro_offset[1] = 0;
    this->gyro_offset[2] = 0;

    this->getOffset();

    std::cout << std::dec << "Accel Offset X: " << this->accel_offset[0] << std::endl;
    std::cout << std::dec << "Accel Offset Y: " << this->accel_offset[1] << std::endl;
    std::cout << std::dec << "Accel Offset Z: " << this->accel_offset[2] << std::endl;

    std::cout << std::dec << "Gyro Offset X: " << this->gyro_offset[0] << std::endl;
    std::cout << std::dec << "Gyro Offset Y: " << this->gyro_offset[1] << std::endl;
    std::cout << std::dec << "Gyro Offset Z: " << this->gyro_offset[2] << std::endl;

    std::cout << "------------------------------------------------------------" << std::endl;
    return 0;
}

void MPU6050::getOffset()
{
    int i = 0;
    while(++i < 100)
    {
        this->update();
        usleep(10000);
    }
    
    this->accel_offset[0] = this->data.ax;
    this->accel_offset[1] = this->data.ay;
    this->accel_offset[2] = this->data.az + 9.81;

    this->gyro_offset[0] = this->data.gx;
    this->gyro_offset[1] = this->data.gy;
    this->gyro_offset[2] = this->data.gz;


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
    
    this->data.ax = ((double)((int16_t)(data_buf[0] << 8 | data_buf[1]))/16384*9.81 - this->accel_offset[0]) * ka + this->data.ax * (1-ka);
    this->data.ay = ((double)((int16_t)(data_buf[2] << 8 | data_buf[3]))/16384*9.81 - this->accel_offset[1]) * ka + this->data.ay * (1-ka);
    this->data.az = (-(double)((int16_t)(data_buf[4] << 8 | data_buf[5]))/16384*9.81 - this->accel_offset[2]) * ka + this->data.az * (1-ka);
    this->data.tp = ((double)((int16_t)(data_buf[6] << 8 | data_buf[7]))/333.87 + 21)  * kt + this->data.tp * (1-kt);
    this->data.gx = ((double)((int16_t)(data_buf[8] << 8 | data_buf[9]))/65.5 - this->gyro_offset[0]) * kg + this->data.gx * (1-kg);
    this->data.gy = ((double)((int16_t)(data_buf[10] << 8 | data_buf[11]))/65.5 - this->gyro_offset[1]) * kg + this->data.gy * (1-kg);
    this->data.gz = ((double)((int16_t)(data_buf[12] << 8 | data_buf[13]))/65.5 - this->gyro_offset[2]) * kg + this->data.gz * (1-kg);

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
        
    while(this->getStatus())
    {
        this->update();

        usleep(10000);
    }
    

}

void MPU6050::getData(cv::Vec3d& accel, cv::Vec3d& gyro)
{
    accel[0] = this->data.ax;
    accel[1] = this->data.ay;
    accel[2] = this->data.az;
    
    gyro[0] = this->data.gx;
    gyro[1] = this->data.gy;
    gyro[2] = this->data.gz;
    
}