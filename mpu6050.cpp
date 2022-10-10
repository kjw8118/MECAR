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

int MPU6050::init()
{
    this->i2c.begin(this->ADR);
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
    this->i2c.writeReg(this->GYRO_CONFIG, this->GYRO_CONFIG_250DPS);
    this->i2c.writeReg(this->ACCEL_CONFIG, this->ACCEL_CONFIG_2G);
    this->i2c.writeReg(this->ACCEL_CONFIG2, this->ACCEL_CONFIG2_5Hz);
    this->i2c.writeReg(this->INT_PIN_CFG, this->INT_PIN_CFG_DAT);
    this->i2c.writeReg(this->INT_ENABLE, this->INT_ENABLE_DAT);
    

    return 0;
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
    
    this->data.ax = ((double)((int16_t)(data_buf[0] << 8 | data_buf[1]))/16384*9.81) * ka + this->data.ax * (1-ka);
    this->data.ay = ((double)((int16_t)(data_buf[2] << 8 | data_buf[3]))/16384*9.81) * ka + this->data.ay * (1-ka);
    this->data.az = -((double)((int16_t)(data_buf[4] << 8 | data_buf[5]))/16384*9.81) * ka + this->data.az * (1-ka);
    this->data.tp = ((double)((int16_t)(data_buf[6] << 8 | data_buf[7]))/333.87 + 21)  * kt + this->data.tp * (1-kt);
    this->data.gx = ((double)((int16_t)(data_buf[8] << 8 | data_buf[9]))/131) * kg + this->data.gx * (1-kg);
    this->data.gy = ((double)((int16_t)(data_buf[10] << 8 | data_buf[11]))/131) * kg + this->data.gy * (1-kg);
    this->data.gz = ((double)((int16_t)(data_buf[12] << 8 | data_buf[13]))/131) * kg + this->data.gz * (1-kg);

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