

#include "hmc5883l.h"

#include "gpio_interface.h"

#include <unistd.h>
#include <iostream>

#include <opencv2/opencv.hpp>

void HMC5883L::init()
{
    this->i2c.begin(this->ADR);
    usleep(50000);
    this->i2c.writeReg(0x00, 0x70); usleep(100000);
    this->i2c.writeReg(0x01, 0xA0); usleep(100000);
    this->i2c.writeReg(0x02, 0x00); usleep(100000);
    //std::cout << std::hex << "CFG_A " << (unsigned)this->i2c.readReg(this->CFG_A) << std::endl;
    //std::cout << std::hex << "CFG_B " << (unsigned)this->i2c.readReg(this->CFG_B) << std::endl;
    //std::cout << std::hex << "MODE " << (unsigned)this->i2c.readReg(this->MOD) << std::endl;
    /*
    //uint8_t ida = this->i2c.readReg(this->IDA); usleep(50000);
    //uint8_t idb = this->i2c.readReg(this->IDB); usleep(50000);
    //uint8_t idc = this->i2c.readReg(this->IDC); usleep(50000);
    //std::cout << (unsigned)ida << "_" << (unsigned)idb << "_" << (unsigned)idc << std::endl;

    //uint8_t cfg_reg_a_buf[3] = {this->WRT, this->CFG_A, 0};
    uint8_t cfg_reg_a = 0;
    cfg_reg_a |= 0<<5; // Sample movemean
    cfg_reg_a |= 4<<2; // 15Hz (Default)
    cfg_reg_a |= 0; // Measurement mode default
    //cfg_reg_a_buf[2] = cfg_reg_a;
    //this->i2c.writeByte(this->WRT); usleep(10000);    
    this->i2c.writeReg(this->CFG_A, cfg_reg_a); usleep(100000);    
    //std::cout << std::hex << "CFG_A " << (unsigned)cfg_reg_a << " -> " << (unsigned)this->i2c.readReg(this->CFG_A) << std::endl;
    //this->i2c.writeBytes(cfg_reg_a_buf, 3); usleep(50000);

    //uint8_t cfg_reg_b_buf[3] = {this->WRT, this->CFG_B, 0};
    uint8_t cfg_reg_b = 0;
    cfg_reg_b |= 1<<5; // Gain 1.3Ga (Default) -> factor for 1090
    //cfg_reg_b_buf[2] = cfg_reg_b;
    //this->i2c.writeByte(this->WRT); usleep(10000);
    this->i2c.writeReg(this->CFG_A, cfg_reg_b); usleep(100000);
    //std::cout << std::hex << "CFG_B " << (unsigned)cfg_reg_b << " -> " << (unsigned)this->i2c.readReg(this->CFG_B) << std::endl;    
    //this->i2c.writeBytes(cfg_reg_b_buf, 3); usleep(50000);

    //uint8_t mode_reg_buf[3] = {this->WRT, this->MOD, 0};
    uint8_t mode_reg = 0; // Operating Mode Continuous-mode
    //mode_reg_buf[2] = mode_reg;
    //this->i2c.writeByte(this->WRT); usleep(10000);
    this->i2c.writeReg(this->MOD, mode_reg); usleep(100000);
    //std::cout << std::hex << "MOD " << (unsigned)mode_reg << " -> " << (unsigned)this->i2c.readReg(this->MOD) << std::endl;
    //this->i2c.writeBytes(mode_reg_buf, 3); usleep(50000);
    */
    
}
void HMC5883L::update()
{
    uint8_t buf[6] = {0,};
    //this->i2c.writeByte(this->RED);
    //uint8_t cmd[2] = {0x0d, 0x06};
    //this->i2c.writeBytes(cmd, 2); usleep(100000);

    this->i2c.readReg(this->XOUT_H, buf, 6);
    this->compass[0] = (double)((int16_t)(buf[0]<<8 | buf[1]))/1090;
    this->compass[1] = (double)((int16_t)(buf[2]<<8 | buf[3]))/1090;
    this->compass[2] = (double)((int16_t)(buf[4]<<8 | buf[5]))/1090;
    //std::cout << (int16_t)(buf[0]<<8 | buf[1]) << "\t" << (int16_t)(buf[2]<<8 | buf[3]) << "\t" << (int16_t)(buf[4]<<8 | buf[5]) << std::endl;
    //this->i2c.writeByte(this->WRT);
    //this->i2c.writeByte(this->XOUT_H);
}
void HMC5883L::run()
{
    this->init();
    std::cout.precision(3);

    while(true)
    {
        this->update();

        std::cout << std::fixed << this->compass[0] << "\t" << this->compass[1] << "\t" << this->compass[2] << std::endl;

        usleep(10000);
        
    }
}