#include "lidar.h"

#include <iostream>
#include <string>
//using namespace std;

uint16_t LiDAR::dec2word(uint8_t *buf)
{
    uint16_t val = 0;
    val = buf[0]&0xff | (buf[1]&0xff)<<8;
    return val;
}
uint32_t LiDAR::dec2int(uint8_t *buf)
{
    uint32_t val = 0;
    val = buf[0]&0xff | (buf[1]&0xff)<<8 | (buf[2]&0xff)<<16 | (buf[3]&0xff)<<24;
    return val;
}
void LiDAR::command(uint8_t cmd)
{
    //std::cout << "Enter >> LiDAR::command(" << (int)cmd << ")" << std::endl;
    uint8_t cmd_buf[2];
    cmd_buf[0] = this->CMD_COM;
    cmd_buf[1] = cmd;
    this->serial.write(cmd_buf, 2);
    if(cmd != this->CMD_RST && cmd != this->CMD_STP)
    {
        //std::cout << "Get header" << std::endl;
        bool flag = true;
        while(flag)
        {
            if(this->serial.read() != 0xa5)            
                if(this->serial.read() != 0x5a)
                    flag =false;
        }
        
        std::cout << "Start sign pass" << std::endl;
        uint8_t pstart[2] = {0xa5, 0x5a};
        //this->serial.read(pstart, 2);
        uint16_t start_sign = this->dec2word(pstart);
        //std::cout << (unsigned)pstart[0] << " " << (unsigned)pstart[1] << std::endl;

        uint8_t plnm[4];
        this->serial.read(plnm, 4);
        //std::cout << plnm[0] << " " << plnm[1] << " " << plnm[2] << " " << plnm[3] << std::endl;
        uint32_t lnm = this->dec2int(plnm);
        uint32_t length = lnm & ~(3 << 30);
        uint8_t mode = lnm >> 30;

        uint8_t type = this->serial.read();
        //std::cout << type << std::endl;
        std::cout << std::hex << "Header >> Start sign: " << start_sign;
        std::cout << std::dec << ", Length: " << length << ", Mode: " << (unsigned)mode << ", Type code: " << (unsigned)type << std::endl;


        
    }
    //std::cout << std::dec << "Return >> LiDAR::command(" << (int)cmd << ")" << std::endl;
}
LiDAR::LiDAR(/* args */)
{
    std::cout << "Enter >> LiDAR::LiDAR()" << std::endl;
    this->tof.angle = new double[1024];
    this->tof.distance = new double[1024];
    this->tof.length = 0;

    this->tof_temp.angle = new double[1024];
    this->tof_temp.distance = new double[1024];
    this->tof_temp.length = 0;
    //std::cout << "Return >> LiDAR::LiDAR()" << std::endl;

}
void LiDAR::init()
{
    //std::cout << "Enter >> LiDAR::init()" << std::endl;
    int ret = this->serial.begin(128000);
    std::cout << "Serial begin "<< ret << std::endl;
    
    
    //std::cout << "Return >> LiDAR::init()" << std::endl;
}
void LiDAR::soft_reset()
{
    //std::cout << "Enter >> LiDAR::soft_reset()" << std::endl;
    this->command(this->CMD_RST);    
    /*while(this->serial.available() > 0)
    {
        std::cout << this->serial.readline();
    }*/
    std::cout << this->serial.readlines();// << std::endl;
    //uint8_t buf[256];
    //int len = this->serial.read(buf, 256);
    //buf[len] = 0;
    //std::cout << "Received bytes: " << len << std::endl;
    //std::cout << std::string((char*)buf) << "\n" << std::endl;
    
    //std::cout << this->serial.readline() << std::endl;

    //std::cout << "Return >> LiDAR::soft_reset()" << std::endl;
}
int LiDAR::get_device_health()
{
    //std::cout << "Get device health" << std::endl;
    this->command(this->CMD_HLT);
    uint8_t status = this->serial.read();
    uint8_t error[2];
    this->serial.read(error, 2);
    std::cout << "Status: " << (unsigned)status << std::endl;
    return (int)status;
}
void LiDAR::get_device_info()
{
    //std::cout << "Get device info" << std::endl;
    this->command(this->CMD_INF);
    uint8_t model_number = this->serial.read();
    uint8_t pfirmware[2];
    this->serial.read(pfirmware, 2);
    uint16_t firmware_version = this->dec2word(pfirmware);
    uint8_t hardware_version = this->serial.read();
    uint8_t pserial_number[16];
    this->serial.read(pserial_number, 16);
    std::cout << "Model num.: " << (unsigned)model_number << " Firmware ver.: " << (unsigned)firmware_version << " Hardware ver.: " << (unsigned)hardware_version << " Serial num.: ";
    for(int i=0; i<16; i++)
    {
        std::cout << std::hex << (unsigned)pserial_number[i];
    }
    std::cout << std::endl;
}
void LiDAR::scan_start()
{
    this->command(this->CMD_STR);

}
void LiDAR::scan_stop()
{
    this->command(this->CMD_STP);
}
void LiDAR::run()
{
    std::cout << "Enter >> LiDAR::run()" << std::endl;
    this->init();
    
    this->soft_reset();

    this->get_device_health();
    this->get_device_info();
    //this->scan_start();
    int ret = 0;
    while(ret >= 0)
    {
        //ret = this->get_one_packet();
    }


}
LiDAR::~LiDAR()
{
    std::cout << "Enter >> LiDAR::~LiDAR()" << std::endl;
    /*delete[] this->tof.angle;
    delete[] this->tof.distance;

    delete[] this->tof_temp.angle;
    delete[] this->tof_temp.distance;*/
    //std::cout << "Return >> LiDAR::~LiDAR()" << std::endl;
}

int LiDAR::get_one_packet()
{
    uint8_t msg;
    bool flag = true;
    std::cout << "One loop";
    while(flag)
    {
        if(this->serial.read() == 0xaa)
        {
            if(this->serial.read() == 0x55)
            {
                flag = false;
            }
        }
        std::cout << ".";
    }
    std::cout << " ";

    uint8_t pstart[2] = {0xaa, 0x55};
    uint16_t start = this->dec2word(pstart);

    uint8_t ct = this->serial.read();
    uint8_t lsn = this->serial.read();
    uint16_t ct_n_lsn = lsn << 8 | ct;
    
    uint8_t pfsa[2];
    this->serial.read(pfsa, 2);
    uint16_t fsa = this->dec2word(pfsa);
    
    uint8_t plsa[2];
    this->serial.read(plsa, 2);
    uint16_t lsa = this->dec2word(plsa);

    uint8_t pcs[2];
    this->serial.read(pcs, 2);
    uint16_t cs = this->dec2word(pcs);

    uint16_t cs_xor = start ^ ct_n_lsn ^ fsa ^ lsa;

    double angle_lsa = (double)(lsa >> 1) / 64;
    double angle_fsa = (double)(fsa >> 1) / 64;

    if(angle_fsa > angle_lsa)
        angle_lsa += 360;
    
    if(ct&1) // Start Period
    {
        std::cout << "Start period, ";
        for(int i=0; i<this->tof_temp.length; i++)
        {
            this->tof.angle[i] = this->tof_temp.angle[i];
            this->tof.distance[i] = this->tof_temp.distance[i];
            this->tof.length = this->tof_temp.length;
        }
        for(int i=this->tof_temp.length; i<400; i++)
        {
            this->tof.angle[i] = 0;
            this->tof.distance[i] = 0;                
        }
        this->tof_temp.length = 0;
    }

    uint8_t pdistance[2];
    this->serial.read(pdistance, 2);
    uint16_t distance_uint16 = this->dec2word(pdistance);

    cs_xor ^= distance_uint16;

    double distance = (double)distance_uint16/4;

    double angle_diff = angle_lsa - angle_fsa;
    double angle = angle_fsa;
    if(distance_uint16 != 0)
    {
        angle += std::atan(21.8 * (155.3 - distance)/(155.3*distance));
    }

    if(angle > 360)
    {
        angle -= 360;
    }

    this->tof_temp.angle[this->tof_temp.length] = angle;
    this->tof_temp.distance[this->tof_temp.length] = distance;
    this->tof_temp.length++;
    std::cout << angle << " -> ";

    for(int i=1; i<lsn; i++)
    {
        this->serial.read(pdistance, 2);
        distance_uint16 = this->dec2word(pdistance);
        cs_xor ^= distance_uint16;

        distance = (double)distance_uint16;

        double angle_temp = angle_diff / (lsn -1) * i + angle_fsa;
        if(distance_uint16 != 0)
        {
            angle = angle_temp + std::atan(21.8 * (155.3 - distance)/(155.3*distance));
        }
        else
        {
            angle = angle_temp;
        }

        if(angle > 360)
        {
            angle -= 360;
        }

        this->tof_temp.angle[this->tof_temp.length] = angle;
        this->tof_temp.distance[this->tof_temp.length] = distance;
        this->tof_temp.length++;
        std::cout << angle << ", ";

    }
    std::cout << std::endl;

    if(cs != cs_xor)
    {
        return -1;
    }
    else
    {
        return (int)lsn;
    }

}
