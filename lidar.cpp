#include "lidar.h"

#include <iostream>
#include <string>
#include <unistd.h>
using namespace std;

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
    //cout << "Enter >> LiDAR::command(" << (int)cmd << ")" << "\n" << flush;
    uint8_t cmd_buf[2];
    cmd_buf[0] = this->CMD_COM;
    cmd_buf[1] = cmd;
    std::cout << (unsigned)cmd_buf[0] << " " << (unsigned)cmd_buf[1] << endl;
    //return;
    //unsigned char cmds[] = { 0xa5};

    //::write(3, )
    this->serial.write(cmd_buf, 2);
    //::usleep(200000);

    /*if(cmd != this->CMD_RST && cmd != this->CMD_STP)
    {
        //cout << "Get header" << "\n" << flush;
        //cout << "Len :" << this->serial.available() << "\n" << flush;
        //string str = this->serial.readline();//((char*)buf);
        //cout << "Msg[" << str.length() << "]: " << str << "\n" << "\n" << flush;
        //return;
        //int len = this->serial.available();
        
        uint8_t pstart[2];
        int len = this->serial.available();
        this->serial.read(pstart, 2);
        std::cout << "Len " << len << "\n" << std::flush;
        uint16_t start_sign = this->dec2word(pstart);
        

        uint8_t plnm[4];
        this->serial.read(plnm, 4);
        uint32_t lnm = this->dec2int(plnm);
        uint32_t length = lnm & ~(3 << 30);
        uint8_t mode = lnm >> 30;

        uint8_t type = this->serial.read();
        
        std::cout << std::hex;
        std::cout << (unsigned)pstart[0] << " " << (unsigned)pstart[1] << "\n" << std::flush;
        std::cout << (unsigned)plnm[0] << " " << (unsigned)plnm[1] << " " << (unsigned)plnm[2] << " " << (unsigned)plnm[3] << "\n" << std::flush;
        std::cout << (unsigned)type << "\n" << std::flush;
        //cout << "Header >> Start sign: " << start_sign << ", Length: " << length << ", Mode: " << mode << ", Type code: " << type << "\n" << flush;


        
    }*/  
    //cout << "Return >> LiDAR::command(" << (int)cmd << ")" << "\n" << flush;
}
LiDAR::LiDAR(/* args */)
{
    //cout << "Enter >> LiDAR::LiDAR()" << "\n" << flush;
    this->tof.angle = new double[400];
    this->tof.distance = new double[400];
    this->tof.length = 0;

    this->tof_temp.angle = new double[400];
    this->tof_temp.distance = new double[400];
    this->tof_temp.length = 0;
    //cout << "Return >> LiDAR::LiDAR()" << "\n" << flush;

}
void LiDAR::init()
{
    //cout << "Enter >> LiDAR::init()" << "\n" << flush;
    int ret = this->serial.begin(128000);
    //sleep(1);
    cout << "Serial return " << ret << endl;
    //this->serial.begin(115200);
    
    //cout << "Return >> LiDAR::init()" << "\n" << flush;
}
void LiDAR::soft_reset()
{    
    this->command(this->CMD_RST);
    
    //sleep(2);
    uint8_t buf[255] = {0};
    int len = 0;
    while(true)
    {
        usleep(1000);
        len = this->serial.read(buf, 1);
        cout << len << " " << (unsigned)buf[0] << endl;

    }
    this->serial.read(buf, 255);
    cout << len << " " << (unsigned)buf[0] << '\n' << endl;
}
int LiDAR::get_device_health()
{
    this->command(this->CMD_HLT);
    uint8_t status = this->serial.read();
    uint8_t error[2];
    this->serial.read(error, 2);
    return (int)status;
}
void LiDAR::get_device_info()
{
    this->command(this->CMD_INF);
    uint8_t model_number = this->serial.read();
    uint8_t pfirmware[2];
    this->serial.read(pfirmware, 2);
    uint16_t firmware_version = this->dec2word(pfirmware);
    uint8_t hardware_version = this->serial.read();
    uint8_t pserial_number[16];
    this->serial.read(pserial_number, 16);        
}
void LiDAR::scan_start()
{
    this->command(this->CMD_STR);
    while(true)
    {
        uint8_t buf[0];
        int len = this->serial.read(buf, 1);
        cout << len << " " << (unsigned)buf[0] << endl;
    }

}
void LiDAR::scan_stop()
{
    this->command(this->CMD_STP);
}
void LiDAR::close()
{
    this->serial.close();
}
void LiDAR::run()
{
    cout << "Enter >> LiDAR::run()" << endl;
    this->init();
    /*unsigned char buf_stp[] = {0xa5, 0x65};
    int len_stp = ::write(3, buf_stp, sizeof(buf_stp));
    cout << "writed len " << len_stp << endl;*/
    unsigned char buf_rst[2];
    buf_rst[0] = 0xa5;
    buf_rst[1] = 0x60;
    int len;
    len = ::write(3, buf_rst, sizeof(buf_rst));
    //len += ::write(3, buf_rst, sizeof(buf_rst));
    //len += ::write(3, buf_rst, sizeof(buf_rst));
    //len += ::write(3, buf_rst, sizeof(buf_rst));
    cout << "writed len " << len << endl;
    char read_buf[2];
    int len_read;
    
    while(true)
    {
        len_read = ::read(3, read_buf, sizeof(read_buf));
        cout << "Read " << len_read << " ";
         for(int i=0; i<2; i++)
         {
            cout << (unsigned)read_buf[0] << " ";
         }
         cout << endl;
    }
    cout << endl;
    //this->soft_reset();
    //this->scan_start();

    //this->get_device_info();
    //this->close();
    //this->scan_start();
    //int ret = 0;
    //cout << "Now enter loop.." << endl;
    //sleep(2);
    //this->close();
    
    /*while(ret >= 0)
    {
        //ret = this->get_one_packet();
        try
        {
            
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            this->close();
        }
        
    }*/


}
LiDAR::~LiDAR()
{
    cout << "Enter >> LiDAR::~LiDAR()" << "\n" << flush;
    /*delete[] this->tof.angle;
    delete[] this->tof.distance;

    delete[] this->tof_temp.angle;
    delete[] this->tof_temp.distance;*/
    cout << "Return >> LiDAR::~LiDAR()" << "\n" << flush;
}

int LiDAR::get_one_packet()
{
    uint8_t msg;
    bool flag = true;
    while(flag)
    {
        if(this->serial.read() == 0xaa)
        {
            if(this->serial.read() == 0x55)
            {
                flag = false;
            }
        }
    }
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

    }

    if(cs != cs_xor)
    {
        return -1;
    }
    else
    {
        return (int)lsn;
    }

}
