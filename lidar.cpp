#include "lidar.h"
#include "pointcloud.h"

#include "timer.h"

#include <opencv2/opencv.hpp>

#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <future>
#include <unistd.h>

//#include <array>

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
    //std::cout << "Enter >> LiDAR::command(" << (int)cmd << ")\n";// << std::endl;
    int step = 0;
    uint8_t cmd_buf[2];
    cmd_buf[0] = this->CMD_COM;
    cmd_buf[1] = cmd;

    this->serial.write(cmd_buf, 2);

    if(cmd != this->CMD_RST && cmd != this->CMD_STP)
    {
        
        
        while(this->serial.available() < 7)
        {
            usleep(10000);
        }
//        int len = this->serial.available();
//        std::cout << len << "_";// << std::endl;
        uint8_t header[7] = {0,};
        int len = this->serial.read(header, 7);
        std::cout << std::hex << "Start Sign: " << (unsigned)header[0] << " " << (unsigned)header[1] << std::endl;
        std::cout << std::hex << "Length: " << (unsigned)(0xffff&(header[2]| header[3]<<8 | header[4]<<16 | header[5]<<24)) << std::endl;
        std::cout << std::hex << "Type: " << (unsigned)header[6] << std::endl;
    }
    else
    {
        //std::cout << "No header commd\n";
    }
    //std::cout << std::dec << "Return >> LiDAR::command(" << (int)cmd << ")" << std::endl;
}
LiDAR::LiDAR(/* args */)
{
    //std::cout << "Enter >> LiDAR::LiDAR()\n";// << std::endl;
    this->tof.angle.clear();//new double[1024];
    this->tof.distance.clear(); //new double[1024];
    this->tof.length = 0;

    this->tof_temp.angle.clear(); //new double[1024];
    this->tof_temp.distance.clear(); //new double[1024];
    this->tof_temp.length = 0;

    //this->pointcloud = PointCloud(1000, 1000);
    //std::cout << "Return >> LiDAR::LiDAR()" << std::endl;
    //this->axes = cv::Mat(1000, 1000, CV_8UC3, cv::Scalar(0, 0, 0));
    /*for(int i=0; i<points.size(); i++)
    {
        points.at(i).fill(0);
    }*/
}
void LiDAR::init()
{
    int ret = this->serial.begin(128000);
    sleep(1);
}
void LiDAR::soft_reset()
{
    this->command(this->CMD_RST);
    usleep(this->TIME_RST);
    std::cout << this->serial.readlines() << std::endl;
    sleep(2);
}
void LiDAR::get_device_health()
{
    this->command(this->CMD_HLT);
    int len  = 0;
    uint8_t content[3] = {0,};
    this->serial.read(content, 3);

    uint8_t status = content[0];//this->serial.read();
    std::cout << std::hex << "Status: " << (unsigned)status << "\n";//std::flush;
}
void LiDAR::get_device_info()
{
    this->command(this->CMD_INF);
    uint8_t content[0x14] = {0,};
    this->serial.read(content, 0x14);
    uint8_t model_number = content[0];
    uint16_t firmware_version = this->dec2word(&content[1]);
    uint8_t hardware_version = content[3];
    std::cout << std::hex << "Model num.: " << (unsigned)model_number << " Firmware ver.: " << (unsigned)firmware_version << " Hardware ver.: " << (unsigned)hardware_version << " Serial num.: ";
    for(int i=0; i<16; i++)
    {
        std::cout << std::hex << (unsigned)content[i+4];
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

void LiDAR::plot()
{    
    
    this->timer.set_t0_ms();
    while(true)
    {
        
        this->pointcloud.putToF(this->tof);
        
        cv::Mat axes = this->pointcloud.getMap();
        
        cv::imshow("Plot", axes);
        
        if(cv::waitKey(1) == 27)
            break;
        
    }
    cv::destroyAllWindows();

    this->working_flag = false;
    this->scan_stop();
    
    std::cout << "Plot exit" << std::endl;
}

void LiDAR::run()
{
    //std::cout << "Enter >> LiDAR::run()" << std::endl;
    this->init();
    
    //std::future<void> soft_reset_async = std::async(std::launch::async, std::bind(&LiDAR::soft_reset, this));
    //soft_reset_async.wait();    
    this->soft_reset();
    //std::cout << "Soft Reset" << std::endl;

    //std::future<void> get_device_health_async = std::async(std::launch::async, std::bind(&LiDAR::get_device_health, this));
    //get_device_health_async.wait();
    //this->get_device_health();
    //std::cout << "Device Health\n";// << std::endl;
    //std::future<void> get_device_info_async = std::async(std::launch::async, std::bind(&LiDAR::get_device_info, this));
    //get_device_info_async.wait();
    //this->get_device_info();
    //std::cout << "Device info" << std::endl;
    
    
    //std::future<void> plot_async = std::async(std::launch::async, std::bind(&LiDAR::plot, this));
    std::thread plot_thread = std::thread(&LiDAR::plot, this);
    
    this->scan_start();
    
    int ret = 0;
    while(this->working_flag)
    {
        //sleep(1);
        ret = this->get_one_packet();
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
    uint8_t header[10] = {0,};
    this->serial.read(header, 10);
    //uint8_t pstart[2] = {0xaa, 0x55};
    uint16_t start = this->dec2word(header);

    uint8_t ct = header[2];//this->serial.read();
    uint8_t lsn = header[3];//this->serial.read();
    uint16_t ct_n_lsn = lsn << 8 | ct;
    
    //uint8_t pfsa[2];
    //this->serial.read(pfsa, 2);
    uint16_t fsa = this->dec2word(&header[4]);
    
    //uint8_t plsa[2];
    //this->serial.read(plsa, 2);
    uint16_t lsa = this->dec2word(&header[6]);

    //uint8_t pcs[2];
    //this->serial.read(pcs, 2);
    uint16_t cs = this->dec2word(&header[8]);

    uint16_t cs_xor = start ^ ct_n_lsn ^ fsa ^ lsa;

    double angle_lsa = (double)(lsa >> 1) / 64;
    double angle_fsa = (double)(fsa >> 1) / 64;

    if(angle_fsa > angle_lsa)
        angle_lsa += 360;
    uint8_t samples[lsn*2];
    this->serial.read(samples, lsn*2);
    if(ct&1) // Start Period
    {
        //this->tex.lock();
        this->tof.angle.clear();
        this->tof.distance.clear();
        for(int i=0; i<this->tof_temp.length; i++)
        {
            this->tof.angle.push_back(this->tof_temp.angle[i]);
            this->tof.distance.push_back(this->tof_temp.distance[i]);
        }
        this->tof.length = this->tof_temp.length;
        /*for(int i=this->tof_temp.length; i<400; i++)
        {
            this->tof.angle[i] = 0;
            this->tof.distance[i] = 0;                
        }*/
        this->tof_temp.angle.clear();
        this->tof_temp.distance.clear();
        this->tof_temp.length = 0;
        //this->tex.unlock();
    }

    //uint8_t pdistance[2];
    //this->serial.read(pdistance, 2);
    uint16_t distance_uint16 = this->dec2word(samples);

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

    this->tof_temp.angle.push_back(angle);
    this->tof_temp.distance.push_back(distance);
    this->tof_temp.length++;
    //std::cout << angle << " -> ";

    for(int i=1; i<lsn; i++)
    {
        
        distance_uint16 = this->dec2word(&samples[i*2]);
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

        this->tof_temp.angle.push_back(angle);
        this->tof_temp.distance.push_back(distance);
        this->tof_temp.length++;
        //std::cout << angle << ", ";

    }
    //std::cout << std::endl;

    if(cs != cs_xor)
    {
        return -1;
    }
    else
    {
        return (int)lsn;
    }

}
