#ifndef __LIDAR_H__
#define __LIDAR_H__

#include "gpio_interface.h"
using namespace GPIO;

#include <cmath>

class LiDAR
{
private:
    Serial_termios serial;
    enum
    {
        CMD_COM = 0xa5,
        CMD_STR = 0x60,
        CMD_STP = 0x65,
        CMD_INF = 0x90,
        CMD_HLT = 0x91,
        CMD_RST = 0x80,
    };

    enum
    {
        NORMALLY = 0,
        WARNING = 1,
        INCORRECTLY = 2,
    };
    const int header_length = 7;

    uint16_t dec2word(uint8_t *buf);
    uint32_t dec2int(uint8_t *buf);
    void command(uint8_t cmd);
    struct ToF
    {        
        double *angle;
        double *distance;
        int length;
    };
    struct ToF tof;
    struct ToF tof_temp;

public:
    LiDAR(/* args */);
    void init();
    void soft_reset();
    int get_device_health();
    void get_device_info();
    int get_one_packet();
    void scan_start();
    void scan_stop();
    void close();
    void run();
    ~LiDAR();
};


#endif