#include "gpio_interface.h"

#define __DEBUG__
#ifdef __DEBUG__
#include <iostream>
using namespace std;
#endif

#define TERMIOS2

#if defined(TERMIOS)
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstring>

#elif defined(TERMIOS2)
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <asm/termbits.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstring>
#endif

#ifdef RASPBERRYPI
#include "pigpio.h"

namespace GPIO
{    
    int init_gpio()
    {
        if(GPIO::is_gpio_init)
        {
            #ifdef __DEBUG__
            std::cout << "Already init" << std::endl;
            #endif

            return 0;
        }
        if (gpioCfgClock(5,1,0) < 0)
        {
            GPIO::is_gpio_init = false;

            #ifdef __DEBUG__
            std::cout << "Clock Fail" << std::endl;
            #endif
            
            return -1;
        }
        if (gpioInitialise() < 0)
        {
            GPIO::is_gpio_init = false;

            #ifdef __DEBUG__
            std::cout << "Init Fail" << std::endl;
            #endif

            return -1;
        }
        
        GPIO::is_gpio_init = true;

        #ifdef __DEBUG__
        std::cout << "Init Succ" << std::endl;
        #endif
        
        return 0;
        
    }

    void pinMode(int Pin, int Mode)
    {
        gpioSetMode(Pin, Mode);
    }

    int digitalRead(int Pin)
    {
        int pin_status = gpioRead(Pin);
        if(pin_status == PI_HIGH)
        {
            return GPIO::HIGH;
        }
        else
        {
            return GPIO::LOW;
        }
        
    }

    void digitalWrite(int Pin, int Status)
    {
        if(Status == GPIO::HIGH)
        {
            gpioWrite(Pin, PI_HIGH);
        }
        else
        {
            gpioWrite(Pin, PI_LOW);
        }

        #ifdef __DEBUG__
        std::cout << "digitalWrite put: " << Status << " in: " << Pin << std::endl;
        #endif
    }

    void analogWrite(int pin, int value)
    {
        gpioPWM(pin, value);
        
        #ifdef __DEBUG__
        std::cout << "analogWrite put: " << value << " in: " << pin << std::endl;
        #endif
    }

    
    
    void Servo::attach(int pin)
    {
        GPIO::init_gpio();
        this->pin = pin;
        gpioSetPWMfrequency(pin, 400);
        gpioSetPWMrange(pin, 2500);
        gpioServo(pin, 0);
    }
    void Servo::stop()
    {
        gpioServo(this->pin, 0);
    }
    void Servo::write(int angle)
    {
        if(angle < 0)
        {
            angle = 0;            
        }
        if(angle > 180)
        {
            angle = 180;
        }
        int duty = 500 + angle*2000/180;
        gpioServo(this->pin, duty);

        #ifdef __DEBUG__
        std::cout << "Servo put: " << duty << " from : " << angle << std::endl;
        #endif
    }
    
    Servo::Servo() {};
    Servo::Servo(int pin)
    {
        this->attach(pin);
    }
    
    Servo::~Servo()
    {
        gpioServo(this->pin, 0);
    }

    void Wire::begin(uint8_t addr)
    {
        GPIO::init_gpio();
        this->handle = i2cOpen(this->bus, addr, 0);        
        this->address = addr;        

    }
    void Wire::writeByte(uint8_t val)
    {
        i2cWriteByte(this->handle, val);
    }
    void Wire::writeBytes(uint8_t *buf, int size)
    {
        i2cWriteDevice(this->handle, (char*)buf, size);
    }
    void Wire::writeReg(uint8_t reg_addr, uint8_t val)
    {
        uint8_t buf[2];
        buf[0] = reg_addr;
        buf[1] = val;
        this->writeBytes(buf, 2);
    }
    uint8_t Wire::readByte()
    {
        return i2cReadByte(this->handle);
    }
    void Wire::readBytes(uint8_t *buf, int size)
    {
        i2cReadDevice(this->handle, (char*)buf, size);

    }
    uint8_t Wire::readReg(uint8_t reg_addr)
    {
        return i2cReadByteData(this->handle, reg_addr);
    }

    int Serial::begin(int baud)
    {
        GPIO::init_gpio();
        char port[] = "/dev/ttyUSB0";
        this->handle = serOpen(port, baud, 0);
        this->baudrate = baud;
        return this->handle;
    }
    void Serial::write(uint8_t val)
    {
        serWriteByte(this->handle, val);
    }
    void Serial::write(uint8_t *buf, int size)
    {
        serWrite(this->handle, (char*)buf, size);
    }
    uint8_t Serial::read()
    {
        return serReadByte(this->handle);
    }
    int Serial::read(uint8_t *buf, int size)
    {
        return serRead(this->handle, (char*)buf, size);
    }
    int Serial::available()
    {
        return serDataAvailable(this->handle);
    }
    
    
    int Serial_termios::begin(int baud)
    {
        std::cout << "Enter >> Serial_termios::begin(" << baud << ")" << std::endl;
        
        char port[] = "/dev/ttyUSB0";        
        this->baudrate = baud;
        ::close(3);
        this->handle = ::open(port, O_RDWR);
        if(this->handle < 0)
        {
            std::cout << "Serial Port open error -> Handle: " << this-> handle << " Errno: " << errno << " " << ::strerror(errno) << std::endl;
            return -1;
        }
        else
        {
            std::cout << "Serial port open at " << this->handle << std::endl;
        }
        
    
        this->tty.c_cflag &= ~PARENB;
        this->tty.c_cflag &= ~CSTOPB;
        this->tty.c_cflag &= ~CSIZE;
        this->tty.c_cflag |= CS8;
        this->tty.c_cflag &= ~CRTSCTS;
        this->tty.c_cflag |= CREAD | CLOCAL;
        this->tty.c_cflag &= ~CBAUD;

        this->tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ECHOCTL | ECHOPRT | ECHOKE);
        this->tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        this->tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

        this->tty.c_oflag &= ~OPOST;
        this->tty.c_oflag &= ~ONLCR;

        this->tty.c_cc[VTIME] = 10;
        this->tty.c_cc[VMIN] = 0;
        
    #if defined(TERMIOS)
        int baud_termios;        
        switch(this->baudrate)
        {
            case BAUDRATE4800:
                baud_termios = B4800;
                break;
            case BAUDRATE9600:
                baud_termios = B9600;
                break;
            case BAUDRATE115200:
                baud_termios = B115200;
                break;
            default:
                baud_termios = this->baudrate;
                break;
        }
        cfsetispeed(&tty, baud_termios);
        cfsetospeed(&tty, baud_termios);

        
        if(::tcsetattr(this->handle, TCSANOW, &(this->tty)) != 0)
        {
            std::cout << "Serial config set err " << errno << " " << ::strerror(errno) << std::endl;
            return -1;
        }
        if(::tcgetattr(this->handle, &(this->tty)))
        {
            std::cout << "Conf get err" << std::endl;
            return -1;
        }

    #elif defined(TERMIOS2)
        ::ioctl(this->handle, TCGETS2, &(this->tty));
        this->tty.c_cflag &= ~CBAUD;
        this->tty.c_cflag |= CBAUDEX;
        this->tty.c_ispeed = this->baudrate;
        this->tty.c_ospeed = this->baudrate;
        if(::ioctl(this->handle, TCSETS2, &(this->tty)))
        {
            std::cout << "Serial config set err " << errno << " " << ::strerror(errno) << std::endl;
            return -1;
        }
        if(::ioctl(this->handle, TCGETS2, &(this->tty)))
        {
            std::cout << "Conf get err" << std::endl;
            return -1;
        }        

    #endif

        std::cout << "Configed baudrate: " << this->tty.c_ispeed << " " << this->tty.c_ospeed << std::endl;
        //std::cout << "Return >> Serial_termios::begin(" << baud << ")" << std::endl;
        return this->handle;
        
        
    }
    void Serial_termios::write(uint8_t val)
    {
        uint8_t buf[0];
        buf[0] = val;
        this->write(buf, 1);
    }
    void Serial_termios::write(uint8_t *buf, int size)
    {
        ::write(this->handle, buf, size*sizeof(uint8_t));
    }
    uint8_t Serial_termios::read()
    {
        uint8_t buf[0];
        this->read(buf, 1);
        return buf[0];
    }
    int Serial_termios::read(uint8_t *buf, int size)
    {
        return ::read(this->handle, (char*)buf, size*sizeof(uint8_t));        
    }
    int Serial_termios::available()
    {
        int bytes = 0;
        int itr = 0;
        while(bytes == 0 && itr < 1000000)
        {
            ::ioctl(this->handle, FIONREAD, &bytes);
            itr++;
        }
        //std::cout << "len " << bytes << std::endl;
        //std::cout << itr << std::endl;
        //uint8_t buf[1000];
        //bytes = this->read(buf, 1000);
        return bytes;
    }
    
    std::string Serial_termios::readline()
    {
        uint8_t buf[256] ={0,};
        /*int len = 0;
        while(this->available() != 0)
        {
            len += this->read(buf+len, 256);
            std::cout << len << std::endl;
        }
        buf[len] = 0;*/
        int len = this->read(buf, 256);
        //buf[len] = 0;
        //std::cout << "readline len " << len << std::endl;
        return std::string((char*)buf);
    }
    std::string Serial_termios::readlines()
    {
        std::string str = "";
        while(this->available() >0)
        {
            str += this->readline();
        }
        return str;
    }

        
}

#endif

