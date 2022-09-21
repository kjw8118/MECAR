#include "gpio_interface.h"

#define __DEBUG__
#ifdef __DEBUG__
#include <iostream>
using namespace std;
#endif



#define TERMIOS
#ifdef TERMIOS
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <unistd.h>
#include <sys/ioctl.h>
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
            cout << "Already init" << "\n" << flush;
            #endif

            return 0;
        }
        if (gpioCfgClock(5,1,0) < 0)
        {
            GPIO::is_gpio_init = false;

            #ifdef __DEBUG__
            cout << "Clock Fail" << "\n" << flush;
            #endif
            
            return -1;
        }
        if (gpioInitialise() < 0)
        {
            GPIO::is_gpio_init = false;

            #ifdef __DEBUG__
            cout << "Init Fail" << "\n" << flush;
            #endif

            return -1;
        }
        
        GPIO::is_gpio_init = true;

        #ifdef __DEBUG__
        cout << "Init Succ" << "\n" << flush;
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
        cout << "digitalWrite put: " << Status << " in: " << Pin << "\n" << flush;
        #endif
    }

    void analogWrite(int pin, int value)
    {
        gpioPWM(pin, value);
        
        #ifdef __DEBUG__
        cout << "analogWrite put: " << value << " in: " << pin << "\n" << flush;
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
        cout << "Servo put: " << duty << " from : " << angle << "\n" << flush;
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
        //cout << "Enter >> Serial_termios::begin(" << baud << ")" << "\n" << flush;
        
        char port[] = "/dev/ttyUSB0";        
        this->handle = open(port, O_RDWR);
        this->baudrate = baud;
        
        if(tcgetattr(this->handle, &this->tty) !=0)
        {
            return -1;
        }

        this->tty.c_cflag &= ~PARENB;
        this->tty.c_cflag &= ~CSTOPB;
        this->tty.c_cflag &= ~CSIZE;
        this->tty.c_cflag |= CS8;
        this->tty.c_cflag &= ~CRTSCTS;
        this->tty.c_cflag |= CREAD | CLOCAL;

        this->tty.c_lflag &= ~ICANON;
        this->tty.c_lflag &= ~ECHO;
        this->tty.c_lflag &= ~ECHOE;
        this->tty.c_lflag &= ~ECHONL;
        this->tty.c_lflag &= ~ISIG;
        this->tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        this->tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

        this->tty.c_oflag &= ~OPOST;
        this->tty.c_oflag &= ~ONLCR;

        this->tty.c_cc[VTIME] = 10;
        this->tty.c_cc[VMIN] = 0;
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
        cout << "Set Baudrate " << baud_termios << endl;
        //cout << "Return >> Serial_termios::begin(" << baud << ")" << "\n" << flush;
        int flag = tcsetattr(this->handle, TCSANOW, &(this->tty));        
        if(flag == 0)
            return this->handle;
        else
            return -1;
        
    }
    void Serial_termios::write(uint8_t val)
    {
        uint8_t buf[0];
        buf[0] = val;
        this->write(buf, sizeof(buf));
    }
    void Serial_termios::write(uint8_t *buf, int size)
    {
        ::write(this->handle, (char*)buf, size*sizeof(uint8_t));
    }
    uint8_t Serial_termios::read()
    {
        uint8_t buf[0];
        this->read(buf, 1);
        return buf[0];
    }
    int Serial_termios::read(uint8_t *buf, int size)
    {
        return ::read(this->handle, (char*)buf, size*sizeof(char));
    }
    void Serial_termios::flush()
    {
        uint8_t buf[0];
        while(true)
        {
            int len = ::read(this->handle, (char*)buf, sizeof(uint8_t));
            if(len == 0)
                break;
        }
    }
    #ifdef STD
    std::string Serial_termios::print()
    {
        uint8_t buf[255] = {0};
        ::read(this->handle, (char*)buf, sizeof(buf));
        std::string str((char*)buf);
        return str;        
    }
    std::string Serial_termios::readline()
    {
        std::string str = "";
        char buf[0];        
        while(true)
        {
            int len = ::read(this->handle, buf, sizeof(char));
            //cout << "len " << len << "\n" << flush;
            if(len != 0)
            {
                str += string(buf);
            }
            else
                break;
        }
        return str;        

    }
    #endif
    int Serial_termios::available()
    {
        int bytes = fcntl(this->handle, F_SETFL, FNDELAY);
        //::ioctl(this->handle, FIONREAD, &bytes);
        //uint8_t buf[1024];
        //bytes = this->read(buf, 1024);
        return bytes;
    }
    void Serial_termios::close()
    {
        ::close(this->handle);
    }
        
}

#endif

