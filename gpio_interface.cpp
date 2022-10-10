#include "gpio_interface.h"

#include <map>

#include <functional>

#include <limits.h>
#include <unistd.h>
#include <fcntl.h>

#include <thread>

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

    void init_gpio_thread()
    {
        if(GPIO::gpio_state == GPIO::GPIO_RDY)
        {
            #ifdef __DEBUG__
            std::cout << "Already init" << std::endl;
            #endif
            
        }
        else
        {
            if (gpioCfgClock(5,1,0) < 0)
            {                
                #ifdef __DEBUG__
                std::cout << "Clock Fail" << std::endl;
                #endif

                GPIO::gpio_state = GPIO::GPIO_ERR;

            }
            else
            {
                if (gpioInitialise() < 0)
                {                    
                    #ifdef __DEBUG__
                    std::cout << "Init Fail" << std::endl;
                    #endif

                    GPIO::gpio_state = GPIO::GPIO_ERR;
                    
                }
                else
                {
                    #ifdef __DEBUG__
                    std::cout << "Init Succ" << std::endl;
                    #endif
                    
                    GPIO::gpio_state = GPIO::GPIO_RDY;
                }
                
            }
            
        }
        //return GPIO::gpio_state;
        
    }

    int init_gpio()
    {
        if(GPIO::gpio_state != GPIO::GPIO_RDY)
        {
            thread gpio_thread = std::thread(GPIO::init_gpio_thread);
            gpio_thread.join();
        }
        return GPIO::gpio_state;

    }


    void pinMode(int Pin, int Mode)
    {
        if(GPIO::gpio_state == GPIO::GPIO_RDY)
            gpioSetMode(Pin, Mode);
        else
            throw GPIO::gpio_state;
    }

    int digitalRead(int Pin)
    {
        if(GPIO::gpio_state == GPIO::GPIO_RDY)
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
        else
        {
            throw GPIO::gpio_state;
            return -1;
        }

        
    }

    void digitalWrite(int Pin, int Status)
    {
        if(GPIO::gpio_state == GPIO::GPIO_RDY)
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
            //std::cout << "digitalWrite put: " << Status << " in: " << Pin << std::endl;
            #endif
        }
        else
            throw GPIO::gpio_state;
    }

    void analogWrite(int pin, int value)
    {
        if(GPIO::gpio_state == GPIO::GPIO_RDY)
        {
            gpioPWM(pin, value);
            
            #ifdef __DEBUG__
            //std::cout << "analogWrite put: " << value << " in: " << pin << std::endl;
            #endif
        }
        else
            throw GPIO::gpio_state;
    }

    void attachInterrupt(int pin, void (*ISR)(), int mode)
    {
        if(GPIO::gpio_state == GPIO::GPIO_RDY)
        {
            switch(mode)
            {
                case GPIO::RISING:
                    gpioSetISRFunc(pin, RISING_EDGE, 1, (gpioISRFunc_t)(ISR));
                    break;
                case GPIO::FALLING:
                    gpioSetISRFunc(pin, FALLING_EDGE, 1, (gpioISRFunc_t)(ISR));
                    break;
                case GPIO::CHANGE:
                    gpioSetISRFunc(pin, EITHER_EDGE, 1, (gpioISRFunc_t)(ISR));
                    break;
                default:
                    throw GPIO::gpio_state;
                    break;
            }            
        }
        else
            throw GPIO::gpio_state;
    }
    
    /*void attachInterrupt(int pin, std::function<void()> ISR, int mode)
    {
        
        int i = 0;
        std::cout << "attachInterrupt " << i++ << std::endl;
        
        std::cout << "attachInterrupt " << i++ << std::endl;
        auto ppISR = ISR.target<void(*)()>();
        std::cout << "attachInterrupt " << i++ << std::endl;
        if(ppISR == nullptr)
        {
            std::cout << "attachInterrupt nullptr" << std::endl;    
        }
        std::cout << "attachInterrupt before" << std::endl;    
        void (*pISR)() = *ppISR;
        std::cout << "attachInterrupt " << i++ << std::endl;
        std::cout << "attachInterrupt " << i++ << std::endl;
        if(GPIO::gpio_state == GPIO::GPIO_RDY)
        {
            switch(mode)
            {
                case GPIO::RISING:
                    gpioSetISRFunc(pin, RISING_EDGE, 1, (gpioISRFunc_t)(pISR));
                    break;
                case GPIO::FALLING:
                    gpioSetISRFunc(pin, FALLING_EDGE, 1, (gpioISRFunc_t)(pISR));
                    break;
                case GPIO::CHANGE:
                    gpioSetISRFunc(pin, EITHER_EDGE, 1, (gpioISRFunc_t)(pISR));
                    break;
                default:
                    throw GPIO::gpio_state;
                    break;
            }            
        }
        else
            throw GPIO::gpio_state;
    }*/

    /*template <class T>
    void attachInterrupt(int pin, void (T::*ISR)(), int mode)
    {
        if(GPIO::gpio_state == GPIO::GPIO_RDY)
        {
            switch(mode)
            {
                case GPIO::RISING:
                    gpioSetISRFunc(pin, RISING_EDGE, 1, (gpioISRFunc_t)(ISR));
                    break;
                case GPIO::FALLING:
                    gpioSetISRFunc(pin, FALLING_EDGE, 1, (gpioISRFunc_t)(ISR));
                    break;
                case GPIO::CHANGE:
                    gpioSetISRFunc(pin, EITHER_EDGE, 1, (gpioISRFunc_t)(ISR));
                    break;
                default:
                    throw GPIO::gpio_state;
                    break;
            }            
        }
        else
            throw GPIO::gpio_state;
    }*/
    
    void Servo::attach(int pin)
    {
        if(this->state == Servo::SERVO_OFF || this->state == Servo::SERVO_ERR)
        {
            GPIO::init_gpio();
            this->pin = pin;
            if(GPIO::gpio_state == GPIO::GPIO_RDY)
            {
                gpioSetPWMfrequency(pin, 400);
                gpioSetPWMrange(pin, 2500);
                gpioServo(pin, 0);

                this->state = Servo::SERVO_RDY;
            }
            else
            {
                this->state = Servo::SERVO_ERR;
            }
        }
        else
        {
            #ifdef __DEBUG__
            std::cout << "Servo already attached" << std::endl;
            #endif
        }
    }
    void Servo::stop()
    {
        if(this->state == Servo::SERVO_RDY)
        {
            gpioServo(this->pin, 0);
        }
    }
    void Servo::write(int angle)
    {
        if(this->state == Servo::SERVO_RDY)
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
            //std::cout << "Servo put: " << duty << " from : " << angle << std::endl;
            #endif
        }
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
        if(this->state == Wire::WIRE_OFF || this->state == Wire::WIRE_ERR)
        {
            GPIO::init_gpio();
            this->address = addr;
            if(GPIO::gpio_state == GPIO::GPIO_RDY)
            {
                this->handle = i2cOpen(this->bus, addr, 0);

                this->state = Wire::WIRE_RDY;
            }
            else
            {
                this->state = Wire::WIRE_ERR;
            }
            
        }
        else
        {
            #ifdef __DEBUG__
            std::cout << "Wire already begin" << std::endl;
            #endif
        }

    }
    void Wire::writeByte(uint8_t val)
    {
        if(this->state == Wire::WIRE_RDY)
        {
            i2cWriteByte(this->handle, val);
        }
    }
    void Wire::writeBytes(uint8_t *buf, int size)
    {
        if(this->state == Wire::WIRE_RDY)
        {
            i2cWriteDevice(this->handle, (char*)buf, size);
        }
    }
    void Wire::writeReg(uint8_t reg_addr, uint8_t val)
    {
        if(this->state == Wire::WIRE_RDY)
        {
            uint8_t buf[2];
            buf[0] = reg_addr;
            buf[1] = val;
            this->writeBytes(buf, 2);
        }
    }
    uint8_t Wire::readByte()
    {
        if(this->state == Wire::WIRE_RDY)
        {
            return i2cReadByte(this->handle);
        }
        else
            return 0;
    }
    void Wire::readBytes(uint8_t *buf, int size)
    {
        if(this->state == Wire::WIRE_RDY)
        {
            i2cReadDevice(this->handle, (char*)buf, size);
        }
    }
    uint8_t Wire::readReg(uint8_t reg_addr)
    {
        if(this->state == Wire::WIRE_RDY)
        {
            return i2cReadByteData(this->handle, reg_addr);
        }
        else
            return 0;
    }
    void Wire::readReg(uint8_t reg_addr, uint8_t *buf, int size)
    {
        if(this->state == Wire::WIRE_RDY)
        {                        
            i2cReadI2CBlockData(this->handle, reg_addr, (char*)buf, size);
        }
    }

    int Serial::begin(int baud)
    {
        if(this->state == Serial::SERIAL_OFF || this->state == Serial::SERIAL_ERR)
        {
            GPIO::init_gpio();
            char port[] = "/dev/ttyUSB0";
            this->baudrate = baud;
            
            if(GPIO::gpio_state == GPIO::GPIO_RDY)
            {
                this->handle = serOpen(port, baud, 0);
                
                this->state = Serial::SERIAL_RDY;
            }
            else
            {
                this->state = Serial::SERIAL_ERR;
            }                        
            
        }
        else
        {
            #ifdef __DEBUG__
            std::cout << "Serial already begin" << std::endl;
            #endif
        }
        
        return this->state;
    }
    void Serial::write(uint8_t val)
    {
        if(this->state == Serial::SERIAL_RDY)
        {
            serWriteByte(this->handle, val);
        }
    }
    void Serial::write(uint8_t *buf, int size)
    {
        if(this->state == Serial::SERIAL_RDY)
        {
            serWrite(this->handle, (char*)buf, size);
        }
    }
    uint8_t Serial::read()
    {
        if(this->state == Serial::SERIAL_RDY)
        {
            return serReadByte(this->handle);
        }
        else
            return 0;
    }
    int Serial::read(uint8_t *buf, int size)
    {
        if(this->state == Serial::SERIAL_RDY)
        {
            return serRead(this->handle, (char*)buf, size);
        }
        else
            return 0;
    }
    int Serial::available()
    {
        if(this->state == Serial::SERIAL_RDY)
        {
            return serDataAvailable(this->handle);
        }
        else
            return 0;
    }
    
    
    int Serial_termios::begin(int baud)
    {
        std::cout << "Enter >> Serial_termios::begin(" << baud << ")" << std::endl;
        
        char port[] = "/dev/ttyUSB0";        
        this->baudrate = baud;
        ::close(3);
        if(this->state == Serial_termios::SERIAL_OFF || this->state == Serial_termios::SERIAL_ERR)
        {
            this->handle = ::open(port, O_RDWR);
            if(this->handle < 0)
            {
                std::cout << "Serial Port open error -> Handle: " << this-> handle << " Errno: " << errno << " " << ::strerror(errno) << std::endl;
                //return -1;
                this->state = Serial_termios::SERIAL_ERR;
                return this->state;
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
                //return -1;
                this->state = Serial_termios::SERIAL_ERR;
                return this->state;
            }
            if(::ioctl(this->handle, TCGETS2, &(this->tty)))
            {
                std::cout << "Conf get err" << std::endl;
                //return -1;
                this->state = Serial_termios::SERIAL_ERR;
                return this->state;
            }        

        #endif

            std::cout << "Configed baudrate: " << this->tty.c_ispeed << " " << this->tty.c_ospeed << std::endl;
            //std::cout << "Return >> Serial_termios::begin(" << baud << ")" << std::endl;
            //return this->handle;
            this->state = Serial_termios::SERIAL_RDY;
            return this->state;
        }
        else
        {
            #ifdef __DEBUG__
            std::cout << "Serial_termios already begin" << std::endl;
            #endif
            return this->state;
        }
        
        
    }
    void Serial_termios::write(uint8_t val)
    {
        if(this->state == Serial_termios::SERIAL_RDY)
        {
            uint8_t buf[0];
            buf[0] = val;
            this->write(buf, 1);
        }
    }
    void Serial_termios::write(uint8_t *buf, int size)
    {
        if(this->state == Serial_termios::SERIAL_RDY)
        {
            ::write(this->handle, buf, size*sizeof(uint8_t));

            //std::cout << "Write " << (unsigned)buf[0] << " " << (unsigned)buf[1] << "\n";
        }
    }
    uint8_t Serial_termios::read()
    {
        if(this->state == Serial_termios::SERIAL_RDY)
        {
            uint8_t buf[0];
            this->read(buf, 1);
            return buf[0];
        }
        else
            return 0;
    }
    int Serial_termios::read(uint8_t *buf, int size)
    {
        if(this->state == Serial_termios::SERIAL_RDY && this->available() > 0)
        {
            int ret = ::read(this->handle, (char*)buf, size*sizeof(uint8_t));

            //std::cout << std::hex << "Read " << ret << "\n";
            return ret;
        }
        else
            return 0;
    }
    int Serial_termios::available()
    {
        if(this->state == Serial_termios::SERIAL_RDY)
        {
            int bytes = 0;
            int itr = 0;
            while(bytes == 0 && itr < 1000)
            {
                itr++;
                ::ioctl(this->handle, FIONREAD, &bytes);                
            }
            //std::cout << itr << std::endl;
            //uint8_t buf[1000];
            //bytes = this->read(buf, 1000);
            return bytes;
        }
        else
        {
            
            return 0;
        }
    }
    
    std::string Serial_termios::readline()
    {
        if(this->state == Serial_termios::SERIAL_RDY)
        {
            while(this->available() == 0) {};
            
            uint8_t buf[256] ={0,};
            /*int len = 0;
            while(this->available() != 0)
            {
                len += this->read(buf+len, 256);
                std::cout << len << std::endl;
            }
            buf[len] = 0;*/
            int len = this->read(buf, 256);
            /*for(int i=0; i<len; i++)
            {
                std::cout << i << ", " << (unsigned)buf[i] << "\n";
            }*/
            //buf[len] = 0;
            //std::cout << "readline len " << len << std::endl;
            return std::string((char*)buf, len);
        }
        else
            return "";
    }
    std::string Serial_termios::readlines()
    {
        if(this->state == Serial_termios::SERIAL_RDY)
        {
            std::string str = "";
            while(this->available() >0)
            {
                str += this->readline();
            }
            return str;
        }
        else
            return "";
    }

    
        
}

/*template <class T>
namespace GPIO
{
    void attachInterrupt(int pin, void (T::*ISR)(), int mode)
    {
        if(GPIO::gpio_state == GPIO::GPIO_RDY)
        {
            switch(mode)
            {
                case GPIO::RISING:
                    gpioSetISRFunc(pin, RISING_EDGE, 1, (gpioISRFunc_t)(ISR));
                    break;
                case GPIO::FALLING:
                    gpioSetISRFunc(pin, FALLING_EDGE, 1, (gpioISRFunc_t)(ISR));
                    break;
                case GPIO::CHANGE:
                    gpioSetISRFunc(pin, EITHER_EDGE, 1, (gpioISRFunc_t)(ISR));
                    break;
                default:
                    throw GPIO::gpio_state;
                    break;
            }            
        }
        else
            throw GPIO::gpio_state;
    }
}*/

#endif

