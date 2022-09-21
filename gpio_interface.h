#ifndef __GPIO_INTERFACE_H__
#define __GPIO_INTERFACE_H__

#define TERMIOS
#ifdef TERMIOS
#include <termios.h>
#endif

#define STD
#ifdef STD
#include <string>
#endif

#define RASPBERRYPI

#ifdef RASPBERRYPI
#include "pigpio.h"

namespace GPIO
{    

    enum PINMODE
    {
        OUTPUT = PI_OUTPUT,
        INPUT = PI_INPUT,
        HIGH = 1,
        LOW = 0,

    };

    static bool is_gpio_init = false;

    int init_gpio();

    void pinMode(int Pin, int Mode);

    int digitalRead(int Pin);

    void digitalWrite(int Pin, int Status);

    void analogWrite(int pin, int value);

    class Servo
    {
    private:
        int pin;
        
        
    public:
        Servo();
        Servo(int pin);
        void attach(int pin);
        void stop();
        void write(int angle);
        ~Servo();
        
    };

    class Wire
    {
    private:
        int bus = 1;
        uint8_t address = 0;
        int handle;
    public:        
        void begin(uint8_t addr);
        void writeByte(uint8_t val);
        void writeBytes(uint8_t *buf, int size);
        void writeReg(uint8_t reg_addr, uint8_t val);
        uint8_t readReg(uint8_t reg_addr);
        uint8_t readByte();
        void readBytes(uint8_t *buf, int size);                
    };

    class Serial_Base
    {
    protected:
        int handle;
        int baudrate;
        enum
        {
            BAUDRATE4800 = 4800,
            BAUDRATE9600 = 9600,
            BAUDRATE115200 = 115200,
            BAUDRATE128000 = 128000,
        };

    public:
        virtual int begin(int baud) = 0;
        virtual void write(uint8_t val) = 0;
        virtual void write(uint8_t *buf, int size) = 0;
        virtual uint8_t read() = 0;
        virtual int read(uint8_t *buf, int size) = 0;
        virtual int available() = 0;

    };
    
    class Serial: public Serial_Base
    {
    public:
        int begin(int baud);
        void write(uint8_t val);
        void write(uint8_t *buf, int size);
        uint8_t read();
        int read(uint8_t *buf, int size);
        int available();

    };

    class Serial_termios: public Serial_Base
    {
    private:
        struct termios tty;
    public:
        int begin(int baud);
        void write(uint8_t val);
        void write(uint8_t *buf, int size);
        uint8_t read();
        int read(uint8_t *buf, int size);
        #ifdef STD
        std::string print();
        std::string readline();
        #endif
        int available();
        void close();
        void flush();
        

    };
    

    

        
}

#endif

#endif