#ifndef __GPIO_INTERFACE_H__
#define __GPIO_INTERFACE_H__

#include <stdint.h>
#include <string>

#define TERMIOS2
#if defined(TERMIOS)
#include <termios.h>
#elif defined(TERMIOS2)
#include <asm/termbits.h>
#endif

#define RASPBERRYPI

#ifdef RASPBERRYPI
#include "pigpio.h"

namespace GPIO
{    

    enum STATE
    {
        GPIO_ERR = -1,
        GPIO_OFF = 0,
        GPIO_RDY = 1,
    };

    enum PINMODE
    {
        OUTPUT = PI_OUTPUT,
        INPUT = PI_INPUT,
        HIGH = 1,
        LOW = 0,

    };

    static int gpio_state = GPIO::GPIO_OFF;

    void init_gpio();

    void pinMode(int Pin, int Mode);

    int digitalRead(int Pin);

    void digitalWrite(int Pin, int Status);

    void analogWrite(int pin, int value);

    class Servo
    {
    private:
        int pin;
        int state = Servo::SERVO_OFF;
                
    public:
        enum STATE
        {
            SERVO_ERR = -1,
            SERVO_OFF = 0,
            SERVO_RDY = 1,
        };
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
        int state = Wire::WIRE_OFF;
    public:
        enum STATE
        {
            WIRE_ERR = -1,
            WIRE_OFF = 0,
            WIRE_RDY = 1,
        };
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
        int state = Serial_Base::SERIAL_OFF;
        enum
        {
            BAUDRATE4800 = 4800,
            BAUDRATE9600 = 9600,
            BAUDRATE115200 = 115200,
            BAUDRATE128000 = 128000,
        };

    public:
        enum STATE
        {
            SERIAL_ERR = -1,
            SERIAL_OFF = 0,
            SERIAL_RDY = 1,
        };
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
    #if defined(TERMIOS)
        struct termios tty;
    #elif defined(TERMIOS2)
        struct termios2 tty;
    #endif

    public:
        int begin(int baud);
        void write(uint8_t val);
        void write(uint8_t *buf, int size);
        uint8_t read();
        int read(uint8_t *buf, int size);
        int available();
        std::string readline();
        std::string readlines();

        int handle_public;

    };
    

    

        
}

#endif

#endif