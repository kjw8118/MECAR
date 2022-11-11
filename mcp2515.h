#ifndef __MCP2515_H__
#define __MCP2515_H__

#include "gpio_interface.h"

class MCP2515
{
private:
    /* data */
    enum
    {
        MCP_RESET = 0xc0,
        MCP_BITMOD = 0x05,
        MCP_CANCTRL = 0x0f,
        MODE_MASK = 0xe0,
        MCP_READ = 0x03,
        MCP_EID0 = 3,
        MCP_EID8 = 2,
        MCP_SIDL = 1,
        MCP_SIDH = 0,
        MCP_TXB_EXIDE_M = 0x08,
        MCP_WRITE = 0x02,
        MCP_8MHz_500kBPS_CFG1 = 0x00,
        MCP_8MHz_500kBPS_CFG2 = 0x90,
        MCP_8MHz_500kBPS_CFG3 = 0x02,
        MCP_CNF1 = 0x2a,
        MCP_CNF2 = 0x29,
        MCP_CNF3 = 0x28,
        MCP_RXM0SIDH = 0x20,
        MCP_RXM1SIDH = 0x24,
        MCP_RXF0SIDH = 0x00,
        MCP_RXF1SIDH = 0x04,
        MCP_RXF2SIDH = 0x08,
        MCP_RXF3SIDH = 0x10,
        MCP_RXF4SIDH = 0x14,
        MCP_RXF5SIDH = 0x18,
        MCP_TXB0CTRL = 0x30,
        MCP_TXB1CTRL = 0x40,
        MCP_TXB2CTRL = 0x50,
        MCP_RXB0CTRL = 0x60,
        MCP_RXB1CTRL = 0x70,

        MODE_CONFIG = 0x80,
        MODE_NORMAL = 0x00,

        MCP_CANINTE = 0x2b,
        MCP_RX0IF = 0x01,
        MCP_RX1IF = 0x02,

        MCP_RXB_RX_MASK = 0x60,
        MCP_RXB_BUKT_MASK = (1<<2),
        MCP_RXB_RX_STDEXT = 0x00,

        MCP_READ_STATUS = 0xa0,
        MCP_STAT_RX0IF = (1<<0),
        MCP_STAT_RX1IF = (1<<1),

        MCP_RXBUF_0 = 0x61,
        MCP_RXBUF_1 = 0x71,

        MCP_CANINTF = 0x2c,

        MCP_DLC_MASK = 0x0f,

        MCP_N_TXBUFFERS = 3,
        
        MCP_TXB_TXREQ_M = 0x08,

        MCP2515_OK = 0,
        MCP2515_FAIL = 1,
        MCP_ALLTXBUSY = 2,

        MCP_RTR_MASK = 0x40,
    };
    
    void reset();
    uint8_t readReg(uint8_t address);
    void readRegs(uint8_t address, uint8_t values[], uint8_t n);
    void writeReg(uint8_t address, uint8_t value);
    void writeRegs(uint8_t address, uint8_t values[], uint8_t n);
    
    void setControlMode(uint8_t newmode);
    void write_id(uint8_t address, uint8_t ext, uint32_t id);
    void read_id(uint8_t address, uint8_t &ext, uint32_t &id);
    void setBaudrate();
    void initCANBuffer();
    void setInterrupt();
    
    
    //uint8_t readMsg();
    GPIO::SPI spi;
public:
    MCP2515(int cs): spi(cs) {};
    ~MCP2515();
    void init();
    uint8_t readStatus();
    void modifyRegister(uint8_t address, uint8_t mask, uint8_t data);
    bool read_canMsg(uint8_t &ext, uint32_t &id, uint8_t &dlc, uint8_t &rtr, uint8_t *pData);
    void write_canMsg(uint8_t buffer_sidh_addr, uint8_t ext, uint32_t id, uint8_t dlc, uint8_t rtr, uint8_t *pData);
    void start_transmit(uint8_t mcp_addr);
    bool getNextFreeTXBuf(uint8_t *txbuf_n);
    bool sendMsgTransmitted(uint8_t txbuf_n);

    

};



#endif