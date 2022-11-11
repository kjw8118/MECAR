#include "mcp2515.h"
#include <iostream>

MCP2515::~MCP2515()
{
}

void MCP2515::reset()
{
    this->spi.select();
    this->spi.transfer(this->MCP_RESET);
    this->spi.unselect();
}
uint8_t MCP2515::readReg(uint8_t address)
{
    uint8_t ret = 0;
    this->spi.select();
    this->spi.transfer(this->MCP_READ);
    this->spi.transfer(address);
    ret = this->spi.read();
    this->spi.unselect();
    
    return ret;

}
void MCP2515::readRegs(uint8_t address, uint8_t values[], uint8_t n)
{
    uint8_t ret = 0;
    this->spi.select();
    this->spi.transfer(this->MCP_READ);
    this->spi.transfer(address);
    for(int i=0; i<n; i++)
    {
        values[i] = this->spi.read();
    }    
    this->spi.unselect();
        

}
void MCP2515::writeReg(uint8_t address, uint8_t value)
{
    this->spi.select();
    this->spi.transfer(this->MCP_WRITE);
    this->spi.transfer(address);
    this->spi.transfer(value);
    this->spi.unselect();
}
void MCP2515::writeRegs(uint8_t address, uint8_t values[], uint8_t n)
{
    this->spi.select();
    this->spi.transfer(this->MCP_WRITE);
    this->spi.transfer(address);
    for(uint8_t i=0; i<n; i++)
    {
        this->spi.transfer(values[i]);
    }
    this->spi.unselect();
}

void MCP2515::modifyRegister(uint8_t address, uint8_t mask, uint8_t data)
{
    this->spi.select();
    this->spi.transfer(this->MCP_BITMOD);
    this->spi.transfer(address);
    this->spi.transfer(mask);
    this->spi.transfer(data);
    this->spi.unselect();

}
void MCP2515::setControlMode(uint8_t newmode)
{
    this->modifyRegister(this->MCP_CANCTRL, this->MODE_MASK, newmode);
    uint8_t i = this->readReg(this->MCP_CANCTRL);
    i *= this->MODE_MASK;

    if(i == newmode)
    {
        return;
    }
    else
    {
        
    }

}
void MCP2515::write_id(uint8_t address, uint8_t ext, uint32_t id)
{
    uint16_t canid = (uint16_t)(id & 0x0ffff);

    uint8_t tbufdata[4] = {0,};
    if(ext == 1)
    {
        tbufdata[this->MCP_EID0] = (uint8_t)(canid & 0xff);
        tbufdata[this->MCP_EID8] = (uint8_t)(canid >> 8);
        canid = (uint16_t)(id >> 16);
        tbufdata[this->MCP_SIDL] = (uint8_t)(canid & 0x03);
        tbufdata[this->MCP_SIDL] += (uint8_t)((canid & 0x1c) << 3);
        tbufdata[this->MCP_SIDL] |= this->MCP_TXB_EXIDE_M;
        tbufdata[this->MCP_SIDH] = (uint8_t)(canid >> 5);
    }
    else
    {
        tbufdata[this->MCP_SIDH] = (uint8_t)(canid >> 3);
        tbufdata[this->MCP_SIDL] = (uint8_t)((canid & 0x07) << 5);
        tbufdata[this->MCP_EID0] = 0;
        tbufdata[this->MCP_EID8] = 0;
    }
    this->writeRegs(address, tbufdata, 4);

}
void MCP2515::read_id(uint8_t address, uint8_t &ext, uint32_t &id)
{
    uint8_t tbufdata[4];
    ext = 0;
    id = 0;
    this->readRegs(address, tbufdata, 4);
    id = (tbufdata[this->MCP_SIDH] << 3) + (tbufdata[this->MCP_SIDL] >> 5);
    if( (tbufdata[this->MCP_SIDL] & this->MCP_TXB_EXIDE_M) == this->MCP_TXB_EXIDE_M)
    {
        // Ext
        id = (id << 2) + (tbufdata[this->MCP_SIDL] & 0x03);
        id = (id << 8) + tbufdata[this->MCP_EID8];
        id = (id << 8) + tbufdata[this->MCP_EID0];
        ext = 1;
    }
}
void MCP2515::setBaudrate()
{
    uint8_t cfg1, cfg2, cfg3;
    cfg1 = this->MCP_8MHz_500kBPS_CFG1;
    cfg2 = this->MCP_8MHz_500kBPS_CFG2;
    cfg3 = this->MCP_8MHz_500kBPS_CFG3;
    
    this->writeReg(this->MCP_CNF1, cfg1);
    this->writeReg(this->MCP_CNF2, cfg2);
    this->writeReg(this->MCP_CNF3, cfg3);

    
}
void MCP2515::initCANBuffer()
{
    uint8_t std = 0;
    uint8_t ext = 1;
    uint32_t ulMask = 0x00, ulFilt = 0x00;

    this->write_id(this->MCP_RXM0SIDH, ext, ulMask);
    this->write_id(this->MCP_RXM1SIDH, ext, ulMask);

    this->write_id(this->MCP_RXF0SIDH, ext, ulFilt);
    this->write_id(this->MCP_RXF1SIDH, std, ulFilt);
    this->write_id(this->MCP_RXF2SIDH, ext, ulFilt);
    this->write_id(this->MCP_RXF3SIDH, std, ulFilt);
    this->write_id(this->MCP_RXF4SIDH, ext, ulFilt);
    this->write_id(this->MCP_RXF5SIDH, std, ulFilt);
    
    uint8_t a1 = this->MCP_TXB0CTRL;
    uint8_t a2 = this->MCP_TXB1CTRL;
    uint8_t a3 = this->MCP_TXB2CTRL;

    for(int i=0; i<14; i++)
    {
        this->writeReg(a1++, 0);
        this->writeReg(a2++, 0);
        this->writeReg(a3++, 0);
    }
    this->writeReg(this->MCP_RXB0CTRL, 0);
	this->writeReg(this->MCP_RXB1CTRL, 0);

}

void MCP2515::setInterrupt()
{
    this->writeReg(this->MCP_CANINTE, this->MCP_RX0IF | this->MCP_RX1IF);
}

uint8_t MCP2515::readStatus()
{
    this->spi.select();
    this->spi.transfer(this->MCP_READ_STATUS);
    uint8_t read_state = this->spi.read();
    this->spi.unselect();
    return read_state;
}
bool MCP2515::getNextFreeTXBuf(uint8_t *txbuf_n)
{        
    uint8_t ctrlregs[this->MCP_N_TXBUFFERS] = {this->MCP_TXB0CTRL, this->MCP_TXB1CTRL, this->MCP_TXB2CTRL};        
    *txbuf_n = 0x00;
    for(int i=0; i<this->MCP_N_TXBUFFERS; i++)
    {
        uint8_t ctrlval = this->readReg(ctrlregs[i]);
        if( (ctrlval & this->MCP_TXB_TXREQ_M) == 0 )
        {
            *txbuf_n = ctrlregs[i] + 1;                
            return true;
        }
    }
    return false;
}

bool MCP2515::sendMsgTransmitted(uint8_t txbuf_n)
{
    return this->readReg(txbuf_n) & 0x08;
}
bool MCP2515::read_canMsg(uint8_t &ext, uint32_t &id, uint8_t &dlc, uint8_t &rtr, uint8_t *pData)
{
    uint8_t status = this->readStatus();
    uint8_t buffer_sidh_addr, mask;
    if(status & this->MCP_STAT_RX0IF)
    {
        buffer_sidh_addr = this->MCP_RXBUF_0;
        mask = this->MCP_RX0IF;
    }
    else if(status & this->MCP_STAT_RX1IF)
    {
        buffer_sidh_addr = this->MCP_RXBUF_1;
        mask = this->MCP_RX1IF;
    }
    else
    {
        return false;
    }
    
    this->read_id(buffer_sidh_addr, ext, id);
    uint8_t ctrl = this->readReg(buffer_sidh_addr - 1);
    dlc = this->readReg(buffer_sidh_addr + 4);
    if(ctrl & 0x08)
    {
        rtr = 1;
    }
    else
    {
        rtr = 0;
    }
    dlc &= this->MCP_DLC_MASK;
    this->readRegs(buffer_sidh_addr + 5, pData, dlc);

    this->modifyRegister(this->MCP_CANINTF, mask, 0);

    return true;
}
void MCP2515::write_canMsg(uint8_t buffer_sidh_addr, uint8_t ext, uint32_t id, uint8_t dlc, uint8_t rtr, uint8_t *pData)
{
    this->writeRegs(buffer_sidh_addr + 5, pData, dlc);
    this->writeReg((buffer_sidh_addr + 4), dlc|(rtr*this->MCP_RTR_MASK));
    this->write_id(buffer_sidh_addr, ext, id);

}
void MCP2515::start_transmit(uint8_t mcp_addr)
{
    this->modifyRegister(mcp_addr - 1, this->MCP_TXB_TXREQ_M, this->MCP_TXB_TXREQ_M);
}


void MCP2515::init()
{
    
    this->spi.begin(4000000);
    //std::cout << "Now init SPI" << std::endl;
    this->reset();
    //std::cout << "Now reset MCP2515" << std::endl;
    this->setControlMode(this->MODE_CONFIG);    
    this->initCANBuffer();
    this->setInterrupt();
    
    this->modifyRegister(this->MCP_RXB0CTRL,
        this->MCP_RXB_RX_MASK | this->MCP_RXB_BUKT_MASK,
        this->MCP_RXB_RX_STDEXT | this->MCP_RXB_BUKT_MASK);
    this->modifyRegister(this->MCP_RXB1CTRL, this->MCP_RXB_RX_MASK,
        this->MCP_RXB_RX_STDEXT);

    this->setControlMode(this->MODE_CONFIG);
    //std::cout << "Now start MCP2515" << std::endl;
}

