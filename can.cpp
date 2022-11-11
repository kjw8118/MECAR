#include "can.h"
#include <iostream>


CANHS::CANHS(/* args */)
    : mcp2515(8)
{
    
}

CANHS::~CANHS()
{
}

void CANHS::setMsg(uint32_t id, uint8_t ext, uint8_t len, uint8_t *pData)
{
    this->msg_ExtFlg = ext;
    this->msg_Id = id;
    this->msg_Dlc = len;
    for(int i=0; i<8; i++)
    {
        this->msg_Data[i] = *(pData + i);
    }
}
void CANHS::clearMsg()
{
    this->msg_Id = 0;
    this->msg_Dlc = 0;
    this->msg_ExtFlg = 0;
    this->msg_Rtr = 0;
    this->msg_filhit = 0;
    for(int i=0; i<8; i++)
    {
        this->msg_Data[i] = 0x00;
    }
}
uint8_t CANHS::sendMsg()
{
    uint16_t timeout = 0;
    uint8_t txbuf_n;
    while (!this->mcp2515.getNextFreeTXBuf(&txbuf_n))
    {
        if(timeout++ > 50)
        {
            return this->CAN_GETTXBFTIMEOUT;
        }
    }
    this->mcp2515.write_canMsg(txbuf_n, this->msg_ExtFlg, this->msg_Id, this->msg_Dlc, this->msg_Rtr, this->msg_Data);
    this->mcp2515.start_transmit(txbuf_n);

    timeout = 0;
    while(!this->mcp2515.sendMsgTransmitted(txbuf_n))
    {
        if(timeout++ > 50)
        {
            return this->CAN_SENDMSGTIMEOUT;
        }
    }
    return this->CAN_OK;

}
uint8_t CANHS::readMsg()
{
    if(this->mcp2515.read_canMsg(this->msg_ExtFlg, this->msg_Id, this->msg_Dlc, this->msg_Rtr, this->msg_Data))
    {
        return this->CAN_OK;
    }
    else
    {
        return this->CAN_NOMSG;
    }

}

void CANHS::begin()
{
    this->mcp2515.init();
}
uint8_t CANHS::sendMsgBuf(uint32_t id, uint8_t ext, uint8_t len, uint8_t *buf)
{
    this->setMsg(id, ext, len, buf);
    this->sendMsg();

    uint8_t ret = 0;
    return ret;
}
uint8_t CANHS::readMsgBuf(uint8_t &len, uint8_t *buf)
{
    this->readMsg();
    len = this->msg_Dlc;
    for(int i=0; i<this->msg_Dlc; i++)
    {
        buf[i] = this->msg_Data[i];
    }

    uint8_t ret = 0;
    return ret;    
}
uint32_t CANHS::getCanId()
{
    return this->msg_Id;
}

