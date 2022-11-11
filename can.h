#ifndef __CAN_H__
#define __CAN_H__

#include <unistd.h>

#include "mcp2515.h"

struct ICAN
{
protected:
    uint8_t msg_ExtFlg;
    uint32_t msg_Id;
    uint8_t msg_Dlc;
    uint8_t msg_Data[8];
    uint8_t msg_Rtr;
    uint8_t msg_filhit;
    enum
    {
        CAN_OK = 0x00,
        CAN_FAILINIT = 0x01,
        CAN_FAILTX = 0x02,
        CAN_MSGAVAIL = 0x03,
        CAN_NOMSG = 0x04,
        CAN_CTRLERROR = 0x05,
        CAN_GETTXBFTIMEOUT = 0x06,
        CAN_SENDMSGTIMEOUT = 0x07,
        CAN_FAIL = 0xff,
    };
    
public:
    virtual ~ICAN() {};
    virtual void begin() = 0;
    virtual uint8_t sendMsgBuf(uint32_t id, uint8_t ext, uint8_t len, uint8_t *buf) = 0;
    virtual uint8_t readMsgBuf(uint8_t &len, uint8_t *buf) = 0;
    virtual uint32_t getCanId() = 0;
};



class CANHS: ICAN
{
private:
    MCP2515 mcp2515;
    void setMsg(uint32_t id, uint8_t ext, uint8_t len, uint8_t *pData);
    void clearMsg();
    uint8_t readMsg();
    uint8_t sendMsg();
public:
    CANHS(/* args */);
    ~CANHS();    
    void begin();
    uint8_t sendMsgBuf(uint32_t id, uint8_t ext, uint8_t len, uint8_t *buf);
    uint8_t readMsgBuf(uint8_t &len, uint8_t *buf);
    uint32_t getCanId();
    
};


#endif