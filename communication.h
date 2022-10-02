#ifndef __COMMUNICATION_H__
#define __COMMUNICATION_H__

#include <arpa/inet.h>
#include <sys/socket.h>

#include <map>
#include <string>

namespace Communication
{
    class TCP_Server
    {
    private:
        typedef struct sockaddr socket_address_t;
        //int status = Communication::COM_INIT;
        int socket;
        struct sockaddr_in address;
        std::map<int, socket_address_t> clients;
    public:
        TCP_Server();
        ~TCP_Server();
        void begin(int port);
        void connect();
        void connect(int q_size);
        void send(std::string msg);
        //void receive();
        
    };

    class TCP_Client
    {
    private:
        int socket;
    public:
        TCP_Client();
        ~TCP_Client();
        void connect(std::string address, int port);
        void receive();

    };
}




#endif