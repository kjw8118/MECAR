#ifndef __COMMUNICATION_H__
#define __COMMUNICATION_H__

#include <sys/socket.h>

#include <map>

namespace Communication
{
    class TCP_Server
    {
    private:
        typedef struct sockaddr socket_address_t;
        int socket;
        struct sockaddr_in address;
        std::map<int, socket_address_t> clients;
    public:
        TCP_Server(int port);
        ~TCP_Server();
        void connect();
        void send();
        //void receive();
        
    }

    class TCP_Client
    {
    private:
        int socket;
    public:
        TCP_Client(int port);
        ~TCP_Client();
        void connect();
        void receive();

    }
}




#endif