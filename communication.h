#ifndef __COMMUNICATION_H__
#define __COMMUNICATION_H__

#include <arpa/inet.h>
#include <sys/socket.h>

#include <map>
#include <string>
#include <functional>

#include <thread>

namespace Communication
{
    enum CONSTANT
    {
        TCP_DEFAULT_PORT = 8118,
    };
    class TCP_Server
    {
    private:
        typedef struct sockaddr socket_address_t;
        
        //int status = Communication::COM_INIT;
        int socket;
        struct sockaddr_in address;
        std::map<int, socket_address_t> clients;        
        std::function<void(void)> service_task;
        //std::thread service_thread;
    public:
        TCP_Server();
        ~TCP_Server();
        void begin();
        void begin(int port);
        void connect();
        void connect(int q_size);
        void send(std::string msg);
        std::pair<int, std::string> receive();
        void response();        
        void regist_task(std::function<void(void)> task);
        void run();
        
        
    };

    class TCP_Client
    {
    private:
        int socket;
    public:
        TCP_Client();
        ~TCP_Client();
        void connect(std::string address);
        void connect(std::string address, int port);
        void receive();
        void request();
        void run();

    };
}




#endif