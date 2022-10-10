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

    template<typename T>
    class Server: public T
    {

    };

    /*class Local_Server
    {
    private:
        static int socket_count;
    protected:
        int socket;
        std::function<void(void)> service_task

    };*/

    class TCP_Server
    {
    protected:
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

    template<typename T>
    class Client: public T
    {

    };

    class TCP_Client
    {
    protected:
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