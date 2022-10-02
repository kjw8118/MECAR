#include "communication.h"

#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

Communication::TCP_Server::TCP_Server()
{
    this->socket = ::socket(PF_INET, SOCK_STREAM, 0);
    /*if(this->socket < 0)
        this->status = Communication::COM_ERROR;
    else
        this->status = Communication::COM_OK;*/
}
Communication::TCP_Server::~TCP_Server(){};

void Communication::TCP_Server::begin()
{
    this->begin(0);
}

void Communication::TCP_Server::begin(int port)
{    
    std::memset(&(this->address), 0, sizeof(this->address));
    this->address.sin_family = AF_INET;
    this->address.sin_addr.s_addr = ::htonl(INADDR_ANY);
    this->address.sin_port = ::htons(port);
    int ret = ::bind(this->socket, (struct sockaddr*) &(this->address), sizeof(this->address));
    
    std::cout << "Server Address: " << INADDR_ANY << std::endl;    

}

void Communication::TCP_Server::connect()
{
    this->connect(5);
}
void Communication::TCP_Server::connect(int q_size)
{
    int ret = ::listen(this->socket, q_size);
    
    struct sockaddr client_address;    
    
    socklen_t client_address_size = sizeof(client_address);

    int client_socket = ::accept(this->socket, (struct sockaddr*) &client_address, &client_address_size);
    
    this->clients[client_socket] = client_address;
        
}

void Communication::TCP_Server::send(std::string msg)
{
    for(const auto &[client_socket, client_address]: this->clients)
    {
        //char msg[] = "Hello world\n";
        ::write(client_socket, msg, msg.length());
    }
}

std::pair<int, std::string> Communication::TCP_Server::receive()
{
    char msg[30];
    int str_len;
    std::string msg_ret = "";
    str_len = ::read(this->socket, msg, sizeof(msg));
    if(str_len > 0)
    {
        std::string msg_ret(msg);
        return std::make_pair(str_len, msg_ret);
    }
    else
    {
        std::string msg_ret = "";
        return std::make_pair(str_len, msg_ret);
    }

}

void Communication::TCP_Server::run()
{
    this->begin();
    this->connect();

    while(true)
    {
        auto [str_len, msg] = this->receive();
        if(str_len < 0)
            break;
        else
        {
            if(str_len > 0 )
            {
                std::cout << msg << std::endl;
            }
        }
    }
}

Communication::TCP_Client::TCP_Client()
{
    this->socket = ::socket(PF_INET, SOCK_STREAM, 0);

}

Communication::TCP_Client::~TCP_Client() {};

void Communication::TCP_Client::connect(std::string address, int port)
{    
    struct sockaddr_in server_address;
    std::memset(&(server_address), 0, sizeof(server_address));
    server_address.sin_family = AF_INET;
    try
        server_address.sin_addr.s_addr = ::inet_addr(address);
    catch
        server_address.sin_addr.s_addr = ::inet_addr("0.0.0.0");
    server_address.sin_port = ::htons(port);

    ::connect(this->socket, (struct sockaddr*)&server_address, sizeof(server_address));
    
    
}

void Communication::TCP_Client::receive()
{
    char msg[30];
    int str_len;
    str_len = ::read(this->socket, msg, sizeof(msg));

    std::cout << msg << std::endl;
}

void Communication::TCP_Client::run()
{

}