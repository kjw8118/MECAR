#include "communication.h"

#include <iostream>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>

Communication::TCP_Server::TCP_Server(int port)
{
    this->socket = ::socket(PF_INET, SOCK_STREAM, 0);

    std::memset(&(this->address), 0, ::sizeof(this->address));
    this->address.sin_family = AF_INET;
    this->address.sin_addr.s_addr = ::htonl(INADDR_ANY);
    this->address.sin_port = ::htons(port);
    ::bind(this->socket, (struct sockaddr*) &(this->address), sizeof(this->address));

    

}
Communication::TCP_Server::~TCP_Server(){};

void Communication::TCP_Server::connect()
{
    ::listen(this->socket, 5);
    
    struct sockaddr client_address;    
    
    socklen_t client_address_size = sizeof(client_address);

    int client_socket = ::accept(this->socket, (struct sockaddr*) &client_address, &client_address_size);
    this->clients[client_socket] = client_address;

}

void Communication::TCP_Server:send()
{
    for(const auto &[client_socket, client_address]: this->clients)
    char msg[] = "Hello world\n";
    ::write(client_socket, msg, sizeof(msg));
}

Communication::TCP_Clinet::TCP_Client()
{
    this->socket = ::socket(PF_INET, SOCK_STREAM, 0);

}

void Communication::TCP_Client::connect()
{
    int port = 0;
    struct sockaddr_in server_address;
    std::memset(&(server_address), 0, ::sizeof(server_address));
    server_address.sin_family = AF_INET;
    server_address.sin_addr.s_addr = ::inet_addr();
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