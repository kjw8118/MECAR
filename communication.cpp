#include "communication.h"

#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#include <thread>
#include <future>

Communication::TCP_Server::TCP_Server()
{
    std::cout << "Server socket generated: " << std::flush;
    this->socket = ::socket(PF_INET, SOCK_STREAM, 0);
    
    std::cout << this->socket << std::endl;

}
Communication::TCP_Server::~TCP_Server(){};

void Communication::TCP_Server::begin()
{
    this->begin(Communication::TCP_DEFAULT_PORT);
}

void Communication::TCP_Server::begin(int port)
{    
    std::cout << "Server Address: " << std::flush;

    std::memset(&(this->address), 0, sizeof(this->address));
    this->address.sin_family = AF_INET;
    this->address.sin_addr.s_addr = ::htonl(INADDR_ANY);
    this->address.sin_port = ::htons(port);
    int ret = ::bind(this->socket, (struct sockaddr*) &(this->address), sizeof(this->address));
    
    std::cout << ret << std::endl;

}

void Communication::TCP_Server::connect()
{
    this->connect(5);
}

void Communication::TCP_Server::connect(int q_size)
{
    std::cout << "Server listen: " << std::flush;
    int ret = ::listen(this->socket, q_size);
    std::cout << ret << std::endl;

    std::cout << "Server accepted: " << std::flush;
    struct sockaddr client_address;    
    
    socklen_t client_address_size = sizeof(client_address);

    int client_socket = ::accept(this->socket, (struct sockaddr*) &client_address, &client_address_size);
    
    std::cout << client_socket << std::endl;

    this->clients[client_socket] = client_address;
    
    
}

void Communication::TCP_Server::send(std::string msg)
{
    for(const auto &[client_socket, client_address]: this->clients)
    {
        //char msg[] = "Hello world\n";
        ::write(client_socket, msg.c_str(), msg.length());
    }
}

std::pair<int, std::string> Communication::TCP_Server::receive()
{
    char msg[30];
    int str_len;
    std::string msg_ret = "";
    int client_socket_latest;
    for(const auto &[client_socket, client_address]: this->clients)
    {
        client_socket_latest = client_socket;
    }
    str_len = ::read(client_socket_latest, msg, sizeof(msg));    
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


void Communication::TCP_Server::regist_task(std::function<void(void)> task)
{
    this->service_task = task;
    
}

void Communication::TCP_Server::response()
{
    auto [ret, dummy] = this->receive();
    if(ret > 0)
    {
        //this->service_task();
        //std::thread service_thread = std::thread(this->service_task);
        //service_thread.join();
        //std::cout << "Async start" << std::endl;
        std::future<void> service_async = std::async(std::launch::async, this->service_task);
        //service_async.wait();
    }
    else if(ret == 0)
    {
        this->connect();
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
    std::cout << "Client socket generated: " << std::flush;
    this->socket = ::socket(PF_INET, SOCK_STREAM, 0);
    std::cout << this->socket << std::endl;
}

Communication::TCP_Client::~TCP_Client() {};

void Communication::TCP_Client::connect(std::string address)
{
    this->connect(address, Communication::TCP_DEFAULT_PORT);
}
void Communication::TCP_Client::connect(std::string address, int port)
{
    
    std::cout << "Client connected: " << std::flush;
    struct sockaddr_in server_address;
    std::memset(&(server_address), 0, sizeof(server_address));
    server_address.sin_family = AF_INET;    
    server_address.sin_addr.s_addr = ::inet_addr(address.c_str());
    server_address.sin_port = ::htons(port);

    int ret = ::connect(this->socket, (struct sockaddr*)&server_address, sizeof(server_address));
    
    std::cout << ret << std::endl;
    
}

void Communication::TCP_Client::receive()
{
    char msg[30];
    int str_len;
    str_len = ::read(this->socket, msg, sizeof(msg));

    std::cout << msg << std::endl;
}
void Communication::TCP_Client::request()
{
    std::cout << "Client request: " << std::flush;
    char msg[] = "Hello Server";
    int ret = ::write(this->socket, msg, sizeof(msg));
    std::cout << ret << std::endl;
}


void Communication::TCP_Client::run()
{

}