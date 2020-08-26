#ifndef TCP_SERVER_CONNECTION_HPP_
#define TCP_SERVER_CONNECTION_HPP_

#include "tcp_connection.hpp"

class TCPServerConnection: public TCPConnection{
    int client_sock_desc_ = 0;
    void init();
    void listen();
public:
    explicit TCPServerConnection(
    const char * server_addr, const int server_port,
    std::function<void(void *)> data_received_callback,
    std::function<void(const char *, const int)> connection_lost_callback);
    bool send(void*, int);
};

#endif