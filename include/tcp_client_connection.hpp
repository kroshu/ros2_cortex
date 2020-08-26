#ifndef TCP_CLIENT_CONNECTION_HPP_
#define TCP_CLIENT_CONNECTION_HPP_

#include "tcp_connection.hpp"

class TCPClientConnection: public TCPConnection{
    void init();
    void listen();
public:
    explicit TCPClientConnection(
    const char * server_addr, const int server_port,
    std::function<void(void *)> data_received_callback,
    std::function<void(const char *, const int)> connection_lost_callback);
};

#endif