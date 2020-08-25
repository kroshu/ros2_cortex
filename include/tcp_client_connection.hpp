#ifndef TCP_CLIENT_CONNECTION_HPP_
#define TCP_CLIENT_CONNECTION_HPP_

#include "tcp_connection.hpp"

class TCPClientConnection: public TCPConnection{
    void init();
    void listen();
public:
    bool send(void*);
};

#endif