#ifndef TCP_SERVER_CONNECTION_HPP_
#define TCP_SERVER_CONNECTION_HPP_

#include "tcp_connection.hpp"

class TCPServerConnection: public TCPConnection{
    void init();
    void listen();
public:
    bool send(void*);
};

#endif