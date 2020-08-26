#include "tcp_client_connection.hpp"

TCPClientConnection::TCPClientConnection(
    const char * server_addr, const int server_port,
    std::function<void(void *)> data_received_callback,
    std::function<void(const char *, const int)> connection_lost_callback):
    TCPConnection(server_addr, server_port,
    data_received_callback,
    connection_lost_callback){
    init();
    }

void TCPClientConnection::init(){
    if (connect(socket_desc_, (struct sockaddr *)&server_, sizeof(server_))) {
        throw std::runtime_error("Could not connect to server");
    }
    connected_ = true;
    pthread_create(&read_thread_, NULL, &TCPConnection::listen_helper, this);
}

void TCPClientConnection::listen(){
    char msg_buffer[max_buffer_size_];
    while (!cancelled_.load()) {
        int length = recv(socket_desc_, msg_buffer, max_buffer_size_, 0);
        if (length < 0) {
            if (cancelled_.load()) {
                break;
            }
            // TODO handle error
        } else if (length == 0) {  // TODO is this the way to check for connection loss?
            connected_ = false;
            connectionLostCallback_(inet_ntoa(server_.sin_addr), ntohs(server_.sin_port));
            break;
        } else {
            msg_buffer[length] = '\0';
            dataReceivedCallback_(msg_buffer);
        }
    }
}