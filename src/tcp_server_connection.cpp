#include "tcp_server_connection.hpp"

void TCPServerConnection::init(){

}

void TCPServerConnection::listen(){
    std::uint8_t msg_buffer[max_buffer_size_];
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
            std::vector<std::uint8_t> server_msg(msg_buffer, msg_buffer + length);
      		dataReceivedCallback_(server_msg.data());
        }
    }
}

bool TCPServerConnection::send(void* data){
}