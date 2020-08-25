#include "tcp_client_connection.hpp"

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

bool TCPClientConnection::send(void* data){
    // int sent_length = write(socket_desc_, data, data);
    // if (sent_length < 0) {
    //     return false;
    // }  // TODO(resizoltan) handle other kind of errors?
    // return true;
}