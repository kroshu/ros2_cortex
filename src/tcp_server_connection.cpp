#include <iostream>

#include "tcp_server_connection.hpp"

TCPServerConnection::TCPServerConnection(
    const char * server_addr, const int server_port,
    std::function<void(void *)> data_received_callback,
    std::function<void(const char *, const int)> connection_lost_callback):
    TCPConnection(server_addr, server_port,
    data_received_callback,
    connection_lost_callback){
    init();
    }

void TCPServerConnection::init(){
    int addrlen = sizeof(server_);
    if(bind(socket_desc_, (struct sockaddr *)&server_, (socklen_t)addrlen)<0){
		std::cerr << "Bind failed" << std::endl; 
        return;
	}
	if(::listen(socket_desc_,3) < 0){
		std::cerr << "Listening failed" << std::endl;
        return;
	}
	if((client_sock_desc_ = accept(socket_desc_, (struct sockaddr *)&server_, (socklen_t *)&addrlen)) < 0){
		std::cerr << "Accepting failed" << std::endl;
        return;
	}

    connected_ = true;
    pthread_create(&read_thread_, NULL, &TCPConnection::listen_helper, this);
}

void TCPServerConnection::listen(){
    std::uint8_t msg_buffer[max_buffer_size_];
    while (!cancelled_.load()) {
        int length = recv(client_sock_desc_, msg_buffer, max_buffer_size_, 0);
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

bool TCPServerConnection::send(void* data, int data_len){
    int sent_length = -1;
    try
    {
      sent_length = ::send(client_sock_desc_, data, data_len, 0);
    }
    catch(...)
    {
      std::cerr << "e.what()" << '\n';
    }
    
    if (sent_length < 0) {
        return false;
    }  // TODO(resizoltan) handle other kind of errors?
    return true;
}