// Copyright 2020 Zoltán Rési
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TCP_CONNECTION_HPP_
#define TCP_CONNECTION_HPP_

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <pthread.h>

#include <stdexcept>
#include <string>
#include <functional>
#include <vector>
#include <atomic>

class TCPConnection
{
public:
  explicit TCPConnection(
    const char * server_addr, const int server_port,
    std::function<void(void *)> data_received_callback,
    std::function<void(const char *, const int)> connection_lost_callback);

  virtual bool send(void*, int);
  void closeConnection();

  ~TCPConnection();
  TCPConnection(const TCPConnection &) = delete;
  TCPConnection & operator=(const TCPConnection &) = delete;
  TCPConnection & operator=(TCPConnection && from);
  
protected:
  virtual void init() = 0;
  static void * listen_helper(void * tcpConnection);
  virtual void listen() = 0;
  std::function<void(void *)> dataReceivedCallback_;
  std::function<void(const char *, int)> connectionLostCallback_;

  int socket_desc_;
  struct sockaddr_in server_;
  pthread_t read_thread_;
  std::atomic_bool cancelled_;
  const int max_buffer_size_ = 5000;
  bool connected_;
};

#endif  // KUKA_SUNRISE__TCP_CONNECTION_HPP_
