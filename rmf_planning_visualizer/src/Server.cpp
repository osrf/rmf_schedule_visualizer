/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <cstdio>

#include "Server.hpp"

namespace rmf_visualizer {
namespace planning {

//==============================================================================

auto Server::make(uint16_t port, Inspector::SharedPtr inspector)
    -> Server::SharedPtr
{
  Server::SharedPtr server(new Server(std::move(inspector)));
  try
  {
    server->init(port);
  }
  catch (std::exception& e)
  {
    std::cerr << "Failed to start the Server: " << e.what() << std::endl;
    return nullptr;
  }
  return server;
}

//==============================================================================

void Server::init(uint16_t port)
{
  using websocketpp::lib::placeholders::_1;
  using websocketpp::lib::placeholders::_2;
  using websocketpp::lib::bind;

  _ws_server.init_asio();
  _ws_server.set_open_handler([&](connection_hdl hdl)
  {
    _ws_connections.insert(hdl);
    printf("Connected with a client...\n");
  });
  _ws_server.set_close_handler([&](connection_hdl hdl)
  {
    _ws_connections.erase(hdl);
    printf("Disconnected with a client...\n");
  });
  _ws_server.set_message_handler(bind(&Server::on_message, this, _1, _2));

  _ws_server.set_reuse_addr(true);
  _ws_server.listen(port);
  _ws_server.start_accept();
  _server_thread = std::thread([&]()
  {
    this->_ws_server.run();
  });

  _is_initialized = true;
}

//==============================================================================

void Server::on_message(connection_hdl hdl, server::message_ptr msg)
{
  std::string response = "";

  if (msg->get_payload().empty())
  {
    printf("Empty request received...\n");
    return;
  }
}

//==============================================================================

Server::Server(Inspector::SharedPtr inspector)
: _inspector(std::move(inspector))
{}

//==============================================================================

Server::~Server()
{}

} // planning
} // rmf_visualizer
