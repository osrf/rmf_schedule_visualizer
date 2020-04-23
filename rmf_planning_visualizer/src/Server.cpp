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
  _ws_server_thread = std::thread([&]()
  {
    this->_ws_server.run();
  });

  _is_initialized = true;
}

//==============================================================================

Server::Server(Inspector::SharedPtr inspector)
: _inspector(std::move(inspector))
{}

//==============================================================================

Server::~Server()
{
  if (!_is_initialized)
    return;

  // Thread safe access to _ws_connections
  const auto connection_copies = _ws_connections;
  for (auto& c : connection_copies)
  {
    _ws_server.close(c, websocketpp::close::status::normal, "shutdown");
  }

  if (_ws_server_thread.joinable())
  {
    _ws_server.stop();
    _ws_server_thread.join();
  }
}

//==============================================================================

void Server::on_message(connection_hdl hdl, server::message_ptr msg)
{
  if (msg->get_payload().empty())
  {
    printf("Empty request received...\n");
    return;
  }

  auto request_type = get_request_type(msg);
  std::string response = "";
  switch(request_type)
  {
    case RequestType::Forward: 
      get_forward_response(msg, response); break;
    case RequestType::Backward: 
      get_backward_response(msg, response); break;
    case RequestType::StepIndex:
      get_step_index_response(msg, response); break;
    default: {
      printf("Undefined request type received...\n");
      break;
    }
  }

  printf("Response: %s", response.c_str());
  server::message_ptr response_msg = std::move(msg);
  response_msg->set_payload(response);
  _ws_server.send(hdl, response_msg);
}

//==============================================================================

auto Server::get_request_type(const server::message_ptr& msg) 
    -> Server::RequestType
{
  return RequestType::Forward;
}

//==============================================================================

void Server::get_forward_response(
    const server::message_ptr& msg, std::string& response)
{
}

//==============================================================================

void Server::get_backward_response(
    const server::message_ptr& msg, std::string& response)
{
}

//==============================================================================

void Server::get_step_index_response(
    const server::message_ptr& msg, std::string& response)
{
}

//==============================================================================

} // planning
} // rmf_visualizer
