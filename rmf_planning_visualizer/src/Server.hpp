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

#ifndef RMF_PLANNING_VISUALIZER__SRC__SERVER__SERVER_HPP
#define RMF_PLANNING_VISUALIZER__SRC__SERVER__SERVER_HPP

#include <set>
#include <memory>

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#include <visualizer_utils/json.hpp>

#include <rmf_traffic/agv/Planner.hpp>

#include "Inspector.hpp"

namespace rmf_visualizer {
namespace planning {

class Server
{

public:

  using SharedPtr = std::shared_ptr<Server>;
  using Planner = rmf_traffic::agv::Planner;
  using server = websocketpp::server<websocketpp::config::asio>;
  using connection_hdl = websocketpp::connection_hdl;

  static SharedPtr make(uint16_t port, Inspector::SharedPtr inspector);

  ~Server();

  void init(uint16_t port);

  enum class RequestType : uint8_t
  {
    Forward,
    Backward,
    StepIndex
  };

private:

  bool _is_initialized = false;

  Inspector::SharedPtr _inspector;

  // Websocket plumbing
  server _ws_server;
  std::set<connection_hdl, std::owner_less<connection_hdl>> _ws_connections;
  uint16_t _ws_port;
  std::thread _ws_server_thread;

  void on_message(connection_hdl hdl, server::message_ptr msg);

  RequestType get_request_type(const server::message_ptr& msg);

  void get_forward_response(
      const server::message_ptr& msg, std::string& response);

  void get_backward_response(
      const server::message_ptr& msg, std::string& response);

  void get_step_index_response(
      const server::message_ptr& msg, std::string& response);

  Server(Inspector::SharedPtr inspector);

  // Templates used for response generation
};

} // namespace planning
} // namespace rmf_visualizer

#endif // RMF_PLANNING_VISUALIZER__SRC__SERVER__SERVER_HPP
