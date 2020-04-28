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
#include <chrono>

#include "Server.hpp"

namespace rmf_visualizer {
namespace planning {

//==============================================================================

auto Server::make(std::string node_name, uint16_t port)
    -> std::shared_ptr<Server>
{
  std::shared_ptr<Server> server(new Server);

  try
  {
    server->init_ros(std::move(node_name));
    server->init_websocket(port);
  }
  catch (std::exception& e)
  {
    std::cerr << "Failed to start the Server: " << e.what() << std::endl;
    return nullptr;
  }
  return server;
}

//==============================================================================

void Server::init_ros(std::string node_name)
{
  _node = rclcpp::Node::make_shared(std::move(node_name));

  // Warm start to get the building map
  auto transient_qos_profile = rclcpp::QoS(1);
  transient_qos_profile.transient_local();
  _map_sub = _node->create_subscription<BuildingMap>(
      "/map",
      transient_qos_profile,
      [&](BuildingMap::UniquePtr msg)
      {
        update_graph(std::move(msg));
      });
  
  _planning_components = nullptr;
  const auto stop_time =
      std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < stop_time)
  {
    rclcpp::spin_some(_node);
    if (_planning_components)
      break;
  }
  if (!_planning_components)
    throw std::runtime_error(
        "Unable to initialize: [No BuildingMap message received.]");
}

//==============================================================================

void Server::init_websocket(uint16_t port)
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
}

//==============================================================================

void Server::start()
{
  _ws_server.start_accept();
  _ws_server_thread = std::thread([&]()
  {
    this->_ws_server.run();
  });

  rclcpp::spin(_node);
}

//==============================================================================

Server::Server()
{}

//==============================================================================

Server::~Server()
{
  rclcpp::shutdown();

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

void Server::update_graph(BuildingMap::UniquePtr msg)
{
  std::lock_guard<std::mutex> guard(_planning_mutex);
  std::unordered_map<std::string, Server::PlanningComponents::Graphs>
      graph_map;
  
  for (const auto& l : msg->levels)
  {
    Server::PlanningComponents::Graphs new_graphs;
    for (const auto& g : l.nav_graphs)
    {
      rmf_traffic::agv::Graph new_graph;
    }
  }

  RCLCPP_INFO(_node->get_logger(), "updating graph.");
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
    case RequestType::PlannerConfig:
      get_planner_config_response(msg, response); break;
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

void Server::get_planner_config_response(
      const server::message_ptr& msg, std::string& response)
{
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

std::string Server::parse_planning_state(
    const Inspector::ConstPlanningStatePtr& state) const
{
  std::string result;
  return result;
}

//==============================================================================

} // planning
} // rmf_visualizer
