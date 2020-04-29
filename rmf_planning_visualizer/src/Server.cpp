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

#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/schedule/Database.hpp>
#include <rmf_traffic/schedule/Participant.hpp>

#include <building_map_msgs/msg/graph_edge.hpp>

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
  // Check if planning components have already been set up
  // TODO: worry about updating the graph and planner later, for now the server
  // needs to be restarted everytime the map server changes the map.
  if (_planning_components)
    return;

  RCLCPP_INFO(_node->get_logger(), "Setting up graph.");
  std::unordered_map<std::string, std::vector<rmf_traffic::agv::Graph>>
      graph_map;
  std::string first_level_name = "";

  for (const auto& l : msg->levels)
  {
    std::vector<rmf_traffic::agv::Graph> graph_set;
    for (const auto& g : l.nav_graphs)
    {
      rmf_traffic::agv::Graph new_graph;
      for (const auto& v : g.vertices)
      {
        new_graph.add_waypoint(l.name, {v.x, v.y});
      }
      for (const auto& e : g.edges)
      {
        using GraphEdge = building_map_msgs::msg::GraphEdge;
        if (e.edge_type == GraphEdge::EDGE_TYPE_BIDIRECTIONAL)
        {
          new_graph.add_lane(e.v1_idx, e.v2_idx);
          new_graph.add_lane(e.v2_idx, e.v1_idx);
        }
        else
        {
          new_graph.add_lane(e.v1_idx, e.v2_idx);
        }
      }

      graph_set.push_back(new_graph);

      // this is to make sure we select a default level and graph index that is
      // populated
      if (first_level_name == "")
        first_level_name = l.name;
    }
    graph_map[l.name] = graph_set;
  }

  // Inital setup with default values
  const auto vehicle_profile = rmf_traffic::Profile {
      rmf_traffic::geometry::make_final_convex<
          rmf_traffic::geometry::Circle>(0.3)};
  const auto vehicle_traits = rmf_traffic::agv::VehicleTraits {
      {0.4, 3.0}, 
      {0.4, 4.0}, 
      vehicle_profile};
  const rmf_traffic::agv::Planner::Configuration planner_config {
      graph_map[first_level_name][0], 
      vehicle_traits};
  rmf_traffic::schedule::Database database;
  auto participant = rmf_traffic::schedule::make_participant(
      rmf_traffic::schedule::ParticipantDescription {
          first_level_name,
          "planning_visualizer",
          rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
          vehicle_profile},
      database);
  rmf_traffic::agv::Planner planner(
      planner_config,
      rmf_traffic::agv::Planner::Options(
          rmf_utils::make_clone<rmf_traffic::agv::ScheduleRouteValidator>(
              database, participant.id(), vehicle_profile)));

  std::lock_guard<std::mutex> guard(_planning_mutex);
  _planning_components.reset(new PlanningComponents {
    std::move(first_level_name),
    0,
    std::move(vehicle_profile),
    std::move(vehicle_traits),
    std::move(graph_map),
    std::move(database),
    std::move(participant),
    std::move(planner)});

  _inspector = Inspector::make(_planning_components->planner);
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
    case RequestType::PlannerConfig:
      get_planner_config_response(msg, response); break;
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
  std::string msg_payload = msg->get_payload();

  try
  {
    json j = json::parse(msg_payload);

    if (j.size() != 2 || j.count("request") != 1 || j.count("param") != 1)
      return RequestType::Undefined;

    if (j["request"] == "forward")
      return RequestType::Forward;
    else if (j["request"] == "backward")
      return RequestType::Backward;
    else if (j["request"] == "step_index" && j["param"].count("index") == 1)
      return RequestType::StepIndex;
    else if (j["request"] == "config" &&
        j["param"].count("linear_velocity") == 1 &&
        j["param"].count("linear_acceleration") == 1 &&
        j["param"].count("angular_velocity") == 1 &&
        j["param"].count("angular_acceleration") == 1 &&
        j["param"].count("profile_shape") == 1 &&
        j["param"].count("profile_radius") == 1 &&
        j["param"].count("linear_velocity") == 1 &&
        j["param"].count("level_name") == 1 &&
        j["param"].count("graph_index") == 1 &&
        j["param"].count("linear_velocity") == 1)
      return RequestType::PlannerConfig;
    else
      return RequestType::Undefined;
  }
  catch(const std::exception& e)
  {
    RCLCPP_ERROR(
        _node->get_logger(),
        "Error: %s",
        std::to_string(*e.what()).c_str());
    return RequestType::Undefined;
  }
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
  std::string msg_payload = msg->get_payload();
  json j_req = json::parse(msg_payload);

  auto planning_state = _inspector->get_state(
      static_cast<std::size_t>(j_req["param"]["index"]));
  
  json j_res = _j_res;
  j_res["response"] = "step_index";
  if (!planning_state)
    j_res["error"] = "Unable to get planning state using index, please "
        "ensure index is within range.";
  else
    j_res["values"] = parse_planning_state(planning_state);
  
  response = j_res.dump();
}

//==============================================================================

void Server::get_planner_config_response(
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
