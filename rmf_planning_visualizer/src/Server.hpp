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
#include <mutex>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <building_map_msgs/msg/building_map.hpp>

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#include <rmf_traffic/agv/Graph.hpp>
#include <rmf_traffic/agv/Planner.hpp>
#include <rmf_traffic/schedule/Database.hpp>
#include <rmf_traffic/schedule/Participant.hpp>

#include <visualizer_utils/json.hpp>

#include "Inspector.hpp"

namespace rmf_visualizer {
namespace planning {

class Server
{

public:

  using server = websocketpp::server<websocketpp::config::asio>;
  using connection_hdl = websocketpp::connection_hdl;
  using json = nlohmann::json;

  using BuildingMap = building_map_msgs::msg::BuildingMap;

  static std::shared_ptr<Server> make(std::string node_name, uint16_t port);

  ~Server();

  void init_ros(std::string node_name);

  void init_websocket(uint16_t port);

  void start();

  enum class RequestType : uint8_t
  {
    PlannerConfig,
    Forward,
    Backward,
    StepIndex,
    Undefined
  };

  struct PlanningComponents
  {
    std::string level_name;

    // TODO: remember to swap out to use std::string when we start using strings
    // instead of indices.
    size_t graph_index;

    rmf_traffic::Profile vehicle_profile;

    rmf_traffic::agv::VehicleTraits vehicle_traits;

    std::unordered_map<std::string, std::vector<rmf_traffic::agv::Graph>> 
        graph_map;

    rmf_traffic::schedule::Database database;

    rmf_traffic::schedule::Participant participant;

    rmf_traffic::agv::Planner planner;
  };

private:

  bool _is_initialized = false;

  // Planning server components
  std::mutex _planning_mutex;
  std::unique_ptr<PlanningComponents> _planning_components;
  Inspector::SharedPtr _inspector;

  // ROS2 plumbing
  rclcpp::Node::SharedPtr _node;
  rclcpp::Subscription<BuildingMap>::SharedPtr _map_sub;

  void update_graph(BuildingMap::UniquePtr msg);

  // Websocket plumbing
  server _ws_server;
  std::set<connection_hdl, std::owner_less<connection_hdl>> _ws_connections;
  uint16_t _ws_port;
  std::thread _ws_server_thread;

  void on_message(connection_hdl hdl, server::message_ptr msg);

  RequestType get_request_type(const server::message_ptr& msg);

  void get_planner_config_response(
        const server::message_ptr& msg, std::string& response);

  void get_forward_response(
      const server::message_ptr& msg, std::string& response);

  void get_backward_response(
      const server::message_ptr& msg, std::string& response);

  void get_step_index_response(
      const server::message_ptr& msg, std::string& response);

  std::string parse_planning_state(
      const Inspector::ConstPlanningStatePtr& state) const;

  Server();

  // Templates used for response generation
  const json _j_res = { {"response", {}}, {"values", {}}, {"error", {}} };
  const json _j_traj =
      { {"id", {}}, {"shape", {}}, {"dimensions", {}}, {"segments", {}}};
  const json _j_seg = { {"x", {}}, {"v", {}}, {"t", {}}};

  const json _j_traits = 
      { {"radius", {}}, {"linear_limits", {}}, {"angular_limits", {}} };
  const json _j_limits = { {"velocity", {}}, {"acceleration", {}} };
};

} // namespace planning
} // namespace rmf_visualizer

#endif // RMF_PLANNING_VISUALIZER__SRC__SERVER__SERVER_HPP
