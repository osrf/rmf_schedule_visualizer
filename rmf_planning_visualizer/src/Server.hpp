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

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#include <rmf_traffic/agv/Graph.hpp>
#include <rmf_traffic/agv/Planner.hpp>
#include <rmf_traffic/schedule/Database.hpp>
#include <rmf_traffic/schedule/Participant.hpp>

#include <visualizer_utils/json.hpp>

#include "Inspector.hpp"
#include "ParseGraph.hpp"

namespace rmf_visualizer {
namespace planning {

class Server
{

public:

  using SharedPtr = std::shared_ptr<Server>;
  using server = websocketpp::server<websocketpp::config::asio>;
  using connection_hdl = websocketpp::connection_hdl;
  using json = nlohmann::json;

  static SharedPtr make();

  ~Server();

  void init();

  // This is a blocking call.
  void run(uint16_t port);

  enum class RequestType : uint8_t
  {
    PlannerConfig,
    StartPlanning,
    Step,
    ClosePlanner,
    Undefined
  };

  struct PlanningInstance
  {
    std::string id;

    rmf_traffic::Profile vehicle_profile;

    rmf_traffic::agv::VehicleTraits vehicle_traits;

    GraphInfo graph_info;

    std::shared_ptr<rmf_traffic::schedule::Database> database;

    rmf_traffic::schedule::Participant participant;

    rmf_traffic::agv::Planner planner;

    Inspector::SharedPtr inspector;
  };

private:

  bool _is_initialized = false;

  // Planning server components
  std::size_t _planning_step = 0;
  std::unique_ptr<PlanningInstance> _planning_instance;

  // Websocket plumbing
  uint16_t _ws_port;
  server _ws_server;
  std::set<connection_hdl, std::owner_less<connection_hdl>> _ws_connections;

  void on_message(connection_hdl hdl, server::message_ptr msg);

  RequestType get_request_type(const server::message_ptr& msg);

  void get_planner_config_response(
      const server::message_ptr& msg, std::string& response);

  void get_start_planning_response(
      const server::message_ptr& msg, std::string& response);

  void get_step_response(
      const server::message_ptr& msg, std::string& response);

  void get_close_planner_response(
      const server::message_ptr& msg, std::string& response);

  void parse_planning_state(
      const Inspector::ConstPlanningStatePtr& state,
      json& j_res) const;

  Server();

  // Templates used for response generation
  const json _j_res = 
      { {"response", {}}, {"id", {}}, {"values", {}}, {"error", {}} };
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
