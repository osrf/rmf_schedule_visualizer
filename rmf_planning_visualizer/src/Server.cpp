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

#include <chrono>
#include <iostream>

#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/schedule/Database.hpp>
#include <rmf_traffic/schedule/Participant.hpp>

#include "Server.hpp"

namespace rmf_visualizer {
namespace planning {

//==============================================================================

auto Server::make() -> SharedPtr
{
  std::shared_ptr<Server> server(new Server);
  try
  {
    server->init();
  }
  catch (std::exception& e)
  {
    std::cerr << "Failed to start the Server: " << e.what() << std::endl;
    return nullptr;
  }
  return server;
}

//==============================================================================

void Server::init()
{
  using websocketpp::lib::placeholders::_1;
  using websocketpp::lib::placeholders::_2;
  using websocketpp::lib::bind;

  _ws_server.init_asio();
  _ws_server.set_open_handler([&](connection_hdl hdl)
  {
    _ws_connections.insert(hdl);
    std::cout << "Connected with a client..." << std::endl;
  });
  _ws_server.set_close_handler([&](connection_hdl hdl)
  {
    _ws_connections.erase(hdl);
    std::cout << "Disconnected with a client..." << std::endl;
  });
  _ws_server.set_message_handler(bind(&Server::on_message, this, _1, _2));

  _is_initialized = true;
}

//==============================================================================

void Server::run(uint16_t port)
{
  assert(_is_initialized);
  _ws_server.set_reuse_addr(true);
  _ws_server.listen(port);
  _ws_server.start_accept();

  std::cout << "Planning Visualizer server starting with websocket port [" 
      << port << "]" << std::endl;
  _ws_server.run();
}

//==============================================================================

Server::Server()
{}

//==============================================================================

Server::~Server()
{
  if (!_is_initialized)
    return;

  std::cout << "Planning Visualizer server shutting down" << std::endl;
  for (auto& c : _ws_connections)
  {
    _ws_server.close(c, websocketpp::close::status::normal, "shutdown");
  }
  _ws_server.stop();
}

//==============================================================================

void Server::on_message(connection_hdl hdl, server::message_ptr msg)
{
  if (msg->get_payload().empty())
  {
    std::cerr << "Empty request received..." << std::endl;
    return;
  }

  auto request_type = get_request_type(msg);
  std::string response = "";
  switch(request_type)
  {
    case RequestType::PlannerConfig:
      get_planner_config_response(msg, response); break;
    case RequestType::StartPlanning:
      get_start_planning_response(msg, response); break;
    case RequestType::Forward: 
      get_forward_response(msg, response); break;
    case RequestType::Backward: 
      get_backward_response(msg, response); break;
    case RequestType::ClosePlanner:
      get_close_planner_response(msg, response); break;
    default: {
      std::cerr << "Undefined request type received..." << std::endl;
      break;
    }
  }

  std::cout << "Response: " << response << std::endl;
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

    if (j["request"] == "planner_config" &&
        j["param"].count("id") == 1 &&
        j["param"].count("profile_radius") == 1 &&
        j["param"].count("linear_velocity") == 1 &&
        j["param"].count("linear_acceleration") == 1 &&
        j["param"].count("angular_velocity") == 1 &&
        j["param"].count("angular_acceleration") == 1 &&
        j["param"].count("graph_file_path") == 1)
      return RequestType::PlannerConfig;
    else if (j["request"] == "start_planning" && 
        j["param"].count("id") == 1 &&
        j["param"].count("start") == 1 &&
        j["param"]["start"].count("x") == 1 &&
        j["param"]["start"].count("y") == 1 && 
        j["param"]["start"].count("yaw") == 1 &&
        j["param"].count("goal") == 1)
      return RequestType::StartPlanning;
    else if (j["request"] == "forward" && j["param"].count("id") == 1)
      return RequestType::Forward;
    else if (j["request"] == "backward" && j["param"].count("id") == 1)
      return RequestType::Backward;
    else if (j["request"] == "close_planner" && j["param"].count("id") == 1)
      return RequestType::ClosePlanner;
    else
      return RequestType::Undefined;
  }
  catch(const std::exception& e)
  {
    std::cerr << "Error: " << std::to_string(*e.what()) << std::endl;
    return RequestType::Undefined;
  }
}

//==============================================================================

void Server::get_planner_config_response(
      const server::message_ptr& msg, std::string& response)
{
  std::string msg_payload = msg->get_payload();
  json j_req = json::parse(msg_payload);
  auto j_param = j_req["param"];

  json j_res = _j_res;
  j_res["response"] = "planner_config";

  // TODO: handle shapes, default to circle for now.
  const double profile_radius = j_param["profile_radius"];
  if (profile_radius < 0.0)
  {
    j_res["error"] = "Profile radius must be positive.";
    response = j_res.dump();
    return;
  }
  const auto profile = rmf_traffic::Profile {
      rmf_traffic::geometry::make_final_convex<
          rmf_traffic::geometry::Circle>(profile_radius)};

  const double linear_v = j_param["linear_velocity"];
  const double linear_a = j_param["linear_acceleration"];
  const double angular_v = j_param["angular_velocity"];
  const double angular_a = j_param["angular_acceleration"];
  if (linear_v < 0.0 || linear_a < 0.0 || angular_v < 0.0 || angular_a < 0.0)
  {
    j_res["error"] = "Velocities and accelerations must be positive.";
    response = j_res.dump();
    return;
  }
  const auto traits = rmf_traffic::agv::VehicleTraits{
      {linear_v, linear_a}, {angular_v, angular_a}, profile};

  const auto graph_info = parse_graph(j_param["graph_file_path"], traits);
  if (!graph_info.has_value())
  {
    j_res["error"] = "Failed to parse graph file.";
    response = j_res.dump();
    return;
  }
  
  const rmf_traffic::agv::Planner::Configuration planner_config {
      graph_info.value().graph, traits};

  rmf_traffic::schedule::Database database;

  std::string planner_id = j_param["id"];
  auto participant = rmf_traffic::schedule::make_participant(
      rmf_traffic::schedule::ParticipantDescription {
          planner_id,
          "rmf_planning_visualizer",
          rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
          profile},
      database);

  rmf_traffic::agv::Planner planner(
      planner_config,
      rmf_traffic::agv::Planner::Options(
          rmf_utils::make_clone<rmf_traffic::agv::ScheduleRouteValidator>(
              database, participant.id(), profile)));

  _planning_instance.reset(new PlanningInstance {
      std::move(profile),
      std::move(traits),
      std::move(graph_info.value()),
      std::move(database),
      std::move(participant),
      std::move(planner)});
}

//==============================================================================

void Server::get_start_planning_response(
    const server::message_ptr& msg, std::string& response)
{
  // std::string msg_payload = msg->get_payload();
  // json j_req = json::parse(msg_payload);
  // json j_start = j_req["param"]["start"];
  // const std::size_t goal_index = j_req["param"]["goal"];
  
  // const auto time = std::chrono::steady_clock::now();
  // bool started = false;
  // Inspector::ConstPlanningStatePtr planning_state = nullptr;

  // {
  //   std::lock_guard<std::mutex> guard(_planning_mutex);
  //   const std::string l_name = _planning_components->level_name;
  //   const std::size_t graph_index = _planning_components->graph_index;
  //   auto starts = rmf_traffic::agv::compute_plan_starts(
  //       _planning_components->graph_map[l_name][graph_index],
  //       {j_start["x"], j_start["y"], j_start["yaw"]},
  //       time_now);
    
  //   _inspector = Inspector::make(_planning_components->planner);
  //   started = _inspector->begin(
  //       starts, 
  //       {goal_index}, 
  //       _planning_components->planner.get_default_options());

  //   planning_state = _inspector->get_state();
  //   _planning_step = 0;
  // }

  // json j_res = _j_res;
  // j_res["response"] = "start_planning";
  // if (!started)
  //   j_res["error"] = "Unable to start planning from provided parameters.";
  // else if (!planning_state)
  //   j_res["error"] = "Unable to retrieve planning state.";
  // else
  //   j_res["values"] = parse_planning_state(planning_state);

  // response = j_res.dump();
}

//==============================================================================

void Server::get_forward_response(
    const server::message_ptr& msg, std::string& response)
{
  // std::string msg_payload = msg->get_payload();
  // json j_req = json::parse(msg_payload);

  // Inspector::ConstPlanningStatePtr planning_state = nullptr;

  // std::lock_guard<std::mutex> guard(_planning_mutex);
  // const std::size_t total_steps_taken = _inspector->step_num();
  // if (_planning_step < total_steps_taken)
  //   planning_state = _inspector->get_state(++_planning_step);
  // if (_)


  // json j_res = _j_res;
  // response = j_res.dump();
}

//==============================================================================

void Server::get_backward_response(
    const server::message_ptr& msg, std::string& response)
{
  std::string msg_payload = msg->get_payload();
  json j_req = json::parse(msg_payload);
}

//==============================================================================

void Server::get_close_planner_response(
    const server::message_ptr& msg, std::string& response)
{
  std::string msg_payload = msg->get_payload();
  json j_req = json::parse(msg_payload);
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
