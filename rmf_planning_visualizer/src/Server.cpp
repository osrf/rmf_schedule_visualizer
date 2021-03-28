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

  std::cout << "Planning Visualizer server starting on port [" << port 
      << "]" << std::endl;
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
    case RequestType::Step: 
      get_step_response(msg, response); break;
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

    if (j.size() != 3 || 
        j.count("request") != 1 || 
        j.count("id") != 1 ||
        j.count("param") != 1)
      return RequestType::Undefined;

    if (j["request"] == "planner_config" &&
        j["param"].count("profile_radius") == 1 &&
        j["param"].count("linear_velocity") == 1 &&
        j["param"].count("linear_acceleration") == 1 &&
        j["param"].count("angular_velocity") == 1 &&
        j["param"].count("angular_acceleration") == 1 &&
        j["param"].count("graph_file_path") == 1)
      return RequestType::PlannerConfig;
    else if (j["request"] == "start_planning" &&
        j["param"].count("start") == 1 &&
        j["param"]["start"].count("x") == 1 &&
        j["param"]["start"].count("y") == 1 && 
        j["param"]["start"].count("yaw") == 1 &&
        j["param"].count("goal") == 1)
      return RequestType::StartPlanning;
    else if (j["request"] == "step")
      return RequestType::Step;
    else if (j["request"] == "close_planner")
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
  std::string planning_instance_id = j_req["id"];
  json j_param = j_req["param"];

  json j_res = _j_res;
  j_res["response"] = "planner_config";
  j_res["id"] = planning_instance_id;

  if (_planning_instance)
  {
    j_res["error"] = 
        "Already running a planning instance of id: [" +
        _planning_instance->id + "].";
    response = j_res.dump();
    return;
  }

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

  auto database = std::make_shared<rmf_traffic::schedule::Database>();

  auto participant = rmf_traffic::schedule::make_participant(
      rmf_traffic::schedule::ParticipantDescription {
          planning_instance_id,
          "rmf_planning_visualizer",
          rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
          profile},
      database);

  rmf_traffic::agv::Planner planner(
      planner_config,
      rmf_traffic::agv::Planner::Options(
          rmf_utils::make_clone<rmf_traffic::agv::ScheduleRouteValidator>(
              database, participant.id(), profile)));

  auto inspector_ptr = Inspector::make(planner);

  _planning_instance.reset(new PlanningInstance {
      std::move(planning_instance_id),
      std::move(profile),
      std::move(traits),
      std::move(graph_info.value()),
      database,
      std::move(participant),
      std::move(planner),
      inspector_ptr});

  // TODO: Provide a more specific success response, other than no-error
  j_res["values"] = "";
  response = j_res.dump();
}

//==============================================================================

void Server::get_start_planning_response(
    const server::message_ptr& msg, std::string& response)
{
  json j_res = _j_res;
  j_res["response"] = "start_planning";
  if (!_planning_instance)
  {
    j_res["error"] = "Planning instance has not yet been started.";
    response = j_res.dump();
    return;
  }

  std::string msg_payload = msg->get_payload();
  json j_req = json::parse(msg_payload);
  std::string planning_instance_id = j_req["id"];
  json j_param = j_req["param"];

  // TODO: Allow multiple planning instances, identified by their ID
  if (planning_instance_id != _planning_instance->id)
  {
    j_res["error"] = "Planning instance does not match the requested ID.";
    response = j_res.dump();
    return;
  }

  // TODO: Allow overwriting existing planning jobs.
  if (_planning_instance->inspector->step_num() != 0)
  {
    j_res["error"] = "Planning instance has already started a planning job. "
        "Create a new planning instance to start a new job.";
    response = j_res.dump();
    return;
  }

  const double start_x = j_param["start"]["x"];
  const double start_y = j_param["start"]["y"];
  const double start_yaw = j_param["start"]["yaw"];
  const std::size_t goal_index = j_param["goal"];

  const auto time_now = std::chrono::steady_clock::now();
  auto starts = rmf_traffic::agv::compute_plan_starts(
      _planning_instance->graph_info.graph,
      "test_map",
      {start_x, start_y, start_yaw},
      time_now);
  bool started = _planning_instance->inspector->begin(
      starts,
      {goal_index},
      _planning_instance->planner.get_default_options());
  if (!started)
  {
    j_res["error"] = "Plan could not be started, please check that the provided"
        " start and goal are valid.";
    response = j_res.dump();
    return;
  }

  auto planning_state = _planning_instance->inspector->get_state();
  parse_planning_state(planning_state, j_res);
  response = j_res.dump();
}

//==============================================================================

void Server::get_step_response(
    const server::message_ptr& msg, std::string& response)
{
  json j_res = _j_res;
  j_res["response"] = "step";
  if (!_planning_instance)
  {
    j_res["error"] = "Planning instance has not yet been started.";
    response = j_res.dump();
    return;
  }

  std::string msg_payload = msg->get_payload();
  json j_req = json::parse(msg_payload);
  std::string planning_instance_id = j_req["id"];

  // TODO: Allow multiple planning instances, identified by their ID
  if (planning_instance_id != _planning_instance->id)
  {
    j_res["error"] = "Planning instance does not match the requested ID.";
    response = j_res.dump();
    return;
  }

  _planning_instance->inspector->step();
  auto planning_state = _planning_instance->inspector->get_state();
  parse_planning_state(planning_state, j_res);
  response = j_res.dump();
}

//==============================================================================

void Server::get_close_planner_response(
    const server::message_ptr& msg, std::string& response)
{
  json j_res = _j_res;
  j_res["response"] = "close_planner";
  if (!_planning_instance)
  {
    j_res["error"] = "Planning instance has not yet been started.";
    response = j_res.dump();
    return;
  }

  std::string msg_payload = msg->get_payload();
  json j_req = json::parse(msg_payload);
  std::string planning_instance_id = j_req["id"];

  // TODO: Allow multiple planning instances, identified by their ID
  if (planning_instance_id != _planning_instance->id)
  {
    j_res["error"] = "Planning instance does not match the requested ID.";
    response = j_res.dump();
    return;
  }

  _planning_instance.release();
  assert(_planning_instance.get() == nullptr);


  // TODO: Provide a more specific success response, other than no-error
  j_res["values"] = "";
  response = j_res.dump();
}

//==============================================================================

void Server::parse_planning_state(
    const Inspector::ConstPlanningStatePtr& state, json& j_res) const
{
  if (!_planning_instance)
    return;

  size_t traj_id = 0;
  for (const auto& n : state->expanded_nodes)
  {
    auto j_traj = _j_traj;
    j_traj["id"] = traj_id++;
    j_traj["shape"] = "circle";
    j_traj["dimensions"] = _planning_instance->vehicle_profile.footprint()
        ->get_characteristic_length();

    auto current_node = n;
    while(current_node)
    {
      const auto& traj = current_node->route_from_parent.trajectory();
      for (auto it = traj.begin(); it != traj.end(); it++)
      {
        auto j_seg = _j_seg;
        auto pos = it->position();
        auto vel = it->velocity();
        j_seg["x"] = {pos[0], pos[1], pos[2]};
        j_seg["v"] = {vel[0], vel[1], vel[2]};
        j_seg["t"] = std::chrono::duration_cast<std::chrono::milliseconds>(
            it->time().time_since_epoch()).count();
        j_traj["segments"].push_back(j_seg);
      }
      current_node = current_node->parent;
    }

    j_res["values"].push_back(j_traj);
  }
}

//==============================================================================

} // planning
} // rmf_visualizer
