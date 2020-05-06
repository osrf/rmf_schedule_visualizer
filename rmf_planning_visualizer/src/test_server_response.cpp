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
#include <cstdio>
#include <iostream>

#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/schedule/Database.hpp>
#include <rmf_traffic/schedule/Participant.hpp>

#include <visualizer_utils/json.hpp>

#include "Inspector.hpp"

namespace rmf_visualizer {
namespace planning {

std::string print_planning_state_response(
    const Inspector::ConstPlanningStatePtr& state, size_t id, double dimension)
{
  using json = nlohmann::json;
  const json _j_res = 
      { {"response", {}}, {"id", {}}, {"values", {}}, {"error", {}} };
  const json _j_traj =
      { {"id", {}}, {"shape", {}}, {"dimensions", {}}, {"segments", {}}};
  const json _j_seg = { {"x", {}}, {"v", {}}, {"t", {}}};

  auto j_res = _j_res;
  j_res["response"] = "step";
  j_res["id"] = id;

  size_t traj_id = 0;
  for (const auto& n : state->expanded_nodes)
  {
    auto j_traj = _j_traj;
    j_traj["id"] = traj_id++;
    j_traj["shape"] = "circle";
    j_traj["dimensions"] = dimension;

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

  return j_res.dump();
}

} // planning
} // rmf_visualizer

int main(int argc, char** argv)
{
  const std::string test_map_name = "test_map";

  rmf_traffic::agv::Graph graph_b;
  graph_b.add_waypoint(test_map_name, {14.052, -23.840}); // 0
  graph_b.add_waypoint(test_map_name, {14.081, -21.636}); // 1
  graph_b.add_waypoint(test_map_name, {10.430, -23.964}); // 2
  graph_b.add_waypoint(test_map_name, {10.432, -21.788}); // 3
  graph_b.add_waypoint(test_map_name, {10.429, -25.995}); // 4
  graph_b.add_waypoint(test_map_name, { 8.093, -23.870}); // 5
  graph_b.add_waypoint(test_map_name, { 8.055, -21.818}); // 6
  graph_b.add_waypoint(test_map_name, { 8.068, -25.961}); // 7
  graph_b.add_waypoint(test_map_name, {14.920, -21.585}); // 8
  graph_b.add_waypoint(test_map_name, {14.918, -23.846}); // 9
  graph_b.add_waypoint(test_map_name, {18.435, -21.633}); // 10
  graph_b.add_waypoint(test_map_name, {18.511, -17.278}); // 11
  graph_b.add_waypoint(test_map_name, {16.830, -17.278}); // 12
  graph_b.add_waypoint(test_map_name, {11.829, -21.687}); // 13
  graph_b.add_waypoint(test_map_name, {11.892, -15.379}); // 14
  graph_b.add_waypoint(test_map_name, {16.876, -15.313}); // 15
  graph_b.add_waypoint(test_map_name, {11.829, -19.079}); // 16

  graph_b.add_lane(0, 1);
  graph_b.add_lane(1, 0);
  graph_b.add_lane(1, 2);
  graph_b.add_lane(2, 1);
  graph_b.add_lane(2, 3);
  graph_b.add_lane(3, 2);
  graph_b.add_lane(2, 4);
  graph_b.add_lane(4, 2);
  graph_b.add_lane(2, 5);
  graph_b.add_lane(5, 2);
  graph_b.add_lane(5, 6);
  graph_b.add_lane(6, 5);
  graph_b.add_lane(5, 7);
  graph_b.add_lane(7, 5);
  graph_b.add_lane(1, 8);
  graph_b.add_lane(8, 1);
  graph_b.add_lane(8, 9);
  graph_b.add_lane(9, 8);
  graph_b.add_lane(8, 10);
  graph_b.add_lane(10, 8);
  graph_b.add_lane(10, 11);
  graph_b.add_lane(11, 10);
  graph_b.add_lane(11, 12);
  graph_b.add_lane(12, 11);
  graph_b.add_lane(13, 1);
  graph_b.add_lane(1, 13);
  graph_b.add_lane(14, 15);
  graph_b.add_lane(15, 14);
  graph_b.add_lane(15, 12);
  graph_b.add_lane(12, 15);
  graph_b.add_lane(14, 16);
  graph_b.add_lane(16, 14);
  graph_b.add_lane(16, 13);
  graph_b.add_lane(13, 16);

  const auto profile_b = rmf_traffic::Profile{
    rmf_traffic::geometry::make_final_convex<
      rmf_traffic::geometry::Circle>(0.3)
  };

  const auto traits_b = rmf_traffic::agv::VehicleTraits{
    {0.4, 3.0}, {0.4, 4.0}, profile_b
  };

  const rmf_traffic::agv::Planner::Configuration config_b{graph_b, traits_b};

  rmf_traffic::schedule::Database database;

  auto b1 = rmf_traffic::schedule::make_participant(
      rmf_traffic::schedule::ParticipantDescription{
          "b1",
          "fleet_b",
          rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
          profile_b
      }, database);

  auto b2 = rmf_traffic::schedule::make_participant(
      rmf_traffic::schedule::ParticipantDescription{
          "b2",
          "fleet_b",
          rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
          profile_b
      }, database);

  rmf_traffic::agv::Planner planner_b(
      config_b,
      rmf_traffic::agv::Planner::Options(
        rmf_utils::make_clone<rmf_traffic::agv::ScheduleRouteValidator>(
          database, b1.id(), profile_b)
      ));

  auto planning_inspector = 
      rmf_visualizer::planning::Inspector::make(planner_b);

  const auto time = std::chrono::steady_clock::now();

  auto b1_starts = rmf_traffic::agv::compute_plan_starts(
      graph_b, {16.858, -15.758, -M_PI/2.0}, time);
  auto b2_starts = rmf_traffic::agv::compute_plan_starts(
      graph_b, {16.83, -17.26, -M_PI/2.0}, time);

  bool started = 
      planning_inspector->begin(
          b1_starts, {11}, planner_b.get_default_options());
  if (!started)
  {
    std::cout << "<ERROR> planning_inspector can't begin plan" << std::endl;
    return 1;
  }

  int step_num = planning_inspector->step_num();
  auto planning_state = planning_inspector->get_state();
  std::string state_printout = 
        rmf_visualizer::planning::print_planning_state_response(
            planning_state, b1.id(), 0.3);
  std::cout << "STEP [" << step_num << "]:" << std::endl;
  std::cout << state_printout << std::endl << std::endl;

  bool plan_completed = false;
  while (!plan_completed)
  {
    planning_inspector->step();
    step_num = planning_inspector->step_num();
    planning_state = planning_inspector->get_state();
    state_printout = 
        rmf_visualizer::planning::print_planning_state_response(
            planning_state, b1.id(), 0.3);
    std::cout << "STEP [" << step_num << "]:" << std::endl;
    std::cout << state_printout << std::endl << std::endl;

    plan_completed = planning_inspector->plan_completed();
  }

  return 0;
}
