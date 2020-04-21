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

#ifndef RMF_PLANNING_VISUALIZER__SRC__PLANNINGINSPECTOR_HPP
#define RMF_PLANNING_VISUALIZER__SRC__PLANNINGINSPECTOR_HPP

#include <string>
#include <memory>

#include <rmf_traffic/agv/Planner.hpp>
#include <rmf_traffic/agv/debug/Planner.hpp>

#include <rclcpp/node.hpp>

namespace rmf_planning_visualizer
{

class PlanningInspector : public rclcpp::Node
{

public:

  using Planner = rmf_traffic::agv::Planner;

  static std::shared_ptr<PlanningInspector> make(
      std::string node_name,
      const Planner& planner);

private:

  std::unique_ptr<Planner::Debug> _debugger;

  PlanningInspector(
      std::string node_name, 
      std::unique_ptr<Planner::Debug> debugger);

};

} // namespace rmf_planning_visualizer

#endif // RMF_PLANNING_VISUALIZER__SRC__PLANNINGINSPECTOR_HPP
