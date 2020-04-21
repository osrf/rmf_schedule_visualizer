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

#include "PlanningInspector.hpp"

namespace rmf_planning_visualizer
{

std::shared_ptr<PlanningInspector> PlanningInspector::make(
    std::string node_name,
    const Planner& planner)
{
  using Planner = rmf_traffic::agv::Planner;
  std::unique_ptr<Planner::Debug> debugger(new Planner::Debug(planner));

  std::shared_ptr<PlanningInspector> planning_inspector(
          new PlanningInspector(
              std::move(node_name), 
              std::move(debugger)));
  
  return planning_inspector;  
}

//==============================================================================

PlanningInspector::PlanningInspector(
    std::string node_name, 
    std::unique_ptr<Planner::Debug> debugger)
: Node(std::move(node_name)),
  _debugger(std::move(debugger))
{}

} // namespace rmf_planning_visualizer
