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

#include "PlanningInspector.hpp"

namespace rmf_planning_visualizer
{

std::shared_ptr<PlanningInspector> PlanningInspector::make(
    const Planner& planner)
{
  using Planner = rmf_traffic::agv::Planner;
  std::unique_ptr<Planner::Debug> debugger(new Planner::Debug(planner));

  std::shared_ptr<PlanningInspector> planning_inspector(
      new PlanningInspector(std::move(debugger)));
  
  return planning_inspector;
}

//==============================================================================

PlanningInspector::PlanningInspector(std::unique_ptr<Planner::Debug> debugger)
: _debugger(std::move(debugger))
{}

//==============================================================================

PlanningInspector::~PlanningInspector()
{}

//==============================================================================

bool PlanningInspector::begin(
    const std::vector<Plan::Start>& starts,
    Plan::Goal goal,
    Planner::Options options)
{
  auto progress = _debugger->begin(
      starts,
      std::move(goal),
      std::move(options));
  
  if (!progress)
    return false;

  using Progress = Planner::Debug::Progress;
  _progress.reset(new Progress(std::move(progress)));

  const PlanningState zeroth_step {
    0,
    rmf_utils::nullopt,
    _progress->queue(),
    _progress->expanded_nodes(),
    _progress->terminal_nodes()
  };

  _planning_states.clear();
  _planning_states.emplace_back(new PlanningState(std::move(zeroth_step)));
  return true;
}

//==============================================================================

void PlanningInspector::step()
{
  if (!_progress)
    return;
  
  const auto new_plan = _progress->step();
  const PlanningState new_step {
    _planning_states.size(),
    std::move(new_plan),
    _progress->queue(),
    _progress->expanded_nodes(),
    _progress->terminal_nodes()
  };

  _planning_states.emplace_back(new PlanningState(std::move(new_step)));
}

//==============================================================================

auto PlanningInspector::plan() const -> rmf_utils::optional<Plan>
{
  if (_planning_states.empty())
    return rmf_utils::nullopt;
  return get_state()->plan;
}

//==============================================================================

bool PlanningInspector::plan_completed() const
{
  if (plan())
    return true;
  return false;
}

//==============================================================================

std::size_t PlanningInspector::step_num() const
{
  return _planning_states.size();
}

//==============================================================================

auto PlanningInspector::get_state() const -> ConstPlanningStatePtr
{
  return _planning_states.back();
}

//==============================================================================

auto PlanningInspector::get_state(std::size_t step_index) const -> ConstPlanningStatePtr
{
  if (step_index > step_num())
    return nullptr;
  return _planning_states[step_index];
}

//==============================================================================

void PlanningInspector::PlanningState::print() const
{
  printf("STEP %zu:\n", step_index);

  std::vector<std::size_t> reversed_route_so_far = 
      {unexpanded_nodes.top()->waypoint.value()};
  auto top_parent = unexpanded_nodes.top()->parent;
  while (top_parent)
  {
    if (!top_parent->waypoint.has_value())
      break;
    reversed_route_so_far.push_back(top_parent->waypoint.value());

    if (!top_parent->parent && top_parent->start_set_index.has_value())
    {
      reversed_route_so_far.push_back(top_parent->start_set_index.value());
      break;
    }
    top_parent = top_parent->parent;
  }

  std::string route_string = "begin";
  for (auto it = reversed_route_so_far.rbegin(); it != reversed_route_so_far.rend(); ++it)
  {
    route_string += " -> " + std::to_string(*it);
  }
  printf("    Current expansion:\n");
  printf("        %s\n", route_string.c_str());

  printf("    Unexpanded: %zu nodes, top: %zu\n",
      unexpanded_nodes.size(),
      unexpanded_nodes.top()->waypoint.value());
  
  printf("    Expanded: %zu nodes\n", expanded_nodes.size());
  std::string expanded_nodes_string = "";
  for (const auto& n : expanded_nodes)
    expanded_nodes_string += std::to_string(n->waypoint.value()) + ", ";
  printf("        {%s}\n", expanded_nodes_string.c_str());

  printf("    Terminal: %zu nodes\n", terminal_nodes.size());
  std::string terminal_nodes_string = "";
  for (const auto& n : terminal_nodes)
    terminal_nodes_string += std::to_string(n->waypoint.value()) + ", ";
  printf("        {%s}\n", terminal_nodes_string.c_str());
  
  printf("\n\n");
  if (plan)
    printf("    PLANNING DONE!\n\n");
}

//==============================================================================

} // namespace rmf_planning_visualizer
