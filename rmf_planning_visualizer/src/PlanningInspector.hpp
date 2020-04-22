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
#include <vector>

#include <rmf_utils/optional.hpp>

#include <rmf_traffic/agv/Planner.hpp>
#include <rmf_traffic/agv/debug/Planner.hpp>

#include <rclcpp/node.hpp>

namespace rmf_planning_visualizer
{

class PlanningInspector
{

public:

  using Plan = rmf_traffic::agv::Plan;
  using Planner = rmf_traffic::agv::Planner;
  using Node = Planner::Debug::Node;

  static std::shared_ptr<PlanningInspector> make(const Planner& planner);

  ~PlanningInspector();

  bool begin(
      const std::vector<Plan::Start>& starts,
      Plan::Goal goal,
      Plan::Options options);

  void step();

  struct PlanningState;
  using ConstPlanningStatePtr = std::shared_ptr<const PlanningState>;

  struct PlanningState
  {
    std::size_t step_index;

    rmf_utils::optional<Plan> plan;
    Node::SearchQueue unexpanded_nodes;
    Node::Vector expanded_nodes;
    Node::Vector terminal_nodes;

    void print() const;
  };

  rmf_utils::optional<Plan> plan() const;

  bool plan_completed() const;

  std::size_t step_num() const;
  
  ConstPlanningStatePtr get_state() const;

  ConstPlanningStatePtr get_state(std::size_t step_index) const;

private:

  std::unique_ptr<Planner::Debug> _debugger;

  std::unique_ptr<Planner::Debug::Progress> _progress;

  std::vector<ConstPlanningStatePtr> _planning_states;

  PlanningInspector(std::unique_ptr<Planner::Debug> debugger);

};

} // namespace rmf_planning_visualizer

#endif // RMF_PLANNING_VISUALIZER__SRC__PLANNINGINSPECTOR_HPP
