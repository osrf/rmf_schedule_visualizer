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

#ifndef RMF_PLANNING_VISUALIZER__SRC__INSPECTOR_HPP
#define RMF_PLANNING_VISUALIZER__SRC__INSPECTOR_HPP

#include <string>
#include <memory>
#include <vector>

#include <boost/functional/hash.hpp>

#include <rmf_utils/optional.hpp>

#include <rmf_traffic/agv/Planner.hpp>
#include <rmf_traffic/agv/debug/Planner.hpp>

namespace rmf_visualizer {
namespace planning { 

class Inspector
{

public:

  using SharedPtr = std::shared_ptr<Inspector>;
  using Plan = rmf_traffic::agv::Plan;
  using Planner = rmf_traffic::agv::Planner;
  using Node = Planner::Debug::Node;

  static SharedPtr make(const Planner& planner);

  ~Inspector();

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
    Node::Vector expanded_nodes;
    Node::Vector terminal_nodes;

    void print_plan(const Planner::Debug::ConstNodePtr& node) const;

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

  Inspector(std::unique_ptr<Planner::Debug> debugger);

};

} // namespace planning
} // namespace rmf_visualizer

#endif // RMF_PLANNING_VISUALIZER__SRC__INSPECTOR_HPP
