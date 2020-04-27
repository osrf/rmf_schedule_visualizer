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

#include <utility>
#include <unordered_set>

#include <rmf_traffic/agv/debug/Planner.hpp>

#include "Tree.hpp"

namespace rmf_visualizer {
namespace planning {

//==============================================================================

auto Tree::make(const Inspector::ConstPlanningStatePtr& planning_state)
    -> SharedPtr
{
  // parse expanded nodes
  Branches tree_branches = {};
  std::unordered_set<
      std::pair<std::size_t, std::size_t>, 
      boost::hash<std::pair<std::size_t, std::size_t>>> unique_branches;
  
  for (const auto& n : planning_state->expanded_nodes)
  {
    auto np = n->parent;
    if (!n->waypoint.has_value() || !np || !np->waypoint.has_value())
      continue;

    while (np && np->waypoint.has_value())
    {
      auto possible_branch = 
          std::make_pair(np->waypoint.value(), n->waypoint.value());

      if (unique_branches.insert(possible_branch).second)
      {
        auto& traj = n->route_from_parent.trajectory();
        for (auto it = traj.cbegin(); it != traj.cend();)
        {
          Eigen::Vector3d curr_pos = it->position();
          ++it;
          
          if (it == traj.cend())
            break;

          tree_branches.emplace_back(curr_pos, it->position());
        }
      }

      np = np->parent;
    }
  }

  // parse plan
  Branches plan_branches = {};

  if (planning_state->plan.has_value())
  {
    auto& itinerary = planning_state->plan.value().get_itinerary();
    for (const auto r : itinerary)
    {
      for (auto it = r.trajectory().cbegin(); it != r.trajectory().cend();)
      {
        Eigen::Vector3d curr_pos = it->position();
        ++it;
        if (it == r.trajectory().cend())
          break;

        plan_branches.emplace_back(curr_pos, it->position());
      }
    }
  }

  SharedPtr tree_ptr(
      new Tree(std::move(tree_branches), std::move(plan_branches)));
  return tree_ptr;
}

//==============================================================================

Tree::Tree(Tree::Branches branches, Tree::Branches plan_branches)
: _branches(std::move(branches)),
  _plan_branches(std::move(plan_branches))
{}

//==============================================================================

auto Tree::get_branches() const -> const Branches&
{
  return _branches;
}

//==============================================================================

auto Tree::get_plan_branches() const -> const Branches&
{
  return _plan_branches;
}

//==============================================================================

} // namespace planning
} // namespace rmf_visualizer
