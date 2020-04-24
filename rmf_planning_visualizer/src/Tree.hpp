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

#ifndef RMF_PLANNING_VISUALIZER__SRC__TREE_HPP
#define RMF_PLANNING_VISUALIZER__SRC__TREE_HPP

#include <vector>
#include <memory>

#include "Inspector.hpp"

namespace rmf_visualizer {
namespace planning {

/// This Tree class holds a less strict definition than the Tree data structure, 
/// there is no strict root, but rather just a collection of branches, with
/// pose information.
class Tree
{

public:

  using SharedPtr = std::shared_ptr<Tree>;

  static SharedPtr make(const Inspector::ConstPlanningStatePtr& planning_state);

  struct Branch
  {
    Eigen::Vector3d begin;
    Eigen::Vector3d end;

    Branch(Eigen::Vector3d begin_, Eigen::Vector3d end_)
    : begin(begin_),
      end(end_)
    {}
  };

  using Branches = std::vector<Branch>;

  const Branches& get_branches() const;

private:

  Branches _branches;

  Tree(Branches _branches);

};

} // namespace planning
} // namespace rmf_visualzier

#endif // RMF_PLANNING_VISUALIZER__SRC__TREE_HPP
