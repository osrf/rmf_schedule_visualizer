/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include "VisualizerData.hpp"

#include <rmf_traffic_ros2/StandardNames.hpp>
#include <rmf_traffic_ros2/Time.hpp>
#include <rmf_traffic_ros2/Trajectory.hpp>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

namespace rmf_schedule_visualizer {

std::shared_ptr<VisualizerDataNode> VisualizerDataNode::make(
  std::string node_name,
  rmf_traffic::Duration wait_time)
{
  const auto start_time = std::chrono::steady_clock::now();
  std::shared_ptr<VisualizerDataNode> visualizer_data(
    new VisualizerDataNode(std::move(node_name)));

  // Creating a mirror manager that queries over all
  // Spacetime in the database schedule
  auto mirror_mgr_future = rmf_traffic_ros2::schedule::make_mirror(
    *visualizer_data, rmf_traffic::schedule::query_all(),
    &visualizer_data->_mutex);

  const auto stop_time = start_time + wait_time;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < stop_time)
  {
    rclcpp::spin_some(visualizer_data);
    using namespace std::chrono_literals;
    bool ready = (mirror_mgr_future.wait_for(0s) == std::future_status::ready);

    if (ready)
    {
      visualizer_data->start(Data{mirror_mgr_future.get()});
      return visualizer_data;
    }
  }

  RCLCPP_ERROR(
    visualizer_data->get_logger(),
    std::string("Mirror was not initialized in enough time ["
    + std::to_string(rmf_traffic::time::to_seconds(wait_time))
    + "s]!").c_str());
  return nullptr;
}

//==============================================================================
VisualizerDataNode::VisualizerDataNode(std::string node_name)
: Node(node_name),
  _node_name(std::move(node_name))
{
  // Do nothing
}

//==============================================================================
void VisualizerDataNode::start(Data _data)
{
  data = std::make_unique<Data>(std::move(_data));
  data->mirror.update();

  _conflict_notice_sub = this->create_subscription<ConflictNotice>(
    rmf_traffic_ros2::NegotiationNoticeTopicName,
    rclcpp::QoS(10),
    [&](ConflictNotice::UniquePtr msg)
    {
      std::lock_guard<std::mutex> guard(_mutex);
      _conflicts[msg->conflict_version] = msg->participants;
    });

  _conflict_conclusion_sub = this->create_subscription<ConflictConclusion>(
    rmf_traffic_ros2::NegotiationConclusionTopicName,
    rclcpp::ServicesQoS(),
    [&](ConflictConclusion::UniquePtr msg)
    {
      std::lock_guard<std::mutex> guard(_mutex);
      _conflicts.erase(msg->conflict_version);
    });

  //Create a subscriber to a /debug topic to print information from this node
  debug_sub = create_subscription<std_msgs::msg::String>(
    _node_name+"/debug", rclcpp::SystemDefaultsQoS(),
    [&](std_msgs::msg::String::UniquePtr msg)
    {
      debug_cb(std::move(msg));
    });

  // retrieve/construct mirrors, snapshots and negotiation object
  _negotiation = rmf_traffic_ros2::schedule::Negotiation(
    *this, data->mirror.snapshot_handle());
}

void VisualizerDataNode::debug_cb(std_msgs::msg::String::UniquePtr msg)
{
  if (msg->data == "info")
  {
    std::lock_guard<std::mutex> guard(_mutex);
    // Display the latest changes made to the mirror
    // along with details of trajectories in the schedule
    try
    {
      RCLCPP_INFO(get_logger(), std::string("Mirror Version: [%d]").c_str(),
        data->mirror.viewer().latest_version());
      // Query since database was created
      auto view = data->mirror.viewer().query(
        rmf_traffic::schedule::query_all());
      if (view.size() == 0)
        RCLCPP_INFO(this->get_logger(),
          std::string("Schedule is empty").c_str());

      else
      {
        for (const auto& element : view)
        {
          auto t = element.route.trajectory();
          RCLCPP_INFO(
            get_logger(),
            std::string("Trajectory id: [%d]\nTrajectory size: [%d]").c_str(),
            element.route_id, t.size());
          int count = 0;
          for (auto it = t.begin(); it != t.end(); it++)
          {
            ++count;
            auto finish_time = it->time();
            auto finish_position = it->position();
            RCLCPP_INFO(get_logger(),
              std::string("waypoint: [%d]\ntime: i "
              "[%s]\nposiiton:[%d, %d, %d]").c_str(),
              count,
              std::to_string(finish_time.time_since_epoch().count()).c_str(),
              finish_position[0], finish_position[1], finish_position[2]);
          }
        }
      }
    }
    catch (std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), std::to_string(*e.what()).c_str());
    }
  }
}

std::vector<rmf_traffic::Trajectory> VisualizerDataNode::get_trajectories(
  RequestParam request_param)
{
  std::vector<rmf_traffic::Trajectory> trajectories;
  const std::vector<std::string> maps {request_param.map_name};
  const auto query = rmf_traffic::schedule::make_query(
    maps,
    &request_param.start_time,
    &request_param.finish_time);

  const auto view = data->mirror.viewer().query(query);
  for (const auto& element : view)
    trajectories.push_back(element.route.trajectory());

  return trajectories;
}

using Element = rmf_traffic::schedule::Viewer::View::Element;
std::vector<Element> VisualizerDataNode::get_elements(
  RequestParam request_param)
{
  std::vector<Element> elements;
  const std::vector<std::string> maps {request_param.map_name};
  const auto query = rmf_traffic::schedule::make_query(
    maps,
    &request_param.start_time,
    &request_param.finish_time);

  const auto view = data->mirror.viewer().query(query);

  for (const auto& element : view)
    elements.push_back(element);

  return elements;
}

//==============================================================================
rmf_traffic::Time VisualizerDataNode::now()
{
  return rmf_traffic_ros2::convert(get_clock()->now());
}

//==============================================================================
std::mutex& VisualizerDataNode::get_mutex()
{
  return _mutex;
}

std::unordered_set<uint64_t> VisualizerDataNode::get_conflicts() const
{
  std::unordered_set<uint64_t> conflict_id;
  for (const auto& conflict : _conflicts)
  {
    std::copy(conflict.second.begin(),
      conflict.second.end(),
      std::inserter(conflict_id, conflict_id.end()));
  }

  return conflict_id;
}

std::vector<std::vector<
    uint64_t>> VisualizerDataNode::get_server_conflicts() const
{
  std::vector<std::vector<uint64_t>> conflicts;
  for (const auto& conflict : _conflicts)
    conflicts.push_back(conflict.second);

  return conflicts;
}

std::vector<Element> VisualizerDataNode::get_negotiation_trajectories(
  uint64_t conflict_version, const std::vector<uint64_t>& sequence) const
{
  std::vector<Element> trajectory_elements;

  const auto table_view = _negotiation->table_view(conflict_version, sequence);
  if (!table_view)
  {
    RCLCPP_WARN(
      this->get_logger(),
      std::string("table_view for conflict %d not found!").c_str(),
      conflict_version);
    return trajectory_elements;
  }

  rmf_traffic::RouteId route_id = 0;
  const auto add_route = [&route_id, &table_view, &trajectory_elements]
      (rmf_traffic::ConstRoutePtr route_ptr,
      rmf_traffic::schedule::ParticipantId id)
    {
      const auto& route = *(route_ptr);

      Element e { id, route_id, route, *table_view->get_description(id) };
      trajectory_elements.push_back(e);
      ++route_id;
    };

  auto itin = table_view->submission();
  if (itin)
  {
    const auto& routes = *itin;
    for (auto route_ptr : routes)
      add_route(route_ptr, table_view->participant_id());
  }

  for (auto proposal : table_view->base_proposals())
  {
    for (auto route : proposal.itinerary)
      add_route(route, proposal.participant);
  }
  return trajectory_elements;
}


} // namespace rmf_schedule_visualizer
