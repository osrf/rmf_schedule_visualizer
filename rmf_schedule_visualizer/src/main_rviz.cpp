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
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <rmf_traffic/geometry/Box.hpp>
#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/Time.hpp>
#include <rmf_traffic_ros2/Time.hpp>

using namespace std::chrono_literals;

class RvizNode : public rclcpp::Node
{
public:
  using Marker = visualization_msgs::msg::Marker;
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  using Point = geometry_msgs::msg::Point;
  using RequestParam = rmf_schedule_visualizer::RequestParam;

  RvizNode(
      std::string node_name,
      rmf_schedule_visualizer::VisualizerDataNode& visualizer_data_node,
      double rate = 1,
      std::string frame_id = "/map")
  : Node(node_name),
    _visualizer_data_node(visualizer_data_node),
    _rate(rate),
    _frame_id(frame_id)
  {
    _count = 0;
    _map_name= "level1";

    _marker_pub = this->create_publisher<Marker>("test_marker", rclcpp::SystemDefaultsQoS());
    _marker_array_pub = this->create_publisher<MarkerArray>("dp2_marker_array", rclcpp::SystemDefaultsQoS());
    _timer = this->create_wall_timer(500ms, std::bind(&RvizNode::timer_callback, this));
  }

private:

  void timer_callback()
  {
    MarkerArray marker_array;

    // TODO store a cache of trajectories to prevent frequent access
    // update chache whenever mirror manager updates 
    // maintain unordered map of id and trajectory

    RequestParam param;
    param.map_name = _map_name;
    param.start_time = std::chrono::steady_clock::now();
    param.finish_time = param.start_time + 120s;
    _trajectories = _visualizer_data_node.get_trajectories(param);

    if (_trajectories.size() > 0)
    {
      // for each trajectory create two markers
      // 1) Current position 
      // 2) Path until param.finish_time

      // Temporary count used as id for each trajectory marker
      // id will be sourced from unordered map
      int count = 0;
      for(auto it = _trajectories.begin(); it != _trajectories.end(); it++)
      {
        ++count;
        auto location_marker = make_location_marker(*it, param, count);
        marker_array.markers.push_back(location_marker);
      }
      RCLCPP_INFO(this->get_logger(),
          "Publishinb marker array of size: " + std::to_string(marker_array.markers.size()));
       _marker_array_pub->publish(marker_array);
    }

  }

  visualization_msgs::msg::Marker make_location_marker(
        rmf_traffic::Trajectory& trajectory,
        const RequestParam param,
        int id)
  {
    // TODO Link the color, shape and size of marker to profile of trajectory

    Marker marker_msg;
    const double radius = static_cast<const rmf_traffic::geometry::Circle&>(
          trajectory.begin()->get_profile()->get_shape()->source()).get_radius();
    std::cout<<"Radius: "<<radius<<std::endl;

    marker_msg.header.frame_id = _frame_id; // map
    marker_msg.header.stamp = rmf_traffic_ros2::convert(param.start_time);
    marker_msg.ns = "trajectory";
    marker_msg.id = id;
    marker_msg.type = marker_msg.SPHERE;
    marker_msg.action = marker_msg.ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    auto motion = trajectory.find(param.start_time)->compute_motion();
    Eigen::Vector3d position =  motion->compute_position(param.start_time);
    marker_msg.pose.position.x = position[0];
    marker_msg.pose.position.y = position[1];
    marker_msg.pose.position.z = 0;

    auto quat = convert(position[2]);
    marker_msg.pose.orientation.x = quat.x;
    marker_msg.pose.orientation.y = quat.y;
    marker_msg.pose.orientation.z = quat.z;
    marker_msg.pose.orientation.w = quat.w;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker_msg.scale.x = radius / 1.0;
    marker_msg.scale.y = radius / 1.0;
    marker_msg.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker_msg.color.r = 0.0f;
    marker_msg.color.g = 1.0f;
    marker_msg.color.b = 0.0f;
    marker_msg.color.a = 1.0;
    
    builtin_interfaces::msg::Duration duration;
    duration.sec = 0.01;
    duration.nanosec = 0;
    marker_msg.lifetime = duration;

    Point point;
    point.x = 0;
    point.y = 0;
    point.z = 0;
    marker_msg.points.push_back(point);

    point.x = 0;
    point.y = 1;
    point.z = 0;
    marker_msg.points.push_back(point);

    return marker_msg;
  }

  struct quaternion
  {
    double w, x, y, z;
  };

  quaternion convert(double yaw, double pitch = 0, double roll =0)
  {
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    quaternion q;
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;

    return q;
  }

  double _rate;
  rclcpp::TimerBase::SharedPtr _timer;
  rclcpp::Publisher<Marker>::SharedPtr _marker_pub;
  rclcpp::Publisher<MarkerArray>::SharedPtr _marker_array_pub;
  rmf_schedule_visualizer::VisualizerDataNode& _visualizer_data_node;
  std::string _map_name;
  int _count;
  std::vector<rmf_traffic::Trajectory> _trajectories;
  std::string _frame_id;

};


bool get_arg(
    const std::vector<std::string>& args,
    const std::string& key,
    std::string& value,
    const std::string& desc,
    const bool mandatory = true)
{
  const auto key_arg = std::find(args.begin(), args.end(), key);
  if(key_arg == args.end())
  {
    if(mandatory)
    {
      std::cerr << "You must specify a " << desc <<" using the " << key
                << " argument!" << std::endl;
    }
    return false;
  }
  else if(key_arg+1 == args.end())
  {
    std::cerr << "The " << key << " argument must be followed by a " << desc
              << "!" << std::endl;
    return false;
  }

  value = *(key_arg+1);
  return true;
}

int main(int argc, char* argv[])
{
  const std::vector<std::string> args =
      rclcpp::init_and_remove_ros_arguments(argc, argv);

  std::string node_name = "viz";
  get_arg(args, "-n", node_name, "node name", false);

  std::string port_string;
  get_arg(args, "-p", port_string, "port",false);
  const uint16_t port = port_string.empty()? 8006 : std::stoul(port_string, nullptr, 0);

  const auto visualizer_data_node =
    rmf_schedule_visualizer::VisualizerDataNode::make(node_name);

  if(!visualizer_data_node)
  {
    std::cerr << "Failed to initialize the fleet adapter node" << std::endl;
    return 1;
  }

  RCLCPP_INFO(
        visualizer_data_node->get_logger(),
        "VisualizerDataNode /" + node_name + " started...");


  // const auto server_ptr = rmf_schedule_visualizer::Server::make(port, *visualizer_data_node);
  
  // if(!server_ptr)
  // {
  //   std::cerr << "Failed to initialize the Server" << std::endl;
  //   return 1;
  // }

  auto rviz_node = std::make_shared<RvizNode>("rviz_node", *visualizer_data_node);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(visualizer_data_node);
  executor.add_node(rviz_node);
  
  RCLCPP_INFO(
        visualizer_data_node->get_logger(),
        "Websocket server started on port: " + std::to_string(port));

  executor.spin();
  RCLCPP_INFO(
        visualizer_data_node->get_logger(),
        "Closing down");

  rclcpp::shutdown();
  return 0;
}
