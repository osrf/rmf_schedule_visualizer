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

using namespace std::chrono_literals;

class RvizNode : public rclcpp::Node
{
public:
  using Marker = visualization_msgs::msg::Marker;
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  using Point = geometry_msgs::msg::Point;

  RvizNode(std::string node_name,
      double rate = 1)
  : Node(node_name),
    _rate(rate)
  {
    _marker_pub = this->create_publisher<Marker>("test_marker", rclcpp::SystemDefaultsQoS());
    _marker_array_pub = this->create_publisher<MarkerArray>("test_marker_array", rclcpp::SystemDefaultsQoS());
    _timer = this->create_wall_timer(500ms, std::bind(&RvizNode::timer_callback, this));
  }

private:

  builtin_interfaces::msg::Time convert_time(std::chrono::steady_clock::time_point time)
  {
    const auto duration = time.time_since_epoch();
    builtin_interfaces::msg::Time result;
    result.sec = std::chrono::duration_cast<
        std::chrono::seconds>(duration).count();

    const auto nanoseconds = duration - std::chrono::seconds(result.sec);
    result.nanosec = nanoseconds.count();

    return result;
  }
  void timer_callback()
  {

    RCLCPP_INFO(this->get_logger(), "Hello");
    MarkerArray marker_array;
    Marker marker_x;
    Marker marker_y;

    marker_x = make_marker(true, 1);
    marker_array.markers.push_back(marker_x);

    if(_count<5)
    {
      marker_y = make_marker(false, 2);
      marker_array.markers.push_back(marker_y);
    }

    std::cout<<marker_array.markers.size()<<std::endl;
    _marker_array_pub->publish(marker_array);

    _count++;

  }

  visualization_msgs::msg::Marker make_marker(bool x, int id)
  {
      Marker marker_msg;

      marker_msg.header.frame_id = "/map";
      auto start_time = std::chrono::steady_clock::now();
      marker_msg.header.stamp = convert_time(start_time);
      marker_msg.ns = "basic_shapes";
      marker_msg.id = id;
      marker_msg.type = marker_msg.SPHERE;
      marker_msg.action = marker_msg.ADD;

      // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
       marker_msg.pose.position.x = 0;
       marker_msg.pose.position.y = 0;
      if(x)
        marker_msg.pose.position.x = 0 + (_count/2.0);
      else
        marker_msg.pose.position.y = 0 +(_count/2.0);
      marker_msg.pose.position.z = 0;
      marker_msg.pose.orientation.x = 0.0;
      marker_msg.pose.orientation.y = 0.0;
      marker_msg.pose.orientation.z = 0.0;
      marker_msg.pose.orientation.w = 1.0;

      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      marker_msg.scale.x = 1.0;
      marker_msg.scale.y = 1.0;
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

  double _rate = 1;
  rclcpp::TimerBase::SharedPtr _timer;
  rclcpp::Publisher<Marker>::SharedPtr _marker_pub;
  rclcpp::Publisher<MarkerArray>::SharedPtr _marker_array_pub;

  // rclcpp::Subscription<String>::SharedPtr _cmd_sub;
  // rclcpp::Publisher<String>::SharedPtr _viz_pub; 
  std::string _map_name;
  int _count;

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

  auto rviz_node = std::make_shared<RvizNode>("rviz_node");

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
