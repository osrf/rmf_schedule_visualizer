
#ifndef RMF_SCHEDULE_VISUALIZER__SRC__NEGOTIATIONSTATUSPUBLISHER_HPP
#define RMF_SCHEDULE_VISUALIZER__SRC__NEGOTIATIONSTATUSPUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rmf_traffic_ros2/schedule/Writer.hpp>
#include <rmf_traffic_ros2/schedule/MirrorManager.hpp>
#include <rmf_traffic_ros2/schedule/Negotiation.hpp>
#include <rmf_traffic_ros2/StandardNames.hpp>
#include <rmf_traffic_msgs/msg/negotiation_status.hpp>
#include <rmf_traffic_msgs/msg/negotiation_status_conclusion.hpp>

namespace rmf_schedule_visualizer {

class NegotiationStatusPublisher
{
public:
  static bool start(rclcpp::Node& node, NegotiationStatusPublisher& data);
 
  using Element = rmf_traffic::schedule::Viewer::View::Element;

  rmf_traffic_ros2::schedule::WriterPtr _writer;
  rmf_utils::optional<rmf_traffic_ros2::schedule::MirrorManager> _mirror_mgr;
  rmf_utils::optional<rmf_traffic_ros2::schedule::Negotiation> _negotiation;

  rclcpp::Publisher<rmf_traffic_msgs::msg::NegotiationStatus>::SharedPtr _publisher;
  rclcpp::Publisher<rmf_traffic_msgs::msg::NegotiationStatusConclusion>::SharedPtr _conclusion_publisher;

  //std::unordered_map<uint64_t, std::vector<Element>> _trajectory_elements;
};

}

#endif
