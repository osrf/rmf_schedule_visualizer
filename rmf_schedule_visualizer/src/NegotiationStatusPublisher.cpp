
#include "NegotiationStatusPublisher.hpp"

namespace rmf_schedule_visualizer {

bool NegotiationStatusPublisher::start(rclcpp::Node& node, NegotiationStatusPublisher& data)
{
  RCLCPP_INFO(node.get_logger(), "STARTING NEGOTIATION STATUS SETUP");

  data._publisher = node.create_publisher<rmf_traffic_msgs::msg::NegotiationStatus>(
    rmf_traffic_ros2::NegotiationStatusTopicName, rclcpp::QoS(10));
  data._conclusion_publisher = node.create_publisher<rmf_traffic_msgs::msg::NegotiationStatusConclusion>(
    rmf_traffic_ros2::NegotiationStatusConclusionTopicName, rclcpp::QoS(10));

  // retrieve/construct mirrors, snapshots and negotiation object
  { 
    auto mirror_future = rmf_traffic_ros2::schedule::make_mirror(
      node, rmf_traffic::schedule::query_all());

    data._writer = rmf_traffic_ros2::schedule::Writer::make(node);

    using namespace std::chrono_literals;

    bool ready = false;
    const auto stop_time = std::chrono::steady_clock::now() + 10s;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < stop_time)
    {
      rclcpp::spin_some(node.get_node_base_interface());

      bool writer_ready = data._writer->ready();
      bool mirror_ready = (mirror_future.wait_for(0s) == std::future_status::ready);

      if (writer_ready && mirror_ready)
      {
        data._mirror_mgr = mirror_future.get();
        data._negotiation = rmf_traffic_ros2::schedule::Negotiation(
          node, data._mirror_mgr->snapshot_handle());
        ready = true;
        break;
      }
    }

    if (!ready)
    {
      RCLCPP_ERROR(node.get_logger(), "Unable to get mirrors/snaphots!");
      return false;
    }
  }

  // setup callbacks for publisher and conclusion_publisher
  auto status_update_cb = [&](
    uint64_t conflict_version,
    rmf_traffic::schedule::Negotiation::Table::ViewerPtr table_view)
  {
    RCLCPP_WARN(node.get_logger(), "======== conflict callback %d! ==========",
      conflict_version);
    
    rmf_traffic_msgs::msg::NegotiationStatus msg;

    msg.conflict_version = conflict_version;
    msg.participant = table_view->participant_id();
    msg.participant_name = table_view->get_description(table_view->participant_id())->name();
    
    if (table_view->defunct())
      msg.status |= msg.STATUS_DEFUNCT;
    if (table_view->rejected())
      msg.status |= msg.STATUS_REJECTED;
    if (table_view->forfeited())
      msg.status |= msg.STATUS_FORFEITED;
      
    auto versioned_sequence = table_view->sequence();
    for (auto versionedkey : versioned_sequence)
      msg.sequence.push_back(versionedkey.participant);

    data._publisher->publish(msg);
  };
  data._negotiation->on_status_update(status_update_cb);

  auto conclusion_cb = [&](
    uint64_t conflict_version, bool resolved)
  {
    RCLCPP_WARN(node.get_logger(), "======== conflict concluded: %llu resolved: %d ==========",
      conflict_version, resolved ? 1 : 0);

    rmf_traffic_msgs::msg::NegotiationStatusConclusion msg;
    msg.conflict_version = conflict_version;
    msg.resolved = resolved;

    data._conclusion_publisher->publish(msg);
  };
  data._negotiation->on_conclusion(conclusion_cb);
  
  RCLCPP_WARN(node.get_logger(), "Negotiation status publisher is ready!");
  return true;
}

}