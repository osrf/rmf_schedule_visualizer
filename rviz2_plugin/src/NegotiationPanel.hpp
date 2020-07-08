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

#ifndef RVIZ2_PLUGIN__SRC__NEGOTIATIONPANEL_HPP
#define RVIZ2_PLUGIN__SRC__NEGOTIATIONPANEL_HPP

#include <rclcpp/rclcpp.hpp>

#include <rviz_common/panel.hpp>
#include <rmf_traffic_msgs/msg/negotiation_notice.hpp>
#include <rmf_traffic_msgs/msg/negotiation_status.hpp>
#include <thread>
#include <mutex>
#include <QGraphicsScene>
#include <atomic>

class QLineEdit;
class QLabel;
class QGraphicsView;

namespace rviz2_plugin {

class GraphicsScene;

class NegotiationPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  NegotiationPanel(QWidget* parent = 0);
  // virtual void load(const rviz_common::Config& config);
  // virtual void save(rviz_common::Config config) const;

  void set_negotiation_printout(const std::string& text);

  using NegotiationStatusMsg = rmf_traffic_msgs::msg::NegotiationStatus;
  void update_negotiation_graph_visuals(NegotiationStatusMsg::SharedPtr msg);

  std::mutex _lock;
  std::atomic_bool _update_gfx = { false };
  void paintEvent(QPaintEvent *event);
protected:
  QLineEdit* _conflict_version_editor;
  QLabel* _statusText;
  GraphicsScene* _scene;
  QGraphicsView* _view;

  std::vector<QRectF> _node_rectangles;

  NegotiationStatusMsg::SharedPtr _status_msg;
  rclcpp::Subscription<NegotiationStatusMsg>::SharedPtr _negotiation_status_sub;

  rclcpp::Node::SharedPtr _node;
  std::thread _thread;

  void redraw();
};

class GraphicsScene : public QGraphicsScene
{
public:
  void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
  void mouseMoveEvent(QGraphicsSceneMouseEvent *event);

};


}

#endif
