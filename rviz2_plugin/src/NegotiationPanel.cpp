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

#include "NegotiationPanel.hpp"

#include <rmf_traffic_ros2/StandardNames.hpp>
#include <QLineEdit>
#include <QHBoxLayout>
#include <QLabel>
#include <QGraphicsScene>
#include <QGraphicsLinearLayout>
#include <QGraphicsWidget>
#include <QGraphicsView>
#include <QTextEdit>
#include <QPushButton>
#include <QGraphicsRectItem>
#include <QPaintEvent>
#include <QApplication>
#include <QGraphicsSceneMouseEvent>
#include <Eigen/Geometry>

namespace rviz2_plugin {

NegotiationPanel::NegotiationPanel(QWidget* parent)
: rviz_common::Panel(parent)
{
  // setup ros
  _node = std::make_shared<rclcpp::Node>("NegotiationPanelNode");

  _negotiation_status_sub = _node->create_subscription<NegotiationStatusMsg>(
    rmf_traffic_ros2::NegotiationStatusTopicName, 
    rclcpp::SystemDefaultsQoS(),
    [this](NegotiationStatusMsg::SharedPtr msg) {
      RCLCPP_WARN(rclcpp::get_logger("asdsad"), "RECVED MSG");

      {
        std::unique_lock<std::mutex> guard(_lock);
        _status_msg = *msg;
        _update_markers = true;
        prev_table_selected = -1;
      }
      QApplication::postEvent(this, new QPaintEvent(QRect()));
    }
  );

  _negotiation_itinerary_markers_pub = _node->create_publisher<MarkerArray>(
    "/negotiation_itinary_markers", rclcpp::SystemDefaultsQoS());

  // UI initialization
  _scene = new GraphicsScene(this);
  _view = new QGraphicsView(_scene);
  _view->show();

  QLayout* glayout = new QVBoxLayout();
  glayout->addWidget(_view);
  
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(glayout);
  layout->addStretch();
  setLayout(layout);

#if 0 //quick test code
  {
    NegotiationStatusMsg msg;
    msg.participants = { 1, 2 };

    NegotiationStatusTable table;
    table.parent_index = -1;
    table.sequence = { 1, 2 };
    msg.tables.push_back(table);

    NegotiationStatusTable table2;
    table2.parent_index = -1;
    table2.sequence = { 2, 1 };
    msg.tables.push_back(table2);

    NegotiationStatusTable table3;
    table3.parent_index = 1;
    table3.depth = 1;
    table3.sequence = { 2, 1 };
    msg.tables.push_back(table3);
    _status_msg = msg;
    update_negotiation_graph_visuals(msg);
  }
#endif
  //@todo: when we unify plugins we should add nodes to an executor
  _thread = std::thread([&]()
    {
      rclcpp::spin(_node);
    });

  using namespace std::chrono_literals;
  _timer = _node->create_wall_timer(0.1s,
    std::bind(&NegotiationPanel::timer_publish_callback, this));
}

void NegotiationPanel::update_negotiation_graph_visuals(const NegotiationStatusMsg& msg)
{
  std::cout << "starting repaint.." << std::endl;
  std::unique_lock<std::mutex> guard(_lock);
  
  _scene->clear();
  _node_rectangles.clear();
  
  float textwidth = 160.0f;  
  float width_per_column = textwidth + 40.f;
  //float x_offset = (-(float)column_count / 2.0f) * width_per_column;
  float x_offset = 0.f;
  
  // conflict version text
  {
    QString str;
    str.sprintf("Conflict version: %lu", msg.conflict_version);
    QGraphicsTextItem* textitem = _scene->addText(str);
    textitem->setScale(2.0);
    //left alignment because aligning center via boundingRect is wrong, qt :(
    textitem->moveBy(x_offset, -80);
  }

  int index = 0;
  int prev_depth = 0;
  uint32_t first_index_in_current_row = 0;
  float row_height = 0.0f;
  float current_height_offset = 0.0f;
  for (auto table : msg.tables)
  {
    // assemble text
    uint64_t participant_for = table.sequence.back();
    QString text;
    text.sprintf("#%d\nFor Participant: %lu\nParent: %s\nAccomodating:\n[", 
      index, participant_for, 
      table.parent_index == -1 ? "None" : std::to_string(table.parent_index).c_str());

    for (uint i=0; i<(table.sequence.size() - 1); ++i)
    {
      QString seq_text;
      seq_text.sprintf("%lu ", table.sequence[i]);

      text.append(seq_text);
    }
    text.append("]\n");

    //status text
    text.append("Ongoing? ");
    text.append(table.ongoing ? "YES\n" : "NO\n");
    text.append("Finished? ");
    text.append(table.finished ? "YES\n" : "NO\n");
    text.append("Forfeited? ");
    text.append(table.forfeited ? "YES\n" : "NO\n");
    text.append("Rejected? ");
    text.append(table.rejected ? "YES\n" : "NO\n");
    text.append("Defunct? ");
    text.append(table.defunct ? "YES\n" : "NO\n");
    
    // add items
    QGraphicsTextItem* textitem = _scene->addText(text);
    textitem->setTextWidth(textwidth);
    QGraphicsRectItem* rect = _scene->addRect(textitem->boundingRect());
    rect->setZValue(-1);
    rect->setFlag(QGraphicsItem::ItemIsSelectable, true);
    rect->setData(0, QVariant(index));

    if (prev_table_selected == index)
      rect->setSelected(true);
    
    /*if (index == 2)
      rect->setBrush(QBrush(Qt::green));*/
    
    if (prev_depth == table.depth)
    {
      if (row_height < rect->boundingRect().height())
        row_height = rect->boundingRect().height();
    }
    else
    {
      first_index_in_current_row = index;
      current_height_offset += row_height + 20.0f;
      prev_depth = table.depth;
      row_height = 0.0f; //reset
    }

    // positioning
    float table_x_offset = x_offset + (float)(index - first_index_in_current_row) * width_per_column;
    rect->moveBy(table_x_offset, current_height_offset);
    textitem->moveBy(table_x_offset, current_height_offset);

    auto box = rect->boundingRect();
    box.translate(table_x_offset, current_height_offset);
    _node_rectangles.push_back(box);

    if (table.parent_index != -1 && table.parent_index < (int)_node_rectangles.size())
    {
      float start_x = (box.left() + box.right()) * 0.5f;
      float start_y = box.top();

      QRectF parent_rect = _node_rectangles[table.parent_index];
      float end_x = (parent_rect.left() + parent_rect.right()) * 0.5f;
      float end_y = parent_rect.bottom();
      _scene->addLine(start_x, start_y, end_x, end_y);
    }

    ++index;
  }

  _scene->update();
  std::cout << "end repaint" << std::endl;
}

void NegotiationPanel::paintEvent(QPaintEvent*)
{
  update_negotiation_graph_visuals(_status_msg);  
}

void NegotiationPanel::timer_publish_callback()
{
  std::unique_lock<std::mutex> guard(_lock);
  
  auto get_selected_table = [&]() -> int
  {
    auto graphicsitems = _scene->selectedItems();
    for (QGraphicsItem* item : graphicsitems)
    {
      if (item->type() != QGraphicsRectItem::Type)
        continue;
      auto rect = (QGraphicsRectItem*)item;
      QVariant variant = rect->data(0);
      int index = variant.toInt();

      auto& tables = _status_msg.tables;
      if (index >= (int)tables.size())
          continue;
      return index;
    }
    return -1;
  };

  int table_index = get_selected_table();
  if (table_index == -1 || table_index >= (int)_status_msg.tables.size())
  {
    //RCLCPP_WARN(rclcpp::get_logger("asdsad"), "Invalid table_idx: %d", table_index);
    return;
  }

  if (table_index != prev_table_selected)
  {
    prev_table_selected = table_index;
    _itineraries.clear();
    _animation_timestamp = _node->now();

    auto& tables = _status_msg.tables;
    auto& table = tables[table_index];
    //assert(table.proposals.size() == table.proposals_id.size());

    for (auto itin : table.proposals)
      _itineraries.push_back(itin);
  }

  if (_itineraries.empty())
    return; //dont publish

  //convert and publish itineraries
  auto color = [](float r, float g, float b, float a)
  {
    std_msgs::msg::ColorRGBA c;
    c.r = r;
    c.g = g;
    c.b = b;
    c.a = a;
    return c;
  };
  auto interp_color = [](const std_msgs::msg::ColorRGBA& c0, const std_msgs::msg::ColorRGBA& c1, 
    float percent) -> std_msgs::msg::ColorRGBA
  {
    std_msgs::msg::ColorRGBA ret;
    ret.r = c0.r + percent * (c1.r - c0.r);
    ret.g = c0.g + percent * (c1.g - c0.g);
    ret.b = c0.b + percent * (c1.b - c0.b);
    ret.a = c0.a;
    return ret;
  };

  // condense itinary to msg
  visualization_msgs::msg::MarkerArray msg;
  int id = 0;
  int route_index = 0;
  float height_offset = 2.0f;
  for (auto& itin : _itineraries)
  {
    for (auto& route : itin.routes)
    {
      visualization_msgs::msg::Marker marker;

      for (uint i=1; i<route.trajectory.waypoints.size(); ++i)
      {
        auto& start_wp = route.trajectory.waypoints[i-1];
        auto& end_wp = route.trajectory.waypoints[i];

        Eigen::Vector3d p0{ start_wp.position[0], start_wp.position[1], 0.0 };
        Eigen::Vector3d p1{ end_wp.position[0], end_wp.position[1], 0.0 };
        Eigen::Vector3d v = p1 - p0;
        double length = v.norm();
        Eigen::Vector3d unit_v;
        if (length > 0.0)
          unit_v = v / length;
        double length_per_pt = 0.5f;
        int in_between_additions = (int)(length / length_per_pt);
        double length_per_segment = length / (double)in_between_additions;
        double height = height_offset + (float)route_index * 2.0f;
        
        auto add_msg_point = [&](const Eigen::Vector3d& pt)
        {
          geometry_msgs::msg::Point geom_pt;
          geom_pt.x = pt.x();
          geom_pt.y = pt.y();
          geom_pt.z = height;
          marker.points.push_back(geom_pt);

          auto base_color = (route_index % 2) ? color(1, 0.1, 0.1, 0.5) : color(0.1, 0.1, 1, 0.5);
          marker.colors.push_back(base_color);
        };
        add_msg_point(p0);

        for (int j=0; j<in_between_additions; ++j)
        {
          Eigen::Vector3d pt = p0 + length_per_segment * (double)j * unit_v;
          add_msg_point(pt);
        }

        add_msg_point(p1);
      }

      marker.scale.x = marker.scale.y = marker.scale.z = 0.0625;
      //marker.header.frame_id = routes.map;
      marker.header.frame_id = "map";
      marker.header.stamp = _node->now();
      marker.id = id++;
      marker.action = marker.ADD;
      marker.type = marker.POINTS;
      marker.lifetime = rclcpp::Duration::from_seconds(0.1);
      
      rclcpp::Duration anim_dur = _node->now() - _animation_timestamp;
      double animate_percent = fmod(anim_dur.seconds(), 1.0);
      int brightest_index = (int)(animate_percent * (float)marker.points.size());
      if (marker.points.size() > 3)
      {
        std_msgs::msg::ColorRGBA target_color = color(1, 1, 1, 1);
        marker.colors[brightest_index] = target_color;
        if (brightest_index > 1)
          marker.colors[brightest_index - 1] = interp_color(marker.colors[brightest_index - 1],
            target_color, 0.5f);
        if (brightest_index < ((int)marker.colors.size() - 1))
          marker.colors[brightest_index + 1] = interp_color(marker.colors[brightest_index + 1],
            target_color, 0.5f);
      }

      msg.markers.push_back(marker);
      ++route_index;
    }
  }
  if (!msg.markers.empty())
    _negotiation_itinerary_markers_pub->publish(msg);
}

GraphicsScene::GraphicsScene(NegotiationPanel* parent)
  :_parent(parent)
{
}

void GraphicsScene::mouseReleaseEvent(QGraphicsSceneMouseEvent *event) 
{
  QGraphicsScene::mouseReleaseEvent(event);
}

void GraphicsScene::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
  //print_items();
  //update();
  auto point = event->scenePos();

  QPoint p = point.toPoint();
  int index = NegotiationPanel::NO_HIGHLIGHT;
  for (uint i=0; i<_parent->_node_rectangles.size(); ++i)
  {
    if (_parent->_node_rectangles[i].contains(p))
    {
      index = (int)i;
      break;
    }
  }

  _parent->_rect_highlight_idx = index;

  QApplication::postEvent(this, new QPaintEvent(QRect()));
  
  QGraphicsScene::mouseMoveEvent(event);
}

}


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz2_plugin::NegotiationPanel, rviz_common::Panel)