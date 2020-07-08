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

      std::unique_lock<std::mutex> guard(_lock);
      
      _status_msg = msg;
      _update_gfx = true;
      QApplication::postEvent(this, new QPaintEvent(QRect()));
    }
  );

  // UI initialization
  _scene = new GraphicsScene();
  _view = new QGraphicsView(_scene);
  _view->show();

  QLayout* glayout = new QVBoxLayout();
  glayout->addWidget(_view);
  
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(glayout);
  layout->addStretch();
  setLayout(layout);

  {
    auto msg = std::make_shared<NegotiationStatusMsg>();
    msg->participants = { 1, 2 };

    rmf_traffic_msgs::msg::NegotiationStatusTable table;
    table.parent_index = -1;
    table.sequence = { 1, 2 };
    msg->tables.push_back(table);

    rmf_traffic_msgs::msg::NegotiationStatusTable table2;
    table2.parent_index = -1;
    table2.sequence = { 2, 1 };
    msg->tables.push_back(table2);

    rmf_traffic_msgs::msg::NegotiationStatusTable table3;
    table3.parent_index = 1;
    table3.depth = 1;
    table3.sequence = { 2, 1 };
    msg->tables.push_back(table3);

    update_negotiation_graph_visuals(msg);
  }
  //@todo: when we unify plugins we should add nodes to an executor
  _thread = std::thread([&]()
    {
      rclcpp::spin(_node);
    });
}

void NegotiationPanel::update_negotiation_graph_visuals(NegotiationStatusMsg::SharedPtr msg)
{
  std::cout << "starting repaint.." << std::endl;
  std::unique_lock<std::mutex> guard(_lock);
  
  _scene->clear();
  _node_rectangles.clear();
  //_scene->addPolygon(); //arrow
  
  uint column_count = msg->participants.size();

  float textwidth = 160.0f;  
  float width_per_column = textwidth + 40.f;
  //float x_offset = (-(float)column_count / 2.0f) * width_per_column;
  float x_offset = 0.f;
  
  // conflict version text
  {
    QString str;
    str.sprintf("Conflict version: %d", msg->conflict_version);
    QGraphicsTextItem* textitem = _scene->addText(str);
    textitem->setScale(2.0);
    //left alignment because aligning center via boundingRect is wrong, qt :(
    textitem->moveBy(x_offset, -80);
  }

  uint32_t index = 0;
  uint32_t prev_depth = 0;
  uint32_t first_index_in_current_row = 0;
  float row_height = 0.0f;
  float current_height_offset = 0.0f;
  for (auto table : msg->tables)
  {
    // assemble text
    uint64_t participant_for = table.sequence.back();
    QString text;
    text.sprintf("#%d\nFor Participant: %d\nParent: %s\nAccomodating:\n[", 
      index, participant_for, 
      table.parent_index == -1 ? "None" : std::to_string(table.parent_index).c_str());

    for (uint i=0; i<(table.sequence.size() - 1); ++i)
    {
      QString seq_text;
      seq_text.sprintf("%d ", table.sequence[i]);

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

    // add items
    QGraphicsTextItem* textitem = _scene->addText(text);
    textitem->setTextWidth(textwidth);
    QGraphicsRectItem* rect = _scene->addRect(textitem->boundingRect());
    rect->setZValue(-1);
    
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

    if (table.parent_index != -1 && table.parent_index < _node_rectangles.size())
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
}

void NegotiationPanel::paintEvent(QPaintEvent*)
{
  if (_update_gfx)
  {
    this->update_negotiation_graph_visuals(_status_msg);  
    _update_gfx = false;
  }
}

void GraphicsScene::mouseReleaseEvent(QGraphicsSceneMouseEvent *event) 
{
  //print_items();
  update();
  QGraphicsScene::mouseReleaseEvent(event);
}

void GraphicsScene::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
  //print_items();
  //update();
  //QPointF cursorPoint = mapToScene(event->pos());
  
  QGraphicsScene::mouseMoveEvent(event);
}

}


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz2_plugin::NegotiationPanel, rviz_common::Panel)