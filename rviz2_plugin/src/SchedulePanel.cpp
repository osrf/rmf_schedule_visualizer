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

#include "SchedulePanel.hpp"
#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

namespace rviz2_plugin {

SchedulePanel::SchedulePanel(QWidget* parent)
    : rviz_common::Panel(parent),
      Node("rviz_plugin_node"),
      _param_topic("/rviz_node/param"),
      _map_name("level1")
{
  // Create layout for output topic box
  QHBoxLayout* topic_layout = new QHBoxLayout;
  topic_layout->addWidget(new QLabel("Output Topic:"));
  _topic_editor = new QLineEdit;
  topic_layout->addWidget(_topic_editor);

  // Create layout for map_name box
  QHBoxLayout* map_name_layout = new QHBoxLayout;
  map_name_layout->addWidget(new QLabel("Map Name:"));
  _map_name_editor = new QLineEdit;
  map_name_layout->addWidget(_map_name_editor);

  // Create layout for map_name box
  QHBoxLayout* finish_duration_layout = new QHBoxLayout;
  finish_duration_layout->addWidget(new QLabel("Finish Time(s):"));
  _finish_duration_editor = new QLineEdit;
  finish_duration_layout->addWidget(_finish_duration_editor);

  //TODO layout for slider widget

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(topic_layout);
  layout->addLayout(map_name_layout);
  layout->addLayout(finish_duration_layout);
  setLayout(layout);

  QTimer* output_timer = new QTimer( this );

  // connect( drive_widget_, SIGNAL( outputVelocity( float, float )), this, SLOT( setVel( float, float )));
  connect( _topic_editor, SIGNAL(editingFinished()), this, SLOT(update_topic()));
  connect( _map_name_editor, SIGNAL(editingFinished()), this, SLOT(update_map()));
  connect( _finish_duration_editor, SIGNAL(editingFinished()), this, SLOT(update_finish_duration()));
  connect( output_timer, SIGNAL(timeout()), this, SLOT(send_param()));

  // Start the timer.
  output_timer->start(100);
}

void SchedulePanel::update_topic()
{
  set_topic(_topic_editor->text());
}

void SchedulePanel::update_map()
{
  set_topic(_map_name_editor->text());
}

void SchedulePanel::update_finish_duration()
{
  set_topic(_finish_duration_editor->text());
}

// Set the topic name we are publishing to.
void SchedulePanel::set_topic(const QString& new_topic)
{
  // Only take action if the name has changed.
  if(new_topic != _param_topic)
  {
    _param_topic = new_topic;
    // If the topic is the empty string, don't publish anything.
    if(_param_topic != "" )
    {
      // update publisher 
      _param_pub = this->create_publisher<RvizParamMsg>(_param_topic.toStdString(), rclcpp::SystemDefaultsQoS());
    }
    Q_EMIT configChanged();
  }
  // Gray out the control widget when the output topic is empty.
  // _slider_widget->setEnabled( _param_topic != "" );
}

void SchedulePanel::set_map_name(const QString& new_name)
{
  // Only take action if the name has changed.
  if(new_name != _map_name)
  {
    _map_name = new_name;
    Q_EMIT configChanged();
  }
  // Gray out the control widget when the output topic is empty.
  // _slider_widget->setEnabled( _param_topic != "" );
}


void SchedulePanel::set_finish_duration(const QString& new_duration)
{
  // Only take action if the name has changed.
  if(new_duration != _finish_duration)
  {
    _finish_duration = new_duration;
    Q_EMIT configChanged();
  }
  // Gray out the control widget when the output topic is empty.
  // _slider_widget->setEnabled( _param_topic != "" );
}



} // namespace rviz2_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz2_plugin::SchedulePanel, rviz_common::Panel)