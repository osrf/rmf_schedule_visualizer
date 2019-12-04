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

#ifndef RVIZ2_PLUGIN__SRC__SCHEDULEPANEL_HPP
#define RVIZ2_PLUGIN__SRC__SCHEDULEPANEL_HPP

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rmf_schedule_visualizer_msgs/msg/rviz_param.hpp>


class QLineEdit;

namespace rviz2_plugin {

using  RvizParamMsg = rmf_schedule_visualizer_msgs::msg::RvizParam;

class SliderWidget;

class SchedulePanel: public rviz_common::Panel, public rclcpp::Node
{
// This class uses Qt slots and is a subclass of QObject, so it needs
// the Q_OBJECT macro.
Q_OBJECT
public:
  // QWidget subclass constructors usually take a parent widget
  // parameter (which usually defaults to 0).  At the same time,
  // pluginlib::ClassLoader creates instances by calling the default
  // constructor (with no arguments).  Taking the parameter and giving
  // a default of 0 lets the default constructor work and also lets
  // someone using the class for something else to pass in a parent
  // widget as they normally would with Qt.
  SchedulePanel( QWidget* parent = 0 );

  // Now we declare overrides of rviz::Panel functions for saving and
  // loading data from the config file.  Here the data is the
  // topic name.
  virtual void load( const rviz_common::Config& config );
  virtual void save( rviz_common::Config config ) const;

  // Next come a couple of public Qt slots.
public Q_SLOTS:
  // The control area, SliderWidget, sends its output to a Qt signal
  // for ease of re-use, so here we declare a Qt slot to receive it.
  void set_start_duration(int seconds);

  // In this example set_topic() does not get connected to any signal
  // (it is called directly), but it is easy to define it as a public
  // slot instead of a private function in case it would be useful to
  // some other user.
  void set_topic(const QString& topic);

  void set_map_name(const QString& map_name);

  void set_finish_duration(const int& seconds);

  // Here we declare some internal slots.
protected Q_SLOTS:
  // send_param() publishes the current velocity values to a ROS
  // topic.  Internally this is connected to a timer which calls it 10
  // times per second.
  void send_param();

  // update_topic() reads the topic name from the QLineEdit and calls
  // setTopic() with the result.
  void update_topic();

  void update_map();
  void update_start_duration();
  void update_finish_duration();

  // Then we finish up with protected member variables.
protected:
  // The control-area widget which turns mouse events into duration
  SliderWidget* _slider_widget;

  // One-line text editor for entering the outgoing ROS topic name.
  QLineEdit* _topic_editor;
  QLineEdit* _map_name_editor;
  QLineEdit* _finish_duration_editor;

  // The current name of the output topic.
  QString _param_topic;
  QString _map_name;
  QString _finish_duration;

  // The ROS publisher for the visualizer parameters
  rclcpp::Publisher<RvizParamMsg>::SharedPtr _param_pub;


  // The latest start_duration values from the drive widget.
  int _start_duration;
};

} // namespace rviz2_plugin


#endif // RVIZ2_PLUGIN__SRC__SCHEDULEPANEL_HPP