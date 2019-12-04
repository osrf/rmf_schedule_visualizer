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
      Node("rviz_plugin_node")
      {
        // Create layout for output topic box
        QHBoxLayout* topic_layout = new QHBoxLayout;
        topic_layout->addWidget(new QLabel("Output Topic:"));
        _topic_editor = new QLineEdit;
        topic_layout->addWidget(_topic_editor);

        // Create layout for map_name box
        // QHBoxLayout* map_name_layout = new QHBoxLayout;
        // map_name_layout->addWidget(new QLabel("Map Name:"));
        // _map_name_editor = new QLineEdit;
        // map_name_layout->addWidget(_map_name_editor);

        QVBoxLayout* layout = new QVBoxLayout;
        layout->addLayout(topic_layout);
        // layout->addWidget(map_name_layout);
        setLayout(layout);

      }


} // namespace rviz2_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz2_plugin::SchedulePanel, rviz_common::Panel)