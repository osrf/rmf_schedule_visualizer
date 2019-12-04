cmake_minimum_required(VERSION 3.5)
project(rviz2_plugin)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmf_schedule_visualizer_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)


include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz2_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz2_QT_VERSION}")
  find_package(Qt5 ${rviz2_QT_VERSION} EXACT REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
endif()

add_definitions(-DQT_NO_KEYWORDS)

## Here we specify the list of source files.
## The generated MOC files are included automatically as headers.
set(SRC_FILES
  src/SchedulePanel.cpp
)

add_library(${PROJECT_NAME} ${SRC_FILES})

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ${QT_LIBRARIES} 
    ${rclcpp_LIBRARIES}
    ${rmf_schedule_visualizer_msgs_LIBRARIES}
    ${rviz_common_LIBRARIES}
    ${rviz_rendering_LIBRARIES}
    ${rviz_default_plugins_LIBRARIES}

)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${rclcpp_INCLUDE_DIRS}
    ${rmf_schedule_visualizer_msgs_INCLUDE_DIRS}
    ${rviz_common_INCLUDE_DIRS}
    ${rviz_rendering_INCLUDE_DIRS}
    ${rviz_default_plugins_INCLUDE_DIRS}
)

ament_export_interfaces(${PROJECT_NAME} HAS_LIBRARY_TARGET)

target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_compile_definitions(${PROJECT_NAME} PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")

install(TARGETS
  ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

#install(FILES
#  plugin_description.xml
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

pluginlib_export_plugin_description_file(${PROJECT_NAME} plugin_description.xml)

ament_package()
