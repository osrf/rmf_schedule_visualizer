cmake_minimum_required(VERSION 3.5)
project(rmf_planning_visualizer)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

find_package(ament_cmake REQUIRED)
find_package(rmf_traffic REQUIRED)
find_package(rclcpp REQUIRED)
find_package(websocketpp REQUIRED)

#===============================================================================

add_library(rmf_planning_inspector SHARED
  src/PlanningInspector.cpp
)
target_link_libraries(rmf_planning_inspector
  PUBLIC
    rmf_traffic::rmf_traffic
)
target_include_directories(rmf_planning_inspector
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

ament_export_interfaces(rmf_planning_inspector HAS_LIBRARY_TARGET)
ament_export_dependencies(rmf_traffic)

#===============================================================================

add_executable(rmf_planning_visualizer
  src/planning_visualizer_node.cpp
)
target_link_libraries(rmf_planning_visualizer
  PRIVATE
    rmf_planning_inspector
    ${rclcpp_LIBRARIES}
)
target_include_directories(rmf_planning_visualizer
  PUBLIC
    ${rclcpp_INCLUDE_DIRS}
)

#===============================================================================

add_executable(test_inspector
  src/test_planning_inspector.cpp
)
target_link_libraries(test_inspector
  PRIVATE
    rmf_traffic::rmf_traffic
    rmf_planning_inspector
)

#===============================================================================

install(
  DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
  TARGETS rmf_planning_inspector test_inspector
  EXPORT rmf_planning_inspector
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
#   add_subdirectory(test)
# endif()

ament_package()
