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
add_executable(planning_visualizer
  src/main.cpp
  src/PlanningInspector.cpp
)
target_link_libraries(planning_visualizer
  PUBLIC
    rmf_traffic::rmf_traffic
    ${rclcpp_LIBRARIES}
)
target_include_directories(planning_visualizer
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${rclcpp_INCLUDE_DIRS}
    ${websocketpp_INCLUDE_DIRS}
)

install(
  TARGETS planning_visualizer
  EXPORT planning_visualizer
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
