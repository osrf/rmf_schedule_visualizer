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
find_package(websocketpp REQUIRED)
find_package(visualizer_utils REQUIRED)
find_package(Boost REQUIRED COMPONENTS system date_time regex random)

#===============================================================================

add_executable(planning_visualizer_server
  src/main.cpp
  src/Inspector.cpp
  src/Server.cpp
)
target_link_libraries(planning_visualizer_server
  PRIVATE
    rmf_traffic::rmf_traffic
    visualizer_utils::visualizer_utils
    Boost::system
    Boost::date_time
    Boost::regex
    Boost::random
)
target_include_directories(planning_visualizer_server
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${visualizer_utils_INCLUDE_DIRS}
    ${websocketpp_INCLUDE_DIR}
)

#===============================================================================

add_executable(test_inspector
  src/test_inspector.cpp
  src/Inspector.cpp
)
target_link_libraries(test_inspector
  PRIVATE
    rmf_traffic::rmf_traffic
)

#===============================================================================

install(
  TARGETS 
    planning_visualizer_server
    test_inspector
  EXPORT
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
