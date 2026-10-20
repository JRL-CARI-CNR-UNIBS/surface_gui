cmake_minimum_required(VERSION 3.12)
project(surface_gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(surface_reconstruction_msgs REQUIRED)
find_package(surface_raster_planner_msgs REQUIRED)
find_package(surface_path_planning_msgs REQUIRED)
find_package(time_parametrization_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)

# The headers are listed so that AUTOMOC processes the Q_OBJECT classes
add_library(${PROJECT_NAME} SHARED
  include/surface_gui/surface_panel.hpp
  include/surface_gui/tab_base.hpp
  include/surface_gui/reconstruction_tab.hpp
  include/surface_gui/trajectory_tab.hpp
  src/surface_panel.cpp
  src/tab_base.cpp
  src/reconstruction_tab.cpp
  src/trajectory_tab.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
# Qt keywords (signals, slots, emit) clash with some ROS/boost headers
target_compile_definitions(${PROJECT_NAME} PRIVATE QT_NO_KEYWORDS)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rviz_common::rviz_common
  pluginlib::pluginlib
  Qt5::Widgets
  ${std_srvs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${trajectory_msgs_TARGETS}
  ${moveit_msgs_TARGETS}
  ${surface_reconstruction_msgs_TARGETS}
  ${surface_raster_planner_msgs_TARGETS}
  ${surface_path_planning_msgs_TARGETS}
  ${time_parametrization_msgs_TARGETS}
)

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)


ament_package()
