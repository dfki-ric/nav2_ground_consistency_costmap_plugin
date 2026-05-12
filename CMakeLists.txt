cmake_minimum_required(VERSION 3.8)
project(nav2_ground_consistency_costmap_plugin)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_ros_common REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

add_library(${PROJECT_NAME} SHARED
  src/ground_consistency_layer.cpp
  src/kpi_tracker.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  rclcpp::rclcpp
  nav2_costmap_2d::filters
  nav2_costmap_2d::layers
  nav2_costmap_2d::nav2_costmap_2d_client
  nav2_costmap_2d::nav2_costmap_2d_core
  nav2_ros_common::nav2_ros_common
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  pluginlib::pluginlib
)

pluginlib_export_plugin_description_file(nav2_costmap_2d ground_consistency_layer.xml)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES ground_consistency_layer.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(rclcpp_lifecycle REQUIRED)

  ament_add_gtest(test_ground_consistency_layer test/test_ground_consistency_layer.cpp)

  target_link_libraries(test_ground_consistency_layer ${PROJECT_NAME})

  target_link_libraries(test_ground_consistency_layer
    rclcpp::rclcpp
    nav2_lifecycle_manager::nav2_lifecycle_manager_core
    nav2_costmap_2d::filters
    nav2_costmap_2d::layers
    nav2_costmap_2d::nav2_costmap_2d_client
    nav2_costmap_2d::nav2_costmap_2d_core
    ${sensor_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    tf2::tf2
    tf2_ros::tf2_ros
  )
endif()

ament_package()